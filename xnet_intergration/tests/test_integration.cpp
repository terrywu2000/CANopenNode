#include "fake_xnet.hpp"
#include "can_state.hpp"
#include "logging_config.h"
#include "timed_history.hpp"
#include "spdlog/logger.h"
#include "spdlog/sinks/base_sink.h"
#include "spdlog/sinks/ostream_sink.h"

#include <cstring>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>

#define CHECK(condition) do { if (!(condition)) throw std::runtime_error( \
    std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #condition); } while (false)

extern "C" int xnet_canopen_c_header_test(void);

namespace {
using namespace std::chrono_literals;

template<class Predicate> void wait_for(Predicate predicate) {
    const auto deadline = xco::Clock::now() + 2s;
    while (!predicate()) {
        if (xco::Clock::now() >= deadline) throw std::runtime_error("timed out waiting for worker");
        std::this_thread::sleep_for(1ms);
    }
}

std::string diagnostic(xnet_canopen_handle d) {
    int32_t code; uint32_t size; char text[2048]{};
    xnet_canopen_get_last_error(d, &code, text, sizeof(text), &size);
    return std::to_string(code) + ": " + text;
}

std::vector<xnet_canopen_timing_event_v1> timing_events(xnet_canopen_handle d) {
    std::vector<xnet_canopen_timing_event_v1> events;
    uint64_t cursor = 0;
    xnet_canopen_timing_event_v1 event{};
    int32_t result;
    while ((result = xnet_canopen_read_timing_event_v1(d, cursor, &event, sizeof(event))) == XNET_CO_OK) {
        events.push_back(event);
        cursor = event.sequence;
    }
    CHECK(result == XNET_CO_NOT_FOUND);
    return events;
}

struct RestoreLogSinks {
    std::shared_ptr<spdlog::logger> logger;
    std::vector<spdlog::sink_ptr> original;
    explicit RestoreLogSinks(std::shared_ptr<spdlog::logger> target)
        : logger(std::move(target)), original(logger->sinks()) {}
    ~RestoreLogSinks() { logger->sinks() = std::move(original); }
};

struct Fixture {
    xnet_canopen_handle device = nullptr;
    std::filesystem::path directory, model_path, saved_path;
    Fixture() {
        fake::reset();
        directory = std::filesystem::temp_directory_path()
            / ("xnet_canopen_test_" + std::to_string(xco::Clock::now().time_since_epoch().count()));
        CHECK(std::filesystem::create_directory(directory));
        model_path = directory / "device.json"; saved_path = directory / "lss.json";
        std::filesystem::copy_file(XNET_CO_EXAMPLE_MODEL, model_path);
        CHECK(xnet_canopen_create(&device) == 0);
    }
    ~Fixture() {
        if (device && xnet_canopen_destroy(device) != 0) std::cerr << "cleanup failed: " << diagnostic(device) << "\n";
        std::error_code ec;
        for (const auto& entry : std::filesystem::directory_iterator(directory))
            if (entry.is_regular_file()) std::filesystem::remove(entry.path(), ec);
        std::filesystem::remove(directory, ec);
    }
    void model(xco::Json j) { std::ofstream file(model_path); file << j.dump(2); }
    int32_t initialize(const char* name = "CAN1") {
        return xnet_canopen_initialize(device, name, model_path.string().c_str(), saved_path.string().c_str());
    }
    void start() {
        if (xnet_canopen_start(device) != 0) throw std::runtime_error(diagnostic(device));
        wait_for([&] { return !fake::transmitted().empty(); });
    }
    struct Binding { uint16_t index; uint8_t subindex; uint32_t size; };
    uint32_t bind(std::initializer_list<Binding> bindings) {
        std::vector<uint16_t> indices;
        std::vector<uint8_t> subindices;
        std::vector<uint32_t> sizes;
        for (const auto& b : bindings) {
            indices.push_back(b.index);
            subindices.push_back(b.subindex);
            sizes.push_back(b.size);
        }
        uint32_t id = 0;
        std::vector<uint32_t> types(bindings.size()), actual_sizes(bindings.size());
        uint32_t buffer_size = 0;
        CHECK(xnet_canopen_bind_batch(device, indices.data(), subindices.data(),
                         static_cast<uint32_t>(bindings.size()), types.data(), actual_sizes.data(),
                         &buffer_size, &id) == 0);
        CHECK(actual_sizes == sizes);
        return id;
    }
    xnet_canopen_status_t status() {
        xnet_canopen_status_t s{}; CHECK(xnet_canopen_get_status(device, &s) == 0); return s;
    }
};

nxFrameCAN_t exchange(uint16_t id, std::initializer_list<uint8_t> data, uint16_t response, int command = -1) {
    fake::clear_transmitted();
    fake::inject(id, data);
    nxFrameCAN_t answer{};
    wait_for([&] {
        for (const auto& frame : fake::transmitted()) {
            if (frame.Identifier == response && (command < 0 || frame.Payload[0] == command)) {
                answer = frame; return true;
            }
        }
        return false;
    });
    return answer;
}

void operational(Fixture& f) {
    fake::inject(0, {1, static_cast<uint8_t>(f.status().active_node_id)});
    wait_for([&] { return f.status().nmt_state == CO_NMT_OPERATIONAL; });
}

nxFrameCAN_t sdo_download(uint16_t index, uint8_t subindex, uint32_t value, uint8_t size) {
    return exchange(0x60a, {static_cast<uint8_t>(0x23 | ((4 - size) << 2)),
        static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8), subindex,
        static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 24)}, 0x58a);
}

uint32_t sdo_upload(uint16_t index) {
    const auto response = exchange(0x60a, {0x40, static_cast<uint8_t>(index),
        static_cast<uint8_t>(index >> 8), 0, 0, 0, 0, 0}, 0x58a);
    if (response.Payload[0] != 0x43 && response.Payload[0] != 0x4b && response.Payload[0] != 0x4f)
        throw std::runtime_error("SDO upload failed for 0x" + std::to_string(index)
            + ", command=0x" + std::to_string(response.Payload[0]));
    const auto size = 4U - ((response.Payload[0] >> 2) & 3U);
    uint32_t value = 0;
    for (uint32_t i = 0; i < size; ++i) value |= uint32_t(response.Payload[4 + i]) << (8 * i);
    return value;
}

nxFrameCAN_t guarding_request() {
    fake::clear_transmitted();
    fake::inject(0x70a, {}, nxFrameType_CAN_Remote);
    nxFrameCAN_t response{};
    wait_for([&] {
        for (const auto& frame : fake::transmitted()) if (frame.Identifier == 0x70a) {
            response = frame;
            return true;
        }
        return false;
    });
    CHECK(response.PayloadLength == 1 && response.Type == nxFrameType_CAN_Data);
    return response;
}

void node_id_offsets() {
    auto j = xco::read_json(XNET_CO_EXAMPLE_MODEL);
    j["objects"][0]["value"] = 100;
    j["objects"][0]["value_offset"] = "NODE_ID";
    j["objects"][1]["value"] = -100;
    j["objects"][1]["value_offset"] = "NODE_ID";
    j["objects"][4]["value_offset"] = "NODE_ID";
    j["objects"][5]["members"][1]["value_offset"] = "NODE_ID";
    j["tpdos"] = xco::Json::array();
    for (int i = 0; i < 4; ++i) j["tpdos"].push_back({{"mapping", xco::Json::array()}});
    j["tpdos"].push_back({{"cob_id", "0x1C0"}, {"cob_id_offset", "NODE_ID"},
        {"mapping", {{{"index", "0x2000"}}}}});
    j["emcy"] = {{"cob_id", "0x80"}, {"cob_id_offset", "NODE_ID"}};
    j["sync"] = {{"cob_id", "0x300"}, {"cob_id_offset", "NODE_ID"}};
    auto read = [](xco::Model& model, uint16_t index, uint8_t sub = 0) {
        uint64_t number = 0;
        auto& value = model.value(index, sub);
        std::memcpy(&number, value.data(), value.size);
        return number;
    };
    xco::Model model; model.load(j); model.restore(true, 10);
    CHECK(read(model, 0x2000) == 110);
    CHECK(read(model, 0x2001) == uint32_t(-90));
    CHECK(read(model, 0x2004, 1) == 11);
    CHECK(read(model, 0x2005, 1) == 52);
    CHECK(read(model, 0x1804, 1) == 0x1ca);
    CHECK(read(model, 0x1014) == 0x8a && read(model, 0x1005) == 0x30a);
    *static_cast<uint16_t*>(model.value(0x2000, 0).data()) = 777;
    model.resolve_node(42);
    CHECK(read(model, 0x2000) == 777 && read(model, 0x1804, 1) == 0x1ea);
    model.resolve_node(42);
    CHECK(read(model, 0x1804, 1) == 0x1ea);
    model.restore(false, 20);
    CHECK(read(model, 0x2000) == 777 && read(model, 0x1804, 1) == 0x1d4);
    model.restore(true, 20);
    CHECK(read(model, 0x2000) == 120 && read(model, 0x2001) == uint32_t(-80));
    model.restore(true, 255);
    CHECK(read(model, 0x2000) == 100 && read(model, 0x1804, 1) == 0x1c0);
    *static_cast<uint32_t*>(model.value(0x1005, 0).data()) = 0x320;
    *static_cast<uint32_t*>(model.value(0x1014, 0).data()) = 0x90;
    model.sync_reconfigured = model.emcy_reconfigured = true;
    model.resolve_node(10);
    CHECK(read(model, 0x1005) == 0x320 && read(model, 0x1014) == 0x90);
    model.restore(false, 10);
    CHECK(read(model, 0x1005) == 0x30a && read(model, 0x1014) == 0x8a);

    auto rejects = [](const xco::Json& bad) {
        try { xco::Model m; m.load(bad); m.restore(true, 10); }
        catch (const std::exception&) { return true; }
        return false;
    };
    auto bad = j; bad["objects"][0]["value_offset"] = "UNKNOWN"; CHECK(rejects(bad));
    bad = j; bad["objects"][4]["values"] = xco::Json::array();
    bad["objects"][4]["value_offset"] = "UNKNOWN"; CHECK(rejects(bad));
    bad = j; bad["objects"][2]["value_offset"] = "NODE_ID"; CHECK(rejects(bad));
    bad = j; bad["objects"][5]["members"][0]["value_offset"] = "NODE_ID"; CHECK(rejects(bad));
    bad = j; bad["tpdos"][4]["cob_id_offset"] = 1; CHECK(rejects(bad));
    bad = j; bad["tpdos"][4].erase("cob_id"); CHECK(rejects(bad));
    bad = j; bad["tpdos"][4]["cob_id"] = "0x7FF"; CHECK(rejects(bad));
    bad = j; bad["tpdos"][4]["cob_id"] = "0x7DA"; CHECK(rejects(bad)); // Resolves to restricted LSS ID.
    bad = j; bad["tpdos"][4]["cob_id"] = "0x200"; CHECK(rejects(bad));
    for (const auto& type : {"UNSIGNED8", "UNSIGNED16", "UNSIGNED32", "UNSIGNED64",
                             "INTEGER8", "INTEGER16", "INTEGER32", "INTEGER64"}) {
        const unsigned bits = static_cast<unsigned>(std::stoul(std::string(type).substr(type[0] == 'U' ? 8 : 7)));
        const bool sign = type[0] == 'I';
        const uint64_t maximum = sign ? (bits == 64 ? uint64_t(INT64_MAX) : (uint64_t(1) << (bits - 1)) - 1)
                                     : (bits == 64 ? UINT64_MAX : (uint64_t(1) << bits) - 1);
        bad = j;
        bad["objects"][0]["type"] = type;
        bad["objects"][0]["value"] = maximum - 10;
        xco::Model boundary; boundary.load(bad); boundary.restore(true, 10);
        CHECK(read(boundary, 0x2000) == maximum);
        bad["objects"][0]["value"] = maximum - 9;
        CHECK(rejects(bad));
    }
    bad = j; bad["node_id"] = "{USER_INPUT}";
    try { xco::Model m; m.load(bad); CHECK(false); }
    catch (const xco::Error& e) { CHECK(std::string(e.what()).find("/node_id: replace {USER_INPUT}") != std::string::npos); }

    Fixture f; f.model(j); CHECK(f.initialize() == 0); f.start();
    CHECK(sdo_upload(0x2000) == 110);
    CHECK(sdo_download(0x1005, 0, 0x320, 4).Payload[0] == 0x60);
    CHECK(xnet_canopen_stop(f.device) == 0); f.start();
    CHECK(sdo_upload(0x1005) == 0x320 && sdo_upload(0x2000) == 110);
    fake::inject(0, {130, 10});
    wait_for([&] { return f.status().reset_communication_count == 1; });
    CHECK(sdo_upload(0x1005) == 0x30a);
    operational(f);
    fake::clear_transmitted();
    CHECK(xnet_canopen_request_tpdo(f.device, 5) == 0);
    wait_for([&] {
        for (const auto& frame : fake::transmitted()) if (frame.Identifier == 0x1ca) return true;
        return false;
    });
    fake::inject(0x7e5, {4, 1, 0, 0, 0, 0, 0, 0});
    CHECK(exchange(0x7e5, {0x11, 42, 0, 0, 0, 0, 0, 0}, 0x7e4, 0x11).Payload[1] == 0);
    fake::inject(0x7e5, {4, 0, 0, 0, 0, 0, 0, 0});
    fake::inject(0, {130, 10});
    wait_for([&] { return f.status().active_node_id == 42; });
    operational(f);
    fake::clear_transmitted();
    CHECK(xnet_canopen_request_tpdo(f.device, 5) == 0);
    wait_for([&] {
        for (const auto& frame : fake::transmitted()) if (frame.Identifier == 0x1ea) return true;
        return false;
    });
    const auto value = exchange(0x62a, {0x40, 0, 0x20, 0, 0, 0, 0, 0}, 0x5aa);
    CHECK(value.Payload[4] == 110); // Reset Communication retained the live application default.
}

void model_validation() {
    auto j = xco::read_json(XNET_CO_EXAMPLE_MODEL);
    xco::Model m; m.load(j);
    CHECK(m.config.CNT_LSS_SLV == 1 && m.config.CNT_TPDO == 1);
    CHECK(m.value(0x2004, 2).size == 2);
    CHECK(m.value(0x2005, 2).type == "VISIBLE_STRING");
    CHECK(reinterpret_cast<uintptr_t>(m.value(0x2002, 0).data()) % alignof(float) == 0);
    m.restore(true, 42);
    uint32_t identifier = 0;
    CHECK(OD_get_u32(OD_find(&m.od, 0x1800), 1, &identifier, true) == ODR_OK);
    CHECK(identifier == 0x1aa);
    auto rejects = [](xco::Json bad) {
        try { xco::Model model; model.load(bad); } catch (const std::exception&) { return true; }
        return false;
    };
    auto bad = j; bad["schema_version"] = 2; CHECK(rejects(bad));
    bad = j; bad["objects"][0]["type"] = "DOMAIN"; CHECK(rejects(bad));
    bad = j; bad["objects"][0]["value"] = 65536; CHECK(rejects(bad));
    bad = j; bad["objects"][1]["value"] = 2147483648ULL; CHECK(rejects(bad));
    bad = j; bad["objects"][0]["index"] = "0x1018"; CHECK(rejects(bad));
    bad = j; bad["objects"].push_back(bad["objects"][0]); CHECK(rejects(bad));
    bad = j; bad["tpdos"][0]["mapping"].push_back({{"index", "0x2001"}}); CHECK(rejects(bad));
    bad = j; bad["objects"][3]["value"] = std::string(32, 'x'); CHECK(rejects(bad));
    bad = j; bad["node_id"] = 128; CHECK(rejects(bad));
    for (const auto& rate : {xco::Json(0), xco::Json(-1), xco::Json(1001), xco::Json(125.5), xco::Json("invalid")}) {
        bad = j; bad["bitrate_kbps"] = rate; CHECK(rejects(bad));
    }
    bad = j; bad["unexpected"] = true; CHECK(rejects(bad));
    for (const auto& value : {xco::Json(1), xco::Json("true"), xco::Json(nullptr)}) {
        bad = j; bad["can_termination"] = value; CHECK(rejects(bad));
    }
    CHECK(OD_find(&m.od, 0x1008) == nullptr);
    for (const auto& name : {xco::Json(42), xco::Json(std::string(4096, 'x')),
                             xco::Json(std::string("a\0b", 3)), xco::Json("a\nb")}) {
        bad = j; bad["device_name"] = name; CHECK(rejects(bad));
    }
    auto maximum_life_time = j;
    maximum_life_time["node_guarding"] = {{"guard_time_ms", 65535}, {"life_time_factor", 65}};
    xco::Model maximum_life_time_model; maximum_life_time_model.load(maximum_life_time);
    bad = maximum_life_time; bad["node_guarding"]["life_time_factor"] = 66; CHECK(rejects(bad));
    bad = j; bad["node_guarding"] = {{"guard_time_ms", 65535}, {"life_time_factor", 255}}; CHECK(rejects(bad));
    bad = j; bad["node_guarding"] = {{"guard_time_ms", -1}}; CHECK(rejects(bad));
    bad = j; bad["node_guarding"] = {{"life_time_factor", 256}}; CHECK(rejects(bad));
    bad = j; bad["node_guarding"] = {{"unknown", 1}}; CHECK(rejects(bad));
    bad = j; bad["sync"] = {{"cob_id", 0x701}}; CHECK(rejects(bad));
    bad = j; bad["sync"] = {{"cob_id", 0x40000080}}; CHECK(rejects(bad));
    bad = j; bad["sync"] = {{"period_us", -1}}; CHECK(rejects(bad));
    bad = j; bad["sync"] = {{"window_us", 0x100000000ULL}}; CHECK(rejects(bad));
    bad = j; bad["sync"] = {{"counter_overflow", 1}}; CHECK(rejects(bad));
    bad = j; bad["sync"] = {{"counter_overflow", 241}}; CHECK(rejects(bad));
    bad = j; bad["emcy"] = {{"cob_id", 0x40000090}}; CHECK(rejects(bad));
    bad = j; bad["emcy"] = {{"cob_id", 0x80}}; CHECK(rejects(bad));
    bad = j; bad["emcy"] = {{"inhibit_100us", 65536}}; CHECK(rejects(bad));
}

xco::Json drive_402_model() {
    auto j = xco::read_json(XNET_CO_EXAMPLE_MODEL);
    j["drive_402"] = {{"profile", "PP"}, {"model", "internal"},
        {"position_tolerance_counts", 0}, {"target_reached_time_us", 0}};
    j["objects"] = xco::Json::array({
        {{"index", "0x603F"}, {"type", "UNSIGNED16"}, {"value", 0}, {"access", "ro"}},
        {{"index", "0x6040"}, {"type", "UNSIGNED16"}, {"value", 0}, {"access", "rw"}, {"pdo", "rx"}},
        {{"index", "0x6041"}, {"type", "UNSIGNED16"}, {"value", 0}, {"access", "ro"}, {"pdo", "tx"}},
        {{"index", "0x6060"}, {"type", "INTEGER8"}, {"value", 1}, {"access", "rw"}, {"pdo", "rx"}},
        {{"index", "0x6061"}, {"type", "INTEGER8"}, {"value", 0}, {"access", "ro"}},
        {{"index", "0x6064"}, {"type", "INTEGER32"}, {"value", 0}, {"access", "ro"}, {"pdo", "tx"}},
        {{"index", "0x607A"}, {"type", "INTEGER32"}, {"value", 0}, {"access", "rw"}, {"pdo", "rx"}},
        {{"index", "0x6081"}, {"type", "UNSIGNED32"}, {"value", 100000}, {"access", "rw"}},
        {{"index", "0x6083"}, {"type", "UNSIGNED32"}, {"value", 1000000}, {"access", "rw"}},
        {{"index", "0x6084"}, {"type", "UNSIGNED32"}, {"value", 1000000}, {"access", "rw"}},
        {{"index", "0x6085"}, {"type", "UNSIGNED32"}, {"value", 1000000}, {"access", "rw"}},
        {{"index", "0x6502"}, {"type", "UNSIGNED32"}, {"value", 1}, {"access", "ro"}}
    });
    j["rpdos"] = {{{"mapping", {{{"index", "0x6040"}, {"subindex", 0}},
                                {{"index", "0x6060"}, {"subindex", 0}},
                                {{"index", "0x607A"}, {"subindex", 0}}}}}};
    j["tpdos"] = {{{"mapping", {{{"index", "0x6041"}, {"subindex", 0}},
                                {{"index", "0x6064"}, {"subindex", 0}}}}}};
    return j;
}

xco::Json external_drive_402_model() {
    auto j = drive_402_model();
    j["drive_402"]["model"] = "external";
    j["drive_402"]["startup_timeout_us"] = 1000000;
    j["drive_402"]["heartbeat_timeout_us"] = 1000000;
    j["drive_402"]["feedback_timeout_us"] = 1000000;
    j["drive_402"]["action_timeout_us"] = 1000000;
    j["drive_402"]["command_max_age_us"] = 100000;
    j["drive_402"]["device_adapter"] = {{"unit_profile", "counts"}, {"counts_per_revolution", 10000},
        {"fault_mapping", {{{"class", 2}, {"detail", 7}, {"device_code", "0x2310"}}}}};
    return j;
}

void drive_402_model_validation() {
    auto j = drive_402_model();
    xco::Model model; model.load(j);
    CHECK(model.drive_402_enabled);
    CHECK(model.value(0x6040, 0).mapped_rpdo);
    CHECK(model.value(0x607a, 0).mapped_rpdo);
    CHECK(model.value(0x6041, 0).mapped_tpdo);
    CHECK(model.value(0x6064, 0).mapped_tpdo);
    auto without_quick_stop_deceleration = j;
    without_quick_stop_deceleration["objects"].erase(10);
    xco::Model optional_6085; optional_6085.load(without_quick_stop_deceleration);
    CHECK(!optional_6085.drive_has_quick_stop_deceleration);
    auto rejects = [](const xco::Json& bad) {
        try { xco::Model candidate; candidate.load(bad); }
        catch (const std::exception&) { return true; }
        return false;
    };
    auto bad = j; bad["objects"].erase(5); CHECK(rejects(bad));
    bad = j; bad["objects"][1]["pdo"] = "none"; CHECK(rejects(bad));
    bad = j; bad["tpdos"][0]["mapping"][1]["index"] = "0x6061"; CHECK(rejects(bad));
    auto minimal = j;
    minimal["objects"].erase(11);
    minimal["objects"].erase(0);
    minimal["rpdos"] = xco::Json::array();
    minimal["tpdos"] = xco::Json::array();
    xco::Model optional_objects; optional_objects.load(minimal);
    CHECK(!optional_objects.drive_has_error_code);
    bad = j; bad["objects"][0]["type"] = "INTEGER16"; CHECK(rejects(bad));
    bad = j; bad["objects"][11]["value"] = 2; CHECK(rejects(bad));
    bad = j; bad["objects"][11]["type"] = "INTEGER32"; CHECK(rejects(bad));
    auto multiple_modes = j; multiple_modes["objects"][11]["value"] = 3;
    xco::Model pp_capable; pp_capable.load(multiple_modes);
    bad = minimal; bad["objects"].erase(0); CHECK(rejects(bad));
    bad = j; bad["drive_402"]["profile"] = "PV"; CHECK(rejects(bad));
    bad = j; bad["drive_402"]["model"] = "external"; CHECK(rejects(bad));
    auto external = external_drive_402_model();
    xco::Model external_model; external_model.load(external);
    CHECK(external_model.drive_402_external);
    for (uint16_t index = 0x2f00; index <= 0x2f05; ++index)
        CHECK(OD_find(&external_model.od, index) == nullptr);
    bad = external; bad["drive_402"].erase("device_adapter"); CHECK(rejects(bad));
    bad = external; bad["drive_402"]["feedback_timeout_us"] = 0; CHECK(rejects(bad));
}

xco::Json adapted_drive_model(const char* profile = "kinco_fd") {
    auto model = external_drive_402_model();
    model["drive_402"]["device_adapter"] = {{"unit_profile", profile}, {"counts_per_revolution", 10000},
        {"direction", -1}, {"zero_counts", 100},
        {"fault_mapping", {{{"class", 2}, {"detail", 7}, {"device_code", "0x2310"}}}}};
    for (uint16_t index : {0x606b, 0x606c, 0x60fc})
        model["objects"].push_back({{"index", index}, {"type", "INTEGER32"}, {"value", 0},
            {"access", "ro"}, {"local_write", false}});
    model["objects"][7]["value"] = 546133;
    model["objects"][8]["value"] = 100000;
    model["objects"][9]["value"] = 100000;
    return model;
}

void drive_unit_adapter() {
    auto json = adapted_drive_model();
    xco::Model model; model.load(json);
    auto& adapter = *model.drive_adapter;
    CHECK(adapter.read(xco::DriveQuantity::profile_velocity) == 33333);
    CHECK(adapter.read(xco::DriveQuantity::profile_acceleration) == 6103516);
    CHECK(adapter.read(xco::DriveQuantity::quick_stop_deceleration) == 1000000);
    CHECK(adapter.raw(xco::DriveQuantity::actual_velocity, 30000) == 491520);
    CHECK(adapter.raw(xco::DriveQuantity::actual_velocity, -30000) == -491520);
    constexpr double tau = 6.2831853071795864769;
    CHECK(std::abs(adapter.position_rad(2600) + tau / 4) < 1e-12);
    CHECK(adapter.position_counts(-tau / 4) == 2600);
    CHECK(adapter.velocity_counts(-3 * tau) == 30000);
    auto report = adapter.report();
    CHECK(report["converted"].size() == 4 && report["unchanged"].size() == 5);
    CHECK(report["assumed_identity"].size() == 4 && report["unbound"].empty());
    CHECK(report["assumed_identity"][0]["role"] == "quick_stop_deceleration");
    CHECK(report["assumed_identity"][0]["confirmed"] == false);
    CHECK(adapter.fault_code(2, 7) == 0x2310);

    const auto rejects = [](const xco::Json& candidate) {
        try { xco::Model loaded; loaded.load(candidate); }
        catch (const std::exception&) { return true; }
        return false;
    };
    auto bad = json; bad["drive_402"]["device_adapter"]["counts_per_revolution"] = 0; CHECK(rejects(bad));
    bad = json; bad["drive_402"]["device_adapter"]["direction"] = 0; CHECK(rejects(bad));
    bad = json; bad["drive_402"]["device_adapter"]["unit_profile"] = "guessed"; CHECK(rejects(bad));
    bad = json; bad["drive_402"]["device_adapter"]["scales"]["unknown"] = {{"numerator", 1}, {"denominator", 1}};
    CHECK(rejects(bad));
    bad = json; bad["drive_402"]["device_adapter"]["scales"]["profile_velocity"] = {{"numerator", 0}, {"denominator", 1}};
    CHECK(rejects(bad));
    bad = json; bad["drive_402"]["device_adapter"]["scales"]["profile_velocity"] = {{"numerator", 1}, {"denominator", UINT32_MAX}};
    CHECK(rejects(bad));
    bad = json; bad["objects"][8]["value"] = UINT32_MAX; CHECK(rejects(bad)); // Conversion, not raw OD, overflows.
    bad = json; bad["objects"].back()["local_write"] = true; CHECK(rejects(bad));
    bad = json; bad["drive_402"]["device_adapter"]["fault_mapping"].push_back(
        bad["drive_402"]["device_adapter"]["fault_mapping"][0]); CHECK(rejects(bad));

    auto override = json;
    override["drive_402"]["device_adapter"]["scales"]["quick_stop_deceleration"] = {{"numerator", 15625}, {"denominator", 256}};
    override["drive_402"]["device_adapter"]["scales"]["target_position"] = {{"numerator", 1}, {"denominator", 2}};
    xco::Model configured; configured.load(override);
    auto& explicit_adapter = *configured.drive_adapter;
    int32_t count = -1; std::memcpy(configured.value(0x607a, 0).data(), &count, sizeof(count));
    CHECK(explicit_adapter.read(xco::DriveQuantity::target_position) == -1);
    count = 1; std::memcpy(configured.value(0x607a, 0).data(), &count, sizeof(count));
    CHECK(explicit_adapter.read(xco::DriveQuantity::target_position) == 1);
    CHECK(explicit_adapter.report()["assumed_identity"].size() == 3);
    CHECK(explicit_adapter.report()["converted"].size() == 6);
    xco::Model counts; counts.load(adapted_drive_model("counts"));
    CHECK(counts.drive_adapter->read(xco::DriveQuantity::profile_velocity) == 546133);
    CHECK(counts.drive_adapter->report()["converted"].empty());
    CHECK(counts.drive_adapter->report()["assumed_identity"].empty());
    xco::Model identity; identity.load(adapted_drive_model("identity"));
    CHECK(identity.drive_adapter->read(xco::DriveQuantity::profile_velocity) == 546133);
    CHECK(identity.drive_adapter->report()["assumed_identity"].size() == 9);
    auto internal = json; internal["drive_402"]["model"] = "internal";
    for (const char* key : {"startup_timeout_us", "heartbeat_timeout_us", "feedback_timeout_us",
                           "action_timeout_us", "command_max_age_us"}) internal["drive_402"].erase(key);
    xco::Model internal_model; internal_model.load(internal);
    CHECK(internal_model.drive_adapter && internal_model.drive_adapter->read(xco::DriveQuantity::profile_velocity) == 33333);
}

void motor_drive_si_exchange() {
    Fixture fixture;
    fixture.model(adapted_drive_model());
    CHECK(fixture.initialize() == XNET_CO_OK);
    motor_drive_endpoint_handle endpoint = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(fixture.device, &endpoint) == XNET_CO_OK);
    motor_drive_port_handle port = nullptr, duplicate = nullptr;
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    CHECK(motor_drive_open_v1(endpoint, &duplicate) == MOTOR_DRIVE_BUSY && duplicate == nullptr);
    const auto tick = [&] {
        std::lock_guard<std::recursive_mutex> lock(fixture.device->mutex);
        fixture.device->process_drive_402(0);
    };
    const auto scalar = [&](uint16_t index) {
        int32_t value; uint32_t size = 0;
        CHECK(xnet_canopen_read_od(fixture.device, index, 0, reinterpret_cast<uint8_t*>(&value),
            sizeof(value), &size) == XNET_CO_OK);
        return value;
    };
    uint32_t report_size = 0;
    CHECK(xnet_canopen_get_drive_unit_report(fixture.device, nullptr, 0, &report_size) == XNET_CO_BUFFER_TOO_SMALL);
    std::vector<char> report_bytes(report_size);
    CHECK(xnet_canopen_get_drive_unit_report(fixture.device, report_bytes.data(), report_size - 1, &report_size)
        == XNET_CO_BUFFER_TOO_SMALL);
    CHECK(xnet_canopen_get_drive_unit_report(fixture.device, report_bytes.data(), report_bytes.size(), &report_size) == XNET_CO_OK);
    const auto report = xco::Json::parse(report_bytes.data());
    CHECK(report["converted"].size() == 4 && report["assumed_identity"].size() == 4);
    CHECK(diagnostic(fixture.device).find("motor endpoint already has a writer") != std::string::npos);

    motor_drive_read_request_v1 request{};
    request.abi_version = MOTOR_DRIVE_ABI_VERSION; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_ASYNC;
    motor_drive_view_v1 view{};
    view.mode = 123;
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_NO_DATA && view.mode == 123);
    tick();
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
    constexpr double tau = 6.2831853071795864769;
    CHECK(std::abs(view.demand_position_rad - tau / 100) < 1e-12);
    CHECK(view.source_cycle.flags == 0 && view.publish_time.flags == MOTOR_DRIVE_TIME_VALID);
    CHECK(std::abs(view.safety_deceleration_rad_s2 - 6103516 * tau / 10000) < 1e-10);
    auto scheduled = request; scheduled.flags = MOTOR_DRIVE_READ_SCHEDULED;
    CHECK(motor_drive_read_v1(port, &scheduled, &view, sizeof(view)) == MOTOR_DRIVE_UNSUPPORTED);
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view) - 1) == MOTOR_DRIVE_BUFFER_TOO_SMALL);

    motor_drive_feedback_v1 feedback{};
    feedback.abi_version = MOTOR_DRIVE_ABI_VERSION; feedback.struct_size = sizeof(feedback);
    feedback.source = view.token;
    feedback.executed_step = {(uint64_t{1} << 53) + 17, 100, 1, 1.0, 0.002, {}};
    feedback.feedback_seq = 1; feedback.flags = 0x17;
    feedback.actual_position_rad = -tau / 4; feedback.actual_velocity_rad_s = -3 * tau;
    feedback.limit_flags = 1;
    CHECK(motor_drive_heartbeat_v1(port, &feedback.source, &feedback.executed_step, 1) == MOTOR_DRIVE_OK);
    const auto heartbeat_time = fixture.device->drive_heartbeat_changed;
    CHECK(motor_drive_heartbeat_v1(port, &feedback.source, &feedback.executed_step, 1) == MOTOR_DRIVE_OK);
    CHECK(fixture.device->drive_heartbeat_changed == heartbeat_time);
    CHECK(motor_drive_heartbeat_v1(port, &feedback.source, &feedback.executed_step, 2) == MOTOR_DRIVE_STALE);
    CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
    const auto staged_time = fixture.device->drive_pending_submitted_at;
    CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
    CHECK(fixture.device->drive_pending_submitted_at == staged_time);
    auto changed = feedback; changed.actual_position_rad += 1e-10; // Even a change below one OD count is not an identical retry.
    CHECK(motor_drive_submit_v1(port, &changed, sizeof(changed)) == MOTOR_DRIVE_INVALID_FEEDBACK);
    changed = feedback; changed.feedback_seq = 2; ++changed.executed_step.pcl_tick; ++changed.executed_step.model_step_seq;
    changed.executed_step.model_time_s += 0.002;
    CHECK(motor_drive_submit_v1(port, &changed, sizeof(changed)) == MOTOR_DRIVE_BUSY);
    uint32_t alive = 99;
    CHECK(xnet_canopen_write_od(fixture.device, 0x2f01, 0, reinterpret_cast<uint8_t*>(&alive), sizeof(alive)) == XNET_CO_NOT_FOUND);
    tick();
    CHECK(scalar(0x6064) == 2600 && scalar(0x606c) == 491520);
    CHECK((fixture.device->drive_402->status().statusword & 0x0800) != 0);
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
    CHECK(view.processed_feedback_seq == 1 && view.processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
    CHECK(scalar(0x6081) == 546133 && scalar(0x6083) == 100000 && scalar(0x607a) == 0);
    uint16_t index = 0x6041;
    uint8_t subindex = 0;
    uint32_t type = 0, size = 0, total = 0, batch = 0;
    CHECK(xnet_canopen_bind_batch(fixture.device, &index, &subindex, 1, &type, &size, &total, &batch) == XNET_CO_OK);
    fixture.device->shutdown_joining = true;
    uint8_t status_bytes[2]{};
    int32_t entry_status = 0;
    const auto od_during_stop = xnet_canopen_read_od(fixture.device, index, 0, status_bytes, sizeof(status_bytes), &size);
    const auto batch_during_stop = xnet_canopen_read_batch(fixture.device, batch, status_bytes, sizeof(status_bytes), &entry_status, 1);
    const auto port_during_stop = motor_drive_read_v1(port, &request, &view, sizeof(view));
    fixture.device->shutdown_joining = false;
    CHECK(od_during_stop == XNET_CO_INVALID_STATE && batch_during_stop == XNET_CO_INVALID_STATE);
    CHECK(port_during_stop == MOTOR_DRIVE_OK);
    const auto accepted_time = fixture.device->drive_feedback_accepted_at;
    CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
    CHECK(fixture.device->drive_feedback_accepted_at == accepted_time && !fixture.device->drive_feedback_pending);
    CHECK(motor_drive_submit_v1(port, &changed, sizeof(changed)) == MOTOR_DRIVE_OK);
    tick();
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
    auto invalid = changed; invalid.source = view.token; invalid.feedback_seq = 3;
    ++invalid.executed_step.pcl_tick; ++invalid.executed_step.model_step_seq; invalid.executed_step.model_time_s += 0.002;
    invalid.actual_position_rad = std::numeric_limits<double>::quiet_NaN();
    CHECK(motor_drive_submit_v1(port, &invalid, sizeof(invalid)) == MOTOR_DRIVE_INVALID_FEEDBACK);
    invalid.actual_position_rad = 1e100;
    CHECK(motor_drive_submit_v1(port, &invalid, sizeof(invalid)) == MOTOR_DRIVE_INVALID_FEEDBACK);
    CHECK(scalar(0x6064) == 2600 && !fixture.device->drive_feedback_pending);
    invalid = changed; invalid.feedback_seq = 3; invalid.source = view.token;
    CHECK(motor_drive_submit_v1(port, &invalid, sizeof(invalid)) == MOTOR_DRIVE_STALE);
    invalid = changed; invalid.feedback_seq = 3; invalid.source = view.token; invalid.limit_flags = 0;
    ++invalid.executed_step.pcl_tick; ++invalid.executed_step.model_step_seq; invalid.executed_step.model_time_s += 0.002;
    invalid.fault_class = 2; invalid.fault_detail = 7;
    CHECK(motor_drive_submit_v1(port, &invalid, sizeof(invalid)) == MOTOR_DRIVE_OK);
    tick();
    CHECK(scalar(0x603f) == 0x2310 && fixture.device->drive_402->status().model_fault);
    CHECK((fixture.device->drive_402->status().statusword & 0x0800) == 0);
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
    invalid.source = view.token; ++invalid.feedback_seq;
    ++invalid.executed_step.pcl_tick; ++invalid.executed_step.model_step_seq; invalid.executed_step.model_time_s += 0.002;
    invalid.fault_class = 99;
    CHECK(motor_drive_submit_v1(port, &invalid, sizeof(invalid)) == MOTOR_DRIVE_INVALID_FEEDBACK);
    CHECK(fixture.device->drive_adapter_fault && diagnostic(fixture.device).find("unmapped motor fault") != std::string::npos);
    CHECK(!fixture.device->drive_feedback_pending && scalar(0x6064) == 2600);

    const auto old_token = view.token;
    {
        std::lock_guard<std::recursive_mutex> lock(fixture.device->mutex);
        fixture.device->drive_402->reset(); fixture.device->start_drive_epoch();
    }
    tick();
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK && view.token.epoch > old_token.epoch);
    CHECK(view.processed_feedback_seq == 0 && view.processed_result == MOTOR_DRIVE_PROCESSED_NONE);
    CHECK(!fixture.device->drive_has_heartbeat);
    CHECK(motor_drive_heartbeat_v1(port, &old_token, &feedback.executed_step, 1) == MOTOR_DRIVE_STALE);
    CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_STALE);
    CHECK(motor_drive_heartbeat_v1(port, &view.token, &feedback.executed_step, 1) == MOTOR_DRIVE_OK);
    CHECK(fixture.device->drive_has_heartbeat);
    auto unmapped = feedback; unmapped.source = view.token; unmapped.fault_class = 99;
    CHECK(motor_drive_submit_v1(port, &unmapped, sizeof(unmapped)) == MOTOR_DRIVE_INVALID_FEEDBACK);
    tick();
    CHECK(fixture.device->drive_402->status().model_fault);
    CHECK(scalar(0x603f) == CO_EMC_GENERIC && CO_isError(fixture.device->co->em, CO_EM_GENERIC_ERROR));
    CHECK(xnet_canopen_uninitialize(fixture.device) == XNET_CO_OK);
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_INVALID_STATE);
    CHECK(fixture.initialize() == XNET_CO_OK);
    motor_drive_endpoint_handle fresh = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(fixture.device, &fresh) == XNET_CO_OK && fresh != endpoint);
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_INVALID_STATE);
    CHECK(motor_drive_close_v1(port) == MOTOR_DRIVE_OK);
    owned.release();
    CHECK(motor_drive_open_v1(fresh, &port) == MOTOR_DRIVE_OK);
    owned.reset(port);
    CHECK(motor_drive_close_v1(port) == MOTOR_DRIVE_OK);
    owned.release();
}

void motor_drive_worker_stop() {
    Fixture fixture;
    fixture.model(adapted_drive_model());
    CHECK(fixture.initialize() == XNET_CO_OK);
    motor_drive_endpoint_handle endpoint = nullptr;
    motor_drive_port_handle port = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(fixture.device, &endpoint) == XNET_CO_OK);
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    fixture.start();
    std::atomic<bool> running{true};
    std::atomic<int32_t> failure{0};
    std::thread model([&] {
        motor_drive_read_request_v1 request{};
        request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_ASYNC;
        uint64_t epoch = 0, step = 0;
        uint32_t seq = 0, waiting = 0;
        double position = 6.2831853071795864769 / 100;
        while (running) {
            motor_drive_view_v1 view{};
            auto result = motor_drive_read_v1(port, &request, &view, sizeof(view));
            if (result == MOTOR_DRIVE_NO_DATA) { std::this_thread::sleep_for(1ms); continue; }
            if (result != MOTOR_DRIVE_OK) { failure = result; break; }
            if (epoch != view.token.epoch) { epoch = view.token.epoch; seq = 0; waiting = 0; }
            ++step;
            motor_drive_model_step_v1 progress{9, step, step, step * 0.002, 0.002, {}};
            result = motor_drive_heartbeat_v1(port, &view.token, &progress, static_cast<uint32_t>(step));
            if (result != MOTOR_DRIVE_OK && result != MOTOR_DRIVE_STALE) { failure = result; break; }
            if (waiting && view.processed_feedback_seq == waiting) waiting = 0;
            if (!waiting) {
                if (view.flags & MOTOR_DRIVE_OUTPUT_ALLOWED) position = view.demand_position_rad;
                motor_drive_feedback_v1 feedback{};
                feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
                feedback.source = view.token; feedback.executed_step = progress; feedback.feedback_seq = ++seq;
                feedback.flags = 0x17; feedback.actual_position_rad = position;
                feedback.actual_velocity_rad_s = (view.flags & MOTOR_DRIVE_OUTPUT_ALLOWED) ? view.demand_velocity_rad_s : 0;
                feedback.action_result = view.action == MOTOR_DRIVE_ACTION_NONE ? MOTOR_DRIVE_RESULT_NONE : MOTOR_DRIVE_RESULT_DONE;
                result = motor_drive_submit_v1(port, &feedback, sizeof(feedback));
                if (result == MOTOR_DRIVE_OK) waiting = seq;
                else if (result != MOTOR_DRIVE_STALE) { failure = result; break; }
            }
            std::this_thread::sleep_for(2ms);
        }
    });
    const auto cleanup = [&] { running = false; model.join(); };
    try {
        const auto drive_status = [&] {
            std::lock_guard<std::recursive_mutex> lock(fixture.device->mutex);
            return fixture.device->drive_402->status();
        };
        wait_for([&] { return drive_status().model_ready; });
        operational(fixture);
        CHECK(sdo_download(0x6040, 0, 6, 2).Payload[0] == 0x60);
        wait_for([&] { return drive_status().pds == xco::d402::Pds::ready_to_switch_on; });
        CHECK(sdo_download(0x6040, 0, 7, 2).Payload[0] == 0x60);
        wait_for([&] { return drive_status().pds == xco::d402::Pds::switched_on; });
        CHECK(sdo_download(0x6040, 0, 15, 2).Payload[0] == 0x60);
        wait_for([&] { return drive_status().output_allowed; });
        CHECK(sdo_download(0x607a, 0, 1000, 4).Payload[0] == 0x60);
        CHECK(sdo_download(0x6040, 0, 31, 2).Payload[0] == 0x60);
        wait_for([&] { return drive_status().actual_position == 1000; });
        CHECK(sdo_upload(0x6081) == 546133 && sdo_upload(0x607a) == 1000);
        CHECK(xnet_canopen_stop(fixture.device) == XNET_CO_OK); // Model remains alive to answer the new-epoch safety command.
        CHECK(fixture.status().runtime_state == XNET_CO_RUNTIME_STOPPED);
        CHECK(!drive_status().reaction_unconfirmed && !drive_status().output_allowed);
    } catch (...) {
        cleanup();
        throw;
    }
    cleanup();
    CHECK(failure == 0);
}

void drive_402_external_exchange() {
    Fixture f;
    auto external = external_drive_402_model();
    external["objects"].erase(0);
    for (uint16_t index = 0x2f00; index <= 0x2f05; ++index)
        external["objects"].push_back({{"index", index}, {"type", "UNSIGNED32"},
            {"value", 42}, {"access", "rw"}, {"local_write", true}});
    f.model(external);
    CHECK(f.initialize() == XNET_CO_OK);
    motor_drive_endpoint_handle endpoint = nullptr;
    motor_drive_port_handle port = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    f.device->process_drive_402(0);
    motor_drive_read_request_v1 request{};
    request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_ASYNC;
    motor_drive_view_v1 view{};
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
    const auto first_command = view.token.command_seq;
    const auto first_record = f.device->drive_command_history[first_command % f.device->drive_command_history.size()];
    CHECK(first_record.command.command_seq == first_command
        && first_record.command.epoch == view.token.epoch && first_record.processed_feedback_seq == 0);
    motor_drive_feedback_v1 feedback{};
    feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
    feedback.source = view.token; feedback.executed_step = {1, 1, 1, 0.002, 0.002, {}};
    feedback.feedback_seq = 1; feedback.flags = 0x17;
    feedback.actual_position_rad = 12 * 6.2831853071795864769 / 10000;
    CHECK(motor_drive_heartbeat_v1(port, &feedback.source, &feedback.executed_step, 1) == MOTOR_DRIVE_OK);
    CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
    CHECK(!f.device->drive_feedback_history[1].valid);
    f.device->process_drive_402(0, false, true);
    CHECK(f.device->drive_402->status().actual_position == 0);
    int32_t published_actual = -1;
    std::memcpy(&published_actual, f.device->model->value(0x6064, 0).data(), sizeof(published_actual));
    CHECK(published_actual == 0);
    const auto& accepted = f.device->drive_feedback_history[1];
    CHECK(accepted.valid && accepted.feedback.feedback_seq == 1
        && accepted.feedback.executed_step.model_step_seq == 1
        && accepted.feedback.actual_position_rad == feedback.actual_position_rad
        && accepted.converted.actual_position == 12 && accepted.submitted <= accepted.accepted);
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
    CHECK(view.processed_feedback_seq == 1 && view.processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
    const auto& old_record = f.device->drive_command_history[first_command % f.device->drive_command_history.size()];
    CHECK(old_record.command.command_seq == first_record.command.command_seq
        && old_record.command.action_seq == first_record.command.action_seq
        && old_record.processed_feedback_seq == first_record.processed_feedback_seq);
    CHECK(f.device->drive_402->select_feedback({accepted.converted.actual_position,
        accepted.converted.actual_velocity, false, false}));
    f.device->process_drive_402(0);
    std::memcpy(&published_actual, f.device->model->value(0x6064, 0).data(), sizeof(published_actual));
    CHECK(published_actual == 12);
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
    for (uint32_t seq = 2; seq <= 65; ++seq) {
        feedback.feedback_seq = seq;
        feedback.source = view.token;
        feedback.executed_step = {1, seq, seq, seq * 0.002, 0.002, {}};
        CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
        f.device->process_drive_402(0);
        CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
        CHECK(view.processed_feedback_seq == seq && view.processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
    }
    CHECK(f.device->drive_feedback_history_overwrites == 1);
    CHECK(f.device->drive_command_history_overwrites > 0);
    CHECK(f.device->drive_feedback_history[1].feedback.feedback_seq == 65);
    CHECK(f.device->drive_feedback_history[2].feedback.feedback_seq == 2);
    for (uint16_t index = 0x2f00; index <= 0x2f05; ++index) {
        uint32_t value = 0, size = 0;
        CHECK(xnet_canopen_read_od(f.device, index, 0, reinterpret_cast<uint8_t*>(&value), sizeof(value), &size) == XNET_CO_OK);
        CHECK(value == 42);
        value = 43;
        CHECK(xnet_canopen_write_od(f.device, index, 0, reinterpret_cast<uint8_t*>(&value), sizeof(value)) == XNET_CO_OK);
    }
    f.start();
    CHECK(sdo_upload(0x2f03) == 43); // These indices belong to the device again, not the model transport.
}

void drive_402_watchdog_deadlines() {
    Fixture f;
    auto external = external_drive_402_model();
    external["drive_402"]["feedback_timeout_us"] = 20000;
    external["drive_402"]["action_timeout_us"] = 50000;
    f.model(external);
    CHECK(f.initialize() == XNET_CO_OK);
    motor_drive_endpoint_handle endpoint = nullptr;
    motor_drive_port_handle port = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    const auto tick = [&](uint32_t dt = 0) {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        f.device->process_drive_402(dt);
    };
    const auto command = [&] {
        motor_drive_read_request_v1 request{};
        request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_ASYNC;
        motor_drive_view_v1 view{};
        CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
        return view;
    };
    const auto scalar = [&](uint16_t index) {
        std::array<uint8_t, 4> bytes{};
        uint32_t size = 0, value = 0;
        CHECK(xnet_canopen_read_od(f.device, index, 0, bytes.data(), bytes.size(), &size) == XNET_CO_OK);
        for (uint32_t i = 0; i < size; ++i) value |= uint32_t(bytes[i]) << (8 * i);
        return value;
    };
    uint32_t seq = 0, heartbeat = 0;
    motor_drive_feedback_v1 last_submission{};
    const auto submit = [&](motor_drive_view_v1 source, uint8_t result = CO_402_ACTION_RESULT_NONE,
                            uint16_t fault = 0, int32_t position = 123, bool consume = true,
                            int32_t expected = MOTOR_DRIVE_OK) {
        ++heartbeat;
        motor_drive_model_step_v1 progress{1, heartbeat, heartbeat, heartbeat * 0.002, 0.002, {}};
        CHECK(motor_drive_heartbeat_v1(port, &source.token, &progress, heartbeat) == MOTOR_DRIVE_OK);
        motor_drive_feedback_v1 frame{};
        frame.abi_version = 1; frame.struct_size = sizeof(frame);
        frame.source = source.token;
        frame.executed_step = progress;
        frame.feedback_seq = ++seq;
        frame.actual_position_rad = position * 6.2831853071795864769 / 10000;
        frame.flags = 0x17;
        frame.fault_class = fault ? 2 : 0;
        frame.fault_detail = fault ? 7 : 0;
        frame.action_result = result;
        last_submission = frame;
        CHECK(motor_drive_submit_v1(port, &last_submission, sizeof(last_submission)) == expected);
        if (consume) tick();
    };
    const auto expire_feedback = [&](bool retry = false, bool safety_only = false) {
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            f.device->drive_feedback_accepted_at = xco::Clock::now() - 20ms;
        }
        if (retry)
            CHECK(motor_drive_submit_v1(port, &last_submission, sizeof(last_submission)) == MOTOR_DRIVE_OK);
        if (safety_only) {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            f.device->process_drive_402(0, false, true);
        } else tick();
    };
    const auto expire_action = [&] {
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            f.device->drive_action_started = xco::Clock::now() - 50ms;
        }
        tick();
    };
    const auto controlword = [&](uint16_t value) {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        std::memcpy(f.device->model->value(0x6040, 0).data(), &value, sizeof(value));
        CO_NMT_sendInternalCommand(f.device->co->NMT, CO_NMT_ENTER_OPERATIONAL);
        CHECK(CO_NMT_process(f.device->co->NMT, nullptr, 0, nullptr) == CO_RESET_NOT);
    };

    tick();
    submit(command());
    tick(1000000); // Elapsed motion dt must not age newly accepted feedback.
    CHECK(f.device->drive_402->status().feedback_fresh && scalar(0x603f) == 0);
    expire_feedback(true, true);
    CHECK(scalar(0x603f) == 0x8130);
    CHECK(f.device->drive_402->status().pds == xco::d402::Pds::fault_reaction_active);
    const auto reaction = command();
    CHECK(reaction.action == CO_402_ACTION_FAULT_REACTION && !(reaction.flags & 1));
    expire_action();
    auto status = f.device->drive_402->status();
    CHECK(status.pds == xco::d402::Pds::fault && status.reaction_unconfirmed && !status.action_pending);
    CHECK(command().token.action_seq != reaction.token.action_seq && scalar(0x6064) == 123);
    submit(reaction, CO_402_ACTION_RESULT_DONE, 0, 999, true, MOTOR_DRIVE_STALE);
    CHECK(scalar(0x6064) == 123);
    CHECK(!f.device->drive_402->status().feedback_fresh && scalar(0x603f) == 0x8130);

    submit(command());
    CHECK(f.device->drive_402->status().feedback_fresh && !f.device->drive_action_timed_out);
    CHECK(f.device->drive_402->status().pds == xco::d402::Pds::fault && scalar(0x603f) == 0x8130);
    controlword(0x80);
    tick();
    CHECK(command().action == CO_402_ACTION_FAULT_RESET);
    CHECK(scalar(0x603f) == 0x8130);
    submit(command(), CO_402_ACTION_RESULT_DONE);
    CHECK(f.device->drive_402->status().pds == xco::d402::Pds::switch_on_disabled);
    CHECK(scalar(0x603f) == 0 && !f.device->drive_402->status().output_allowed);

    controlword(0);
    submit(command(), CO_402_ACTION_RESULT_NONE, 0x2310);
    CHECK(scalar(0x603f) == 0x2310);
    const auto fifo_before = f.device->co->em->fifoWrPtr;
    expire_feedback();
    CHECK(scalar(0x603f) == 0x2310);
    CHECK(f.device->co->em->fifoWrPtr != fifo_before); // A distinct timeout still reports EMCY.
    CHECK(CO_isError(f.device->co->em, CO_EM_MANUFACTURER_START + 2));
    expire_action();
    submit(command());
    controlword(0x80);
    tick();
    submit(command(), CO_402_ACTION_RESULT_DONE);
    CHECK(scalar(0x603f) == 0);

    controlword(0x06);
    tick();
    submit(command(), CO_402_ACTION_RESULT_DONE, 0, 123, false);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        f.device->drive_action_started = xco::Clock::now() - 50ms;
        f.device->drive_pending_submitted_at = f.device->drive_action_started + 49999us;
    }
    tick(); // Completion received before the deadline remains valid if worker consumption is late.
    CHECK(command().processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED && scalar(0x603f) == 0);
    CHECK(f.device->drive_402->status().pds == xco::d402::Pds::ready_to_switch_on);
    controlword(0);
    tick();
    submit(command(), CO_402_ACTION_RESULT_DONE);

    for (int lateness_us : {0, 1}) {
        controlword(0x06);
        tick();
        const auto shutdown = command();
        submit(shutdown, CO_402_ACTION_RESULT_DONE, 0, 123, false);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            f.device->drive_action_started = xco::Clock::now() - 50ms;
            f.device->drive_pending_submitted_at = f.device->drive_action_started
                + 50ms + std::chrono::microseconds(lateness_us);
        }
        tick();
        CHECK(command().processed_result == MOTOR_DRIVE_PROCESSED_INVALID);
        CHECK(command().action == CO_402_ACTION_FAULT_REACTION && scalar(0x603f) == 0x8130);
        expire_action();
        submit(command());
        controlword(0x80);
        tick();
        submit(command(), CO_402_ACTION_RESULT_DONE);
        CHECK(scalar(0x603f) == 0);
    }
}

void drive_402_missing_model_startup() {
    Fixture f;
    f.model(external_drive_402_model());
    CHECK(f.initialize() == XNET_CO_OK);
    std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
    f.device->drive_started_at = xco::Clock::now() - 1s;
    f.device->process_drive_402(0);
    auto status = f.device->drive_402->status();
    CHECK(status.pds == xco::d402::Pds::fault_reaction_active && !status.feedback_fresh && !status.output_allowed);
    CHECK(f.device->drive_latched_fault == 0x8130);
    f.device->drive_action_started = xco::Clock::now() - 1s;
    f.device->process_drive_402(0);
    status = f.device->drive_402->status();
    CHECK(status.pds == xco::d402::Pds::fault && status.reaction_unconfirmed && !status.action_pending);
    CHECK(status.actual_position == 0 && !f.device->drive_has_last_feedback);
}

void drive_402_sdo_only() {
    Fixture f;
    auto j = drive_402_model();
    j["objects"].erase(11);
    j["objects"].erase(0);
    j["rpdos"] = xco::Json::array();
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    f.start();
    operational(f);
    CHECK(sdo_download(0x6040, 0, 0x06, 2).Payload[0] == 0x60);
    CHECK(sdo_download(0x6040, 0, 0x07, 2).Payload[0] == 0x60);
    CHECK(sdo_download(0x6040, 0, 0x0f, 2).Payload[0] == 0x60);
    CHECK((sdo_upload(0x6041) & 0x006f) == 0x0027);
    CHECK(sdo_download(0x607a, 0, 1234, 4).Payload[0] == 0x60);
    CHECK(sdo_download(0x6040, 0, 0x1f, 2).Payload[0] == 0x60);
    wait_for([&] { return sdo_upload(0x6064) == 1234; });
    wait_for([&] { return (sdo_upload(0x6041) & 0x0400) != 0; });
    CHECK(sdo_download(0x6060, 0, 3, 1).Payload[0] == 0x80);
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
}

void drive_402_integration() {
    Fixture f;
    f.model(drive_402_model());
    CHECK(f.initialize() == XNET_CO_OK);
    f.start();

    // The lower CANopen NMT state gates DS402 enable transitions.
    CHECK(sdo_download(0x6040, 0, 0x0f, 2).Payload[0] == 0x60);
    std::this_thread::sleep_for(5ms);
    CHECK(sdo_upload(0x6041) == 0x0240);

    operational(f);
    CHECK(sdo_download(0x6060, 0, 8, 1).Payload[0] == 0x80);
    CHECK(sdo_upload(0x6060) == 1);
    CHECK(sdo_download(0x6040, 0, 0x06, 2).Payload[0] == 0x60);
    CHECK(sdo_download(0x6040, 0, 0x07, 2).Payload[0] == 0x60);
    CHECK(sdo_download(0x6040, 0, 0x0f, 2).Payload[0] == 0x60);
    CHECK((sdo_upload(0x6041) & 0x006f) == 0x0027);

    const uint32_t target = 1234;
    const auto target_bytes = std::array<uint8_t, 4>{
        static_cast<uint8_t>(target), static_cast<uint8_t>(target >> 8),
        static_cast<uint8_t>(target >> 16), static_cast<uint8_t>(target >> 24)};
    fake::inject(0x20a, {0x1f, 0x00, 3, target_bytes[0], target_bytes[1], target_bytes[2], target_bytes[3]});
    wait_for([&] { return sdo_upload(0x6060) == 1; });
    CHECK(sdo_upload(0x6040) == 0x1b);
    wait_for([&] { return (sdo_upload(0x6041) & 0x006f) == 0x0040; });
    CHECK(sdo_upload(0x6064) == 0);
    CHECK(sdo_download(0x6040, 0, 0x06, 2).Payload[0] == 0x60);
    CHECK(sdo_download(0x6040, 0, 0x07, 2).Payload[0] == 0x60);
    CHECK(sdo_download(0x6040, 0, 0x0f, 2).Payload[0] == 0x60);
    fake::inject(0x20a, {0x1f, 0x00, 1, target_bytes[0], target_bytes[1], target_bytes[2], target_bytes[3]});
    wait_for([&] { return sdo_upload(0x607a) == target; });
    wait_for([&] { return (sdo_upload(0x6041) & 0x1000) != 0; });
    fake::inject(0x20a, {0x0f, 0x00, 1, target_bytes[0], target_bytes[1], target_bytes[2], target_bytes[3]});
    const auto deadline = xco::Clock::now() + 2s;
    while ((sdo_upload(0x6041) & 0x0400) == 0) {
        if (xco::Clock::now() >= deadline) throw std::runtime_error("DS402 PP target was not reached");
    }
    CHECK(sdo_upload(0x6061) == 1);
    CHECK(static_cast<int32_t>(sdo_upload(0x6064)) == static_cast<int32_t>(target));

    const auto resets_before = f.status().reset_communication_count;
    exchange(0, {0x82, 10}, 0x70a, 0);
    CHECK(f.status().reset_communication_count == resets_before + 1);
    CHECK((sdo_upload(0x6041) & 0x006f) == 0x0040);
    CHECK(sdo_upload(0x6060) == 1);
    operational(f);
    CHECK(sdo_download(0x6040, 0, 0x06, 2).Payload[0] == 0x60);
    CHECK(sdo_download(0x6040, 0, 0x07, 2).Payload[0] == 0x60);
    CHECK(sdo_download(0x6040, 0, 0x0f, 2).Payload[0] == 0x60);

    const uint32_t next_target = 1000000;
    const auto next_bytes = std::array<uint8_t, 4>{
        static_cast<uint8_t>(next_target), static_cast<uint8_t>(next_target >> 8),
        static_cast<uint8_t>(next_target >> 16), static_cast<uint8_t>(next_target >> 24)};
    fake::inject(0x20a, {0x1f, 0x00, 1, next_bytes[0], next_bytes[1], next_bytes[2], next_bytes[3]});
    wait_for([&] { return (sdo_upload(0x6041) & 0x1000) != 0; });
    fake::inject(0x20a, {0x0f, 0x00, 1, next_bytes[0], next_bytes[1], next_bytes[2], next_bytes[3]});
    wait_for([&] { return sdo_upload(0x607a) == next_target; });
    std::this_thread::sleep_for(5ms);
    CHECK(static_cast<int32_t>(sdo_upload(0x6064)) > static_cast<int32_t>(target));
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    std::array<uint8_t, 2> stopped_statusword_bytes{};
    uint32_t read_size = 0;
    CHECK(xnet_canopen_read_od(f.device, 0x6041, 0, stopped_statusword_bytes.data(),
        stopped_statusword_bytes.size(), &read_size) == XNET_CO_OK);
    uint16_t stopped_statusword = 0;
    std::memcpy(&stopped_statusword, stopped_statusword_bytes.data(), sizeof(stopped_statusword));
    CHECK(read_size == stopped_statusword_bytes.size());
    CHECK((stopped_statusword & 0x006f) == 0x0040);
}

void adapter_session_configuration() {
    Fixture f;
    CHECK(f.initialize() == XNET_CO_OK);
    CHECK(f.status().runtime_state == XNET_CO_RUNTIME_STOPPED);
    CHECK(fake::sessions() == 2 && fake::transmitted().empty());
    unsigned output_auto_start = 0;
    for (const auto& event : fake::events()) {
        CHECK(event.operation != "input_auto_start" && event.operation != "start" && event.operation != "write");
        CHECK(event.operation != "input_queue_size" && event.operation != "output_queue_size");
        CHECK(event.operation != "termination");
        if (event.operation == "output_auto_start") {
            CHECK(event.value == 0);
            ++output_auto_start;
        }
    }
    CHECK(output_auto_start == 1);
    u8 off = 0;
    CHECK(nxSetProperty(f.device->adapter.input, nxPropSession_AutoStart, sizeof(off), &off)
          == nxErrInputSessionMustAutoStart);
    u32 wrong_bool_size = 0;
    CHECK(nxSetProperty(f.device->adapter.input, nxPropSession_AutoStart, sizeof(wrong_bool_size), &wrong_bool_size)
          == nxErrInvalidPropertySize);
    CHECK(nxSetProperty(f.device->adapter.input, nxPropSession_QueueSize, sizeof(off), &off)
          == nxErrInvalidPropertySize);
    f.start();
    CHECK(f.status().runtime_state == XNET_CO_RUNTIME_RUNNING);
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    CHECK(f.status().runtime_state == XNET_CO_RUNTIME_STOPPED);
}

void can_termination_configuration() {
    for (const bool enabled : {false, true}) {
        Fixture f;
        auto j = xco::read_json(f.model_path);
        j["can_termination"] = enabled; f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        unsigned applied = 0;
        for (const auto& event : fake::events()) {
            CHECK(event.operation != "start");
            if (event.operation == "termination") {
                CHECK(event.value == (enabled ? nxCANTerm_On : nxCANTerm_Off));
                ++applied;
            }
        }
        CHECK(applied == 1);
        f.start();
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
        f.start();
        exchange(0, {0x82, 10}, 0x70a, 0);
        applied = 0;
        for (const auto& event : fake::events())
            if (event.operation == "termination") ++applied;
        CHECK(applied == 1);
    }
    Fixture f;
    auto j = xco::read_json(f.model_path);
    j["can_termination"] = true; f.model(j);
    fake::fail_next("termination", nxErrInvalidPropertySize);
    CHECK(f.initialize() == XNET_CO_XNET_ERROR);
    CHECK(diagnostic(f.device).find("set CAN termination") != std::string::npos);
    CHECK(fake::sessions() == 0);
}

void startup_bitrates() {
    Fixture f;
    auto j = xco::read_json(f.model_path);
    for (uint16_t rate : {1, 100, 123, 1000}) {
        j["bitrate_kbps"] = rate; f.model(j);
        CHECK(f.initialize() == 0);
        const auto status = f.status();
        CHECK(status.active_bitrate_kbps == rate && status.pending_bitrate_kbps == rate);
        const auto events = fake::events();
        CHECK(!events.empty() && events.back().operation == "bitrate" && events.back().value == uint64_t(rate) * 1000);
        CHECK(xnet_canopen_uninitialize(f.device) == 0 && fake::sessions() == 0);
    }
    j["bitrate_kbps"] = 123; f.model(j);
    CHECK(f.initialize() == 0); f.start();
    fake::inject(0x7e5, {4, 1, 0, 0, 0, 0, 0, 0});
    auto answer = exchange(0x7e5, {0x13, 0, 9, 0, 0, 0, 0, 0}, 0x7e4, 0x13);
    CHECK(answer.Payload[1] != 0 && f.status().pending_bitrate_kbps == 123);
    answer = exchange(0x7e5, {0x17, 0, 0, 0, 0, 0, 0, 0}, 0x7e4, 0x17);
    CHECK(answer.Payload[1] == 0);
    CHECK(xco::read_json(f.saved_path).at("bitrate_kbps") == 123);
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    j["bitrate_kbps"] = 250; f.model(j);
    CHECK(f.initialize() == 0);
    CHECK(f.status().active_bitrate_kbps == 123 && f.status().pending_bitrate_kbps == 123);
    f.start();
    exchange(0, {0x81, 10}, 0x70a, 0);
    CHECK(f.status().runtime_state == XNET_CO_RUNTIME_RUNNING);
    CHECK(f.status().active_bitrate_kbps == 123 && f.status().pending_bitrate_kbps == 123);
}

void scalar_limits() {
    auto j = xco::read_json(XNET_CO_EXAMPLE_MODEL);
    j["objects"].push_back({{"index", "0x2010"}, {"type", "UNSIGNED64"}, {"value", "0xFFFFFFFFFFFFFFFF"}});
    j["objects"].push_back({{"index", "0x2011"}, {"type", "INTEGER64"}, {"value", INT64_MIN}});
    j["objects"].push_back({{"index", "0x2012"}, {"type", "OCTET_STRING"}, {"size", 5}, {"value", {0, 255, 1}}});
    j["objects"].push_back({{"index", "0x2013"}, {"type", "BOOLEAN"}, {"value", true}});
    xco::Model m; m.load(j);
    uint64_t u; int64_t s;
    std::memcpy(&u, m.value(0x2010, 0).data(), 8); CHECK(u == UINT64_MAX);
    std::memcpy(&s, m.value(0x2011, 0).data(), 8); CHECK(s == INT64_MIN);
    auto* bytes = static_cast<uint8_t*>(m.value(0x2012, 0).data());
    CHECK(bytes[0] == 0 && bytes[1] == 255 && bytes[3] == 0);
}

void batches_and_lifecycle() {
    Fixture f; CHECK(f.initialize() == 0);
    CHECK(fake::sessions() == 2);
    auto batch = f.bind({{0x2000, 0, 2}, {0x2001, 0, 4}, {0x2002, 0, 4}});
    uint8_t buffer[10] = {0x34, 0x12, 0xfe, 0xff, 0xff, 0xff};
    float real = 3.25f; std::memcpy(buffer + 6, &real, 4);
    int32_t statuses[3]{};
    CHECK(xnet_canopen_write_batch(f.device, batch, buffer, sizeof(buffer), statuses, 3) == 0);
    CHECK(statuses[0] == 0 && statuses[1] == 0 && statuses[2] == 0);
    std::memset(buffer, 0, sizeof(buffer));
    CHECK(xnet_canopen_read_batch(f.device, batch, buffer, sizeof(buffer), statuses, 3) == 0);
    CHECK(buffer[0] == 0x34 && buffer[1] == 0x12 && buffer[2] == 0xfe && buffer[5] == 0xff);
    std::memcpy(&real, buffer + 6, 4); CHECK(real == 3.25f);
    CHECK(xnet_canopen_read_batch(f.device, batch, buffer, 9, statuses, 3) == XNET_CO_BUFFER_TOO_SMALL);
    CHECK(xnet_canopen_read_batch(f.device, batch, buffer, 10, statuses, 2) == XNET_CO_INVALID_ARGUMENT);
    f.start();
    CHECK(xnet_canopen_start(f.device) == 0);
    CHECK(xnet_canopen_stop(f.device) == 0);
    CHECK(xnet_canopen_stop(f.device) == 0);
    CHECK(xnet_canopen_read_batch(f.device, batch, buffer, 10, statuses, 3) == 0 && buffer[0] == 0x34);
    CHECK(xnet_canopen_uninitialize(f.device) == 0 && fake::sessions() == 0);
    CHECK(f.initialize() == 0);
    CHECK(xnet_canopen_read_batch(f.device, batch, buffer, 10, statuses, 3) == XNET_CO_NOT_FOUND);
    CHECK(f.bind({{0x2000, 0, 2}}) != batch);
}

void packed_batches() {
    Fixture f;
    auto j = xco::read_json(f.model_path);
    j["objects"].push_back({{"index", "0x2010"}, {"type", "UNSIGNED64"}, {"value", 0}, {"local_write", true}});
    j["objects"].push_back({{"index", "0x2011"}, {"type", "BOOLEAN"}, {"value", false}, {"local_write", true}});
    f.model(j); CHECK(f.initialize() == 0);
    uint16_t indices[] = {0x2011, 0x2010, 0x2002, 0x2003};
    uint8_t subindices[] = {0, 0, 0, 0};
    uint32_t sizes[] = {1, 8, 4, 32};
    uint32_t types[4]{}, buffer_size = 0;
    uint32_t batch = 0;
    CHECK(xnet_canopen_bind_batch(f.device, indices, subindices, 4, types, sizes, &buffer_size, &batch) == 0);
    CHECK(buffer_size == 45 && sizes[0] == 1 && sizes[1] == 8 && sizes[2] == 4 && sizes[3] == 32);
    CHECK(types[0] == XNET_CO_TYPE_BOOLEAN && types[1] == XNET_CO_TYPE_UNSIGNED64
        && types[2] == XNET_CO_TYPE_REAL32 && types[3] == XNET_CO_TYPE_VISIBLE_STRING);
    indices[0] = 0; subindices[1] = 255; sizes[2] = 0;
    uint8_t source[47]{};
    source[0] = 0xa5; source[46] = 0x5a;
    auto* bytes = source + 1;
    bytes[0] = 1;
    for (uint8_t i = 0; i < 8; ++i) bytes[1 + i] = i + 1;
    const float real = 3.25f;
    std::memcpy(bytes + 9, &real, sizeof(real));
    std::memcpy(bytes + 13, "packed", 7);
    int32_t statuses[5] = {-99, -99, -99, -99, -99};
    CHECK(xnet_canopen_write_batch(f.device, batch, bytes, 45, statuses, 4) == 0);
    for (int i = 0; i < 4; ++i) CHECK(statuses[i] == XNET_CO_OK);
    CHECK(statuses[4] == -99 && source[0] == 0xa5 && source[46] == 0x5a);
    uint8_t output[47]{};
    output[0] = 0xa5; output[46] = 0x5a;
    CHECK(xnet_canopen_read_batch(f.device, batch, output + 1, 45, statuses, 4) == 0);
    CHECK(std::memcmp(source, output, sizeof(source)) == 0);
    bytes[0] = 2; bytes[1] = 0x55;
    CHECK(xnet_canopen_write_batch(f.device, batch, bytes, 45, statuses, 4) == XNET_CO_PARTIAL_FAILURE);
    CHECK(statuses[0] == XNET_CO_ACCESS_ERROR && statuses[1] == 0 && statuses[2] == 0 && statuses[3] == 0);
    CHECK(xnet_canopen_read_batch(f.device, batch, output + 1, 45, statuses, 4) == 0);
    CHECK(output[1] == 1 && output[2] == 0x55);
}

void batch_validation() {
    Fixture f;
    auto j = xco::read_json(f.model_path);
    for (uint16_t i = 0; i < 257; ++i)
        j["objects"].push_back({{"index", 0x3000 + i}, {"type", "OCTET_STRING"}, {"size", 4096}, {"value", {0}}});
    f.model(j); CHECK(f.initialize() == 0);
    uint16_t indices[] = {0x2000, 0x2001};
    uint8_t subindices[] = {0, 0};
    uint32_t sizes[] = {99, 99};
    uint32_t types[] = {99, 99}, buffer_size = 99;
    uint32_t batch = 0xabcdef;
    CHECK(xnet_canopen_bind_batch(f.device, nullptr, subindices, 2, types, sizes, &buffer_size, &batch) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_bind_batch(f.device, indices, nullptr, 2, types, sizes, &buffer_size, &batch) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_bind_batch(f.device, indices, subindices, 2, nullptr, sizes, &buffer_size, &batch) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_bind_batch(f.device, indices, subindices, 2, types, nullptr, &buffer_size, &batch) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_bind_batch(f.device, indices, subindices, 2, types, sizes, nullptr, &batch) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_bind_batch(f.device, indices, subindices, 2, types, sizes, &buffer_size, nullptr) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_bind_batch(f.device, indices, subindices, 0, types, sizes, &buffer_size, &batch) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_bind_batch(f.device, indices, subindices, 4097, types, sizes, &buffer_size, &batch) == XNET_CO_INVALID_ARGUMENT);
    indices[1] = indices[0];
    CHECK(xnet_canopen_bind_batch(f.device, indices, subindices, 2, types, sizes, &buffer_size, &batch) == XNET_CO_INVALID_ARGUMENT);
    subindices[1] = 255;
    CHECK(xnet_canopen_bind_batch(f.device, indices, subindices, 2, types, sizes, &buffer_size, &batch) == XNET_CO_NOT_FOUND);
    CHECK(batch == 0xabcdef);
    CHECK(types[0] == 99 && types[1] == 99 && sizes[0] == 99 && sizes[1] == 99 && buffer_size == 99);
    indices[1] = 0x2001; subindices[1] = 0;
    CHECK(xnet_canopen_bind_batch(f.device, indices, subindices, 2, types, sizes, &buffer_size, &batch) == 0);
    CHECK(types[0] == XNET_CO_TYPE_UNSIGNED16 && types[1] == XNET_CO_TYPE_INTEGER32);
    CHECK(sizes[0] == 2 && sizes[1] == 4 && buffer_size == 6);
    uint8_t bytes[6] = {0x55};
    int32_t statuses[3] = {-99, -99, -99};
    CHECK(xnet_canopen_write_batch(f.device, batch, bytes, 5, statuses, 2) == XNET_CO_BUFFER_TOO_SMALL);
    CHECK(xnet_canopen_write_batch(f.device, batch, bytes, 6, statuses, 1) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_write_batch(f.device, batch, bytes, 6, statuses, 3) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_write_batch(f.device, batch, nullptr, 6, statuses, 2) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_write_batch(f.device, batch, bytes, 6, nullptr, 2) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_read_batch(f.device, batch, nullptr, 6, statuses, 2) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_read_batch(f.device, batch, bytes, 6, nullptr, 2) == XNET_CO_INVALID_ARGUMENT);
    CHECK(statuses[0] == -99 && statuses[1] == -99 && statuses[2] == -99);
    CHECK(xnet_canopen_read_batch(f.device, batch, bytes, 6, statuses, 2) == 0 && bytes[0] == 0);
    CHECK(statuses[0] == 0 && statuses[1] == 0 && statuses[2] == -99);

    std::vector<uint16_t> large_indices;
    std::vector<uint8_t> large_subindices(257, 0);
    std::vector<uint32_t> large_sizes(257, 4096);
    std::vector<uint32_t> large_types(257, 99);
    for (uint16_t i = 0; i < 257; ++i) large_indices.push_back(0x3000 + i);
    CHECK(xnet_canopen_bind_batch(f.device, large_indices.data(), large_subindices.data(),
                                 256, large_types.data(), large_sizes.data(), &buffer_size, &batch) == 0);
    CHECK(buffer_size == 1024 * 1024 && large_types[255] == XNET_CO_TYPE_OCTET_STRING);
    const auto full_batch = batch;
    CHECK(xnet_canopen_bind_batch(f.device, large_indices.data(), large_subindices.data(),
                                 257, large_types.data(), large_sizes.data(), &buffer_size, &batch) == XNET_CO_INVALID_ARGUMENT);
    CHECK(batch == full_batch && buffer_size == 1024 * 1024 && large_types[256] == 99);
    std::vector<uint8_t> full_buffer(1024 * 1024, 0xff);
    std::vector<int32_t> full_status(256, -99);
    CHECK(xnet_canopen_read_batch(f.device, full_batch, full_buffer.data(),
                                 static_cast<uint32_t>(full_buffer.size()), full_status.data(), 256) == 0);
    CHECK(full_buffer.front() == 0 && full_buffer.back() == 0 && full_status.back() == 0);
}

void partial_and_strings() {
    Fixture f; CHECK(f.initialize() == 0);
    auto batch = f.bind({{0x1018, 1, 4}, {0x2000, 0, 2}});
    uint8_t buffer[6]{9, 0, 0, 0, 0x78, 0x56}; int32_t status[2];
    CHECK(xnet_canopen_write_batch(f.device, batch, buffer, 6, status, 2) == XNET_CO_PARTIAL_FAILURE);
    CHECK(status[0] == XNET_CO_ACCESS_ERROR && status[1] == 0);
    CHECK(xnet_canopen_read_batch(f.device, batch, buffer, 6, status, 2) == 0 && buffer[4] == 0x78);
    auto strings = f.bind({{0x2003, 0, 32}});
    uint8_t text[32]; std::memset(text, 'a', 32);
    CHECK(xnet_canopen_write_batch(f.device, strings, text, 32, status, 1) == XNET_CO_PARTIAL_FAILURE);
    std::memset(text, 0, 32); std::memcpy(text, "manual", 6);
    CHECK(xnet_canopen_write_batch(f.device, strings, text, 32, status, 1) == 0);
    std::memset(text, 0, 32);
    CHECK(xnet_canopen_read_batch(f.device, strings, text, 32, status, 1) == 0);
    CHECK(std::strcmp(reinterpret_cast<char*>(text), "manual") == 0);
}

void single_od_access() {
    Fixture f;
    uint8_t bytes[32]{}; uint32_t size = 0;
    CHECK(xnet_canopen_read_od(f.device, 0x2000, 0, bytes, 2, &size) == XNET_CO_INVALID_STATE);
    CHECK(f.initialize() == 0);
    CHECK(xnet_canopen_read_od(f.device, 0x2000, 0, bytes, 2, nullptr) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_read_od(f.device, 0x2000, 0, nullptr, 2, &size) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_read_od(f.device, 0x2000, 255, bytes, 2, &size) == XNET_CO_NOT_FOUND);
    CHECK(xnet_canopen_clear_error(f.device) == 0);
    CHECK(xnet_canopen_read_od(f.device, 0x2003, 0, nullptr, 0, &size) == XNET_CO_BUFFER_TOO_SMALL && size == 32);
    CHECK(f.device->error_code == 0);
    CHECK(xnet_canopen_read_od(f.device, 0x2000, 0, bytes, 1, &size) == XNET_CO_BUFFER_TOO_SMALL && size == 2);

    const uint8_t value[2] = {0x34, 0x12};
    CHECK(xnet_canopen_write_od(f.device, 0x2000, 0, value, 1) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_write_od(f.device, 0x2000, 0, value, 4) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_write_od(f.device, 0x2000, 0, nullptr, 2) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_write_od(f.device, 0x2000, 0, value, 2) == 0);
    CHECK(xnet_canopen_read_od(f.device, 0x2000, 0, bytes, sizeof(bytes), &size) == 0);
    CHECK(size == 2 && bytes[0] == 0x34 && bytes[1] == 0x12);
    auto batch = f.bind({{0x2000, 0, 2}});
    int32_t status = -99;
    CHECK(xnet_canopen_read_batch(f.device, batch, bytes, 2, &status, 1) == 0 && bytes[0] == 0x34);

    // Same permission and validation rules as batch entries.
    const uint8_t serial[4] = {9, 0, 0, 0};
    CHECK(xnet_canopen_write_od(f.device, 0x1018, 1, serial, 4) == XNET_CO_ACCESS_ERROR);
    CHECK(f.device->error_code == XNET_CO_ACCESS_ERROR);
    uint8_t text[32]; std::memset(text, 'a', 32);
    CHECK(xnet_canopen_write_od(f.device, 0x2003, 0, text, 32) == XNET_CO_ACCESS_ERROR);
    std::memset(text, 0, 32); std::memcpy(text, "single", 6);
    CHECK(xnet_canopen_write_od(f.device, 0x2003, 0, text, 32) == 0);
    std::memset(text, 0, 32);
    CHECK(xnet_canopen_read_od(f.device, 0x2003, 0, text, 32, &size) == 0);
    CHECK(std::strcmp(reinterpret_cast<char*>(text), "single") == 0);

    f.start();
    CHECK(xnet_canopen_read_od(f.device, 0x2000, 0, bytes, 2, &size) == 0 && bytes[0] == 0x34);
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    CHECK(xnet_canopen_write_od(f.device, 0x2000, 0, value, 2) == XNET_CO_INVALID_STATE);
}

void device_name_sdo() {
    for (const std::string name : {"Xnet", "Hello-From-Xnet-Integration"}) {
        Fixture f;
        auto j = xco::read_json(XNET_CO_EXAMPLE_MODEL);
        j["device_name"] = name;
        f.model(j);
        CHECK(f.initialize() == 0); f.start();
        CHECK(f.status().nmt_state == CO_NMT_PRE_OPERATIONAL);
        auto answer = exchange(0x60a, {0x40, 8, 0x10, 0, 0, 0, 0, 0}, 0x58a);
        std::string received;
        if (name.size() <= 4) {
            CHECK(answer.Payload[0] == 0x43);
            received.assign(reinterpret_cast<const char*>(answer.Payload + 4), 4);
        } else {
            CHECK(answer.Payload[0] == 0x41);
            uint32_t length = 0;
            std::memcpy(&length, answer.Payload + 4, sizeof(length));
            CHECK(length == name.size());
            uint8_t toggle = 0;
            do {
                answer = exchange(0x60a, {static_cast<uint8_t>(0x60 | toggle), 0, 0, 0, 0, 0, 0, 0}, 0x58a);
                CHECK((answer.Payload[0] & 0xf0) == toggle);
                const size_t count = 7 - ((answer.Payload[0] >> 1) & 7);
                received.append(reinterpret_cast<const char*>(answer.Payload + 1), count);
                toggle ^= 0x10;
            } while ((answer.Payload[0] & 1) == 0 && received.size() < name.size());
            CHECK((answer.Payload[0] & 1) != 0);
        }
        CHECK(received == name);
        answer = sdo_download(0x1008, 0, 0, 1);
        CHECK(answer.Payload[0] == 0x80);
        uint32_t abort = 0;
        std::memcpy(&abort, answer.Payload + 4, sizeof(abort));
        CHECK(abort == CO_SDO_AB_READONLY);
        const std::vector<uint8_t> value(name.size() + 1, 0);
        CHECK(xnet_canopen_write_od(f.device, 0x1008, 0, value.data(),
            static_cast<uint32_t>(value.size())) == XNET_CO_ACCESS_ERROR);
    }
}

void protocol_basics() {
    Fixture f; CHECK(f.initialize() == 0); f.start();
    auto answer = exchange(0x60a, {0x40, 0x18, 0x10, 1, 0, 0, 0, 0}, 0x58a);
    CHECK(answer.Payload[0] == 0x43 && answer.Payload[4] == 1);
    answer = exchange(0x60a, {0x2b, 0, 0x18, 2, 1, 0, 0, 0}, 0x58a);
    CHECK(answer.Payload[0] == 0x80 && answer.Payload[4] == 2 && answer.Payload[6] == 1 && answer.Payload[7] == 6);
    answer = exchange(0x60a, {0x40, 3, 0x20, 0, 0, 0, 0, 0}, 0x58a);
    CHECK(answer.Payload[0] == 0x41 && answer.Payload[4] <= 32);
    answer = exchange(0x60a, {0x60, 0, 0, 0, 0, 0, 0, 0}, 0x58a);
    CHECK(answer.Payload[1] == 'X' && answer.Payload[2] == 'N');
    operational(f);
    CHECK(xnet_canopen_request_tpdo(f.device, 1) == 0);
    wait_for([] { for (const auto& frame : fake::transmitted()) if (frame.Identifier == 0x18a) return true; return false; });
    fake::inject(0x20a, {0x78, 0x56, 0x34, 0x12});
    auto batch = f.bind({{0x2001, 0, 4}});
    uint8_t bytes[4]{}; int32_t status;
    wait_for([&] { CHECK(xnet_canopen_read_batch(f.device, batch, bytes, 4, &status, 1) == 0); return bytes[0] == 0x78; });
    CHECK(bytes[3] == 0x12);
    CHECK(xnet_canopen_set_fault(f.device, 0x40, 0x5000, 123, 1) == 0);
    wait_for([] { for (const auto& frame : fake::transmitted()) if (frame.Identifier == 0x8a && frame.Payload[1] == 0x50) return true; return false; });
    CHECK(xnet_canopen_set_fault(f.device, 0x40, 0, 0, 0) == 0);
}

xco::Json eight_pdo_model() {
    auto j = xco::read_json(XNET_CO_EXAMPLE_MODEL);
    j["rpdos"] = xco::Json::array();
    j["tpdos"] = xco::Json::array();
    for (uint16_t i = 0; i < 8; ++i) {
        j["objects"].push_back({{"index", 0x2100 + i}, {"type", "UNSIGNED16"}, {"value", 0},
            {"pdo", "rx"}});
        j["objects"].push_back({{"index", 0x2200 + i}, {"type", "UNSIGNED16"}, {"value", 100 + i},
            {"access", "ro"}, {"pdo", "tx"}, {"local_write", true}});
        xco::Json rx = {{"mapping", {{{"index", 0x2100 + i}}}}};
        xco::Json tx = {{"mapping", {{{"index", 0x2200 + i}}}}};
        if (i >= 4) {
            rx["cob_id"] = 0x220 + i - 4;
            tx["cob_id"] = 0x320 + i - 4;
        }
        j["rpdos"].push_back(rx);
        j["tpdos"].push_back(tx);
    }
    return j;
}

void eight_pdo_validation() {
    const auto j = eight_pdo_model();
    auto rejects = [](const xco::Json& bad) {
        try {
            xco::Model model; model.load(bad); model.restore(true, 10);
        } catch (const xco::Error& e) { return e.code == XNET_CO_MODEL_ERROR; }
        return false;
    };
    xco::Model model; model.load(j); model.restore(true, 10);
    CHECK(model.config.CNT_RPDO == 8 && model.config.CNT_TPDO == 8);
    CHECK(OD_find(&model.od, 0x1407) && OD_find(&model.od, 0x1607));
    CHECK(OD_find(&model.od, 0x1807) && OD_find(&model.od, 0x1a07));
    for (const char* direction : {"rpdos", "tpdos"}) {
        auto bad = j; bad[direction].push_back(bad[direction][7]); CHECK(rejects(bad));
        for (size_t i = 4; i < 8; ++i) {
            bad = j; bad[direction][i].erase("cob_id"); CHECK(rejects(bad));
        }
        for (const auto& id : {xco::Json(0), xco::Json(0x800), xco::Json(0x80000320U),
                               xco::Json(0x58a), xco::Json(0x60a), xco::Json(0x70a), xco::Json(0x80)}) {
            bad = j; bad[direction][7]["cob_id"] = id; CHECK(rejects(bad));
        }
        bad = j; bad[direction][7]["cob_id"] = j[direction][4]["cob_id"]; CHECK(rejects(bad));
        bad = j; bad[direction][7]["cob_id"] = 0x18a; CHECK(rejects(bad));
        bad = j; bad[direction][7]["enabled"] = 1; CHECK(rejects(bad));
        bad = j; bad[direction][7]["mapping"] = xco::Json::array();
        bad[direction][7]["enabled"] = true; CHECK(rejects(bad));
        bad = j; bad[direction][7]["mapping"] = xco::Json::object(); CHECK(rejects(bad));
        bad = j; bad[direction][7]["enabled"] = false;
        bad[direction][7]["cob_id"] = 0x18a; CHECK(rejects(bad));
    }
    auto bad = j; bad["tpdos"][7]["cob_id"] = j["rpdos"][7]["cob_id"]; CHECK(rejects(bad));
}

void eight_pdo_traffic() {
    Fixture f; f.model(eight_pdo_model()); CHECK(f.initialize() == 0); f.start(); operational(f);
    for (uint16_t i = 0; i < 8; ++i) {
        const auto rx_id = static_cast<uint16_t>(i < 4 ? 0x20a + i * 0x100 : 0x220 + i - 4);
        const auto tx_id = static_cast<uint16_t>(i < 4 ? 0x18a + i * 0x100 : 0x320 + i - 4);
        auto batch = f.bind({{static_cast<uint16_t>(0x2100 + i), 0, 2}});
        uint8_t bytes[2]{}; int32_t status;
        fake::inject(rx_id, {static_cast<uint8_t>(i + 1), 0x12});
        wait_for([&] {
            CHECK(xnet_canopen_read_batch(f.device, batch, bytes, 2, &status, 1) == 0);
            return bytes[0] == i + 1 && bytes[1] == 0x12;
        });
        fake::clear_transmitted();
        CHECK(xnet_canopen_request_tpdo(f.device, i + 1) == 0);
        wait_for([&] {
            for (const auto& frame : fake::transmitted())
                if (frame.Identifier == tx_id && frame.PayloadLength == 2
                    && frame.Payload[0] == 100 + i && frame.Payload[1] == 0) return true;
            return false;
        });
    }
    CHECK(xnet_canopen_request_tpdo(f.device, 9) == XNET_CO_INVALID_ARGUMENT);
    CHECK(sdo_download(0x1807, 1, 0x80000323, 4).Payload[0] == 0x80);
    CHECK(sdo_download(0x1a07, 0, 0, 1).Payload[0] == 0x80);
    CHECK(sdo_download(0x1407, 1, 0x80000223, 4).Payload[0] == 0x80);
    CHECK(sdo_download(0x1607, 0, 0, 1).Payload[0] == 0x80);
    for (const uint32_t id : {0x223U, 0x323U}) {
        CHECK(sdo_download(0x1005, 0, id, 4).Payload[0] == 0x80);
        CHECK(sdo_download(0x1014, 0, 0x8000008a, 4).Payload[0] == 0x60);
        CHECK(sdo_download(0x1014, 0, id, 4).Payload[0] == 0x80);
    }
    auto feedback = f.bind({{0x2207, 0, 2}});
    uint8_t bytes[2]{0xab, 0x12}; int32_t status;
    CHECK(xnet_canopen_write_batch(f.device, feedback, bytes, 2, &status, 1) == 0);
    auto verify_eighth = [&] {
        operational(f);
        fake::clear_transmitted();
        CHECK(xnet_canopen_request_tpdo(f.device, 8) == 0);
        wait_for([] {
            for (const auto& frame : fake::transmitted())
                if (frame.Identifier == 0x323 && frame.Payload[0] == 0xab && frame.Payload[1] == 0x12) return true;
            return false;
        });
    };
    CHECK(xnet_canopen_stop(f.device) == 0); f.start(); verify_eighth();
    exchange(0, {0x82, 10}, 0x70a, 0); verify_eighth();
    exchange(0, {0x81, 10}, 0x70a, 0);
    CHECK(xnet_canopen_read_batch(f.device, feedback, bytes, 2, &status, 1) == 0 && bytes[0] == 107 && bytes[1] == 0);
    fake::inject(0x7e5, {4, 1, 0, 0, 0, 0, 0, 0});
    CHECK(exchange(0x7e5, {0x11, 42, 0, 0, 0, 0, 0, 0}, 0x7e4, 0x11).Payload[1] == 0);
    fake::inject(0x7e5, {4, 0, 0, 0, 0, 0, 0, 0});
    exchange(0, {0x82, 10}, 0x72a, 0);
    wait_for([&] { return f.status().active_node_id == 42; });
    operational(f);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->co->TPDO[0].CANtxBuff->ident == 0x1aa);
        CHECK(f.device->co->TPDO[7].CANtxBuff->ident == 0x323);
    }
    fake::clear_transmitted();
    CHECK(xnet_canopen_request_tpdo(f.device, 8) == 0);
    wait_for([] {
        for (const auto& frame : fake::transmitted())
            if (frame.Identifier == 0x323 && frame.Payload[0] == 107) return true;
        return false;
    });
    fake::inject(0x223, {0x55, 0x66});
    const auto received = f.bind({{0x2107, 0, 2}});
    wait_for([&] {
        CHECK(xnet_canopen_read_batch(f.device, received, bytes, 2, &status, 1) == 0);
        return bytes[0] == 0x55 && bytes[1] == 0x66;
    });
}

void inactive_pdo_slots() {
    Fixture f; auto j = eight_pdo_model();
    for (const char* direction : {"rpdos", "tpdos"}) {
        j[direction][1]["mapping"] = xco::Json::array();
        j[direction][5]["mapping"] = xco::Json::array();
        j[direction][6]["enabled"] = false;
        j[direction][7]["mapping"] = xco::Json::array();
        j[direction][7]["enabled"] = false;
    }
    f.model(j); CHECK(f.initialize() == 0); f.start(); operational(f);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(!CO_isError(f.device->co->em, CO_EM_PDO_WRONG_MAPPING));
        for (size_t i = 0; i < 8; ++i) {
            const bool enabled = i != 1 && i != 5 && i != 6 && i != 7;
            CHECK((f.device->co->TPDO[i].PDO_common.valid != 0) == enabled);
            CHECK((f.device->co->RPDO[i].PDO_common.valid != 0) == enabled);
        }
    }
    for (uint16_t number : {2, 6, 7, 8})
        CHECK(xnet_canopen_request_tpdo(f.device, number) == XNET_CO_INVALID_STATE);
    for (uint32_t id : {0x222U, 0x322U}) {
        CHECK(sdo_download(0x1005, 0, id, 4).Payload[0] == 0x80);
        CHECK(sdo_download(0x1014, 0, 0x8000008a, 4).Payload[0] == 0x60);
        CHECK(sdo_download(0x1014, 0, id, 4).Payload[0] == 0x80);
    }
    auto configuration = f.bind({{0x1807, 1, 4}, {0x1a07, 0, 1}, {0x1407, 1, 4}, {0x1607, 0, 1}});
    uint8_t config[10]{}; int32_t statuses[4]{};
    CHECK(xnet_canopen_read_batch(f.device, configuration, config, sizeof(config), statuses, 4) == 0);
    CHECK(config[0] == 0x23 && config[1] == 3 && config[3] == 0x80 && config[4] == 0);
    CHECK(config[5] == 0x23 && config[6] == 2 && config[8] == 0x80 && config[9] == 0);
    fake::clear_transmitted();
    fake::inject(0x222, {0x99, 0x88});
    CHECK(sdo_upload(0x2106) == 0);
    for (const auto& frame : fake::transmitted())
        CHECK(frame.Identifier != 0x28a && frame.Identifier != 0x321
            && frame.Identifier != 0x322 && frame.Identifier != 0x323);
    CHECK(xnet_canopen_request_tpdo(f.device, 5) == 0);
    wait_for([] {
        for (const auto& frame : fake::transmitted())
            if (frame.Identifier == 0x320 && frame.Payload[0] == 104) return true;
        return false;
    });
    exchange(0, {0x82, 10}, 0x70a, 0); operational(f);
    CHECK(xnet_canopen_request_tpdo(f.device, 8) == XNET_CO_INVALID_STATE);
    CHECK(xnet_canopen_stop(f.device) == 0); f.start(); operational(f);
    CHECK(xnet_canopen_request_tpdo(f.device, 8) == XNET_CO_INVALID_STATE);
    exchange(0, {0x81, 10}, 0x70a, 0); operational(f);
    CHECK(xnet_canopen_request_tpdo(f.device, 8) == XNET_CO_INVALID_STATE);
}

void eighth_synchronous_pdo() {
    Fixture f; auto j = eight_pdo_model();
    j["tpdos"][7]["transmission_type"] = 0;
    j["tpdos"][7]["cob_id"] = 0x43f;
    j["rpdos"][7]["transmission_type"] = 1;
    f.model(j); CHECK(f.initialize() == 0); f.start(); operational(f);
    const auto batch = f.bind({{0x2107, 0, 2}});
    uint8_t bytes[2]{}; int32_t status;
    fake::inject(0x223, {0x34, 0x12});
    CHECK(sdo_upload(0x2107) == 0);
    fake::clear_transmitted();
    CHECK(xnet_canopen_request_tpdo(f.device, 8) == 0);
    fake::inject(0x80, {});
    wait_for([&] {
        CHECK(xnet_canopen_read_batch(f.device, batch, bytes, 2, &status, 1) == 0);
        return bytes[0] == 0x34 && bytes[1] == 0x12;
    });
    wait_for([] {
        for (const auto& frame : fake::transmitted())
            if (frame.Identifier == 0x43f && frame.Payload[0] == 107) return true;
        return false;
    });
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    j["tpdos"][7]["transmission_type"] = 1;
    f.model(j); CHECK(f.initialize() == 0); f.start(); operational(f);
    CHECK(xnet_canopen_request_tpdo(f.device, 8) == XNET_CO_INVALID_STATE);
    fake::clear_transmitted();
    fake::inject(0x80, {});
    wait_for([] {
        for (const auto& frame : fake::transmitted())
            if (frame.Identifier == 0x43f && frame.Payload[0] == 107) return true;
        return false;
    });
}

void ordered_sync_boundaries() {
    Fixture f;
    auto j = drive_402_model();
    j["rpdos"][0]["transmission_type"] = 1;
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    f.device->adapter.start();
    CO_CANsetNormalMode(f.device->co->CANmodule);
    CO_NMT_sendInternalCommand(f.device->co->NMT, CO_NMT_ENTER_OPERATIONAL);
    CHECK(CO_process(f.device->co, false, 0, nullptr) == CO_RESET_NOT);
    CO_process_RPDO(f.device->co, false, 0, nullptr);
    using Boundary = std::pair<uint64_t, int32_t>;
    std::vector<Boundary> seen;
    const auto capture = [](xnet_canopen_device& d, const xnet_canopen_sync_event_t& event, void* context) {
        int32_t target = 0;
        std::memcpy(&target, d.model->value(0x607a, 0).data(), sizeof(target));
        static_cast<std::vector<Boundary>*>(context)->emplace_back(event.sequence, target);
    };
    fake::inject(0x20a, {6, 0, 1, 11, 0, 0, 0});
    fake::inject(0x80, {});
    fake::inject(0x20a, {7, 0, 1, 22, 0, 0, 0});
    fake::inject(0x80, {});
    fake::inject(0x20a, {15, 0, 1, 33, 0, 0, 0});
    fake::inject(0x80, {1}); // Invalid DLC must not create a boundary.
    fake::inject(0x80, {});
    fake::inject(0, {2, 10}); // NMT stop: this SYNC is not operational.
    fake::inject(0x80, {});
    fake::inject(0, {1, 10});
    fake::inject(0x20a, {15, 0, 1, 44, 0, 0, 0});
    fake::inject(0x80, {});
    f.device->receive(capture, &seen);
    CHECK((seen == std::vector<Boundary>{{1, 11}, {2, 22}, {3, 33}, {5, 44}}));
    CHECK(f.device->sync_observation_sequence == 5);
}

void worker_pdo_independent_cadence() {
    {
        Fixture f;
        auto j = drive_402_model();
        j["sync"] = {{"period_us", 0}, {"window_us", 0}};
        j["objects"][7]["pdo"] = "both";
        j["rpdos"][0]["mapping"] = {{{"index", "0x6081"}}};
        j["rpdos"][0]["transmission_type"] = 3;
        j["tpdos"][0]["transmission_type"] = 3;
        j["tpdos"][0]["mapping"] = {{{"index", "0x6081"}}};
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        f.start();
        operational(f);
        fake::clear_transmitted();
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            fake::inject(0x20a, {11, 0, 0, 0});
            fake::inject(0x80, {});
            fake::inject(0x80, {});
            fake::inject(0x80, {});
            fake::inject(0x20a, {22, 0, 0, 0});
            fake::inject(0x80, {});
            fake::inject(0x80, {});
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_observation_sequence == 5;
        });
        std::vector<uint8_t> values;
        for (const auto& frame : fake::transmitted())
            if (frame.Identifier == 0x18a) values.push_back(frame.Payload[0]);
        CHECK((values == std::vector<uint8_t>{11, 22}));
        CHECK(f.status().runtime_state == XNET_CO_RUNTIME_RUNNING);
    }
    {
        Fixture f;
        auto j = drive_402_model();
        j["sync"] = {{"period_us", 0}, {"window_us", 0}};
        j["objects"][7]["pdo"] = "both";
        j["rpdos"][0]["mapping"] = {{{"index", "0x6081"}}};
        j["tpdos"][0]["mapping"] = {{{"index", "0x6081"}}};
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        f.start();
        operational(f);
        fake::inject(0x20a, {33, 0, 0, 0});
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            int32_t target = 0;
            std::memcpy(&target, f.device->model->value(0x6081, 0).data(), sizeof(target));
            return target == 33;
        });
        fake::clear_transmitted();
        CHECK(xnet_canopen_request_tpdo(f.device, 1) == XNET_CO_OK);
        wait_for([] {
            for (const auto& frame : fake::transmitted())
                if (frame.Identifier == 0x18a && frame.Payload[0] == 33) return true;
            return false;
        });
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_observation_sequence == 0);
    }
    {
        Fixture f;
        auto j = drive_402_model();
        j["sync"] = {{"period_us", 0}, {"window_us", 0}};
        j["objects"][7]["pdo"] = "both";
        j["rpdos"][0]["mapping"] = {{{"index", "0x6081"}}};
        j["rpdos"][0]["transmission_type"] = 1;
        j["tpdos"][0]["mapping"] = {{{"index", "0x6081"}}};
        j["tpdos"][0]["transmission_type"] = 0;
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        f.start();
        operational(f);
        fake::clear_transmitted();
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            fake::inject(0x20a, {44, 0, 0, 0});
            fake::inject(0x80, {});
            fake::inject(0x80, {});
            fake::inject(0x80, {});
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_observation_sequence == 3;
        });
        unsigned count = 0;
        for (const auto& frame : fake::transmitted()) if (frame.Identifier == 0x18a) ++count;
        CHECK(count == 1);
        fake::clear_transmitted();
        CHECK(xnet_canopen_request_tpdo(f.device, 1) == XNET_CO_OK);
        fake::inject(0x80, {});
        wait_for([] {
            for (const auto& frame : fake::transmitted())
                if (frame.Identifier == 0x18a && frame.Payload[0] == 44) return true;
            return false;
        });
    }
}

struct TestSyncClock final : xco::SyncClock {
    std::atomic<uint64_t> now{0};
    std::atomic<bool> valid{true};
    std::atomic<int64_t> model_offset_us{0};
    bool map_rx_us(uint64_t raw, uint64_t& mapped) const override {
        mapped = raw;
        return valid;
    }
    bool now_us(uint64_t& current) const override {
        current = now;
        return valid;
    }
    bool map_model_us(const motor_drive_time_v1& time, uint64_t& mapped) const override {
        if (!valid || time.clock_domain_id != 42 || time.flags != MOTOR_DRIVE_TIME_VALID
            || !std::isfinite(time.time_s) || time.time_s < 0 || time.time_s > 1000) return false;
        const auto result = std::llround(time.time_s * 1000000) + model_offset_us.load();
        if (result < 0) return false;
        mapped = static_cast<uint64_t>(result);
        return true;
    }
};

std::shared_ptr<TestSyncClock> install_test_sync_clock(Fixture& f) {
    auto clock = std::make_shared<TestSyncClock>();
    f.device->sync_execution = std::make_unique<xco::SyncExecution>();
    f.device->sync_execution->clock = clock;
    f.device->sync_execution->max_interval_us = 10000;
    f.device->sync_execution->tpdo_window_us = 5000;
    return clock;
}

void worker_timed_sparse_pdo() {
    Fixture f;
    auto j = drive_402_model();
    j["objects"][7]["pdo"] = "both";
    j["rpdos"][0]["mapping"] = {{{"index", "0x6081"}}};
    j["rpdos"][0]["transmission_type"] = 3;
    j["tpdos"][0]["mapping"] = {{{"index", "0x6081"}}};
    j["tpdos"][0]["transmission_type"] = 3;
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.start();
    operational(f);
    fake::clear_transmitted();
    for (uint64_t i = 1; i <= 5; ++i) {
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = i * 1000;
            if (i == 1) fake::inject(0x20a, {11, 0, 0, 0}, nxFrameType_CAN_Data, 0, 900);
            if (i == 4) fake::inject(0x20a, {22, 0, 0, 0}, nxFrameType_CAN_Data, 0, 3900);
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, i * 1000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_execution->invalid || f.device->sync_observation_sequence == i;
        });
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(!f.device->sync_execution->invalid);
    }
    std::vector<uint8_t> values;
    for (const auto& frame : fake::transmitted())
        if (frame.Identifier == 0x18a) values.push_back(frame.Payload[0]);
    CHECK((values == std::vector<uint8_t>{11, 22}));
}

void timing_diagnostic_ring_and_lifecycle() {
    Fixture f;
    xnet_canopen_timing_event_v1 event{};
    event.sequence = 999;
    CHECK(xnet_canopen_read_timing_event_v1(f.device, 0, &event, sizeof(event)) == XNET_CO_NOT_FOUND);
    CHECK(event.sequence == 999);
    auto j = external_drive_402_model();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    f.start();
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    CHECK(xnet_canopen_read_timing_event_v1(f.device, 0, &event, sizeof(event)) == XNET_CO_NOT_FOUND);
    install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.start();
    uint64_t first_epoch = 0;
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        first_epoch = f.device->drive_epoch;
        for (unsigned i = 0; i < 130; ++i)
            f.device->record_timing_event(XNET_CO_TIMING_COMMAND_PUBLISHED);
        f.device->fail(XNET_CO_ACCESS_ERROR, "diagnostic sentinel");
    }
    const auto original_error = diagnostic(f.device);
    CHECK(xnet_canopen_read_timing_event_v1(f.device, 1, &event, sizeof(event)) == XNET_CO_ACCESS_ERROR);
    CHECK(xnet_canopen_read_timing_event_v1(f.device, 131, &event, sizeof(event)) == XNET_CO_INVALID_ARGUMENT);
    CHECK(xnet_canopen_read_timing_event_v1(f.device, 0, &event, sizeof(event) - 1) == XNET_CO_BUFFER_TOO_SMALL);
    CHECK(event.sequence == 999 && diagnostic(f.device) == original_error);
    CHECK(xnet_canopen_read_timing_event_v1(f.device, 0, &event, sizeof(event)) == XNET_CO_OK);
    CHECK(event.sequence == 3 && event.drive_epoch == first_epoch
        && event.abi_version == XNET_CO_TIMING_EVENT_ABI_VERSION && event.struct_size == sizeof(event));
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    f.start();
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->drive_epoch != first_epoch);
        f.device->record_timing_event(XNET_CO_TIMING_COMMAND_PUBLISHED);
    }
    CHECK(xnet_canopen_read_timing_event_v1(f.device, 130, &event, sizeof(event)) == XNET_CO_OK);
    CHECK(event.sequence == 131 && event.drive_epoch != first_epoch);
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    CHECK(xnet_canopen_uninitialize(f.device) == XNET_CO_OK);
    CHECK(xnet_canopen_read_timing_event_v1(f.device, 130, &event, sizeof(event)) == XNET_CO_OK);
    CHECK(event.sequence == 131);
    CHECK(f.initialize() == XNET_CO_OK);
    CHECK(xnet_canopen_read_timing_event_v1(f.device, 130, &event, sizeof(event)) == XNET_CO_OK);
    CHECK(event.sequence == 131);
    install_test_sync_clock(f);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        f.device->timing_event_sequence = UINT64_MAX - 1;
        f.device->record_timing_event(XNET_CO_TIMING_COMMAND_PUBLISHED);
        f.device->record_timing_event(XNET_CO_TIMING_COMMAND_PUBLISHED);
    }
    CHECK(xnet_canopen_read_timing_event_v1(f.device, UINT64_MAX - 1, &event, sizeof(event)) == XNET_CO_OK);
    CHECK(event.sequence == UINT64_MAX);
    CHECK(xnet_canopen_read_timing_event_v1(f.device, UINT64_MAX, &event, sizeof(event)) == XNET_CO_STACK_ERROR);
    CHECK(diagnostic(f.device).find("timing diagnostic sequence exhausted") != std::string::npos);
}

void worker_sampling_deadline_crossed_at_boundary() {
    struct BoundaryClock final : xco::SyncClock {
        std::shared_ptr<TestSyncClock> clock;
        mutable std::atomic<bool> armed{true};
        explicit BoundaryClock(std::shared_ptr<TestSyncClock> source) : clock(std::move(source)) {}
        bool map_rx_us(uint64_t raw, uint64_t& mapped) const override {
            if (raw == 1500 && armed.exchange(false)) clock->now = 2000;
            return clock->map_rx_us(raw, mapped);
        }
        bool now_us(uint64_t& now) const override { return clock->now_us(now); }
        bool map_model_us(const motor_drive_time_v1& time, uint64_t& mapped) const override {
            return clock->map_model_us(time, mapped);
        }
    };
    Fixture f;
    auto j = external_drive_402_model();
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    auto& execution = *f.device->sync_execution;
    execution.clock = std::make_shared<BoundaryClock>(clock);
    execution.test_external_capture = true;
    execution.test_command_plan = xco::TestCommandPlan{500, 0, 10000, 123, 42};
    execution.test_sampling_plan = xco::TestSamplingPlan{1, 1, 1, 1, 1, 1000, 1000, 1000};
    execution.test_sampling_watermark = true;
    f.start();
    operational(f);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 1500;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1500);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return execution.invalid;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(execution.last_sync_us && *execution.last_sync_us == 1500);
        CHECK(std::string(execution.invalid_reason.data()).find("acceptance deadline missed") != std::string::npos);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
}

void worker_required_rpdo_boundaries() {
    for (unsigned scenario = 0; scenario < 11; ++scenario) {
        Fixture f;
        auto j = external_drive_402_model();
        j["objects"][7]["pdo"] = "rx";
        j["objects"][8]["pdo"] = "rx";
        j["rpdos"][0]["transmission_type"] = 1;
        j["rpdos"].push_back({{"transmission_type", 1}, {"mapping", {{{"index", "0x6081"}}}}});
        j["rpdos"].push_back({{"transmission_type", 1}, {"mapping", {{{"index", "0x6083"}}}}});
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        auto& execution = *f.device->sync_execution;
        execution.test_external_capture = true;
        execution.test_ordered_sync_watermark = true;
        execution.test_command_plan = xco::TestCommandPlan{500, 0, 10000, 123, 42};
        execution.test_rpdo_plan = xco::TestRpdoPlan{3, 1};
        f.start();
        operational(f);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = 2000;
            if (scenario == 10) {
                f.device->co->RPDO[0].PDO_common.OD_IO[1].write =
                    [](OD_stream_t*, const void*, OD_size_t, OD_size_t*) { return ODR_INVALID_VALUE; };
            }
            if (scenario == 6) fake::inject(0, {2, 10});
            if (scenario == 2) fake::inject(0x20a, {0, 0, 1}, nxFrameType_CAN_Data, 0, 100);
            else if (scenario == 7) fake::inject(0x20a, {0, 0, 1, 11, 0, 0, 0, 0}, nxFrameType_CAN_Data, 0, 100);
            else if (scenario == 9)
                fake::inject(0x20a, {0, 0, 3, 11, 0, 0, 0}, nxFrameType_CAN_Data, 0, 100);
            else fake::inject(0x20a, {0, 0, 1, 11, 0, 0, 0}, nxFrameType_CAN_Data, 0, 100);
            if (scenario == 3)
                fake::inject(0x20a, {0, 0, 1, 12, 0, 0, 0}, nxFrameType_CAN_Data, 0, 150);
            const uint64_t second_time = scenario == 4 ? 999 : scenario == 8 ? 99 : 200;
            fake::inject(0x30a, {20, 0, 0, 0}, nxFrameType_CAN_Data, 0, second_time);
            if (scenario == 6) fake::inject(0, {1, 10});
            fake::inject(0x40a, {40, 0, 0, 0}, nxFrameType_CAN_Data, 0, 300);
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
            if (scenario <= 1 || scenario == 5) {
                fake::inject(0x20a, {0, 0, 1, 22, 0, 0, 0}, nxFrameType_CAN_Data, 0,
                    scenario == 5 ? 999 : 1100);
                if (scenario != 1)
                    fake::inject(0x30a, {30, 0, 0, 0}, nxFrameType_CAN_Data, 0, 1200);
                fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 2000);
                if (scenario == 1) fake::inject(0x30a, {99, 0, 0, 0}, nxFrameType_CAN_Data, 0, 2100);
            }
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return execution.invalid || f.device->sync_observation_sequence == (scenario <= 1 || scenario == 5 ? 2 : 1);
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            if (scenario == 0) {
                CHECK(!execution.invalid);
                CHECK(execution.test_rpdo_seen == 0);
                CHECK(execution.commands.close_through(f.device->drive_epoch, 2000, 2000) == xco::TimedHistoryResult::ok);
                xco::TimedHistory<xco::TimedDriveCommand, 64>::Record command{};
                CHECK(execution.commands.select(f.device->drive_epoch, 1000, 0, 2000, 3000, 1000, command)
                    == xco::TimedHistoryResult::ok);
                CHECK(command.value.command.target_position == 11 && command.value.command.profile_velocity == 20
                    && command.value.command.profile_acceleration == 40);
                CHECK(execution.commands.select(f.device->drive_epoch, 2000, 0, 2000, 3000, 1000, command)
                    == xco::TimedHistoryResult::ok);
                CHECK(command.value.command.target_position == 22 && command.value.command.profile_velocity == 30
                    && command.value.command.profile_acceleration == 40);
            } else {
                CHECK(execution.invalid);
                const char* reason = scenario == 1 || scenario == 6 ? "required RPDO missing"
                    : scenario == 2 || scenario == 7 ? "wrong length"
                    : scenario == 3 ? "duplicate required RPDO"
                    : scenario == 4 ? "overlaps SYNC"
                    : scenario == 9 ? "unsupported DS402 mode"
                    : scenario == 10 ? "OD application failed" : "timestamp invalid or unordered";
                CHECK(std::string(execution.invalid_reason.data()).find(reason) != std::string::npos);
                CHECK(!f.device->drive_402->status().output_allowed);
                const auto events = timing_events(f.device);
                CHECK(!events.empty() && events.back().kind == XNET_CO_TIMING_FAILURE);
                CHECK(events.back().reason == XNET_CO_TIMING_REASON_RPDO);
                CHECK(events.back().flags & XNET_CO_TIMING_HAS_FRAME);
                if (scenario == 1 || scenario == 6) {
                    CHECK(events.back().flags & XNET_CO_TIMING_HAS_BOUNDARY);
                    CHECK(events.back().boundary.sequence == (scenario == 1 ? 2 : 1));
                }
                if (scenario == 2) {
                    CHECK(!(events.back().flags & XNET_CO_TIMING_HAS_BOUNDARY));
                    CHECK(events.back().frame_can_id == 0x20a && events.back().frame_timestamp_100ns == 100);
                }
                if (scenario == 1) {
                    CHECK(execution.test_sync_watermark_anchor
                        && execution.test_sync_watermark_anchor->mapped_us == 1000);
                    int32_t target = 0;
                    uint32_t velocity = 0;
                    std::memcpy(&target, f.device->model->value(0x607a, 0).data(), sizeof(target));
                    std::memcpy(&velocity, f.device->model->value(0x6081, 0).data(), sizeof(velocity));
                    CHECK(target == 11 && velocity == 20);
                }
            }
        }
        if (scenario == 0) {
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                clock->now = 2500;
                fake::inject(0x20a, {0, 0, 1, 33, 0, 0, 0}, nxFrameType_CAN_Data, 0, 2400);
            }
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return execution.test_rpdo_seen == 1;
            });
            CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
            f.start();
            operational(f);
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(execution.test_rpdo_seen == 0);
                clock->now = 3000;
                fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 3000);
            }
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return execution.invalid;
            });
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(std::string(execution.invalid_reason.data()).find("required RPDO missing") != std::string::npos);
            }
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    }
}

void worker_required_rpdo_plan_validation() {
    for (unsigned scenario = 0; scenario < 4; ++scenario) {
        Fixture f;
        auto j = drive_402_model();
        if (scenario != 2) j["rpdos"][0]["transmission_type"] = 1;
        if (scenario == 3) j["rpdos"][0]["enabled"] = false;
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        install_test_sync_clock(f);
        f.device->sync_execution->test_rpdo_plan = xco::TestRpdoPlan{scenario == 0 ? 0U : scenario == 1 ? 2U : 1U, 0};
        CHECK(xnet_canopen_start(f.device) == XNET_CO_INVALID_ARGUMENT);
        CHECK(diagnostic(f.device).find("RPDO") != std::string::npos);
    }
    Fixture f;
    auto j = drive_402_model();
    j["rpdos"][0]["transmission_type"] = 1;
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    install_test_sync_clock(f);
    f.device->sync_execution->test_rpdo_plan = xco::TestRpdoPlan{1, 0};
    std::ostringstream messages;
    {
        RestoreLogSinks restore(f.device->logger);
        f.device->logger->sinks() = {std::make_shared<spdlog::sinks::ostream_sink_mt>(messages)};
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        f.device->start_test_rpdo_plan();
    }
    CHECK(messages.str().find("strict test RPDO plan requires every selected RPDO before every SYNC")
        != std::string::npos);
}

void worker_ordered_sync_execution() {
    Fixture f;
    auto j = drive_402_model();
    j["rpdos"][0]["transmission_type"] = 1;
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.start();
    operational(f);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 3000;
        fake::inject(0x20a, {6, 0, 1, 0, 0, 0, 0});
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
        fake::inject(0x20a, {7, 0, 1, 0, 0, 0, 0});
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 2000);
        fake::inject(0x20a, {15, 0, 1, 0, 0, 0, 0});
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 3000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->drive_402->status().pds == xco::d402::Pds::operation_enabled;
    });
    clock->now = 3500;
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->co->SYNC->timer == 500;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(!f.device->sync_execution->invalid);
        fake::inject(0x20a, {0x1f, 0, 1, 0x88, 0x13, 0, 0});
    }
    std::this_thread::sleep_for(10ms);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->drive_402->status().demand_position == 0);
        clock->now = 4000;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 4000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return (f.device->drive_402->status().statusword & 0x1000) != 0;
    });
    int32_t demand = 0;
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        demand = f.device->drive_402->status().demand_position;
    }
    std::this_thread::sleep_for(10ms);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->drive_402->status().demand_position == demand);
        clock->now = 14001;
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_execution->invalid;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(!f.device->drive_402->status().output_allowed);
        CHECK(f.device->drive_402->status().model_fault);
    }
    CHECK(diagnostic(f.device).find("SYNC clock invalid or SYNC interval exceeded") != std::string::npos);
}

void worker_sync_tpdo_deadline() {
    Fixture f;
    auto j = drive_402_model();
    j["rpdos"][0]["transmission_type"] = 1;
    j["tpdos"][0]["transmission_type"] = 1;
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.start();
    operational(f);
    fake::clear_transmitted();
    fake::output_full(true);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 1000;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        for (const auto& pending : f.device->outgoing) if (pending.buffer->syncFlag) return true;
        return false;
    });
    clock->now = 6000;
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_execution->invalid;
    });
    fake::output_full(false);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        for (const auto& pending : f.device->outgoing) CHECK(!pending.buffer->syncFlag);
        CHECK(std::string(f.device->sync_execution->invalid_reason.data()).find("TPDO expired") != std::string::npos);
    }
    for (const auto& frame : fake::transmitted()) CHECK(frame.Identifier != 0x18a);
    const auto events = timing_events(f.device);
    CHECK(!events.empty() && events.back().reason == XNET_CO_TIMING_REASON_DEADLINE);
    CHECK(!(events.back().flags & XNET_CO_TIMING_XNET_ACCEPTED));
}

void worker_sync_backlog_is_not_replayed() {
    Fixture f;
    auto j = drive_402_model();
    j["rpdos"][0]["transmission_type"] = 1;
    j["tpdos"][0]["transmission_type"] = 1;
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.start();
    operational(f);
    fake::clear_transmitted();
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 2000;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 2000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_execution->invalid;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_observation_sequence == 2);
        CHECK(std::string(f.device->sync_execution->invalid_reason.data()).find("previous synchronous TPDO") != std::string::npos);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    for (const auto& frame : fake::transmitted()) CHECK(frame.Identifier != 0x18a);
}

void worker_sync_write_crosses_deadline() {
    struct WriteClock { TestSyncClock* clock; uint64_t after_write; };
    for (const auto& test : {std::pair<uint64_t, const char*>{6000, "after deadline"},
                             {999, "clock invalid"}}) {
        WriteClock write_clock{};
        Fixture f;
        auto j = drive_402_model();
        j["tpdos"][0]["transmission_type"] = 1;
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        f.start();
        operational(f);
        fake::clear_transmitted();
        write_clock = {clock.get(), test.first};
        fake::write_hook([](uint32_t id, void* context) {
            auto& data = *static_cast<WriteClock*>(context);
            if (id == 0x18a) data.clock->now = data.after_write;
        }, &write_clock);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = 1000;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_execution->invalid;
        });
        fake::write_hook(nullptr, nullptr);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            const std::string reason = f.device->sync_execution->invalid_reason.data();
            CHECK(reason.find(test.second) != std::string::npos);
            CHECK(reason.find("cannot be recalled") != std::string::npos);
            CHECK((f.device->co->CANmodule->CANerrorStatus & CO_CAN_ERRTX_PDO_LATE) != 0);
            CHECK(!f.device->drive_402->status().output_allowed);
        }
        unsigned count = 0;
        for (const auto& frame : fake::transmitted()) if (frame.Identifier == 0x18a) ++count;
        CHECK(count == 1);
        const auto events = timing_events(f.device);
        CHECK(events.size() >= 2 && events[events.size() - 2].kind == XNET_CO_TIMING_TPDO_ACCEPTED);
        CHECK(events.back().kind == XNET_CO_TIMING_FAILURE);
        CHECK(events.back().reason == (test.first == 6000 ? XNET_CO_TIMING_REASON_DEADLINE : XNET_CO_TIMING_REASON_CLOCK));
        CHECK((events.back().flags & XNET_CO_TIMING_XNET_ACCEPTED) != 0);
        CHECK(events.back().frame_can_id == 0x18a && events.back().boundary.sequence == 1);
    }
}

void worker_sync_write_failure() {
    Fixture f;
    auto j = drive_402_model();
    j["tpdos"][0]["transmission_type"] = 1;
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.start();
    operational(f);
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->outgoing.empty();
    });
    fake::clear_transmitted();
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 1000;
        fake::fail_next("write", nxErrInternalError);
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->state == XNET_CO_RUNTIME_FAILED;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_execution->invalid);
        CHECK(std::string(f.device->sync_execution->invalid_reason.data()).find("XNET write failed") != std::string::npos);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    for (const auto& frame : fake::transmitted()) CHECK(frame.Identifier != 0x18a);
}

void worker_sync_missing_and_overflow() {
    {
        Fixture f;
        auto j = drive_402_model();
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        f.start();
        clock->now = 10001;
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_execution->invalid;
        });
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    {
        Fixture f;
        auto j = drive_402_model();
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        install_test_sync_clock(f);
        f.start();
        fake::fail_next("read", nxErrInputQueueOverflow);
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_execution->invalid;
        });
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(std::string(f.device->sync_execution->invalid_reason.data()).find("input queue overflow") != std::string::npos);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
}

void worker_sync_counter_and_clock() {
    {
        Fixture f;
        auto j = drive_402_model();
        j["tpdos"] = xco::Json::array();
        j["sync"] = {{"counter_overflow", 3}};
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        f.start();
        operational(f);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = 4000;
            fake::inject(0x80, {1}, nxFrameType_CAN_Data, 0, 1000);
            fake::inject(0x80, {2}, nxFrameType_CAN_Data, 0, 2000);
            fake::inject(0x80, {3}, nxFrameType_CAN_Data, 0, 3000);
            fake::inject(0x80, {1}, nxFrameType_CAN_Data, 0, 4000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_execution->last_counter == 1;
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(!f.device->sync_execution->invalid);
            clock->now = 5000;
            fake::inject(0x80, {3}, nxFrameType_CAN_Data, 0, 5000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_execution->invalid;
        });
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(std::string(f.device->sync_execution->invalid_reason.data()).find("counter") != std::string::npos);
    }
    {
        Fixture f;
        auto j = drive_402_model();
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        f.start();
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = 1000;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_execution->last_sync_us == 1000;
        });
        clock->now = 999;
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_execution->invalid;
        });
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
}

void worker_sync_external_rejected() {
    Fixture f;
    f.model(external_drive_402_model());
    CHECK(f.initialize() == XNET_CO_OK);
    install_test_sync_clock(f);
    CHECK(xnet_canopen_start(f.device) == XNET_CO_INVALID_STATE);
    CHECK(diagnostic(f.device).find("internal model") != std::string::npos);
}

// Some history tests inject active plans to exercise transitions that a frozen configuration cannot request.
void worker_external_timed_history() {
    Fixture f;
    auto j = external_drive_402_model();
    j["drive_402"]["action_timeout_us"] = 10000;
    j["rpdos"][0]["transmission_type"] = 1;
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.device->sync_execution->test_auto_retire = true;
    f.start();
    operational(f);
    motor_drive_endpoint_handle endpoint = nullptr;
    motor_drive_port_handle port = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 3000;
        fake::inject(0x20a, {0, 0, 1, 11, 0, 0, 0});
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
        fake::inject(0x20a, {0, 0, 1, 22, 0, 0, 0});
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 2000);
        fake::inject(0x20a, {0, 0, 1, 33, 0, 0, 0});
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 3000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_observation_sequence == 3;
    });
    using Result = xco::TimedHistoryResult;
    using CommandHistory = xco::TimedHistory<xco::TimedDriveCommand, 64>;
    using FeedbackHistory = xco::TimedHistory<xco::TimedDriveFeedback, 64>;
    xco::Clock::time_point accepted_at;
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        auto& execution = *f.device->sync_execution;
        CommandHistory::Record selected{};
        const auto epoch = f.device->drive_epoch;
        CHECK(!execution.invalid);
        CHECK(execution.commands.select(epoch, 2500, 0, 3000, 4000, 5000, selected) == Result::not_closed);
        CHECK(execution.commands.close_through(epoch, 2500, 3000) == Result::ok);
        CHECK(execution.commands.select(epoch, 2500, 0, 3000, 4000, 5000, selected) == Result::ok);
        CHECK(selected.sequence == 2 && selected.source_time == 2000 && selected.available_time == 3000);
        CHECK(selected.value.sync.sequence == 2 && selected.value.sync.timestamp_100ns == 2000);
        CHECK(selected.value.command.epoch == epoch && selected.value.command.target_position == 22);
        CHECK(execution.commands.retire_through(epoch, 1500) == Result::ok);
        CHECK(execution.commands.select(epoch, 2500, 0, 3000, 4000, 1000, selected) == Result::history_gap);
        CHECK(execution.commands.select(epoch, 2500, 0, 3000, 4000, 999, selected) == Result::ok);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    motor_drive_read_request_v1 request{};
    request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_ASYNC;
    for (uint32_t seq = 1; seq <= 2; ++seq) {
        motor_drive_view_v1 view{};
        CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
        motor_drive_feedback_v1 feedback{};
        feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
        feedback.source = view.token;
        feedback.executed_step = {1, seq, seq, seq * 0.002, 0.002,
            {42, MOTOR_DRIVE_TIME_VALID, seq * 0.0005}};
        feedback.feedback_seq = seq; feedback.flags = 0x17;
        feedback.actual_position_rad = seq * 10 * 6.2831853071795864769 / 10000;
        CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->drive_processed_feedback_seq == seq;
        });
    }
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        auto& execution = *f.device->sync_execution;
        FeedbackHistory::Record selected{};
        const auto epoch = f.device->drive_epoch;
        CHECK(!execution.invalid);
        CHECK(execution.feedback.select(epoch, 750, 0, 3000, 4000, 1000, selected) == Result::not_closed);
        CHECK(execution.feedback.close_through(epoch, 750, 3000) == Result::ok);
        CHECK(execution.feedback.select(epoch, 750, 0, 3000, 4000, 1000, selected) == Result::ok);
        CHECK(selected.value.feedback.feedback_seq == 1 && selected.value.converted.actual_position == 10);
        CHECK(selected.value.feedback.executed_step.model_step_seq == 1);
        CHECK(execution.feedback.close_through(epoch, 1000, 3000) == Result::ok);
        CHECK(execution.feedback.select(epoch, 1000, 0, 3000, 4000, 1000, selected) == Result::ok);
        CHECK(selected.value.feedback.feedback_seq == 2 && selected.value.converted.actual_position == 20);
        CHECK(f.device->drive_402->status().actual_position == 0);
        CHECK(!f.device->drive_402->status().output_allowed);
        accepted_at = f.device->drive_feedback_accepted_at;
        CHECK(execution.feedback.close_through(epoch, 3000, 3000) == Result::ok);
        execution.active_test_feedback_plan = xco::TestFeedbackPlan{1000, 1, 3002};
        clock->now = 4001;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 4000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_observation_sequence == 4;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(!f.device->sync_execution->invalid);
        int32_t actual = 0;
        std::memcpy(&actual, f.device->model->value(0x6064, 0).data(), sizeof(actual));
        CHECK(actual == 20 && f.device->drive_402->status().actual_position == 20);
        CHECK(f.device->drive_feedback_accepted_at == accepted_at);
        CHECK(!f.device->drive_402->status().output_allowed);
        FeedbackHistory::Record selected{};
        CHECK(f.device->sync_execution->feedback.select(f.device->drive_epoch, 1000, 0,
            4001, 5000, 501, selected) == Result::history_gap);
        CHECK(f.device->sync_execution->feedback.select(f.device->drive_epoch, 1000, 0,
            4001, 5000, 0, selected) == Result::ok);
        CHECK(selected.value.feedback.feedback_seq == 2);
        CHECK(f.device->sync_execution->feedback.close_through(f.device->drive_epoch, 4000, 4001) == Result::ok);
        f.device->sync_execution->active_test_feedback_plan->max_age_us = 4001;
        clock->now = 5000;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 5000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_observation_sequence == 5;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(!f.device->sync_execution->invalid);
        CHECK(f.device->drive_feedback_accepted_at == accepted_at);
        CHECK(!f.device->drive_402->status().output_allowed);
        CHECK(f.device->sync_execution->feedback.close_through(f.device->drive_epoch, 5000, 5000) == Result::ok);
        f.device->sync_execution->active_test_feedback_plan->max_age_us = 5000;
        clock->now = 6000;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 6000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_execution->invalid;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(std::string(f.device->sync_execution->invalid_reason.data()).find("missing or stale") != std::string::npos);
        CHECK(!f.device->drive_402->status().feedback_fresh);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
}

void worker_external_scheduled_auto_retirement() {
    Fixture f;
    auto j = external_drive_402_model();
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.device->sync_execution->test_auto_retire = true;
    f.device->sync_execution->test_ordered_sync_watermark = true;
    f.device->sync_execution->test_command_plan = xco::TestCommandPlan{500, 0, 1500, 123, 42};
    f.start();
    operational(f);
    motor_drive_endpoint_handle endpoint = nullptr;
    motor_drive_port_handle port = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    motor_drive_read_request_v1 request{};
    request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_SCHEDULED;
    motor_drive_view_v1 view{};
    for (uint32_t i = 1; i <= 70; ++i) {
        const uint64_t source = i * 1000ULL, target = source + 500;
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = source;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, source);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_observation_sequence == i || f.device->sync_execution->invalid;
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(!f.device->sync_execution->invalid);
            if (i > 1) CHECK(f.device->drive_402->status().actual_position == static_cast<int32_t>(i - 1));
        }
        request.target_step = {1, i, i, i * 0.001, 0.001, {42, MOTOR_DRIVE_TIME_VALID, target / 1000000.0}};
        request.source_cutoff = {42, MOTOR_DRIVE_TIME_VALID, source / 1000000.0};
        CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
        CHECK(view.source_cycle.sync_cycle_id == i);
        motor_drive_feedback_v1 feedback{};
        feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
        feedback.source = view.token; feedback.source_cycle = view.source_cycle;
        feedback.executed_step = request.target_step; feedback.feedback_seq = i; feedback.flags = 0x17;
        feedback.actual_position_rad = i * 6.2831853071795864769 / 10000;
        clock->now = target;
        CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->drive_processed_feedback_seq == i || f.device->sync_execution->invalid;
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(!f.device->sync_execution->invalid);
            CHECK(f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
            auto& history = f.device->sync_execution->feedback;
            CHECK(history.close_through(f.device->drive_epoch, target, target) == xco::TimedHistoryResult::ok);
            if (i == 1) f.device->sync_execution->active_test_feedback_plan = xco::TestFeedbackPlan{500, 0, 1500};
        }
    }
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
        const auto& history = f.device->sync_execution->feedback;
        CHECK(history.select(f.device->drive_epoch, 70500, 0, 70500, 71000, 2000, selected)
            == xco::TimedHistoryResult::ok);
        CHECK(selected.value.feedback.feedback_seq == 70);
        CHECK(history.select(f.device->drive_epoch, 70500, 0, 70500, 71000, 2001, selected)
            == xco::TimedHistoryResult::history_gap);
        f.device->sync_execution->active_test_command_plan->max_age_us = 100000;
    }
    request.target_step = {1, 71, 71, 0.071, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.0715}};
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_STALE);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_execution->invalid);
        CHECK(std::string(f.device->sync_execution->invalid_reason.data()).find("retired") != std::string::npos);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    CHECK(timing_events(f.device).back().reason == XNET_CO_TIMING_REASON_HISTORY_GAP);
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
}

void worker_external_ordered_command_watermark() {
    struct RxMappingClock final : xco::SyncClock {
        std::shared_ptr<TestSyncClock> clock;
        std::atomic<bool> shifted{false};
        std::atomic<uint64_t> shifted_raw{UINT64_MAX};
        explicit RxMappingClock(std::shared_ptr<TestSyncClock> source) : clock(std::move(source)) {}
        bool map_rx_us(uint64_t raw, uint64_t& mapped) const override {
            const bool adjust = shifted || raw == shifted_raw;
            if (!clock->map_rx_us(raw, mapped) || (adjust && mapped == UINT64_MAX)) return false;
            if (adjust) ++mapped;
            return true;
        }
        bool now_us(uint64_t& now) const override { return clock->now_us(now); }
        bool map_model_us(const motor_drive_time_v1& time, uint64_t& mapped) const override {
            return clock->map_model_us(time, mapped);
        }
    };
    for (unsigned scenario = 0; scenario < 6; ++scenario) {
        Fixture f;
        auto j = external_drive_402_model();
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        const auto mapping = std::make_shared<RxMappingClock>(clock);
        auto& execution = *f.device->sync_execution;
        execution.clock = mapping;
        execution.test_external_capture = true;
        execution.test_ordered_sync_watermark = true;
        execution.test_command_plan = xco::TestCommandPlan{500, 1, 10000, 123, 42};
        f.start();
        operational(f);
        motor_drive_endpoint_handle endpoint = nullptr;
        motor_drive_port_handle port = nullptr;
        CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
        CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
        std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
        motor_drive_read_request_v1 request{};
        request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_SCHEDULED;
        request.target_step = {1, 1, 1, 0.001, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.005}};
        request.source_cutoff = {42, MOTOR_DRIVE_TIME_VALID, 0.0015};
        motor_drive_view_v1 view{};
        view.token.command_seq = 0xabcdef;
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            execution.test_ordered_sync_watermark = false;
            execution.test_command_plan->lead_us = 100000;
            clock->now = 4000;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 2000);
            CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_NO_DATA);
            CHECK(view.token.command_seq == 0xabcdef);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_observation_sequence == 2 || execution.invalid;
        });
        request.source_cutoff.time_s = 0.0025;
        CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_NO_DATA);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(!execution.invalid && execution.active_test_ordered_sync_watermark);
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 3000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_observation_sequence == 3 || execution.invalid;
        });
        if (scenario == 3) mapping->shifted_raw = 2000;
        const auto read_result = motor_drive_read_v1(port, &request, &view, sizeof(view));
        CHECK(read_result == (scenario == 3 ? MOTOR_DRIVE_INVALID_STATE : MOTOR_DRIVE_OK));
        if (scenario != 3)
            CHECK(view.source_cycle.sync_cycle_id == 2 && view.source_cycle.rx_time.time_s == 0.002);
        if (scenario == 0) {
            auto future = request;
            future.target_step = {1, 2, 2, 0.002, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.006}};
            future.source_cutoff.time_s = 0.003001;
            CHECK(motor_drive_read_v1(port, &future, &view, sizeof(view)) == MOTOR_DRIVE_NO_DATA);
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1500);
            }
        } else if (scenario == 1) {
            mapping->shifted = true;
            CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_INVALID_STATE);
        } else if (scenario == 4) {
            mapping->shifted_raw = 2000;
            CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_INVALID_STATE);
        } else if (scenario == 2 || scenario == 5) {
            motor_drive_feedback_v1 feedback{};
            feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
            feedback.source = view.token; feedback.source_cycle = view.source_cycle;
            feedback.executed_step = request.target_step; feedback.feedback_seq = 1; feedback.flags = 0x17;
            feedback.fault_class = MOTOR_DRIVE_FAULT_OVERCURRENT; feedback.fault_detail = 7;
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            if (scenario == 5)
                CHECK(execution.commands.retire_through(f.device->drive_epoch, 2000) == xco::TimedHistoryResult::ok);
            clock->now = 5000;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
            if (scenario == 2) mapping->shifted = true;
            else mapping->shifted_raw = 2000;
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return execution.invalid;
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            const char* reason = scenario == 0 ? "SYNC timestamp regressed"
                : scenario <= 2 ? "watermark clock mapping changed" : "source clock mapping changed";
            CHECK(std::string(execution.invalid_reason.data()).find(reason) != std::string::npos);
            CHECK(!f.device->drive_402->status().output_allowed);
            if (scenario == 2 || scenario == 5) {
                CHECK(f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_INVALID);
                CHECK(f.device->drive_402->status().model_fault);
            }
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    }
    Fixture f;
    f.model(external_drive_402_model());
    CHECK(f.initialize() == XNET_CO_OK);
    install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.device->sync_execution->test_ordered_sync_watermark = true;
    CHECK(xnet_canopen_start(f.device) == XNET_CO_INVALID_STATE);
    CHECK(diagnostic(f.device).find("ordered SYNC test watermark") != std::string::npos);
}

void worker_external_scheduled_read() {
    Fixture f;
    auto j = external_drive_402_model();
    j["drive_402"]["action_timeout_us"] = 10000;
    j["rpdos"][0]["transmission_type"] = 1;
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.device->sync_execution->test_command_plan = xco::TestCommandPlan{500, 1, 3001, 123, 42};
    f.start();
    operational(f);
    motor_drive_endpoint_handle endpoint = nullptr;
    motor_drive_port_handle port = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 3000;
        for (uint8_t i = 1; i <= 3; ++i) {
            fake::inject(0x20a, {0, 0, 1, static_cast<uint8_t>(i * 11), 0, 0, 0});
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, i * 1000);
        }
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_observation_sequence == 3;
    });
    motor_drive_read_request_v1 request{};
    request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_SCHEDULED;
    request.target_step = {1, 10, 10, 0.02, 0.002, {42, MOTOR_DRIVE_TIME_VALID, 0.005}};
    request.source_cutoff = {42, MOTOR_DRIVE_TIME_VALID, 0.0025};
    motor_drive_view_v1 view{};
    view.token.command_seq = 0xabcdef;
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_NO_DATA);
    CHECK(view.token.command_seq == 0xabcdef);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_execution->commands.close_through(f.device->drive_epoch, 2500, 3000)
            == xco::TimedHistoryResult::ok);
    }
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
    CHECK(view.source_cycle.sync_domain_id == 123 && view.source_cycle.sync_cycle_id == 2);
    CHECK(view.source_cycle.sync_domain_epoch != 0 && view.source_cycle.sync_counter == 0);
    CHECK(view.source_cycle.rx_time.clock_domain_id == 42
        && view.source_cycle.rx_time.flags == MOTOR_DRIVE_TIME_VALID
        && view.source_cycle.rx_time.time_s == 0.002);
    CHECK(view.publish_time.clock_domain_id == 42 && view.publish_time.time_s == 0.003);
    CHECK(view.target_step.pcl_epoch == 1 && view.target_step.pcl_tick == 10
        && view.target_step.model_step_seq == 10);
    CHECK((view.flags & MOTOR_DRIVE_OUTPUT_ALLOWED) == 0);
    motor_drive_view_v1 retry{};
    CHECK(motor_drive_read_v1(port, &request, &retry, sizeof(retry)) == MOTOR_DRIVE_OK);
    CHECK(retry.token.command_seq == view.token.command_seq && port->test_issue_count == 1);
    auto changed_cutoff = request;
    changed_cutoff.source_cutoff.time_s = 0.002499;
    CHECK(motor_drive_read_v1(port, &changed_cutoff, &retry, sizeof(retry)) == MOTOR_DRIVE_INVALID_ARGUMENT);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        xco::TimedHistory<xco::TimedDriveCommand, 64>::Record selected{};
        CHECK(f.device->sync_execution->commands.select(f.device->drive_epoch, 2500, 1, 3000, 5000, 500, selected)
            == xco::TimedHistoryResult::ok);
        CHECK(view.token.epoch == selected.value.command.epoch
            && view.token.command_seq == selected.value.command.command_seq
            && selected.value.command.target_position == 22);
    }
    auto invalid_request = request;
    invalid_request.target_step.pcl_epoch = 2;
    CHECK(motor_drive_read_v1(port, &invalid_request, &view, sizeof(view)) == MOTOR_DRIVE_STALE);
    invalid_request = request;
    invalid_request.source_cutoff.clock_domain_id = 99;
    CHECK(motor_drive_read_v1(port, &invalid_request, &view, sizeof(view)) == MOTOR_DRIVE_INVALID_ARGUMENT);
    motor_drive_feedback_v1 feedback{};
    feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
    feedback.source = view.token; feedback.source_cycle = view.source_cycle;
    feedback.executed_step = request.target_step;
    feedback.feedback_seq = 1; feedback.flags = 0x17;
    auto forged = feedback;
    ++forged.source_cycle.sync_cycle_id;
    CHECK(motor_drive_submit_v1(port, &forged, sizeof(forged)) == MOTOR_DRIVE_INVALID_FEEDBACK);
    forged = feedback;
    forged.source_cycle = {};
    CHECK(motor_drive_submit_v1(port, &forged, sizeof(forged)) == MOTOR_DRIVE_INVALID_FEEDBACK);
    forged = feedback;
    ++forged.executed_step.model_step_seq;
    CHECK(motor_drive_submit_v1(port, &forged, sizeof(forged)) == MOTOR_DRIVE_INVALID_FEEDBACK);
    forged = feedback;
    forged.executed_step = {1, 11, 11, 0.022, 0.002, {42, MOTOR_DRIVE_TIME_VALID, 0.007}};
    CHECK(motor_drive_submit_v1(port, &forged, sizeof(forged)) == MOTOR_DRIVE_INVALID_FEEDBACK);
    CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_STALE);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        for (unsigned i = 0; i < 65; ++i) f.device->process_drive_402(0, false, true);
        const auto& overwritten = f.device->drive_command_history[view.token.command_seq % f.device->drive_command_history.size()];
        CHECK(overwritten.seq != view.token.command_seq);
    }
    clock->now = 5000;
    CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->drive_processed_feedback_seq == 1;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
        CHECK(f.device->drive_feedback_history[1].feedback.source_cycle.sync_cycle_id == 2);
    }
    CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
    forged = feedback;
    ++forged.source_cycle.sync_cycle_id;
    CHECK(motor_drive_submit_v1(port, &forged, sizeof(forged)) == MOTOR_DRIVE_INVALID_FEEDBACK);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        f.device->sync_execution->active_test_command_plan->max_age_us = 6000;
    }
    auto next = request;
    next.target_step = {1, 11, 11, 0.022, 0.002, {42, MOTOR_DRIVE_TIME_VALID, 0.007}};
    motor_drive_view_v1 next_view{};
    CHECK(motor_drive_read_v1(port, &next, &next_view, sizeof(next_view)) == MOTOR_DRIVE_OK);
    CHECK(port->test_issue_count == 2);
    auto next_feedback = feedback;
    next_feedback.source = next_view.token; next_feedback.source_cycle = next_view.source_cycle;
    next_feedback.executed_step = next.target_step; next_feedback.feedback_seq = 2;
    next_feedback.actual_position_rad = 30 * 6.2831853071795864769 / 10000;
    clock->now = 7000;
    CHECK(motor_drive_submit_v1(port, &next_feedback, sizeof(next_feedback)) == MOTOR_DRIVE_OK);
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->drive_processed_feedback_seq == 2;
    });
    CHECK(port->test_issue_count == 1 && port->test_issues[0].view.target_step.model_step_seq == 11);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
        CHECK(f.device->sync_execution->feedback.close_through(f.device->drive_epoch, 7000, 7000)
            == xco::TimedHistoryResult::ok);
        f.device->sync_execution->active_test_feedback_plan = xco::TestFeedbackPlan{1000, 0, 2000};
        clock->now = 8000;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 8000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_observation_sequence == 4;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        int32_t actual = 0;
        std::memcpy(&actual, f.device->model->value(0x6064, 0).data(), sizeof(actual));
        CHECK(actual == 30);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    {
        const auto events = timing_events(f.device);
        const xnet_canopen_timing_event_v1 *assigned = nullptr, *accepted = nullptr, *selected = nullptr;
        unsigned assigned_count = 0;
        for (const auto& event : events) {
            if (event.kind == XNET_CO_TIMING_COMMAND_ASSIGNED) {
                ++assigned_count;
                if (event.target_step.model_step_seq == 11) assigned = &event;
            } else if (event.kind == XNET_CO_TIMING_FEEDBACK_ACCEPTED && event.feedback_seq == 2) accepted = &event;
            else if (event.kind == XNET_CO_TIMING_FEEDBACK_SELECTED && event.feedback_seq == 2) selected = &event;
        }
        CHECK(assigned_count == 2);
        CHECK(assigned);
        CHECK(accepted);
        CHECK(selected);
        CHECK(assigned->sequence < accepted->sequence && accepted->sequence < selected->sequence);
        CHECK(assigned->source.command_seq == next_view.token.command_seq
            && accepted->source.command_seq == assigned->source.command_seq
            && selected->source.command_seq == assigned->source.command_seq);
        CHECK(assigned->source_cycle.sync_cycle_id == 2 && accepted->source_cycle.sync_cycle_id == 2
            && selected->source_cycle.sync_cycle_id == 2);
        CHECK(assigned->target_step.model_step_seq == 11 && accepted->target_step.model_step_seq == 11
            && selected->target_step.model_step_seq == 11 && selected->executed_step.model_step_seq == 11);
        CHECK((selected->flags & (XNET_CO_TIMING_HAS_SOURCE_CYCLE | XNET_CO_TIMING_HAS_TARGET
            | XNET_CO_TIMING_HAS_FEEDBACK | XNET_CO_TIMING_HAS_BOUNDARY | XNET_CO_TIMING_HAS_SUBMITTED_STEADY_TIME))
            == (XNET_CO_TIMING_HAS_SOURCE_CYCLE | XNET_CO_TIMING_HAS_TARGET
                | XNET_CO_TIMING_HAS_FEEDBACK | XNET_CO_TIMING_HAS_BOUNDARY | XNET_CO_TIMING_HAS_SUBMITTED_STEADY_TIME));
        CHECK(selected->boundary.sequence == 4 && selected->source_us == 7000
            && selected->available_us == 7000 && selected->decision_us == 8000);
        CHECK(accepted->submitted_steady_us == selected->submitted_steady_us);
    }
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_STALE);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_execution->invalid);
        CHECK(std::string(f.device->sync_execution->invalid_reason.data()).find("deadline missed") != std::string::npos);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    CHECK(timing_events(f.device).back().reason == XNET_CO_TIMING_REASON_DEADLINE);
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
}

void worker_external_scheduled_assignment_overflow() {
    Fixture f;
    auto j = external_drive_402_model();
    j["drive_402"]["action_timeout_us"] = 10000;
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.device->sync_execution->test_auto_retire = true;
    f.device->sync_execution->test_command_plan = xco::TestCommandPlan{500, 0, 10000, 123, 42};
    f.start();
    operational(f);
    motor_drive_endpoint_handle endpoint = nullptr;
    motor_drive_port_handle port = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 2000;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_observation_sequence == 1;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_execution->commands.close_through(f.device->drive_epoch, 1500, 2000)
            == xco::TimedHistoryResult::ok);
    }
    motor_drive_read_request_v1 request{};
    request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_SCHEDULED;
    request.source_cutoff = {42, MOTOR_DRIVE_TIME_VALID, 0.0015};
    for (uint64_t seq = 1; seq <= 65; ++seq) {
        request.target_step = {1, seq, seq, seq * 0.002, 0.002,
            {42, MOTOR_DRIVE_TIME_VALID, (3000 + seq) / 1000000.0}};
        motor_drive_view_v1 view{};
        const auto result = motor_drive_read_v1(port, &request, &view, sizeof(view));
        CHECK(result == (seq <= 64 ? MOTOR_DRIVE_OK : MOTOR_DRIVE_INVALID_STATE));
    }
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_execution->invalid);
        CHECK(std::string(f.device->sync_execution->invalid_reason.data()).find("assignment history overflow") != std::string::npos);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    CHECK(timing_events(f.device).back().reason == XNET_CO_TIMING_REASON_HISTORY_OVERFLOW);
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
}

void worker_external_scheduled_safety_preemption() {
    Fixture f;
    auto j = external_drive_402_model();
    j["drive_402"]["action_timeout_us"] = 10000;
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.device->sync_execution->test_command_plan = xco::TestCommandPlan{500, 0, 10000, 123, 42};
    f.start();
    operational(f);
    motor_drive_endpoint_handle endpoint = nullptr;
    motor_drive_port_handle port = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 2000;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_observation_sequence == 1;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_execution->commands.close_through(f.device->drive_epoch, 1500, 2000)
            == xco::TimedHistoryResult::ok);
    }
    motor_drive_read_request_v1 scheduled{};
    scheduled.abi_version = 1; scheduled.struct_size = sizeof(scheduled); scheduled.flags = MOTOR_DRIVE_READ_SCHEDULED;
    scheduled.source_cutoff = {42, MOTOR_DRIVE_TIME_VALID, 0.0015};
    scheduled.target_step = {1, 10, 10, 0.02, 0.002, {42, MOTOR_DRIVE_TIME_VALID, 0.005}};
    motor_drive_view_v1 issued{};
    CHECK(motor_drive_read_v1(port, &scheduled, &issued, sizeof(issued)) == MOTOR_DRIVE_OK);
    motor_drive_read_request_v1 asynchronous{};
    asynchronous.abi_version = 1; asynchronous.struct_size = sizeof(asynchronous); asynchronous.flags = MOTOR_DRIVE_READ_ASYNC;
    motor_drive_view_v1 current{};
    CHECK(motor_drive_read_v1(port, &asynchronous, &current, sizeof(current)) == MOTOR_DRIVE_OK);
    CHECK(current.token.command_seq != issued.token.command_seq);
    motor_drive_feedback_v1 feedback{};
    feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
    feedback.source = current.token;
    feedback.executed_step = {1, 1, 1, 0.002, 0.002, {42, MOTOR_DRIVE_TIME_VALID, 0.0015}};
    feedback.feedback_seq = 1; feedback.flags = 0x17;
    feedback.fault_class = MOTOR_DRIVE_FAULT_OVERCURRENT; feedback.fault_detail = 7;
    CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->drive_current_fault == 0x2310;
    });
    CHECK(motor_drive_read_v1(port, &scheduled, &current, sizeof(current)) == MOTOR_DRIVE_STALE);
    scheduled.target_step = {1, 11, 11, 0.022, 0.002, {42, MOTOR_DRIVE_TIME_VALID, 0.006}};
    CHECK(motor_drive_read_v1(port, &scheduled, &current, sizeof(current)) == MOTOR_DRIVE_STALE);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(!f.device->sync_execution->invalid);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
}

void worker_external_scheduled_feedback_ages_before_acceptance() {
    Fixture f;
    auto j = external_drive_402_model();
    j["drive_402"]["action_timeout_us"] = 10000;
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.device->sync_execution->test_command_plan = xco::TestCommandPlan{500, 0, 10000, 123, 42};
    f.start();
    operational(f);
    motor_drive_endpoint_handle endpoint = nullptr;
    motor_drive_port_handle port = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 2000;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_observation_sequence == 1;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_execution->commands.close_through(f.device->drive_epoch, 1500, 2000)
            == xco::TimedHistoryResult::ok);
    }
    motor_drive_read_request_v1 request{};
    request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_SCHEDULED;
    request.source_cutoff = {42, MOTOR_DRIVE_TIME_VALID, 0.0015};
    request.target_step = {1, 1, 1, 0.002, 0.002, {42, MOTOR_DRIVE_TIME_VALID, 0.003}};
    motor_drive_view_v1 view{};
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_execution->commands.retire_through(f.device->drive_epoch, 1000)
            == xco::TimedHistoryResult::ok);
    }
    motor_drive_view_v1 retry{};
    CHECK(motor_drive_read_v1(port, &request, &retry, sizeof(retry)) == MOTOR_DRIVE_OK);
    CHECK(retry.token.command_seq == view.token.command_seq && retry.source_cycle.sync_cycle_id == 1);
    motor_drive_feedback_v1 feedback{};
    feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
    feedback.source = view.token; feedback.source_cycle = view.source_cycle;
    feedback.executed_step = request.target_step; feedback.feedback_seq = 1; feedback.flags = 0x17;
    feedback.fault_class = MOTOR_DRIVE_FAULT_OVERCURRENT; feedback.fault_detail = 7;
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 3000;
        CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
        clock->now = 102000;
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->drive_processed_feedback_seq == 1;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_STALE);
        CHECK(f.device->drive_current_fault == 0x2310);
        CHECK(f.device->drive_402->status().model_fault);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    {
        const auto events = timing_events(f.device);
        bool rejected = false, accepted = false;
        for (const auto& event : events) {
            if (event.feedback_seq != 1) continue;
            if (event.kind == XNET_CO_TIMING_FEEDBACK_REJECTED) {
                rejected = event.reason == XNET_CO_TIMING_REASON_STALE;
            }
            if (event.kind == XNET_CO_TIMING_FEEDBACK_ACCEPTED) accepted = true;
        }
        CHECK(rejected && !accepted);
    }
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
}

void worker_external_selected_feedback_source_validity() {
    struct SourceClock final : xco::SyncClock {
        std::shared_ptr<TestSyncClock> clock;
        std::atomic<unsigned> changed{0};
        explicit SourceClock(std::shared_ptr<TestSyncClock> source) : clock(std::move(source)) {}
        bool map_rx_us(uint64_t raw, uint64_t& mapped) const override {
            if (!clock->map_rx_us(raw, mapped)) return false;
            if (raw == 1000) {
                if (changed == 2) return false;
                if (changed == 1) ++mapped;
            }
            return true;
        }
        bool now_us(uint64_t& now) const override { return clock->now_us(now); }
        bool map_model_us(const motor_drive_time_v1& time, uint64_t& mapped) const override {
            return clock->map_model_us(time, mapped);
        }
    };
    for (unsigned scenario = 0; scenario < 5; ++scenario) {
        Fixture f;
        auto j = external_drive_402_model();
        j["drive_402"]["command_max_age_us"] = 4000;
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        const auto mapping = std::make_shared<SourceClock>(clock);
        auto& execution = *f.device->sync_execution;
        execution.clock = mapping;
        execution.test_external_capture = true;
        execution.test_ordered_sync_watermark = scenario != 4;
        execution.test_command_plan = xco::TestCommandPlan{500, 100, 10000, 123, 42};
        f.start();
        operational(f);
        motor_drive_endpoint_handle endpoint = nullptr;
        motor_drive_port_handle port = nullptr;
        CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
        CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
        std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = 2000;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 2000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_observation_sequence == 2 || execution.invalid;
        });
        if (scenario == 4) {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(execution.commands.close_through(f.device->drive_epoch, 2000, 2000)
                == xco::TimedHistoryResult::ok);
        }
        motor_drive_read_request_v1 request{};
        request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_SCHEDULED;
        request.source_cutoff = {42, MOTOR_DRIVE_TIME_VALID, 0.0015};
        request.target_step = {1, 1, 1, 0.001, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.003}};
        motor_drive_view_v1 view{};
        CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
        CHECK(view.source_cycle.sync_cycle_id == 1);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(execution.commands.retire_through(f.device->drive_epoch, 1000) == xco::TimedHistoryResult::ok);
        }
        motor_drive_feedback_v1 feedback{};
        feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
        feedback.source = view.token; feedback.source_cycle = view.source_cycle;
        feedback.executed_step = request.target_step; feedback.feedback_seq = 1; feedback.flags = 0x17;
        feedback.actual_position_rad = 17 * 6.2831853071795864769 / 10000;
        clock->now = 3000;
        CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->drive_processed_feedback_seq == 1 || execution.invalid;
        });
        owned.reset();
        const uint64_t boundary = scenario == 1 ? 4901 : 4900;
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(!execution.invalid && f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
            clock->now = boundary;
            CHECK(execution.feedback.close_through(f.device->drive_epoch, boundary - 1000, boundary)
                == xco::TimedHistoryResult::ok);
            execution.active_test_feedback_plan = xco::TestFeedbackPlan{1000, 0, 6000};
            if (scenario >= 2) mapping->changed = scenario == 3 ? 2 : 1;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, boundary);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_observation_sequence == 3 || execution.invalid;
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            int32_t actual = 0;
            std::memcpy(&actual, f.device->model->value(0x6064, 0).data(), sizeof(actual));
            if (scenario == 0) {
                CHECK(!execution.invalid && actual == 17);
            } else {
                CHECK(execution.invalid && actual == 0);
                const char* reason = scenario == 1 ? "feedback source command stale"
                    : "feedback source command clock mapping changed";
                CHECK(std::string(execution.invalid_reason.data()).find(reason) != std::string::npos);
                CHECK(!f.device->drive_402->status().output_allowed);
                const auto events = timing_events(f.device);
                CHECK(events.back().kind == XNET_CO_TIMING_FAILURE);
                CHECK(events.back().reason == (scenario == 1 ? XNET_CO_TIMING_REASON_STALE : XNET_CO_TIMING_REASON_CLOCK));
                CHECK(events.back().feedback_seq == 1 && events.back().target_step.model_step_seq == 1);
                CHECK(events.back().boundary.sequence == 3);
            }
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    }
}

void worker_external_scheduled_mapping_change() {
    for (bool feedback_path : {false, true}) {
        Fixture f;
        auto j = external_drive_402_model();
        j["drive_402"]["action_timeout_us"] = 10000;
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        f.device->sync_execution->test_external_capture = true;
        f.device->sync_execution->test_command_plan = xco::TestCommandPlan{500, 0, 10000, 123, 42};
        f.start();
        operational(f);
        motor_drive_endpoint_handle endpoint = nullptr;
        motor_drive_port_handle port = nullptr;
        CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
        CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
        std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = 2000;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_observation_sequence == 1;
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(f.device->sync_execution->commands.close_through(f.device->drive_epoch, 1500, 2000)
                == xco::TimedHistoryResult::ok);
        }
        motor_drive_read_request_v1 request{};
        request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_SCHEDULED;
        request.source_cutoff = {42, MOTOR_DRIVE_TIME_VALID, 0.0015};
        request.target_step = {1, 1, 1, 0.002, 0.002, {42, MOTOR_DRIVE_TIME_VALID, 0.003}};
        motor_drive_view_v1 view{};
        CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
        clock->model_offset_us = 1;
        if (feedback_path) {
            clock->now = 3000;
            motor_drive_feedback_v1 feedback{};
            feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
            feedback.source = view.token; feedback.source_cycle = view.source_cycle;
            feedback.executed_step = request.target_step; feedback.feedback_seq = 1; feedback.flags = 0x17;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_INVALID_STATE);
        } else {
            CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_INVALID_STATE);
        }
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(f.device->sync_execution->invalid);
            CHECK(std::string(f.device->sync_execution->invalid_reason.data()).find("clock mapping changed") != std::string::npos);
            CHECK(!f.device->drive_402->status().output_allowed);
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    }
}

void worker_external_scheduled_command_hold() {
    struct FeedbackMappingClock final : xco::SyncClock {
        std::shared_ptr<TestSyncClock> clock;
        std::atomic<unsigned> changed{0};
        explicit FeedbackMappingClock(std::shared_ptr<TestSyncClock> source) : clock(std::move(source)) {}
        bool map_rx_us(uint64_t raw, uint64_t& mapped) const override { return clock->map_rx_us(raw, mapped); }
        bool now_us(uint64_t& now) const override { return clock->now_us(now); }
        bool map_model_us(const motor_drive_time_v1& time, uint64_t& mapped) const override {
            if (!clock->map_model_us(time, mapped)) return false;
            if (mapped == 5000) {
                if (changed == 2) return false;
                if (changed == 1) ++mapped;
            }
            return true;
        }
    };
    for (unsigned scenario = 0; scenario < 8; ++scenario) {
        Fixture f;
        auto j = external_drive_402_model();
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        auto& execution = *f.device->sync_execution;
        const auto mapping = std::make_shared<FeedbackMappingClock>(clock);
        execution.clock = mapping;
        execution.test_external_capture = true;
        execution.test_command_plan = xco::TestCommandPlan{500, 0, scenario == 2 ? 3500ULL : 10000ULL, 123, 42};
        execution.test_hold_plan = xco::TestHoldPlan{1, 1000, 2, scenario == 1 ? 1ULL : 3ULL};
        execution.test_sampling_plan = xco::TestSamplingPlan{1, 1, 2, 10, 4, 3000, 1000, 1500};
        execution.test_sampling_watermark = scenario == 0 || scenario >= 4;
        f.start();
        operational(f);
        motor_drive_endpoint_handle endpoint = nullptr;
        motor_drive_port_handle port = nullptr;
        CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
        CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
        std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = 2000;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 2000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_observation_sequence == 2 || execution.invalid;
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(!execution.invalid);
            CHECK(execution.commands.close_through(f.device->drive_epoch, 2500, 2000) == xco::TimedHistoryResult::ok);
            if (execution.active_test_sampling_watermark) {
                xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
                CHECK(execution.feedback.select(f.device->drive_epoch, 3000, 0, 2000, 4000, 3000, selected)
                    == xco::TimedHistoryResult::not_closed);
            }
            if (scenario == 0 || scenario >= 6) {
                execution.test_sampling_plan->step_stride = 99;
                execution.test_sampling_watermark = false;
            }
        }
        motor_drive_read_request_v1 request{};
        request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_SCHEDULED;
        request.source_cutoff = {42, MOTOR_DRIVE_TIME_VALID, 0.0015};
        request.target_step = {1, 10, 1, 0.001, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.003}};
        motor_drive_view_v1 first{}, next{};
        CHECK(motor_drive_read_v1(port, &request, &first, sizeof(first)) == MOTOR_DRIVE_OK);
        CHECK(first.source_cycle.sync_cycle_id == 1);
        if (scenario == 0 || scenario >= 6) {
            request.source_cutoff.time_s = 0.0025;
            request.target_step = {1, 17, 5, 0.005, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.007}};
            CHECK(motor_drive_read_v1(port, &request, &next, sizeof(next)) == MOTOR_DRIVE_INVALID_ARGUMENT);
            request.target_step.pcl_tick = 18;
            CHECK(motor_drive_read_v1(port, &request, &next, sizeof(next)) == MOTOR_DRIVE_OK);
            CHECK(next.source_cycle.sync_cycle_id == 2);
        }
        motor_drive_feedback_v1 feedback{};
        feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
        feedback.source = first.token; feedback.source_cycle = first.source_cycle;
        feedback.executed_step = first.target_step; feedback.feedback_seq = 1; feedback.flags = 0x17;
        feedback.actual_position_rad = 10 * 6.2831853071795864769 / 10000;
        clock->now = 3000;
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
            xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
            CHECK(execution.feedback.select(f.device->drive_epoch, 3000, 0, 3000, 4000, 3000, selected)
                == xco::TimedHistoryResult::not_closed);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->drive_processed_feedback_seq == 1 || execution.invalid;
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(!execution.invalid && f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
            CHECK(execution.test_sampling_cursor->next_step == 3);
            if (execution.active_test_sampling_watermark) {
                xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
                CHECK(execution.feedback.select(f.device->drive_epoch, 3000, 0, 3000, 4000, 3000, selected)
                    == xco::TimedHistoryResult::ok);
                CHECK(selected.value.feedback.feedback_seq == 1);
                CHECK(execution.feedback.select(f.device->drive_epoch, 3001, 0, 3000, 4000, 3000, selected)
                    == xco::TimedHistoryResult::not_closed);
            }
        }
        if (scenario == 5) {
            clock->model_offset_us = 1;
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return execution.invalid;
            });
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(std::string(execution.invalid_reason.data()).find("watermark clock mapping changed")
                    != std::string::npos);
                CHECK(!f.device->drive_402->status().output_allowed);
            }
            CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
            continue;
        }
        if (scenario == 4) {
            motor_drive_read_request_v1 asynchronous{};
            asynchronous.abi_version = 1; asynchronous.struct_size = sizeof(asynchronous);
            asynchronous.flags = MOTOR_DRIVE_READ_ASYNC;
            motor_drive_view_v1 current{};
            CHECK(motor_drive_read_v1(port, &asynchronous, &current, sizeof(current)) == MOTOR_DRIVE_OK);
            motor_drive_feedback_v1 safety{};
            safety.abi_version = 1; safety.struct_size = sizeof(safety); safety.source = current.token;
            safety.executed_step = {1, 12, 2, 0.002, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.004}};
            safety.feedback_seq = 2; safety.flags = 0x17;
            safety.fault_class = MOTOR_DRIVE_FAULT_OVERCURRENT; safety.fault_detail = 7;
            clock->now = 4000;
            CHECK(motor_drive_submit_v1(port, &safety, sizeof(safety)) == MOTOR_DRIVE_OK);
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return f.device->drive_current_fault == 0x2310;
            });
            feedback.executed_step = {1, 14, 3, 0.003, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.005}};
            feedback.feedback_seq = 3;
            clock->now = 5000;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_STALE);
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(f.device->drive_402->status().model_fault);
                CHECK(!f.device->drive_402->status().output_allowed);
                CHECK(execution.test_sampling_cursor->next_step == 3);
                xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
                CHECK(execution.feedback.select(f.device->drive_epoch, 5000, 0, 5000, 6000, 5000, selected)
                    == xco::TimedHistoryResult::not_closed);
            }
            CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
            continue;
        }
        feedback.executed_step = {1, 14, 3, 0.003, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.005}};
        feedback.feedback_seq = 2;
        feedback.actual_position_rad = 20 * 6.2831853071795864769 / 10000;
        auto future = feedback;
        future.executed_step = {1, 12, 2, 0.002, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.004}};
        CHECK(motor_drive_submit_v1(port, &future, sizeof(future)) == MOTOR_DRIVE_STALE);
        if (scenario == 0 || scenario >= 6) {
            clock->now = 4000;
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return execution.test_sampling_closed_us && *execution.test_sampling_closed_us == 4000;
            });
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
            CHECK(execution.feedback.select(f.device->drive_epoch, 4000, 0, 4000, 5000, 4000, selected)
                == xco::TimedHistoryResult::ok);
            CHECK(selected.value.feedback.feedback_seq == 1);
            CHECK(execution.feedback.select(f.device->drive_epoch, 5000, 0, 4000, 6000, 5000, selected)
                == xco::TimedHistoryResult::not_closed);
        }
        clock->now = 5000;
        auto forged = feedback;
        ++forged.executed_step.pcl_tick;
        CHECK(motor_drive_submit_v1(port, &forged, sizeof(forged)) == MOTOR_DRIVE_INVALID_FEEDBACK);
        if (scenario == 1) {
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_INVALID_FEEDBACK);
        } else if (scenario == 2) {
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_STALE);
        } else if (scenario == 3) {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            feedback.fault_class = MOTOR_DRIVE_FAULT_OVERCURRENT; feedback.fault_detail = 7;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
            clock->model_offset_us = 1;
        } else {
            forged = feedback;
            forged.source = next.token; forged.source_cycle = next.source_cycle;
            CHECK(motor_drive_submit_v1(port, &forged, sizeof(forged)) == MOTOR_DRIVE_INVALID_FEEDBACK);
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
                xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
                CHECK(execution.feedback.select(f.device->drive_epoch, 5000, 0, 5000, 6000, 5000, selected)
                    == xco::TimedHistoryResult::not_closed);
            }
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return f.device->drive_processed_feedback_seq == 2 || execution.invalid;
            });
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(!execution.invalid && f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
                CHECK(execution.test_sampling_cursor->next_step == 5 && port->test_issue_count == 2);
                execution.active_test_feedback_plan = xco::TestFeedbackPlan{0, 0, 3000};
                clock->now = 6000;
                fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 6000);
            }
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return f.device->sync_observation_sequence == 3 || execution.invalid;
            });
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(!execution.invalid);
                int32_t actual = 0;
                std::memcpy(&actual, f.device->model->value(0x6064, 0).data(), sizeof(actual));
                CHECK(actual == 20);
            }
            {
                const auto events = timing_events(f.device);
                bool found = false;
                for (const auto& event : events) {
                    if (event.kind != XNET_CO_TIMING_FEEDBACK_SELECTED || event.feedback_seq != 2) continue;
                    CHECK(event.target_step.model_step_seq == 1 && event.executed_step.model_step_seq == 3);
                    CHECK(event.source_cycle.sync_cycle_id == 1 && event.boundary.sequence == 3);
                    found = true;
                }
                CHECK(found);
            }
            feedback.executed_step = next.target_step; feedback.feedback_seq = 3;
            feedback.actual_position_rad = 30 * 6.2831853071795864769 / 10000;
            clock->now = 7000;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_INVALID_FEEDBACK);
            feedback.source = next.token; feedback.source_cycle = next.source_cycle;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return f.device->drive_processed_feedback_seq == 3 || execution.invalid;
            });
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(!execution.invalid && f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
                CHECK(execution.test_sampling_cursor->next_step == 7 && port->test_issue_count == 1);
                xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
                CHECK(execution.feedback.select(f.device->drive_epoch, 5000, 0, 7000, 8000, 5000, selected)
                    == xco::TimedHistoryResult::ok);
                CHECK(selected.value.feedback.feedback_seq == 2
                    && selected.value.feedback.source_cycle.sync_cycle_id == 1
                    && selected.value.feedback.executed_step.model_step_seq == 3);
                CHECK(execution.feedback.select(f.device->drive_epoch, 7001, 0, 7000, 8000, 7000, selected)
                    == xco::TimedHistoryResult::not_closed);
                execution.active_test_feedback_plan = xco::TestFeedbackPlan{2000, 0, 4000};
                if (scenario >= 6) mapping->changed = scenario == 6 ? 1 : 2;
                clock->now = 8000;
                fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 8000);
            }
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return f.device->sync_observation_sequence == 4 || execution.invalid;
            });
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                int32_t actual = 0;
                std::memcpy(&actual, f.device->model->value(0x6064, 0).data(), sizeof(actual));
                CHECK(actual == 20);
                if (scenario >= 6) {
                    CHECK(execution.invalid);
                    CHECK(std::string(execution.invalid_reason.data()).find("feedback source clock mapping changed")
                        != std::string::npos);
                    CHECK(!f.device->drive_402->status().output_allowed);
                } else CHECK(!execution.invalid);
            }
            if (scenario >= 6) {
                CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
                continue;
            }
            const auto old_epoch = f.device->drive_epoch;
            CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
            owned.reset();
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                execution.test_sampling_plan->step_stride = 2;
                execution.test_sampling_plan->first_sample_us = 30000;
                execution.test_sampling_watermark = true;
            }
            f.start();
            operational(f);
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
                CHECK(f.device->drive_epoch > old_epoch);
                CHECK(execution.feedback.select(old_epoch, 7000, 0, 8000, 9000, 7000, selected)
                    == xco::TimedHistoryResult::stale_epoch);
                CHECK(execution.feedback.select(f.device->drive_epoch, 7000, 0, 8000, 9000, 7000, selected)
                    == xco::TimedHistoryResult::not_closed);
                CHECK(execution.test_sampling_cursor->next_sample_us == 30000);
            }
        }
        if (scenario == 3) {
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return execution.invalid;
            });
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(std::string(execution.invalid_reason.data()).find("mapping changed") != std::string::npos);
            CHECK(f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_INVALID);
            CHECK(f.device->drive_402->status().model_fault);
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    }
}

void worker_external_fixed_feedback_schedule() {
    for (unsigned scenario = 0; scenario < 5; ++scenario) {
        Fixture f;
        auto j = external_drive_402_model();
        j["drive_402"]["action_timeout_us"] = 10000;
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        f.device->sync_execution->test_external_capture = true;
        f.device->sync_execution->test_command_plan = xco::TestCommandPlan{500, 0, 10000, 123, 42};
        f.device->sync_execution->test_sampling_plan = xco::TestSamplingPlan{1, 1, 2, 1, 2, 3000, 1000, 1500};
        f.device->sync_execution->test_sampling_watermark = true;
        f.start();
        operational(f);
        motor_drive_endpoint_handle endpoint = nullptr;
        motor_drive_port_handle port = nullptr;
        CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
        CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
        std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = 2000;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_observation_sequence == 1;
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(f.device->sync_execution->commands.close_through(f.device->drive_epoch, 1500, 2000)
                == xco::TimedHistoryResult::ok);
        }
        motor_drive_read_request_v1 request{};
        request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_SCHEDULED;
        request.source_cutoff = {42, MOTOR_DRIVE_TIME_VALID, 0.0015};
        request.target_step = {1, 1, 1, 0.001, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.003}};
        motor_drive_view_v1 view{};
        CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
        motor_drive_feedback_v1 feedback{};
        feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
        feedback.source = view.token; feedback.source_cycle = view.source_cycle;
        feedback.executed_step = request.target_step; feedback.feedback_seq = 1; feedback.flags = 0x17;
        if (scenario == 1) {
            motor_drive_read_request_v1 asynchronous{};
            asynchronous.abi_version = 1; asynchronous.struct_size = sizeof(asynchronous);
            asynchronous.flags = MOTOR_DRIVE_READ_ASYNC;
            motor_drive_view_v1 safety{};
            CHECK(motor_drive_read_v1(port, &asynchronous, &safety, sizeof(safety)) == MOTOR_DRIVE_OK);
            motor_drive_feedback_v1 safety_feedback{};
            safety_feedback.abi_version = 1; safety_feedback.struct_size = sizeof(safety_feedback);
            safety_feedback.source = safety.token;
            safety_feedback.executed_step = {1, 1, 1, 0.001, 0.001, {}};
            safety_feedback.feedback_seq = 1; safety_feedback.flags = 0x17;
            clock->now = 4000;
            CHECK(motor_drive_submit_v1(port, &safety_feedback, sizeof(safety_feedback)) == MOTOR_DRIVE_OK);
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return f.device->drive_processed_feedback_seq == 1;
            });
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(f.device->drive_has_last_feedback);
                CHECK(f.device->sync_execution->test_sampling_cursor->next_step == 1);
                xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
                CHECK(f.device->sync_execution->feedback.select(f.device->drive_epoch, 3000, 0, 4000, 4500, 4000, selected)
                    == xco::TimedHistoryResult::not_closed);
            }
            clock->now = 4500;
        } else if (scenario >= 2) {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = 3000;
            feedback.fault_class = MOTOR_DRIVE_FAULT_OVERCURRENT; feedback.fault_detail = 7;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
            if (scenario == 2) clock->now = 4500;
            else if (scenario == 3) clock->valid = false;
            else f.device->invalidate_sync_execution("injected timing failure");
        } else {
            clock->now = 3000;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return f.device->drive_processed_feedback_seq == 1;
            });
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
                CHECK(f.device->sync_execution->test_sampling_cursor->next_step == 3);
                CHECK(f.device->sync_execution->test_sampling_cursor->next_tick == 3);
                CHECK(f.device->sync_execution->test_sampling_cursor->next_sample_us == 5000);
            }
            motor_drive_read_request_v1 asynchronous{};
            asynchronous.abi_version = 1; asynchronous.struct_size = sizeof(asynchronous);
            asynchronous.flags = MOTOR_DRIVE_READ_ASYNC;
            motor_drive_view_v1 safety{};
            CHECK(motor_drive_read_v1(port, &asynchronous, &safety, sizeof(safety)) == MOTOR_DRIVE_OK);
            motor_drive_feedback_v1 safety_feedback{};
            safety_feedback.abi_version = 1; safety_feedback.struct_size = sizeof(safety_feedback);
            safety_feedback.source = safety.token;
            safety_feedback.executed_step = {1, 2, 2, 0.002, 0.001, {}};
            safety_feedback.feedback_seq = 2; safety_feedback.flags = 0x17;
            clock->now = 4000;
            CHECK(motor_drive_submit_v1(port, &safety_feedback, sizeof(safety_feedback)) == MOTOR_DRIVE_OK);
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return f.device->drive_processed_feedback_seq == 2;
            });
            request.target_step = {1, 3, 3, 0.003, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.005}};
            CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
            feedback.source = view.token; feedback.source_cycle = view.source_cycle;
            feedback.executed_step = request.target_step; feedback.feedback_seq = 3;
            clock->now = 5000;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return f.device->drive_processed_feedback_seq == 3;
            });
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                auto& execution = *f.device->sync_execution;
                CHECK(!execution.invalid && execution.test_sampling_cursor->next_step == 5);
                xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
                CHECK(execution.feedback.select(f.device->drive_epoch, 4500, 0, 5000, 6000, 5000, selected)
                    == xco::TimedHistoryResult::ok);
                CHECK(selected.value.feedback.feedback_seq == 1);
            }
            request.target_step = {1, 4, 4, 0.004, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.006}};
            CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
            feedback.source = view.token; feedback.source_cycle = view.source_cycle;
            feedback.executed_step = request.target_step; feedback.feedback_seq = 4;
            feedback.fault_class = MOTOR_DRIVE_FAULT_OVERCURRENT; feedback.fault_detail = 7;
            clock->now = 6000;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_execution->invalid;
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            const auto& execution = *f.device->sync_execution;
            const char* reason = scenario == 0 ? "missed fixed sampling step"
                : scenario == 3 ? "acceptance clock invalid"
                : scenario == 4 ? "injected timing failure" : "acceptance deadline missed";
            CHECK(std::string(execution.invalid_reason.data()).find(reason) != std::string::npos);
            CHECK(!f.device->drive_402->status().output_allowed);
            if (scenario != 1) {
                CHECK(f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_INVALID);
                CHECK(f.device->drive_current_fault == 0x2310);
                CHECK(f.device->drive_402->status().model_fault);
            }
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    }
}

void worker_external_fixed_feedback_plan_validation() {
    for (const auto& plan : {
             xco::TestSamplingPlan{1, 1, 0, 1, 1, 3000, 1000, 1500},
             xco::TestSamplingPlan{1, 1, 2, 1, 2, 3000, UINT64_MAX, 1500},
             xco::TestSamplingPlan{1, 1, 2, 1, 2, 3000, 1000, 2001},
             xco::TestSamplingPlan{1, 1, 2, 1, 2, UINT64_MAX - 100, 1000, 1500}}) {
        Fixture f;
        f.model(external_drive_402_model());
        CHECK(f.initialize() == XNET_CO_OK);
        install_test_sync_clock(f);
        f.device->sync_execution->test_external_capture = true;
        f.device->sync_execution->test_command_plan = xco::TestCommandPlan{500, 0, 10000, 123, 42};
        f.device->sync_execution->test_sampling_plan = plan;
        CHECK(xnet_canopen_start(f.device) == XNET_CO_INVALID_ARGUMENT);
        CHECK(diagnostic(f.device).find("sampling") != std::string::npos);
    }
    Fixture f;
    f.model(external_drive_402_model());
    CHECK(f.initialize() == XNET_CO_OK);
    install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.device->sync_execution->test_sampling_watermark = true;
    CHECK(xnet_canopen_start(f.device) == XNET_CO_INVALID_STATE);
    CHECK(diagnostic(f.device).find("watermark") != std::string::npos);
}

void worker_external_hold_plan_validation() {
    for (unsigned scenario = 0; scenario < 4; ++scenario) {
        Fixture f;
        f.model(external_drive_402_model());
        CHECK(f.initialize() == XNET_CO_OK);
        install_test_sync_clock(f);
        auto& execution = *f.device->sync_execution;
        execution.test_external_capture = true;
        if (scenario != 0) execution.test_command_plan = xco::TestCommandPlan{500, 0, 10000, 123, 42};
        execution.test_hold_plan = xco::TestHoldPlan{1, 1000, 2, 3};
        if (scenario == 1) execution.test_hold_plan->tick_stride = 0;
        if (scenario == 2) execution.test_hold_plan->max_hold_steps = 0;
        if (scenario == 3)
            execution.test_sampling_plan = xco::TestSamplingPlan{1, 1, 2, 1, 2, 3000, 1000, 1500};
        CHECK(xnet_canopen_start(f.device) == (scenario == 0 ? XNET_CO_INVALID_STATE : XNET_CO_INVALID_ARGUMENT));
        CHECK(diagnostic(f.device).find("hold") != std::string::npos);
    }
}

void worker_external_budget_plan_freeze() {
    for (unsigned scenario = 0; scenario < 3; ++scenario) {
        Fixture f;
        f.model(external_drive_402_model());
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        auto& execution = *f.device->sync_execution;
        execution.test_external_capture = true;
        execution.test_command_plan = xco::TestCommandPlan{500, 100, 1000, 123, 42};
        execution.test_feedback_plan = xco::TestFeedbackPlan{200, 100, 1000};
        if (scenario == 1) execution.test_command_plan->lead_us = 901;
        if (scenario == 2) execution.test_feedback_plan->epsilon_us = 1001;
        if (scenario != 0) {
            CHECK(xnet_canopen_start(f.device) == XNET_CO_INVALID_ARGUMENT);
            continue;
        }
        execution.test_auto_retire = true;
        f.start();
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            execution.test_command_plan->lead_us = 700;
            execution.test_feedback_plan->delay_us = 400;
            execution.test_auto_retire = false;
            execution.clock.reset();
            execution.max_interval_us = 1;
            execution.tpdo_window_us = 1;
            execution.test_external_capture = false;
            CHECK(execution.active_test_command_plan->lead_us == 500);
            CHECK(execution.active_test_feedback_plan->delay_us == 200);
            CHECK(execution.active_test_auto_retire);
            CHECK(execution.active_clock == clock);
            CHECK(execution.active_max_interval_us == 10000 && execution.active_tpdo_window_us == 5000);
            CHECK(execution.active_test_external_capture);
            execution.clock = clock;
            execution.max_interval_us = 10000;
            execution.tpdo_window_us = 5000;
            execution.test_external_capture = true;
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
        f.start();
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(execution.active_test_command_plan->lead_us == 700);
            CHECK(execution.active_test_feedback_plan->delay_us == 400);
            CHECK(!execution.active_test_auto_retire);
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    }
}

void worker_external_history_budget() {
    for (unsigned scenario = 0; scenario < 6; ++scenario) {
        Fixture f;
        auto j = external_drive_402_model();
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        auto& execution = *f.device->sync_execution;
        execution.test_external_capture = true;
        execution.test_command_plan = xco::TestCommandPlan{500, 0, 1000, 123, 42};
        execution.test_command_history_budget = {2, 1000, 1000};
        if (scenario == 1) execution.test_command_history_budget.capacity = 1;
        if (scenario == 2) execution.test_command_history_budget.retention_us = 999;
        if (scenario == 3) execution.test_command_history_budget.capacity = 65;
        if (scenario == 4) execution.test_feedback_history_budget = {2, 2000, 1000};
        if (scenario == 5) execution.test_feedback_history_budget = {2, 1000, 0};
        if (scenario != 0) {
            CHECK(xnet_canopen_start(f.device) == XNET_CO_INVALID_ARGUMENT);
            continue;
        }
        f.start();
        operational(f);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            execution.test_command_history_budget.capacity = 4;
            clock->now = 3000;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 2000);
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 3000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return execution.invalid;
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(std::string(execution.invalid_reason.data()).find("history overflow") != std::string::npos);
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
        f.start();
        operational(f);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = 6000;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 4000);
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 5000);
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 6000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return execution.invalid || (execution.last_sync_us && *execution.last_sync_us == 6000);
        });
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(!execution.invalid);
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    }
}

void worker_external_timing_provider_contract() {
    struct Provider final : xco::TestTimingProvider {
        std::atomic<bool> available{true}, command_closed{false}, feedback_closed{false};
        std::atomic<uint64_t> generation{1}, error{1};
        std::atomic<uint64_t> command_through{0}, command_observed{0};
        std::atomic<uint64_t> feedback_through{0}, feedback_observed{0};
        bool snapshot(xco::TestTimingEvidence& result) const override {
            if (!available) return false;
            result.generation = generation;
            result.max_error_us = error;
            if (command_closed) result.commands = xco::TestWatermarkProof{command_through, command_observed};
            if (feedback_closed) result.feedback = xco::TestWatermarkProof{feedback_through, feedback_observed};
            return true;
        }
    };
    for (unsigned scenario = 0; scenario < 15; ++scenario) {
        Fixture f;
        auto j = external_drive_402_model();
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        const auto provider = std::make_shared<Provider>();
        auto& execution = *f.device->sync_execution;
        execution.test_external_capture = true;
        execution.test_command_plan = xco::TestCommandPlan{500, 1, 10000, 123, 42};
        execution.test_timing_provider = provider;
        execution.test_max_clock_error_us = 1;
        if (scenario == 11) provider->generation = 0;
        if (scenario == 12) provider->error = 2;
        if (scenario == 13) execution.test_command_plan->epsilon_us = 0;
        if (scenario == 14) execution.test_ordered_sync_watermark = true;
        if (scenario >= 11) {
            CHECK(xnet_canopen_start(f.device) == XNET_CO_INVALID_ARGUMENT);
            continue;
        }
        f.start();
        operational(f);
        motor_drive_endpoint_handle endpoint = nullptr;
        motor_drive_port_handle port = nullptr;
        CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
        CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
        std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            clock->now = 2000;
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
        }
        wait_for([&] {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            return f.device->sync_observation_sequence == 1 || execution.invalid;
        });
        motor_drive_read_request_v1 request{};
        request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_SCHEDULED;
        request.source_cutoff = {42, MOTOR_DRIVE_TIME_VALID, 0.0015};
        request.target_step = {1, 1, 1, 0.001, 0.001, {42, MOTOR_DRIVE_TIME_VALID, 0.005}};
        motor_drive_view_v1 view{};
        CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_NO_DATA);
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            provider->command_through = 1500;
            provider->command_observed = scenario == 4 ? 3000 : scenario == 9 ? 1400 : 2000;
            provider->command_closed = true;
            CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view))
                == (scenario == 4 || scenario == 9 ? MOTOR_DRIVE_INVALID_STATE : MOTOR_DRIVE_OK));
            if (scenario == 0) provider->generation = 2;
            else if (scenario == 1) provider->command_through = 1400;
            else if (scenario == 2) provider->error = 2;
            else if (scenario == 3) provider->available = false;
            else if (scenario == 5) fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1500);
            else if (scenario == 8) provider->command_closed = false;
            else if (scenario == 10) provider->command_observed = 1900;
        }
        if (scenario == 1 || scenario == 8 || scenario == 10)
            CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_INVALID_STATE);
        if (scenario == 6 || scenario == 7) {
            motor_drive_feedback_v1 feedback{};
            feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
            feedback.source = view.token; feedback.source_cycle = view.source_cycle;
            feedback.executed_step = request.target_step; feedback.feedback_seq = 1; feedback.flags = 0x17;
            feedback.actual_position_rad = 17 * 6.2831853071795864769 / 10000;
            clock->now = 5000;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return f.device->drive_processed_feedback_seq == 1 || execution.invalid;
            });
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(!execution.invalid && f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_ACCEPTED);
                execution.active_test_feedback_plan = xco::TestFeedbackPlan{999, 1, 3000};
                if (scenario == 6) {
                    provider->feedback_through = 5001;
                    provider->feedback_observed = 6000;
                    provider->feedback_closed = true;
                }
                clock->now = 6000;
                fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 6000);
            }
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return execution.invalid || f.device->sync_observation_sequence == 2;
            });
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            int32_t actual = 0;
            std::memcpy(&actual, f.device->model->value(0x6064, 0).data(), sizeof(actual));
            if (scenario == 6) CHECK(!execution.invalid && actual == 17);
            else CHECK(execution.invalid && actual == 0);
        } else {
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return execution.invalid;
            });
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            CHECK(!f.device->drive_402->status().output_allowed);
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
        if (scenario == 0) {
            owned.reset();
            provider->command_closed = false;
            f.start();
            {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(!execution.invalid && execution.test_timing_generation == 2);
                CHECK(!execution.test_command_proof);
                xco::TimedHistory<xco::TimedDriveCommand, 64>::Record selected{};
                CHECK(execution.commands.select(f.device->drive_epoch, 1500, 1, 2000, 5000, 10000, selected)
                    == xco::TimedHistoryResult::not_closed);
            }
            CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
        }
    }
}

void worker_external_feedback_watermark_required() {
    Fixture f;
    auto j = external_drive_402_model();
    j["drive_402"]["action_timeout_us"] = 10000;
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.device->sync_execution->test_feedback_plan = xco::TestFeedbackPlan{0, 0, 1000};
    f.start();
    operational(f);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 1000;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_execution->invalid;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(std::string(f.device->sync_execution->invalid_reason.data()).find("watermark not closed") != std::string::npos);
        CHECK(!f.device->drive_402->status().output_allowed);
        CHECK(!f.device->drive_402->status().feedback_fresh);
    }
    const auto events = timing_events(f.device);
    CHECK(events.back().reason == XNET_CO_TIMING_REASON_WATERMARK
        && events.back().boundary.sequence == 1);
}

void worker_external_timed_safety_without_sync() {
    Fixture f;
    auto j = external_drive_402_model();
    j["drive_402"]["action_timeout_us"] = 10000;
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.start();
    operational(f);
    motor_drive_endpoint_handle endpoint = nullptr;
    motor_drive_port_handle port = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    clock->now = 1000;
    motor_drive_read_request_v1 request{};
    request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_ASYNC;
    motor_drive_view_v1 view{};
    CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
    motor_drive_feedback_v1 feedback{};
    feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
    feedback.source = view.token;
    feedback.executed_step = {1, 1, 1, 0.002, 0.002, {42, MOTOR_DRIVE_TIME_VALID, 0.0005}};
    feedback.feedback_seq = 1; feedback.flags = 0x17;
    feedback.fault_class = MOTOR_DRIVE_FAULT_OVERCURRENT; feedback.fault_detail = 7;
    CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->drive_current_fault == 0x2310;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CHECK(f.device->sync_observation_sequence == 0);
        CHECK(!f.device->sync_execution->invalid);
        CHECK(f.device->drive_402->status().model_fault);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
}

void worker_external_timed_history_overflow() {
    Fixture f;
    auto j = external_drive_402_model();
    j["drive_402"]["action_timeout_us"] = 10000;
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.device->sync_execution->test_auto_retire = true;
    f.start();
    operational(f);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        clock->now = 65;
        for (uint64_t time = 1; time <= 65; ++time)
            fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, time);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_execution->invalid;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        const auto& execution = *f.device->sync_execution;
        CHECK(std::string(execution.invalid_reason.data()).find("command history overflow") != std::string::npos);
        xco::TimedHistory<xco::TimedDriveCommand, 64>::Record selected{};
        CHECK(execution.commands.select(f.device->drive_epoch, 65, 0, 65, 100, 100, selected)
            == xco::TimedHistoryResult::invalidated);
        CHECK(f.device->drive_adapter_fault);
        CHECK(!f.device->drive_402->status().output_allowed);
    }
}

void worker_external_timed_history_epoch() {
    Fixture f;
    auto j = external_drive_402_model();
    j["tpdos"] = xco::Json::array();
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    const auto clock = install_test_sync_clock(f);
    f.device->sync_execution->test_external_capture = true;
    f.device->sync_execution->test_ordered_sync_watermark = true;
    f.device->sync_execution->test_command_plan = xco::TestCommandPlan{500, 0, 10000, 123, 42};
    f.start();
    operational(f);
    uint64_t old_epoch = 0;
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        old_epoch = f.device->drive_epoch;
        clock->now = 1000;
        fake::inject(0x80, {}, nxFrameType_CAN_Data, 0, 1000);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_observation_sequence == 1;
    });
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        xco::TimedHistory<xco::TimedDriveCommand, 64>::Record command{};
        CHECK(f.device->sync_execution->commands.select(old_epoch, 1000, 0, 1000, 2000, 1000, command)
            == xco::TimedHistoryResult::ok);
    }
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    f.start();
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        const auto& execution = *f.device->sync_execution;
        const auto epoch = f.device->drive_epoch;
        CHECK(epoch > old_epoch);
        CHECK(!execution.test_sync_watermark_anchor);
        xco::TimedHistory<xco::TimedDriveCommand, 64>::Record command{};
        xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record feedback{};
        CHECK(execution.commands.select(old_epoch, 1000, 0, 1000, 2000, 1000, command)
            == xco::TimedHistoryResult::stale_epoch);
        CHECK(execution.commands.select(epoch, 1000, 0, 1000, 2000, 1000, command)
            == xco::TimedHistoryResult::not_closed);
        CHECK(execution.feedback.select(epoch, 1000, 0, 1000, 2000, 1000, feedback)
            == xco::TimedHistoryResult::not_closed);
    }
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
}

void worker_external_timed_feedback_faults() {
    for (bool overflow : {false, true}) {
        Fixture f;
        auto j = external_drive_402_model();
        j["drive_402"]["action_timeout_us"] = 10000;
        j["tpdos"] = xco::Json::array();
        f.model(j);
        CHECK(f.initialize() == XNET_CO_OK);
        const auto clock = install_test_sync_clock(f);
        f.device->sync_execution->test_external_capture = true;
        f.start();
        operational(f);
        motor_drive_endpoint_handle endpoint = nullptr;
        motor_drive_port_handle port = nullptr;
        CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
        CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
        std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
        clock->now = 1000;
        motor_drive_read_request_v1 request{};
        request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_ASYNC;
        const uint32_t count = overflow ? 65 : 2;
        for (uint32_t seq = 1; seq <= count; ++seq) {
            motor_drive_view_v1 view{};
            CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
            motor_drive_feedback_v1 feedback{};
            feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
            feedback.source = view.token;
            feedback.executed_step = {1, seq, seq, seq * 0.002, 0.002,
                (overflow || seq > 1) ? motor_drive_time_v1{42, MOTOR_DRIVE_TIME_VALID, seq * 0.000001}
                                      : motor_drive_time_v1{}};
            feedback.feedback_seq = seq; feedback.flags = 0x17;
            CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
            wait_for([&] {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                return f.device->drive_processed_feedback_seq == seq;
            });
            if ((overflow && seq == 65) || (!overflow && seq == 1)) {
                std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
                CHECK(f.device->drive_processed_result == MOTOR_DRIVE_PROCESSED_INVALID);
            }
        }
        {
            std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
            const auto& execution = *f.device->sync_execution;
            CHECK(execution.invalid);
            CHECK(std::string(execution.invalid_reason.data()).find(overflow
                ? "feedback history overflow" : "feedback clock mapping invalid") != std::string::npos);
            xco::TimedHistory<xco::TimedDriveCommand, 64>::Record command{};
            xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
            CHECK(execution.commands.select(f.device->drive_epoch, 1000, 0, 1000, 2000, 1000, command)
                == xco::TimedHistoryResult::invalidated);
            CHECK(execution.feedback.select(f.device->drive_epoch, 1000, 0, 1000, 2000, 1000, selected)
                == xco::TimedHistoryResult::invalidated);
            CHECK(f.device->drive_adapter_fault);
            CHECK(!f.device->drive_402->status().output_allowed);
        }
        CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    }
}

void timed_history_auto_retirement() {
    using History = xco::TimedHistory<int, 3>;
    using Result = xco::TimedHistoryResult;
    History history;
    History::Record selected{};
    CHECK(history.start_epoch(1) == Result::ok);
    CHECK(history.retire_before(1, 0) == Result::ok);
    CHECK(history.retire_before(1, 1) == Result::invalid);
    CHECK(history.retire_before(2, 0) == Result::stale_epoch);
    for (uint64_t i = 1; i <= 70; ++i) {
        CHECK(history.append({1, i, i, i, static_cast<int>(i)}) == Result::ok);
        CHECK(history.close_through(1, i, i) == Result::ok);
        CHECK(history.select(1, i, 0, i, i + 1, 1, selected) == Result::ok);
        CHECK(selected.value == static_cast<int>(i));
        CHECK(history.retire_before(1, i > 1 ? i - 1 : 0) == Result::ok);
    }
    CHECK(history.select(1, 70, 0, 70, 71, 1, selected) == Result::ok);
    CHECK(history.select(1, 70, 1, 70, 71, 1, selected) == Result::ok);
    CHECK(selected.source_time == 69);
    CHECK(history.select(1, 70, 0, 70, 71, 2, selected) == Result::history_gap);
    CHECK(history.start_epoch(2) == Result::ok);
    CHECK(history.append({2, 1, 1, 1, 1}) == Result::ok);
    CHECK(history.close_through(2, 1, 1) == Result::ok);
    CHECK(history.select(2, 1, 0, 1, 2, 1, selected) == Result::ok);
}

void timed_history_bounded_arrivals() {
    using History = xco::TimedHistory<int, 4>;
    using Result = xco::TimedHistoryResult;
    History history;
    CHECK(history.start_epoch(1, 0) == Result::invalid);
    CHECK(history.start_epoch(1, 5) == Result::invalid);
    CHECK(history.start_epoch(1, 2, 10) == Result::ok);
    CHECK(history.append({1, 1, 10, 10, 1}) == Result::ok);
    CHECK(history.close_through(1, 10, 10) == Result::ok);
    CHECK(history.retire_through(1, 10) == Result::ok);
    CHECK(history.append({1, 2, 19, 19, 2}) == Result::invalidated);
    CHECK(history.start_epoch(2, 2, 10) == Result::ok);
    CHECK(history.append({2, 1, 10, 10, 1}) == Result::ok);
    CHECK(history.append({2, 2, 20, 20, 2}) == Result::ok);
    CHECK(history.append({2, 3, 30, 30, 3}) == Result::overflow);
    CHECK(history.start_epoch(3, 3, 10) == Result::ok);
    CHECK(history.append({3, 1, 10, 10, 1}) == Result::ok);
    CHECK(history.append({3, 2, 20, 20, 2}) == Result::ok);
    CHECK(history.append({3, 3, 30, 30, 3}) == Result::ok);
}

void timed_history_selection() {
    using History = xco::TimedHistory<motor_drive_view_v1, 4>;
    using Result = xco::TimedHistoryResult;
    History history;
    History::Record selected{};
    CHECK(history.start_epoch(4) == Result::ok);
    motor_drive_view_v1 command{};
    command.token = {4, 1, 7};
    command.demand_position_rad = 1.5;
    CHECK(history.append({4, 1, 90, 101, command}) == Result::ok);
    command.token.command_seq = 2;
    command.demand_position_rad = 2.5;
    CHECK(history.append({4, 2, 95, 102, command}) == Result::ok);
    command.token.command_seq = 3;
    command.demand_position_rad = 3.5;
    CHECK(history.append({4, 3, 95, 103, command}) == Result::ok);
    command.token.command_seq = 4;
    CHECK(history.append({4, 4, 96, 104, command}) == Result::ok);
    CHECK(history.select(4, 100, 5, 110, 120, 20, selected) == Result::not_closed);
    CHECK(history.close_through(4, 99, 105) == Result::ok);
    CHECK(history.select(4, 100, 5, 110, 120, 20, selected) == Result::not_closed);
    CHECK(history.close_through(4, 100, 105) == Result::ok);
    CHECK(history.close_through(4, 100, 104) == Result::invalid);
    CHECK(history.retire_through(4, 101) == Result::invalid);
    CHECK(history.select(4, 100, 5, 110, 120, 20, selected) == Result::ok);
    CHECK(selected.sequence == 3 && selected.value.token.action_seq == 7
        && selected.value.demand_position_rad == 3.5);
    CHECK(history.select(4, 100, 6, 110, 120, 20, selected) == Result::ok);
    CHECK(selected.sequence == 1 && selected.value.demand_position_rad == 1.5);
    CHECK(history.select(4, 100, 11, 110, 120, 20, selected) == Result::no_candidate);
    CHECK(history.select(4, 100, 5, 120, 120, 20, selected) == Result::deadline_missed);
    CHECK(history.select(4, 100, 5, 102, 120, 20, selected) == Result::invalid);
    CHECK(history.select(4, 100, 5, 110, 120, 4, selected) == Result::no_candidate);
    CHECK(history.select(3, 100, 5, 110, 120, 20, selected) == Result::stale_epoch);

    CHECK(history.retire_through(4, 90) == Result::ok);
    CHECK(history.select(4, 100, 5, 110, 120, 10, selected) == Result::history_gap);
    CHECK(history.select(4, 100, 5, 110, 120, 9, selected) == Result::ok);
    CHECK(selected.sequence == 3);
    CHECK(history.append({4, 5, 100, 111, command}) == Result::invalidated);
    CHECK(history.select(4, 100, 5, 110, 120, 9, selected) == Result::invalidated);
    CHECK(history.start_epoch(4) == Result::stale_epoch);
    CHECK(history.start_epoch(5) == Result::ok);
    CHECK(history.select(4, 100, 0, 110, 120, 20, selected) == Result::stale_epoch);
    CHECK(history.select(5, 100, 0, 110, 120, 20, selected) == Result::not_closed);
    command.token.epoch = 5;
    CHECK(history.append({5, 1, 150, 152, command}) == Result::ok);
    CHECK(history.append({5, 2, 140, 153, command}) == Result::ok);
    CHECK(history.close_through(5, 150, 154) == Result::ok);
    CHECK(history.select(5, 150, 0, 154, 155, 10, selected) == Result::ok);
    CHECK(selected.sequence == 1);
    CHECK(history.select(5, 150, 0, 153, 155, 10, selected) == Result::invalid);
    CHECK(history.retire_through(5, 140) == Result::ok);
    CHECK(history.select(5, 150, 0, 154, 155, 9, selected) == Result::ok);
    CHECK(history.append({5, 3, 151, 155, command}) == Result::ok);
    CHECK(history.append({5, 3, 152, 156, command}) == Result::invalidated);

    using FeedbackHistory = xco::TimedHistory<motor_drive_feedback_v1, 1>;
    FeedbackHistory feedback_history;
    FeedbackHistory::Record feedback_selected{};
    motor_drive_feedback_v1 feedback{};
    feedback.source = {8, 12, 4};
    feedback.feedback_seq = 21;
    feedback.executed_step.model_step_seq = 19;
    feedback.actual_position_rad = 0.75;
    CHECK(feedback_history.start_epoch(8) == Result::ok);
    CHECK(feedback_history.append({8, 21, 200, 220, feedback}) == Result::ok);
    CHECK(feedback_history.close_through(8, 200, 220) == Result::ok);
    CHECK(feedback_history.select(8, 200, 0, 220, 221, 100, feedback_selected) == Result::ok);
    CHECK(feedback_selected.value.source.command_seq == 12
        && feedback_selected.value.executed_step.model_step_seq == 19
        && feedback_selected.value.actual_position_rad == 0.75);
    CHECK(feedback_history.select(8, 200, 0, 220, 220, 100, feedback_selected)
        == Result::deadline_missed);
    CHECK(feedback_history.append({8, 22, 201, 221, feedback}) == Result::overflow);
    CHECK(feedback_history.select(8, 200, 0, 220, 221, 100, feedback_selected)
        == Result::invalidated);

    History clock_history;
    CHECK(clock_history.start_epoch(1) == Result::ok);
    CHECK(clock_history.append({1, 1, 10, 110, command}) == Result::ok);
    CHECK(clock_history.append({1, 2, 150, 120, command}) == Result::ok);
    CHECK(clock_history.close_through(1, 100, 119) == Result::invalid);
    CHECK(clock_history.close_through(1, 100, 130) == Result::ok);
    CHECK(clock_history.select(1, 100, 0, 125, 140, 100, selected) == Result::invalid);
    CHECK(clock_history.select(1, 100, 0, 130, 140, 100, selected) == Result::ok);
    CHECK(selected.sequence == 1);
    CHECK(clock_history.append({1, 3, 151, 129, command}) == Result::invalidated);
    CHECK(clock_history.select(1, 100, 0, 130, 140, 100, selected) == Result::invalidated);
}

void timed_feedback_selection_from_accepted_records() {
    Fixture f;
    f.model(external_drive_402_model());
    CHECK(f.initialize() == XNET_CO_OK);
    motor_drive_endpoint_handle endpoint = nullptr;
    motor_drive_port_handle port = nullptr;
    CHECK(xnet_canopen_get_motor_drive_endpoint(f.device, &endpoint) == XNET_CO_OK);
    CHECK(motor_drive_open_v1(endpoint, &port) == MOTOR_DRIVE_OK);
    std::unique_ptr<motor_drive_port, decltype(&motor_drive_close_v1)> owned(port, motor_drive_close_v1);
    motor_drive_read_request_v1 request{};
    request.abi_version = 1; request.struct_size = sizeof(request); request.flags = MOTOR_DRIVE_READ_ASYNC;
    motor_drive_view_v1 view{};
    using History = xco::TimedHistory<xnet_canopen_device::DriveFeedbackRecord, 4>;
    using Result = xco::TimedHistoryResult;
    History history;
    const auto epoch = f.device->drive_epoch;
    CHECK(history.start_epoch(epoch) == Result::ok);
    f.device->process_drive_402(0);
    for (uint32_t seq = 1; seq <= 2; ++seq) {
        CHECK(motor_drive_read_v1(port, &request, &view, sizeof(view)) == MOTOR_DRIVE_OK);
        motor_drive_feedback_v1 feedback{};
        feedback.abi_version = 1; feedback.struct_size = sizeof(feedback);
        feedback.source = view.token;
        feedback.executed_step = {1, seq, seq, seq * 0.002, 0.002, {}};
        feedback.feedback_seq = seq; feedback.flags = 0x17;
        feedback.actual_position_rad = seq * 10 * 6.2831853071795864769 / 10000;
        CHECK(motor_drive_submit_v1(port, &feedback, sizeof(feedback)) == MOTOR_DRIVE_OK);
        f.device->process_drive_402(0, false, true);
        const auto record = f.device->drive_feedback_history[seq];
        CHECK(record.valid && record.converted.actual_position == static_cast<int32_t>(seq * 10));
        CHECK(history.append({epoch, seq, seq * 10, seq * 10 + 5, record}) == Result::ok);
    }
    CHECK(f.device->drive_402->status().actual_position == 0);
    CHECK(history.close_through(epoch, 15, 26) == Result::ok);
    History::Record selected{};
    CHECK(history.select(epoch, 15, 0, 26, 30, 100, selected) == Result::ok);
    CHECK(selected.value.feedback.executed_step.model_step_seq == 1);
    CHECK(f.device->drive_402->select_feedback({selected.value.converted.actual_position,
        selected.value.converted.actual_velocity, false, false}));
    f.device->process_drive_402(0);
    int32_t actual = 0;
    std::memcpy(&actual, f.device->model->value(0x6064, 0).data(), sizeof(actual));
    CHECK(actual == 10);
    CHECK(history.select(epoch, 25, 0, 26, 30, 100, selected) == Result::not_closed);
    CHECK(f.device->drive_402->status().actual_position == 10);
    CHECK(history.close_through(epoch, 25, 27) == Result::ok);
    CHECK(history.select(epoch, 25, 0, 27, 30, 100, selected) == Result::ok);
    CHECK(selected.value.feedback.executed_step.model_step_seq == 2);
    CHECK(f.device->drive_402->select_feedback({selected.value.converted.actual_position,
        selected.value.converted.actual_velocity, false, false}));
    f.device->process_drive_402(0);
    std::memcpy(&actual, f.device->model->value(0x6064, 0).data(), sizeof(actual));
    CHECK(actual == 20);
    CHECK(history.select(epoch, 25, 0, 30, 30, 100, selected) == Result::deadline_missed);
}

void lss_assignment_and_resets() {
    Fixture f;
    auto j = xco::read_json(f.model_path); j["node_id"] = 255; f.model(j);
    CHECK(f.initialize() == 0);
    CHECK(xnet_canopen_start(f.device) == 0);
    wait_for([&] { return f.status().runtime_state == XNET_CO_RUNTIME_RUNNING; });
    auto answer = exchange(0x7e5, {0x51, 0, 0, 0, 0, 0x80, 0, 0}, 0x7e4, 0x4f);
    CHECK(answer.Payload[0] == 0x4f);
    fake::inject(0x7e5, {0x40, 1, 0, 0, 0, 0, 0, 0});
    fake::inject(0x7e5, {0x41, 1, 0, 0, 0, 0, 0, 0});
    fake::inject(0x7e5, {0x42, 1, 0, 0, 0, 0, 0, 0});
    answer = exchange(0x7e5, {0x43, 0xe9, 3, 0, 0, 0, 0, 0}, 0x7e4, 0x44);
    CHECK(answer.Payload[0] == 0x44);
    answer = exchange(0x7e5, {0x11, 42, 0, 0, 0, 0, 0, 0}, 0x7e4, 0x11);
    CHECK(answer.Payload[1] == 0);
    fake::inject(0x7e5, {4, 0, 0, 0, 0, 0, 0, 0});
    wait_for([&] { return f.status().active_node_id == 42; });
    answer = exchange(0x62a, {0x40, 0, 0x18, 1, 0, 0, 0, 0}, 0x5aa);
    CHECK(answer.Payload[4] == 0xaa && answer.Payload[5] == 1);
    auto batch = f.bind({{0x2000, 0, 2}});
    uint8_t data[2]{77, 0}; int32_t status;
    CHECK(xnet_canopen_write_batch(f.device, batch, data, 2, &status, 1) == 0);
    exchange(0, {0x82, 42}, 0x72a, 0);
    CHECK(xnet_canopen_read_batch(f.device, batch, data, 2, &status, 1) == 0 && data[0] == 77);
    fake::inject(0, {0x81, 42});
    wait_for([&] { return f.status().active_node_id == 255; });
    CHECK(xnet_canopen_read_batch(f.device, batch, data, 2, &status, 1) == 0 && data[0] == 0);
}

void lss_bitrate_and_storage() {
    Fixture f; CHECK(f.initialize() == 0); f.start();
    fake::inject(0x7e5, {4, 1, 0, 0, 0, 0, 0, 0});
    auto answer = exchange(0x7e5, {0x11, 19, 0, 0, 0, 0, 0, 0}, 0x7e4, 0x11);
    CHECK(answer.Payload[1] == 0);
    for (uint8_t command = 0x5a; command <= 0x5e; ++command) {
        answer = exchange(0x7e5, {command, 0, 0, 0, 0, 0, 0, 0}, 0x7e4, command);
        CHECK(answer.Payload[1] == (command == 0x5d ? 0xe9 : command == 0x5e ? 10 : 1));
    }
    answer = exchange(0x7e5, {0x13, 0, 9, 0, 0, 0, 0, 0}, 0x7e4, 0x13);
    CHECK(answer.Payload[1] != 0);
    answer = exchange(0x7e5, {0x13, 0, 2, 0, 0, 0, 0, 0}, 0x7e4, 0x13);
    CHECK(answer.Payload[1] == 0 && f.status().pending_bitrate_kbps == 500);
    const auto requested = xco::Clock::now();
    fake::inject(0x7e5, {0x15, 10, 0, 0, 0, 0, 0, 0});
    wait_for([&] { return f.status().active_bitrate_kbps == 500; });
    std::this_thread::sleep_for(25ms);
    auto events = fake::events();
    xco::Clock::time_point changed{};
    bool resumed = false;
    for (const auto& e : events) {
        if (e.operation == "bitrate" && e.value == 500000) {
            changed = e.when; CHECK(e.when - requested >= 10ms);
        }
        if (changed != xco::Clock::time_point{} && e.operation == "start") {
            CHECK(e.when - changed >= 10ms); resumed = true;
        }
        if (changed != xco::Clock::time_point{} && e.operation == "write") CHECK(e.when - changed >= 10ms);
    }
    CHECK(resumed);
    answer = exchange(0x7e5, {0x17, 0, 0, 0, 0, 0, 0, 0}, 0x7e4, 0x17);
    CHECK(answer.Payload[1] == 0);
    auto stored = xco::read_json(f.saved_path);
    CHECK(stored["node_id"] == 19 && stored["bitrate_kbps"] == 500);
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    CHECK(f.initialize() == 0);
    CHECK(f.status().active_node_id == 19 && f.status().active_bitrate_kbps == 500);
}

void storage_failures() {
    Fixture f; f.saved_path = f.directory / "absent" / "lss.json";
    CHECK(f.initialize() == 0); f.start();
    fake::inject(0x7e5, {4, 1, 0, 0, 0, 0, 0, 0});
    auto answer = exchange(0x7e5, {0x17, 0, 0, 0, 0, 0, 0, 0}, 0x7e4, 0x17);
    CHECK(answer.Payload[1] != 0);
    CHECK(diagnostic(f.device).find("persistence") != std::string::npos || diagnostic(f.device).find("create") != std::string::npos);
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    f.saved_path = f.directory / "lss.json";
    { std::ofstream file(f.saved_path); file << "{broken"; }
    auto check_fallback = [&] {
        CHECK(f.initialize() == 0 && fake::sessions() == 2);
        const auto status = f.status();
        CHECK(status.active_node_id == 10 && status.pending_node_id == 10);
        CHECK(status.active_bitrate_kbps == 125 && status.pending_bitrate_kbps == 125);
        CHECK(diagnostic(f.device).find(std::to_string(XNET_CO_IO_ERROR) + ": ") == 0);
        CHECK(xnet_canopen_uninitialize(f.device) == 0 && fake::sessions() == 0);
    };
    check_fallback();
    xco::Model model; model.load(xco::read_json(f.model_path));
    const auto valid = xco::Json{{"schema_version", 1}, {"identity", model.identity},
        {"node_id", 20}, {"bitrate_kbps", 500}};
    auto bad = valid; bad["identity"] = {9, 9, 9, 9};
    xco::write_json_atomic(f.saved_path, bad); check_fallback();
    bad = valid; bad["schema_version"] = 2;
    xco::write_json_atomic(f.saved_path, bad); check_fallback();
    bad = valid; bad.erase("bitrate_kbps"); bad["unexpected"] = 0;
    xco::write_json_atomic(f.saved_path, bad); check_fallback();
    for (const auto& node : {xco::Json(0), xco::Json(128), xco::Json(256), xco::Json("invalid")}) {
        bad = valid; bad["node_id"] = node;
        xco::write_json_atomic(f.saved_path, bad); check_fallback();
    }
    for (const auto& rate : {xco::Json(0), xco::Json(-1), xco::Json(1001), xco::Json(125.5), xco::Json("invalid")}) {
        bad = valid; bad["bitrate_kbps"] = rate;
        xco::write_json_atomic(f.saved_path, bad); check_fallback();
    }
    f.saved_path = f.directory;
    check_fallback();
}

void reset_with_invalid_persistence() {
    Fixture f;
    xco::Model model; model.load(xco::read_json(f.model_path));
    xco::write_json_atomic(f.saved_path, {{"schema_version", 1}, {"identity", model.identity},
        {"node_id", 20}, {"bitrate_kbps", 500}});
    CHECK(f.initialize() == 0);
    CHECK(f.status().active_node_id == 20 && f.status().active_bitrate_kbps == 500);
    f.start();
    { std::ofstream file(f.saved_path); file << "{broken"; }
    fake::inject(0, {0x81, 20});
    wait_for([&] { return f.status().active_node_id == 10; });
    const auto status = f.status();
    CHECK(status.runtime_state == XNET_CO_RUNTIME_RUNNING);
    CHECK(status.pending_node_id == 10);
    CHECK(status.active_bitrate_kbps == 125 && status.pending_bitrate_kbps == 125);
    CHECK(diagnostic(f.device).find(std::to_string(XNET_CO_IO_ERROR) + ": ") == 0);
}

void reset_status_lifecycle() {
    Fixture f;
    auto check_counts = [&](uint32_t node, uint32_t communication) {
        const auto status = f.status();
        CHECK(status.reset_node_count == node && status.reset_communication_count == communication);
    };
    const auto fresh = f.status();
    CHECK(fresh.runtime_state == XNET_CO_RUNTIME_UNINITIALIZED && fresh.nmt_state == 0
        && fresh.node_guarding_timeout == 0);
    check_counts(0, 0);
    CHECK(xnet_canopen_get_status(f.device, nullptr) == XNET_CO_INVALID_ARGUMENT);
    CHECK(f.initialize() == 0); check_counts(0, 0);
    f.start(); operational(f); check_counts(0, 0);
    const auto batch = f.bind({{0x2000, 0, 2}});
    uint8_t bytes[2]{77, 0}; int32_t entry_status;
    CHECK(xnet_canopen_write_batch(f.device, batch, bytes, 2, &entry_status, 1) == 0);

    exchange(0, {0x82, 10}, 0x70a, 0);
    auto status = f.status();
    CHECK(status.reset_node_count == 0 && status.reset_communication_count == 1
        && status.nmt_state == CO_NMT_PRE_OPERATIONAL && status.node_guarding_timeout == 0);
    CHECK(xnet_canopen_read_batch(f.device, batch, bytes, 2, &entry_status, 1) == 0 && bytes[0] == 77);
    exchange(0, {0x81, 10}, 0x70a, 0);
    status = f.status();
    CHECK(status.reset_node_count == 1 && status.reset_communication_count == 1
        && status.nmt_state == CO_NMT_PRE_OPERATIONAL);
    CHECK(xnet_canopen_read_batch(f.device, batch, bytes, 2, &entry_status, 1) == 0 && bytes[0] == 0);
    for (int i = 0; i < 3; ++i) check_counts(1, 1);
    exchange(0, {0x82, 0}, 0x70a, 0); check_counts(1, 2);
    exchange(0, {0x82, 10}, 0x70a, 0); check_counts(1, 3);
    exchange(0, {0x81, 0}, 0x70a, 0); check_counts(2, 3);

    fake::inject(0, {0x81, 11});
    CHECK(sdo_upload(0x1000) == 0); check_counts(2, 3);
    operational(f);
    fake::inject(0, {2, 10});
    wait_for([&] { return f.status().nmt_state == CO_NMT_STOPPED; });
    fake::inject(0, {0x80, 10});
    wait_for([&] { return f.status().nmt_state == CO_NMT_PRE_OPERATIONAL; });
    check_counts(2, 3);
    CHECK(xnet_canopen_start(f.device) == 0); check_counts(2, 3);
    CHECK(xnet_canopen_stop(f.device) == 0); check_counts(2, 3);
    CHECK(xnet_canopen_stop(f.device) == 0);
    f.start(); check_counts(2, 3);
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    status = f.status();
    CHECK(status.runtime_state == XNET_CO_RUNTIME_UNINITIALIZED && status.nmt_state == 0
        && status.node_guarding_timeout == 0);
    check_counts(2, 3);
    CHECK(xnet_canopen_uninitialize(f.device) == 0); check_counts(2, 3);
    fake::fail_next("create_output", nxErrInternalError);
    CHECK(f.initialize() == XNET_CO_XNET_ERROR); check_counts(2, 3);
    CHECK(f.initialize() == 0); check_counts(0, 0);
    f.start(); check_counts(0, 0);
}

void failed_reset_status() {
    for (const uint8_t command : {0x81, 0x82}) {
        for (const char* operation : {"stop", "start"}) {
            Fixture f; CHECK(f.initialize() == 0); f.start();
            exchange(0, {0x81, 10}, 0x70a, 0);
            exchange(0, {0x82, 10}, 0x70a, 0);
            fake::fail_next(operation, nxErrInternalError);
            fake::inject(0, {command, 10});
            wait_for([&] { return f.status().runtime_state == XNET_CO_RUNTIME_FAILED; });
            const auto failed = f.status();
            CHECK(failed.reset_node_count == 1 && failed.reset_communication_count == 1);
            CHECK(diagnostic(f.device).find("fake XNET error") != std::string::npos);
            CHECK(xnet_canopen_uninitialize(f.device) == 0);
            CHECK(f.status().reset_node_count == 1 && f.status().reset_communication_count == 1);
            CHECK(f.initialize() == 0);
            CHECK(f.status().reset_node_count == 0 && f.status().reset_communication_count == 0);
        }
    }
}

void reset_status_wraparound() {
    Fixture f; CHECK(f.initialize() == 0); f.start();
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        f.device->reset_node_count = UINT32_MAX;
        f.device->reset_communication_count = UINT32_MAX;
    }
    exchange(0, {0x82, 10}, 0x70a, 0);
    auto status = f.status();
    CHECK(status.reset_node_count == UINT32_MAX && status.reset_communication_count == 0);
    exchange(0, {0x81, 10}, 0x70a, 0);
    status = f.status();
    CHECK(status.reset_node_count == 0 && status.reset_communication_count == 0);
}

void reset_status_before_bootup_transmission() {
    Fixture f; CHECK(f.initialize() == 0); f.start();
    fake::clear_transmitted();
    fake::output_full(true);
    fake::inject(0, {0x81, 10});
    wait_for([&] { return f.status().reset_node_count == 1; });
    const auto status = f.status();
    CHECK(status.runtime_state == XNET_CO_RUNTIME_RUNNING && status.reset_communication_count == 0
        && status.active_node_id == 10 && status.node_guarding_timeout == 0);
    CHECK(fake::transmitted().empty());
    fake::output_full(false);
    wait_for([] {
        for (const auto& frame : fake::transmitted())
            if (frame.Identifier == 0x70a && frame.PayloadLength == 1 && frame.Payload[0] == 0) return true;
        return false;
    });
    CHECK(f.status().reset_node_count == 1);
}

void errors_and_cleanup() {
    Fixture f;
    fake::fail_next("create_output", nxErrInternalError);
    CHECK(f.initialize() == XNET_CO_XNET_ERROR && fake::sessions() == 0);
    CHECK(f.initialize() == 0);
    int32_t code; uint32_t required;
    CHECK(xnet_canopen_get_last_error(f.device, &code, nullptr, 0, &required) == XNET_CO_BUFFER_TOO_SMALL);
    CHECK(code == XNET_CO_XNET_ERROR && required > 1);
    CHECK(xnet_canopen_clear_error(f.device) == 0);
    f.start();
    fake::fail_next("read", nxErrInternalError);
    wait_for([&] { return f.status().runtime_state == XNET_CO_RUNTIME_FAILED; });
    CHECK(diagnostic(f.device).find("read CAN") != std::string::npos);
    CHECK(xnet_canopen_start(f.device) == XNET_CO_INVALID_STATE);
    CHECK(xnet_canopen_uninitialize(f.device) == 0 && fake::sessions() == 0);
    CHECK(f.initialize() == 0);
    fake::fail_next("clear", nxErrInternalError);
    CHECK(xnet_canopen_uninitialize(f.device) == XNET_CO_XNET_ERROR);
    CHECK(xnet_canopen_uninitialize(f.device) == 0 && fake::sessions() == 0);
}

void can_state_status() {
    Fixture f;
    CHECK(f.status().can_state_valid == 0 && f.status().can_state_raw == 0);
    CHECK(f.initialize() == 0);
    CHECK(f.status().can_state_valid == 0);
    f.start();
    wait_for([&] { return f.status().can_state_valid == 1; });
    CHECK(f.status().can_state_raw == 0);
    const uint32_t passive = (128U << 24) | (97U << 16) | (nxCANLastErr_CRC << 8)
        | nxCANCommState_ErrorPassive | 0x30U;
    fake::bus_state(passive);
    wait_for([&] { return f.status().can_state_raw == passive; });
    auto status = f.status();
    CHECK(status.can_state_valid == 1 && status.runtime_state == XNET_CO_RUNTIME_RUNNING);
    CHECK(status.can_error_status == (CO_CAN_ERRTX_WARNING | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE));
    fake::bus_state(0);
    wait_for([&] { return f.status().can_state_raw == 0 && f.status().can_error_status == 0; });
    CHECK(diagnostic(f.device).find("CAN communication error flags") != std::string::npos);
    exchange(0, {0x82, 10}, 0x70a, 0);
    CHECK(f.status().can_state_valid == 1);
    fake::bus_state(passive);
    wait_for([&] { return f.status().can_state_raw == passive; });
    CHECK(xnet_canopen_stop(f.device) == 0);
    CHECK(f.status().can_state_valid == 1 && f.status().can_state_raw == passive);
    CHECK(xnet_canopen_clear_error(f.device) == 0);
    CHECK(f.status().can_state_raw == passive);

    // Status reads must use the cache, not consume another NI-XNET state query.
    fake::fail_next("state", nxErrInternalError);
    CHECK(f.status().can_state_raw == passive);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CO_CANmodule_process(f.device->co->CANmodule);
    }
    CHECK(f.status().runtime_state == XNET_CO_RUNTIME_FAILED);
    CHECK(f.status().can_state_valid == 1 && f.status().can_state_raw == passive);
    CHECK(diagnostic(f.device).find("read CAN state (-1074384895)") != std::string::npos);
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    CHECK(f.status().can_state_valid == 0 && f.status().can_state_raw == 0);

    fake::fail_next("create_input", nxErrInternalError);
    CHECK(f.initialize() == XNET_CO_XNET_ERROR);
    CHECK(f.status().can_state_valid == 0);
    CHECK(f.initialize() == 0);
    fake::bus_state(passive, nxErrMissingBusPower);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CO_CANmodule_process(f.device->co->CANmodule);
    }
    CHECK(f.status().runtime_state == XNET_CO_RUNTIME_FAILED);
    CHECK(f.status().can_state_valid == 0 && f.status().can_state_raw == 0);
    CHECK(diagnostic(f.device).find("CAN state fault (-1074384856)") != std::string::npos);
}

void caller_managed_resources() {
    Fixture f;
    f.saved_path = std::filesystem::path(".") / std::filesystem::relative(f.saved_path);
    CHECK(f.initialize("can1") == 0);
    CHECK(f.device->interface_name == "can1");
    CHECK(f.device->persistence_path == f.saved_path);
    CHECK(f.device->persistence_path.is_relative());
    xnet_canopen_handle other; CHECK(xnet_canopen_create(&other) == 0);
    const auto path = (f.directory / "other.json").string();
    CHECK(xnet_canopen_initialize(other, "can1", f.model_path.string().c_str(), path.c_str()) == 0);
    CHECK(fake::sessions() == 4);
    CHECK(xnet_canopen_uninitialize(other) == 0);
    CHECK(xnet_canopen_initialize(other, "CAN2", f.model_path.string().c_str(), f.saved_path.string().c_str()) == 0);
    CHECK(fake::sessions() == 4);
    CHECK(xnet_canopen_uninitialize(other) == 0);
    CHECK(xnet_canopen_initialize(other, "can1", f.model_path.string().c_str(), f.saved_path.string().c_str()) == 0);
    CHECK(fake::sessions() == 4);
    CHECK(xnet_canopen_destroy(other) == 0);
}

void queues_and_filtering() {
    Fixture f; CHECK(f.initialize() == 0); f.start(); operational(f);
    auto batch = f.bind({{0x2001, 0, 4}});
    uint8_t data[4]{}; int32_t status;
    fake::inject(0x20a, {9, 0, 0, 0}, nxFrameType_CAN_Data, nxFrameFlags_TransmitEcho);
    fake::inject(0x20a, {8, 0, 0, 0}, nxFrameType_CANFD_Data);
    std::this_thread::sleep_for(15ms);
    CHECK(xnet_canopen_read_batch(f.device, batch, data, 4, &status, 1) == 0 && data[0] == 0);
    fake::output_full(true);
    CHECK(xnet_canopen_request_tpdo(f.device, 1) == 0);
    wait_for([&] { return (f.status().can_error_status & CO_CAN_ERRTX_OVERFLOW) != 0; });
    fake::output_full(false);
    wait_for([] { for (const auto& frame : fake::transmitted()) if (frame.Identifier == 0x18a) return true; return false; });
    CHECK(xnet_canopen_stop(f.device) == 0);
    fake::inject(0x20a, {7, 0, 0, 0});
    CHECK(xnet_canopen_stop(f.device) == 0);
    f.start(); operational(f);
    CHECK(xnet_canopen_read_batch(f.device, batch, data, 4, &status, 1) == 0 && data[0] == 0);
    fake::bus_state(nxCANCommState_BusOff);
    wait_for([&] { return (f.status().can_error_status & CO_CAN_ERRTX_BUS_OFF) != 0; });
}

void concurrent_channels() {
    Fixture f; CHECK(f.initialize() == 0); f.start();
    auto batch = f.bind({{0x2000, 0, 2}});
    std::atomic<int> errors{0};
    auto operation = [&] {
        for (int i = 0; i < 500; ++i) {
            uint8_t bytes[2]{static_cast<uint8_t>(i), static_cast<uint8_t>(i)};
            int32_t status;
            if (xnet_canopen_write_batch(f.device, batch, bytes, 2, &status, 1) != 0) ++errors;
            if (xnet_canopen_read_batch(f.device, batch, bytes, 2, &status, 1) != 0 || bytes[0] != bytes[1]) ++errors;
        }
    };
    std::thread first(operation), second(operation);
    first.join(); second.join();
    CHECK(errors == 0);
}

void synchronous_requests() {
    Fixture f;
    auto j = xco::read_json(f.model_path);
    j["tpdos"][0]["transmission_type"] = 0;
    f.model(j); CHECK(f.initialize() == 0); f.start(); operational(f);
    fake::clear_transmitted();
    CHECK(xnet_canopen_request_tpdo(f.device, 1) == 0);
    std::this_thread::sleep_for(10ms);
    for (const auto& frame : fake::transmitted()) CHECK(frame.Identifier != 0x18a);
    fake::inject(0x80, {});
    wait_for([] { for (const auto& frame : fake::transmitted()) if (frame.Identifier == 0x18a) return true; return false; });
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    j["tpdos"][0]["transmission_type"] = 1;
    f.model(j); CHECK(f.initialize() == 0); f.start(); operational(f);
    CHECK(xnet_canopen_request_tpdo(f.device, 1) == XNET_CO_INVALID_STATE);
}

void node_guarding() {
    Fixture f;
    auto j = xco::read_json(f.model_path);
    j["heartbeat_ms"] = 0;
    j["node_guarding"] = {{"guard_time_ms", 100}, {"life_time_factor", 3}};
    f.model(j); CHECK(f.initialize() == 0); f.start();
    CHECK(sdo_upload(0x100c) == 100 && sdo_upload(0x100d) == 3 && sdo_upload(0x100e) == 0x70a);
    fake::clear_transmitted();
    fake::inject(0x70a, {0});
    std::this_thread::sleep_for(15ms);
    for (const auto& frame : fake::transmitted()) CHECK(frame.Identifier != 0x70a);
    CHECK(guarding_request().Payload[0] == CO_NMT_PRE_OPERATIONAL);
    CHECK(guarding_request().Payload[0] == (CO_NMT_PRE_OPERATIONAL | 0x80));
    operational(f);
    CHECK(guarding_request().Payload[0] == CO_NMT_OPERATIONAL);
    wait_for([&] { return f.status().node_guarding_timeout == 1; });
    wait_for([] {
        for (const auto& frame : fake::transmitted())
            if (frame.Identifier == 0x8a && frame.Payload[0] == 0x30 && frame.Payload[1] == 0x81) return true;
        return false;
    });
    wait_for([&] { return f.status().nmt_state == CO_NMT_PRE_OPERATIONAL; });
    guarding_request();
    CHECK(f.status().node_guarding_timeout == 0);
    CHECK(sdo_download(0x100c, 0, 65535, 2).Payload[0] == 0x60);
    CHECK(sdo_download(0x100d, 0, 255, 1).Payload[0] == 0x80);
    CHECK(sdo_upload(0x100d) == 3);
    CHECK(sdo_download(0x100c, 0, 60, 2).Payload[0] == 0x60);
    CHECK(sdo_download(0x100d, 0, 4, 1).Payload[0] == 0x60);
    CHECK(xnet_canopen_stop(f.device) == 0);
    f.start();
    CHECK(f.status().node_guarding_timeout == 0);
    CHECK(sdo_upload(0x100c) == 60 && sdo_upload(0x100d) == 4);
    CHECK(guarding_request().Payload[0] == CO_NMT_PRE_OPERATIONAL);
    exchange(0, {0x82, 10}, 0x70a, 0);
    CHECK(sdo_upload(0x100c) == 100 && sdo_upload(0x100d) == 3);
    CHECK(f.status().node_guarding_timeout == 0);
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    CHECK(f.status().node_guarding_timeout == 0);
}

void supervision_switching() {
    Fixture f;
    auto j = xco::read_json(f.model_path);
    j["node_guarding"] = {{"guard_time_ms", 20}, {"life_time_factor", 1}};
    f.model(j); CHECK(f.initialize() == 0); f.start();
    fake::clear_transmitted();
    fake::inject(0x70a, {}, nxFrameType_CAN_Remote);
    std::this_thread::sleep_for(40ms);
    CHECK(f.status().node_guarding_timeout == 0);
    for (const auto& frame : fake::transmitted()) CHECK(frame.Identifier != 0x70a);
    CHECK(sdo_download(0x1017, 0, 0, 2).Payload[0] == 0x60);
    guarding_request();
    wait_for([&] { return f.status().node_guarding_timeout == 1; });
    CHECK(sdo_download(0x1017, 0, 50, 2).Payload[0] == 0x60);
    CHECK(f.status().node_guarding_timeout == 0);
    wait_for([] {
        for (const auto& frame : fake::transmitted()) if (frame.Identifier == 0x70a) return true;
        return false;
    });
    CHECK(sdo_download(0x1017, 0, 0, 2).Payload[0] == 0x60);
    CHECK(sdo_download(0x100d, 0, 0, 1).Payload[0] == 0x60);
    CHECK((guarding_request().Payload[0] & 0x7f) == CO_NMT_PRE_OPERATIONAL);
    std::this_thread::sleep_for(40ms);
    CHECK(f.status().node_guarding_timeout == 0);
    CHECK(sdo_download(0x100d, 0, 1, 1).Payload[0] == 0x60);
    guarding_request();
    wait_for([&] { return f.status().node_guarding_timeout == 1; });
    CHECK(sdo_download(0x100c, 0, 0, 2).Payload[0] == 0x60);
    CHECK(f.status().node_guarding_timeout == 0);
}

void configurable_services() {
    Fixture f;
    auto j = xco::read_json(f.model_path);
    j["sync"] = {{"cob_id", "0x90"}};
    j["emcy"] = {{"cob_id", "0xa0"}, {"inhibit_100us", 10}};
    j["tpdos"][0]["transmission_type"] = 0;
    f.model(j); CHECK(f.initialize() == 0); f.start(); operational(f);
    CHECK(sdo_upload(0x1005) == 0x90 && sdo_upload(0x1014) == 0xa0 && sdo_upload(0x1015) == 10);
    auto check_sync = [&](uint16_t old_id, uint16_t new_id) {
        fake::clear_transmitted();
        CHECK(xnet_canopen_request_tpdo(f.device, 1) == 0);
        fake::inject(old_id, {});
        std::this_thread::sleep_for(15ms);
        for (const auto& frame : fake::transmitted()) CHECK(frame.Identifier != 0x18a);
        fake::inject(new_id, {});
        wait_for([] {
            for (const auto& frame : fake::transmitted()) if (frame.Identifier == 0x18a) return true;
            return false;
        });
    };
    check_sync(0x80, 0x90);
    CHECK(sdo_download(0x1005, 0, 0x91, 4).Payload[0] == 0x60);
    check_sync(0x90, 0x91);
    for (uint32_t invalid : {0xa0U, 0x18aU, 0x40000080U, 0x70aU})
        CHECK(sdo_download(0x1005, 0, invalid, 4).Payload[0] == 0x80);
    CHECK(sdo_upload(0x1005) == 0x91);
    CHECK(sdo_download(0x1014, 0, 0xa1, 4).Payload[0] == 0x80);
    CHECK(sdo_download(0x1014, 0, 0x800000a0, 4).Payload[0] == 0x60);
    CHECK(sdo_upload(0x1014) == 0x800000a0);
    CHECK(sdo_download(0x1014, 0, 0x800000a1, 4).Payload[0] == 0x60);
    CHECK(sdo_download(0x1014, 0, 0x91, 4).Payload[0] == 0x80);
    CHECK(sdo_download(0x1014, 0, 0x18a, 4).Payload[0] == 0x80);
    CHECK(sdo_download(0x1014, 0, 0xa1, 4).Payload[0] == 0x60);
    CHECK(sdo_download(0x1015, 0, 20, 2).Payload[0] == 0x60);
    fake::clear_transmitted();
    CHECK(xnet_canopen_set_fault(f.device, 0x40, 0x5000, 42, 1) == 0);
    wait_for([] {
        for (const auto& frame : fake::transmitted())
            if (frame.Identifier == 0xa1 && frame.Payload[1] == 0x50) return true;
        return false;
    });
    CHECK(xnet_canopen_stop(f.device) == 0); f.start();
    CHECK(sdo_upload(0x1005) == 0x91 && sdo_upload(0x1014) == 0xa1 && sdo_upload(0x1015) == 20);
    exchange(0, {0x82, 10}, 0x70a, 0);
    CHECK(sdo_upload(0x1005) == 0x90 && sdo_upload(0x1014) == 0xa0 && sdo_upload(0x1015) == 10);
    CHECK(sdo_download(0x1800, 1, 0x8000018a, 4).Payload[0] == 0x80);
    CHECK(sdo_download(0x1a00, 0, 0, 1).Payload[0] == 0x80);
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    j.erase("emcy");
    f.model(j); CHECK(f.initialize() == 0); f.start();
    CHECK(sdo_upload(0x1014) == 0x8a);
    CHECK(sdo_download(0x1014, 0, 0x8000008a, 4).Payload[0] == 0x60);
    CHECK(xnet_canopen_stop(f.device) == 0); f.start();
    CHECK(sdo_upload(0x1014) == 0x8000008a);
    CHECK(sdo_download(0x1014, 0, 0xa2, 4).Payload[0] == 0x60);
    CHECK(xnet_canopen_stop(f.device) == 0); f.start();
    CHECK(sdo_upload(0x1014) == 0xa2);
    exchange(0, {0x82, 10}, 0x70a, 0);
    CHECK(sdo_upload(0x1014) == 0x8a);
}

void sync_configuration_and_validation() {
    Fixture f;
    auto j = xco::read_json(f.model_path);
    j["sync"] = {{"cob_id", 0x90}};
    j["emcy"] = {{"cob_id", 0x90}};
    f.model(j); CHECK(f.initialize() == XNET_CO_MODEL_ERROR);
    CHECK(fake::sessions() == 0);
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    j["emcy"]["cob_id"] = 0xa0;
    j["tpdos"][0]["cob_id"] = 0x90;
    f.model(j); CHECK(f.initialize() == XNET_CO_MODEL_ERROR);
    CHECK(xnet_canopen_uninitialize(f.device) == 0);
    j["tpdos"][0].erase("cob_id");
    j["sync"]["counter_overflow"] = 2;
    f.model(j); CHECK(f.initialize() == 0); f.start();
    CHECK(sdo_upload(0x1019) == 2);
    CHECK(sdo_download(0x1006, 0, 20000, 4).Payload[0] == 0x60);
    CHECK(sdo_download(0x1007, 0, 1000, 4).Payload[0] == 0x60);
    CHECK(sdo_upload(0x1006) == 20000 && sdo_upload(0x1007) == 1000);
    fake::inject(0x90, {1});
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->co->SYNC->syncIsOutsideWindow && f.device->co->SYNC->timeoutError == 2;
    });
    CHECK(xnet_canopen_stop(f.device) == 0); f.start();
    CHECK(sdo_upload(0x1006) == 20000 && sdo_upload(0x1007) == 1000);
    exchange(0, {0x81, 10}, 0x70a, 0);
    CHECK(sdo_upload(0x1006) == 0 && sdo_upload(0x1007) == 0 && sdo_upload(0x1019) == 2);
}

void sync_observations() {
    Fixture f;
    auto j = xco::read_json(f.model_path);
    j["sync"] = {{"cob_id", 0x90}, {"counter_overflow", 2}};
    f.model(j);
    CHECK(f.initialize() == XNET_CO_OK);
    f.start();
    xnet_canopen_sync_event_t event{};
    CHECK(xnet_canopen_read_sync_event(f.device, 0, 0, &event) == XNET_CO_NOT_FOUND);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        fake::inject(0x90, {}, nxFrameType_CAN_Data, 0, 100);
        fake::inject(0x90, {1}, nxFrameType_CAN_Data, 0, 101);
        fake::inject(0x90, {2}, nxFrameType_CAN_Data, 0, 102);
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_observation_sequence == 2;
    });
    CHECK(xnet_canopen_read_sync_event(f.device, 0, 0, &event) == XNET_CO_OK);
    const auto epoch = event.epoch;
    CHECK(event.sequence == 1 && event.timestamp_100ns == 101 && event.counter == 1
        && event.counter_present == 1 && event.can_id == 0x90);
    CHECK(xnet_canopen_read_sync_event(f.device, epoch, 1, &event) == XNET_CO_OK);
    CHECK(event.sequence == 2 && event.timestamp_100ns == 102 && event.counter == 2);
    CHECK(xnet_canopen_read_sync_event(f.device, epoch, 2, &event) == XNET_CO_NOT_FOUND);
    {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        for (int i = 0; i < 65; ++i) fake::inject(0x90, {1});
    }
    wait_for([&] {
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        return f.device->sync_observation_sequence == 67;
    });
    CHECK(xnet_canopen_read_sync_event(f.device, epoch, 2, &event) == XNET_CO_ACCESS_ERROR);
    CHECK(xnet_canopen_read_sync_event(f.device, 0, 0, &event) == XNET_CO_OK && event.sequence == 4);
    fake::fail_next("read", nxErrInputQueueOverflow);
    wait_for([&] { return xnet_canopen_read_sync_event(f.device, epoch, 67, &event) == XNET_CO_INVALID_STATE; });
    fake::inject(0x90, {2}, nxFrameType_CAN_Data, 0, 123);
    wait_for([&] { return xnet_canopen_read_sync_event(f.device, 0, 0, &event) == XNET_CO_OK; });
    CHECK(event.epoch != epoch && event.sequence == 1 && event.timestamp_100ns == 123);
    const auto overflow_epoch = event.epoch;
    CHECK(sdo_download(0x1005, 0, 0x91, 4).Payload[0] == 0x60);
    CHECK(xnet_canopen_read_sync_event(f.device, overflow_epoch, 1, &event) == XNET_CO_INVALID_STATE);
    fake::inject(0x91, {1}, nxFrameType_CAN_Data, 0, 124);
    wait_for([&] { return xnet_canopen_read_sync_event(f.device, 0, 0, &event) == XNET_CO_OK; });
    CHECK(event.can_id == 0x91 && event.sequence == 1 && event.timestamp_100ns == 124);
    const auto changed_epoch = event.epoch;
    CHECK(xnet_canopen_stop(f.device) == XNET_CO_OK);
    f.start();
    CHECK(xnet_canopen_read_sync_event(f.device, changed_epoch, 1, &event) == XNET_CO_INVALID_STATE);
}

void persistence_atomicity() {
    Fixture f;
    const auto contents = xco::Json{{"original", true}};
    xco::write_json_atomic(f.saved_path, contents);
    xco::write_json_atomic(f.saved_path, {{"replacement", true}});
    CHECK(xco::read_json(f.saved_path) == xco::Json({{"replacement", true}}));
    const auto target = f.directory / "directory.json";
    CHECK(std::filesystem::create_directory(target));
    bool rejected = false;
    try { xco::write_json_atomic(target, contents); } catch (const xco::Error&) { rejected = true; }
    CHECK(rejected && std::filesystem::is_directory(target));
    for (const auto& entry : std::filesystem::directory_iterator(f.directory))
        CHECK(entry.path().filename().string().find(".tmp.") == std::string::npos);
    std::filesystem::remove(target);
}

class FailingLogSink : public spdlog::sinks::base_sink<std::mutex> {
    void sink_it_(const spdlog::details::log_msg&) override {
        throw spdlog::spdlog_ex("injected log write failure");
    }
    void flush_() override {}
};

void can_state_logging() {
    Fixture f; CHECK(f.initialize() == 0);
    std::ostringstream messages;
    RestoreLogSinks restore(f.device->logger);
    f.device->logger->sinks() = {std::make_shared<spdlog::sinks::ostream_sink_mt>(messages)};
    auto sample = [&](uint32_t raw) {
        fake::bus_state(raw);
        std::lock_guard<std::recursive_mutex> lock(f.device->mutex);
        CO_CANmodule_process(f.device->co->CANmodule);
        CHECK(f.device->can_state.valid && f.device->can_state.raw == raw);
    };
    auto records = [&] {
        const auto text = messages.str();
        size_t count = 0, offset = 0;
        while ((offset = text.find("CAN controller ", offset)) != std::string::npos) { ++count; ++offset; }
        return count;
    };
    sample(0);
    CHECK(records() == 1);
    CHECK(messages.str().find("CAN controller sample: raw=0x00000000 comm=error-active(0)") != std::string::npos);
    sample(0);
    CHECK(records() == 1);
    // Make the throttle interval deterministic without sleeping or running a worker.
    f.device->can_state.last_log = xco::Clock::now();
    sample(1U << 24);
    sample(2U << 24);
    CHECK(records() == 1 && f.status().can_state_raw == (2U << 24));
    f.device->can_state.last_log -= 1s;
    sample(2U << 24);
    CHECK(records() == 2);
    CHECK(messages.str().find("rx_errors=2") != std::string::npos);
    CHECK(messages.str().find("counter_changes_suppressed=2") != std::string::npos);
    sample((128U << 24) | (97U << 16) | (nxCANLastErr_CRC << 8) | nxCANCommState_ErrorPassive | 0x30U);
    CHECK(records() == 3);
    CHECK(messages.str().find("comm=error-passive(1) tx_errors=97 rx_errors=128 last_error=CRC(6) "
        "transceiver_error=1 sleep=1 canopen_flags=0x0301") != std::string::npos);
    sample(0);
    CHECK(records() == 4 && f.status().can_error_status == 0);
    CHECK(messages.str().find("CAN controller recovered: raw=0x00000000") != std::string::npos);
    CHECK(diagnostic(f.device).find("CAN communication error flags") != std::string::npos);
    sample(nxCANCommState_BusOff);
    CHECK(records() == 5 && (f.status().can_error_status & CO_CAN_ERRTX_BUS_OFF) != 0);
    sample(0);
    CHECK(records() == 6);
    for (uint32_t code = 0; code <= 7; ++code) sample(code << 8);
    CHECK(messages.str().find("last_error=stuff(1)") != std::string::npos);
    CHECK(messages.str().find("last_error=form(2)") != std::string::npos);
    CHECK(messages.str().find("last_error=ACK(3)") != std::string::npos);
    CHECK(messages.str().find("last_error=bit-1(4)") != std::string::npos);
    CHECK(messages.str().find("last_error=bit-0(5)") != std::string::npos);
    CHECK(messages.str().find("last_error=unknown(7)") != std::string::npos);
    sample(nxCANCommState_Init);
    sample(0xFU);
    CHECK(messages.str().find("comm=initializing(3)") != std::string::npos);
    CHECK(messages.str().find("comm=unknown(15)") != std::string::npos);
    CHECK(std::string(xco::can_communication_name(0)) == "error-active");
}

void file_logging() {
    Fixture f; CHECK(f.initialize() == 0); f.start(); operational(f);
    const auto instance = f.device->log_instance;
    CHECK(xnet_canopen_set_fault(f.device, 0x40, 0x5000, 42, 1) == 0);
    CHECK(xnet_canopen_set_fault(f.device, 0x40, 0, 0, 0) == 0);
    fake::inject(0x7e5, {4, 1, 0, 0, 0, 0, 0, 0});
    CHECK(exchange(0x7e5, {0x11, 19, 0, 0, 0, 0, 0, 0}, 0x7e4, 0x11).Payload[1] == 0);
    fake::inject(0x7e5, {4, 0, 0, 0, 0, 0, 0, 0});
    exchange(0, {0x82, 10}, 0x713, 0);
    wait_for([&] { return f.status().active_node_id == 19; });
    CHECK(f.device->log_instance == instance);
    CHECK(xnet_canopen_stop(f.device) == 0);

    // The device is stopped and its tag is immutable during these writes.
    const auto marker = "concurrent logging check " + std::to_string(xco::Clock::now().time_since_epoch().count());
    auto messages = [&] {
        for (int i = 0; i < 50; ++i) f.device->log(xco::LogLevel::info, marker);
    };
    std::thread first(messages), second(messages);
    first.join(); second.join();
    const std::string long_message(4096, 'z');
    f.device->log(xco::LogLevel::info, "formatted {} hex=0x{:08x} text={}", 42, 0x1234U, long_message);
    f.device->log(xco::LogLevel::info, "literal braces: {} {not a format}");
    int32_t entry_status;
    uint8_t bytes[1]{};
    CHECK(xnet_canopen_read_batch(f.device, 0, bytes, 1, &entry_status, 1) == XNET_CO_NOT_FOUND);

    auto read_log = [] {
        std::ifstream file(XNET_CANOPEN_LOG_PATH);
        CHECK(file.is_open());
        return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    };
    const auto text = read_log();
    const auto tag = "[device=" + std::to_string(instance)
        + " interface=CAN1 identity=00000001:00000001:00000001:000003e9 node=";
    CHECK(text.find(tag + "10 ") != std::string::npos);
    CHECK(text.find(tag + "19 ") != std::string::npos);
    CHECK(text.find("NMT Operational") != std::string::npos);
    CHECK(text.find("fault asserted: bit=0x40 code=0x5000") != std::string::npos);
    CHECK(text.find("fault cleared: bit=0x40") != std::string::npos);
    CHECK(text.find("error code=-7: unknown or invalidated batch ID") != std::string::npos);
    CHECK(text.find("formatted 42 hex=0x00001234 text=" + long_message) != std::string::npos);
    CHECK(text.find("literal braces: {} {not a format}") != std::string::npos);
    size_t records = 0, offset = 0;
    while ((offset = text.find(marker, offset)) != std::string::npos) {
        ++records; ++offset;
    }
    CHECK(records == 100);
    const std::string rotation_payload(4096, 'x');
    for (int i = 0; i < 1300; ++i) f.device->log(xco::LogLevel::info, rotation_payload);
    const std::filesystem::path log_path(XNET_CANOPEN_LOG_PATH);
    const auto backup = log_path.parent_path() / (log_path.stem().string() + ".1" + log_path.extension().string());
    CHECK(std::filesystem::exists(backup));
    CHECK(std::filesystem::file_size(backup) <= 5U * 1024U * 1024U);
    CHECK(std::filesystem::file_size(log_path) <= 5U * 1024U * 1024U);
    {
        RestoreLogSinks restore(f.device->logger);
        f.device->logger->sinks() = {std::make_shared<FailingLogSink>()};
        CHECK(xnet_canopen_clear_error(f.device) == 0);
        f.device->log(xco::LogLevel::info, "this log write fails: {}", 42);
        CHECK(f.device->error_code == XNET_CO_IO_ERROR);
        CHECK(diagnostic(f.device).find("injected log write failure") != std::string::npos);
        f.device->fail(XNET_CO_ACCESS_ERROR, "original operation failure");
        CHECK(f.device->error_code == XNET_CO_ACCESS_ERROR);
        const auto failure = diagnostic(f.device);
        CHECK(failure.find("original operation failure") != std::string::npos
            && failure.find("file logging failed") != std::string::npos);
        f.device->fail(XNET_CO_ACCESS_ERROR, long_message);
        CHECK(f.device->error_code == XNET_CO_ACCESS_ERROR);
        CHECK(diagnostic(f.device) == std::to_string(XNET_CO_ACCESS_ERROR) + ": "
            + long_message.substr(0, f.device->error_message.size() - 1));
    }
    CHECK(xnet_canopen_clear_error(f.device) == 0);
    f.device->log(xco::LogLevel::info, "{:{}}", 42, -1);
    CHECK(f.device->error_code == XNET_CO_IO_ERROR);
    CHECK(diagnostic(f.device).find("file logging failed") != std::string::npos);
    CHECK(xnet_canopen_clear_error(f.device) == 0);
    f.device->log(xco::LogLevel::info, "logging recovered after sink failure");
    CHECK(read_log().find("logging recovered after sink failure") != std::string::npos);
}
}

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--validate-model") {
        try {
            xco::Model model; model.load(xco::read_json(argv[2]));
            model.restore(true, model.startup_node);
            return 0;
        } catch (const std::exception& e) {
            std::cerr << e.what() << "\n";
            return 1;
        }
    }
    if (!xnet_canopen_c_header_test()) return 1;
    const std::vector<std::pair<const char*, std::function<void()>>> tests = {
        {"model validation", model_validation}, {"DS402 model validation", drive_402_model_validation},
        {"drive unit adapter", drive_unit_adapter}, {"motor drive SI exchange", motor_drive_si_exchange},
        {"motor drive worker stop", motor_drive_worker_stop},
        {"DS402 SDO only", drive_402_sdo_only},
        {"DS402 PP integration", drive_402_integration},
        {"DS402 external exchange", drive_402_external_exchange},
        {"DS402 watchdog deadlines", drive_402_watchdog_deadlines}, {"NODE_ID offsets", node_id_offsets},
        {"DS402 missing model startup", drive_402_missing_model_startup},
        {"adapter session configuration", adapter_session_configuration},
        {"CAN termination configuration", can_termination_configuration},
        {"startup bitrates", startup_bitrates}, {"scalar limits", scalar_limits},
        {"batches and lifecycle", batches_and_lifecycle}, {"partial writes and strings", partial_and_strings},
        {"single OD access", single_od_access},
        {"packed batch buffers", packed_batches}, {"batch argument validation and limits", batch_validation},
        {"SDO PDO EMCY NMT", protocol_basics},
        {"device name SDO", device_name_sdo},
        {"eight PDO validation", eight_pdo_validation}, {"eight PDO traffic and lifecycle", eight_pdo_traffic},
        {"inactive PDO slots", inactive_pdo_slots}, {"eighth synchronous PDO", eighth_synchronous_pdo},
        {"ordered SYNC boundaries", ordered_sync_boundaries}, {"worker independent PDO cadence", worker_pdo_independent_cadence},
        {"timed history selection", timed_history_selection},
        {"timed history auto retirement", timed_history_auto_retirement},
        {"timed history bounded arrivals", timed_history_bounded_arrivals},
        {"timed feedback selection from accepted records", timed_feedback_selection_from_accepted_records},
        {"worker ordered SYNC execution", worker_ordered_sync_execution}, {"worker timed sparse PDO", worker_timed_sparse_pdo},
        {"timing diagnostic ring and lifecycle", timing_diagnostic_ring_and_lifecycle},
        {"worker sampling deadline at boundary", worker_sampling_deadline_crossed_at_boundary},
        {"worker required RPDO boundaries", worker_required_rpdo_boundaries},
        {"worker required RPDO plan validation", worker_required_rpdo_plan_validation},
        {"worker SYNC TPDO deadline", worker_sync_tpdo_deadline},
        {"worker SYNC backlog", worker_sync_backlog_is_not_replayed},
        {"worker SYNC write crosses deadline", worker_sync_write_crosses_deadline},
        {"worker SYNC write failure", worker_sync_write_failure},
        {"worker SYNC missing and overflow", worker_sync_missing_and_overflow},
        {"worker SYNC counter and clock", worker_sync_counter_and_clock},
        {"worker SYNC external rejected", worker_sync_external_rejected},
        {"worker external timed history", worker_external_timed_history},
        {"worker scheduled auto retirement", worker_external_scheduled_auto_retirement},
        {"worker ordered command watermark", worker_external_ordered_command_watermark},
        {"worker external scheduled read", worker_external_scheduled_read},
        {"worker scheduled assignment overflow", worker_external_scheduled_assignment_overflow},
        {"worker scheduled safety preemption", worker_external_scheduled_safety_preemption},
        {"worker scheduled feedback acceptance age", worker_external_scheduled_feedback_ages_before_acceptance},
        {"worker selected feedback source validity", worker_external_selected_feedback_source_validity},
        {"worker scheduled mapping change", worker_external_scheduled_mapping_change},
        {"worker scheduled command hold", worker_external_scheduled_command_hold},
        {"worker fixed feedback schedule", worker_external_fixed_feedback_schedule},
        {"worker fixed feedback plan validation", worker_external_fixed_feedback_plan_validation},
        {"worker scheduled hold plan validation", worker_external_hold_plan_validation},
        {"worker scheduled budget plan freeze", worker_external_budget_plan_freeze},
        {"worker scheduled history budget", worker_external_history_budget},
        {"worker timing provider contract", worker_external_timing_provider_contract},
        {"worker external feedback watermark", worker_external_feedback_watermark_required},
        {"worker external timed safety", worker_external_timed_safety_without_sync},
        {"worker external timed history overflow", worker_external_timed_history_overflow},
        {"worker external timed history epoch", worker_external_timed_history_epoch},
        {"worker external timed feedback faults", worker_external_timed_feedback_faults},
        {"LSS assignment and resets", lss_assignment_and_resets},
        {"LSS bitrate and persistence", lss_bitrate_and_storage}, {"storage failures", storage_failures},
        {"Reset Node with invalid persistence", reset_with_invalid_persistence},
        {"reset status lifecycle", reset_status_lifecycle}, {"failed reset status", failed_reset_status},
        {"reset status wraparound", reset_status_wraparound},
        {"reset status before boot-up transmission", reset_status_before_bootup_transmission},
        {"errors and cleanup", errors_and_cleanup}, {"CAN state status", can_state_status},
        {"CAN state logging", can_state_logging}, {"caller-managed resources", caller_managed_resources},
        {"queues and filtering", queues_and_filtering}, {"concurrent channels", concurrent_channels},
        {"synchronous TPDO requests", synchronous_requests}, {"node guarding", node_guarding},
        {"heartbeat and guarding switching", supervision_switching}, {"configurable SYNC and EMCY", configurable_services},
        {"SYNC timers and service validation", sync_configuration_and_validation}, {"SYNC observations", sync_observations},
        {"atomic persistence", persistence_atomicity}, {"file logging", file_logging}};
    int failed = 0;
    for (const auto& test : tests) {
        try { test.second(); std::cout << "PASS " << test.first << "\n"; }
        catch (const std::exception& e) { ++failed; std::cerr << "FAIL " << test.first << ": " << e.what() << "\n"; }
    }
    std::cout << tests.size() - failed << "/" << tests.size() << " tests passed\n";
    return failed ? 1 : 0;
}
