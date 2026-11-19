#include "internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

namespace xco {
namespace {
constexpr long double tau = 6.283185307179586476925286766559005768L;

int64_t rounded(long double value, bool is_unsigned, const std::string& where) {
    require(std::isfinite(value), XNET_CO_ACCESS_ERROR, where + ": non-finite conversion");
    const auto result = std::round(value);
    require(result >= (is_unsigned ? 0.L : static_cast<long double>(INT32_MIN))
        && result <= (is_unsigned ? static_cast<long double>(UINT32_MAX) : static_cast<long double>(INT32_MAX)),
        XNET_CO_ACCESS_ERROR, where + ": conversion outside 32-bit range");
    return static_cast<int64_t>(result);
}
int64_t scaled_integer(int64_t value, uint32_t numerator, uint32_t denominator, bool is_unsigned, const char* where) {
    require(value >= (is_unsigned ? 0LL : static_cast<int64_t>(INT32_MIN))
        && value <= (is_unsigned ? static_cast<int64_t>(UINT32_MAX) : static_cast<int64_t>(INT32_MAX)),
        XNET_CO_ACCESS_ERROR, std::string(where) + ": source outside 32-bit range");
    const auto magnitude = static_cast<uint64_t>(value < 0 ? -value : value);
    const uint64_t product = magnitude * numerator;
    uint64_t result = product / denominator;
    if ((product % denominator) * 2 >= denominator) ++result;
    const uint64_t maximum = is_unsigned ? UINT32_MAX : value < 0 ? uint64_t{1} << 31 : INT32_MAX;
    require(result <= maximum, XNET_CO_ACCESS_ERROR, std::string(where) + ": conversion outside 32-bit range");
    return value < 0 ? -static_cast<int64_t>(result) : static_cast<int64_t>(result);
}
const DriveUnitBinding& binding(const DriveAdapter& adapter, DriveQuantity quantity) {
    return adapter.bindings.at(static_cast<size_t>(quantity));
}
const char* core_unit(DriveQuantity quantity) {
    switch (quantity) {
        case DriveQuantity::target_position:
        case DriveQuantity::demand_position:
        case DriveQuantity::actual_position: return "counts";
        case DriveQuantity::profile_velocity:
        case DriveQuantity::demand_velocity:
        case DriveQuantity::actual_velocity: return "counts/s";
        default: return "counts/s2";
    }
}
}

void DriveAdapter::load(const Json& config) {
    keys(config, {"unit_profile", "counts_per_revolution", "direction", "zero_counts", "scales", "fault_mapping"},
        "drive_402.device_adapter");
    require(config.at("unit_profile").is_string(), XNET_CO_MODEL_ERROR, "device_adapter.unit_profile must be a string");
    unit_profile = config.at("unit_profile").get<std::string>();
    require(unit_profile == "counts" || unit_profile == "kinco_fd" || unit_profile == "identity", XNET_CO_MODEL_ERROR,
        "device_adapter.unit_profile must be counts, kinco_fd or identity");
    counts_per_revolution = static_cast<uint32_t>(unsigned_number(config.at("counts_per_revolution"),
        UINT32_MAX, "device_adapter.counts_per_revolution"));
    require(counts_per_revolution != 0, XNET_CO_MODEL_ERROR, "counts_per_revolution must be positive");
    const auto signed_option = [&](const char* key, int32_t fallback) {
        if (!config.contains(key)) return fallback;
        const auto& value = config.at(key);
        require(value.is_number_integer(), XNET_CO_MODEL_ERROR, std::string("device_adapter.") + key + ": expected integer");
        const long double number = value.get<long double>();
        require(number >= INT32_MIN && number <= INT32_MAX, XNET_CO_MODEL_ERROR,
            std::string("device_adapter.") + key + ": outside int32 range");
        return value.get<int32_t>();
    };
    direction = signed_option("direction", 1);
    zero_counts = signed_option("zero_counts", 0);
    require(direction == 1 || direction == -1, XNET_CO_MODEL_ERROR, "device_adapter.direction must be +1 or -1");
    bindings = {{
        {"target_position", 0x607a, nullptr, 1, 1, false, true, "position increment"},
        {"profile_velocity", 0x6081, nullptr, 1, 1, true, true, "configured counts units"},
        {"profile_acceleration", 0x6083, nullptr, 1, 1, true, true, "configured counts units"},
        {"profile_deceleration", 0x6084, nullptr, 1, 1, true, true, "configured counts units"},
        {"quick_stop_deceleration", 0x6085, nullptr, 1, 1, true, true, "configured counts units"},
        {"demand_position", 0x60fc, nullptr, 1, 1, false, true, "configured counts units"},
        {"demand_velocity", 0x606b, nullptr, 1, 1, false, true, "configured counts units"},
        {"actual_position", 0x6064, nullptr, 1, 1, false, true, "configured counts units"},
        {"actual_velocity", 0x606c, nullptr, 1, 1, false, true, "configured counts units"}
    }};
    if (unit_profile == "kinco_fd") {
        for (auto quantity : {DriveQuantity::profile_velocity, DriveQuantity::actual_velocity}) {
            auto& item = bindings[static_cast<size_t>(quantity)];
            item.numerator = 125; item.denominator = 2048;
            item.basis = "Kinco manual velocity rule";
        }
        for (auto quantity : {DriveQuantity::profile_acceleration, DriveQuantity::profile_deceleration}) {
            auto& item = bindings[static_cast<size_t>(quantity)];
            item.numerator = 15625; item.denominator = 256;
            item.basis = "Kinco manual acceleration rule";
        }
        for (auto quantity : {DriveQuantity::quick_stop_deceleration, DriveQuantity::demand_position,
                               DriveQuantity::demand_velocity, DriveQuantity::actual_position}) {
            auto& item = bindings[static_cast<size_t>(quantity)];
            item.confirmed = false;
            item.basis = "unconfirmed rule: assumed identical units, review required";
        }
    }
    if (unit_profile == "identity") {
        for (auto& item : bindings) {
            item.confirmed = false;
            item.basis = "unconfirmed rule: assumed identical units, review required";
        }
    }
    if (config.contains("scales")) {
        const auto& scales = config.at("scales");
        require(scales.is_object(), XNET_CO_MODEL_ERROR, "device_adapter.scales must be an object");
        for (auto it = scales.begin(); it != scales.end(); ++it) {
            auto item = std::find_if(bindings.begin(), bindings.end(), [&](const auto& candidate) {
                return it.key() == candidate.role;
            });
            require(item != bindings.end(), XNET_CO_MODEL_ERROR, "unknown device scale role: " + it.key());
            keys(it.value(), {"numerator", "denominator"}, "device_adapter.scales." + it.key());
            item->numerator = static_cast<uint32_t>(unsigned_number(it.value().at("numerator"), UINT32_MAX, it.key()));
            item->denominator = static_cast<uint32_t>(unsigned_number(it.value().at("denominator"), UINT32_MAX, it.key()));
            require(item->numerator && item->denominator, XNET_CO_MODEL_ERROR, "scale numerator/denominator must be positive");
            item->confirmed = true; item->basis = "explicit configured scale";
        }
    }
    fault_mapping.clear();
    if (config.contains("fault_mapping")) {
        const auto& mappings = config.at("fault_mapping");
        require(mappings.is_array(), XNET_CO_MODEL_ERROR, "device_adapter.fault_mapping must be an array");
        for (const auto& entry : mappings) {
            keys(entry, {"class", "detail", "device_code"}, "device_adapter.fault_mapping");
            const auto cls = static_cast<uint32_t>(unsigned_number(entry.at("class"), UINT32_MAX, "fault class"));
            const auto detail = static_cast<uint32_t>(unsigned_number(entry.at("detail"), UINT32_MAX, "fault detail"));
            const auto code = static_cast<uint16_t>(unsigned_number(entry.at("device_code"), UINT16_MAX, "device fault code"));
            require(cls && code, XNET_CO_MODEL_ERROR, "fault class and device_code must be nonzero");
            require(fault_mapping.emplace(std::make_pair(cls, detail), code).second, XNET_CO_MODEL_ERROR,
                "duplicate fault mapping");
        }
    }
}

void DriveAdapter::bind(Model& model) {
    for (auto& item : bindings) {
        auto object = std::find_if(model.objects.begin(), model.objects.end(), [&](const auto& candidate) {
            return candidate->index == item.index;
        });
        if (object == model.objects.end()) continue; // Mandatory PP objects are validated by Model.
        auto& obj = **object;
        require(obj.is_var_object && obj.values.size() == 1 && obj.values[0].sub == 0, XNET_CO_MODEL_ERROR,
            std::string(item.role) + ": expected scalar OD");
        auto& value = obj.values[0];
        require(value.size == 4 && value.type == (item.is_unsigned ? "UNSIGNED32" : "INTEGER32"),
            XNET_CO_MODEL_ERROR, std::string(item.role) + ": invalid OD type");
        const bool output = item.index == 0x60fc || item.index == 0x606b || item.index == 0x6064 || item.index == 0x606c;
        require(!output || ((value.attr & ODA_SDO_R) && !(value.attr & ODA_SDO_W)
            && !value.allows_local_write && !value.mapped_rpdo), XNET_CO_MODEL_ERROR,
            std::string(item.role) + ": output must be readable and not writable");
        item.value = &value;
    }
    try {
        for (auto quantity : {DriveQuantity::profile_velocity, DriveQuantity::profile_acceleration,
                               DriveQuantity::profile_deceleration, DriveQuantity::quick_stop_deceleration}) {
            if (binding(*this, quantity).value)
                require(read(quantity) > 0, XNET_CO_MODEL_ERROR, "positive profile rate quantizes to zero");
        }
        (void)read(DriveQuantity::target_position);
    } catch (const Error& error) {
        throw Error(XNET_CO_MODEL_ERROR, std::string("device_adapter configuration: ") + error.what());
    }
}

int64_t DriveAdapter::read(DriveQuantity quantity) const {
    const auto& item = binding(*this, quantity);
    require(item.value != nullptr, XNET_CO_MODEL_ERROR, std::string(item.role) + ": unbound OD");
    int64_t value = 0;
    if (item.is_unsigned) {
        uint32_t number; std::memcpy(&number, item.value->storage.data(), sizeof(number)); value = number;
    } else {
        int32_t number; std::memcpy(&number, item.value->storage.data(), sizeof(number)); value = number;
    }
    const auto result = scaled_integer(value, item.numerator, item.denominator,
        item.is_unsigned, item.role);
    require(!item.is_unsigned || result > 0, XNET_CO_ACCESS_ERROR, std::string(item.role) + ": positive rate required");
    return result;
}

int64_t DriveAdapter::raw(DriveQuantity quantity, int64_t counts) const {
    const auto& item = binding(*this, quantity);
    return scaled_integer(counts, item.denominator, item.numerator, item.is_unsigned, item.role);
}

void DriveAdapter::publish(const d402::Status& status, bool actual) {
    const std::pair<DriveQuantity, int32_t> outputs[] = {
        {DriveQuantity::demand_position, status.demand_position}, {DriveQuantity::demand_velocity, status.demand_velocity},
        {DriveQuantity::actual_position, status.actual_position}, {DriveQuantity::actual_velocity, status.actual_velocity}
    };
    std::array<int32_t, 4> values{};
    const size_t count = actual ? values.size() : 2;
    for (size_t i = 0; i < count; ++i)
        if (binding(*this, outputs[i].first).value) values[i] = static_cast<int32_t>(raw(outputs[i].first, outputs[i].second));
    for (size_t i = 0; i < count; ++i)
        if (auto* value = binding(*this, outputs[i].first).value) std::memcpy(value->data(), &values[i], sizeof(values[i]));
}

double DriveAdapter::position_rad(int32_t counts) const {
    return static_cast<double>(direction * (static_cast<long double>(counts) - zero_counts) * tau / counts_per_revolution);
}
double DriveAdapter::velocity_rad_s(int64_t counts_s) const {
    return static_cast<double>(direction * static_cast<long double>(counts_s) * tau / counts_per_revolution);
}
int32_t DriveAdapter::position_counts(double radians) const {
    return static_cast<int32_t>(rounded(zero_counts + direction * static_cast<long double>(radians)
        * counts_per_revolution / tau, false, "SI position"));
}
int32_t DriveAdapter::velocity_counts(double radians_s) const {
    return static_cast<int32_t>(rounded(direction * static_cast<long double>(radians_s)
        * counts_per_revolution / tau, false, "SI velocity"));
}
uint16_t DriveAdapter::fault_code(uint32_t fault_class, uint32_t detail) const {
    if (!fault_class) {
        require(detail == 0, XNET_CO_ACCESS_ERROR, "fault detail without fault class");
        return 0;
    }
    const auto found = fault_mapping.find({fault_class, detail});
    require(found != fault_mapping.end(), XNET_CO_ACCESS_ERROR, "unmapped motor fault class/detail");
    return found->second;
}
Json DriveAdapter::report() const {
    Json result = {{"unit_profile", unit_profile}, {"counts_per_revolution", counts_per_revolution},
        {"direction", direction}, {"zero_counts", zero_counts},
        {"core_to_si", {{"radians_per_count", static_cast<double>(tau / counts_per_revolution)},
                        {"units", "rad, rad/s, rad/s2"}, {"direction_source", "explicit adapter coordinate configuration"}}},
        {"converted", Json::array()}, {"unchanged", Json::array()},
        {"assumed_identity", Json::array()}, {"unbound", Json::array()}};
    for (size_t i = 0; i < bindings.size(); ++i) {
        const auto& item = bindings[i];
        std::ostringstream index; index << "0x" << std::hex << item.index;
        Json entry = {{"role", item.role}, {"index", index.str()}, {"subindex", 0},
            {"numerator", item.numerator}, {"denominator", item.denominator},
            {"core_unit", core_unit(static_cast<DriveQuantity>(i))}, {"basis", item.basis},
            {"confirmed", item.confirmed}};
        const char* list = !item.value ? "unbound" : item.numerator == item.denominator ? "unchanged" : "converted";
        result[list].push_back(entry);
        if (!item.confirmed) result["assumed_identity"].push_back(entry);
    }
    return result;
}
} // namespace xco
