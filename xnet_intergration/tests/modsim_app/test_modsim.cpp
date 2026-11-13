/**
 * @file Real CANopen stack plus external model verification without hardware.
 * @copyright 2026 CANopenNode contributors
 */
#include "simulator.hpp"
#include "../fake_xnet.hpp"
#include <functional>

#define CHECK(condition) do { if (!(condition)) throw std::runtime_error( \
    std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #condition); } while (false)

namespace {
using namespace std::chrono_literals;
void wait_until(const std::function<bool()>& predicate, unsigned line) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!predicate()) {
        if (std::chrono::steady_clock::now() >= deadline)
            throw std::runtime_error("modsim test timed out at line " + std::to_string(line));
        std::this_thread::sleep_for(1ms);
    }
    #define wait_for(...) wait_until(__VA_ARGS__, __LINE__)
}

void plant() {
    modsim::Parameters parameters;
    modsim::Motor motor(parameters);
    CO_402_command_frame_t command{};
    command.epoch = command.command_seq = command.action_seq = 1;
    command.mode = 1;
    command.flags = 1;
    command.target_position = 900000;
    command.demand_position = 1000;
    auto feedback = motor.step(command, 0);
    CHECK(feedback.actual_position > 0 && feedback.actual_position < command.demand_position);
    for (int i = 0; i < 1000; ++i) feedback = motor.step(command, i * parameters.period_us);
    CHECK(std::abs(feedback.actual_position - 1000) <= 1);
    command.demand_position = -1000;
    for (int i = 0; i < 1000; ++i) feedback = motor.step(command, i * parameters.period_us);
    CHECK(std::abs(feedback.actual_position + 1000) <= 1);
    command.demand_position = INT32_MAX;
    for (int i = 0; i < 3000; ++i) feedback = motor.step(command, i * parameters.period_us);
    CHECK(feedback.actual_position == parameters.max_position && (feedback.flags & 8));
    command.epoch = 2;
    command.flags = 0;
    command.action = CO_402_ACTION_ENABLE_OPERATION;
    feedback = motor.step(command, 0);
    CHECK(feedback.action_result == CO_402_ACTION_RESULT_BUSY);
    command.action = CO_402_ACTION_DISABLE_VOLTAGE;
    ++command.action_seq;
    feedback = motor.step(command, 0);
    CHECK(feedback.action_result == CO_402_ACTION_RESULT_BUSY);
    CHECK(feedback.actual_position == parameters.max_position);
    for (int i = 0; i < 10; ++i) feedback = motor.step(command, 0);
    CHECK(feedback.action_result == CO_402_ACTION_RESULT_DONE);

    parameters.initial_position = INT32_MIN;
    parameters.min_position = INT32_MIN;
    parameters.max_position = INT32_MAX;
    parameters.fault_code = 0x2310;
    parameters.fault_after_us = 20000;
    parameters.fault_duration_us = 20000;
    parameters.freeze_feedback_after_us = 10000;
    parameters.error_action = CO_402_ACTION_SWITCH_ON;
    modsim::Motor faulty(parameters);
    command.action = CO_402_ACTION_NONE;
    feedback = faulty.step(command, 20000);
    CHECK(feedback.actual_position == INT32_MIN && feedback.fault_code == 0x2310 && !(feedback.flags & 0x10));
    feedback = faulty.step(command, 40000);
    CHECK(feedback.fault_code == 0 && (feedback.flags & 0x10));
    command.action = CO_402_ACTION_SWITCH_ON;
    feedback = faulty.step(command, 40000);
    CHECK(feedback.action_result == CO_402_ACTION_RESULT_ERROR);
    CHECK(!faulty.feedback_enabled(10000) && faulty.heartbeat_enabled(10000));
    CHECK(modsim::next_sequence(UINT32_MAX) == 1);
}

struct Device {
    xnet_canopen_handle handle = nullptr;
    std::filesystem::path directory;
    Device() {
        fake::reset();
        directory = std::filesystem::temp_directory_path()
            / ("modsim-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        CHECK(std::filesystem::create_directory(directory));
        CHECK(xnet_canopen_create(&handle) == XNET_CO_OK);
        CHECK(xnet_canopen_initialize(handle, "CAN1", MODSIM_DEVICE_MODEL,
            (directory / "lss.json").string().c_str()) == XNET_CO_OK);
    }
    ~Device() {
        if (handle && xnet_canopen_destroy(handle) != XNET_CO_OK) std::cerr << "test device cleanup failed\n";
        std::error_code error;
        std::filesystem::remove(directory / "lss.json", error);
        std::filesystem::remove(directory, error);
    }
    uint32_t read(uint16_t index, unsigned expected_size) {
        std::array<uint8_t, 4> bytes{};
        uint32_t size = 0;
        CHECK(xnet_canopen_read_od(handle, index, 0, bytes.data(), bytes.size(), &size) == XNET_CO_OK);
        CHECK(size == expected_size);
        return modsim::little_endian(bytes.data(), size);
    }
};
modsim::ExchangeApi api() {
    return {xnet_canopen_bind_batch, xnet_canopen_read_batch, xnet_canopen_write_batch,
            xnet_canopen_stop, xnet_canopen_get_last_error};
}
void rpdo(uint16_t controlword, int32_t target = 0) {
    const uint32_t bits = static_cast<uint32_t>(target);
    fake::inject(0x20a, {static_cast<uint8_t>(controlword), static_cast<uint8_t>(controlword >> 8), 1,
        static_cast<uint8_t>(bits), static_cast<uint8_t>(bits >> 8),
        static_cast<uint8_t>(bits >> 16), static_cast<uint8_t>(bits >> 24)});
}
void enable(Device& device) {
    fake::inject(0, {1, 10});
    rpdo(0x06);
    wait_for([&] { return (device.read(0x6041, 2) & 0x6f) == 0x21; });
    rpdo(0x07);
    wait_for([&] { return (device.read(0x6041, 2) & 0x6f) == 0x23; });
    rpdo(0x0f);
    wait_for([&] { return (device.read(0x6041, 2) & 0x6f) == 0x27; });
}

void motion_and_reset() {
    Device device;
    modsim::Simulator simulator(api(), device.handle, {});
    CHECK(xnet_canopen_start(device.handle) == XNET_CO_OK);
    simulator.start();
    wait_for([&] { return simulator.snapshot().accepted > 2; });
    enable(device);
    rpdo(0x1f, 1000);
    wait_for([&] { return (device.read(0x6041, 2) & 0x1000) != 0; });
    rpdo(0x0f, 1000);
    wait_for([&] {
        return (device.read(0x6041, 2) & 0x400) && std::abs(static_cast<int32_t>(device.read(0x6064, 4)) - 1000) <= 5;
    });
    CHECK(device.read(0x607a, 4) == 1000);
    wait_for([&] {
        for (const auto& frame : fake::transmitted())
            if (frame.Identifier == 0x18a && frame.PayloadLength == 7
                && std::abs(static_cast<int32_t>(modsim::little_endian(frame.Payload + 3, 4)) - 1000) <= 5)
                return true;
        return false;
    });
    rpdo(0x1f, 10000);
    wait_for([&] { return simulator.snapshot().feedback.actual_velocity > 100; });
    rpdo(0x0b, 10000);
    wait_for([&] { return (device.read(0x6041, 2) & 0x6f) == 0x40; });
    CHECK(std::abs(simulator.snapshot().feedback.actual_velocity) <= 1);
    const auto epoch = simulator.snapshot().command.epoch;
    const auto accepted = simulator.snapshot().accepted;
    const auto position = device.read(0x6064, 4);
    fake::inject(0, {0x82, 10});
    wait_for([&] { return simulator.snapshot().command.epoch > epoch && simulator.snapshot().accepted > accepted + 2; });
    CHECK(device.read(0x607a, 4) == 10000);
    CHECK(device.read(0x6064, 4) == position);
    enable(device);
    rpdo(0x1f, 20000);
    wait_for([&] { return simulator.snapshot().feedback.actual_velocity > 100; });
    CHECK(simulator.stop());
    CHECK(!simulator.failed());
    xnet_canopen_status_t status{};
    CHECK(xnet_canopen_get_status(device.handle, &status) == XNET_CO_OK);
    CHECK(status.runtime_state == XNET_CO_RUNTIME_STOPPED);
    CHECK(fake::sessions() > 0);
    const auto stopped_position = device.read(0x6064, 4);
    const auto heartbeat = device.read(0x2f01, 4);
    CHECK(xnet_canopen_start(device.handle) == XNET_CO_OK);
    simulator.start();
    wait_for([&] { return simulator.snapshot().accepted > 2 && device.read(0x2f01, 4) > heartbeat; });
    CHECK(device.read(0x6064, 4) == stopped_position);
    const auto restarted_epoch = simulator.snapshot().command.epoch;
    fake::inject(0, {0x81, 10});
    wait_for([&] { return simulator.snapshot().command.epoch > restarted_epoch; });
    CHECK(device.read(0x607a, 4) == 0);
    CHECK(simulator.stop());
}

void faults(unsigned scenario) {
    Device device;
    modsim::Parameters parameters;
    const bool timeout = scenario != 0;
    if (scenario == 1) parameters.freeze_feedback_after_us = 150000;
    else if (scenario == 2) parameters.freeze_heartbeat_after_us = 150000;
    else {
        parameters.fault_code = 0x2310;
        parameters.fault_after_us = 150000;
        parameters.fault_duration_us = 250000;
    }
    modsim::Simulator simulator(api(), device.handle, parameters);
    CHECK(xnet_canopen_start(device.handle) == XNET_CO_OK);
    simulator.start();
    wait_for([&] { return simulator.snapshot().accepted > 2; });
    if (scenario == 1) {
        enable(device);
        rpdo(0x1f, 10000);
        wait_for([&] { return simulator.snapshot().feedback.actual_velocity > 100; });
    } else fake::inject(0, {1, 10});
    if (scenario == 1) {
        try {
            wait_for([&] {
                const auto state = simulator.snapshot();
                return device.read(0x603f, 2) == 0x8130
                    && state.command.action == CO_402_ACTION_FAULT_REACTION && !(state.command.flags & 1);
            });
        } catch (const std::exception&) {
            const auto state = simulator.snapshot();
            std::cerr << "Timeout snapshot: action=" << unsigned(state.command.action)
                      << " pds=" << unsigned(state.command.pds) << " flags=" << unsigned(state.command.flags)
                      << " accepted=" << state.accepted << " heartbeat=" << device.read(0x2f01, 4)
                      << " current_fault=" << device.read(0x2f00, 2) << " error=" << device.read(0x603f, 2) << '\n';
            throw;
        }
        const auto state = simulator.snapshot();
        CHECK(state.command.action == CO_402_ACTION_FAULT_REACTION && !(state.command.flags & 1));
        CHECK((device.read(0x6041, 2) & 0x6f) == 0x0f);
        const auto last_position = device.read(0x6064, 4);
        wait_for([&] { return (device.read(0x6041, 2) & 0x6f) == 8; });
        CHECK(device.read(0x6064, 4) == last_position);
        CHECK(device.read(0x603f, 2) == 0x8130);
    } else {
        wait_for([&] { return (device.read(0x6041, 2) & 0x6f) == 8; });
    }
    if (!timeout) CHECK(device.read(0x603f, 2) == 0x2310);
    if (scenario == 2) CHECK(device.read(0x603f, 2) == 0x8130);
    if (timeout) {
        CHECK(simulator.snapshot().feedback.fault_code == 0);
        CHECK(device.read(0x2f01, 4) > 0);
    } else {
        wait_for([&] { return device.read(0x2f00, 2) == 0; });
        CHECK(device.read(0x603f, 2) == 0x2310);
        rpdo(0x80);
        wait_for([&] { return (device.read(0x6041, 2) & 0x6f) == 0x40; });
        CHECK(device.read(0x603f, 2) == 0);
    }
    CHECK(simulator.stop());
}

void missing_feedback_during_teardown(bool reset) {
    Device device;
    modsim::Parameters parameters;
    parameters.freeze_feedback_after_us = 300000;
    modsim::Simulator simulator(api(), device.handle, parameters);
    CHECK(xnet_canopen_start(device.handle) == XNET_CO_OK);
    const auto begin = std::chrono::steady_clock::now();
    simulator.start();
    wait_for([&] { return simulator.snapshot().accepted > 2; });
    enable(device);
    rpdo(0x1f, 10000);
    wait_for([&] { return simulator.snapshot().feedback.actual_velocity > 100; });
    std::this_thread::sleep_until(begin + 350ms);
    const auto accepted = simulator.snapshot().accepted;
    std::this_thread::sleep_for(20ms);
    CHECK(simulator.snapshot().accepted == accepted);
    if (reset) {
        fake::inject(0, {0x82, 10});
        wait_for([&] {
            xnet_canopen_status_t status{};
            CHECK(xnet_canopen_get_status(device.handle, &status) == XNET_CO_OK);
            return status.reset_communication_count == 1;
        });
    }
    CHECK(simulator.stop());
    xnet_canopen_status_t status{};
    CHECK(xnet_canopen_get_status(device.handle, &status) == XNET_CO_OK);
    CHECK(status.runtime_state == XNET_CO_RUNTIME_STOPPED);
    int32_t code = 0;
    uint32_t required = 0;
    char diagnostic[2048]{};
    CHECK(xnet_canopen_get_last_error(device.handle, &code, diagnostic, sizeof(diagnostic), &required) == XNET_CO_OK);
    CHECK(code == XNET_CO_STACK_ERROR);
    CHECK(std::string(diagnostic).find("physical stop unconfirmed") != std::string::npos);
    if (!reset) {
        CHECK(device.read(0x603f, 2) == 0x8130);
        CHECK((device.read(0x6041, 2) & 0x6f) == 8);
    }
}
}

int main() {
    try {
        plant();
        motion_and_reset();
        faults(0);
        faults(1);
        faults(2);
        missing_feedback_during_teardown(false);
        missing_feedback_during_teardown(true);
        std::cout << "modsim plant, RPDO/TPDO, resets/restart, fault/recovery and missing-feedback teardown passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
