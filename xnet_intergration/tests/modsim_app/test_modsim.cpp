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
constexpr double radians_per_count = 6.2831853071795864769 / 10000;
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
    motor_drive_view_v1 command{};
    command.token.epoch = command.token.command_seq = command.token.action_seq = 1;
    command.mode = 1;
    command.flags = 1;
    command.demand_position_rad = 1000 * radians_per_count;
    auto feedback = motor.step(command, 0);
    CHECK(feedback.actual_position_rad > 0 && feedback.actual_position_rad < command.demand_position_rad);
    for (int i = 0; i < 1000; ++i) feedback = motor.step(command, i * parameters.period_us);
    CHECK(std::abs(feedback.actual_position_rad / radians_per_count - 1000) <= 1);
    command.demand_position_rad = -1000 * radians_per_count;
    for (int i = 0; i < 1000; ++i) feedback = motor.step(command, i * parameters.period_us);
    CHECK(std::abs(feedback.actual_position_rad / radians_per_count + 1000) <= 1);
    command.demand_position_rad = INT32_MAX * radians_per_count;
    for (int i = 0; i < 3000; ++i) feedback = motor.step(command, i * parameters.period_us);
    CHECK(feedback.actual_position_rad == parameters.max_position && feedback.limit_flags);
    command.token.epoch = 2;
    command.flags = 0;
    command.action = MOTOR_DRIVE_ACTION_ENABLE_OPERATION;
    feedback = motor.step(command, 0);
    CHECK(feedback.action_result == MOTOR_DRIVE_RESULT_BUSY);
    command.action = MOTOR_DRIVE_ACTION_DISABLE_VOLTAGE;
    ++command.token.action_seq;
    feedback = motor.step(command, 0);
    CHECK(feedback.action_result == MOTOR_DRIVE_RESULT_BUSY);
    CHECK(feedback.actual_position_rad == parameters.max_position);
    for (int i = 0; i < 10; ++i) feedback = motor.step(command, 0);
    CHECK(feedback.action_result == MOTOR_DRIVE_RESULT_DONE);

    parameters.initial_position = INT32_MIN * radians_per_count;
    parameters.min_position = parameters.initial_position;
    parameters.max_position = INT32_MAX * radians_per_count;
    parameters.fault_class = MOTOR_DRIVE_FAULT_OVERCURRENT;
    parameters.fault_detail = 7;
    parameters.fault_after_us = 20000;
    parameters.fault_duration_us = 20000;
    parameters.freeze_feedback_after_us = 10000;
    parameters.error_action = MOTOR_DRIVE_ACTION_SWITCH_ON;
    modsim::Motor faulty(parameters);
    command.action = MOTOR_DRIVE_ACTION_NONE;
    feedback = faulty.step(command, 20000);
    CHECK(feedback.actual_position_rad == parameters.initial_position
        && feedback.fault_class == MOTOR_DRIVE_FAULT_OVERCURRENT && !(feedback.flags & 0x10));
    feedback = faulty.step(command, 40000);
    CHECK(feedback.fault_class == 0 && (feedback.flags & 0x10));
    command.action = MOTOR_DRIVE_ACTION_SWITCH_ON;
    feedback = faulty.step(command, 40000);
    CHECK(feedback.action_result == MOTOR_DRIVE_RESULT_ERROR);
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
    return {xnet_canopen_get_motor_drive_endpoint, motor_drive_open_v1, motor_drive_read_v1,
            motor_drive_submit_v1, motor_drive_heartbeat_v1, motor_drive_close_v1,
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
    wait_for([&] { return simulator.snapshot().feedback.actual_velocity_rad_s > 100 * radians_per_count; });
    rpdo(0x0b, 10000);
    wait_for([&] { return (device.read(0x6041, 2) & 0x6f) == 0x40; });
    CHECK(std::abs(simulator.snapshot().feedback.actual_velocity_rad_s) <= radians_per_count);
    const auto epoch = simulator.snapshot().command.token.epoch;
    const auto accepted = simulator.snapshot().accepted;
    const auto position = device.read(0x6064, 4);
    fake::inject(0, {0x82, 10});
    wait_for([&] { return simulator.snapshot().command.token.epoch > epoch && simulator.snapshot().accepted > accepted + 2; });
    CHECK(device.read(0x607a, 4) == 10000);
    CHECK(device.read(0x6064, 4) == position);
    enable(device);
    rpdo(0x1f, 20000);
    wait_for([&] { return simulator.snapshot().feedback.actual_velocity_rad_s > 100 * radians_per_count; });
    CHECK(simulator.stop());
    CHECK(!simulator.failed());
    xnet_canopen_status_t status{};
    CHECK(xnet_canopen_get_status(device.handle, &status) == XNET_CO_OK);
    CHECK(status.runtime_state == XNET_CO_RUNTIME_STOPPED);
    CHECK(fake::sessions() > 0);
    const auto stopped_position = device.read(0x6064, 4);
    const auto heartbeat = simulator.snapshot().heartbeat_seq;
    CHECK(xnet_canopen_start(device.handle) == XNET_CO_OK);
    simulator.start();
    wait_for([&] { return simulator.snapshot().accepted > 2 && simulator.snapshot().heartbeat_seq > heartbeat; });
    CHECK(device.read(0x6064, 4) == stopped_position);
    const auto restarted_epoch = simulator.snapshot().command.token.epoch;
    fake::inject(0, {0x81, 10});
    wait_for([&] { return simulator.snapshot().command.token.epoch > restarted_epoch; });
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
        parameters.fault_class = MOTOR_DRIVE_FAULT_OVERCURRENT;
        parameters.fault_detail = 7;
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
        wait_for([&] { return simulator.snapshot().feedback.actual_velocity_rad_s > 100 * radians_per_count; });
    } else fake::inject(0, {1, 10});
    if (scenario == 1) {
        try {
            wait_for([&] {
                const auto state = simulator.snapshot();
                return device.read(0x603f, 2) == 0x8130
                    && state.command.action == MOTOR_DRIVE_ACTION_FAULT_REACTION && !(state.command.flags & 1);
            });
        } catch (const std::exception&) {
            const auto state = simulator.snapshot();
            std::cerr << "Timeout snapshot: action=" << unsigned(state.command.action)
                      << " flags=" << state.command.flags
                      << " accepted=" << state.accepted << " heartbeat=" << state.heartbeat_seq
                      << " fault_class=" << state.feedback.fault_class << " error=" << device.read(0x603f, 2) << '\n';
            throw;
        }
        const auto state = simulator.snapshot();
        CHECK(state.command.action == MOTOR_DRIVE_ACTION_FAULT_REACTION && !(state.command.flags & 1));
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
        CHECK(simulator.snapshot().feedback.fault_class == 0);
        CHECK(simulator.snapshot().heartbeat_seq > 0);
    } else {
        wait_for([&] {
            const auto state = simulator.snapshot();
            return state.feedback.fault_class == 0 && state.accepted > 2;
        });
        const auto cleared = simulator.snapshot().accepted;
        wait_for([&] { return simulator.snapshot().accepted > cleared; });
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
    wait_for([&] { return simulator.snapshot().feedback.actual_velocity_rad_s > 100 * radians_per_count; });
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
