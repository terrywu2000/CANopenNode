#pragma once

/**
 * @file Single-writer external model exchange and thread lifecycle.
 * @copyright 2026 CANopenNode contributors
 */
#include "motor.hpp"
#include "xnet_canopen.h"
#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace modsim {
struct ExchangeApi {
    decltype(&xnet_canopen_get_motor_drive_endpoint) endpoint;
    decltype(&motor_drive_open_v1) open;
    decltype(&motor_drive_read_v1) read;
    decltype(&motor_drive_submit_v1) submit;
    decltype(&motor_drive_heartbeat_v1) heartbeat;
    decltype(&motor_drive_close_v1) close;
    decltype(&xnet_canopen_stop) stop;
    decltype(&xnet_canopen_get_last_error) error;
};

inline uint32_t next_sequence(uint32_t value) {
    ++value;
    return value ? value : 1;
}
inline uint32_t little_endian(const uint8_t* bytes, unsigned size) {
    uint32_t value = 0;
    for (unsigned i = 0; i < size; ++i) value |= uint32_t(bytes[i]) << (8 * i);
    return value;
}

struct Snapshot {
    motor_drive_view_v1 command{};
    motor_drive_feedback_v1 feedback{};
    uint64_t accepted = 0, rejected = 0, overruns = 0, epochs = 0;
    uint32_t pending_seq = 0;
    uint16_t processed_result = 0;
    uint32_t heartbeat_seq = 0;
};

class Simulator {
public:
    Simulator(ExchangeApi api, xnet_canopen_handle handle, Parameters parameters)
        : api_(api), handle_(handle), parameters_(parameters), motor_(parameters) {
        motor_drive_endpoint_handle endpoint = nullptr;
        check(api_.endpoint(handle_, &endpoint), "get motor endpoint");
        check(api_.open(endpoint, &port_), "open motor port");
    }
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;
    ~Simulator() {
        stop();
        if (api_.close(port_) != MOTOR_DRIVE_OK) std::cerr << "Cannot close motor port\n";
    }

    void start() {
        if (thread_.joinable()) throw std::runtime_error("model thread already started");
        if (failed_) throw std::runtime_error("failed model requires a new simulator instance");
        running_ = true;
        thread_ = std::thread([this] {
            try { loop(); }
            catch (const std::exception& error) {
                std::cerr << "Model thread failed: " << error.what() << '\n';
                failed_ = true;
                running_ = false;
            }
        });
    }
    bool stop() {
        if (!thread_.joinable()) return !failed_;
        // Keep answering the new-epoch safety action until native stop has finished.
        const auto result = api_.stop(handle_);
        if (result != XNET_CO_OK) {
            std::cerr << diagnostic("xnet_canopen_stop", result) << '\n';
            failed_ = true;
        }
        running_ = false;
        thread_.join();
        return !failed_;
    }
    bool failed() const { return failed_; }
    Snapshot snapshot() const {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return snapshot_;
    }

private:
    using Clock = std::chrono::steady_clock;
    std::string diagnostic(const char* operation, int32_t result) const {
        int32_t code = 0;
        uint32_t required = 0;
        char text[2048]{};
        const auto error_result = api_.error(handle_, &code, text, sizeof(text), &required);
        return std::string(operation) + " returned " + std::to_string(result)
            + "; diagnostic result=" + std::to_string(error_result) + " code=" + std::to_string(code) + " " + text;
    }
    void check(int32_t result, const char* operation) const {
        if (result != XNET_CO_OK) throw std::runtime_error(diagnostic(operation, result));
    }
    int32_t read_view(motor_drive_view_v1& view) const {
        motor_drive_read_request_v1 request{};
        request.abi_version = MOTOR_DRIVE_ABI_VERSION;
        request.struct_size = sizeof(request);
        request.flags = MOTOR_DRIVE_READ_ASYNC;
        const auto result = api_.read(port_, &request, &view, sizeof(view));
        if (result != MOTOR_DRIVE_NO_DATA) check(result, "read motor view");
        return result;
    }
    void loop() {
        const auto begin = Clock::now();
        const auto period = std::chrono::microseconds(parameters_.period_us);
        auto next = begin;
        uint64_t epoch = 0, step = 0;
        uint32_t feedback_seq = 0, pending_seq = 0;
        Snapshot state{};
        while (running_) {
            motor_drive_view_v1 command{};
            if (read_view(command) == MOTOR_DRIVE_NO_DATA) {
                std::this_thread::sleep_for(period);
                continue;
            }
            if (epoch != command.token.epoch) {
                epoch = command.token.epoch;
                pending_seq = feedback_seq = 0;
                ++state.epochs;
            }
            state.processed_result = static_cast<uint16_t>(command.processed_result);
            if (pending_seq && command.processed_feedback_seq == pending_seq) {
                if (state.processed_result == 1) ++state.accepted;
                else if (state.processed_result == 2 || state.processed_result == 3) {
                    ++state.rejected;
                    std::cerr << "Feedback rejected: seq=" << pending_seq << " result=" << state.processed_result << '\n';
                } else throw std::runtime_error("unknown feedback processing result");
                pending_seq = 0;
            }
            const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - begin).count();
            auto candidate = motor_;
            auto feedback = candidate.step(command, static_cast<uint64_t>(elapsed));
            motor_drive_view_v1 latest{};
            const auto latest_result = read_view(latest);
            if (latest_result == MOTOR_DRIVE_NO_DATA) continue;
            if (latest.token.epoch != command.token.epoch || latest.token.action_seq != command.token.action_seq
                || latest.flags != command.flags || latest.mode != command.mode || latest.action != command.action) {
                if (epoch != latest.token.epoch) {
                    epoch = latest.token.epoch;
                    pending_seq = feedback_seq = 0;
                    ++state.epochs;
                }
                command = latest;
                candidate = motor_;
                feedback = candidate.step(command, static_cast<uint64_t>(elapsed));
            }
            // Revoke a superseded calculation before changing the simulated plant.
            motor_ = candidate;
            ++step;
            feedback.executed_step = {1, step, step, step * parameters_.period_us / 1e6,
                parameters_.period_us / 1e6, {}};
            if (motor_.heartbeat_enabled(elapsed)) {
                heartbeat_ = next_sequence(heartbeat_);
                const auto result = api_.heartbeat(port_, &command.token, &feedback.executed_step, heartbeat_);
                if (result != MOTOR_DRIVE_STALE) check(result, "motor heartbeat");
                state.heartbeat_seq = heartbeat_;
            }
            if (!pending_seq && motor_.feedback_enabled(elapsed)) {
                feedback.feedback_seq = feedback_seq = next_sequence(feedback_seq);
                const auto result = api_.submit(port_, &feedback, sizeof(feedback));
                if (result == MOTOR_DRIVE_OK)
                    pending_seq = feedback.feedback_seq;
                else if (result == MOTOR_DRIVE_STALE) {
                    ++state.rejected;
                    std::cerr << diagnostic("motor submit", result) << "; will retry from a fresh command\n";
                } else check(result, "motor submit");
            }
            state.command = command;
            state.feedback = feedback;
            state.pending_seq = pending_seq;
            next += period;
            if (next <= Clock::now()) {
                ++state.overruns;
                next = Clock::now() + period;
            }
            {
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                snapshot_ = state;
            }
            std::this_thread::sleep_until(next);
        }
    }

    ExchangeApi api_;
    xnet_canopen_handle handle_;
    Parameters parameters_;
    Motor motor_;
    motor_drive_port_handle port_ = nullptr;
    uint32_t heartbeat_ = 0;
    std::atomic<bool> running_{false}, failed_{false};
    std::thread thread_;
    mutable std::mutex snapshot_mutex_;
    Snapshot snapshot_{};
};
}
