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
    decltype(&xnet_canopen_bind_batch) bind;
    decltype(&xnet_canopen_read_batch) read;
    decltype(&xnet_canopen_write_batch) write;
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
    CO_402_command_frame_t command{};
    CO_402_feedback_frame_t feedback{};
    uint64_t accepted = 0, rejected = 0, overruns = 0, epochs = 0;
    uint32_t pending_seq = 0;
    uint16_t processed_result = 0;
};

class Simulator {
public:
    Simulator(ExchangeApi api, xnet_canopen_handle handle, Parameters parameters)
        : api_(api), handle_(handle), parameters_(parameters), motor_(parameters) {
        const std::array<uint16_t, 3> indices{0x2f02, 0x2f04, 0x2f05};
        const std::array<uint8_t, 3> subs{};
        std::array<uint32_t, 3> types{}, sizes{};
        uint32_t total = 0;
        check(api_.bind(handle_, indices.data(), subs.data(), 3, types.data(), sizes.data(), &total, &command_id_),
              "bind command_view");
        if (types != std::array<uint32_t, 3>{XNET_CO_TYPE_OCTET_STRING, XNET_CO_TYPE_UNSIGNED32, XNET_CO_TYPE_UNSIGNED16}
            || sizes != std::array<uint32_t, 3>{56, 4, 2} || total != command_size)
            throw std::runtime_error("command_view binding does not match protocol v1");
        feedback_id_ = bind_one(0x2f03, XNET_CO_TYPE_OCTET_STRING, CO_402_FEEDBACK_FRAME_SIZE);
        heartbeat_id_ = bind_one(0x2f01, XNET_CO_TYPE_UNSIGNED32, 4);
    }
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;
    ~Simulator() { stop(); }

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
    static constexpr uint32_t command_size = CO_402_COMMAND_FRAME_SIZE + 6;
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
    uint32_t bind_one(uint16_t index, uint32_t expected_type, uint32_t expected_size) {
        uint8_t sub = 0;
        uint32_t type = 0, size = 0, total = 0, id = 0;
        check(api_.bind(handle_, &index, &sub, 1, &type, &size, &total, &id), "bind model output");
        if (type != expected_type || size != expected_size || total != expected_size)
            throw std::runtime_error("model output binding does not match protocol v1");
        return id;
    }
    template<size_t N>
    int32_t write(uint32_t id, const std::array<uint8_t, N>& bytes, const char* operation, bool recoverable = false) {
        int32_t entry = XNET_CO_INTERNAL_ERROR;
        const auto result = api_.write(handle_, id, bytes.data(), N, &entry, 1);
        if (result == XNET_CO_OK && entry == XNET_CO_OK) return XNET_CO_OK;
        if (recoverable && result == XNET_CO_PARTIAL_FAILURE && entry == XNET_CO_ACCESS_ERROR) {
            std::cerr << diagnostic(operation, entry) << "; will retry from a fresh command\n";
            return entry;
        }
        check(result, operation);
        check(entry, operation);
        return XNET_CO_OK;
    }
    CO_402_command_frame_t read_view(std::array<uint8_t, command_size>& bytes) const {
        std::array<int32_t, 3> entries{};
        check(api_.read(handle_, command_id_, bytes.data(), bytes.size(), entries.data(), entries.size()), "read command_view");
        for (auto entry : entries) check(entry, "read command_view entry");
        CO_402_command_frame_t command{};
        if (!CO_402_command_unpack(&command, bytes.data(), CO_402_COMMAND_FRAME_SIZE))
            throw std::runtime_error("invalid command frame (requires an enabled external PP model)");
        return command;
    }
    void loop() {
        const auto begin = Clock::now();
        const auto period = std::chrono::microseconds(parameters_.period_us);
        auto next = begin;
        uint64_t epoch = 0;
        uint32_t feedback_seq = 0, pending_seq = 0;
        Snapshot state{};
        while (running_) {
            std::array<uint8_t, command_size> bytes{};
            auto command = read_view(bytes);
            if (epoch != command.epoch) {
                epoch = command.epoch;
                pending_seq = feedback_seq = 0;
                ++state.epochs;
            }
            state.processed_result = static_cast<uint16_t>(little_endian(bytes.data() + 60, 2));
            if (pending_seq && little_endian(bytes.data() + 56, 4) == pending_seq) {
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
            const auto latest = read_view(bytes);
            if (latest.epoch != command.epoch || latest.action_seq != command.action_seq
                || latest.flags != command.flags || latest.mode != command.mode || latest.action != command.action) {
                if (epoch != latest.epoch) {
                    epoch = latest.epoch;
                    pending_seq = feedback_seq = 0;
                    ++state.epochs;
                }
                command = latest;
                candidate = motor_;
                feedback = candidate.step(command, static_cast<uint64_t>(elapsed));
            }
            // Revoke a superseded calculation before changing the simulated plant.
            motor_ = candidate;
            if (motor_.heartbeat_enabled(elapsed)) {
                heartbeat_ = next_sequence(heartbeat_);
                std::array<uint8_t, 4> alive{};
                for (unsigned i = 0; i < alive.size(); ++i) alive[i] = static_cast<uint8_t>(heartbeat_ >> (8 * i));
                write(heartbeat_id_, alive, "write model_alive");
            }
            if (!pending_seq && motor_.feedback_enabled(elapsed)) {
                feedback.feedback_seq = feedback_seq = next_sequence(feedback_seq);
                std::array<uint8_t, CO_402_FEEDBACK_FRAME_SIZE> submission{};
                if (!CO_402_feedback_pack(&feedback, submission.data(), submission.size()))
                    throw std::runtime_error("cannot encode model feedback");
                if (write(feedback_id_, submission, "write feedback_submit", true) == XNET_CO_OK)
                    pending_seq = feedback.feedback_seq;
                else ++state.rejected;
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
    uint32_t command_id_ = 0, feedback_id_ = 0, heartbeat_id_ = 0;
    uint32_t heartbeat_ = 0;
    std::atomic<bool> running_{false}, failed_{false};
    std::thread thread_;
    mutable std::mutex snapshot_mutex_;
    Snapshot snapshot_{};
};
}
