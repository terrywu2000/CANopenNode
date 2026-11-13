#pragma once

/**
 * @file Slave-side PP tracking plant, without a PDS or profile generator.
 * @copyright 2026 CANopenNode contributors
 */
#include "402/CO_402_protocol.h"
#include <cstdint>
#include <filesystem>

namespace modsim {
struct Parameters {
    uint32_t period_us = 2000;
    uint32_t action_delay_us = 10000;
    double position_gain = 40;
    double velocity_time_constant_s = 0.01;
    double max_velocity = 200000;
    double max_acceleration = 2000000;
    double stop_deceleration = 1000000;
    int32_t min_position = -1000000;
    int32_t max_position = 1000000;
    int32_t initial_position = 0;
    uint16_t fault_code = 0;
    uint32_t fault_after_us = 0;
    uint32_t fault_duration_us = 0;
    uint32_t freeze_feedback_after_us = 0;
    uint32_t freeze_heartbeat_after_us = 0;
    uint8_t error_action = CO_402_ACTION_NONE;

    void validate() const;
};

Parameters read_parameters(const std::filesystem::path& path);

class Motor {
public:
    explicit Motor(Parameters parameters);
    CO_402_feedback_frame_t step(const CO_402_command_frame_t& command, uint64_t elapsed_us);
    bool feedback_enabled(uint64_t elapsed_us) const;
    bool heartbeat_enabled(uint64_t elapsed_us) const;
private:
    Parameters parameters_;
    double position_, velocity_ = 0;
    uint64_t epoch_ = 0, action_age_us_ = 0;
    uint32_t action_seq_ = 0;
};
}
