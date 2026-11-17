#pragma once

/**
 * @file Slave-side PP tracking plant, without a PDS or profile generator.
 * @copyright 2026 CANopenNode contributors
 */
#include "motor_drive.h"
#include <cstdint>
#include <filesystem>

namespace modsim {
struct Parameters {
    uint32_t period_us = 2000;
    uint32_t action_delay_us = 10000;
    double position_gain = 40;
    double velocity_time_constant_s = 0.01;
    double max_velocity = 125.66370614359172;
    double max_acceleration = 1256.6370614359173;
    double stop_deceleration = 628.3185307179587;
    double min_position = -628.3185307179587;
    double max_position = 628.3185307179587;
    double initial_position = 0;
    uint32_t fault_class = 0;
    uint32_t fault_detail = 0;
    uint32_t fault_after_us = 0;
    uint32_t fault_duration_us = 0;
    uint32_t freeze_feedback_after_us = 0;
    uint32_t freeze_heartbeat_after_us = 0;
    uint8_t error_action = MOTOR_DRIVE_ACTION_NONE;

    void validate() const;
};

Parameters read_parameters(const std::filesystem::path& path);

class Motor {
public:
    explicit Motor(Parameters parameters);
    motor_drive_feedback_v1 step(const motor_drive_view_v1& command, uint64_t elapsed_us);
    bool feedback_enabled(uint64_t elapsed_us) const;
    bool heartbeat_enabled(uint64_t elapsed_us) const;
private:
    Parameters parameters_;
    double position_, velocity_ = 0;
    uint64_t epoch_ = 0, action_age_us_ = 0;
    uint32_t action_seq_ = 0;
};
}
