/**
 * @file Bounded discrete motor/drive response used only for software validation.
 * @copyright 2026 CANopenNode contributors
 */
#include "motor.hpp"
#include "nlohmann/json.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace modsim {
namespace {
constexpr double stopped_velocity_rad_s = 0.0003141592653589793;
}

void Parameters::validate() const {
    const auto positive = [](double value) { return std::isfinite(value) && value > 0; };
    if (period_us < 100 || period_us > 10000 || !positive(position_gain)
        || !positive(velocity_time_constant_s) || velocity_time_constant_s < period_us / 1e6
        || !positive(max_velocity)
        || !positive(max_acceleration) || !positive(stop_deceleration)
        || !std::isfinite(min_position) || !std::isfinite(max_position) || !std::isfinite(initial_position)
        || min_position >= max_position || initial_position < min_position || initial_position > max_position
        || error_action > MOTOR_DRIVE_ACTION_DISABLE_VOLTAGE
        || (!fault_class && (fault_detail || fault_after_us || fault_duration_us)))
        throw std::runtime_error("invalid motor parameters: check period, dynamics, limits and fault injection");
}

Parameters read_parameters(const std::filesystem::path& path) {
    Parameters result;
    if (path.empty()) return result;
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open motor parameters: " + path.string());
    const auto json = nlohmann::json::parse(input);
    if (!json.is_object()) throw std::runtime_error("motor parameters must be a JSON object");
    const auto unsigned_value = [](const nlohmann::json& value, uint64_t maximum) {
        if ((!value.is_number_unsigned() && !value.is_number_integer())
            || (value.is_number_integer() && !value.is_number_unsigned() && value.get<int64_t>() < 0)
            || value.get<uint64_t>() > maximum)
            throw std::runtime_error("motor integer is outside its unsigned range");
        return value.get<uint64_t>();
    };
    for (const auto& item : json.items()) {
        const auto& key = item.key();
        const auto& value = item.value();
#define READ_U32(field) if (key == #field) { result.field = static_cast<uint32_t>(unsigned_value(value, UINT32_MAX)); continue; }
        READ_U32(period_us)
        READ_U32(action_delay_us)
        READ_U32(fault_after_us)
        READ_U32(fault_duration_us)
        READ_U32(freeze_feedback_after_us)
        READ_U32(freeze_heartbeat_after_us)
        READ_U32(fault_class)
        READ_U32(fault_detail)
#undef READ_U32
#define READ_DOUBLE(field) if (key == #field) { if (!value.is_number()) throw std::runtime_error(key + " must be numeric"); result.field = value.get<double>(); continue; }
        READ_DOUBLE(position_gain)
        READ_DOUBLE(velocity_time_constant_s)
        READ_DOUBLE(max_velocity)
        READ_DOUBLE(max_acceleration)
        READ_DOUBLE(stop_deceleration)
        READ_DOUBLE(min_position)
        READ_DOUBLE(max_position)
        READ_DOUBLE(initial_position)
#undef READ_DOUBLE
        if (key == "error_action") result.error_action = static_cast<uint8_t>(unsigned_value(value, 8));
        else throw std::runtime_error("unknown motor parameter: " + key);
    }
    result.validate();
    return result;
}

Motor::Motor(Parameters parameters) : parameters_(parameters), position_(parameters.initial_position) {
    parameters_.validate();
}

bool Motor::feedback_enabled(uint64_t elapsed_us) const {
    return !parameters_.freeze_feedback_after_us || elapsed_us < parameters_.freeze_feedback_after_us;
}
bool Motor::heartbeat_enabled(uint64_t elapsed_us) const {
    return !parameters_.freeze_heartbeat_after_us || elapsed_us < parameters_.freeze_heartbeat_after_us;
}

motor_drive_feedback_v1 Motor::step(const motor_drive_view_v1& command, uint64_t elapsed_us) {
    if (epoch_ != command.token.epoch || action_seq_ != command.token.action_seq) {
        epoch_ = command.token.epoch;
        action_seq_ = command.token.action_seq;
        action_age_us_ = 0;
    }
    action_age_us_ += parameters_.period_us;
    const bool fault = parameters_.fault_class && elapsed_us >= parameters_.fault_after_us
        && (!parameters_.fault_duration_us
            || elapsed_us - parameters_.fault_after_us < parameters_.fault_duration_us);
    const bool action_error = command.action != MOTOR_DRIVE_ACTION_NONE && command.action == parameters_.error_action;
    const bool output = (command.flags & 1U) && command.mode == 1
        && command.action == MOTOR_DRIVE_ACTION_NONE && !fault;
    const double dt = parameters_.period_us / 1e6;
    double acceleration;
    uint32_t limit_flags = 0;
    if (output) {
        const double requested_velocity = command.demand_velocity_rad_s
            + parameters_.position_gain * (command.demand_position_rad - position_);
        const double requested_acceleration = (requested_velocity - velocity_) / parameters_.velocity_time_constant_s;
        const double bounded_velocity = std::clamp(requested_velocity, -parameters_.max_velocity, parameters_.max_velocity);
        acceleration = std::clamp((bounded_velocity - velocity_) / parameters_.velocity_time_constant_s,
                                  -parameters_.max_acceleration, parameters_.max_acceleration);
        if (requested_velocity != bounded_velocity || std::abs(requested_acceleration) > parameters_.max_acceleration)
            limit_flags |= MOTOR_DRIVE_LIMIT_VELOCITY;
    } else {
        const double deceleration = command.action == MOTOR_DRIVE_ACTION_QUICK_STOP
            && (command.flags & MOTOR_DRIVE_SAFETY_DECELERATION_VALID)
            ? std::min(command.safety_deceleration_rad_s2, parameters_.max_acceleration)
            : parameters_.stop_deceleration;
        acceleration = std::clamp(-velocity_ / dt, -deceleration, deceleration);
    }
    velocity_ = std::clamp(velocity_ + acceleration * dt, -parameters_.max_velocity, parameters_.max_velocity);
    const double next_position = position_ + velocity_ * dt;
    position_ = std::clamp(next_position, double(parameters_.min_position), double(parameters_.max_position));
    if (position_ != next_position) {
        velocity_ = 0;
        limit_flags |= MOTOR_DRIVE_LIMIT_POSITION;
    }
    motor_drive_feedback_v1 feedback{};
    feedback.abi_version = MOTOR_DRIVE_ABI_VERSION;
    feedback.struct_size = sizeof(feedback);
    feedback.source = command.token;
    feedback.actual_position_rad = position_;
    feedback.actual_velocity_rad_s = velocity_;
    feedback.fault_class = fault ? parameters_.fault_class : 0;
    feedback.fault_detail = fault ? parameters_.fault_detail : 0;
    feedback.flags = MOTOR_DRIVE_POSITION_VALID | MOTOR_DRIVE_VELOCITY_VALID
        | (!fault ? MOTOR_DRIVE_FEEDBACK_PP_READY : 0)
        | (!fault && std::abs(velocity_) < stopped_velocity_rad_s ? MOTOR_DRIVE_RESET_PERMITTED : 0);
    feedback.limit_flags = limit_flags;
    if (command.action != MOTOR_DRIVE_ACTION_NONE) {
        const bool stopping = command.action == MOTOR_DRIVE_ACTION_QUICK_STOP
            || command.action == MOTOR_DRIVE_ACTION_FAULT_REACTION || command.action == MOTOR_DRIVE_ACTION_DISABLE_VOLTAGE
            || command.action == MOTOR_DRIVE_ACTION_DISABLE_OPERATION || command.action == MOTOR_DRIVE_ACTION_SHUTDOWN;
        const bool done = action_age_us_ >= parameters_.action_delay_us
            && (!stopping || std::abs(velocity_) < stopped_velocity_rad_s)
            && (command.action != MOTOR_DRIVE_ACTION_FAULT_RESET || !fault);
        feedback.action_result = action_error ? MOTOR_DRIVE_RESULT_ERROR
            : done ? MOTOR_DRIVE_RESULT_DONE : MOTOR_DRIVE_RESULT_BUSY;
    }
    return feedback;
}
}
