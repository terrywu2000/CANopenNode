/**
 * @file Bounded discrete motor/drive response used only for software validation.
 * @copyright 2026 CANopenNode contributors
 */
#include "motor.hpp"
#include "nlohmann/json.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace modsim {
void Parameters::validate() const {
    const auto positive = [](double value) { return std::isfinite(value) && value > 0; };
    if (period_us < 100 || period_us > 10000 || !positive(position_gain)
        || !positive(velocity_time_constant_s) || velocity_time_constant_s < period_us / 1e6
        || !positive(max_velocity) || max_velocity > std::numeric_limits<int32_t>::max()
        || !positive(max_acceleration) || !positive(stop_deceleration)
        || min_position >= max_position || initial_position < min_position || initial_position > max_position
        || error_action > CO_402_ACTION_DISABLE_VOLTAGE
        || (!fault_code && (fault_after_us || fault_duration_us)))
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
    const auto signed_value = [](const nlohmann::json& value) {
        if ((!value.is_number_integer() && !value.is_number_unsigned())
            || (value.is_number_unsigned() && value.get<uint64_t>() > uint64_t(INT32_MAX))
            || (!value.is_number_unsigned() && (value.get<int64_t>() < INT32_MIN || value.get<int64_t>() > INT32_MAX)))
            throw std::runtime_error("motor position is outside INTEGER32 range");
        return value.get<int32_t>();
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
#undef READ_U32
#define READ_DOUBLE(field) if (key == #field) { if (!value.is_number()) throw std::runtime_error(key + " must be numeric"); result.field = value.get<double>(); continue; }
        READ_DOUBLE(position_gain)
        READ_DOUBLE(velocity_time_constant_s)
        READ_DOUBLE(max_velocity)
        READ_DOUBLE(max_acceleration)
        READ_DOUBLE(stop_deceleration)
#undef READ_DOUBLE
        if (key == "min_position") result.min_position = signed_value(value);
        else if (key == "max_position") result.max_position = signed_value(value);
        else if (key == "initial_position") result.initial_position = signed_value(value);
        else if (key == "fault_code") result.fault_code = static_cast<uint16_t>(unsigned_value(value, UINT16_MAX));
        else if (key == "error_action") result.error_action = static_cast<uint8_t>(unsigned_value(value, 8));
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

CO_402_feedback_frame_t Motor::step(const CO_402_command_frame_t& command, uint64_t elapsed_us) {
    if (epoch_ != command.epoch || action_seq_ != command.action_seq) {
        epoch_ = command.epoch;
        action_seq_ = command.action_seq;
        action_age_us_ = 0;
    }
    action_age_us_ += parameters_.period_us;
    const bool fault = parameters_.fault_code && elapsed_us >= parameters_.fault_after_us
        && (!parameters_.fault_duration_us
            || elapsed_us - parameters_.fault_after_us < parameters_.fault_duration_us);
    const bool action_error = command.action != CO_402_ACTION_NONE && command.action == parameters_.error_action;
    const bool output = (command.flags & 1U) && command.mode == 1
        && command.action == CO_402_ACTION_NONE && !fault;
    const double dt = parameters_.period_us / 1e6;
    double acceleration;
    bool limited = false;
    if (output) {
        const double requested_velocity = command.demand_velocity
            + parameters_.position_gain * (command.demand_position - position_);
        const double requested_acceleration = (requested_velocity - velocity_) / parameters_.velocity_time_constant_s;
        const double bounded_velocity = std::clamp(requested_velocity, -parameters_.max_velocity, parameters_.max_velocity);
        acceleration = std::clamp((bounded_velocity - velocity_) / parameters_.velocity_time_constant_s,
                                  -parameters_.max_acceleration, parameters_.max_acceleration);
        limited = requested_velocity != bounded_velocity || std::abs(requested_acceleration) > parameters_.max_acceleration;
    } else {
        const double deceleration = command.action == CO_402_ACTION_QUICK_STOP && command.quick_stop_deceleration
            ? std::min(double(command.quick_stop_deceleration), parameters_.max_acceleration)
            : parameters_.stop_deceleration;
        acceleration = std::clamp(-velocity_ / dt, -deceleration, deceleration);
    }
    velocity_ = std::clamp(velocity_ + acceleration * dt, -parameters_.max_velocity, parameters_.max_velocity);
    const double next_position = position_ + velocity_ * dt;
    position_ = std::clamp(next_position, double(parameters_.min_position), double(parameters_.max_position));
    if (position_ != next_position) {
        velocity_ = 0;
        limited = true;
    }
    CO_402_feedback_frame_t feedback{};
    feedback.epoch = command.epoch;
    feedback.command_seq = command.command_seq;
    feedback.action_seq = command.action_seq;
    feedback.actual_position = static_cast<int32_t>(std::round(position_));
    feedback.actual_velocity = static_cast<int32_t>(std::round(velocity_));
    feedback.fault_code = fault ? parameters_.fault_code : 0;
    feedback.flags = static_cast<uint8_t>(0x03U | (!fault ? 0x04U : 0U) | (limited ? 0x08U : 0U)
        | (!fault && std::abs(velocity_) < 0.5 ? 0x10U : 0U));
    if (command.action != CO_402_ACTION_NONE) {
        const bool stopping = command.action == CO_402_ACTION_QUICK_STOP
            || command.action == CO_402_ACTION_FAULT_REACTION || command.action == CO_402_ACTION_DISABLE_VOLTAGE
            || command.action == CO_402_ACTION_DISABLE_OPERATION || command.action == CO_402_ACTION_SHUTDOWN;
        const bool done = action_age_us_ >= parameters_.action_delay_us
            && (!stopping || std::abs(velocity_) < 0.5) && (command.action != CO_402_ACTION_FAULT_RESET || !fault);
        feedback.action_result = action_error ? CO_402_ACTION_RESULT_ERROR
            : done ? CO_402_ACTION_RESULT_DONE : CO_402_ACTION_RESULT_BUSY;
    }
    return feedback;
}
}
