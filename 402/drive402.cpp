#include "drive402.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace xco::d402 {
namespace {

constexpr uint16_t cw_switch_on = 0x01;
constexpr uint16_t cw_disable_voltage = 0x02;
constexpr uint16_t cw_quick_stop = 0x04;
constexpr uint16_t cw_enable_operation = 0x08;
constexpr uint16_t cw_new_setpoint = 0x10;
constexpr uint16_t cw_change_immediately = 0x20;
constexpr uint16_t cw_relative = 0x40;
constexpr uint16_t cw_fault_reset = 0x80;
constexpr uint16_t cw_halt = 0x100;

constexpr uint16_t sw_remote = 0x0200;
constexpr uint16_t sw_target_reached = 0x0400;
constexpr uint16_t sw_setpoint_ack_or_speed = 0x1000;

bool shutdown_request(uint16_t cw) { return (cw & 0x87) == 0x06; }
bool switch_on_request(uint16_t cw) { return (cw & 0x8F) == 0x07; }
bool enable_request(uint16_t cw) { return (cw & 0x8F) == 0x0F; }

int32_t clamp_i32(double value) {
    constexpr double low = std::numeric_limits<int32_t>::min();
    constexpr double high = std::numeric_limits<int32_t>::max();
    return static_cast<int32_t>(std::lround(std::clamp(value, low, high)));
}

uint16_t base_statusword(Pds pds) {
    switch (pds) {
    case Pds::not_ready_to_switch_on: return 0x0000;
    case Pds::switch_on_disabled: return 0x0040;
    case Pds::ready_to_switch_on: return 0x0021;
    case Pds::switched_on: return 0x0023;
    case Pds::operation_enabled: return 0x0027;
    case Pds::quick_stop_active: return 0x0007;
    case Pds::fault_reaction_active: return 0x000F;
    case Pds::fault: return 0x0008;
    }
    return 0x0000;
}

} // namespace

Feedback InternalModel::sample(int32_t demand_position, int32_t demand_velocity, uint32_t) {
    return Feedback{demand_position, demand_velocity, false};
}

void ExternalModel::post(const Feedback& feedback) {
    last_ = feedback;
    age_us_ = 0;
    has_feedback_ = true;
}

Feedback ExternalModel::sample(int32_t, int32_t, uint32_t dt_us) {
    age_us_ += dt_us;
    return last_;
}

Drive::Drive(uint32_t feedback_timeout_us) : external_(feedback_timeout_us) {}

void Drive::use_external_model(bool external) {
    use_external_ = external;
}

bool Drive::post_feedback(const Feedback& feedback) {
    if (!use_external_) return false;
    external_.post(feedback);
    feedback_ = feedback;
    return true;
}

bool Drive::mode_supported(int8_t mode) {
    return mode == mode_profile_position || mode == mode_profile_velocity;
}

void Drive::reset() {
    pds_ = Pds::switch_on_disabled;
    demand_pos_ = feedback_.position;
    demand_vel_ = 0.0;
    pending_valid_ = false;
    pp_setpoint_ack_ = false;
    target_reached_ = false;
    speed_reached_ = false;
    halt_ = false;
    mode_display_ = 0;
    previous_mode_ = 0;
}

void Drive::step(const Inputs& in, uint32_t dt_us) {
    const double dt_s = dt_us / 1e6;
    const int8_t display = mode_supported(in.mode_requested) ? in.mode_requested : int8_t{0};
    if (display != previous_mode_) {
        // Mode change restarts the profile from the actual state.
        demand_pos_ = feedback_.position;
        demand_vel_ = feedback_.velocity;
        pending_valid_ = false;
        pp_setpoint_ack_ = false;
        target_reached_ = false;
        speed_reached_ = false;
        previous_mode_ = display;
    }
    mode_display_ = display;

    if (model_faulted() && pds_ != Pds::fault_reaction_active && pds_ != Pds::fault) {
        reaction_option_ = in.fault_reaction_option;
        pds_ = Pds::fault_reaction_active;
    }
    transition(in);
    fault_reset_prev_ = (in.controlword & cw_fault_reset) != 0;
    motion(in, dt_s);

    feedback_ = model().sample(clamp_i32(demand_pos_), clamp_i32(demand_vel_), dt_us);
}

bool Drive::model_faulted() const {
    if (feedback_.fault) return true;
    return use_external_ && !external_.fresh();
}

MotorModel& Drive::model() {
    if (use_external_) return external_;
    return internal_;
}

void Drive::transition(const Inputs& in) {
    const uint16_t cw = in.controlword;
    const bool disable_voltage = (cw & cw_disable_voltage) == 0;
    const bool quick_stop_request = (cw & cw_quick_stop) == 0;

    switch (pds_) {
    case Pds::not_ready_to_switch_on:
        pds_ = Pds::switch_on_disabled;
        break;
    case Pds::switch_on_disabled:
        if (shutdown_request(cw)) pds_ = Pds::ready_to_switch_on;
        break;
    case Pds::ready_to_switch_on:
        if (disable_voltage) pds_ = Pds::switch_on_disabled;
        else if (switch_on_request(cw)) pds_ = Pds::switched_on;
        break;
    case Pds::switched_on:
        if (disable_voltage) pds_ = Pds::switch_on_disabled;
        else if (shutdown_request(cw)) pds_ = Pds::ready_to_switch_on;
        else if (enable_request(cw)) pds_ = Pds::operation_enabled;
        break;
    case Pds::operation_enabled:
        if (disable_voltage) pds_ = Pds::switch_on_disabled;
        else if (quick_stop_request) {
            quick_stop_option_ = in.quick_stop_option;
            pds_ = in.quick_stop_option == 0 ? Pds::switch_on_disabled : Pds::quick_stop_active;
        } else if (switch_on_request(cw)) pds_ = Pds::switched_on;
        else if (shutdown_request(cw)) pds_ = Pds::ready_to_switch_on;
        break;
    case Pds::quick_stop_active:
        if (disable_voltage) {
            pds_ = Pds::switch_on_disabled;
        } else if ((quick_stop_option_ == 5 || quick_stop_option_ == 6) && enable_request(cw)) {
            // Options 5 and 6 resume operation when the quick-stop request is withdrawn.
            pds_ = Pds::operation_enabled;
        } else if ((quick_stop_option_ == 1 || quick_stop_option_ == 2) && std::fabs(demand_vel_) < 1e-9) {
            pds_ = Pds::switch_on_disabled;
        }
        break;
    case Pds::fault_reaction_active:
        if (std::fabs(demand_vel_) < 1e-9) pds_ = Pds::fault;
        break;
    case Pds::fault:
        if ((cw & cw_fault_reset) && !fault_reset_prev_ && !model_faulted()) pds_ = Pds::switch_on_disabled;
        break;
    }
}

void Drive::motion(const Inputs& in, double dt_s) {
    halt_ = (in.controlword & cw_halt) != 0;
    if ((in.controlword & cw_new_setpoint) == 0) pp_setpoint_ack_ = false;

    const double vmax_pp = in.max_profile_velocity && in.max_profile_velocity < in.profile_velocity
        ? in.max_profile_velocity : in.profile_velocity;

    switch (pds_) {
    case Pds::operation_enabled:
        if (halt_) {
            approach(0.0, in.profile_acceleration, in.profile_deceleration, dt_s);
            target_reached_ = false;
            speed_reached_ = false;
        } else if (mode_display_ == mode_profile_position) {
            latch_setpoint(in);
            track_target(dt_s, vmax_pp, in.profile_acceleration, in.profile_deceleration);
        } else if (mode_display_ == mode_profile_velocity) {
            double vt = in.target_velocity;
            if (in.max_profile_velocity) {
                vt = std::clamp(vt, -double(in.max_profile_velocity), double(in.max_profile_velocity));
            }
            approach(vt, in.profile_acceleration, in.profile_deceleration, dt_s);
            speed_reached_ = std::fabs(demand_vel_ - vt) < 1e-9;
            target_reached_ = speed_reached_;
            demand_pos_ += demand_vel_ * dt_s;
        } else {
            approach(0.0, in.profile_acceleration, in.profile_deceleration, dt_s);
        }
        break;
    case Pds::quick_stop_active:
    case Pds::fault_reaction_active: {
        const uint32_t decel = (pds_ == Pds::quick_stop_active)
            ? ((quick_stop_option_ == 2 || quick_stop_option_ == 6) ? in.quick_stop_deceleration : in.profile_deceleration)
            : (reaction_option_ == 2 ? in.quick_stop_deceleration : in.profile_deceleration);
        const bool immediate = (pds_ == Pds::fault_reaction_active) ? reaction_option_ == 0 : quick_stop_option_ == 0;
        if (immediate) demand_vel_ = 0.0;
        else approach(0.0, 0.0, decel, dt_s);
        demand_pos_ += demand_vel_ * dt_s;
        target_reached_ = false;
        speed_reached_ = false;
        break;
    }
    default:
        // Unpowered states hold no velocity, so the demand stops at once.
        demand_vel_ = 0.0;
        target_reached_ = false;
        speed_reached_ = false;
        break;
    }
}

void Drive::latch_setpoint(const Inputs& in) {
    if ((in.controlword & cw_new_setpoint) == 0 || pp_setpoint_ack_) return;
    const double value = (in.controlword & cw_relative)
        ? demand_pos_ + double(in.target_position)
        : double(in.target_position);
    pp_setpoint_ack_ = true;
    const bool in_motion = !target_reached_ && (std::fabs(target_pos_ - demand_pos_) > 0.5 || std::fabs(demand_vel_) > 1e-9);
    if ((in.controlword & cw_change_immediately) || !in_motion) {
        target_pos_ = value;
        pending_valid_ = false;
        target_reached_ = false;
    } else {
        pending_pos_ = value;
        pending_valid_ = true;
    }
}

void Drive::track_target(double dt_s, double vmax, uint32_t accel, uint32_t decel) {
    const double d = target_pos_ - demand_pos_;
    if (std::fabs(d) < 0.5 && std::fabs(demand_vel_) < 1e-9) {
        demand_pos_ = target_pos_;
        target_reached_ = true;
        if (pending_valid_) {
            target_pos_ = pending_pos_;
            pending_valid_ = false;
            target_reached_ = false;
        }
        return;
    }
    // Braking curve: the velocity that can still stop exactly at the target.
    double vd = vmax;
    if (decel) vd = std::min(vmax, std::sqrt(2.0 * decel * std::fabs(d)));
    vd = (d < 0) ? -vd : vd;
    approach(vd, accel, decel, dt_s);
    demand_pos_ += demand_vel_ * dt_s;
    if ((d > 0 && demand_pos_ > target_pos_) || (d < 0 && demand_pos_ < target_pos_)) {
        demand_pos_ = target_pos_;
        demand_vel_ = 0.0;
    }
}

void Drive::approach(double target_velocity, uint32_t accel, uint32_t decel, double dt_s) {
    const bool speeding_up = std::fabs(target_velocity) > std::fabs(demand_vel_);
    const double rate = speeding_up ? double(accel) : double(decel);
    if (rate <= 0.0) {
        demand_vel_ = target_velocity;
        return;
    }
    const double step = rate * dt_s;
    const double delta = target_velocity - demand_vel_;
    if (std::fabs(delta) <= step) demand_vel_ = target_velocity;
    else demand_vel_ += (delta > 0 ? step : -step);
}

Status Drive::status() const {
    Status s;
    s.pds = pds_;
    s.external_model = use_external_;
    s.mode_display = mode_display_;
    s.demand_position = clamp_i32(demand_pos_);
    s.demand_velocity = clamp_i32(demand_vel_);
    s.actual_position = feedback_.position;
    s.actual_velocity = feedback_.velocity;
    s.model_fault = model_faulted();
    s.feedback_fresh = use_external_ ? external_.fresh() : true;

    uint16_t sw = base_statusword(pds_) | sw_remote;
    if (target_reached_) sw |= sw_target_reached;
    if (mode_display_ == mode_profile_position && pp_setpoint_ack_) sw |= sw_setpoint_ack_or_speed;
    if (mode_display_ == mode_profile_velocity && speed_reached_) sw |= sw_setpoint_ack_or_speed;
    s.statusword = sw;
    return s;
}

} // namespace xco::d402
