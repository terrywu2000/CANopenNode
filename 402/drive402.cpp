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
constexpr uint16_t sw_internal_limit = 0x0800;
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

void ExternalModel::refresh() {
    age_us_ = 0;
    has_feedback_ = true;
}

void ExternalModel::advance_age(uint32_t dt_us) {
    age_us_ += std::min<uint64_t>(dt_us, UINT64_MAX - age_us_);
}

Feedback ExternalModel::sample(int32_t, int32_t, uint32_t) {
    return last_;
}

Drive::Drive(uint32_t feedback_timeout_us) : external_(feedback_timeout_us) {}

void Drive::use_external_model(bool external) {
    use_external_ = external;
}

bool Drive::post_feedback(const Feedback& feedback) {
    return post_safety_feedback(feedback) && select_feedback(feedback);
}

bool Drive::post_safety_feedback(const Feedback& feedback) {
    if (!use_external_) return false;
    external_.refresh();
    safety_feedback_ = feedback;
    return true;
}

bool Drive::select_feedback(const Feedback& feedback) {
    if (!use_external_ || !external_.has_feedback()) return false;
    external_.select(feedback);
    feedback_ = feedback;
    selected_feedback_valid_ = true;
    return true;
}

void Drive::invalidate_selected_feedback() {
    selected_feedback_valid_ = false;
}

void Drive::set_external_fault(bool fault) {
    external_transport_fault_ = fault;
}

void Drive::set_external_permissions(bool ready, bool reset_permitted) {
    external_model_ready_ = ready;
    external_reset_permitted_ = reset_permitted;
}

bool Drive::mode_supported(int8_t mode) {
    return mode == mode_profile_position;
}

void Drive::reset() {
    external_.reset();
    safety_feedback_ = {};
    selected_feedback_valid_ = false;
    pds_ = Pds::not_ready_to_switch_on;
    local_initialization_complete_ = false;
    demand_pos_ = feedback_.position;
    demand_vel_ = 0.0;
    pending_valid_ = false;
    pp_setpoint_ack_ = false;
    target_reached_ = false;
    target_reached_age_us_ = 0;
    speed_reached_ = false;
    halt_ = false;
    mode_display_ = 0;
    previous_mode_ = 0;
    action_ = pending_action_ = CO_402_ACTION_NONE;
    action_seq_ = 1;
    action_pending_ = false;
    pending_pds_ = Pds::switch_on_disabled;
    action_failed_ = false;
    reaction_unconfirmed_ = false;
    external_transport_fault_ = false;
    external_model_ready_ = false;
    external_reset_permitted_ = false;
    fault_reset_prev_ = false;
}

void Drive::step(const Inputs& in, uint32_t dt_us) {
    if (use_external_) {
        if (in.external_feedback_age_valid) external_.set_age(in.external_feedback_age_us);
        else external_.advance_age(dt_us);
    }
    const double dt_s = dt_us / 1e6;
    const int8_t display = pds_ != Pds::not_ready_to_switch_on && mode_supported(in.mode_requested)
        ? in.mode_requested : int8_t{0};
    if (display != previous_mode_) {
        // Mode change restarts the profile from the actual state.
        demand_pos_ = feedback_.position;
        demand_vel_ = feedback_.velocity;
        pending_valid_ = false;
        pp_setpoint_ack_ = false;
        target_reached_ = false;
        target_reached_age_us_ = 0;
        speed_reached_ = false;
        previous_mode_ = display;
    }
    mode_display_ = display;

    transition(in);
    fault_reset_prev_ = (in.controlword & cw_fault_reset) != 0;
    motion(in, dt_s);

    feedback_ = model().sample(clamp_i32(demand_pos_), clamp_i32(demand_vel_), dt_us);
    const bool within_tolerance = std::fabs(double(feedback_.position) - target_pos_)
        <= double(in.position_tolerance);
    if (pds_ == Pds::operation_enabled && !action_pending_ && in.nmt_operational
        && mode_display_ == mode_profile_position
        && (!use_external_ || (external_.fresh() && selected_feedback_valid_)) && !model_faulted()
        && std::fabs(demand_vel_) < 1e-9 && within_tolerance) {
        target_reached_age_us_ = std::min<uint64_t>(
            target_reached_age_us_ + dt_us, std::numeric_limits<uint32_t>::max());
        target_reached_ = target_reached_age_us_ >= in.target_reached_time_us;
    } else {
        target_reached_age_us_ = 0;
        target_reached_ = false;
    }
}

void Drive::process_safety(const Inputs& in, uint32_t dt_us) {
    if (use_external_) {
        if (in.external_feedback_age_valid) external_.set_age(in.external_feedback_age_us);
        else external_.advance_age(dt_us);
    }
    transition(in, true);
    fault_reset_prev_ = (in.controlword & cw_fault_reset) != 0;
    const bool stopping = pds_ == Pds::quick_stop_active || pds_ == Pds::fault_reaction_active
        || (action_pending_ && (pending_action_ == CO_402_ACTION_QUICK_STOP
                               || pending_action_ == CO_402_ACTION_FAULT_REACTION));
    if (stopping) {
        motion(in, dt_us / 1e6);
        feedback_ = model().sample(clamp_i32(demand_pos_), clamp_i32(demand_vel_), dt_us);
    }
    if (stopping || pds_ != Pds::operation_enabled || action_pending_
        || !in.nmt_operational || model_faulted()) {
        target_reached_ = false;
        target_reached_age_us_ = 0;
        speed_reached_ = false;
    }
}

bool Drive::model_faulted() const {
    if ((use_external_ ? safety_feedback_.fault : feedback_.fault) || external_transport_fault_) return true;
    return use_external_ && external_.has_feedback() && !external_.fresh();
}

bool Drive::model_stopped() const {
    return use_external_ ? external_.fresh() && safety_feedback_.velocity_valid && safety_feedback_.velocity == 0
                         : std::fabs(demand_vel_) < 1e-9;
}

MotorModel& Drive::model() {
    if (use_external_) return external_;
    return internal_;
}

void Drive::transition(const Inputs& in, bool safety_only) {
    uint16_t cw = in.controlword;
    if (!in.nmt_operational) {
        // NMT remains owned by CANopen; leaving Operational requests the DS402
        // quick-stop reaction rather than allowing new network motion commands.
        cw = static_cast<uint16_t>((cw | cw_disable_voltage) & ~cw_quick_stop);
    }
    const bool disable_voltage = (cw & cw_disable_voltage) == 0;
    const bool quick_stop_request = (cw & cw_quick_stop) == 0
        || (use_external_ && !in.model_ready && pds_ == Pds::operation_enabled);
    const bool fault_reaction = model_faulted() || (action_failed_ && pds_ != Pds::fault);
    const bool can_enable = !use_external_ || (in.model_ready && external_model_ready_
                                                && external_.fresh() && selected_feedback_valid_);
    const bool reset_edge = pds_ == Pds::fault && (cw & cw_fault_reset) && !fault_reset_prev_;

    if (use_external_ && action_pending_ && in.action_timeout_seq == action_seq_) {
        action_failed_ = true;
        if (pending_action_ == CO_402_ACTION_FAULT_REACTION) fail_fault_reaction();
        else begin_fault_reaction(in);
        return;
    }
    if (use_external_ && fault_reaction && (pds_ != Pds::fault || action_pending_)
        && (!action_pending_ || pending_action_ != CO_402_ACTION_FAULT_REACTION)) {
        begin_fault_reaction(in);
        return;
    }

    if (use_external_ && action_pending_) {
        if (!in.nmt_operational && pending_action_ == CO_402_ACTION_FAULT_RESET) {
            ++action_seq_;
            if (action_seq_ == 0) ++action_seq_;
            action_pending_ = false;
            action_ = pending_action_ = CO_402_ACTION_NONE;
            return;
        }
        const bool enabling = pending_action_ == CO_402_ACTION_ENABLE_OPERATION;
        if (pending_action_ != CO_402_ACTION_FAULT_REACTION) {
            if (disable_voltage && pending_action_ != CO_402_ACTION_FAULT_RESET)
                request_transition(CO_402_ACTION_DISABLE_VOLTAGE, Pds::switch_on_disabled);
            else if ((enabling && (!can_enable || quick_stop_request))
                     || (pds_ == Pds::operation_enabled && quick_stop_request)) {
                if (pending_action_ != CO_402_ACTION_QUICK_STOP && pending_action_ != CO_402_ACTION_DISABLE_VOLTAGE) {
                    quick_stop_option_ = in.quick_stop_option;
                    request_transition(CO_402_ACTION_QUICK_STOP,
                        in.quick_stop_option == 0 ? Pds::switch_on_disabled : Pds::quick_stop_active);
                }
            } else if (quick_stop_request && pending_action_ != CO_402_ACTION_QUICK_STOP
                       && (pds_ == Pds::ready_to_switch_on || pds_ == Pds::switched_on
                           || pending_action_ == CO_402_ACTION_SHUTDOWN)) {
                request_transition(CO_402_ACTION_DISABLE_VOLTAGE, Pds::switch_on_disabled);
            }
        }
        if (in.action_response_seq == action_seq_) {
            const bool quick_stop_confirmed = pending_action_ != CO_402_ACTION_QUICK_STOP || model_stopped();
            if (in.action_result == CO_402_ACTION_RESULT_DONE && quick_stop_confirmed) {
                const bool reaction_completed = pending_action_ == CO_402_ACTION_FAULT_REACTION;
                pds_ = pending_pds_;
                if (pending_action_ == CO_402_ACTION_FAULT_RESET) {
                    action_failed_ = false;
                    reaction_unconfirmed_ = false;
                }
                action_pending_ = false;
                action_ = pending_action_ = CO_402_ACTION_NONE;
                if (reaction_completed) return;
            } else if (in.action_result == CO_402_ACTION_RESULT_ERROR) {
                action_failed_ = true;
                if (pending_action_ == CO_402_ACTION_FAULT_REACTION) fail_fault_reaction();
                else begin_fault_reaction(in);
                return;
            }
        }
        if (action_pending_) return;
    }

    if (fault_reaction && pds_ != Pds::fault_reaction_active && pds_ != Pds::fault) {
        reaction_option_ = in.fault_reaction_option;
        pds_ = Pds::fault_reaction_active;
    }

    uint8_t action = CO_402_ACTION_NONE;
    Pds target = pds_;
    switch (pds_) {
    case Pds::not_ready_to_switch_on:
        if (local_initialization_complete_) pds_ = Pds::switch_on_disabled;
        return;
    case Pds::switch_on_disabled:
        if (!safety_only && shutdown_request(cw)) { action = CO_402_ACTION_SHUTDOWN; target = Pds::ready_to_switch_on; }
        break;
    case Pds::ready_to_switch_on:
        if (disable_voltage || quick_stop_request) { action = CO_402_ACTION_DISABLE_VOLTAGE; target = Pds::switch_on_disabled; }
        else if (!safety_only && (switch_on_request(cw) || enable_request(cw))) {
            action = CO_402_ACTION_SWITCH_ON; target = Pds::switched_on;
        }
        break;
    case Pds::switched_on:
        if (disable_voltage || quick_stop_request) { action = CO_402_ACTION_DISABLE_VOLTAGE; target = Pds::switch_on_disabled; }
        else if (!safety_only && shutdown_request(cw)) { action = CO_402_ACTION_SHUTDOWN; target = Pds::ready_to_switch_on; }
        else if (!safety_only && enable_request(cw) && can_enable) {
            action = CO_402_ACTION_ENABLE_OPERATION; target = Pds::operation_enabled;
        }
        break;
    case Pds::operation_enabled:
        if (disable_voltage) { action = CO_402_ACTION_DISABLE_VOLTAGE; target = Pds::switch_on_disabled; }
        else if (quick_stop_request) {
            quick_stop_option_ = in.quick_stop_option;
            action = CO_402_ACTION_QUICK_STOP;
            target = in.quick_stop_option == 0 ? Pds::switch_on_disabled : Pds::quick_stop_active;
        } else if (!safety_only && switch_on_request(cw)) { action = CO_402_ACTION_DISABLE_OPERATION; target = Pds::switched_on; }
        else if (!safety_only && shutdown_request(cw)) { action = CO_402_ACTION_SHUTDOWN; target = Pds::ready_to_switch_on; }
        break;
    case Pds::quick_stop_active:
        if (disable_voltage || (in.force_shutdown && model_stopped())) {
            action = CO_402_ACTION_DISABLE_VOLTAGE; target = Pds::switch_on_disabled;
        } else if (!safety_only && (quick_stop_option_ == 5 || quick_stop_option_ == 6) && enable_request(cw)
                   && model_stopped() && can_enable) {
            action = CO_402_ACTION_ENABLE_OPERATION; target = Pds::operation_enabled;
        } else if ((quick_stop_option_ == 1 || quick_stop_option_ == 2) && model_stopped()) {
            action = CO_402_ACTION_DISABLE_VOLTAGE; target = Pds::switch_on_disabled;
        }
        break;
    case Pds::fault_reaction_active:
        if (std::fabs(demand_vel_) < 1e-9) pds_ = Pds::fault;
        break;
    case Pds::fault:
        if (in.nmt_operational && reset_edge && !model_faulted()
            && (!use_external_ || (in.reset_permitted && external_reset_permitted_))) {
            action = CO_402_ACTION_FAULT_RESET; target = Pds::switch_on_disabled;
        }
        break;
    }
    if (action != CO_402_ACTION_NONE) request_transition(action, target);
}

void Drive::request_transition(uint8_t action, Pds target) {
    if (!use_external_) {
        pds_ = target;
        return;
    }
    if (action_pending_ && pending_action_ == action && pending_pds_ == target) return;
    ++action_seq_;
    if (action_seq_ == 0) ++action_seq_;
    action_ = pending_action_ = action;
    pending_pds_ = target;
    action_pending_ = true;
}

void Drive::begin_fault_reaction(const Inputs& in) {
    reaction_option_ = in.fault_reaction_option;
    pds_ = Pds::fault_reaction_active;
    request_transition(CO_402_ACTION_FAULT_REACTION, Pds::fault);
}

void Drive::fail_fault_reaction() {
    // Retire the action without manufacturing a successful model response.
    ++action_seq_;
    if (action_seq_ == 0) ++action_seq_;
    action_ = pending_action_ = CO_402_ACTION_NONE;
    action_pending_ = false;
    pds_ = pending_pds_ = Pds::fault;
    reaction_unconfirmed_ = true;
}

void Drive::motion(const Inputs& in, double dt_s) {
    halt_ = (in.controlword & cw_halt) != 0;
    if ((in.controlword & cw_new_setpoint) == 0) pp_setpoint_ack_ = false;

    const double vmax_pp = in.max_profile_velocity && in.max_profile_velocity < in.profile_velocity
        ? in.max_profile_velocity : in.profile_velocity;

    const bool pending_stop = action_pending_
        && (pending_action_ == CO_402_ACTION_QUICK_STOP || pending_action_ == CO_402_ACTION_FAULT_REACTION);
    const Pds motion_pds = pending_stop
        ? (pending_action_ == CO_402_ACTION_FAULT_REACTION ? Pds::fault_reaction_active : Pds::quick_stop_active)
        : pds_;
    switch (motion_pds) {
    case Pds::operation_enabled:
        if (use_external_ && !selected_feedback_valid_) {
            demand_vel_ = 0.0;
            target_reached_ = false;
            speed_reached_ = false;
        } else if (halt_) {
            approach(0.0, in.profile_acceleration, in.profile_deceleration, dt_s);
            target_reached_ = false;
            speed_reached_ = false;
        } else if (mode_display_ == mode_profile_position && in.nmt_operational && !action_pending_) {
            latch_setpoint(in);
            track_target(dt_s, vmax_pp, in.profile_acceleration, in.profile_deceleration);
        } else {
            approach(0.0, in.profile_acceleration, in.profile_deceleration, dt_s);
        }
        break;
    case Pds::quick_stop_active:
    case Pds::fault_reaction_active: {
        const uint32_t decel = (motion_pds == Pds::quick_stop_active)
            ? ((quick_stop_option_ == 2 || quick_stop_option_ == 6) ? in.quick_stop_deceleration : in.profile_deceleration)
            : (reaction_option_ == 2 ? in.quick_stop_deceleration : in.profile_deceleration);
        const bool immediate = (motion_pds == Pds::fault_reaction_active) ? reaction_option_ == 0 : quick_stop_option_ == 0;
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
    s.feedback_fresh = use_external_ ? external_.fresh() && selected_feedback_valid_ : true;
    s.model_ready = !use_external_ || (external_model_ready_ && selected_feedback_valid_);
    s.action = action_;
    s.action_seq = action_seq_;
    s.action_pending = action_pending_;
    s.reaction_unconfirmed = reaction_unconfirmed_;
    s.output_allowed = pds_ == Pds::operation_enabled && !action_pending_ && !s.model_fault
        && (!use_external_ || selected_feedback_valid_);

    uint16_t sw = base_statusword(pds_) | sw_remote;
    if (target_reached_) sw |= sw_target_reached;
    if (s.feedback_fresh && feedback_.limited) sw |= sw_internal_limit;
    if (mode_display_ == mode_profile_position && pp_setpoint_ack_) sw |= sw_setpoint_ack_or_speed;
    s.statusword = sw;
    return s;
}

} // namespace xco::d402
