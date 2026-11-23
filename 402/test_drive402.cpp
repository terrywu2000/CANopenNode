#include "drive402.hpp"

#include <cstdio>
#include <cstdlib>

using namespace xco::d402;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                    \
        }                                                                  \
    } while (0)

namespace {

constexpr uint16_t cw_shutdown = 0x06;
constexpr uint16_t cw_switch_on = 0x07;
constexpr uint16_t cw_enable = 0x0F;

Inputs base_inputs(int8_t mode = mode_profile_position) {
    Inputs in;
    in.mode_requested = mode;
    in.profile_velocity = 1000;
    in.max_profile_velocity = 2000;
    in.profile_acceleration = 10000;
    in.profile_deceleration = 10000;
    in.quick_stop_deceleration = 20000;
    in.quick_stop_option = 2;
    in.fault_reaction_option = 0;
    return in;
}

void run(Drive& d, Inputs in, int cycles, uint32_t dt_us = 1000) {
    for (int i = 0; i < cycles; ++i) {
        d.set_external_permissions(in.model_ready, in.reset_permitted);
        d.step(in, dt_us);
        const auto status = d.status();
        if (status.external_model && status.action_pending) {
            in.action_response_seq = status.action_seq;
            in.action_result = CO_402_ACTION_RESULT_DONE;
            d.step(in, dt_us);
            in.action_response_seq = 0;
            in.action_result = CO_402_ACTION_RESULT_NONE;
        }
    }
}

/** Drive from switch-on disabled to operation enabled with the given mode. */
void enable(Drive& d, Inputs& in) {
    d.complete_local_initialization();
    d.process_safety(in, 0);
    in.controlword = cw_shutdown;
    run(d, in, 1);
    in.controlword = cw_switch_on;
    run(d, in, 1);
    in.controlword = cw_enable;
    run(d, in, 1);
}

} // namespace

static void test_pds_path() {
    Drive d;
    Inputs in = base_inputs();
    in.controlword = cw_shutdown;
    in.nmt_operational = false;
    d.step(in, 0);
    CHECK((d.status().statusword & 0x006f) == 0x0000);
    CHECK(d.status().mode_display == 0 && !d.status().output_allowed);
    d.complete_local_initialization();
    in.nmt_operational = true;
    d.process_safety(in, 0);
    CHECK(d.status().pds == Pds::switch_on_disabled);
    CHECK(d.status().statusword == 0x0240);
    enable(d, in);
    CHECK(d.status().pds == Pds::operation_enabled);
    CHECK((d.status().statusword & 0x006F) == 0x0027);
    CHECK(d.status().statusword & 0x0200);
    in.controlword = 0x00; // disable voltage
    run(d, in, 1);
    CHECK(d.status().pds == Pds::switch_on_disabled);
    d.reset();
    d.process_safety(in, 0);
    CHECK((d.status().statusword & 0x006f) == 0x0000);
    d.complete_local_initialization();
    in.nmt_operational = false;
    d.process_safety(in, 0);
    CHECK((d.status().statusword & 0x006f) == 0x0040);
}

static void test_pp_profile_reaches_target() {
    Drive d;
    Inputs in = base_inputs(mode_profile_position);
    enable(d, in);
    in.target_position = 5000;
    in.controlword = cw_enable | 0x10; // new setpoint
    run(d, in, 1);
    CHECK(d.status().statusword & 0x1000); // setpoint acknowledged
    in.controlword = cw_enable;            // level handshake: clear the request
    run(d, in, 1);
    run(d, in, 8000); // 5000 counts at 1000 counts/s, plus braking
    Status s = d.status();
    CHECK(s.demand_position == 5000);
    CHECK(s.actual_position == 5000);
    CHECK(s.statusword & 0x0400); // target reached
    CHECK(s.mode_display == mode_profile_position);
}

static void test_pp_relative_move() {
    Drive d;
    Inputs in = base_inputs(mode_profile_position);
    enable(d, in);
    in.target_position = 1000;
    in.controlword = cw_enable | 0x10 | 0x40; // relative
    run(d, in, 1);
    in.controlword = cw_enable;
    run(d, in, 3000);
    CHECK(d.status().actual_position == 1000);
}

static void test_unsupported_mode_has_no_motion() {
    Drive d;
    Inputs in = base_inputs(6); // homing: not implemented
    enable(d, in);
    in.target_position = 1000;
    run(d, in, 100);
    Status s = d.status();
    CHECK(s.mode_display == 0);
    CHECK(s.demand_velocity == 0);
    CHECK(!Drive::mode_supported(6));
    CHECK(Drive::mode_supported(mode_profile_position));
}

static void test_quick_stop_option2_then_disable() {
    Drive d;
    Inputs in = base_inputs(mode_profile_position);
    enable(d, in);
    in.target_position = 1000;
    in.controlword = cw_enable | 0x10;
    run(d, in, 1);
    in.controlword = cw_enable;
    run(d, in, 50);
    in.controlword = cw_enable & ~0x04; // quick stop request
    run(d, in, 1);
    CHECK(d.status().pds == Pds::quick_stop_active);
    run(d, in, 200);
    CHECK(d.status().demand_velocity == 0);
    CHECK(d.status().pds == Pds::switch_on_disabled);
}

static void test_fault_reset_is_edge_triggered() {
    Drive d;
    d.use_external_model(true);
    Inputs in = base_inputs(mode_profile_position);
    enable(d, in);
    d.post_feedback(Feedback{0, 0, true}); // model fault
    run(d, in, 1);
    CHECK(d.status().model_fault);
    run(d, in, 5);
    CHECK(d.status().pds == Pds::fault);
    CHECK(d.status().statusword == 0x0208);
    in.controlword = 0x80; // reset pulse while the model still reports a fault
    run(d, in, 1);
    CHECK(d.status().pds == Pds::fault);
    d.post_feedback(Feedback{0, 0, false});
    run(d, in, 1); // level still high: no rising edge, stays in fault
    CHECK(d.status().pds == Pds::fault);
    in.controlword = 0x00;
    run(d, in, 1);
    in.controlword = 0x80; // rising edge with the model healthy
    run(d, in, 1);
    CHECK(d.status().pds == Pds::switch_on_disabled);
}

static void test_external_stale_faults_and_freezes_actuals() {
    Drive d(5000); // 5 ms timeout
    d.use_external_model(true);
    Inputs in = base_inputs(mode_profile_position);
    CHECK(d.post_feedback(Feedback{100, 10, false}));
    enable(d, in);
    run(d, in, 20); // more than 5 ms with no further feedback
    Status s = d.status();
    CHECK(!s.feedback_fresh);
    CHECK(s.model_fault);
    CHECK(s.pds == Pds::fault_reaction_active || s.pds == Pds::fault);
    CHECK(s.actual_position == 100);
}

static void test_external_requires_mode_selection() {
    Drive d;
    CHECK(!d.post_feedback(Feedback{1, 1, false})); // internal model: rejected
    CHECK(!d.status().external_model);
}

static void test_external_action_acknowledgement_is_matched() {
    Drive d;
    d.use_external_model(true);
    d.set_external_permissions(true, true);
    CHECK(d.post_feedback(Feedback{0, 0, false}));
    Inputs in = base_inputs();
    d.complete_local_initialization();
    d.process_safety(in, 0);
    in.controlword = cw_shutdown;
    d.step(in, 1000);
    auto status = d.status();
    CHECK(status.action_pending && status.action == CO_402_ACTION_SHUTDOWN);
    CHECK(status.pds == Pds::switch_on_disabled && !status.output_allowed);

    in.action_response_seq = status.action_seq + 1;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.step(in, 1000);
    CHECK(d.status().action_pending && d.status().pds == Pds::switch_on_disabled);

    in.action_response_seq = status.action_seq;
    in.action_result = CO_402_ACTION_RESULT_BUSY;
    d.step(in, 1000);
    CHECK(d.status().action_pending);

    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.step(in, 1000);
    CHECK(!d.status().action_pending && d.status().pds == Pds::ready_to_switch_on);
}

static void test_internal_model_follows_demand() {
    Drive d;
    Inputs in = base_inputs(mode_profile_position);
    enable(d, in);
    in.target_position = 300;
    in.controlword = cw_enable | 0x10;
    run(d, in, 1);
    in.controlword = cw_enable;
    run(d, in, 2000);
    Status s = d.status();
    CHECK(s.feedback_fresh);
    CHECK(s.actual_position == s.demand_position);
    CHECK(s.actual_position == 300);
}

static void test_nmt_exit_requests_quick_stop() {
    Drive d;
    Inputs in = base_inputs();
    enable(d, in);
    in.target_position = 5000;
    in.controlword = cw_enable | 0x10;
    run(d, in, 1);
    in.controlword = cw_enable;
    run(d, in, 20);
    in.nmt_operational = false;
    run(d, in, 1);
    CHECK(d.status().pds == Pds::quick_stop_active);
    run(d, in, 1000);
    CHECK(d.status().pds == Pds::switch_on_disabled);
    CHECK(d.status().demand_velocity == 0);
}

static void test_target_reached_waits_for_actual_tolerance() {
    Drive d(10000000);
    d.use_external_model(true);
    Inputs in = base_inputs();
    in.target_reached_time_us = 3000;
    CHECK(d.post_feedback(Feedback{0, 0, false}));
    enable(d, in);
    in.target_position = 100;
    in.controlword = cw_enable | 0x10;
    run(d, in, 1);
    in.controlword = cw_enable;
    run(d, in, 2000);
    CHECK(!(d.status().statusword & 0x0400));
    CHECK(d.post_feedback(Feedback{100, 0, false}));
    run(d, in, 2);
    CHECK(!(d.status().statusword & 0x0400));
    run(d, in, 1);
    CHECK(d.status().statusword & 0x0400);
}

static void test_external_deadline_and_recovery() {
    Drive d(5000);
    d.use_external_model(true);
    d.set_external_permissions(true, true);
    CHECK(d.post_feedback(Feedback{123, 45, false}));
    Inputs in = base_inputs();
    in.external_feedback_age_valid = true;
    in.external_feedback_age_us = 4999;
    d.step(in, 1000000);
    CHECK(d.status().feedback_fresh && !d.status().model_fault);
    in.external_feedback_age_us = 5000;
    d.step(in, 0);
    auto reaction = d.status();
    CHECK(!reaction.feedback_fresh && reaction.model_fault);
    CHECK(reaction.pds == Pds::fault_reaction_active && reaction.action_pending);
    CHECK((reaction.statusword & 0x6f) == 0x0f && !reaction.output_allowed);
    CHECK(reaction.actual_position == 123 && reaction.actual_velocity == 45);

    in.external_feedback_age_us = 5001;
    in.action_response_seq = reaction.action_seq;
    in.action_result = CO_402_ACTION_RESULT_BUSY;
    d.step(in, 1000000);
    CHECK(d.status().pds == Pds::fault_reaction_active && d.status().action_pending);
    in.action_timeout_seq = reaction.action_seq + 1;
    d.step(in, 0);
    CHECK(d.status().action_pending);
    in.action_timeout_seq = reaction.action_seq;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.step(in, 0);
    auto fault = d.status();
    CHECK(fault.pds == Pds::fault && !fault.action_pending && fault.reaction_unconfirmed);
    CHECK(fault.action_seq != reaction.action_seq && fault.action == CO_402_ACTION_NONE);
    CHECK((fault.statusword & 0x6f) == 8 && !fault.output_allowed);
    CHECK(fault.actual_position == 123 && fault.actual_velocity == 45);

    in.action_timeout_seq = 0;
    d.step(in, 0); // A late DONE cannot resurrect the retired action.
    CHECK(d.status().pds == Pds::fault);
    CHECK(d.post_feedback(Feedback{123, 0, false}));
    in.external_feedback_age_us = 0;
    in.controlword = 0x80;
    in.action_result = CO_402_ACTION_RESULT_NONE;
    d.step(in, 0);
    CHECK(d.status().action == CO_402_ACTION_FAULT_RESET && d.status().pds == Pds::fault);
    in.action_response_seq = d.status().action_seq;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.step(in, 0);
    CHECK(d.status().pds == Pds::switch_on_disabled && !d.status().reaction_unconfirmed);
    CHECK(!d.status().output_allowed);
}

static void test_fault_preempts_quick_stop_and_failed_reaction() {
    Drive d;
    d.use_external_model(true);
    CHECK(d.post_feedback(Feedback{0, 0, false}));
    Inputs in = base_inputs();
    enable(d, in);
    CHECK(d.status().pds == Pds::operation_enabled);
    in.controlword = 0x0b;
    d.step(in, 0);
    const auto quick_stop_seq = d.status().action_seq;
    CHECK(d.status().action == CO_402_ACTION_QUICK_STOP);
    d.set_external_fault(true);
    in.action_response_seq = quick_stop_seq;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.step(in, 0);
    CHECK(d.status().pds == Pds::fault_reaction_active && d.status().action == CO_402_ACTION_FAULT_REACTION);
    CHECK(d.status().action_seq != quick_stop_seq && !d.status().output_allowed);
    d.step(in, 0);
    CHECK(d.status().pds == Pds::fault_reaction_active && d.status().action_pending);
    in.action_response_seq = d.status().action_seq;
    in.action_result = CO_402_ACTION_RESULT_ERROR;
    d.step(in, 0);
    CHECK(d.status().pds == Pds::fault && d.status().reaction_unconfirmed && !d.status().action_pending);
}

static void test_action_timeout_takes_priority_over_done() {
    Drive d;
    d.use_external_model(true);
    CHECK(d.post_feedback(Feedback{0, 0, false}));
    Inputs in = base_inputs();
    d.complete_local_initialization();
    d.process_safety(in, 0);
    in.controlword = cw_shutdown;
    d.step(in, 0);
    const auto seq = d.status().action_seq;
    in.action_response_seq = in.action_timeout_seq = seq;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.step(in, 0);
    CHECK(d.status().pds == Pds::fault_reaction_active);
    CHECK(d.status().action == CO_402_ACTION_FAULT_REACTION && d.status().action_seq != seq);
}

static void test_reset_edge_during_reaction_is_not_deferred() {
    Drive d;
    d.use_external_model(true);
    d.set_external_permissions(true, true);
    CHECK(d.post_feedback(Feedback{0, 0, true}));
    Inputs in = base_inputs();
    d.step(in, 0);
    const auto reaction_seq = d.status().action_seq;
    CHECK(d.post_feedback(Feedback{0, 0, false}));
    in.controlword = 0x80;
    in.action_response_seq = reaction_seq;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.step(in, 0);
    CHECK(d.status().pds == Pds::fault && !d.status().action_pending);
    d.step(in, 0);
    CHECK(d.status().pds == Pds::fault && !d.status().action_pending);
    in.controlword = 0;
    d.step(in, 0);
    in.controlword = 0x80;
    d.step(in, 0);
    CHECK(d.status().action == CO_402_ACTION_FAULT_RESET);
}

static void test_safety_without_motion_boundary() {
    Drive d;
    d.use_external_model(true);
    d.set_external_permissions(true, true);
    CHECK(d.post_feedback(Feedback{0, 0, false}));
    Inputs in = base_inputs();
    d.complete_local_initialization();
    in.controlword = cw_shutdown;
    d.process_safety(in, 1000);
    CHECK(!d.status().action_pending && d.status().pds == Pds::switch_on_disabled);
    CHECK(d.status().mode_display == 0);

    d.step(in, 0);
    auto action = d.status();
    CHECK(action.action == CO_402_ACTION_SHUTDOWN);
    in.controlword = cw_switch_on;
    in.action_response_seq = action.action_seq;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.process_safety(in, 1000);
    CHECK(d.status().pds == Pds::ready_to_switch_on && !d.status().action_pending);
    in.action_result = CO_402_ACTION_RESULT_NONE;
    d.step(in, 0);
    CHECK(d.status().action == CO_402_ACTION_SWITCH_ON);
    in.action_response_seq = d.status().action_seq;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.process_safety(in, 1000);
    CHECK(d.status().pds == Pds::switched_on && !d.status().action_pending);
    in.action_result = CO_402_ACTION_RESULT_NONE;
    in.controlword = cw_enable;
    d.step(in, 0);
    in.action_response_seq = d.status().action_seq;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.process_safety(in, 0);
    CHECK(d.status().pds == Pds::operation_enabled && d.status().output_allowed);
    in.action_result = CO_402_ACTION_RESULT_NONE;
    in.controlword = cw_enable | 0x10;
    in.target_position = 1000;
    for (int i = 0; i < 10; ++i) d.process_safety(in, 1000);
    CHECK(d.status().demand_position == 0 && !(d.status().statusword & 0x1000));
    d.step(in, 1000);
    CHECK(d.status().statusword & 0x1000);

    in.controlword = 0x0b;
    d.process_safety(in, 1000);
    CHECK(d.status().action == CO_402_ACTION_QUICK_STOP && !d.status().output_allowed);
    const auto quick_stop = d.status().action_seq;
    d.set_external_fault(true);
    in.action_response_seq = quick_stop;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.process_safety(in, 1000);
    CHECK(d.status().action == CO_402_ACTION_FAULT_REACTION
        && d.status().action_seq != quick_stop && !d.status().output_allowed);
    in.action_timeout_seq = d.status().action_seq;
    in.action_response_seq = in.action_timeout_seq;
    d.process_safety(in, 1000);
    CHECK(d.status().pds == Pds::fault && d.status().reaction_unconfirmed);
}

static void test_safety_ages_feedback_without_motion_boundary() {
    Drive d(5000);
    d.use_external_model(true);
    d.set_external_permissions(true, true);
    CHECK(d.post_feedback(Feedback{7, 0, false}));
    Inputs in = base_inputs();
    in.controlword = cw_enable;
    in.external_feedback_age_valid = true;
    in.external_feedback_age_us = 4999;
    d.process_safety(in, 10000);
    CHECK(d.status().feedback_fresh && !d.status().model_fault);
    in.external_feedback_age_us = 5000;
    d.process_safety(in, 0);
    CHECK(!d.status().feedback_fresh && d.status().model_fault);
    CHECK(d.status().pds == Pds::fault_reaction_active && !d.status().output_allowed);

    Drive internal;
    Inputs motion = base_inputs();
    motion.target_reached_time_us = 5000;
    enable(internal, motion);
    internal.step(motion, 0);
    CHECK(!(internal.status().statusword & 0x0400));
    internal.process_safety(motion, 10000);
    CHECK(!(internal.status().statusword & 0x0400));
    internal.step(motion, 5000);
    CHECK(internal.status().statusword & 0x0400);
}

static void test_safety_does_not_defer_fault_reset_edge() {
    Drive d;
    d.use_external_model(true);
    d.set_external_permissions(true, true);
    CHECK(d.post_feedback(Feedback{0, 0, true}));
    Inputs in = base_inputs();
    d.process_safety(in, 0);
    const auto reaction = d.status().action_seq;
    CHECK(d.post_feedback(Feedback{0, 0, false}));
    in.controlword = 0x80;
    in.action_response_seq = reaction;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.process_safety(in, 0);
    CHECK(d.status().pds == Pds::fault && !d.status().action_pending);
    in.action_result = CO_402_ACTION_RESULT_NONE;
    d.step(in, 0);
    CHECK(d.status().pds == Pds::fault && !d.status().action_pending);
    in.controlword = 0;
    d.process_safety(in, 0);
    in.controlword = 0x80;
    d.process_safety(in, 0);
    CHECK(d.status().action == CO_402_ACTION_FAULT_RESET);
    const auto reset = d.status().action_seq;
    d.step(in, 0);
    CHECK(d.status().action == CO_402_ACTION_FAULT_RESET && d.status().action_seq == reset);
}

static void test_safety_preempts_pending_enable() {
    Drive d;
    d.use_external_model(true);
    d.set_external_permissions(true, true);
    CHECK(d.post_feedback(Feedback{0, 0, false}));
    Inputs in = base_inputs();
    d.complete_local_initialization();
    d.process_safety(in, 0);
    in.controlword = cw_shutdown;
    run(d, in, 1, 0);
    in.controlword = cw_switch_on;
    run(d, in, 1, 0);
    in.controlword = cw_enable;
    d.step(in, 0);
    const auto enabling = d.status().action_seq;
    CHECK(d.status().action == CO_402_ACTION_ENABLE_OPERATION);
    in.controlword = 0;
    in.action_response_seq = enabling;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.process_safety(in, 0);
    CHECK(d.status().action == CO_402_ACTION_DISABLE_VOLTAGE
        && d.status().action_seq != enabling && !d.status().output_allowed);
}

static void test_quick_stop_requires_valid_standstill() {
    Drive d;
    d.use_external_model(true);
    CHECK(d.post_feedback(Feedback{}));
    auto in = base_inputs();
    enable(d, in);
    CHECK(d.status().output_allowed);
    in.quick_stop_option = 6;
    in.controlword = 0x0b;
    d.step(in, 0);
    in.action_response_seq = d.status().action_seq;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    CHECK(d.post_feedback(Feedback{0, 10, false}));
    d.step(in, 0);
    CHECK(d.status().action_pending && !d.status().output_allowed);
    CHECK(d.post_feedback(Feedback{0, 0, false, false, false}));
    d.step(in, 0);
    CHECK(d.status().action_pending);
    CHECK(d.post_feedback(Feedback{0, 0, false}));
    d.step(in, 0);
    CHECK(d.status().pds == Pds::quick_stop_active && !d.status().action_pending);
    in.controlword = cw_enable;
    d.step(in, 0);
    CHECK(d.status().action == CO_402_ACTION_ENABLE_OPERATION);
}

static void test_selected_feedback_is_independent_of_safety() {
    Drive d;
    d.use_external_model(true);
    d.set_external_permissions(true, true);
    Inputs in = base_inputs();
    CHECK(!d.select_feedback(Feedback{40, 0, false}));
    CHECK(d.post_safety_feedback(Feedback{100, 0, false}));
    CHECK(d.status().actual_position == 0 && !d.status().feedback_fresh && !d.status().model_ready);
    enable(d, in);
    CHECK(d.status().pds == Pds::switched_on && !d.status().output_allowed);
    CHECK(d.select_feedback(Feedback{40, 0, false}));
    d.set_external_permissions(true, true);
    in.controlword = cw_enable;
    d.step(in, 0);
    CHECK(d.status().action == CO_402_ACTION_ENABLE_OPERATION);
    in.action_response_seq = d.status().action_seq;
    in.action_result = CO_402_ACTION_RESULT_DONE;
    d.process_safety(in, 0);
    CHECK(d.status().output_allowed && d.status().actual_position == 40);
    in.action_result = CO_402_ACTION_RESULT_NONE;
    CHECK(d.post_safety_feedback(Feedback{200, 1, false}));
    d.step(in, 0);
    CHECK(d.status().actual_position == 40 && !d.status().model_fault);
    d.invalidate_selected_feedback();
    CHECK(!d.status().output_allowed && !d.status().feedback_fresh);
    in.target_position = 1000;
    in.controlword = cw_enable | 0x10;
    const auto previous = d.status().demand_position;
    d.step(in, 1000);
    CHECK(d.status().demand_position == previous && !(d.status().statusword & 0x1000));
    CHECK(d.post_safety_feedback(Feedback{300, 0, true}));
    CHECK(d.select_feedback(Feedback{40, 0, false}));
    CHECK(d.status().model_fault && d.status().actual_position == 40);
    d.process_safety(in, 0);
    CHECK(d.status().pds == Pds::fault_reaction_active && !d.status().output_allowed);

    Drive aged(5000);
    aged.use_external_model(true);
    CHECK(aged.post_safety_feedback(Feedback{100, 0, false}));
    Inputs age = base_inputs();
    age.external_feedback_age_valid = true;
    age.external_feedback_age_us = 4999;
    aged.process_safety(age, 0);
    CHECK(aged.select_feedback(Feedback{40, 0, false}));
    age.external_feedback_age_us = 5000;
    aged.process_safety(age, 0);
    CHECK(!aged.status().feedback_fresh && aged.status().model_fault);
}

int main() {
    test_pds_path();
    test_pp_profile_reaches_target();
    test_pp_relative_move();
    test_unsupported_mode_has_no_motion();
    test_quick_stop_option2_then_disable();
    test_fault_reset_is_edge_triggered();
    test_external_stale_faults_and_freezes_actuals();
    test_external_requires_mode_selection();
    test_external_action_acknowledgement_is_matched();
    test_internal_model_follows_demand();
    test_nmt_exit_requests_quick_stop();
    test_target_reached_waits_for_actual_tolerance();
    test_external_deadline_and_recovery();
    test_fault_preempts_quick_stop_and_failed_reaction();
    test_action_timeout_takes_priority_over_done();
    test_reset_edge_during_reaction_is_not_deferred();
    test_safety_without_motion_boundary();
    test_safety_ages_feedback_without_motion_boundary();
    test_safety_does_not_defer_fault_reset_edge();
    test_safety_preempts_pending_enable();
    test_quick_stop_requires_valid_standstill();
    test_selected_feedback_is_independent_of_safety();
    if (failures) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("drive402 tests passed");
    return EXIT_SUCCESS;
}
