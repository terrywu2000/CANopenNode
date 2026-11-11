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
    for (int i = 0; i < cycles; ++i) d.step(in, dt_us);
}

/** Drive from switch-on disabled to operation enabled with the given mode. */
void enable(Drive& d, Inputs& in) {
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
    CHECK(d.status().pds == Pds::switch_on_disabled);
    CHECK(d.status().statusword == 0x0240);
    enable(d, in);
    CHECK(d.status().pds == Pds::operation_enabled);
    CHECK((d.status().statusword & 0x006F) == 0x0027);
    CHECK(d.status().statusword & 0x0200);
    in.controlword = 0x00; // disable voltage
    run(d, in, 1);
    CHECK(d.status().pds == Pds::switch_on_disabled);
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

static void test_pv_profile_ramps_to_target_velocity() {
    Drive d;
    Inputs in = base_inputs(mode_profile_velocity);
    enable(d, in);
    in.target_velocity = 1000;
    run(d, in, 300); // 10000 counts/s^2 reaches 1000 counts/s in 100 ms
    Status s = d.status();
    CHECK(s.statusword & 0x1000); // speed reached
    CHECK(s.mode_display == mode_profile_velocity);
    in.target_velocity = 0;
    run(d, in, 500);
    CHECK(d.status().demand_velocity == 0);
}

static void test_unsupported_mode_has_no_motion() {
    Drive d;
    Inputs in = base_inputs(6); // homing: not implemented
    enable(d, in);
    in.target_velocity = 1000;
    run(d, in, 100);
    Status s = d.status();
    CHECK(s.mode_display == 0);
    CHECK(s.demand_velocity == 0);
    CHECK(!Drive::mode_supported(6));
    CHECK(Drive::mode_supported(mode_profile_position));
    CHECK(Drive::mode_supported(mode_profile_velocity));
}

static void test_quick_stop_option2_then_disable() {
    Drive d;
    Inputs in = base_inputs(mode_profile_velocity);
    enable(d, in);
    in.target_velocity = 1000;
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
    Inputs in = base_inputs(mode_profile_velocity);
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
    Inputs in = base_inputs(mode_profile_velocity);
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

int main() {
    test_pds_path();
    test_pp_profile_reaches_target();
    test_pp_relative_move();
    test_pv_profile_ramps_to_target_velocity();
    test_unsupported_mode_has_no_motion();
    test_quick_stop_option2_then_disable();
    test_fault_reset_is_edge_triggered();
    test_external_stale_faults_and_freezes_actuals();
    test_external_requires_mode_selection();
    test_internal_model_follows_demand();
    if (failures) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::puts("drive402 tests passed");
    return EXIT_SUCCESS;
}
