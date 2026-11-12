#include "CO_402_protocol.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
        ++failures; \
    } \
} while (0)

static void test_command_round_trip(void) {
    uint8_t bytes[CO_402_COMMAND_FRAME_SIZE];
    CO_402_command_frame_t input = {
        .epoch = UINT64_C(0x1020304050607080),
        .command_seq = 0x12345678,
        .action_seq = 0x87654321,
        .controlword = 0x001f,
        .statusword = 0x0627,
        .action = CO_402_ACTION_ENABLE_OPERATION,
        .flags = 0x03,
        .pds = 4,
        .mode = 1,
        .demand_position = INT32_MIN,
        .target_position = INT32_MAX,
        .profile_velocity = 100000,
        .profile_acceleration = 200000,
        .profile_deceleration = 300000,
        .quick_stop_deceleration = 400000,
        .demand_velocity = -1
    };
    CO_402_command_frame_t output;
    CHECK(CO_402_command_pack(&input, bytes, sizeof(bytes)));
    CHECK(bytes[0] == 1 && bytes[1] == CO_402_COMMAND_FRAME_SIZE);
    CHECK(bytes[4] == 0x80 && bytes[11] == 0x10);
    CHECK(CO_402_command_unpack(&output, bytes, sizeof(bytes)));
    CHECK(input.epoch == output.epoch && input.command_seq == output.command_seq
        && input.action_seq == output.action_seq && input.controlword == output.controlword
        && input.statusword == output.statusword && input.action == output.action
        && input.flags == output.flags && input.pds == output.pds && input.mode == output.mode
        && input.demand_position == output.demand_position && input.target_position == output.target_position
        && input.profile_velocity == output.profile_velocity
        && input.profile_acceleration == output.profile_acceleration
        && input.profile_deceleration == output.profile_deceleration
        && input.quick_stop_deceleration == output.quick_stop_deceleration
        && input.demand_velocity == output.demand_velocity);
    CHECK(!CO_402_command_pack(&input, bytes, sizeof(bytes) - 1));
    bytes[26] = 1;
    CHECK(!CO_402_command_unpack(&output, bytes, sizeof(bytes)));
    bytes[26] = 0;
    bytes[25] = 0xff;
    CHECK(!CO_402_command_unpack(&output, bytes, sizeof(bytes)));
    bytes[25] = 1;
    bytes[3] = 0x80;
    CHECK(!CO_402_command_unpack(&output, bytes, sizeof(bytes)));
}

static void test_feedback_round_trip(void) {
    uint8_t bytes[CO_402_FEEDBACK_FRAME_SIZE];
    CO_402_feedback_frame_t input = {
        .epoch = 7,
        .feedback_seq = 5,
        .command_seq = 9,
        .action_seq = 3,
        .actual_position = INT32_MIN,
        .actual_velocity = INT32_MAX,
        .fault_code = 0x8123,
        .action_result = CO_402_ACTION_RESULT_BUSY,
        .flags = 0x1f
    };
    CO_402_feedback_frame_t output;
    CHECK(CO_402_feedback_pack(&input, bytes, sizeof(bytes)));
    CHECK(CO_402_feedback_unpack(&output, bytes, sizeof(bytes)));
    CHECK(input.epoch == output.epoch && input.feedback_seq == output.feedback_seq
        && input.command_seq == output.command_seq && input.action_seq == output.action_seq
        && input.actual_position == output.actual_position && input.actual_velocity == output.actual_velocity
        && input.fault_code == output.fault_code && input.action_result == output.action_result
        && input.flags == output.flags);
    CHECK(!CO_402_feedback_pack(&input, bytes, sizeof(bytes) + 1));
    bytes[34] = 1;
    CHECK(!CO_402_feedback_unpack(&output, bytes, sizeof(bytes)));
    bytes[34] = 0;
    bytes[2] = 4;
    CHECK(!CO_402_feedback_unpack(&output, bytes, sizeof(bytes)));
    bytes[2] = CO_402_ACTION_RESULT_DONE;
    bytes[3] = 0;
    CHECK(!CO_402_feedback_unpack(&output, bytes, sizeof(bytes)));
}

int main(void) {
    test_command_round_trip();
    test_feedback_round_trip();
    if (failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    puts("drive402 protocol tests passed");
    return 0;
}
