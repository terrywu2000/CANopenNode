#ifndef CO_402_PROTOCOL_H
#define CO_402_PROTOCOL_H

/**
 * @file
 * Version-1, little-endian DS402 model exchange frames.
 *
 * The wire-independent C structs are logical values only; callers must use the
 * pack/unpack functions rather than copying struct memory into the OD.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CO_402_PROTOCOL_VERSION 1U
#define CO_402_COMMAND_FRAME_SIZE 56U
#define CO_402_FEEDBACK_FRAME_SIZE 36U

#define CO_402_CMD_OFF_VERSION 0U
#define CO_402_CMD_OFF_LENGTH 1U
#define CO_402_CMD_OFF_ACTION 2U
#define CO_402_CMD_OFF_FLAGS 3U
#define CO_402_CMD_OFF_EPOCH 4U
#define CO_402_CMD_OFF_COMMAND_SEQ 12U
#define CO_402_CMD_OFF_ACTION_SEQ 16U
#define CO_402_CMD_OFF_CONTROLWORD 20U
#define CO_402_CMD_OFF_STATUSWORD 22U
#define CO_402_CMD_OFF_PDS 24U
#define CO_402_CMD_OFF_MODE 25U
#define CO_402_CMD_OFF_RESERVED 26U
#define CO_402_CMD_OFF_DEMAND_POSITION 28U
#define CO_402_CMD_OFF_TARGET_POSITION 32U
#define CO_402_CMD_OFF_PROFILE_VELOCITY 36U
#define CO_402_CMD_OFF_PROFILE_ACCELERATION 40U
#define CO_402_CMD_OFF_PROFILE_DECELERATION 44U
#define CO_402_CMD_OFF_QUICK_STOP_DECELERATION 48U
#define CO_402_CMD_OFF_DEMAND_VELOCITY 52U

#define CO_402_FB_OFF_VERSION 0U
#define CO_402_FB_OFF_LENGTH 1U
#define CO_402_FB_OFF_ACTION_RESULT 2U
#define CO_402_FB_OFF_FLAGS 3U
#define CO_402_FB_OFF_EPOCH 4U
#define CO_402_FB_OFF_FEEDBACK_SEQ 12U
#define CO_402_FB_OFF_COMMAND_SEQ 16U
#define CO_402_FB_OFF_ACTION_SEQ 20U
#define CO_402_FB_OFF_ACTUAL_POSITION 24U
#define CO_402_FB_OFF_ACTUAL_VELOCITY 28U
#define CO_402_FB_OFF_FAULT_CODE 32U
#define CO_402_FB_OFF_RESERVED 34U

typedef enum {
    CO_402_ACTION_NONE = 0,
    CO_402_ACTION_SHUTDOWN = 1,
    CO_402_ACTION_SWITCH_ON = 2,
    CO_402_ACTION_ENABLE_OPERATION = 3,
    CO_402_ACTION_DISABLE_OPERATION = 4,
    CO_402_ACTION_QUICK_STOP = 5,
    CO_402_ACTION_FAULT_REACTION = 6,
    CO_402_ACTION_FAULT_RESET = 7,
    CO_402_ACTION_DISABLE_VOLTAGE = 8
} CO_402_action_t;

typedef enum {
    CO_402_ACTION_RESULT_NONE = 0,
    CO_402_ACTION_RESULT_BUSY = 1,
    CO_402_ACTION_RESULT_DONE = 2,
    CO_402_ACTION_RESULT_ERROR = 3
} CO_402_action_result_t;

typedef struct {
    uint64_t epoch;
    uint32_t command_seq;
    uint32_t action_seq;
    uint16_t controlword;
    uint16_t statusword;
    uint8_t action;
    uint8_t flags; /* bit 0: output allowed, bit 1: PP model ready */
    uint8_t pds;
    int8_t mode;
    int32_t demand_position;
    int32_t target_position;
    uint32_t profile_velocity;
    uint32_t profile_acceleration;
    uint32_t profile_deceleration;
    uint32_t quick_stop_deceleration;
    int32_t demand_velocity;
} CO_402_command_frame_t;

typedef struct {
    uint64_t epoch;
    uint32_t feedback_seq;
    uint32_t command_seq;
    uint32_t action_seq;
    int32_t actual_position;
    int32_t actual_velocity;
    uint16_t fault_code;
    uint8_t action_result;
    uint8_t flags; /* bits 0..4: position valid, velocity valid, PP ready, limited, reset permitted */
} CO_402_feedback_frame_t;

bool CO_402_command_pack(const CO_402_command_frame_t* frame, uint8_t* bytes, uint32_t size);
bool CO_402_command_unpack(CO_402_command_frame_t* frame, const uint8_t* bytes, uint32_t size);
bool CO_402_feedback_pack(const CO_402_feedback_frame_t* frame, uint8_t* bytes, uint32_t size);
bool CO_402_feedback_unpack(CO_402_feedback_frame_t* frame, const uint8_t* bytes, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif
