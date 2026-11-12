/**
 * @file
 * Strict version-1 DS402 model exchange frame codec.
 */

#include "CO_402_protocol.h"

#include <string.h>

static void put_u16(uint8_t* p, uint16_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t* p, uint32_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static void put_u64(uint8_t* p, uint64_t value) {
    put_u32(p, (uint32_t)value);
    put_u32(p + 4, (uint32_t)(value >> 32));
}

static uint16_t get_u16(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t get_i32(const uint8_t* p) {
    const uint32_t value = get_u32(p);
    if (value <= INT32_MAX) return (int32_t)value;
    return -1 - (int32_t)(UINT32_MAX - value);
}

static uint64_t get_u64(const uint8_t* p) {
    return (uint64_t)get_u32(p) | ((uint64_t)get_u32(p + 4) << 32);
}

static bool command_valid(const CO_402_command_frame_t* f) {
    return f != NULL && f->epoch != 0 && f->command_seq != 0 && f->action_seq != 0
        && f->action <= CO_402_ACTION_DISABLE_VOLTAGE && (f->flags & 0xfcU) == 0
        && f->pds <= 7 && (f->mode == 0 || f->mode == 1);
}

static bool feedback_valid(const CO_402_feedback_frame_t* f) {
    return f != NULL && f->epoch != 0 && f->feedback_seq != 0
        && f->command_seq != 0 && f->action_seq != 0
        && f->action_result <= CO_402_ACTION_RESULT_ERROR
        && (f->flags & 0xe0U) == 0 && (f->flags & 0x01U) != 0;
}

bool CO_402_command_pack(const CO_402_command_frame_t* f, uint8_t* b, uint32_t size) {
    if (!command_valid(f) || b == NULL || size != CO_402_COMMAND_FRAME_SIZE) return false;
    memset(b, 0, size);
    b[0] = CO_402_PROTOCOL_VERSION;
    b[1] = CO_402_COMMAND_FRAME_SIZE;
    b[2] = f->action;
    b[3] = f->flags;
    put_u64(b + 4, f->epoch);
    put_u32(b + 12, f->command_seq);
    put_u32(b + 16, f->action_seq);
    put_u16(b + 20, f->controlword);
    put_u16(b + 22, f->statusword);
    b[24] = f->pds;
    b[25] = (uint8_t)f->mode;
    put_u32(b + 28, (uint32_t)f->demand_position);
    put_u32(b + 32, (uint32_t)f->target_position);
    put_u32(b + 36, f->profile_velocity);
    put_u32(b + 40, f->profile_acceleration);
    put_u32(b + 44, f->profile_deceleration);
    put_u32(b + 48, f->quick_stop_deceleration);
    put_u32(b + 52, (uint32_t)f->demand_velocity);
    return true;
}

bool CO_402_command_unpack(CO_402_command_frame_t* f, const uint8_t* b, uint32_t size) {
    if (f == NULL || b == NULL || size != CO_402_COMMAND_FRAME_SIZE
        || b[0] != CO_402_PROTOCOL_VERSION || b[1] != CO_402_COMMAND_FRAME_SIZE
        || b[25] > 1 || b[26] != 0 || b[27] != 0) return false;
    f->action = b[2];
    f->flags = b[3];
    f->epoch = get_u64(b + 4);
    f->command_seq = get_u32(b + 12);
    f->action_seq = get_u32(b + 16);
    f->controlword = get_u16(b + 20);
    f->statusword = get_u16(b + 22);
    f->pds = b[24];
    f->mode = (int8_t)b[25];
    f->demand_position = get_i32(b + 28);
    f->target_position = get_i32(b + 32);
    f->profile_velocity = get_u32(b + 36);
    f->profile_acceleration = get_u32(b + 40);
    f->profile_deceleration = get_u32(b + 44);
    f->quick_stop_deceleration = get_u32(b + 48);
    f->demand_velocity = get_i32(b + 52);
    return command_valid(f);
}

bool CO_402_feedback_pack(const CO_402_feedback_frame_t* f, uint8_t* b, uint32_t size) {
    if (!feedback_valid(f) || b == NULL || size != CO_402_FEEDBACK_FRAME_SIZE) return false;
    memset(b, 0, size);
    b[0] = CO_402_PROTOCOL_VERSION;
    b[1] = CO_402_FEEDBACK_FRAME_SIZE;
    b[2] = f->action_result;
    b[3] = f->flags;
    put_u64(b + 4, f->epoch);
    put_u32(b + 12, f->feedback_seq);
    put_u32(b + 16, f->command_seq);
    put_u32(b + 20, f->action_seq);
    put_u32(b + 24, (uint32_t)f->actual_position);
    put_u32(b + 28, (uint32_t)f->actual_velocity);
    put_u16(b + 32, f->fault_code);
    return true;
}

bool CO_402_feedback_unpack(CO_402_feedback_frame_t* f, const uint8_t* b, uint32_t size) {
    if (f == NULL || b == NULL || size != CO_402_FEEDBACK_FRAME_SIZE
        || b[0] != CO_402_PROTOCOL_VERSION || b[1] != CO_402_FEEDBACK_FRAME_SIZE
        || b[34] != 0 || b[35] != 0) return false;
    f->action_result = b[2];
    f->flags = b[3];
    f->epoch = get_u64(b + 4);
    f->feedback_seq = get_u32(b + 12);
    f->command_seq = get_u32(b + 16);
    f->action_seq = get_u32(b + 20);
    f->actual_position = get_i32(b + 24);
    f->actual_velocity = get_i32(b + 28);
    f->fault_code = get_u16(b + 32);
    return feedback_valid(f);
}
