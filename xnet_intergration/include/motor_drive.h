#ifndef MOTOR_DRIVE_H
#define MOTOR_DRIVE_H

#include <stdint.h>

#if defined(_WIN32)
#define MOTOR_DRIVE_API __declspec(dllexport)
#else
#define MOTOR_DRIVE_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_DRIVE_ABI_VERSION 1U

typedef struct motor_drive_endpoint* motor_drive_endpoint_handle;
typedef struct motor_drive_port* motor_drive_port_handle;

enum {
    MOTOR_DRIVE_OK = 0,
    MOTOR_DRIVE_NO_DATA = 1,
    MOTOR_DRIVE_INVALID_ARGUMENT = -1,
    MOTOR_DRIVE_INVALID_STATE = -2,
    MOTOR_DRIVE_BUSY = -3,
    MOTOR_DRIVE_STALE = -4,
    MOTOR_DRIVE_INVALID_FEEDBACK = -5,
    MOTOR_DRIVE_BUFFER_TOO_SMALL = -6,
    MOTOR_DRIVE_BACKEND_ERROR = -7,
    MOTOR_DRIVE_UNSUPPORTED = -8
};
enum {
    MOTOR_DRIVE_READ_ASYNC = 1,
    MOTOR_DRIVE_READ_BOOTSTRAP = 2,
    MOTOR_DRIVE_READ_SAFETY = 4,
    MOTOR_DRIVE_READ_SCHEDULED = 8,
    MOTOR_DRIVE_TIME_VALID = 1,
    MOTOR_DRIVE_TIME_HARDWARE = 2,
    MOTOR_DRIVE_OUTPUT_ALLOWED = 1,
    MOTOR_DRIVE_PP_READY = 2,
    MOTOR_DRIVE_SAFETY_DECELERATION_VALID = 4,
    MOTOR_DRIVE_POSITION_VALID = 1,
    MOTOR_DRIVE_VELOCITY_VALID = 2,
    MOTOR_DRIVE_FEEDBACK_PP_READY = 4,
    MOTOR_DRIVE_RESET_PERMITTED = 16
};
enum {
    MOTOR_DRIVE_FAULT_NONE = 0,
    MOTOR_DRIVE_FAULT_GENERIC = 1,
    MOTOR_DRIVE_FAULT_OVERCURRENT = 2,
    MOTOR_DRIVE_FAULT_OVERVOLTAGE = 3,
    MOTOR_DRIVE_FAULT_UNDERVOLTAGE = 4,
    MOTOR_DRIVE_FAULT_OVERTEMPERATURE = 5,
    MOTOR_DRIVE_FAULT_FEEDBACK = 6,
    MOTOR_DRIVE_FAULT_INTERNAL = 7,
    MOTOR_DRIVE_LIMIT_POSITION = 1,
    MOTOR_DRIVE_LIMIT_VELOCITY = 2,
    MOTOR_DRIVE_LIMIT_CURRENT = 4,
    MOTOR_DRIVE_LIMIT_TORQUE = 8
};
enum {
    MOTOR_DRIVE_ACTION_NONE = 0,
    MOTOR_DRIVE_ACTION_SHUTDOWN = 1,
    MOTOR_DRIVE_ACTION_SWITCH_ON = 2,
    MOTOR_DRIVE_ACTION_ENABLE_OPERATION = 3,
    MOTOR_DRIVE_ACTION_DISABLE_OPERATION = 4,
    MOTOR_DRIVE_ACTION_QUICK_STOP = 5,
    MOTOR_DRIVE_ACTION_FAULT_REACTION = 6,
    MOTOR_DRIVE_ACTION_FAULT_RESET = 7,
    MOTOR_DRIVE_ACTION_DISABLE_VOLTAGE = 8,
    MOTOR_DRIVE_RESULT_NONE = 0,
    MOTOR_DRIVE_RESULT_BUSY = 1,
    MOTOR_DRIVE_RESULT_DONE = 2,
    MOTOR_DRIVE_RESULT_ERROR = 3,
    MOTOR_DRIVE_PROCESSED_NONE = 0,
    MOTOR_DRIVE_PROCESSED_ACCEPTED = 1,
    MOTOR_DRIVE_PROCESSED_STALE = 2,
    MOTOR_DRIVE_PROCESSED_INVALID = 3
};

typedef struct {
    uint32_t clock_domain_id;
    uint32_t flags;
    double time_s;
} motor_drive_time_v1;
typedef struct {
    uint64_t sync_domain_id;
    uint64_t sync_domain_epoch;
    uint64_t sync_cycle_id;
    uint32_t sync_counter;
    uint32_t flags;
    motor_drive_time_v1 rx_time;
    motor_drive_time_v1 sync_time;
} motor_drive_cycle_v1;
typedef struct {
    uint64_t pcl_epoch;
    uint64_t pcl_tick;
    uint64_t model_step_seq;
    double model_time_s;
    double step_duration_s;
    motor_drive_time_v1 sample_time;
} motor_drive_model_step_v1;
typedef struct {
    uint64_t epoch;
    uint32_t command_seq;
    uint32_t action_seq;
} motor_drive_token_v1;
typedef struct {
    uint32_t abi_version;
    uint32_t struct_size;
    uint32_t flags;
    uint32_t reserved;
    motor_drive_model_step_v1 target_step;
    motor_drive_time_v1 source_cutoff;
} motor_drive_read_request_v1;
typedef struct {
    uint32_t abi_version;
    uint32_t struct_size;
    motor_drive_token_v1 token;
    motor_drive_cycle_v1 source_cycle;
    motor_drive_model_step_v1 target_step;
    motor_drive_time_v1 publish_time;
    uint32_t action;
    uint32_t flags;
    int32_t mode;
    uint32_t reserved;
    double demand_position_rad;
    double demand_velocity_rad_s;
    double safety_deceleration_rad_s2;
    uint32_t processed_feedback_seq;
    uint32_t processed_result;
} motor_drive_view_v1;
typedef struct {
    uint32_t abi_version;
    uint32_t struct_size;
    motor_drive_token_v1 source;
    motor_drive_cycle_v1 source_cycle;
    motor_drive_model_step_v1 executed_step;
    uint32_t feedback_seq;
    uint32_t action_result;
    uint32_t flags;
    uint32_t limit_flags;
    double actual_position_rad;
    double actual_velocity_rad_s;
    uint32_t fault_class;
    uint32_t fault_detail;
} motor_drive_feedback_v1;

/** Backend owns the endpoint; one port is the exclusive model writer.
 * Do not overlap close/destroy with calls. Close does not stop the drive.
 * Existing ports remain closable but cannot exchange after backend uninitialize.
 */
MOTOR_DRIVE_API int32_t motor_drive_open_v1(
    motor_drive_endpoint_handle endpoint, motor_drive_port_handle* port);
/** Output is usable only on OK. This backend implements ASYNC/BOOTSTRAP/SAFETY,
 * not SCHEDULED: SYNC/clock mapping is not inferred from worker iterations.
 * Unavailable time/cycle metadata is zero. Request target_step/cutoff are zero.
 */
MOTOR_DRIVE_API int32_t motor_drive_read_v1(
    motor_drive_port_handle port, const motor_drive_read_request_v1* request,
    motor_drive_view_v1* view, uint32_t capacity);
/** Full staging only; processed_* reports worker acceptance, not action DONE.
 * One unacknowledged feedback; identical retries never refresh sample age.
 * Fault class/detail requires an explicit backend device-code mapping.
 */
MOTOR_DRIVE_API int32_t motor_drive_submit_v1(
    motor_drive_port_handle port, const motor_drive_feedback_v1* feedback, uint32_t size);
/** Forward actual model step progress, not CD loop progress. A duplicate is
 * idempotent; a new heartbeat sequence with an unchanged model step is stale.
 * Restart the node/session when the PCL epoch changes.
 */
MOTOR_DRIVE_API int32_t motor_drive_heartbeat_v1(
    motor_drive_port_handle port, const motor_drive_token_v1* source,
    const motor_drive_model_step_v1* progress, uint32_t heartbeat_seq);
MOTOR_DRIVE_API int32_t motor_drive_close_v1(motor_drive_port_handle port);

#ifdef __cplusplus
}
#endif
#endif
