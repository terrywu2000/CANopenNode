#ifndef XNET_CANOPEN_EXTENSIONS_H
#define XNET_CANOPEN_EXTENSIONS_H

#include "xnet_canopen.h"
#include "motor_drive.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Versioned system snapshots. v1 layouts are frozen; capacity is in bytes. */
#define XNET_CO_SYSTEM_ABI_VERSION 1U
enum {
    XNET_CO_DRIVE_NONE = 0,
    XNET_CO_DRIVE_INTERNAL = 1,
    XNET_CO_DRIVE_EXTERNAL = 2
};
enum {
    XNET_CO_CAP_OD_BATCH = 1U << 0,
    XNET_CO_CAP_TPDO_REQUEST = 1U << 1,
    XNET_CO_CAP_MANUFACTURER_FAULT = 1U << 2,
    XNET_CO_CAP_SYNC_OBSERVATION = 1U << 3,
    XNET_CO_CAP_MOTOR_ASYNC = 1U << 4,
    XNET_CO_CAP_MOTOR_BOOTSTRAP = 1U << 5,
    XNET_CO_CAP_MOTOR_SAFETY = 1U << 6,
    XNET_CO_CAP_MOTOR_SCHEDULED = 1U << 7
};
enum {
    XNET_CO_MODE_PROFILE_POSITION = 1U << 0
};
enum {
    XNET_CO_PDS_NOT_READY = 0,
    XNET_CO_PDS_SWITCH_ON_DISABLED = 1,
    XNET_CO_PDS_READY_TO_SWITCH_ON = 2,
    XNET_CO_PDS_SWITCHED_ON = 3,
    XNET_CO_PDS_OPERATION_ENABLED = 4,
    XNET_CO_PDS_QUICK_STOP_ACTIVE = 5,
    XNET_CO_PDS_FAULT_REACTION_ACTIVE = 6,
    XNET_CO_PDS_FAULT = 7
};
enum {
    XNET_CO_DRIVE_STATUS_FEEDBACK_FRESH = 1U << 0,
    XNET_CO_DRIVE_STATUS_MODEL_FAULT = 1U << 1,
    XNET_CO_DRIVE_STATUS_MODEL_READY = 1U << 2,
    XNET_CO_DRIVE_STATUS_ACTION_PENDING = 1U << 3,
    XNET_CO_DRIVE_STATUS_OUTPUT_ALLOWED = 1U << 4,
    XNET_CO_DRIVE_STATUS_REACTION_UNCONFIRMED = 1U << 5,
    XNET_CO_DRIVE_STATUS_PORT_OPEN = 1U << 6
};
typedef struct {
    uint32_t abi_version, struct_size;
    xnet_canopen_status_t communication;
    uint32_t drive_valid;       /**< Remaining drive fields are valid only when 1. */
    uint32_t pds_state;         /**< XNET_CO_PDS_*, independent of NMT. */
    uint32_t statusword;
    int32_t mode_display;
    uint32_t drive_flags;       /**< XNET_CO_DRIVE_STATUS_* */
    uint32_t action;            /**< MOTOR_DRIVE_ACTION_* */
    uint32_t action_seq;
} xnet_canopen_system_status_v1;
typedef struct {
    uint32_t abi_version, struct_size;
    uint32_t drive_model;       /**< XNET_CO_DRIVE_* configured for this device. */
    uint32_t capabilities;      /**< XNET_CO_CAP_* */
    uint32_t implemented_modes; /**< XNET_CO_MODE_*, not the device's 0x6502. */
} xnet_canopen_capabilities_v1;

enum {
    XNET_CO_DIAG_NONE = 0,
    XNET_CO_DIAG_XNET = 1,      /**< source_code is XNET_CO_*, not a raw NI-XNET status. */
    XNET_CO_DIAG_MOTOR = 2      /**< source_code is MOTOR_DRIVE_*. */
};
typedef struct {
    uint32_t abi_version, struct_size;
    uint32_t source_domain;     /**< XNET_CO_DIAG_* */
    int32_t source_code;
    int32_t backend_code;       /**< Legacy XNET_CO_* code; motor rejections may use a mapped code. */
    uint32_t message_required;  /**< Bytes including NUL. */
    char message[2048];
} xnet_canopen_diagnostic_v1;

/** Snapshot communication and 402 state under the same runtime lock.
 * The communication fields retain get_status semantics. drive_valid is zero
 * when no drive core exists; retained drive data can be historical after stop
 * or failure. No timing or physical safety guarantee is implied.
 * Failure leaves output unchanged; capacity must be at least sizeof(*status).
 */
XNET_CANOPEN_API int32_t xnet_canopen_get_system_status_v1(
    xnet_canopen_handle handle, xnet_canopen_system_status_v1* status, uint32_t capacity);
/** Query capabilities of the initialized configuration, not EDS claims or
 * current readiness. SCHEDULED is not supported by the deployed backend.
 * Failure leaves output unchanged; capacity must be at least sizeof(*capabilities).
 */
XNET_CANOPEN_API int32_t xnet_canopen_get_capabilities_v1(
    xnet_canopen_handle handle, xnet_canopen_capabilities_v1* capabilities, uint32_t capacity);
/** Latest per-device diagnostic, including motor errors and their backend code.
 * Synchronized, not call-correlated: another thread may replace it. A motor
 * operation without an associated live device cannot record a diagnostic;
 * a failed runtime retains its worker diagnostic during existing port exchange.
 * Retrieval never clears/replaces it, including on invalid output/capacity.
 * Failure leaves output unchanged; capacity must be at least sizeof(*diagnostic).
 */
XNET_CANOPEN_API int32_t xnet_canopen_get_diagnostic_v1(
    xnet_canopen_handle handle, xnet_canopen_diagnostic_v1* diagnostic, uint32_t capacity);

/** Backend-only endpoint binding; requires external PP and device_adapter.
 * Endpoint is borrowed until uninitialize. No existing xnet ABI is changed.
 */
XNET_CANOPEN_API int32_t xnet_canopen_get_motor_drive_endpoint(
    xnet_canopen_handle handle, motor_drive_endpoint_handle* endpoint);
/** JSON conversion audit: converted, unchanged, assumed_identity and unbound.
 * Required byte count includes NUL; a NULL/0 size query returns BUFFER_TOO_SMALL.
 */
XNET_CANOPEN_API int32_t xnet_canopen_get_drive_unit_report(
    xnet_canopen_handle handle, char* buffer, uint32_t capacity, uint32_t* required_size);

/** A received SYNC observation, not a processed control boundary. The sequence
 * is local to this handle and epoch; independent ports cannot compare sequences.
 * timestamp_100ns is the raw XNET timestamp and is not mapped to PCL time.
 */
typedef struct {
    uint64_t epoch;
    uint64_t sequence;
    uint64_t timestamp_100ns;
    uint32_t can_id;
    uint32_t counter;
    uint32_t counter_present;
    uint32_t reserved;
} xnet_canopen_sync_event_t;

/** Read the next observation after (epoch, sequence). Pass (0, 0) to start at
 * the oldest retained event. NOT_FOUND means no later event; ACCESS_ERROR means
 * the cursor was overwritten; INVALID_STATE means its epoch is no longer active.
 * Restart, protocol reset, SYNC COB-ID change, RX overflow and LSS silence
 * invalidate the epoch. The producer counter is not checked for continuity.
 * History is bounded to 64 records. Output is usable only on OK. This API does
 * not imply the event was processed by the control loop or establish a clock map.
 */
XNET_CANOPEN_API int32_t xnet_canopen_read_sync_event(
    xnet_canopen_handle handle, uint64_t epoch, uint64_t sequence, xnet_canopen_sync_event_t* event);

/** Internal test-timing diagnostics; these events do not enable scheduled mode. */
#define XNET_CO_TIMING_EVENT_ABI_VERSION 1U
enum {
    XNET_CO_TIMING_COMMAND_PUBLISHED = 1,
    XNET_CO_TIMING_COMMAND_ASSIGNED = 2,
    XNET_CO_TIMING_FEEDBACK_ACCEPTED = 3,
    XNET_CO_TIMING_FEEDBACK_SELECTED = 4,
    XNET_CO_TIMING_TPDO_ACCEPTED = 5,
    XNET_CO_TIMING_FAILURE = 6,
    XNET_CO_TIMING_FEEDBACK_REJECTED = 7
};
enum {
    XNET_CO_TIMING_REASON_NONE = 0,
    XNET_CO_TIMING_REASON_OTHER = 1,
    XNET_CO_TIMING_REASON_RPDO = 2,
    XNET_CO_TIMING_REASON_CLOCK = 3,
    XNET_CO_TIMING_REASON_WATERMARK = 4,
    XNET_CO_TIMING_REASON_HISTORY_OVERFLOW = 5,
    XNET_CO_TIMING_REASON_HISTORY_GAP = 6,
    XNET_CO_TIMING_REASON_HISTORY_INVALID = 7,
    XNET_CO_TIMING_REASON_STALE = 8,
    XNET_CO_TIMING_REASON_DEADLINE = 9,
    XNET_CO_TIMING_REASON_TRANSPORT = 10
};
enum {
    XNET_CO_TIMING_HAS_BOUNDARY = 1U << 0,
    XNET_CO_TIMING_HAS_FRAME = 1U << 1,
    XNET_CO_TIMING_HAS_FRAME_TIMESTAMP = 1U << 2,
    XNET_CO_TIMING_HAS_SOURCE = 1U << 3,
    XNET_CO_TIMING_HAS_TARGET = 1U << 4,
    XNET_CO_TIMING_HAS_FEEDBACK = 1U << 5,
    XNET_CO_TIMING_HAS_SOURCE_TIME = 1U << 6,
    XNET_CO_TIMING_HAS_AVAILABLE_TIME = 1U << 7,
    XNET_CO_TIMING_HAS_DECISION_TIME = 1U << 8,
    XNET_CO_TIMING_XNET_ACCEPTED = 1U << 9,
    XNET_CO_TIMING_HAS_SOURCE_CYCLE = 1U << 10,
    XNET_CO_TIMING_HAS_CUTOFF = 1U << 11,
    XNET_CO_TIMING_HAS_TARGET_TIME = 1U << 12,
    XNET_CO_TIMING_HAS_SUBMITTED_STEADY_TIME = 1U << 13
};
typedef struct {
    uint32_t abi_version, struct_size, kind, reason;
    uint64_t sequence, drive_epoch;
    uint32_t node_id, flags;
    xnet_canopen_sync_event_t boundary;
    uint32_t frame_can_id, feedback_seq;
    uint64_t frame_timestamp_100ns;
    motor_drive_token_v1 source;
    motor_drive_cycle_v1 source_cycle;
    motor_drive_model_step_v1 target_step, executed_step;
    uint64_t source_us, available_us, decision_us, cutoff_us, target_us;
    int64_t submitted_steady_us;
    char message[128];
} xnet_canopen_timing_event_v1;

/** Read the next event after sequence, or use zero to begin at the oldest.
 * The handle-local sequence and bounded 128-event ring survive stop, reset,
 * uninitialize and reinitialize, and end at destroy. ACCESS_ERROR reports an
 * overwritten cursor; NOT_FOUND means no later event; a future cursor is
 * INVALID_ARGUMENT. On sequence exhaustion STACK_ERROR is returned after the
 * final event. Output is unchanged on failure. Capacity is in bytes and must
 * be at least sizeof(xnet_canopen_timing_event_v1). Reading never clears or
 * replaces the last-error diagnostic. Flags identify fields actually known;
 * source_us/available_us/decision_us/cutoff_us/target_us use the injected
 * clock domain. submitted_steady_us is instead the local steady-clock time
 * of the first complete feedback staging, and must not be subtracted from
 * injected-clock timestamps without a separately verified mapping.
 * For COMMAND_PUBLISHED/ASSIGNED, boundary is the command's
 * source SYNC; for FEEDBACK_SELECTED it is the SYNC updating actual values;
 * for a TPDO event it is the associated preceding SYNC. A failure's boundary,
 * when present, is the SYNC being processed or associated with a pending TPDO.
 * FEEDBACK_ACCEPTED is a normal timed-history commit, not a submit or safety
 * acceptance. FEEDBACK_REJECTED reports worker rejection, not every API refusal.
 * TPDO_ACCEPTED is XNET queue acceptance, not proof of physical transmission.
 * The first timing invalidation is recorded; later API errors do not replace it.
 */
XNET_CANOPEN_API int32_t xnet_canopen_read_timing_event_v1(
    xnet_canopen_handle handle, uint64_t sequence, xnet_canopen_timing_event_v1* event, uint32_t capacity);

#ifdef __cplusplus
}
#endif
#endif
