#ifndef XNET_CANOPEN_H
#define XNET_CANOPEN_H

/**
 * @file
 * C ABI for one CANopen slave simulated over NI-XNET.
 *
 * The library owns CAN transport, protocol processing and OD storage; an external
 * model supplies ordinary application behavior through batch access. A JSON model
 * may explicitly enable the integrated single-axis CiA 402 PP profile. PDO tx/rx
 * directions are relative to the simulated slave, not to the VeriStand model.
 *
 * Runtime state, CANopen NMT state and an application's drive state are separate.
 * Unless explicitly documented otherwise, output values are usable only after
 * checking the operation result and any required per-entry results.
 */
#include <stdint.h>
#include "motor_drive.h"

#if defined(_WIN32)
#define XNET_CANOPEN_API __declspec(dllexport)
#else
#define XNET_CANOPEN_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Opaque device owned by the caller until a successful destroy.
 * Do not use stale handles or overlap destroy with any other call.
 */
typedef struct xnet_canopen_device* xnet_canopen_handle;

/** Operation results and per-entry batch results; these are not runtime states. */
enum {
    XNET_CO_OK = 0,                 /**< Operation or batch entry completed successfully. */
    XNET_CO_INVALID_ARGUMENT = -1,  /**< Invalid pointer, count, value, or buffer layout. */
    XNET_CO_INVALID_STATE = -2,     /**< Lifecycle/NMT state forbids the operation. */
    XNET_CO_MODEL_ERROR = -3,       /**< JSON schema, OD description, or configured value is invalid. */
    XNET_CO_IO_ERROR = -4,          /**< File access, persistence, or cleanup failed. */
    XNET_CO_XNET_ERROR = -5,        /**< XNET call or adapter communication failed; inspect the message. */
    XNET_CO_STACK_ERROR = -6,       /**< CANopen initialization or protocol processing failed. */
    XNET_CO_NOT_FOUND = -7,         /**< OD variable or batch ID does not exist. */
    XNET_CO_ACCESS_ERROR = -8,      /**< OD permission, value validation, or callback transfer failed. */
    XNET_CO_BUFFER_TOO_SMALL = -9,  /**< Batch or diagnostic output buffer lacks capacity. */
    XNET_CO_PARTIAL_FAILURE = -10,  /**< At least one batch entry failed; inspect every entry result. */
    XNET_CO_INTERNAL_ERROR = -11   /**< Allocation or another unexpected internal operation failed. */
};

/** Standard CANopen data type codes; API type arrays use uint32_t storage. */
typedef enum {
    XNET_CO_TYPE_BOOLEAN = 0x0001,
    XNET_CO_TYPE_INTEGER8 = 0x0002,
    XNET_CO_TYPE_INTEGER16 = 0x0003,
    XNET_CO_TYPE_INTEGER32 = 0x0004,
    XNET_CO_TYPE_UNSIGNED8 = 0x0005,
    XNET_CO_TYPE_UNSIGNED16 = 0x0006,
    XNET_CO_TYPE_UNSIGNED32 = 0x0007,
    XNET_CO_TYPE_REAL32 = 0x0008,
    XNET_CO_TYPE_VISIBLE_STRING = 0x0009,
    XNET_CO_TYPE_OCTET_STRING = 0x000A,
    XNET_CO_TYPE_REAL64 = 0x0011,
    XNET_CO_TYPE_INTEGER64 = 0x0015,
    XNET_CO_TYPE_UNSIGNED64 = 0x001B
} xnet_canopen_data_type_t;

/** Values of xnet_canopen_status_t.runtime_state; independent of CANopen NMT state. */
enum {
    XNET_CO_RUNTIME_UNINITIALIZED = 0, /**< Context exists without an initialized device. */
    XNET_CO_RUNTIME_STOPPED = 1,       /**< Initialized resources are retained; worker is not running. */
    XNET_CO_RUNTIME_RUNNING = 2,       /**< Worker owns processing; node may still await LSS/NMT. */
    XNET_CO_RUNTIME_FAILED = 3         /**< Runtime/setup failed; uninitialize before reinitializing. */
};

/**
 * Synchronized 48-byte status snapshot, represented by twelve consecutive uint32s.
 * The first ten fields retain their previous offsets; can_state_raw and
 * can_state_valid are appended at byte offsets 40 and 44. This extends the former
 * 40-byte ABI: rebuild C callers and enlarge LabVIEW output descriptors before calling.
 * get_status has no output-size argument and cannot detect an old/small buffer.
 *
 * Reset counters belong to the handle, not to the resettable Object Dictionary.
 * Each counts successful protocol-reset completions, not requests or boot-up
 * frames. Reset Node increments only reset_node_count, even though it also
 * reinitializes communication. Failed resets, ordinary initialization, and host
 * stop/start do not increment either counter.
 *
 * Both counters start at zero and clear only on successful initialize. They
 * survive protocol resets, stop/start, uninitialize, and failed initialize
 * attempts. All fields are zero on a newly created handle; without a stack,
 * fields other than runtime_state and the retained counters are zero.
 * Unsigned counters wrap modulo 2^32. Compare against the previous snapshot to
 * detect resets; uint32 subtraction gives the count modulo 2^32, not event order.
 * A full wrap between reads is indistinguishable from no change. Establish a
 * new baseline after initialization; values are not persisted across handles.
 *
 * node_guarding_timeout is a current condition, not a latched event history.
 * A new guarding request, disabled monitoring, enabled heartbeat, or stack
 * restart/reset clears it. The external model decides the drive-level reaction;
 * the communication stack independently applies its NMT/error-reporting policy.
 *
 * can_state_raw caches the last NI-XNET CAN communication state for which both
 * nxReadState and its fault output were successful. can_state_valid is 1 once
 * such a sample exists; raw zero is a valid healthy sample, not "unavailable".
 * The cache survives stop, protocol reset and runtime failure for diagnosis.
 * It is historical while stopped/failed or during LSS silence, not a live health
 * guarantee even when valid. Initialize attempts and successful uninitialize
 * clear it. get_status and clear_error neither poll hardware nor clear the cache.
 *
 * Decode raw bits: 0..3 communication (0 active, 1 passive, 2 bus-off, 3 init),
 * bit 4 transceiver error, bit 5 sleep, bits 8..11 last bus error (0 none,
 * 1 stuff, 2 form, 3 ACK, 4 bit-1, 5 bit-0, 6 CRC), bits 16..23 TX error
 * count, bits 24..31 RX error count. Preserve unknown codes; the last bus
 * error is historical, not necessarily an active fault or an NI-XNET status code.
 */
typedef struct {
    uint32_t runtime_state;        /**< One of XNET_CO_RUNTIME_*, not an API result. */
    uint32_t nmt_state;            /**< CANopen NMT state value; 0 while initializing/unconfigured. */
    uint32_t active_node_id;       /**< Current node ID, or 255 while awaiting LSS assignment. */
    uint32_t pending_node_id;      /**< LSS-configured node ID, applied at communication reset. */
    uint32_t active_bitrate_kbps;  /**< Current physical interface bitrate in kbit/s. */
    uint32_t pending_bitrate_kbps; /**< LSS-configured bitrate, applied by activation. */
    uint32_t can_error_status;     /**< CANopenNode CO_CAN_ERR_* bitmask, not an API result. */
    uint32_t node_guarding_timeout; /**< 1 while life guarding has timed out; otherwise 0. */
    uint32_t reset_node_count;     /**< Completed Reset Node operations; application OD defaults were restored. */
    uint32_t reset_communication_count; /**< Completed Reset Communication operations; application OD values were retained. */
    uint32_t can_state_raw;        /**< Last successful NI-XNET CAN state; decode only when can_state_valid is 1. */
    uint32_t can_state_valid;      /**< 1 if a successful controller sample is cached, otherwise 0. */
} xnet_canopen_status_t;

/**
 * Allocate an uninitialized context and write its handle to a required output.
 * Sets *handle to NULL on allocation failure. No hardware resources are acquired.
 * All APIs return XNET_CO_* operation results; negative values indicate failure.
 */
XNET_CANOPEN_API int32_t xnet_canopen_create(xnet_canopen_handle* handle);

/**
 * Stop and release all resources, then invalidate the handle on success.
 * Caller must prevent concurrent/new calls. On failure the handle remains valid
 * for diagnostics and a cleanup retry. NULL is an invalid argument.
 */
XNET_CANOPEN_API int32_t xnet_canopen_destroy(xnet_canopen_handle handle);

/**
 * Open an interface and construct the OD/stack from a version-1 JSON model.
 * Requires an uninitialized handle and nonempty interface/model/persistence paths.
 * Strings are copied without normalization; caller ensures correct names/paths
 * and exclusive interface/persistence-file use. Missing or unloadable persistence
 * uses model startup settings; load exceptions are recorded in diagnostics
 * without failing initialization. Sessions are not started.
 * Failure releases partial resources when possible; inspect diagnostics and
 * retry uninitialize if cleanup also fails. See README.md for the JSON schema.
 * Generated EDS templates must have every {USER_INPUT} placeholder replaced.
 * Optional NODE_ID offsets are resolved by the model; no expression evaluator
 * or template substitution is performed by this API.
 * The macro-configured log file is opened before XNET sessions; logging startup
 * failure returns XNET_CO_IO_ERROR. Later logging failures are reported through
 * diagnostics/stderr without rolling back communication operations.
 */
XNET_CANOPEN_API int32_t xnet_canopen_initialize(
    xnet_canopen_handle handle, const char* interface_name,
    const char* model_path, const char* persistence_path);

/**
 * Stop/join the worker and release sessions, stack, OD, and batch IDs.
 * Retains the context and diagnostics for another initialize. Also succeeds for
 * an already uninitialized context; cleanup failure is reported and retryable.
 */
XNET_CANOPEN_API int32_t xnet_canopen_uninitialize(xnet_canopen_handle handle);

/**
 * Start/restart a stopped initialized device; already running is a no-op.
 * Rebuilds protocol state while retaining OD values/bindings. A configured node
 * emits boot-up and starts Pre-operational, not NMT Operational. Inspect status
 * for later asynchronous failures; a FAILED runtime must first be uninitialized.
 */
XNET_CANOPEN_API int32_t xnet_canopen_start(xnet_canopen_handle handle);

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

/**
 * Stop/join processing and discard pending network traffic, retaining OD values,
 * configuration, bindings, and sessions. Already stopped is a no-op.
 * Requires initialization. This is not CANopen NMT Stop, which keeps processing.
 */
XNET_CANOPEN_API int32_t xnet_canopen_stop(xnet_canopen_handle handle);

/**
 * Bind 1..4096 distinct OD variables while initialized and not FAILED.
 * indices/subindices are inputs; types/sizes are required caller-allocated
 * uint32 output arrays. Each array has count elements. Types are XNET_CO_TYPE_*
 * codes; sizes are exact model lengths, including full string slots and NUL
 * capacity. buffer_size receives their sum; batch_id receives the new ID.
 * All outputs remain unchanged on failure. No caller array pointer is retained.
 * Buffer slots follow array order, tightly packed without padding: offset[i] is
 * the sum of preceding sizes. Total size must be <=1 MiB.
 * IDs survive stop/start and master resets, but not uninitialize. Prefer binding
 * during setup; no caller array pointer is retained. In LabVIEW, pass numeric
 * arrays as Array Data Pointer, not array handles; verify their lengths first.
 * Binding validates addresses and sizes, not permission for a later operation.
 * Keep master-owned command reads separate from model-owned feedback writes.
 */
XNET_CANOPEN_API int32_t xnet_canopen_bind_batch(
    xnet_canopen_handle handle, const uint16_t* indices,
    const uint8_t* subindices, uint32_t count, uint32_t* types,
    uint32_t* sizes, uint32_t* buffer_size, uint32_t* batch_id);

/**
 * Read a bound group while initialized and not FAILED.
 * buffer_size is byte capacity (at least the sum of bound sizes). entry_status
 * is a required caller-allocated int32 output array with entry_count elements;
 * entry_count must equal the bound variable count, not the byte-buffer size.
 * entry_status[i] reports the result for variable i in binding order.
 * Call-wide errors leave entry_status unchanged. Numeric values are little-endian;
 * string slots have fixed capacity. Only successful entry results identify valid
 * output. Entry failures do not stop later entries; overall result is
 * XNET_CO_PARTIAL_FAILURE if any entry fails. OD callbacks run under the OD lock.
 * Local reads currently require SDO-readable metadata, even if an object can be
 * written by the master. A successful read reports the stored value, not its age
 * or proof that a new RPDO/SDO command arrived since the previous read.
 */
XNET_CANOPEN_API int32_t xnet_canopen_read_batch(
    xnet_canopen_handle handle, uint32_t batch_id, uint8_t* buffer,
    uint32_t buffer_size, int32_t* entry_status, uint32_t entry_count);

/**
 * Attempt every bound entry while initialized and not FAILED.
 * Buffer/count/status rules match read_batch. The caller decides when to invoke
 * a write; there is no enable mask. Bind separate groups for independent writes.
 * Values need model local_write permission; VISIBLE_STRING slots require NUL.
 * Call-wide validation precedes writes. Entry failures allow later writes and
 * return XNET_CO_PARTIAL_FAILURE, without rollback of earlier successes.
 * Serialized writes are last-successful-write-wins; no TPDO is requested here.
 * local_write is independent of SDO access: SDO read-only feedback can be written
 * by the model when local_write is true. SDO read/write permission alone does
 * not authorize local writes. Successful local writes do not emit EMCY or run
 * an application state machine; the caller explicitly supplies those actions.
 */
XNET_CANOPEN_API int32_t xnet_canopen_write_batch(
    xnet_canopen_handle handle, uint32_t batch_id, const uint8_t* buffer,
    uint32_t buffer_size, int32_t* entry_status, uint32_t entry_count);

/**
 * Read one OD variable without binding, for diagnostics and tests.
 * Lifecycle, lock, permission, and callback rules match one read_batch entry;
 * the entry result is returned directly. value_size is required and receives the
 * exact OD length once the variable is found. NULL buffer with capacity 0 queries
 * the size and returns BUFFER_TOO_SMALL without changing diagnostics.
 * Each call locks separately; use read_batch for cyclic or consistent multi-value I/O.
 */
XNET_CANOPEN_API int32_t xnet_canopen_read_od(
    xnet_canopen_handle handle, uint16_t index, uint8_t subindex,
    uint8_t* buffer, uint32_t buffer_size, uint32_t* value_size);

/**
 * Write one OD variable without binding, for diagnostics and tests.
 * buffer_size must equal the exact OD length. Permission, validation, and
 * no-TPDO/no-EMCY rules match one write_batch entry; the result is returned directly.
 */
XNET_CANOPEN_API int32_t xnet_canopen_write_od(
    xnet_canopen_handle handle, uint16_t index, uint8_t subindex,
    const uint8_t* buffer, uint32_t buffer_size);

/**
 * Request one enabled TPDO by one-based number on a running Operational node.
 * Types 254/255 obey inhibit time; type 0 waits for SYNC. Other types and requests
 * during bitrate switching are rejected. Requests may coalesce; success means
 * accepted by the stack, not transmitted/acknowledged on the physical bus.
 */
XNET_CANOPEN_API int32_t xnet_canopen_request_tpdo(
    xnet_canopen_handle handle, uint16_t tpdo_number);

/**
 * Assert/reset a simulated manufacturer fault on a running, configured node.
 * error_bit must be 0x40..0x4F and active must be 0 or 1. Asserting requires a
 * nonzero CANopen error_code; reset uses the no-error code. info_code is the
 * manufacturer diagnostic payload. The stack updates EMCY/error history; this
 * is separate from host API diagnostics and does not bypass NMT transmission rules.
 */
XNET_CANOPEN_API int32_t xnet_canopen_set_fault(
    xnet_canopen_handle handle, uint8_t error_bit, uint16_t error_code,
    uint32_t info_code, uint8_t active);

/**
 * Copy a read-only, synchronized snapshot, even before initialization or after
 * failure. Reading does not acknowledge/clear counters or the guarding condition.
 * Counters, NMT state, and guarding status are sampled under the same runtime
 * lock. Current state/timeout are not a history of intermediate transitions.
 *
 * The external model interprets reset counters; this call does not reset its
 * state or implement CiA 402 behavior. OD batch calls are separately locked:
 * a reset may occur between status and batch calls. For a read-only input
 * snapshot, bracket the batch read with status reads and discard/retry if either
 * reset counter changed. This does not make later feedback writes atomic with
 * the snapshot or prevent a reset between the final check and a write.
 */
XNET_CANOPEN_API int32_t xnet_canopen_get_status(
    xnet_canopen_handle handle, xnet_canopen_status_t* status);

/**
 * Copy the latest per-handle diagnostic without clearing or replacing it.
 * error_code and message_required are required outputs. message_required includes
 * NUL; NULL message with capacity 0 queries size and returns BUFFER_TOO_SMALL.
 * Code/required size are returned even when capacity is insufficient; no partial
 * message is copied. Messages are capped at 2047 bytes plus NUL. Successful
 * unrelated calls do not erase failures; diagnostics survive stop/uninitialize.
 * The returned operation result is distinct from the retrieved error_code.
 */
XNET_CANOPEN_API int32_t xnet_canopen_get_last_error(
    xnet_canopen_handle handle, int32_t* error_code, char* message,
    uint32_t message_capacity, uint32_t* message_required);

/** Clear only the latest host diagnostic; does not reset runtime state, CAN flags, or simulated faults. */
XNET_CANOPEN_API int32_t xnet_canopen_clear_error(xnet_canopen_handle handle);

#ifdef __cplusplus
}
#endif
#endif
