#ifndef XNET_CO_INTERNAL_HPP
#define XNET_CO_INTERNAL_HPP

/**
 * @file
 * Private ownership and synchronization model shared by the integration units.
 * Public callers see only the opaque handle in xnet_canopen.h. Stack-facing
 * pointers below borrow storage owned by Model or by the current CO_t allocation;
 * teardown and reset ordering must respect those different lifetimes.
 */
#define OD_DEFINITION
#include "CANopen.h"
#include "402/CO_402_protocol.h"
#include "402/drive402.hpp"
#include "timed_history.hpp"
#include "xnet_canopen_extensions.h"
#include "nlohmann/json.hpp"
#include "spdlog/common.h"
#include "xnet/nixnet.h"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace spdlog { class logger; }

namespace xco {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

enum class LogLevel { info, warning, error };
uint64_t next_log_instance() noexcept;
/** Open the macro-selected file before hardware acquisition; failure is an I/O error. */
std::shared_ptr<spdlog::logger> file_logger();

/** Internal exception carrying a public API result code and a diagnostic message. */
struct Error : std::runtime_error {
    int32_t code; /**< XNET_CO_* failure code; what() supplies the associated message. */
    Error(int32_t c, const std::string& text) : std::runtime_error(text), code(c) {}
};
void require(bool condition, int32_t code, const std::string& message);
void keys(const Json& value, std::initializer_list<const char*> allowed, const std::string& where);
/** Numeric range for configured Classical CAN rates in kbit/s; XNET checks hardware support. */
bool valid_bitrate(uint16_t kbps);
uint64_t unsigned_number(const Json& value, uint64_t maximum, const std::string& where);
/** Read one bounded-size JSON file and reject trailing non-whitespace content. */
Json read_json(const std::filesystem::path& path);
/** Replace one JSON file durably; errors after replacement do not imply rollback. */
void write_json_atomic(const std::filesystem::path& path, const Json& value);

/**
 * One Object Dictionary (OD) subentry: a scalar or a fixed-size string/byte buffer.
 * A VAR object has one Value at subindex 0; compound objects have multiple Values.
 * Owns the live bytes exposed to CANopenNode and an immutable default-byte snapshot.
 */
struct Value {
    /** Live storage rounded up to uint64_t words for typed OD pointer alignment. */
    std::vector<uint64_t> storage;
    std::vector<uint8_t> initial; /**< Exactly size default bytes, used for resets and COB-ID rebasing. */
    std::string type; /**< Model type name, e.g. UNSIGNED16, REAL32, or VISIBLE_STRING. */
    uint32_t size = 0; /**< Logical byte length, excluding storage padding; includes string NUL capacity. */
    uint8_t sub = 0; /**< Subindex within the containing OD object. */
    OD_attr_t attr = 0; /**< CANopenNode access, PDO mapping, string, and multibyte attribute flags. */
    bool allows_local_write = false; /**< Allows host batch writes; independent of network SDO write permission. */
    bool is_node_relative = false; /**< Live uint32_t COB-ID is its default base plus node ID (unless unconfigured). */
    bool default_node_relative = false; /**< Add node ID to an integer application default only when that default is restored. */
    bool mapped_rpdo = false; /**< Present in an enabled RPDO mapping. */
    bool mapped_tpdo = false; /**< Present in an enabled TPDO mapping. */
    /** Address published to OD descriptors; stable while storage is not resized. */
    void* data() { return storage.data(); }
};

enum class DriveQuantity : size_t {
    target_position, profile_velocity, profile_acceleration, profile_deceleration,
    quick_stop_deceleration, demand_position, demand_velocity, actual_position, actual_velocity, count
};

struct Model;
struct DriveUnitBinding {
    const char* role = nullptr;
    uint16_t index = 0;
    Value* value = nullptr;
    uint32_t numerator = 1, denominator = 1;
    bool is_unsigned = false;
    bool confirmed = true;
    std::string basis;
};
struct DriveAdapter {
    std::array<DriveUnitBinding, static_cast<size_t>(DriveQuantity::count)> bindings{};
    std::map<std::pair<uint32_t, uint32_t>, uint16_t> fault_mapping;
    std::string unit_profile;
    uint32_t counts_per_revolution = 0;
    int32_t direction = 1, zero_counts = 0;
    void load(const Json& config);
    void bind(Model& model);
    int64_t read(DriveQuantity quantity) const;
    int64_t raw(DriveQuantity quantity, int64_t counts) const;
    void publish(const d402::Status& status, bool actual = true);
    double position_rad(int32_t counts) const;
    double velocity_rad_s(int64_t counts_s) const;
    int32_t position_counts(double radians) const;
    int32_t velocity_counts(double radians_s) const;
    uint16_t fault_code(uint32_t fault_class, uint32_t detail) const;
    Json report() const;
};

/**
 * Owns all subentries and CANopenNode descriptors for one 16-bit OD index.
 * JSON ARRAY and RECORD objects both use record descriptors with per-subentry
 * attributes and storage; a scalar VAR instead uses the single var descriptor.
 * A descriptor is non-owning metadata: a pointer to live bytes plus their access
 * flags and length (and subindex for a record). It does not duplicate the value.
 * CANopenNode reaches values[0].storage through var.dataOrig for a VAR, or each
 * Value's storage through the corresponding record descriptor for a REC.
 */
struct Object {
    uint16_t index = 0; /**< OD index shared by all values in this object. */
    bool is_var_object = false; /**< True for one subentry at index:0 (ODT_VAR), false for ODT_REC; not mutability. */
    std::vector<Value> values; /**< Owning subentry storage, finalized before publishing descriptors. */
    OD_obj_var_t var{}; /**< Non-owning view of values[0]: data pointer, access flags, and byte length; VAR only. */
    std::vector<OD_obj_record_t> record; /**< Compound descriptors pointing into values; used only for REC. */
};

/**
 * Runtime OD built from a validated JSON model and generated communication objects.
 * Owns the storage referenced by OD descriptors and the stack configuration.
 * These objects must outlive the stack; published descriptor/storage addresses
 * must remain stable until the stack is deleted.
 */
struct Model {
    std::vector<std::unique_ptr<Object>> objects; /**< Owning objects, sorted by index during load. */
    std::vector<OD_entry_t> entries; /**< Sorted stack-facing entries plus a trailing empty sentinel. */
    OD_t od{}; /**< Non-owning dictionary view of entries, excluding the sentinel from its count. */
    CO_config_t config{}; /**< Service counts and OD entry pointers used to allocate/initialize the stack. */
    std::array<uint32_t, 4> identity{}; /**< Vendor ID, product code, revision number, serial number (LSS/0x1018). */
    uint8_t startup_node = 10; /**< JSON startup node ID: 1..127, or 255 for LSS-unconfigured. */
    uint16_t startup_bitrate = 125; /**< JSON startup bitrate in kbit/s, before persistence overrides. */
    std::optional<bool> can_termination;
    uint16_t rpdo_count = 0, tpdo_count = 0; /**< Configured receive/transmit PDO counts, respectively. */
    uint32_t worker_period_us = 1000; /**< Worker wait interval in microseconds; not a hard real-time guarantee. */
    bool drive_402_enabled = false; /**< True only when the JSON explicitly enables the PP profile. */
    bool drive_402_external = false; /**< Selects the motor_drive_ private model mailbox. */
    std::optional<DriveAdapter> drive_adapter;
    bool drive_has_error_code = false; /**< Optional 0x603F fault-code output. */
    bool drive_has_quick_stop_deceleration = false; /**< Optional 0x6085; defaults to PP profile deceleration. */
    uint32_t drive_position_tolerance = 0; /**< Allowed actual-to-target position error in counts. */
    uint32_t drive_target_reached_time_us = 0; /**< In-tolerance duration before Statusword bit 10 is set. */
    uint32_t drive_startup_timeout_us = 0;
    uint32_t drive_heartbeat_timeout_us = 0;
    uint32_t drive_feedback_timeout_us = 0;
    uint32_t drive_action_timeout_us = 0;
    uint32_t drive_command_max_age_us = 0;
    bool emcy_reconfigured = false; /**< An SDO-selected EMCY ID must survive stop/start without startup rebasing. */
    bool sync_reconfigured = false; /**< An SDO-selected SYNC ID overrides a node-relative startup default until reset. */
    /** Construct storage/descriptors before stack allocation; not a runtime OD merge. */
    void load(const Json& root);
    /** Restore communication defaults, optionally application defaults, then resolve node-relative IDs. */
    void restore(bool reset_all_objects, uint8_t node);
    /** Rebase startup-relative IDs without overwriting live application values. */
    void resolve_node(uint8_t node);
    Value& value(uint16_t index, uint8_t sub);
    /** Check a proposed SYNC/EMCY ID against reserved IDs and other local services. */
    bool communication_id_available(uint16_t index, uint32_t cob_id);
    void configure();
};

/**
 * One OD subentry's slot in a host batch buffer.
 * Stores an address by index/subindex, not an OD stream pointer: stack resets
 * replace extensions, so each batch operation resolves a fresh OD stream.
 */
struct Binding {
    uint16_t index; /**< Target OD index. */
    uint8_t subindex; /**< Target OD subindex. */
    uint32_t offset; /**< Byte offset from the start of the caller's packed batch buffer. */
    uint32_t size; /**< Slot length in bytes; must equal the target Value's logical size. */
};

/** Ordered host read/write layout; slots are byte-packed without alignment gaps. */
struct Batch {
    std::vector<Binding> bindings; /**< Slot order also determines per-entry status-array order. */
    uint32_t size = 0; /**< Total required buffer length in bytes (sum of binding sizes). */
};

/**
 * Owns a device's two raw NI-XNET frame-stream sessions on one physical interface.
 * No CAN database is used. Session references are released explicitly by close();
 * failed releases retain their references so device cleanup can be retried.
 */
struct Adapter {
    nxSessionRef_t input = 0, output = 0; /**< Input/output stream references, respectively; 0 means none owned. */
    bool sessions_started = false; /**< Both starts completed; cleared by stop/close, not a live hardware-state query. */
    void open(const std::string& name, uint16_t bitrate, std::optional<bool> termination);
    void bitrate(uint16_t kbps);
    void start();
    void stop_flush();
    void close();
    void check(nxStatus_t status, const char* operation);
};

/** Internal injected clock; no production provider or public activation exists yet. */
struct SyncClock {
    virtual ~SyncClock() = default;
    virtual bool map_rx_us(uint64_t raw_timestamp, uint64_t& mapped_us) const = 0;
    virtual bool now_us(uint64_t& current_us) const = 0;
    virtual bool map_model_us(const motor_drive_time_v1&, uint64_t&) const { return false; }
};

struct TestWatermarkProof {
    uint64_t through_us = 0, observed_us = 0;
};

struct TestTimingEvidence {
    uint64_t generation = 0, max_error_us = 0;
    std::optional<TestWatermarkProof> commands, feedback;
};

/* Snapshots are coherent with the associated clock generation. Proofs attest
 * processed events through the watermark, not merely an empty source queue. */
struct TestTimingProvider {
    virtual ~TestTimingProvider() = default;
    virtual bool snapshot(TestTimingEvidence&) const = 0;
};

struct TimedDriveCommand {
    CO_402_command_frame_t command{};
    xnet_canopen_sync_event_t sync{};
    uint32_t processed_feedback_seq = 0;
    uint16_t processed_result = 0;
};

struct TimedFeedbackSource {
    uint64_t raw_timestamp = 0, source_us = 0, epsilon_us = 0;
    motor_drive_model_step_v1 target_step{};
};

struct TimedDriveFeedback {
    motor_drive_feedback_v1 feedback{};
    CO_402_feedback_frame_t converted{};
    Clock::time_point submitted{};
    std::optional<TimedFeedbackSource> source;
};

struct TestFeedbackPlan {
    uint64_t delay_us = 0, epsilon_us = 0, max_age_us = 0;
};

struct TestCommandPlan {
    uint64_t lead_us = 0, epsilon_us = 0, max_age_us = 0, sync_domain_id = 0;
    uint32_t clock_domain_id = 0;
};

struct TestSamplingPlan {
    uint64_t pcl_epoch = 0, first_step = 0, step_stride = 0;
    uint64_t first_tick = 0, tick_stride = 0;
    uint64_t first_sample_us = 0, model_period_us = 0, acceptance_window_us = 0;
};

struct TestSamplingCursor {
    uint64_t next_step = 0, next_tick = 0, next_sample_us = 0, interval_us = 0;
};

struct TestHoldPlan {
    uint64_t pcl_epoch = 0, model_period_us = 0, tick_stride = 0, max_hold_steps = 0;
};

struct TestRpdoPlan {
    uint32_t required_mask = 0;
    uint64_t epsilon_us = 0;
};

struct TestHistoryBudget {
    size_t capacity = 64;
    uint64_t retention_us = 0, min_spacing_us = 0;
};

struct TestSyncWatermarkAnchor {
    uint64_t raw = 0, mapped_us = 0;
};

struct TestScheduledIssue {
    motor_drive_view_v1 view{};
    motor_drive_time_v1 cutoff{};
    uint64_t source_us = 0, epsilon_us = 0, target_us = 0, cutoff_us = 0;
    uint64_t max_age_us = 0;
    uint64_t raw_source_timestamp = 0;
};

struct SyncExecution {
    std::shared_ptr<const SyncClock> clock;
    std::shared_ptr<const SyncClock> active_clock;
    std::shared_ptr<const TestTimingProvider> test_timing_provider;
    std::shared_ptr<const TestTimingProvider> active_test_timing_provider;
    uint64_t test_max_clock_error_us = 0, active_test_max_clock_error_us = 0;
    uint64_t test_timing_generation = 0;
    std::optional<TestWatermarkProof> test_command_proof, test_feedback_proof;
    uint32_t max_interval_us = 0;
    uint32_t tpdo_window_us = 0;
    uint32_t active_max_interval_us = 0;
    uint32_t active_tpdo_window_us = 0;
    bool test_external_capture = false;
    bool active_test_external_capture = false;
    bool test_auto_retire = false;
    bool active_test_auto_retire = false;
    bool test_ordered_sync_watermark = false;
    bool active_test_ordered_sync_watermark = false;
    std::optional<TestSyncWatermarkAnchor> test_sync_watermark_anchor;
    TestHistoryBudget test_command_history_budget;
    TestHistoryBudget test_feedback_history_budget;
    TimedHistory<TimedDriveCommand, 64> commands;
    TimedHistory<TimedDriveFeedback, 64> feedback;
    uint64_t feedback_order = 0;
    std::optional<TestFeedbackPlan> test_feedback_plan;
    std::optional<TestCommandPlan> test_command_plan;
    std::optional<TestFeedbackPlan> active_test_feedback_plan;
    std::optional<TestCommandPlan> active_test_command_plan;
    std::optional<TestSamplingPlan> test_sampling_plan;
    std::optional<TestSamplingPlan> active_test_sampling_plan;
    std::optional<TestSamplingCursor> test_sampling_cursor;
    bool test_sampling_watermark = false;
    bool active_test_sampling_watermark = false;
    std::optional<uint64_t> test_sampling_closed_us;
    std::optional<uint64_t> test_sampling_anchor_us;
    std::optional<motor_drive_time_v1> test_sampling_anchor_time;
    std::optional<TestHoldPlan> test_hold_plan;
    std::optional<TestHoldPlan> active_test_hold_plan;
    std::optional<TestRpdoPlan> test_rpdo_plan;
    std::optional<TestRpdoPlan> active_test_rpdo_plan;
    uint32_t test_rpdo_seen = 0;
    std::optional<uint64_t> test_rpdo_last_us;
    std::optional<uint64_t> last_sync_us;
    std::optional<xnet_canopen_sync_event_t> last_sync_event;
    std::optional<xnet_canopen_timing_event_v1> diagnostic_context;
    std::optional<uint32_t> last_counter;
    std::optional<uint64_t> started_us;
    std::optional<uint64_t> last_now_us;
    bool invalid = false;
    std::array<char, 128> invalid_reason{};
};

struct TimingDiagnosticScope {
    SyncExecution* execution;
    std::optional<xnet_canopen_timing_event_v1> previous;
    explicit TimingDiagnosticScope(SyncExecution* active) : execution(active) {
        if (execution) {
            previous = execution->diagnostic_context;
            execution->diagnostic_context = previous.value_or(xnet_canopen_timing_event_v1{});
        }
    }
    ~TimingDiagnosticScope() {
        if (execution) execution->diagnostic_context = previous;
    }
    xnet_canopen_timing_event_v1* get() {
        return execution ? &*execution->diagnostic_context : nullptr;
    }
};
} // namespace xco

struct motor_drive_endpoint : std::enable_shared_from_this<motor_drive_endpoint> {
    std::atomic<xnet_canopen_handle> device{nullptr};
    std::atomic<bool> writer_open{false};
};
struct motor_drive_port {
    std::shared_ptr<motor_drive_endpoint> endpoint;
    uint64_t epoch = 0, pcl_epoch = 0;
    uint32_t heartbeat_seq = 0;
    motor_drive_model_step_v1 heartbeat_step{};
    std::optional<motor_drive_feedback_v1> submitted;
    std::array<xco::TestScheduledIssue, 64> test_issues{};
    size_t test_issue_count = 0;
};

/**
 * Per-device context behind the opaque public xnet_canopen_handle.
 * Owns the model, stack, XNET sessions, worker, batches, and diagnostics.
 * Public wrapped calls acquire lifecycle before mutex; the worker only acquires
 * mutex. Stop/join releases mutex so the worker can finish without deadlocking.
 * The caller must prevent concurrent calls during destruction and enforce
 * interface/persistence-path exclusivity outside this library.
 */
struct xnet_canopen_device {
    std::mutex lifecycle; /**< Serializes wrapped API calls, including while mutex is released for joins. */
    std::recursive_mutex mutex; /**< Protects OD/runtime state; recursive for stack callbacks into the driver. */
    std::condition_variable_any wake; /**< Interrupts the worker's timed wait when shutdown is requested. */
    std::thread worker; /**< Protocol/frame-processing thread; joined before releasing its resources. */
    std::atomic<bool> stop_requested{false}; /**< Shutdown request visible even when the worker releases mutex. */
    bool shutdown_joining = false; /**< Stop is waiting for a model safety action; only model OD traffic is admitted. */
    uint32_t state = XNET_CO_RUNTIME_UNINITIALIZED; /**< XNET_CO_RUNTIME_* lifecycle state, not NMT state. */
    /**
     * Runtime-lock-protected completion counters, kept outside Model/CO_t so
     * restoring OD defaults or rebuilding the stack cannot erase notification.
     * Reset only after successful initialize, not during release/failure cleanup.
     */
    uint32_t reset_node_count = 0;
    uint32_t reset_communication_count = 0;
    struct CANState {
        uint32_t raw = 0;
        bool valid = false;
        uint16_t logged_flags = 0;
        xco::Clock::time_point last_log{};
        uint64_t suppressed_changes = 0;
    } can_state; /**< Runtime-lock-protected controller sample and counter-only log throttling. */
    int32_t error_code = 0; /**< Latest per-handle XNET_CO_* diagnostic code; 0 when cleared. */
    std::array<char, 2048> error_message{}; /**< NUL-terminated diagnostic text, truncated to fit. */
    uint32_t diagnostic_domain = XNET_CO_DIAG_NONE;
    int32_t diagnostic_source_code = 0;
    const uint64_t log_instance = xco::next_log_instance(); /**< Process-local handle identity, stable across LSS/reset. */
    std::array<uint32_t, 4> log_identity{}; /**< Validated CANopen identity retained for failure/cleanup logging. */
    std::shared_ptr<spdlog::logger> logger; /**< Shared private file logger; null before the first initialize attempt opens it. */
    std::string interface_name; /**< Caller-supplied XNET interface name, copied without normalization. */
    std::filesystem::path persistence_path; /**< Caller-supplied LSS JSON file path, not canonicalized. */
    std::unique_ptr<xco::Model> model; /**< Owning runtime OD/model; null when uninitialized. */
    xco::Adapter adapter; /**< Sessions used exclusively by this context's frame I/O. */
    CO_t* co = nullptr; /**< Owning CANopenNode allocation, released with CO_delete; may be replaced on reset. */
    /**
     * Decorates one stack-installed OD callback with integration validation.
     * original borrows the current stack's callback/context; it must be refreshed
     * after every CO_t rebuild. extension itself has stable device-owned storage.
     * Delegation must restore original.stream.object, not pass this wrapper's
     * context to a callback expecting CO_SYNC_t, CO_EM_t, or a guarding/NMT object.
     */
    struct CommunicationOD {
        xnet_canopen_device* device = nullptr;
        uint16_t index = 0;
        OD_IO_t original{};
        OD_extension_t extension{};
    };
    std::array<CommunicationOD, 8> communication_od{}; /**< Writable SYNC, guarding, EMCY and heartbeat settings. */
    struct DriveModeOD {
        xnet_canopen_device* device = nullptr;
        OD_IO_t original{};
        OD_extension_t extension{};
    } drive_mode_od; /**< SDO mode-write guard; refreshed after each stack rebuild. */
    OD_extension_t drive_controlword_od{};
    OD_extension_t drive_option_od{};
    bool drive_controlword_rearm = false;
    int32_t worker_result = XNET_CO_OK;
    std::unique_ptr<xco::d402::Drive> drive_402; /**< Present only when enabled by JSON. */
    std::shared_ptr<motor_drive_endpoint> drive_endpoint;
    uint64_t sync_observation_epoch = 0;
    uint64_t sync_observation_sequence = 0;
    std::array<xnet_canopen_sync_event_t, 64> sync_observations{};
    std::unique_ptr<xco::SyncExecution> sync_execution;
    std::array<xnet_canopen_timing_event_v1, 128> timing_events{};
    uint64_t timing_event_sequence = 0;
    bool timing_event_exhausted = false;
    bool drive_adapter_fault = false;
    uint64_t drive_epoch = 0;
    uint32_t drive_command_seq = 0, drive_action_seq = 1, drive_last_feedback_seq = 0;
    CO_402_command_frame_t drive_command{};
    uint32_t drive_processed_feedback_seq = 0;
    uint16_t drive_processed_result = 0;
    uint32_t drive_last_submitted_seq = 0;
    struct DriveCommandRecord {
        uint32_t seq = 0;
        uint32_t action_seq = 0;
        xco::Clock::time_point published{};
        CO_402_command_frame_t command{};
        uint32_t processed_feedback_seq = 0;
        uint16_t processed_result = 0;
        bool test_scheduled = false;
    };
    std::array<DriveCommandRecord, 64> drive_command_history{};
    uint64_t drive_command_history_overwrites = 0;
    CO_402_feedback_frame_t drive_pending_feedback{};
    CO_402_feedback_frame_t drive_last_feedback{};
    motor_drive_feedback_v1 drive_pending_motor_feedback{};
    std::optional<xco::TestScheduledIssue> drive_pending_timed_source;
    struct DriveFeedbackRecord {
        bool valid = false;
        motor_drive_feedback_v1 feedback{};
        CO_402_feedback_frame_t converted{};
        xco::Clock::time_point submitted{}, accepted{};
    };
    std::array<DriveFeedbackRecord, 64> drive_feedback_history{};
    uint64_t drive_feedback_history_overwrites = 0;
    std::array<uint8_t, CO_402_FEEDBACK_FRAME_SIZE> drive_pending_feedback_bytes{};
    std::array<uint8_t, CO_402_FEEDBACK_FRAME_SIZE> drive_last_feedback_bytes{};
    std::array<uint8_t, CO_402_FEEDBACK_FRAME_SIZE> drive_last_submitted_bytes{};
    bool drive_feedback_pending = false, drive_has_last_feedback = false;
    xco::Clock::time_point drive_pending_submitted_at{}, drive_feedback_accepted_at{};
    bool drive_has_last_submitted = false;
    uint32_t drive_last_heartbeat = 0;
    bool drive_has_heartbeat = false;
    xco::Clock::time_point drive_heartbeat_changed{};
    xco::Clock::time_point drive_started_at{};
    bool drive_startup_complete = false;
    uint32_t drive_timed_action_seq = 0;
    xco::Clock::time_point drive_action_started{};
    bool drive_action_timed_out = false;
    uint8_t drive_transport_faults = 0, drive_reported_transport_faults = 0;
    uint16_t drive_latched_fault = 0;
    uint16_t drive_current_fault = 0;
    uint16_t drive_reported_model_fault = 0;
    xco::d402::Pds drive_previous_pds = xco::d402::Pds::switch_on_disabled;
    bool drive_has_previous_pds = false;
    bool drive_pp_ready = false, drive_reset_permitted = false;
    uint8_t pending_node = 10, active_node = 10; /**< LSS-selected and stack-applied node IDs; 255 means unconfigured. */
    uint16_t pending_bitrate = 125, active_bitrate = 125; /**< Requested and applied physical bitrates in kbit/s. */
    uint32_t next_batch = 1; /**< Next batch ID; never reused within this handle, with 0 marking exhaustion. */
    std::map<uint32_t, xco::Batch> batches; /**< Batch ID to layout; cleared on uninitialize, retained across resets. */
    /** Immutable queued frame snapshot paired with its originating stack transmit buffer. */
    struct Pending {
        CO_CANtx_t* buffer; /**< Non-owning stack pointer; bufferFull tracks this frame until sent/cancelled. */
        CO_CANrxMsg_t frame; /**< Copied identifier, DLC, and payload, immune to later transmit-buffer edits. */
    };
    std::deque<Pending> outgoing; /**< FIFO with at most one pending copy per stack transmit buffer. */
    /**
     * Non-owning pointer to run()'s outer OD lock; null outside worker execution.
     * Worker callbacks use it to release/reacquire mutex around persistence I/O.
     */
    std::unique_lock<std::recursive_mutex>* worker_lock = nullptr;
    /** LSS bitrate activation phases; protocol/frame I/O is suspended in both silence intervals. */
    enum class Switch {
        none,   /**< Normal protocol processing and frame I/O. */
        before, /**< Sessions quiesced; wait before changing the physical bitrate. */
        after   /**< Bitrate changed; wait again before restarting sessions. */
    };
    Switch switching = Switch::none; /**< Current LSS bitrate activation phase. */
    xco::Clock::time_point switch_deadline{}; /**< Monotonic deadline for the current silence interval. */
    uint16_t switch_delay_ms = 0; /**< LSS-requested delay in milliseconds, applied before and after the change. */

    void fail(int32_t code, std::string_view message) noexcept;
    void fail_motor(int32_t result, int32_t backend_code, std::string_view message) noexcept;
    /** Caller serializes device state; backend sink serializes writes across handles. */
    void log(xco::LogLevel level, std::string_view message) noexcept;
    template<class Arg, class... Args>
    void log(xco::LogLevel level, spdlog::format_string_t<Arg, Args...> format, Arg&& arg, Args&&... args) noexcept {
        if (!logger) return;
        try {
            const auto message = spdlog::fmt_lib::format(format, std::forward<Arg>(arg), std::forward<Args>(args)...);
            log(level, std::string_view(message));
        } catch (const std::exception& e) {
            log_failure(e.what());
        }
    }
    void log_failure(const char* message) noexcept;
    void record_can_state(uint32_t raw, uint16_t flags) noexcept;
    void initialized() const;
    void load_persistent();
    void setup_stack();
    void publish_drive_status(const xco::d402::Status& status);
    void reset_stack(bool reset_all_objects);
    void start_test_rpdo_plan();
    void start_test_timing_provider();
    bool test_timing_provider_valid() const;
    bool apply_test_timing_watermark(bool commands, uint64_t now);
    void advance_test_sampling(uint64_t now);
    bool test_sync_source_valid(uint64_t raw, uint64_t expected_us) const;
    bool test_sync_watermark_valid() const;
    bool validate_test_sync_watermark();
    void run();
    using SyncBoundaryHandler = void (*)(xnet_canopen_device&, const xnet_canopen_sync_event_t&, void*);
    void receive(SyncBoundaryHandler boundary = nullptr, void* context = nullptr);
    static void process_sync_boundary(xnet_canopen_device&, const xnet_canopen_sync_event_t&, void*);
    void process_sync_iteration(uint32_t dt_us);
    void record_timing_event(uint32_t kind, uint32_t reason = XNET_CO_TIMING_REASON_NONE,
                             const char* message = nullptr);
    void invalidate_sync_execution(const char* reason, uint32_t category = XNET_CO_TIMING_REASON_OTHER);
    bool sync_now(uint64_t& now);
    void transmit();
    void stop();
    void release();
    static bool_t check_bitrate(void*, uint16_t rate);
    static void activate_bitrate(void*, uint16_t delay);
    static bool_t store(void*, uint8_t node, uint16_t bitrate);
    static ODR_t read_communication(OD_stream_t*, void*, OD_size_t, OD_size_t*);
    static ODR_t write_communication(OD_stream_t*, const void*, OD_size_t, OD_size_t*);
    static ODR_t read_drive_mode(OD_stream_t*, void*, OD_size_t, OD_size_t*);
    static ODR_t write_drive_mode(OD_stream_t*, const void*, OD_size_t, OD_size_t*);
    static ODR_t write_drive_controlword(OD_stream_t*, const void*, OD_size_t, OD_size_t*);
    static ODR_t write_drive_option(OD_stream_t*, const void*, OD_size_t, OD_size_t*);
    CO_NMT_reset_cmd_t process_network(uint32_t dt_us);
    ODR_t stage_drive_feedback(const CO_402_feedback_frame_t& frame, const uint8_t* bytes);
    void process_drive_402(uint32_t dt_us, bool force_non_operational = false, bool safety_only = false);
    bool drain_drive_402();
    void start_drive_epoch();
    void start_sync_observation_epoch();
};

#endif
