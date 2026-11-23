/**
 * @file
 * Public ABI, device lifecycle and the single protocol-processing worker.
 * The lifecycle mutex serializes host calls; the recursive runtime mutex protects
 * OD/service state shared with the worker. Application/drive behavior remains in
 * a JSON-enabled DS402 core and ordinary application OD values.
 */
#include "internal.hpp"
#include "co_device.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <set>
#include <tuple>

using xco::Error;
using xco::require;

struct co_device {
    xnet_canopen_handle backend = nullptr;
};

namespace {
constexpr uint8_t drive_fault_startup = 0x01;
constexpr uint8_t drive_fault_heartbeat = 0x02;
constexpr uint8_t drive_fault_feedback = 0x04;
constexpr uint8_t drive_fault_action = 0x08;
constexpr uint8_t drive_transport_error_bit = CO_EM_MANUFACTURER_START;
int32_t motor_result(int32_t result);
/**
 * C ABI gateway: serialize the operation and translate typed/standard exceptions
 * to operation results plus a per-handle diagnostic. The lifecycle lock stays
 * owned even when an operation unlocks the runtime mutex to join the worker.
 * This cannot make a fabricated handle or concurrent destroy safe.
 */
template<class Function> int32_t call(xnet_canopen_handle d, Function function, bool allow_shutdown = false,
                                      bool motor_domain = false) noexcept {
    if (!d) return XNET_CO_INVALID_ARGUMENT;
    try {
        // Keep lifecycle ownership even when a call releases the OD lock to join the worker.
        std::lock_guard<std::mutex> lifecycle(d->lifecycle);
        std::unique_lock<std::recursive_mutex> lock(d->mutex);
        require(allow_shutdown || !d->shutdown_joining, XNET_CO_INVALID_STATE,
            "device shutdown is waiting for the DS402 model safety response");
        return function(*d, lock);
    } catch (const Error& e) {
        std::lock_guard<std::recursive_mutex> lock(d->mutex);
        if (motor_domain) d->fail_motor(motor_result(e.code), e.code, e.what());
        else d->fail(e.code, e.what());
        return e.code;
    } catch (const xco::Json::exception& e) {
        std::lock_guard<std::recursive_mutex> lock(d->mutex);
        if (motor_domain) d->fail_motor(MOTOR_DRIVE_BACKEND_ERROR, XNET_CO_MODEL_ERROR, e.what());
        else d->fail(XNET_CO_MODEL_ERROR, e.what());
        return XNET_CO_MODEL_ERROR;
    } catch (const std::exception& e) {
        std::lock_guard<std::recursive_mutex> lock(d->mutex);
        if (motor_domain) d->fail_motor(MOTOR_DRIVE_BACKEND_ERROR, XNET_CO_INTERNAL_ERROR, e.what());
        else d->fail(XNET_CO_INTERNAL_ERROR, e.what());
        return XNET_CO_INTERNAL_ERROR;
    }
}
void stack_ok(CO_ReturnError_t status, uint32_t info, const char* stage) {
    // An LSS-unconfigured node is a valid initialized runtime, not a setup failure.
    require(status == CO_ERROR_NO || status == CO_ERROR_NODE_ID_UNCONFIGURED_LSS,
        XNET_CO_STACK_ERROR, std::string(stage) + ": stack status " + std::to_string(status) + ", OD index " + std::to_string(info));
}
const char* nmt_log_message(CO_NMT_internalState_t state) {
    switch (state) {
        case CO_NMT_INITIALIZING: return "NMT Initializing";
        case CO_NMT_PRE_OPERATIONAL: return "NMT Pre-operational";
        case CO_NMT_OPERATIONAL: return "NMT Operational";
        case CO_NMT_STOPPED: return "NMT Stopped";
        default: return "NMT Unknown";
    }
}
template<class T> T read_drive_value(xco::Model& model, uint16_t index) {
    T value{};
    std::memcpy(&value, model.value(index, 0).data(), sizeof(value));
    return value;
}
template<class T> void write_drive_value(xco::Model& model, uint16_t index, T value) {
    std::memcpy(model.value(index, 0).data(), &value, sizeof(value));
}
bool sequence_newer(uint32_t value, uint32_t previous) {
    const uint32_t distance = value - previous;
    return distance != 0 && distance < 0x80000000U;
}
uint32_t increment_nonzero(uint32_t value) {
    ++value;
    return value == 0 ? 1 : value;
}
int32_t motor_result(int32_t result) {
    switch (result) {
        case XNET_CO_OK: return MOTOR_DRIVE_OK;
        case XNET_CO_INVALID_ARGUMENT: return MOTOR_DRIVE_INVALID_ARGUMENT;
        case XNET_CO_INVALID_STATE: return MOTOR_DRIVE_INVALID_STATE;
        case XNET_CO_ACCESS_ERROR: return MOTOR_DRIVE_INVALID_FEEDBACK;
        case XNET_CO_BUFFER_TOO_SMALL: return MOTOR_DRIVE_BUFFER_TOO_SMALL;
        default: return MOTOR_DRIVE_BACKEND_ERROR;
    }
}
int32_t motor_reject(xnet_canopen_device& d, int32_t result, const char* message) {
    d.fail_motor(result, result == MOTOR_DRIVE_INVALID_ARGUMENT ? XNET_CO_INVALID_ARGUMENT : XNET_CO_ACCESS_ERROR, message);
    return result;
}
int32_t open_motor_port(xnet_canopen_device& d, motor_drive_endpoint_handle endpoint,
                        motor_drive_port_handle* port) {
    d.initialized();
    require(port != nullptr, XNET_CO_INVALID_ARGUMENT, "motor port output is required");
    require(endpoint && d.state != XNET_CO_RUNTIME_FAILED && d.drive_endpoint.get() == endpoint
        && d.model->drive_adapter && d.model->drive_402_external,
        XNET_CO_INVALID_STATE, "motor endpoint is not active");
    if (endpoint->writer_open)
        return motor_reject(d, MOTOR_DRIVE_BUSY, "motor endpoint already has a writer");
    require(!d.drive_feedback_pending && !d.drive_has_last_submitted, XNET_CO_INVALID_STATE,
        "restart the model epoch before replacing a model writer");
    auto opened = std::make_unique<motor_drive_port>();
    opened->endpoint = endpoint->shared_from_this();
    opened->epoch = d.drive_epoch;
    endpoint->writer_open = true;
    *port = opened.release();
    return MOTOR_DRIVE_OK;
}
template<class Function> int32_t motor_call(motor_drive_port_handle port, Function function) {
    if (!port) return MOTOR_DRIVE_INVALID_ARGUMENT;
    const auto device = port->endpoint->device.load();
    if (!device) return MOTOR_DRIVE_INVALID_STATE;
    int32_t result = MOTOR_DRIVE_BACKEND_ERROR;
    const auto backend = call(device, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        d.initialized();
        if (d.state == XNET_CO_RUNTIME_FAILED) {
            result = MOTOR_DRIVE_INVALID_STATE;
            return XNET_CO_OK; // Preserve the worker's failure diagnostic during model teardown.
        }
        require(d.drive_endpoint.get() == port->endpoint.get()
            && port->endpoint->writer_open && d.model->drive_adapter, XNET_CO_INVALID_STATE, "motor port is not active");
        if (port->epoch != d.drive_epoch) {
            port->epoch = d.drive_epoch; port->pcl_epoch = 0; port->heartbeat_seq = 0;
            port->heartbeat_step = {}; port->submitted.reset(); port->test_issue_count = 0;
        }
        result = function(d, *port);
        return XNET_CO_OK;
    }, true, true);
    return backend == XNET_CO_OK ? result : motor_result(backend);
}
bool empty_time(const motor_drive_time_v1& time) {
    return time.clock_domain_id == 0 && time.flags == 0 && time.time_s == 0;
}
bool empty_step(const motor_drive_model_step_v1& step) {
    return step.pcl_epoch == 0 && step.pcl_tick == 0 && step.model_step_seq == 0
        && step.model_time_s == 0 && step.step_duration_s == 0 && empty_time(step.sample_time);
}
bool empty_cycle(const motor_drive_cycle_v1& cycle) {
    return cycle.sync_domain_id == 0 && cycle.sync_domain_epoch == 0 && cycle.sync_cycle_id == 0
        && cycle.sync_counter == 0 && cycle.flags == 0 && empty_time(cycle.rx_time) && empty_time(cycle.sync_time);
}
void timing_feedback(xnet_canopen_timing_event_v1& event, const motor_drive_feedback_v1& feedback,
                     const motor_drive_model_step_v1* target) {
    event.flags &= ~(XNET_CO_TIMING_HAS_SOURCE | XNET_CO_TIMING_HAS_SOURCE_CYCLE | XNET_CO_TIMING_HAS_TARGET
        | XNET_CO_TIMING_HAS_FEEDBACK | XNET_CO_TIMING_HAS_SOURCE_TIME | XNET_CO_TIMING_HAS_AVAILABLE_TIME
        | XNET_CO_TIMING_HAS_DECISION_TIME | XNET_CO_TIMING_HAS_CUTOFF | XNET_CO_TIMING_HAS_TARGET_TIME
        | XNET_CO_TIMING_HAS_SUBMITTED_STEADY_TIME);
    event.flags |= XNET_CO_TIMING_HAS_SOURCE | XNET_CO_TIMING_HAS_FEEDBACK;
    event.source = feedback.source;
    event.feedback_seq = feedback.feedback_seq;
    event.executed_step = feedback.executed_step;
    if (!empty_cycle(feedback.source_cycle)) {
        event.flags |= XNET_CO_TIMING_HAS_SOURCE_CYCLE;
        event.source_cycle = feedback.source_cycle;
    }
    if (target) {
        event.flags |= XNET_CO_TIMING_HAS_TARGET;
        event.target_step = *target;
    }
}
void timing_submission(xnet_canopen_timing_event_v1& event, xco::Clock::time_point submitted) {
    const auto value = std::chrono::duration<long double, std::micro>(submitted.time_since_epoch()).count();
    if (value >= static_cast<long double>(std::numeric_limits<int64_t>::min())
        && value < -static_cast<long double>(std::numeric_limits<int64_t>::min())) {
        event.flags |= XNET_CO_TIMING_HAS_SUBMITTED_STEADY_TIME;
        event.submitted_steady_us = static_cast<int64_t>(value);
    }
}
uint32_t timing_history_reason(xco::TimedHistoryResult result) {
    switch (result) {
        case xco::TimedHistoryResult::not_closed: return XNET_CO_TIMING_REASON_WATERMARK;
        case xco::TimedHistoryResult::no_candidate:
        case xco::TimedHistoryResult::stale_epoch: return XNET_CO_TIMING_REASON_STALE;
        case xco::TimedHistoryResult::deadline_missed: return XNET_CO_TIMING_REASON_DEADLINE;
        case xco::TimedHistoryResult::history_gap: return XNET_CO_TIMING_REASON_HISTORY_GAP;
        case xco::TimedHistoryResult::overflow: return XNET_CO_TIMING_REASON_HISTORY_OVERFLOW;
        default: return XNET_CO_TIMING_REASON_HISTORY_INVALID;
    }
}
bool same_time(const motor_drive_time_v1& a, const motor_drive_time_v1& b) {
    return std::tie(a.clock_domain_id, a.flags, a.time_s) == std::tie(b.clock_domain_id, b.flags, b.time_s);
}
bool same_cycle(const motor_drive_cycle_v1& a, const motor_drive_cycle_v1& b) {
    return std::tie(a.sync_domain_id, a.sync_domain_epoch, a.sync_cycle_id, a.sync_counter, a.flags)
        == std::tie(b.sync_domain_id, b.sync_domain_epoch, b.sync_cycle_id, b.sync_counter, b.flags)
        && same_time(a.rx_time, b.rx_time) && same_time(a.sync_time, b.sync_time);
}
bool valid_step(const motor_drive_model_step_v1& step) {
    const auto& time = step.sample_time;
    const bool valid_time = empty_time(time) || ((time.flags & MOTOR_DRIVE_TIME_VALID) && !(time.flags & ~3U)
        && time.clock_domain_id != 0
        && std::isfinite(time.time_s) && time.time_s >= 0);
    return step.pcl_epoch && step.pcl_tick && step.model_step_seq && std::isfinite(step.model_time_s)
        && step.model_time_s >= 0 && std::isfinite(step.step_duration_s) && step.step_duration_s > 0 && valid_time;
}
bool same_step(const motor_drive_model_step_v1& a, const motor_drive_model_step_v1& b) {
    return std::tie(a.pcl_epoch, a.pcl_tick, a.model_step_seq, a.model_time_s, a.step_duration_s,
                    a.sample_time.clock_domain_id, a.sample_time.flags, a.sample_time.time_s)
        == std::tie(b.pcl_epoch, b.pcl_tick, b.model_step_seq, b.model_time_s, b.step_duration_s,
                    b.sample_time.clock_domain_id, b.sample_time.flags, b.sample_time.time_s);
}
bool advancing_step(const motor_drive_model_step_v1& step, const motor_drive_model_step_v1& previous) {
    return step.pcl_epoch == previous.pcl_epoch && step.pcl_tick > previous.pcl_tick
        && step.model_step_seq > previous.model_step_seq && step.model_time_s >= previous.model_time_s
        && step.step_duration_s == previous.step_duration_s
        && (!(step.sample_time.flags && previous.sample_time.flags)
            || (step.sample_time.clock_domain_id == previous.sample_time.clock_domain_id
                && step.sample_time.time_s >= previous.sample_time.time_s));
}
bool test_hold_step(const xco::TestHoldPlan& plan, const motor_drive_model_step_v1& first,
                    uint64_t first_us, const motor_drive_model_step_v1& step, uint64_t step_us) {
    if (first.pcl_epoch != plan.pcl_epoch || step.pcl_epoch != plan.pcl_epoch
        || first.step_duration_s != plan.model_period_us / 1000000.0
        || step.step_duration_s != first.step_duration_s
        || step.sample_time.clock_domain_id != first.sample_time.clock_domain_id
        || step.sample_time.flags != first.sample_time.flags
        || step.model_step_seq < first.model_step_seq || step.pcl_tick < first.pcl_tick
        || step.model_time_s < first.model_time_s) return false;
    const uint64_t steps = step.model_step_seq - first.model_step_seq;
    if (steps == 0) return first_us == step_us && same_step(first, step);
    if (steps > UINT64_MAX / plan.tick_stride || steps > UINT64_MAX / plan.model_period_us) return false;
    const uint64_t ticks = steps * plan.tick_stride;
    const uint64_t elapsed = steps * plan.model_period_us;
    return first.pcl_tick <= UINT64_MAX - ticks && step.pcl_tick == first.pcl_tick + ticks
        && first_us <= UINT64_MAX - elapsed && step_us == first_us + elapsed;
}
bool same_feedback(const motor_drive_feedback_v1& a, const motor_drive_feedback_v1& b) {
    return std::tie(a.source.epoch, a.source.command_seq, a.source.action_seq, a.feedback_seq, a.action_result,
                    a.flags, a.limit_flags, a.actual_position_rad, a.actual_velocity_rad_s, a.fault_class, a.fault_detail)
        == std::tie(b.source.epoch, b.source.command_seq, b.source.action_seq, b.feedback_seq, b.action_result,
                    b.flags, b.limit_flags, b.actual_position_rad, b.actual_velocity_rad_s, b.fault_class, b.fault_detail)
        && same_step(a.executed_step, b.executed_step) && same_cycle(a.source_cycle, b.source_cycle);
}
}

void xnet_canopen_device::fail(int32_t code, std::string_view message) noexcept {
    // Fixed-capacity diagnostics remain usable while reporting allocation errors.
    // Successful operations do not clear this snapshot; callers clear explicitly.
    const auto size = std::min(message.size(), error_message.size() - 1);
    const bool changed = error_code != code || std::strlen(error_message.data()) != size
        || std::memcmp(error_message.data(), message.data(), size) != 0;
    error_code = code;
    diagnostic_domain = code == 0 ? XNET_CO_DIAG_NONE : XNET_CO_DIAG_XNET;
    diagnostic_source_code = code;
    std::memcpy(error_message.data(), message.data(), size);
    error_message[size] = 0;
    // Repeated identical queue/driver errors must not flood the file each cycle.
    if (changed) {
        log(xco::LogLevel::error, "error code={}: {}", code, error_message.data());
    }
}
void xnet_canopen_device::fail_motor(int32_t result, int32_t backend_code, std::string_view message) noexcept {
    fail(backend_code, message);
    diagnostic_domain = XNET_CO_DIAG_MOTOR;
    diagnostic_source_code = result;
}
void xnet_canopen_device::initialized() const {
    require(model && co && state != XNET_CO_RUNTIME_UNINITIALIZED, XNET_CO_INVALID_STATE, "device is not initialized");
}

/**
 * Load only persisted LSS identity/node/bitrate settings, not application OD data.
 * Both settings fall back together to JSON startup values on a load error, which
 * is retained in diagnostics. Disk I/O precedes the short locked state update.
 */
void xnet_canopen_device::load_persistent() {
    auto node = model->startup_node;
    auto rate = model->startup_bitrate;
    try {
        if (std::filesystem::exists(persistence_path)) {
            auto saved = xco::read_json(persistence_path);
            require(saved.is_object() && saved.size() == 4 && saved.at("schema_version") == 1,
                XNET_CO_IO_ERROR, "unsupported LSS persistence schema: " + persistence_path.string());
            require(saved.at("identity") == xco::Json(model->identity), XNET_CO_IO_ERROR, "persistence identity mismatch: " + persistence_path.string());
            const auto saved_node = static_cast<uint8_t>(xco::unsigned_number(saved.at("node_id"), 255, "saved node ID"));
            const auto saved_rate = static_cast<uint16_t>(xco::unsigned_number(saved.at("bitrate_kbps"), 1000, "saved bitrate"));
            require((saved_node >= 1 && saved_node <= 127) || saved_node == 255, XNET_CO_IO_ERROR, "invalid saved node ID");
            require(xco::valid_bitrate(saved_rate), XNET_CO_IO_ERROR, "invalid saved bitrate");
            node = saved_node;
            rate = saved_rate;
        }
    } catch (const std::exception& e) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        fail(XNET_CO_IO_ERROR, e.what());
    } catch (...) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        fail(XNET_CO_IO_ERROR, "cannot load LSS persistence; using model startup settings");
    }
    std::lock_guard<std::recursive_mutex> lock(mutex);
    pending_node = node;
    pending_bitrate = rate;
}

/**
 * Recreate protocol services over the existing Model and XNET sessions.
 * Called with runtime access serialized, before worker launch or by the worker
 * during a reset. OD storage/batch addresses survive; stack allocations and their
 * extensions do not. This function deliberately does not restore OD defaults.
 */
void xnet_canopen_device::setup_stack() {
    start_sync_observation_epoch();
    outgoing.clear();
    // Extensions point into the old stack allocation; detach them before rebuilding services.
    for (auto& entry : model->entries) entry.extension = nullptr;
    drive_mode_od = {};
    if (co) { CO_delete(co); co = nullptr; }
    // Also reached by ordinary start/restart: update node-dependent identifiers,
    // but do not restore OD defaults here, since stop/start preserves device data.
    model->resolve_node(pending_node);
    co = CO_new(&model->config, nullptr);
    require(co != nullptr, XNET_CO_STACK_ERROR, "CANopen stack allocation failed");
    stack_ok(CO_CANinit(co, this, active_bitrate), 0, "CAN initialization");
    CO_LSS_address_t address{};
    std::copy(model->identity.begin(), model->identity.end(), address.addr);
    stack_ok(CO_LSSinit(co, &address, &pending_node, &pending_bitrate), 0, "LSS initialization");
    CO_LSSslave_initCkBitRateCall(co->LSSslave, this, check_bitrate);
    CO_LSSslave_initActBitRateCall(co->LSSslave, this, activate_bitrate);
    CO_LSSslave_initCfgStoreCall(co->LSSslave, this, store);
    active_node = pending_node;
    uint32_t info = 0;
    // Keep NMT startup Pre-operational and apply the existing bus-off/heartbeat
    // error policy. A life-guard timeout shares the stack's heartbeat-error bit;
    // this policy is separate from the external model's drive fault reaction.
    auto init_result = CO_CANopenInit(co, nullptr, nullptr, &model->od, nullptr,
        CO_NMT_ERR_ON_BUSOFF_HB, 500, 1000, 1000, false, active_node, &info);
    stack_ok(init_result, info, "CANopen initialization");
    if (!co->nodeIdUnconfigured && model->drive_402_enabled) {
        drive_controlword_od = {};
        drive_controlword_od.object = this;
        drive_controlword_od.read = OD_readOriginal;
        drive_controlword_od.write = write_drive_controlword;
        require(OD_extension_init(OD_find(&model->od, 0x6040), &drive_controlword_od) == ODR_OK,
            XNET_CO_STACK_ERROR, "cannot install DS402 Controlword OD handler");
        drive_option_od = {};
        drive_option_od.read = OD_readOriginal;
        drive_option_od.write = write_drive_option;
        for (uint16_t index : {0x605a, 0x605e}) {
            if (auto* entry = OD_find(&model->od, index))
                require(OD_extension_init(entry, &drive_option_od) == ODR_OK,
                    XNET_CO_STACK_ERROR, "cannot install DS402 stop option OD handler");
        }
    }
    info = 0;
    auto result = CO_CANopenInitPDO(co, co->em, &model->od, active_node, &info);
    stack_ok(result, info, "PDO initialization");
    if (!co->nodeIdUnconfigured) {
        // Capture the stack-installed callbacks before decorating them. Keeping
        // those callbacks (rather than raw OD writes) preserves filter/timer
        // updates and EMCY's computed readback. Repeat after every stack rebuild.
        const uint16_t indices[] = {0x1005, 0x1006, 0x1007, 0x100c, 0x100d, 0x1014, 0x1015, 0x1017};
        for (size_t i = 0; i < communication_od.size(); ++i) {
            auto& binding = communication_od[i];
            binding.device = this;
            binding.index = indices[i];
            auto* entry = OD_find(&model->od, binding.index);
            require(OD_getSub(entry, 0, &binding.original, false) == ODR_OK,
                XNET_CO_STACK_ERROR, "cannot bind communication OD handler");
            binding.extension = {};
            binding.extension.object = &binding;
            binding.extension.read = read_communication;
            binding.extension.write = write_communication;
            require(OD_extension_init(entry, &binding.extension) == ODR_OK,
                XNET_CO_STACK_ERROR, "cannot install communication OD handler");
        }
        if (model->drive_402_enabled) {
            drive_mode_od.device = this;
            drive_mode_od.original = {};
            drive_mode_od.extension = {};
            auto* entry = OD_find(&model->od, 0x6060);
            require(OD_getSub(entry, 0, &drive_mode_od.original, false) == ODR_OK,
                XNET_CO_STACK_ERROR, "cannot bind DS402 mode OD handler");
            drive_mode_od.extension.object = &drive_mode_od;
            drive_mode_od.extension.read = read_drive_mode;
            drive_mode_od.extension.write = write_drive_mode;
            require(OD_extension_init(entry, &drive_mode_od.extension) == ODR_OK,
                XNET_CO_STACK_ERROR, "cannot install DS402 mode OD handler");
        }
    }
    if (drive_402) {
        publish_drive_status(drive_402->status());
        drive_402->complete_local_initialization();
    }
    log(xco::LogLevel::info, "CANopen services initialized");
}

void xnet_canopen_device::publish_drive_status(const xco::d402::Status& status) {
    write_drive_value(*model, 0x6041, status.statusword);
    write_drive_value(*model, 0x6061, status.mode_display);
}

void xnet_canopen_device::start_sync_observation_epoch() {
    require(sync_observation_epoch != UINT64_MAX, XNET_CO_STACK_ERROR, "SYNC observation epoch exhausted");
    ++sync_observation_epoch;
    sync_observation_sequence = 0;
    sync_observations = {};
}

/**
 * Delegate readback using the original service's context and caller's progress.
 * A local stream copy prevents the wrapper from losing its own dispatch context;
 * only transfer progress is copied back for the next OD access segment.
 */
ODR_t xnet_canopen_device::read_communication(OD_stream_t* stream, void* buffer, OD_size_t count, OD_size_t* transferred) {
    auto& binding = *static_cast<CommunicationOD*>(stream->object);
    auto original = *stream;
    original.object = binding.original.stream.object;
    const auto result = binding.original.read(&original, buffer, count, transferred);
    stream->dataOffset = original.dataOffset;
    return result;
}

/**
 * Add cross-service checks before invoking the stack's validated OD writer.
 * Called under the runtime mutex through OD I/O; no lifecycle lock is acquired.
 * Returning ODR_* lets the SDO server report a protocol abort without first
 * accepting an invalid value or changing the active filter/timer configuration.
 */
ODR_t xnet_canopen_device::write_communication(
    OD_stream_t* stream, const void* buffer, OD_size_t count, OD_size_t* transferred) {
    auto& binding = *static_cast<CommunicationOD*>(stream->object);
    auto& d = *binding.device;
    if (!buffer || !transferred || stream->subIndex != 0 || count != stream->dataLength) return ODR_TYPE_MISMATCH;
    if (binding.index == 0x1005 || binding.index == 0x1014) {
        if (!d.model->communication_id_available(binding.index, CO_getUint32(buffer))) return ODR_INVALID_VALUE;
    } else if (binding.index == 0x100c || binding.index == 0x100d) {
        // Combine the proposed field with the other field's active value. Widen
        // before multiplying: the individual OD types fit but their product may not.
        const auto guard = binding.index == 0x100c ? uint64_t(CO_getUint16(buffer))
                                                  : uint64_t(d.co->NGslave->guardTime_us / 1000);
        const auto factor = binding.index == 0x100d ? uint64_t(CO_getUint8(buffer))
                                                   : uint64_t(d.co->NGslave->lifeTimeFactor);
        if (guard * factor * 1000 > UINT32_MAX) return ODR_INVALID_VALUE;
    }
    auto original = *stream;
    original.object = binding.original.stream.object;
    const bool sync_id_change = binding.index == 0x1005 && d.co->SYNC
        && (CO_getUint32(buffer) & 0x7ffU) != d.co->SYNC->CAN_ID;
    if (sync_id_change && d.sync_observation_epoch == UINT64_MAX) return ODR_DEV_INCOMPAT;
    const auto result = binding.original.write(&original, buffer, count, transferred);
    stream->dataOffset = original.dataOffset;
    if (result == ODR_OK) {
        // Preserve accepted EMCY routing, including its disable bit, across a
        // stop/start. An explicit communication/node reset restores JSON defaults.
        if (binding.index == 0x1014) d.model->emcy_reconfigured = true;
        if (binding.index == 0x1005) {
            d.model->sync_reconfigured = true;
            if (sync_id_change) {
                d.start_sync_observation_epoch();
                d.invalidate_sync_execution("SYNC COB-ID changed; timing source invalidated");
            }
        }
        if (d.co->NMT->HBproducerTime_us > 0 || d.co->NGslave->lifeTime_us == 0) {
            // Stopping supervision must also clear a previously raised error.
            // HB consumer is disabled in this port, so this shared stack error
            // bit represents life guarding here, not another monitored peer.
            if (d.co->NGslave->lifeTimeTimeout) CO_errorReset(d.co->em, CO_EM_HEARTBEAT_CONSUMER, 0);
            d.co->NGslave->lifeTimeTimeout = false;
            d.co->NGslave->lifeTimer = 0;
        }
        const uint32_t value = count == 1 ? CO_getUint8(buffer) : count == 2 ? CO_getUint16(buffer) : CO_getUint32(buffer);
        d.log(xco::LogLevel::info, "communication OD updated: index=0x{:04x} value=0x{:08x}",
            unsigned(binding.index), unsigned(value));
    }
    return result;
}

ODR_t xnet_canopen_device::write_drive_mode(
    OD_stream_t* stream, const void* buffer, OD_size_t count, OD_size_t* transferred) {
    auto& binding = *static_cast<DriveModeOD*>(stream->object);
    if (!buffer || !transferred || stream->subIndex != 0 || count != sizeof(int8_t))
        return ODR_TYPE_MISMATCH;
    int8_t mode = 0;
    std::memcpy(&mode, buffer, sizeof(mode));
    if (mode != xco::d402::mode_profile_position) return ODR_INVALID_VALUE;
    auto original = *stream;
    original.object = binding.original.stream.object;
    const auto result = binding.original.write(&original, buffer, count, transferred);
    stream->dataOffset = original.dataOffset;
    return result;
}

ODR_t xnet_canopen_device::write_drive_controlword(
    OD_stream_t* stream, const void* buffer, OD_size_t count, OD_size_t* transferred) {
    auto& d = *static_cast<xnet_canopen_device*>(stream->object);
    const auto result = OD_writeOriginal(stream, buffer, count, transferred);
    if (result == ODR_OK && CO_NMT_getInternalState(d.co->NMT) == CO_NMT_OPERATIONAL)
        d.drive_controlword_rearm = false;
    return result;
}

ODR_t xnet_canopen_device::write_drive_option(
    OD_stream_t* stream, const void* buffer, OD_size_t count, OD_size_t* transferred) {
    int16_t option = 0;
    if (!buffer || count != sizeof(option)) return ODR_TYPE_MISMATCH;
    std::memcpy(&option, buffer, sizeof(option));
    if (!xco::d402::option_supported(stream->index, option)) return ODR_INVALID_VALUE;
    return OD_writeOriginal(stream, buffer, count, transferred);
}

ODR_t xnet_canopen_device::read_drive_mode(
    OD_stream_t* stream, void* buffer, OD_size_t count, OD_size_t* transferred) {
    auto& binding = *static_cast<DriveModeOD*>(stream->object);
    auto original = *stream;
    original.object = binding.original.stream.object;
    const auto result = binding.original.read(&original, buffer, count, transferred);
    stream->dataOffset = original.dataOffset;
    return result;
}

ODR_t xnet_canopen_device::stage_drive_feedback(const CO_402_feedback_frame_t& frame, const uint8_t* bytes) {
    if (drive_feedback_pending)
        return frame.feedback_seq == drive_pending_feedback.feedback_seq
            && std::memcmp(bytes, drive_pending_feedback_bytes.data(), CO_402_FEEDBACK_FRAME_SIZE) == 0
            ? ODR_OK : ODR_DATA_DEV_STATE;
    if (drive_has_last_submitted && frame.feedback_seq == drive_last_submitted_seq
        && std::memcmp(bytes, drive_last_submitted_bytes.data(), CO_402_FEEDBACK_FRAME_SIZE) == 0) return ODR_OK;
    if (drive_has_last_submitted && !sequence_newer(frame.feedback_seq, drive_last_submitted_seq))
        return ODR_DATA_DEV_STATE;
    drive_pending_feedback = frame;
    drive_pending_submitted_at = xco::Clock::now();
    drive_last_submitted_seq = frame.feedback_seq;
    std::memcpy(drive_last_submitted_bytes.data(), bytes, CO_402_FEEDBACK_FRAME_SIZE);
    drive_has_last_submitted = true;
    std::memcpy(drive_pending_feedback_bytes.data(), bytes, CO_402_FEEDBACK_FRAME_SIZE);
    drive_feedback_pending = true;
    return ODR_OK;
}

void xnet_canopen_device::start_drive_epoch() {
    if (!model || !model->drive_402_external) return;
    require(drive_epoch != UINT64_MAX, XNET_CO_STACK_ERROR, "DS402 external epoch exhausted");
    ++drive_epoch;
    if (sync_execution && sync_execution->active_test_external_capture) {
        const auto& command_plan = sync_execution->test_command_plan;
        const auto& feedback_plan = sync_execution->test_feedback_plan;
        require(!command_plan || (command_plan->sync_domain_id && command_plan->clock_domain_id
            && command_plan->epsilon_us <= command_plan->max_age_us
            && command_plan->lead_us <= command_plan->max_age_us - command_plan->epsilon_us),
            XNET_CO_INVALID_ARGUMENT, "invalid DS402 test command budget");
        require(!feedback_plan || (feedback_plan->epsilon_us <= feedback_plan->max_age_us
            && feedback_plan->delay_us <= feedback_plan->max_age_us - feedback_plan->epsilon_us),
            XNET_CO_INVALID_ARGUMENT, "invalid DS402 test feedback budget");
        const auto validate_history_budget = [](const xco::TestHistoryBudget& budget, uint64_t max_age) {
            if (!budget.capacity || budget.capacity > 64) return false;
            if (!budget.min_spacing_us) return budget.retention_us == 0;
            return budget.retention_us >= max_age
                && budget.retention_us / budget.min_spacing_us < budget.capacity;
        };
        const auto command_budget = sync_execution->test_command_history_budget;
        const auto feedback_budget = sync_execution->test_feedback_history_budget;
        require(validate_history_budget(command_budget, command_plan ? command_plan->max_age_us : 0)
            && validate_history_budget(feedback_budget, feedback_plan ? feedback_plan->max_age_us : 0),
            XNET_CO_INVALID_ARGUMENT, "invalid DS402 test history capacity budget");
        sync_execution->active_test_command_plan = command_plan;
        sync_execution->active_test_feedback_plan = feedback_plan;
        sync_execution->active_test_auto_retire = sync_execution->test_auto_retire;
        require(sync_execution->commands.start_epoch(drive_epoch, command_budget.capacity, command_budget.min_spacing_us)
                == xco::TimedHistoryResult::ok
            && sync_execution->feedback.start_epoch(drive_epoch, feedback_budget.capacity, feedback_budget.min_spacing_us)
                == xco::TimedHistoryResult::ok,
            XNET_CO_STACK_ERROR, "cannot start DS402 timed history epoch");
        sync_execution->feedback_order = 0;
        sync_execution->active_test_ordered_sync_watermark = sync_execution->test_ordered_sync_watermark;
        sync_execution->test_sync_watermark_anchor.reset();
        sync_execution->active_test_sampling_plan.reset();
        sync_execution->active_test_sampling_watermark = false;
        sync_execution->test_sampling_closed_us.reset();
        sync_execution->test_sampling_anchor_us.reset();
        sync_execution->test_sampling_anchor_time.reset();
        sync_execution->test_sampling_cursor.reset();
        sync_execution->active_test_hold_plan.reset();
        if (sync_execution->test_hold_plan) {
            const auto& hold = *sync_execution->test_hold_plan;
            require(hold.pcl_epoch && hold.model_period_us && hold.tick_stride && hold.max_hold_steps,
                XNET_CO_INVALID_ARGUMENT, "invalid DS402 test hold plan");
            if (sync_execution->test_sampling_plan) {
                const auto& sampling = *sync_execution->test_sampling_plan;
                require(sampling.pcl_epoch == hold.pcl_epoch && sampling.model_period_us == hold.model_period_us
                    && sampling.step_stride <= UINT64_MAX / hold.tick_stride
                    && sampling.tick_stride == sampling.step_stride * hold.tick_stride,
                    XNET_CO_INVALID_ARGUMENT, "DS402 test hold and sampling plans disagree");
            }
            sync_execution->active_test_hold_plan = hold;
        }
        if (sync_execution->test_sampling_plan) {
            const auto& plan = *sync_execution->test_sampling_plan;
            require(plan.pcl_epoch && plan.first_step && plan.step_stride && plan.first_tick && plan.tick_stride && plan.model_period_us
                && plan.acceptance_window_us && plan.step_stride <= UINT64_MAX / plan.model_period_us,
                XNET_CO_INVALID_ARGUMENT, "invalid DS402 test sampling plan");
            const uint64_t interval = plan.step_stride * plan.model_period_us;
            require(plan.acceptance_window_us <= interval
                && plan.first_sample_us <= UINT64_MAX - plan.acceptance_window_us,
                XNET_CO_INVALID_ARGUMENT, "invalid DS402 test sampling deadline");
            sync_execution->test_sampling_cursor =
                xco::TestSamplingCursor{plan.first_step, plan.first_tick, plan.first_sample_us, interval};
            sync_execution->active_test_sampling_plan = plan;
            sync_execution->active_test_sampling_watermark = sync_execution->test_sampling_watermark;
        }
    }
    drive_adapter_fault = false;
    drive_command_seq = 0;
    drive_command = {};
    drive_processed_feedback_seq = 0;
    drive_processed_result = 0;
    drive_action_seq = 1;
    drive_last_feedback_seq = 0;
    drive_last_submitted_seq = 0;
    drive_command_history = {};
    drive_command_history_overwrites = 0;
    drive_feedback_pending = false;
    drive_has_last_feedback = false;
    drive_has_last_submitted = false;
    drive_pending_submitted_at = drive_feedback_accepted_at = {};
    drive_pending_feedback = {};
    drive_last_feedback = {};
    drive_pending_motor_feedback = {};
    drive_pending_timed_source.reset();
    drive_feedback_history = {};
    drive_feedback_history_overwrites = 0;
    drive_pending_feedback_bytes = {};
    drive_last_feedback_bytes = {};
    drive_last_submitted_bytes = {};
    drive_has_heartbeat = false;
    drive_last_heartbeat = 0;
    drive_heartbeat_changed = {};
    drive_started_at = xco::Clock::now();
    drive_startup_complete = false;
    drive_timed_action_seq = 0;
    drive_action_started = {};
    drive_action_timed_out = false;
    drive_transport_faults = drive_reported_transport_faults = 0;
    drive_latched_fault = 0;
    drive_current_fault = 0;
    drive_reported_model_fault = 0;
    drive_previous_pds = xco::d402::Pds::switch_on_disabled;
    drive_has_previous_pds = false;
    drive_pp_ready = false;
    drive_reset_permitted = false;
}

void xnet_canopen_device::process_drive_402(uint32_t dt_us, bool force_non_operational, bool safety_only) {
    if (!model || !model->drive_402_enabled) return;
    require(drive_402 != nullptr, XNET_CO_INTERNAL_ERROR, "DS402 profile is enabled without a drive instance");
    const int8_t requested_mode = read_drive_value<int8_t>(*model, 0x6060);
    const bool unsupported_mode = !xco::d402::Drive::mode_supported(requested_mode);

    xco::d402::Inputs input;
    input.controlword = read_drive_value<uint16_t>(*model, 0x6040);
    if (drive_controlword_rearm) input.controlword = 0x0002U;
    if (unsupported_mode)
        input.controlword = static_cast<uint16_t>((input.controlword | 0x0002U) & ~0x0004U);
    input.mode_requested = requested_mode;
    input.nmt_operational = !force_non_operational && !stop_requested
        && CO_NMT_getInternalState(co->NMT) == CO_NMT_OPERATIONAL;
    input.force_shutdown = force_non_operational || stop_requested;
    const auto read_quantity = [&](xco::DriveQuantity quantity, uint16_t index, bool is_unsigned) -> int64_t {
        if (model->drive_adapter) return model->drive_adapter->read(quantity);
        return is_unsigned ? static_cast<int64_t>(read_drive_value<uint32_t>(*model, index))
                           : static_cast<int64_t>(read_drive_value<int32_t>(*model, index));
    };
    input.target_position = static_cast<int32_t>(read_quantity(xco::DriveQuantity::target_position, 0x607a, false));
    input.profile_velocity = static_cast<uint32_t>(read_quantity(xco::DriveQuantity::profile_velocity, 0x6081, true));
    input.profile_acceleration = static_cast<uint32_t>(read_quantity(xco::DriveQuantity::profile_acceleration, 0x6083, true));
    input.profile_deceleration = static_cast<uint32_t>(read_quantity(xco::DriveQuantity::profile_deceleration, 0x6084, true));
    input.quick_stop_deceleration = model->drive_has_quick_stop_deceleration
        ? static_cast<uint32_t>(read_quantity(xco::DriveQuantity::quick_stop_deceleration, 0x6085, true))
        : input.profile_deceleration;
    if (OD_find(&model->od, 0x605a)) input.quick_stop_option = read_drive_value<int16_t>(*model, 0x605a);
    if (OD_find(&model->od, 0x605e)) input.fault_reaction_option = read_drive_value<int16_t>(*model, 0x605e);
    input.position_tolerance = model->drive_position_tolerance;
    input.target_reached_time_us = model->drive_target_reached_time_us;
    uint16_t external_fault_code = 0;
    const auto now = xco::Clock::now();
    const auto previous_status = drive_402->status();
    if (model->drive_402_external) {
        const bool capture = sync_execution && sync_execution->active_test_external_capture;
        xco::TimingDiagnosticScope diagnostic(capture && drive_feedback_pending ? sync_execution.get() : nullptr);
        if (auto* context = diagnostic.get()) {
            timing_feedback(*context, drive_pending_motor_feedback,
                drive_pending_timed_source ? &drive_pending_timed_source->view.target_step : nullptr);
            timing_submission(*context, drive_pending_submitted_at);
        }
        const char* capture_failure = nullptr;
        uint32_t capture_reason = XNET_CO_TIMING_REASON_CLOCK;
        const bool heartbeat_expired = drive_has_heartbeat
            && now - drive_heartbeat_changed >= std::chrono::microseconds(model->drive_heartbeat_timeout_us);
        const auto action_deadline = drive_action_started + std::chrono::microseconds(model->drive_action_timeout_us);
        bool action_expired = previous_status.action_pending && drive_timed_action_seq == previous_status.action_seq
            && now >= action_deadline;

        if (drive_feedback_pending) {
            const auto frame = drive_pending_feedback;
            const auto& record = drive_command_history[frame.command_seq % drive_command_history.size()];
            bool identity_valid = frame.epoch == drive_epoch && frame.action_seq == drive_action_seq;
            bool command_expired = false;
            if (drive_pending_timed_source) {
                uint64_t current = 0, mapped_target = 0, mapped_step = 0;
                const auto& timed = *drive_pending_timed_source;
                if (!sync_execution || sync_execution->invalid) {
                    identity_valid = false;
                } else if (!test_sync_watermark_valid()) {
                    identity_valid = false;
                    capture_failure = "ordered SYNC watermark clock mapping changed";
                } else if (!test_sync_source_valid(timed.raw_source_timestamp, timed.source_us)) {
                    identity_valid = false;
                    capture_failure = "ordered SYNC source clock mapping changed";
                } else if (!sync_now(current) || current < timed.source_us) {
                    identity_valid = false;
                    capture_failure = "scheduled feedback acceptance clock invalid";
                } else if (!sync_execution->active_clock->map_model_us(timed.view.target_step.sample_time, mapped_target)
                    || mapped_target != timed.target_us
                    || !sync_execution->active_clock->map_model_us(drive_pending_motor_feedback.executed_step.sample_time, mapped_step)
                    || (sync_execution->active_test_hold_plan
                        ? !test_hold_step(*sync_execution->active_test_hold_plan, timed.view.target_step,
                                          mapped_target, drive_pending_motor_feedback.executed_step, mapped_step)
                        : mapped_step != timed.target_us)) {
                    identity_valid = false;
                    capture_failure = "scheduled feedback acceptance clock mapping changed";
                } else {
                    command_expired = timed.epsilon_us > model->drive_command_max_age_us
                        || current - timed.source_us > model->drive_command_max_age_us - timed.epsilon_us;
                }
            } else {
                identity_valid = identity_valid && record.seq == frame.command_seq && record.action_seq == frame.action_seq;
                command_expired = identity_valid
                    && now - record.published > std::chrono::microseconds(model->drive_command_max_age_us);
            }
            const bool late_completion = previous_status.action_pending
                && (frame.action_result == CO_402_ACTION_RESULT_DONE || frame.action_result == CO_402_ACTION_RESULT_ERROR)
                && drive_timed_action_seq == frame.action_seq && drive_pending_submitted_at >= action_deadline;
            const bool command_valid = identity_valid && !command_expired && !late_completion;
            const bool safety_fault_on_timing_failure = drive_pending_timed_source && frame.epoch == drive_epoch
                && (capture_failure || !sync_execution || sync_execution->invalid || command_expired || late_completion)
                && (frame.fault_code != 0 || frame.action_result == CO_402_ACTION_RESULT_ERROR);
            uint16_t result = command_valid ? 1 : command_expired ? 2 : 3;
            if (command_valid || safety_fault_on_timing_failure) {
                if (frame.action_result == CO_402_ACTION_RESULT_DONE && frame.action_seq == drive_timed_action_seq)
                    action_expired = false;
                const bool velocity_valid = (frame.flags & 0x02U) != 0;
                xco::d402::Feedback feedback{frame.actual_position,
                    velocity_valid ? frame.actual_velocity : drive_402->status().actual_velocity,
                    frame.fault_code != 0 || frame.action_result == CO_402_ACTION_RESULT_ERROR,
                    (frame.flags & 0x08U) != 0, velocity_valid};
                require((safety_only || capture) ? drive_402->post_safety_feedback(feedback) : drive_402->post_feedback(feedback),
                    XNET_CO_INTERNAL_ERROR, "DS402 external feedback rejected by drive core");
                drive_pp_ready = (frame.flags & 0x04U) != 0;
                drive_reset_permitted = (frame.flags & 0x10U) != 0;
                drive_402->set_external_permissions(drive_pp_ready && !capture, drive_reset_permitted);
                drive_last_feedback = frame;
                drive_last_feedback_seq = frame.feedback_seq;
                drive_last_feedback_bytes = drive_pending_feedback_bytes;
                drive_has_last_feedback = true;
                drive_feedback_accepted_at = now;
                auto& feedback_record = drive_feedback_history[frame.feedback_seq % drive_feedback_history.size()];
                if (feedback_record.valid) {
                    if (drive_feedback_history_overwrites == 0)
                        log(xco::LogLevel::warning, "asynchronous DS402 feedback history overwrote an older record; epoch={}",
                            drive_epoch);
                    if (drive_feedback_history_overwrites != UINT64_MAX) ++drive_feedback_history_overwrites;
                }
                feedback_record = {true, drive_pending_motor_feedback, frame, drive_pending_submitted_at, now};
                const bool scheduled_sample = drive_pending_timed_source.has_value();
                const bool sampling_enabled = capture && sync_execution->active_test_sampling_plan.has_value();
                if (capture && command_valid && !sync_execution->invalid && !capture_failure
                    && (!sampling_enabled || (scheduled_sample && !force_non_operational && !stop_requested))) {
                    uint64_t source = 0, accepted = 0;
                    const auto& sample = drive_pending_motor_feedback.executed_step.sample_time;
                    if (!(sample.flags & MOTOR_DRIVE_TIME_VALID)
                        || !sync_execution->active_clock->map_model_us(sample, source) || !sync_now(accepted)) {
                        capture_failure = "DS402 feedback clock mapping invalid";
                    } else if (sampling_enabled && !sync_execution->test_sampling_cursor) {
                        capture_failure = "DS402 fixed sampling plan not armed";
                        capture_reason = XNET_CO_TIMING_REASON_OTHER;
                    } else if (sampling_enabled) {
                        const auto& plan = *sync_execution->active_test_sampling_plan;
                        const auto& cursor = *sync_execution->test_sampling_cursor;
                        const auto& step = drive_pending_motor_feedback.executed_step;
                        if (step.pcl_epoch != plan.pcl_epoch || step.model_step_seq != cursor.next_step
                            || step.pcl_tick != cursor.next_tick || source != cursor.next_sample_us
                            || step.step_duration_s != plan.model_period_us / 1000000.0) {
                            capture_failure = "DS402 feedback missed fixed sampling step";
                            capture_reason = XNET_CO_TIMING_REASON_STALE;
                        } else if (accepted >= cursor.next_sample_us + plan.acceptance_window_us) {
                            capture_failure = "DS402 fixed feedback acceptance deadline missed";
                            capture_reason = XNET_CO_TIMING_REASON_DEADLINE;
                        }                         else if (cursor.next_step > UINT64_MAX - plan.step_stride
                            || cursor.next_tick > UINT64_MAX - plan.tick_stride
                            || cursor.next_sample_us > UINT64_MAX - cursor.interval_us
                            || cursor.next_sample_us + cursor.interval_us > UINT64_MAX - plan.acceptance_window_us) {
                            capture_failure = "DS402 fixed sampling plan exhausted";
                            capture_reason = XNET_CO_TIMING_REASON_OTHER;
                        }
                    }
                    if (!capture_failure && sync_execution->feedback_order == UINT64_MAX) {
                        capture_failure = "DS402 timed feedback sequence exhausted";
                        capture_reason = XNET_CO_TIMING_REASON_HISTORY_INVALID;
                    } else if (!capture_failure) {
                        xco::TimedDriveFeedback timed{drive_pending_motor_feedback, frame, drive_pending_submitted_at, {}};
                        if (drive_pending_timed_source) {
                            const auto& issued = *drive_pending_timed_source;
                            timed.source = xco::TimedFeedbackSource{
                                issued.raw_source_timestamp, issued.source_us, issued.epsilon_us, issued.view.target_step};
                        }
                        const auto result = sync_execution->feedback.append(
                            {drive_epoch, ++sync_execution->feedback_order, source, accepted, timed});
                        if (result != xco::TimedHistoryResult::ok) {
                            capture_failure = result == xco::TimedHistoryResult::overflow
                                ? "DS402 timed feedback history overflow" : "DS402 timed feedback history invalid";
                            capture_reason = timing_history_reason(result);
                        } else if (sampling_enabled && sync_execution->active_test_sampling_watermark
                            && sync_execution->feedback.close_through(drive_epoch, source, accepted)
                                != xco::TimedHistoryResult::ok) {
                            capture_failure = "DS402 fixed feedback watermark invalid";
                            capture_reason = XNET_CO_TIMING_REASON_WATERMARK;
                        } else if (sampling_enabled) {
                            if (sync_execution->active_test_sampling_watermark) {
                                sync_execution->test_sampling_closed_us = source;
                                sync_execution->test_sampling_anchor_us = source;
                                sync_execution->test_sampling_anchor_time = sample;
                            }
                            auto& cursor = *sync_execution->test_sampling_cursor;
                            cursor.next_step += sync_execution->active_test_sampling_plan->step_stride;
                            cursor.next_tick += sync_execution->active_test_sampling_plan->tick_stride;
                            cursor.next_sample_us += cursor.interval_us;
                        }
                        if (!capture_failure) {
                            auto& context = *diagnostic.get();
                            context.flags |= XNET_CO_TIMING_HAS_SOURCE_TIME | XNET_CO_TIMING_HAS_AVAILABLE_TIME;
                            context.source_us = source;
                            context.available_us = accepted;
                            record_timing_event(XNET_CO_TIMING_FEEDBACK_ACCEPTED);
                        }
                    }
                }
                if (command_valid && (!capture || !sync_execution->invalid)) drive_adapter_fault = false;
                drive_current_fault = frame.fault_code
                    ? frame.fault_code
                    : frame.action_result == CO_402_ACTION_RESULT_ERROR ? CO_EMC_GENERIC : 0;
                input.action_response_seq = frame.action_seq;
                input.action_result = frame.action_result;
            }
            drive_processed_feedback_seq = frame.feedback_seq;
            drive_processed_result = capture_failure ? static_cast<uint16_t>(MOTOR_DRIVE_PROCESSED_INVALID) : result;
            if (capture && !command_valid && !capture_failure)
                record_timing_event(XNET_CO_TIMING_FEEDBACK_REJECTED,
                    late_completion ? XNET_CO_TIMING_REASON_DEADLINE : XNET_CO_TIMING_REASON_STALE,
                    "feedback rejected before normal timed acceptance");
            drive_feedback_pending = false;
            drive_pending_timed_source.reset();
        }
        if (capture_failure) {
            invalidate_sync_execution(capture_failure, capture_reason);
            return;
        }
        input.external_feedback_age_valid = true;
        input.external_feedback_age_us = drive_has_last_feedback
            ? static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now - drive_feedback_accepted_at).count())
            : 0;
        const bool feedback_expired = drive_has_last_feedback
            && input.external_feedback_age_us >= model->drive_feedback_timeout_us;
        const bool healthy = drive_has_heartbeat && !heartbeat_expired && drive_has_last_feedback
            && !feedback_expired && drive_current_fault == 0 && drive_pp_ready && drive_reset_permitted;
        if (drive_has_heartbeat && !heartbeat_expired && drive_has_last_feedback && !feedback_expired)
            drive_startup_complete = true;
        const bool startup_expired = !drive_startup_complete && now - drive_started_at
            >= std::chrono::microseconds(model->drive_startup_timeout_us);
        if (action_expired) {
            input.action_timeout_seq = previous_status.action_seq;
            drive_action_timed_out = true;
            log(xco::LogLevel::warning, "DS402 action deadline expired: epoch={} action={} action_seq={}",
                drive_epoch, unsigned(previous_status.action), previous_status.action_seq);
        } else if (previous_status.pds == xco::d402::Pds::fault && !previous_status.action_pending && healthy) {
            drive_action_timed_out = false;
        }
        const uint8_t faults = static_cast<uint8_t>((startup_expired ? drive_fault_startup : 0)
            | (heartbeat_expired ? drive_fault_heartbeat : 0) | (feedback_expired ? drive_fault_feedback : 0)
            | (drive_action_timed_out ? drive_fault_action : 0));
        if (faults != drive_transport_faults) {
            log(xco::LogLevel::warning, "DS402 watchdog sources changed: epoch={} startup={} heartbeat={} feedback={} action={}",
                drive_epoch, startup_expired, heartbeat_expired, feedback_expired, drive_action_timed_out);
        }
        drive_transport_faults = faults;
        drive_402->set_external_fault(faults != 0 || drive_adapter_fault);
        external_fault_code = drive_current_fault ? drive_current_fault
            : drive_adapter_fault ? CO_EMC_GENERIC : faults ? 0x8130 : 0;
        input.model_ready = drive_startup_complete && drive_has_heartbeat && !heartbeat_expired
            && drive_has_last_feedback && !feedback_expired && drive_pp_ready && external_fault_code == 0;
        input.reset_permitted = healthy;
    }
    if (safety_only) drive_402->process_safety(input, dt_us);
    else drive_402->step(input, dt_us);
    const auto status = drive_402->status();
    if (model->drive_402_external) {
        if (status.action_pending && status.action_seq != drive_timed_action_seq) {
            drive_timed_action_seq = status.action_seq;
            drive_action_started = now;
        } else if (!status.action_pending) {
            drive_timed_action_seq = 0;
            drive_action_started = {};
        }
        if (status.reaction_unconfirmed && !previous_status.reaction_unconfirmed) {
            fail(XNET_CO_STACK_ERROR, "DS402 fault reaction failed or timed out; output revoked, physical stop unconfirmed");
        }
        drive_action_seq = status.action_seq;
        uint32_t command_seq = increment_nonzero(drive_command_seq);
        drive_command_seq = command_seq;
        CO_402_command_frame_t command{};
        command.epoch = drive_epoch;
        command.command_seq = command_seq;
        command.action_seq = status.action_seq;
        command.controlword = input.controlword;
        command.statusword = status.statusword;
        command.action = status.action;
        command.flags = static_cast<uint8_t>((status.output_allowed ? 0x01U : 0U)
            | (status.model_ready ? 0x02U : 0U));
        command.pds = static_cast<uint8_t>(status.pds);
        command.mode = status.mode_display;
        command.demand_position = status.demand_position;
        command.target_position = input.target_position;
        command.profile_velocity = input.profile_velocity;
        command.profile_acceleration = input.profile_acceleration;
        command.profile_deceleration = input.profile_deceleration;
        command.quick_stop_deceleration = input.quick_stop_deceleration;
        command.demand_velocity = status.demand_velocity;
        drive_command = command;
        auto& command_record = drive_command_history[command_seq % drive_command_history.size()];
        if (command_record.seq) {
            if (drive_command_history_overwrites == 0)
                log(xco::LogLevel::warning, "asynchronous DS402 command history overwrote an older record; epoch={}",
                    drive_epoch);
            if (drive_command_history_overwrites != UINT64_MAX) ++drive_command_history_overwrites;
        }
        command_record =
            DriveCommandRecord{command_seq, status.action_seq, xco::Clock::now(), command,
                drive_processed_feedback_seq, drive_processed_result};
        const bool fault_active = status.model_fault || external_fault_code != 0;
        const uint16_t current_fault = external_fault_code;
        require(!status.model_fault || current_fault != 0, XNET_CO_STACK_ERROR, "DS402 model fault has no classified cause");
        const bool reset_completed = drive_has_previous_pds
            && drive_previous_pds == xco::d402::Pds::fault
            && status.pds == xco::d402::Pds::switch_on_disabled && !fault_active;
        if (fault_active) {
            if (drive_latched_fault == 0) drive_latched_fault = current_fault;
            for (uint8_t source = 0; source < 4; ++source) {
                const uint8_t mask = static_cast<uint8_t>(1U << source);
                if ((drive_transport_faults & mask) && !(drive_reported_transport_faults & mask) && co && co->em) {
                    CO_errorReport(co->em, static_cast<uint8_t>(drive_transport_error_bit + source), 0x8130, mask);
                    drive_reported_transport_faults |= mask;
                }
            }
        } else if (reset_completed) {
            drive_latched_fault = 0;
            for (uint8_t source = 0; source < 4; ++source) {
                if ((drive_reported_transport_faults & (1U << source)) && co && co->em)
                    CO_errorReset(co->em, static_cast<uint8_t>(drive_transport_error_bit + source), 0);
            }
            drive_reported_transport_faults = 0;
        }
        const uint16_t model_fault = drive_current_fault ? drive_current_fault : drive_adapter_fault ? CO_EMC_GENERIC : 0;
        if (model_fault != drive_reported_model_fault && co && co->em) {
            const auto info = static_cast<uint32_t>(drive_epoch);
            if (drive_reported_model_fault) CO_errorReset(co->em, CO_EM_GENERIC_ERROR, info);
            if (model_fault) CO_errorReport(co->em, CO_EM_GENERIC_ERROR, model_fault, info);
            drive_reported_model_fault = model_fault;
        }
        if (model->drive_has_error_code) write_drive_value(*model, 0x603f, drive_latched_fault);
        drive_previous_pds = status.pds;
        drive_has_previous_pds = true;
    } else if (model->drive_has_error_code) {
        write_drive_value(*model, 0x603f, static_cast<uint16_t>(status.model_fault ? 0x1000 : 0));
    }
    if (model->drive_adapter) model->drive_adapter->publish(status, !safety_only);
    else if (!safety_only) write_drive_value(*model, 0x6064, status.actual_position);
    publish_drive_status(status);
    if (unsupported_mode) {
        log(xco::LogLevel::warning, "unsupported 0x6060 value {} received through PDO; requesting quick stop and restoring PP",
            int(requested_mode));
        write_drive_value(*model, 0x6040, input.controlword);
        write_drive_value(*model, 0x6060, xco::d402::mode_profile_position);
    }
}

bool xnet_canopen_device::drain_drive_402() {
    if (!drive_402) return true;
    const auto terminal = [&] {
        const auto status = drive_402->status();
        return !status.action_pending && status.pds != xco::d402::Pds::operation_enabled
            && status.pds != xco::d402::Pds::quick_stop_active && status.pds != xco::d402::Pds::fault_reaction_active;
    };
    auto previous = xco::Clock::now();
    const uint64_t budget_us = model->drive_402_external
        ? uint64_t(model->drive_action_timeout_us) * 2 + uint64_t(model->worker_period_us) * 2 : 10000000;
    const auto deadline = previous + std::chrono::microseconds(budget_us);
    while (!terminal()) {
        const auto now = xco::Clock::now();
        const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(now - previous).count();
        previous = now;
        process_drive_402(static_cast<uint32_t>(std::min<int64_t>(dt, UINT32_MAX)), true);
        if (terminal()) break;
        if (xco::Clock::now() >= deadline) {
            fail(XNET_CO_STACK_ERROR, "DS402 did not reach a terminal state before the shutdown/reset deadline");
            return false;
        }
        worker_lock->unlock();
        std::this_thread::sleep_for(std::chrono::microseconds(model->worker_period_us));
        worker_lock->lock();
    }
    return !drive_402->status().reaction_unconfirmed;
}

void xnet_canopen_device::start_test_rpdo_plan() {
    if (!sync_execution) return;
    auto& execution = *sync_execution;
    execution.active_test_rpdo_plan.reset();
    execution.test_rpdo_seen = 0;
    execution.test_rpdo_last_us.reset();
    if (!execution.test_rpdo_plan) return;
    const auto& plan = *execution.test_rpdo_plan;
    require(plan.required_mask && !(plan.required_mask >> model->rpdo_count)
        && plan.epsilon_us <= UINT64_MAX / 2 && !co->nodeIdUnconfigured,
        XNET_CO_INVALID_ARGUMENT, "invalid DS402 test required RPDO plan");
    for (uint16_t i = 0; i < model->rpdo_count; ++i) {
        if (!(plan.required_mask & (1U << i))) continue;
        const auto& rpdo = co->RPDO[i];
        require(rpdo.PDO_common.valid && rpdo.synchronous && rpdo.PDO_common.dataLength
            && rpdo.PDO_common.mappedObjectsCount && rpdo.PDO_common.mappedObjectsCount <= CO_PDO_MAX_MAPPED_ENTRIES,
            XNET_CO_INVALID_ARGUMENT, "required test RPDO must be enabled, mapped and synchronous");
        for (uint8_t mapped = 0; mapped < rpdo.PDO_common.mappedObjectsCount; ++mapped) {
            const auto& io = rpdo.PDO_common.OD_IO[mapped];
            require(io.write && io.read && io.stream.dataOffset
                && io.stream.dataOffset == io.stream.dataLength,
                XNET_CO_INVALID_ARGUMENT, "required test RPDO mappings must be readable and writable");
        }
    }
    execution.active_test_rpdo_plan = plan;
    log(xco::LogLevel::warning,
        "strict test RPDO plan requires every selected RPDO before every SYNC; "
        "valid sparse or asynchronous master traffic does not satisfy this test contract");
}

void xnet_canopen_device::start_test_timing_provider() {
    if (!sync_execution) return;
    auto& execution = *sync_execution;
    require(!execution.test_ordered_sync_watermark
        || (model->drive_402_external && execution.test_external_capture && execution.test_command_plan),
        XNET_CO_INVALID_STATE, "ordered SYNC test watermark requires external scheduled command capture");
    require(!execution.test_sampling_watermark || execution.test_sampling_plan,
        XNET_CO_INVALID_STATE, "test sampling watermark requires a fixed sampling plan");
    require(!execution.test_hold_plan
        || (model->drive_402_external && execution.test_external_capture && execution.test_command_plan),
        XNET_CO_INVALID_STATE, "test hold requires external scheduled command capture");
    require(!execution.test_sampling_plan
        || (model->drive_402_external && execution.test_external_capture && execution.test_command_plan),
        XNET_CO_INVALID_STATE, "fixed test sampling requires external scheduled command capture");
    require(model->drive_402_enabled && (!model->drive_402_external || execution.test_external_capture)
        && execution.clock && execution.max_interval_us && execution.tpdo_window_us,
        XNET_CO_INVALID_STATE, "SYNC execution requires an injected clock and internal model or test capture");
    execution.active_clock = execution.clock;
    execution.active_max_interval_us = execution.max_interval_us;
    execution.active_tpdo_window_us = execution.tpdo_window_us;
    execution.active_test_external_capture = execution.test_external_capture;
    execution.active_test_timing_provider = execution.test_timing_provider;
    execution.active_test_max_clock_error_us = execution.test_max_clock_error_us;
    execution.test_timing_generation = 0;
    execution.test_command_proof.reset();
    execution.test_feedback_proof.reset();
    if (!execution.active_test_timing_provider) return;
    require(model && model->drive_402_external && execution.test_external_capture
        && !execution.test_ordered_sync_watermark && !execution.test_sampling_watermark,
        XNET_CO_INVALID_ARGUMENT, "test timing provider requires external capture and exclusive watermarks");
    const uint64_t error = execution.active_test_max_clock_error_us;
    require((!execution.test_command_plan || execution.test_command_plan->epsilon_us >= error)
        && (!execution.test_feedback_plan || execution.test_feedback_plan->epsilon_us >= error)
        && (!execution.test_rpdo_plan || execution.test_rpdo_plan->epsilon_us >= error),
        XNET_CO_INVALID_ARGUMENT, "test timing provider error exceeds comparison budget");
    xco::TestTimingEvidence evidence{};
    require(execution.active_test_timing_provider->snapshot(evidence) && evidence.generation
        && evidence.max_error_us <= error,
        XNET_CO_INVALID_ARGUMENT, "test timing provider unavailable or inaccurate");
    execution.test_timing_generation = evidence.generation;
}

bool xnet_canopen_device::test_timing_provider_valid() const {
    const auto& execution = *sync_execution;
    if (!execution.active_test_timing_provider) return true;
    xco::TestTimingEvidence evidence{};
    return execution.active_test_timing_provider->snapshot(evidence)
        && evidence.generation == execution.test_timing_generation
        && evidence.max_error_us <= execution.active_test_max_clock_error_us;
}

bool xnet_canopen_device::apply_test_timing_watermark(bool commands, uint64_t now) {
    auto& execution = *sync_execution;
    if (!execution.active_test_timing_provider) return true;
    xco::TestTimingEvidence evidence{};
    if (!execution.active_test_timing_provider->snapshot(evidence)
        || evidence.generation != execution.test_timing_generation
        || evidence.max_error_us > execution.active_test_max_clock_error_us) {
        invalidate_sync_execution("test timing provider invalid", XNET_CO_TIMING_REASON_CLOCK);
        return false;
    }
    const auto& proof = commands ? evidence.commands : evidence.feedback;
    auto& previous = commands ? execution.test_command_proof : execution.test_feedback_proof;
    if (!proof) {
        if (!previous) return true;
        invalidate_sync_execution("test timing provider withdrew watermark", XNET_CO_TIMING_REASON_WATERMARK);
        return false;
    }
    if (proof->through_us > proof->observed_us || proof->observed_us > now
        || (previous && (proof->through_us < previous->through_us || proof->observed_us < previous->observed_us))) {
        invalidate_sync_execution("test timing provider watermark invalid", XNET_CO_TIMING_REASON_WATERMARK);
        return false;
    }
    if (!previous || proof->through_us > previous->through_us) {
        const auto result = commands
            ? execution.commands.close_through(drive_epoch, proof->through_us, proof->observed_us)
            : execution.feedback.close_through(drive_epoch, proof->through_us, proof->observed_us);
        if (result != xco::TimedHistoryResult::ok) {
            invalidate_sync_execution("test timing provider watermark violated", XNET_CO_TIMING_REASON_WATERMARK);
            return false;
        }
    }
    previous = proof;
    return true;
}

void xnet_canopen_device::reset_stack(bool reset_all_objects) {
    log(xco::LogLevel::info, reset_all_objects ? "Reset Node requested" : "Reset Communication requested");
    require(drain_drive_402(), XNET_CO_STACK_ERROR, "DS402 reset aborted: physical stop unconfirmed");
    drive_controlword_rearm = drive_402 != nullptr;
    adapter.stop_flush();
    // Reset Node reloads saved/startup LSS settings and restores all OD defaults.
    // Reset Communication preserves device data and the pending LSS settings.
    if (reset_all_objects) {
        // The worker is the only reset owner; filesystem reads need no OD lock.
        worker_lock->unlock();
        try { load_persistent(); } catch (...) { worker_lock->lock(); throw; }
        worker_lock->lock();
        active_bitrate = pending_bitrate;
        adapter.bitrate(active_bitrate);
    }
    model->restore(reset_all_objects, pending_node);
    if (drive_402) drive_402->reset();
    start_test_timing_provider();
    start_drive_epoch();
    if (sync_execution) {
        sync_execution->last_sync_us.reset();
        sync_execution->last_sync_event.reset();
        sync_execution->last_counter.reset();
        sync_execution->started_us.reset();
        sync_execution->invalid = false;
        sync_execution->invalid_reason = {};
    }
    setup_stack();
    start_test_rpdo_plan();
    if (sync_execution) {
        uint64_t now = 0;
        if (sync_now(now)) sync_execution->started_us = now;
        else invalidate_sync_execution("SYNC clock invalid after reset", XNET_CO_TIMING_REASON_CLOCK);
    }
    adapter.start();
    CO_CANsetNormalMode(co->CANmodule);
    // Publish completion only after all fallible reset work succeeds. The worker
    // holds mutex here, so status cannot see the restored OD/new stack with an
    // old completion count. Reset Node is one operation, not an additional
    // Reset Communication notification. Unsigned overflow intentionally wraps.
    if (reset_all_objects) ++reset_node_count;
    else ++reset_communication_count;
    log(xco::LogLevel::info, "CANopen reset completed");
}

/**
 * LSS validation callback: context is this device and rate is in kbit/s.
 * CANopenNode validates the standard table selector/index before calling this
 * function with its numerical rate. Fixed entries are accepted within 1..1000;
 * hardware support is checked when XNET applies the rate, not by this callback.
 * Index 9 arrives as rate 0, which is rejected because automatic detection is
 * not implemented. Startup JSON is not restricted to the LSS table's fixed rates.
 */
bool_t xnet_canopen_device::check_bitrate(void* context, uint16_t rate) {
    auto& d = *static_cast<xnet_canopen_device*>(context);
    if (xco::valid_bitrate(rate)) return true;
    d.fail(XNET_CO_INVALID_ARGUMENT, "unsupported LSS bitrate (automatic detection is not available)");
    return false;
}

/**
 * LSS activation callback, invoked by the worker while holding the OD mutex.
 * delay is the silence interval in milliseconds on each side of the bitrate change.
 * Stop/flush immediately; run() later changes to pending_bitrate and restarts
 * sessions after the second interval, without blocking this callback for the delay.
 */
void xnet_canopen_device::activate_bitrate(void* context, uint16_t delay) {
    auto& d = *static_cast<xnet_canopen_device*>(context);
    d.switch_delay_ms = delay;
    d.switching = Switch::before;
    try {
        // Stop immediately so queued frames cannot violate either LSS silence interval.
        d.start_sync_observation_epoch();
        d.invalidate_sync_execution("bitrate switch invalidated SYNC timing");
        d.adapter.stop_flush();
        for (auto& pending : d.outgoing) pending.buffer->bufferFull = false;
        d.outgoing.clear();
        d.switch_deadline = xco::Clock::now() + std::chrono::milliseconds(delay);
        d.log(xco::LogLevel::info, "LSS bitrate activation: pre-switch silence");
    } catch (const Error& e) {
        d.fail(e.code, e.what()); d.state = XNET_CO_RUNTIME_FAILED; d.stop_requested = true;
    }
}

/**
 * LSS Store Configuration callback: save the supplied node ID and kbit/s rate
 * with this device's identity. Runs on the worker and temporarily releases its
 * OD lock during disk I/O; lifecycle teardown joins the worker before freeing it.
 * Return true only when atomic storage completes, so LSS can acknowledge success;
 * storage exceptions record a diagnostic and return false for an LSS error reply.
 */
bool_t xnet_canopen_device::store(void* context, uint8_t node, uint16_t bitrate) {
    auto& d = *static_cast<xnet_canopen_device*>(context);
    const auto saved = xco::Json{{"schema_version", 1}, {"identity", d.model->identity},
        {"node_id", node}, {"bitrate_kbps", bitrate}};
    const auto path = d.persistence_path;
    // Snapshot before unlocking. Teardown joins this worker, so its context stays alive.
    // Returning true only after durable storage lets the LSS stack acknowledge honestly.
    d.worker_lock->unlock();
    try {
        xco::write_json_atomic(path, saved);
        d.worker_lock->lock();
        d.log(xco::LogLevel::info, "LSS configuration stored");
        return true;
    } catch (const std::exception& e) {
        d.worker_lock->lock();
        d.fail(XNET_CO_IO_ERROR, e.what());
        return false;
    }
}

void xnet_canopen_device::record_timing_event(uint32_t kind, uint32_t reason, const char* message) {
    if (!sync_execution || timing_event_exhausted) return;
    if (timing_event_sequence == UINT64_MAX) {
        timing_event_exhausted = true;
        fail(XNET_CO_STACK_ERROR, "timing diagnostic sequence exhausted");
        return;
    }
    auto event = sync_execution->diagnostic_context.value_or(xnet_canopen_timing_event_v1{});
    event.abi_version = XNET_CO_TIMING_EVENT_ABI_VERSION;
    event.struct_size = sizeof(event);
    event.kind = kind;
    event.reason = reason;
    event.sequence = ++timing_event_sequence;
    event.drive_epoch = drive_epoch;
    event.node_id = active_node;
    if (message) {
        const auto length = std::min(std::strlen(message), sizeof(event.message) - 1);
        std::memcpy(event.message, message, length);
        event.message[length] = '\0';
    }
    timing_events[(event.sequence - 1) % timing_events.size()] = event;
}

void xnet_canopen_device::invalidate_sync_execution(const char* reason, uint32_t category) {
    if (!sync_execution || sync_execution->invalid) return;
    record_timing_event(XNET_CO_TIMING_FAILURE, category, reason);
    sync_execution->invalid = true;
    if (sync_execution->active_test_external_capture) {
        sync_execution->commands.invalidate();
        sync_execution->feedback.invalidate();
    }
    const auto length = std::min(std::strlen(reason), sync_execution->invalid_reason.size() - 1);
    std::memcpy(sync_execution->invalid_reason.data(), reason, length);
    sync_execution->invalid_reason[length] = '\0';
    if (co) CO_CANclearPendingSyncPDOs(co->CANmodule);
    fail(XNET_CO_STACK_ERROR, reason);
    if (drive_402) {
        drive_adapter_fault = true;
        if (model && model->drive_402_external) drive_402->invalidate_selected_feedback();
        drive_402->set_external_fault(true);
        process_drive_402(0, false, true);
    }
}

bool xnet_canopen_device::sync_now(uint64_t& now) {
    auto& execution = *sync_execution;
    if (!test_timing_provider_valid() || !execution.active_clock->now_us(now)
        || (execution.last_now_us && now < *execution.last_now_us)) return false;
    execution.last_now_us = now;
    return true;
}

bool xnet_canopen_device::test_sync_source_valid(uint64_t raw, uint64_t expected_us) const {
    const auto& execution = *sync_execution;
    if (!execution.active_test_ordered_sync_watermark) return true;
    uint64_t mapped = 0;
    return execution.active_clock->map_rx_us(raw, mapped) && mapped == expected_us;
}

bool xnet_canopen_device::test_sync_watermark_valid() const {
    const auto& anchor = sync_execution->test_sync_watermark_anchor;
    return !anchor || test_sync_source_valid(anchor->raw, anchor->mapped_us);
}

bool xnet_canopen_device::validate_test_sync_watermark() {
    if (test_sync_watermark_valid()) return true;
    invalidate_sync_execution("ordered SYNC watermark clock mapping changed", XNET_CO_TIMING_REASON_CLOCK);
    return false;
}

void xnet_canopen_device::advance_test_sampling(uint64_t now) {
    auto& execution = *sync_execution;
    if (execution.invalid || !execution.active_test_sampling_plan) return;
    if (!execution.test_sampling_cursor) {
        invalidate_sync_execution("DS402 fixed sampling plan not armed");
        return;
    }
    const auto& cursor = *execution.test_sampling_cursor;
    if (now >= cursor.next_sample_us + execution.active_test_sampling_plan->acceptance_window_us) {
        invalidate_sync_execution("DS402 fixed feedback acceptance deadline missed", XNET_CO_TIMING_REASON_DEADLINE);
        return;
    }
    if (execution.active_test_sampling_watermark && execution.test_sampling_closed_us) {
        uint64_t anchor_us = 0, next_us = 0;
        if (!execution.test_sampling_anchor_time || !execution.test_sampling_anchor_us) {
            invalidate_sync_execution("DS402 fixed feedback watermark anchor missing", XNET_CO_TIMING_REASON_WATERMARK);
            return;
        }
        auto next = *execution.test_sampling_anchor_time;
        next.time_s += cursor.interval_us / 1000000.0;
        if (!std::isfinite(next.time_s) || next.time_s <= execution.test_sampling_anchor_time->time_s
            || !execution.active_clock->map_model_us(*execution.test_sampling_anchor_time, anchor_us)
            || anchor_us != *execution.test_sampling_anchor_us
            || !execution.active_clock->map_model_us(next, next_us) || next_us != cursor.next_sample_us) {
            invalidate_sync_execution("DS402 fixed feedback watermark clock mapping changed", XNET_CO_TIMING_REASON_CLOCK);
            return;
        }
        const uint64_t through = std::min(now, cursor.next_sample_us - 1);
        if (through > *execution.test_sampling_closed_us) {
            if (execution.feedback.close_through(drive_epoch, through, now) != xco::TimedHistoryResult::ok)
                invalidate_sync_execution("DS402 fixed feedback watermark invalid", XNET_CO_TIMING_REASON_WATERMARK);
            else execution.test_sampling_closed_us = through;
        }
    }
}

void xnet_canopen_device::process_sync_boundary(xnet_canopen_device& d,
                                                const xnet_canopen_sync_event_t& event, void*) {
    auto& execution = *d.sync_execution;
    if (execution.invalid) return;
    xco::TimingDiagnosticScope diagnostic(&execution);
    auto& context = *diagnostic.get();
    context.flags |= XNET_CO_TIMING_HAS_BOUNDARY;
    context.boundary = event;
    if (!d.validate_test_sync_watermark()) return;
    uint64_t source = 0, now = 0;
    const bool valid_clock = execution.active_clock->map_rx_us(event.timestamp_100ns, source) && d.sync_now(now)
        && execution.started_us && source >= *execution.started_us && now >= source;
    if (!valid_clock || now - source >= execution.active_tpdo_window_us) {
        d.invalidate_sync_execution("SYNC timestamp invalid or TPDO deadline missed",
            valid_clock ? XNET_CO_TIMING_REASON_DEADLINE : XNET_CO_TIMING_REASON_CLOCK);
        return;
    }
    if (event.counter_present) {
        const auto overflow = d.co->SYNC->counterOverflowValue;
        if (event.counter == 0 || event.counter > overflow
            || (execution.last_counter && event.counter != (*execution.last_counter == overflow ? 1 : *execution.last_counter + 1))) {
            d.invalidate_sync_execution("SYNC counter invalid or discontinuous");
            return;
        }
        execution.last_counter = event.counter;
    } else if (execution.last_counter) {
        d.invalidate_sync_execution("SYNC counter configuration changed");
        return;
    }
    if (std::any_of(d.outgoing.begin(), d.outgoing.end(),
                    [](const Pending& frame) { return frame.buffer->syncFlag; })) {
        d.invalidate_sync_execution("previous synchronous TPDO remained queued at next SYNC", XNET_CO_TIMING_REASON_DEADLINE);
        return;
    }
    uint32_t interval = 0;
    if (execution.last_sync_us) {
        if (source <= *execution.last_sync_us
            || source - *execution.last_sync_us > execution.active_max_interval_us) {
            d.invalidate_sync_execution("SYNC timestamp regressed or interval exceeded", XNET_CO_TIMING_REASON_CLOCK);
            return;
        }
        interval = static_cast<uint32_t>(source - *execution.last_sync_us);
    }
    execution.last_sync_us = source;
    execution.last_sync_event = event;
    if (execution.active_test_rpdo_plan
        && !xco::d402::Drive::mode_supported(read_drive_value<int8_t>(*d.model, 0x6060))) {
        d.invalidate_sync_execution("required RPDO selected an unsupported DS402 mode", XNET_CO_TIMING_REASON_RPDO);
        return;
    }
    d.process_drive_402(0, false, true);
    if (execution.invalid) return;
    if (execution.active_test_sampling_plan) {
        uint64_t current = 0;
        if (!d.sync_now(current)) d.invalidate_sync_execution("DS402 fixed sampling clock invalid", XNET_CO_TIMING_REASON_CLOCK);
        else d.advance_test_sampling(current);
        if (execution.invalid) return;
    }
    bool feedback_selected = false;
    if (d.model->drive_402_external && execution.active_test_external_capture && execution.active_test_feedback_plan) {
        const auto& plan = *execution.active_test_feedback_plan;
        uint64_t decision = 0;
        if (source < plan.delay_us || source > UINT64_MAX - execution.active_tpdo_window_us
            || !d.sync_now(decision) || decision < source) {
            d.invalidate_sync_execution("DS402 feedback selection timing invalid", XNET_CO_TIMING_REASON_CLOCK);
            return;
        }
        const uint64_t elapsed = decision - source;
        if (!d.apply_test_timing_watermark(false, decision)) return;
        if (plan.max_age_us < plan.delay_us || plan.max_age_us - plan.delay_us < elapsed
            || plan.max_age_us - plan.delay_us - elapsed < plan.epsilon_us) {
            d.invalidate_sync_execution("DS402 feedback sample missing or stale", XNET_CO_TIMING_REASON_STALE);
            return;
        }
        xco::TimedHistory<xco::TimedDriveFeedback, 64>::Record selected{};
        const auto result = execution.feedback.select(d.drive_epoch, source - plan.delay_us, plan.epsilon_us,
            decision, source + execution.active_tpdo_window_us,
            plan.max_age_us - plan.delay_us - elapsed - plan.epsilon_us, selected);
        if (result != xco::TimedHistoryResult::ok) {
            const char* reason = "DS402 feedback selection history invalid";
            if (result == xco::TimedHistoryResult::not_closed) reason = "DS402 feedback watermark not closed";
            else if (result == xco::TimedHistoryResult::no_candidate) reason = "DS402 feedback sample missing or stale";
            else if (result == xco::TimedHistoryResult::deadline_missed) reason = "DS402 feedback selection deadline missed";
            else if (result == xco::TimedHistoryResult::history_gap) reason = "DS402 feedback selection intersects retired history";
            else if (result == xco::TimedHistoryResult::stale_epoch) reason = "DS402 feedback selection epoch invalid";
            d.invalidate_sync_execution(reason, timing_history_reason(result));
            return;
        }
        timing_feedback(context, selected.value.feedback,
            selected.value.source ? &selected.value.source->target_step : nullptr);
        timing_submission(context, selected.value.submitted);
        context.flags |= XNET_CO_TIMING_HAS_SOURCE_TIME | XNET_CO_TIMING_HAS_AVAILABLE_TIME
            | XNET_CO_TIMING_HAS_DECISION_TIME;
        context.source_us = selected.source_time;
        context.available_us = selected.available_time;
        context.decision_us = decision;
        uint64_t mapped_sample = 0;
        if (!execution.active_clock->map_model_us(selected.value.feedback.executed_step.sample_time, mapped_sample)
            || mapped_sample != selected.source_time) {
            d.invalidate_sync_execution("DS402 feedback source clock mapping changed", XNET_CO_TIMING_REASON_CLOCK);
            return;
        }
        if (selected.value.source) {
            const auto& command = *selected.value.source;
            uint64_t mapped_command = 0;
            if (!execution.active_clock->map_rx_us(command.raw_timestamp, mapped_command)
                || mapped_command != command.source_us) {
                d.invalidate_sync_execution("DS402 feedback source command clock mapping changed", XNET_CO_TIMING_REASON_CLOCK);
                return;
            }
            if (decision < command.source_us || command.epsilon_us > d.model->drive_command_max_age_us
                || decision - command.source_us > d.model->drive_command_max_age_us - command.epsilon_us) {
                d.invalidate_sync_execution("DS402 feedback source command stale", XNET_CO_TIMING_REASON_STALE);
                return;
            }
        }
        const auto& frame = selected.value.converted;
        if (!(frame.flags & MOTOR_DRIVE_VELOCITY_VALID)
            || !d.drive_402->select_feedback({frame.actual_position, frame.actual_velocity,
                frame.fault_code != 0 || frame.action_result == CO_402_ACTION_RESULT_ERROR,
                (frame.flags & 0x08U) != 0})) {
            d.invalidate_sync_execution("DS402 selected feedback is incomplete");
            return;
        }
        if (execution.active_test_auto_retire) {
            const uint64_t cutoff = source - plan.delay_us;
            const uint64_t age = plan.max_age_us - plan.delay_us - elapsed - plan.epsilon_us;
            const uint64_t oldest = cutoff > age ? cutoff - age : 0;
            if (execution.feedback.retire_before(d.drive_epoch, oldest) != xco::TimedHistoryResult::ok) {
                d.invalidate_sync_execution("DS402 feedback history retirement failed", XNET_CO_TIMING_REASON_HISTORY_INVALID);
                return;
            }
        }
        feedback_selected = true;
    }
    if (!d.drive_402->status().output_allowed) interval = 0;
    d.process_drive_402(interval);
    if (execution.invalid) return;
    if (feedback_selected) {
        d.record_timing_event(XNET_CO_TIMING_FEEDBACK_SELECTED);
        context.flags &= ~(XNET_CO_TIMING_HAS_SOURCE | XNET_CO_TIMING_HAS_SOURCE_CYCLE
            | XNET_CO_TIMING_HAS_TARGET | XNET_CO_TIMING_HAS_FEEDBACK | XNET_CO_TIMING_HAS_SOURCE_TIME
            | XNET_CO_TIMING_HAS_AVAILABLE_TIME | XNET_CO_TIMING_HAS_DECISION_TIME
            | XNET_CO_TIMING_HAS_SUBMITTED_STEADY_TIME);
    }
    if (d.model->drive_402_external && execution.active_test_external_capture) {
        uint64_t available = 0;
        if (!d.sync_now(available)) {
            d.invalidate_sync_execution("DS402 command publication clock invalid", XNET_CO_TIMING_REASON_CLOCK);
            return;
        }
        const xco::TimedDriveCommand command{d.drive_command, event,
            d.drive_processed_feedback_seq, d.drive_processed_result};
        const auto result = execution.commands.append({d.drive_epoch, event.sequence, source, available, command});
        if (result != xco::TimedHistoryResult::ok) {
            d.invalidate_sync_execution(result == xco::TimedHistoryResult::overflow
                ? "DS402 timed command history overflow" : "DS402 timed command history invalid", timing_history_reason(result));
            return;
        }
        {
            xco::TimingDiagnosticScope publication(&execution);
            auto& published = *publication.get();
            published.flags |= XNET_CO_TIMING_HAS_SOURCE | XNET_CO_TIMING_HAS_SOURCE_TIME
                | XNET_CO_TIMING_HAS_AVAILABLE_TIME;
            published.source = {d.drive_command.epoch, d.drive_command.command_seq, d.drive_command.action_seq};
            published.source_us = source;
            published.available_us = available;
            d.record_timing_event(XNET_CO_TIMING_COMMAND_PUBLISHED);
        }
        if (execution.active_test_ordered_sync_watermark) {
            if (execution.commands.close_through(d.drive_epoch, source, available) != xco::TimedHistoryResult::ok) {
                d.invalidate_sync_execution("ordered SYNC command watermark invalid", XNET_CO_TIMING_REASON_WATERMARK);
                return;
            }
            execution.test_sync_watermark_anchor = xco::TestSyncWatermarkAnchor{event.timestamp_100ns, source};
        }
    }
    CO_process_TPDO(d.co, true, 0, nullptr);
}

CO_NMT_reset_cmd_t xnet_canopen_device::process_network(uint32_t dt_us) {
    const auto before = CO_NMT_getInternalState(co->NMT);
    const auto reset = CO_process(co, false, dt_us, nullptr);
    if (drive_402 && before == CO_NMT_OPERATIONAL && CO_NMT_getInternalState(co->NMT) != before) {
        drive_controlword_rearm = true;
        process_drive_402(0, false, true);
    }
    return reset;
}

void xnet_canopen_device::process_sync_iteration(uint32_t dt_us) {
    auto reset = process_network(dt_us);
    if (reset != CO_RESET_NOT) {
        reset_stack(reset == CO_RESET_APP);
        return;
    }
    if (CO_NMT_getInternalState(co->NMT) != CO_NMT_OPERATIONAL) {
        sync_execution->test_rpdo_seen = 0;
        sync_execution->test_rpdo_last_us.reset();
    }
    if (switching != Switch::none || stop_requested) return;
    if (!co->nodeIdUnconfigured) {
        CO_process_RPDO(co, false, dt_us, nullptr);
        CO_process_TPDO(co, false, dt_us, nullptr);
        process_drive_402(dt_us, false, true);
    }
    if (!sync_execution->invalid && sync_execution->active_test_sampling_plan) {
        uint64_t now = 0;
        if (!sync_now(now)) invalidate_sync_execution("DS402 fixed sampling clock invalid", XNET_CO_TIMING_REASON_CLOCK);
        else advance_test_sampling(now);
    }
    receive(process_sync_boundary);
    if (switching != Switch::none || stop_requested) return;
    if (!co->nodeIdUnconfigured) process_drive_402(0, false, true);
    auto& execution = *sync_execution;
    if (!execution.invalid && execution.started_us) {
        uint64_t now = 0;
        const uint64_t anchor = execution.last_sync_us.value_or(*execution.started_us);
        const bool valid_clock = sync_now(now) && now >= anchor;
        if (!valid_clock || now - anchor > execution.active_max_interval_us)
            invalidate_sync_execution("SYNC clock invalid or SYNC interval exceeded",
                valid_clock ? XNET_CO_TIMING_REASON_DEADLINE : XNET_CO_TIMING_REASON_CLOCK);
        else if (execution.last_sync_us && now - *execution.last_sync_us >= execution.active_tpdo_window_us
                 && std::any_of(outgoing.begin(), outgoing.end(),
                                [](const Pending& frame) { return frame.buffer->syncFlag; })) {
            xco::TimingDiagnosticScope diagnostic(&execution);
            if (execution.last_sync_event) {
                diagnostic.get()->flags |= XNET_CO_TIMING_HAS_BOUNDARY;
                diagnostic.get()->boundary = *execution.last_sync_event;
            }
            invalidate_sync_execution("synchronous TPDO expired before submission", XNET_CO_TIMING_REASON_DEADLINE);
        }
        else if (!co->nodeIdUnconfigured) {
            const uint64_t age = now - anchor;
            if (age < co->SYNC->timer)
                invalidate_sync_execution("SYNC clock regressed relative to protocol timer", XNET_CO_TIMING_REASON_CLOCK);
            else if (CO_process_SYNC(co, static_cast<uint32_t>(age - co->SYNC->timer), nullptr))
                invalidate_sync_execution("locally produced SYNC is not supported by this timing source");
        }
    }
    if (execution.invalid && !co->nodeIdUnconfigured) CO_process_SYNC(co, dt_us, nullptr);
    if (!stop_requested) transmit();
}

/**
 * Sole owner of frame dispatch and protocol time advancement while running.
 * Elapsed time is measured between loop starts, not assumed equal to the wait
 * interval. Host batch calls can interleave only while the runtime lock is
 * released; the external model is not executed between RPDO and TPDO processing.
 */
void xnet_canopen_device::run() {
    std::unique_lock<std::recursive_mutex> lock(mutex);
    worker_lock = &lock;
    auto previous = xco::Clock::now();
    CO_NMT_internalState_t previous_nmt = CO_NMT_UNKNOWN;
    bool previous_guard_timeout = false;
    uint8_t previous_pending_node = pending_node;
    uint16_t previous_pending_bitrate = pending_bitrate;
    try {
        while (!stop_requested) {
            auto now = xco::Clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - previous).count();
            previous = now;
            uint32_t dt = static_cast<uint32_t>(std::min<int64_t>(elapsed, UINT32_MAX));
            if (switching != Switch::none) {
                // Neither protocol processing nor queued transmission may break LSS silence.
                if (sync_execution && !co->nodeIdUnconfigured) process_drive_402(dt, false, true);
                if (now >= switch_deadline) {
                    if (switching == Switch::before) {
                        adapter.bitrate(pending_bitrate);
                        active_bitrate = pending_bitrate;
                        log(xco::LogLevel::info, "LSS bitrate applied: post-switch silence");
                        switching = Switch::after;
                        switch_deadline = xco::Clock::now() + std::chrono::milliseconds(switch_delay_ms);
                    } else {
                        adapter.start();
                        switching = Switch::none;
                        log(xco::LogLevel::info, "LSS bitrate activation completed");
                    }
                }
            } else {
                if (sync_execution) {
                    process_sync_iteration(dt);
                } else {
                    bool processed_sync = false;
                    const auto boundary = [](xnet_canopen_device& d, const xnet_canopen_sync_event_t&, void* context) {
                        *static_cast<bool*>(context) = true;
                        d.process_drive_402(0);
                        CO_process_TPDO(d.co, true, 0, nullptr);
                        d.transmit();
                    };
                    if (drive_402) receive(boundary, &processed_sync);
                    else receive();
                    // Receive dispatch can request a reset, stop, or bitrate switch in a callback.
                    if (switching == Switch::none && !stop_requested) {
                        auto reset = process_network(dt);
                        if (switching == Switch::none && !stop_requested) {
                            if (reset != CO_RESET_NOT) reset_stack(reset == CO_RESET_APP);
                            else if (!co->nodeIdUnconfigured) {
                                // Apply RPDO commands before packing values for this cycle's TPDOs.
                                bool_t sync = CO_process_SYNC(co, processed_sync ? 0 : dt, nullptr);
                                CO_process_RPDO(co, sync, dt, nullptr);
                                process_drive_402(dt);
                                CO_process_TPDO(co, sync, dt, nullptr);
                            }
                            if (!stop_requested) transmit();
                        }
                    }
                }
            }
            const auto current_nmt = CO_NMT_getInternalState(co->NMT);
            if (current_nmt != previous_nmt) {
                log(xco::LogLevel::info, nmt_log_message(current_nmt));
                previous_nmt = current_nmt;
            }
            const bool guard_timeout = co->NGslave->lifeTimeTimeout != 0;
            if (guard_timeout != previous_guard_timeout) {
                log(guard_timeout ? xco::LogLevel::warning : xco::LogLevel::info,
                    guard_timeout ? "life guarding timed out" : "life guarding timeout cleared");
                previous_guard_timeout = guard_timeout;
            }
            if (pending_node != previous_pending_node || pending_bitrate != previous_pending_bitrate) {
                log(xco::LogLevel::info, "LSS pending settings: node_id={} bitrate_kbps={}",
                    unsigned(pending_node), unsigned(pending_bitrate));
                previous_pending_node = pending_node;
                previous_pending_bitrate = pending_bitrate;
            }
            // wait_for releases the OD lock, allowing channel calls between worker cycles.
            wake.wait_for(lock, std::chrono::microseconds(model->worker_period_us), [&] { return stop_requested.load(); });
        }
        if (!drain_drive_402()) worker_result = XNET_CO_STACK_ERROR;
    } catch (const Error& e) {
        worker_result = e.code;
        fail(e.code, e.what()); state = XNET_CO_RUNTIME_FAILED; stop_requested = true;
    } catch (const std::exception& e) {
        worker_result = XNET_CO_INTERNAL_ERROR;
        fail(XNET_CO_INTERNAL_ERROR, e.what()); state = XNET_CO_RUNTIME_FAILED; stop_requested = true;
    }
    try { adapter.stop_flush(); }
    catch (const Error& e) {
        if (worker_result == XNET_CO_OK) worker_result = e.code;
        fail(e.code, e.what()); state = XNET_CO_RUNTIME_FAILED;
    }
    for (auto& pending : outgoing) pending.buffer->bufferFull = false;
    outgoing.clear();
    if (co) co->CANmodule->CANnormal = false;
    worker_lock = nullptr;
}

/**
 * Join outside the runtime lock, then discard traffic while retaining the OD.
 * Public callers keep lifecycle ownership across this unlocked phase so another
 * host call cannot start/free resources before shutdown has completed.
 */
void xnet_canopen_device::stop() {
    const bool had_worker = worker.joinable();
    stop_requested = true;
    wake.notify_all();
    // The worker reacquires the OD lock during shutdown; never join while holding it.
    if (worker.joinable()) worker.join();
    std::lock_guard<std::recursive_mutex> lock(mutex);
    initialized();
    adapter.stop_flush();
    outgoing.clear();
    switching = Switch::none;
    co->CANmodule->CANnormal = false;
    if (state != XNET_CO_RUNTIME_FAILED) state = XNET_CO_RUNTIME_STOPPED;
    if (had_worker && drive_402) drive_controlword_rearm = true;
    if (had_worker) log(xco::LogLevel::info, "device stopped");
}

/**
 * Release in dependency order: join worker, close XNET, free stack, then OD.
 * If session release fails, leave stack/model resources available for a cleanup
 * retry instead of invalidating the handle while hardware references remain.
 */
void xnet_canopen_device::release() {
    if (worker.joinable()) stop();
    std::lock_guard<std::recursive_mutex> lock(mutex);
    adapter.close();
    if (drive_endpoint) {
        drive_endpoint->device = nullptr;
        drive_endpoint->writer_open = false;
        drive_endpoint.reset();
    }
    can_state = {};
    outgoing.clear();
    if (co || model) log(xco::LogLevel::info, "device uninitialized");
    if (co) { CO_delete(co); co = nullptr; }
    drive_402.reset();
    sync_execution.reset();
    model.reset();
    batches.clear();
    interface_name.clear(); persistence_path.clear();
    state = XNET_CO_RUNTIME_UNINITIALIZED;
}

extern "C" {
int32_t xnet_canopen_create(xnet_canopen_handle* handle) {
    if (!handle) return XNET_CO_INVALID_ARGUMENT;
    *handle = nullptr;
    try { *handle = new xnet_canopen_device; return XNET_CO_OK; }
    catch (const std::exception&) { return XNET_CO_INTERNAL_ERROR; }
}

int32_t xnet_canopen_destroy(xnet_canopen_handle handle) {
    // Delete only after the guarded cleanup succeeds; failures retain diagnostics
    // and a valid context for the caller's explicit cleanup retry.
    auto result = call(handle, [](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>& lock) {
        lock.unlock(); d.release(); return XNET_CO_OK;
    });
    if (result == XNET_CO_OK) delete handle;
    return result;
}

int32_t xnet_canopen_initialize(xnet_canopen_handle handle, const char* interface_name,
                               const char* model_path, const char* persistence_path) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>& lock) {
        require(d.state == XNET_CO_RUNTIME_UNINITIALIZED, XNET_CO_INVALID_STATE, "initialize requires an uninitialized handle");
        require(interface_name && *interface_name && model_path && *model_path && persistence_path && *persistence_path,
                XNET_CO_INVALID_ARGUMENT, "interface, JSON model and persistence paths are required");
        d.can_state = {};
        try {
            d.interface_name = interface_name;
            d.logger = xco::file_logger();
            d.log_identity = {};
            d.persistence_path = persistence_path;
            d.model = std::make_unique<xco::Model>();
            d.model->load(xco::read_json(model_path));
            if (d.model->drive_adapter) {
                const auto unit_report = d.model->drive_adapter->report();
                d.log(xco::LogLevel::info, "drive unit conversion report: {}", unit_report.dump());
                if (!unit_report["assumed_identity"].empty())
                    d.log(xco::LogLevel::warning, "unconfirmed drive units use identity scales; review the unit conversion report");
                if (d.model->drive_402_external) {
                    d.drive_endpoint = std::make_shared<motor_drive_endpoint>();
                    d.drive_endpoint->device = &d;
                }
            }
            if (d.model->drive_402_enabled) {
                d.drive_402 = std::make_unique<xco::d402::Drive>(
                    d.model->drive_402_external ? d.model->drive_feedback_timeout_us : 1000000);
                d.drive_402->use_external_model(d.model->drive_402_external);
            }
            d.log_identity = d.model->identity;
            d.load_persistent();
            d.active_bitrate = d.pending_bitrate;
            d.model->restore(true, d.pending_node);
            d.start_drive_epoch();
            d.adapter.open(d.interface_name, d.active_bitrate, d.model->can_termination);
            if (d.model->can_termination.has_value())
                d.log(xco::LogLevel::info, "CAN termination configured: {}", *d.model->can_termination ? "on" : "off");
            d.setup_stack();
            // Commit a new counting lifetime only after initialization succeeds;
            // failed attempts must preserve the previous lifetime's counts.
            d.reset_node_count = 0;
            d.reset_communication_count = 0;
            d.state = XNET_CO_RUNTIME_STOPPED;
            d.log(xco::LogLevel::info, "device initialized; communication not started");
        } catch (const std::exception& original) {
            const std::string reason = original.what();
            lock.unlock();
            // A failed cleanup retains resources so uninitialize can retry safely.
            try { d.release(); }
            catch (const std::exception& cleanup) {
                d.state = XNET_CO_RUNTIME_FAILED;
                throw Error(XNET_CO_IO_ERROR, reason + "; cleanup failed: " + cleanup.what() + "; retry uninitialize");
            }
            throw;
        }
        return XNET_CO_OK;
    });
}

int32_t xnet_canopen_uninitialize(xnet_canopen_handle handle) {
    return call(handle, [](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>& lock) {
        lock.unlock(); d.release(); return XNET_CO_OK;
    });
}
int32_t xnet_canopen_start(xnet_canopen_handle handle) {
    return call(handle, [](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        d.initialized();
        if (d.state == XNET_CO_RUNTIME_RUNNING) return XNET_CO_OK;
        require(d.state == XNET_CO_RUNTIME_STOPPED, XNET_CO_INVALID_STATE, "uninitialize a failed device before restarting");
        if (d.sync_execution) {
            d.start_test_timing_provider();
            d.sync_execution->last_now_us.reset();
            uint64_t now = 0;
            require(d.sync_now(now), XNET_CO_INVALID_STATE, "SYNC clock is unavailable");
            d.sync_execution->last_sync_us.reset();
            d.sync_execution->last_sync_event.reset();
            d.sync_execution->last_counter.reset();
            d.sync_execution->started_us = now;
            d.sync_execution->invalid = false;
            d.sync_execution->invalid_reason = {};
        }
        try {
            // A new stack restarts protocol state without resetting live OD
            // values. Launch only after sessions and CANnormal are ready.
            if (d.drive_402) d.drive_402->reset();
            d.start_drive_epoch();
            d.setup_stack();
            d.start_test_rpdo_plan();
            d.adapter.start();
            CO_CANsetNormalMode(d.co->CANmodule);
            d.stop_requested = false;
            d.worker_result = XNET_CO_OK;
            d.state = XNET_CO_RUNTIME_RUNNING;
            d.worker = std::thread(&xnet_canopen_device::run, &d);
            d.log(xco::LogLevel::info, "device started");
        } catch (...) { d.state = XNET_CO_RUNTIME_FAILED; throw; }
        return XNET_CO_OK;
    });
}
int32_t xnet_canopen_stop(xnet_canopen_handle handle) {
    if (!handle) return XNET_CO_INVALID_ARGUMENT;
    bool owns_shutdown = false;
    try {
        std::unique_lock<std::mutex> lifecycle(handle->lifecycle);
        std::unique_lock<std::recursive_mutex> runtime(handle->mutex);
        require(!handle->shutdown_joining, XNET_CO_INVALID_STATE, "device shutdown is already in progress");
        handle->initialized();
        if (!handle->drive_402) {
            runtime.unlock();
            handle->stop();
            return XNET_CO_OK;
        }
        if (!handle->worker.joinable()) {
            runtime.unlock();
            lifecycle.unlock();
            return call(handle, [](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>& lock) {
                lock.unlock(); d.stop(); lock.lock(); return d.worker_result;
            });
        }
        handle->shutdown_joining = true;
        owns_shutdown = true;
        handle->stop_requested = true;
        handle->wake.notify_all();
        runtime.unlock();
        lifecycle.unlock();
        handle->worker.join();
        lifecycle.lock();
        runtime.lock();
        handle->shutdown_joining = false;
        owns_shutdown = false;
        handle->stop();
        return handle->worker_result;
    } catch (const Error& e) {
        std::lock_guard<std::recursive_mutex> lock(handle->mutex);
        if (owns_shutdown) handle->shutdown_joining = false;
        handle->fail(e.code, e.what());
        return e.code;
    } catch (const std::exception& e) {
        std::lock_guard<std::recursive_mutex> lock(handle->mutex);
        if (owns_shutdown) handle->shutdown_joining = false;
        handle->fail(XNET_CO_INTERNAL_ERROR, e.what());
        return XNET_CO_INTERNAL_ERROR;
    }
}

int32_t xnet_canopen_bind_batch(xnet_canopen_handle handle, const uint16_t* indices,
                               const uint8_t* subindices, uint32_t count, uint32_t* types,
                               uint32_t* sizes, uint32_t* buffer_size, uint32_t* batch_id) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        d.initialized();
        require(d.state != XNET_CO_RUNTIME_FAILED, XNET_CO_INVALID_STATE, "device failed; inspect diagnostics and uninitialize");
        require(indices && subindices && types && sizes && buffer_size && batch_id && count > 0 && count <= 4096,
                XNET_CO_INVALID_ARGUMENT, "invalid batch arguments");
        require(d.next_batch != 0, XNET_CO_INVALID_STATE, "batch ID space exhausted");
        static const std::map<std::string, uint32_t> type_codes = {
            {"BOOLEAN", XNET_CO_TYPE_BOOLEAN}, {"INTEGER8", XNET_CO_TYPE_INTEGER8},
            {"INTEGER16", XNET_CO_TYPE_INTEGER16}, {"INTEGER32", XNET_CO_TYPE_INTEGER32},
            {"INTEGER64", XNET_CO_TYPE_INTEGER64}, {"UNSIGNED8", XNET_CO_TYPE_UNSIGNED8},
            {"UNSIGNED16", XNET_CO_TYPE_UNSIGNED16}, {"UNSIGNED32", XNET_CO_TYPE_UNSIGNED32},
            {"UNSIGNED64", XNET_CO_TYPE_UNSIGNED64}, {"REAL32", XNET_CO_TYPE_REAL32},
            {"REAL64", XNET_CO_TYPE_REAL64}, {"VISIBLE_STRING", XNET_CO_TYPE_VISIBLE_STRING},
            {"OCTET_STRING", XNET_CO_TYPE_OCTET_STRING}};
        xco::Batch batch;
        std::vector<uint32_t> resolved_types;
        for (uint32_t i = 0; i < count; ++i) {
            // Slots are byte-packed; scalar alignment must not add padding.
            const auto& value = d.model->value(indices[i], subindices[i]);
            const xco::Binding b{indices[i], subindices[i], batch.size, value.size};
            require(b.size <= 1024 * 1024 - batch.size,
                    XNET_CO_INVALID_ARGUMENT, "batch total size exceeds 1 MiB");
            for (const auto& previous : batch.bindings) {
                require(b.index != previous.index || b.subindex != previous.subindex, XNET_CO_INVALID_ARGUMENT, "duplicate batch OD variable");
            }
            batch.size += b.size;
            batch.bindings.push_back(b);
            resolved_types.push_back(type_codes.at(value.type));
        }
        const auto inserted = d.batches.emplace(d.next_batch, std::move(batch));
        const auto& stored = inserted.first->second;
        for (uint32_t i = 0; i < count; ++i) {
            types[i] = resolved_types[i];
            sizes[i] = stored.bindings[i].size;
        }
        *buffer_size = stored.size;
        *batch_id = d.next_batch++;
        return XNET_CO_OK;
    });
}

static void require_od_access(xnet_canopen_device& d) {
    d.initialized();
    require(d.state != XNET_CO_RUNTIME_FAILED, XNET_CO_INVALID_STATE, "device failed; inspect diagnostics and uninitialize");
}

/**
 * Per-entry OD access shared by batch and single-entry calls; caller holds the
 * runtime lock and has validated buffer size against v.size. Fresh OD streams
 * preserve service callbacks after resets and between transfers.
 */
static int32_t entry_io(xnet_canopen_device& d, uint16_t index, uint8_t subindex, const xco::Value& v,
                        uint8_t* read_buffer, const uint8_t* write_buffer, bool is_write) {
    // Local-write ownership is separate from network SDO permissions.
    // Local reads currently still require the network SDO-readable flag.
    if ((is_write && !v.allows_local_write) || (!is_write && !(v.attr & ODA_SDO_R))) return XNET_CO_ACCESS_ERROR;
    OD_IO_t io{};
    // Stack resets replace extensions. Resolve a fresh stream rather than caching callbacks.
    auto result = OD_getSub(OD_find(&d.model->od, index), subindex, &io, false);
    OD_size_t transferred = 0;
    if (result == ODR_OK) {
        if (is_write && v.type == "BOOLEAN" && write_buffer[0] > 1) result = ODR_INVALID_VALUE;
        else if (is_write && v.type == "VISIBLE_STRING" && !std::memchr(write_buffer, 0, v.size)) result = ODR_INVALID_VALUE;
        else if (is_write) result = io.write(&io.stream, write_buffer, v.size, &transferred);
        else result = io.read(&io.stream, read_buffer, v.size, &transferred);
    }
    return result == ODR_OK && transferred == v.size ? XNET_CO_OK : XNET_CO_ACCESS_ERROR;
}

/**
 * Shared batch executor. Validate the whole buffer/layout before touching values,
 * then attempt every entry independently under one runtime lock. This prevents
 * worker interleaving, not partial commits: earlier writes are not rolled back.
 */
static int32_t batch_io(xnet_canopen_handle handle, uint32_t batch_id, uint8_t* read_buffer, const uint8_t* write_buffer,
                        uint32_t buffer_size, int32_t* statuses, uint32_t count, bool is_write) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        require_od_access(d);
        auto found = d.batches.find(batch_id);
        require(found != d.batches.end(), XNET_CO_NOT_FOUND, "unknown or invalidated batch ID");
        const auto& batch = found->second;
        require(statuses && (is_write ? write_buffer != nullptr : read_buffer != nullptr) && count == batch.bindings.size(),
                XNET_CO_INVALID_ARGUMENT, "buffer/status pointer or entry count mismatch");
        require(buffer_size >= batch.size, XNET_CO_BUFFER_TOO_SMALL, "batch buffer too small");
        bool has_entry_errors = false;
        for (uint32_t i = 0; i < count; ++i) {
            const auto& b = batch.bindings[i];
            statuses[i] = entry_io(d, b.index, b.subindex, d.model->value(b.index, b.subindex),
                is_write ? nullptr : read_buffer + b.offset, is_write ? write_buffer + b.offset : nullptr, is_write);
            if (statuses[i] < 0) {
                has_entry_errors = true;
                d.fail(statuses[i], "batch entry " + std::to_string(i) + " OD " + std::to_string(b.index)
                    + ":" + std::to_string(b.subindex) + (is_write ? " write failed" : " read failed"));
            }
        }
        return has_entry_errors ? XNET_CO_PARTIAL_FAILURE : XNET_CO_OK;
    });
}

int32_t xnet_canopen_read_batch(xnet_canopen_handle handle, uint32_t id, uint8_t* buffer,
                               uint32_t size, int32_t* statuses, uint32_t count) {
    return batch_io(handle, id, buffer, nullptr, size, statuses, count, false);
}
int32_t xnet_canopen_write_batch(xnet_canopen_handle handle, uint32_t id, const uint8_t* buffer,
                                uint32_t size, int32_t* statuses, uint32_t count) {
    return batch_io(handle, id, nullptr, buffer, size, statuses, count, true);
}
int32_t xnet_canopen_read_od(xnet_canopen_handle handle, uint16_t index, uint8_t subindex,
                            uint8_t* buffer, uint32_t buffer_size, uint32_t* value_size) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) -> int32_t {
        require_od_access(d);
        require(value_size && (buffer || buffer_size == 0), XNET_CO_INVALID_ARGUMENT, "invalid OD read arguments");
        const auto& v = d.model->value(index, subindex);
        *value_size = v.size;
        // A size query must not replace the latest diagnostic.
        if (!buffer) return XNET_CO_BUFFER_TOO_SMALL;
        require(buffer_size >= v.size, XNET_CO_BUFFER_TOO_SMALL, "OD read buffer too small");
        const auto result = entry_io(d, index, subindex, v, buffer, nullptr, false);
        if (result < 0) d.fail(result, "OD " + std::to_string(index) + ":" + std::to_string(subindex) + " read failed");
        return result;
    });
}
int32_t xnet_canopen_write_od(xnet_canopen_handle handle, uint16_t index, uint8_t subindex,
                             const uint8_t* buffer, uint32_t buffer_size) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) -> int32_t {
        require_od_access(d);
        require(buffer != nullptr, XNET_CO_INVALID_ARGUMENT, "OD write buffer is required");
        const auto& v = d.model->value(index, subindex);
        require(buffer_size == v.size, XNET_CO_INVALID_ARGUMENT, "OD write size must equal the OD length");
        const auto result = entry_io(d, index, subindex, v, nullptr, buffer, true);
        if (result < 0) d.fail(result, "OD " + std::to_string(index) + ":" + std::to_string(subindex) + " write failed");
        return result;
    });
}
int32_t xnet_canopen_request_tpdo(xnet_canopen_handle handle, uint16_t number) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        d.initialized();
        require(d.state == XNET_CO_RUNTIME_RUNNING && d.switching == xnet_canopen_device::Switch::none && !d.co->nodeIdUnconfigured
            && CO_NMT_getInternalState(d.co->NMT) == CO_NMT_OPERATIONAL, XNET_CO_INVALID_STATE, "TPDO request requires Operational running device");
        require(number >= 1 && number <= d.model->tpdo_count, XNET_CO_INVALID_ARGUMENT, "invalid one-based TPDO number");
        auto& tpdo = d.co->TPDO[number - 1];
        require(tpdo.PDO_common.valid && (tpdo.transmissionType == 0 || tpdo.transmissionType >= 254),
            XNET_CO_INVALID_STATE, "TPDO disabled or not application-request driven");
        CO_TPDOsendRequest(&tpdo);
        return XNET_CO_OK;
    });
}
int32_t xnet_canopen_set_fault(xnet_canopen_handle handle, uint8_t bit, uint16_t code, uint32_t info, uint8_t active) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        d.initialized();
        require(d.state == XNET_CO_RUNTIME_RUNNING && !d.co->nodeIdUnconfigured, XNET_CO_INVALID_STATE, "fault requires initialized running node");
        require(bit >= 0x40 && bit <= 0x4f && active <= 1 && (!active || code != 0),
            XNET_CO_INVALID_ARGUMENT, "invalid manufacturer fault bit/code/active");
        const bool changed = (CO_isError(d.co->em, bit) != 0) != (active != 0);
        CO_error(d.co->em, active != 0, bit, active ? code : CO_EMC_NO_ERROR, info);
        if (changed) {
            d.log(active ? xco::LogLevel::warning : xco::LogLevel::info,
                "fault {}: bit=0x{:02x} code=0x{:04x} info=0x{:08x}",
                active ? "asserted" : "cleared", unsigned(bit), active ? unsigned(code) : 0U, unsigned(info));
        }
        return XNET_CO_OK;
    });
}
static xnet_canopen_status_t communication_status(const xnet_canopen_device& d) {
    xnet_canopen_status_t status{};
    status.runtime_state = d.state;
    status.reset_node_count = d.reset_node_count;
    status.reset_communication_count = d.reset_communication_count;
    status.can_state_raw = d.can_state.raw;
    status.can_state_valid = d.can_state.valid ? 1U : 0U;
    if (d.co) {
        status.nmt_state = CO_NMT_getInternalState(d.co->NMT);
        status.can_error_status = d.co->CANmodule->CANerrorStatus;
        status.active_node_id = d.active_node; status.pending_node_id = d.pending_node;
        status.active_bitrate_kbps = d.active_bitrate; status.pending_bitrate_kbps = d.pending_bitrate;
        status.node_guarding_timeout = d.co->NGslave->lifeTimeTimeout ? 1U : 0U;
    }
    return status;
}
static uint32_t public_pds(xco::d402::Pds pds) {
    switch (pds) {
        case xco::d402::Pds::not_ready_to_switch_on: return XNET_CO_PDS_NOT_READY;
        case xco::d402::Pds::switch_on_disabled: return XNET_CO_PDS_SWITCH_ON_DISABLED;
        case xco::d402::Pds::ready_to_switch_on: return XNET_CO_PDS_READY_TO_SWITCH_ON;
        case xco::d402::Pds::switched_on: return XNET_CO_PDS_SWITCHED_ON;
        case xco::d402::Pds::operation_enabled: return XNET_CO_PDS_OPERATION_ENABLED;
        case xco::d402::Pds::quick_stop_active: return XNET_CO_PDS_QUICK_STOP_ACTIVE;
        case xco::d402::Pds::fault_reaction_active: return XNET_CO_PDS_FAULT_REACTION_ACTIVE;
        case xco::d402::Pds::fault: return XNET_CO_PDS_FAULT;
    }
    throw Error(XNET_CO_INTERNAL_ERROR, "invalid drive PDS state");
}
int32_t xnet_canopen_get_status(xnet_canopen_handle handle, xnet_canopen_status_t* status) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        require(status != nullptr, XNET_CO_INVALID_ARGUMENT, "status output is required");
        *status = communication_status(d);
        return XNET_CO_OK;
    });
}
int32_t xnet_canopen_get_system_status_v1(xnet_canopen_handle handle, xnet_canopen_system_status_v1* status,
                                           uint32_t capacity) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        require(status != nullptr, XNET_CO_INVALID_ARGUMENT, "system status output is required");
        require(capacity >= sizeof(*status), XNET_CO_BUFFER_TOO_SMALL, "system status buffer too small");
        xnet_canopen_system_status_v1 output{};
        output.abi_version = XNET_CO_SYSTEM_ABI_VERSION;
        output.struct_size = sizeof(output);
        output.communication = communication_status(d);
        if (d.drive_402) {
            const auto drive = d.drive_402->status();
            output.drive_valid = 1;
            output.pds_state = public_pds(drive.pds);
            output.statusword = drive.statusword;
            output.mode_display = drive.mode_display;
            if (drive.feedback_fresh) output.drive_flags |= XNET_CO_DRIVE_STATUS_FEEDBACK_FRESH;
            if (drive.model_fault) output.drive_flags |= XNET_CO_DRIVE_STATUS_MODEL_FAULT;
            if (drive.model_ready) output.drive_flags |= XNET_CO_DRIVE_STATUS_MODEL_READY;
            if (drive.action_pending) output.drive_flags |= XNET_CO_DRIVE_STATUS_ACTION_PENDING;
            if (drive.output_allowed) output.drive_flags |= XNET_CO_DRIVE_STATUS_OUTPUT_ALLOWED;
            if (drive.reaction_unconfirmed) output.drive_flags |= XNET_CO_DRIVE_STATUS_REACTION_UNCONFIRMED;
            if (d.drive_endpoint && d.drive_endpoint->writer_open) output.drive_flags |= XNET_CO_DRIVE_STATUS_PORT_OPEN;
            output.action = drive.action;
            output.action_seq = drive.action_seq;
        }
        *status = output;
        return XNET_CO_OK;
    });
}
int32_t xnet_canopen_get_capabilities_v1(xnet_canopen_handle handle, xnet_canopen_capabilities_v1* capabilities,
                                         uint32_t capacity) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        require(capabilities != nullptr, XNET_CO_INVALID_ARGUMENT, "capabilities output is required");
        require(capacity >= sizeof(*capabilities), XNET_CO_BUFFER_TOO_SMALL, "capabilities buffer too small");
        d.initialized();
        xnet_canopen_capabilities_v1 output{};
        output.abi_version = XNET_CO_SYSTEM_ABI_VERSION;
        output.struct_size = sizeof(output);
        output.capabilities = XNET_CO_CAP_OD_BATCH | XNET_CO_CAP_TPDO_REQUEST
            | XNET_CO_CAP_MANUFACTURER_FAULT | XNET_CO_CAP_SYNC_OBSERVATION;
        if (d.model->drive_402_enabled) {
            output.implemented_modes = XNET_CO_MODE_PROFILE_POSITION;
            output.drive_model = d.model->drive_402_external ? XNET_CO_DRIVE_EXTERNAL : XNET_CO_DRIVE_INTERNAL;
            if (d.model->drive_402_external)
                output.capabilities |= XNET_CO_CAP_MOTOR_ASYNC | XNET_CO_CAP_MOTOR_BOOTSTRAP | XNET_CO_CAP_MOTOR_SAFETY;
        }
        *capabilities = output;
        return XNET_CO_OK;
    });
}
int32_t xnet_canopen_get_diagnostic_v1(xnet_canopen_handle handle, xnet_canopen_diagnostic_v1* diagnostic,
                                       uint32_t capacity) {
    if (!handle || !diagnostic) return XNET_CO_INVALID_ARGUMENT;
    if (capacity < sizeof(*diagnostic)) return XNET_CO_BUFFER_TOO_SMALL;
    std::lock_guard<std::recursive_mutex> lock(handle->mutex);
    xnet_canopen_diagnostic_v1 output{};
    output.abi_version = XNET_CO_SYSTEM_ABI_VERSION;
    output.struct_size = sizeof(output);
    output.source_domain = handle->diagnostic_domain;
    output.source_code = handle->diagnostic_source_code;
    output.backend_code = handle->error_code;
    output.message_required = static_cast<uint32_t>(std::strlen(handle->error_message.data()) + 1);
    std::memcpy(output.message, handle->error_message.data(), output.message_required);
    *diagnostic = output;
    return XNET_CO_OK;
}
int32_t xnet_canopen_get_last_error(xnet_canopen_handle handle, int32_t* code, char* message, uint32_t capacity, uint32_t* required) {
    // Diagnostic queries must not replace the error they are trying to retrieve.
    if (!handle || !code || !required || (!message && capacity)) return XNET_CO_INVALID_ARGUMENT;
    std::lock_guard<std::recursive_mutex> lock(handle->mutex);
    *code = handle->error_code;
    *required = static_cast<uint32_t>(std::strlen(handle->error_message.data()) + 1);
    if (capacity < *required) return XNET_CO_BUFFER_TOO_SMALL;
    std::memcpy(message, handle->error_message.data(), *required);
    return XNET_CO_OK;
}
int32_t xnet_canopen_clear_error(xnet_canopen_handle handle) {
    return call(handle, [](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        d.fail(0, ""); return XNET_CO_OK;
    });
}

int32_t xnet_canopen_get_motor_drive_endpoint(xnet_canopen_handle handle, motor_drive_endpoint_handle* endpoint) {
    if (endpoint) *endpoint = nullptr;
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        d.initialized();
        require(endpoint != nullptr, XNET_CO_INVALID_ARGUMENT, "endpoint output is required");
        require(d.drive_endpoint && d.model->drive_adapter && d.model->drive_402_external,
            XNET_CO_INVALID_STATE, "external PP device_adapter is required for a motor endpoint");
        *endpoint = d.drive_endpoint.get();
        return XNET_CO_OK;
    });
}
int32_t xnet_canopen_get_drive_unit_report(xnet_canopen_handle handle, char* buffer, uint32_t capacity, uint32_t* required_size) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        d.initialized();
        require(required_size && (buffer || capacity == 0), XNET_CO_INVALID_ARGUMENT, "invalid unit report buffer");
        require(d.model->drive_adapter.has_value(), XNET_CO_INVALID_STATE, "device_adapter is not configured");
        const auto report = d.model->drive_adapter->report().dump(2);
        require(report.size() < UINT32_MAX, XNET_CO_INTERNAL_ERROR, "unit report too large");
        *required_size = static_cast<uint32_t>(report.size() + 1);
        if (!buffer || capacity < *required_size) return XNET_CO_BUFFER_TOO_SMALL;
        std::memcpy(buffer, report.c_str(), *required_size);
        return XNET_CO_OK;
    }, true);
}

int32_t xnet_canopen_read_sync_event(xnet_canopen_handle handle, uint64_t epoch, uint64_t sequence,
                                    xnet_canopen_sync_event_t* event) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        d.initialized();
        require(event && (epoch != 0 || sequence == 0), XNET_CO_INVALID_ARGUMENT, "invalid SYNC observation cursor");
        require(epoch == 0 || epoch == d.sync_observation_epoch, XNET_CO_INVALID_STATE, "SYNC observation epoch changed");
        const auto last = d.sync_observation_sequence;
        const auto oldest = last > d.sync_observations.size() ? last - d.sync_observations.size() + 1 : 1;
        if (epoch != 0) {
            require(sequence <= last, XNET_CO_INVALID_ARGUMENT, "SYNC cursor is ahead of observations");
            require(sequence >= oldest - 1, XNET_CO_ACCESS_ERROR, "SYNC observation history overwritten");
            if (sequence == last) return XNET_CO_NOT_FOUND;
        }
        const auto next = epoch == 0 ? oldest : sequence + 1;
        if (next > last || last == 0) return XNET_CO_NOT_FOUND;
        *event = d.sync_observations[(next - 1) % d.sync_observations.size()];
        return XNET_CO_OK;
    });
}

int32_t xnet_canopen_read_timing_event_v1(xnet_canopen_handle handle, uint64_t sequence,
                                          xnet_canopen_timing_event_v1* event, uint32_t capacity) {
    if (!handle || !event) return XNET_CO_INVALID_ARGUMENT;
    if (capacity < sizeof(*event)) return XNET_CO_BUFFER_TOO_SMALL;
    std::lock_guard<std::recursive_mutex> lock(handle->mutex);
    const auto last = handle->timing_event_sequence;
    const auto oldest = last > handle->timing_events.size() ? last - handle->timing_events.size() + 1 : 1;
    if (sequence > last) return XNET_CO_INVALID_ARGUMENT;
    if (sequence && sequence < oldest - 1) return XNET_CO_ACCESS_ERROR;
    if (sequence == last && sequence != 0)
        return handle->timing_event_exhausted ? XNET_CO_STACK_ERROR : XNET_CO_NOT_FOUND;
    if (!last) return XNET_CO_NOT_FOUND;
    const auto next = sequence == 0 ? oldest : sequence + 1;
    *event = handle->timing_events[(next - 1) % handle->timing_events.size()];
    return XNET_CO_OK;
}

int32_t motor_drive_open_v1(motor_drive_endpoint_handle endpoint, motor_drive_port_handle* port) {
    if (port) *port = nullptr;
    if (!endpoint) return MOTOR_DRIVE_INVALID_ARGUMENT;
    const auto device = endpoint->device.load();
    if (!device) return MOTOR_DRIVE_INVALID_STATE;
    int32_t result = MOTOR_DRIVE_BACKEND_ERROR;
    const auto backend = call(device, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        result = open_motor_port(d, endpoint, port);
        return XNET_CO_OK;
    }, false, true);
    return backend == XNET_CO_OK ? result : motor_result(backend);
}
int32_t motor_drive_read_v1(motor_drive_port_handle port, const motor_drive_read_request_v1* request,
                           motor_drive_view_v1* view, uint32_t capacity) {
    return motor_call(port, [&](xnet_canopen_device& d, motor_drive_port& p) {
        require(request && view, XNET_CO_INVALID_ARGUMENT, "motor request and view are required");
        require(capacity >= sizeof(*view), XNET_CO_BUFFER_TOO_SMALL, "motor view buffer too small");
        require(request->abi_version == MOTOR_DRIVE_ABI_VERSION && request->struct_size == sizeof(*request)
            && request->reserved == 0, XNET_CO_INVALID_ARGUMENT, "invalid motor read request ABI");
        const bool scheduled = request->flags == MOTOR_DRIVE_READ_SCHEDULED;
        xco::TimingDiagnosticScope diagnostic(scheduled && d.sync_execution
            && d.sync_execution->active_test_external_capture ? d.sync_execution.get() : nullptr);
        if (auto* context = diagnostic.get()) {
            context->flags |= XNET_CO_TIMING_HAS_TARGET;
            context->target_step = request->target_step;
        }
        if (!scheduled && (!empty_step(request->target_step) || !empty_time(request->source_cutoff)))
            return motor_reject(d, MOTOR_DRIVE_UNSUPPORTED, "target step and cutoff require scheduled exchange");
        require(scheduled || request->flags == MOTOR_DRIVE_READ_ASYNC || request->flags == MOTOR_DRIVE_READ_BOOTSTRAP
            || request->flags == MOTOR_DRIVE_READ_SAFETY, XNET_CO_INVALID_ARGUMENT, "invalid motor read mode");
        CO_402_command_frame_t command{};
        motor_drive_cycle_v1 cycle{};
        motor_drive_time_v1 publish_time{};
        uint32_t processed_seq = 0;
        uint16_t processed_result = 0;
        uint64_t scheduled_source = 0, scheduled_epsilon = 0, scheduled_target = 0, scheduled_cutoff = 0, scheduled_raw = 0;
        uint64_t scheduled_oldest = 0;
        if (scheduled) {
            if (!d.sync_execution || !d.sync_execution->active_test_external_capture || !d.sync_execution->active_test_command_plan)
                return motor_reject(d, MOTOR_DRIVE_UNSUPPORTED, "scheduled exchange is not available");
            auto& execution = *d.sync_execution;
            const auto& plan = *execution.active_test_command_plan;
            if (execution.invalid) return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "scheduled timing is invalid");
            require(plan.sync_domain_id && plan.clock_domain_id && valid_step(request->target_step)
                && (request->target_step.sample_time.flags & MOTOR_DRIVE_TIME_VALID)
                && (request->source_cutoff.flags & MOTOR_DRIVE_TIME_VALID)
                && !(request->source_cutoff.flags & ~3U) && request->source_cutoff.clock_domain_id
                && std::isfinite(request->source_cutoff.time_s) && request->source_cutoff.time_s >= 0,
                XNET_CO_INVALID_ARGUMENT, "invalid scheduled timing request");
            if (p.pcl_epoch && p.pcl_epoch != request->target_step.pcl_epoch)
                return motor_reject(d, MOTOR_DRIVE_STALE, "scheduled target belongs to another PCL epoch");
            uint64_t target = 0, cutoff = 0, decision = 0;
            require(execution.active_clock->map_model_us(request->target_step.sample_time, target)
                && execution.active_clock->map_model_us(request->source_cutoff, cutoff),
                XNET_CO_INVALID_ARGUMENT, "scheduled target or cutoff clock mapping unavailable");
            require(target >= cutoff && target - cutoff >= plan.lead_us,
                XNET_CO_INVALID_ARGUMENT, "scheduled cutoff violates target lead time");
            auto& timing = *diagnostic.get();
            timing.flags |= XNET_CO_TIMING_HAS_TARGET_TIME | XNET_CO_TIMING_HAS_CUTOFF;
            timing.target_us = target;
            timing.cutoff_us = cutoff;
            if (!d.sync_now(decision)) {
                d.invalidate_sync_execution("scheduled command clock invalid", XNET_CO_TIMING_REASON_CLOCK);
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "scheduled command clock invalid");
            }
            timing.flags |= XNET_CO_TIMING_HAS_DECISION_TIME;
            timing.decision_us = decision;
            if (!d.validate_test_sync_watermark())
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "ordered SYNC watermark clock mapping changed");
            if (!d.apply_test_timing_watermark(true, decision))
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "test timing provider watermark invalid");
            if (decision >= target) {
                d.invalidate_sync_execution("scheduled command deadline missed", XNET_CO_TIMING_REASON_DEADLINE);
                return motor_reject(d, MOTOR_DRIVE_STALE, "scheduled command deadline missed");
            }
            if (execution.active_test_hold_plan
                && !test_hold_step(*execution.active_test_hold_plan, request->target_step, target,
                                   request->target_step, target))
                return motor_reject(d, MOTOR_DRIVE_INVALID_ARGUMENT, "scheduled target violates test hold plan");
            if (p.test_issue_count) {
                const auto& previous = p.test_issues[p.test_issue_count - 1];
                if (same_step(request->target_step, previous.view.target_step)) {
                    if (!same_time(request->source_cutoff, previous.cutoff))
                        return motor_reject(d, MOTOR_DRIVE_INVALID_ARGUMENT, "scheduled target retry changed cutoff");
                    if (target != previous.target_us || cutoff != previous.cutoff_us) {
                        d.invalidate_sync_execution("scheduled command clock mapping changed", XNET_CO_TIMING_REASON_CLOCK);
                        return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "scheduled command clock mapping changed");
                    }
                    if (!d.test_sync_source_valid(previous.raw_source_timestamp, previous.source_us)) {
                        d.invalidate_sync_execution("ordered SYNC source clock mapping changed", XNET_CO_TIMING_REASON_CLOCK);
                        return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "ordered SYNC source clock mapping changed");
                    }
                    if (previous.view.token.action_seq != d.drive_action_seq
                        || ((previous.view.flags & MOTOR_DRIVE_OUTPUT_ALLOWED) && !d.drive_402->status().output_allowed))
                        return motor_reject(d, MOTOR_DRIVE_STALE, "scheduled command was superseded by a safety transition");
                    *view = previous.view;
                    return static_cast<int32_t>(MOTOR_DRIVE_OK);
                }
                if (!advancing_step(request->target_step, previous.view.target_step))
                    return motor_reject(d, MOTOR_DRIVE_STALE, "scheduled target step did not advance");
                if (execution.active_test_hold_plan) {
                    uint64_t previous_target = 0;
                    if (!execution.active_clock->map_model_us(previous.view.target_step.sample_time, previous_target)
                        || previous_target != previous.target_us) {
                        d.invalidate_sync_execution("scheduled command clock mapping changed", XNET_CO_TIMING_REASON_CLOCK);
                        return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "scheduled command clock mapping changed");
                    }
                    if (!test_hold_step(*execution.active_test_hold_plan, previous.view.target_step,
                                        previous.target_us, request->target_step, target))
                        return motor_reject(d, MOTOR_DRIVE_INVALID_ARGUMENT, "scheduled target violates test hold plan");
                }
            }
            if (target - cutoff > plan.max_age_us
                || plan.max_age_us - (target - cutoff) < plan.epsilon_us) {
                d.invalidate_sync_execution("scheduled command age budget is insufficient", XNET_CO_TIMING_REASON_STALE);
                return motor_reject(d, MOTOR_DRIVE_STALE, "scheduled command age budget is insufficient");
            }
            xco::TimedHistory<xco::TimedDriveCommand, 64>::Record selected{};
            const auto result = execution.commands.select(d.drive_epoch, cutoff, plan.epsilon_us, decision, target,
                plan.max_age_us - (target - cutoff) - plan.epsilon_us, selected);
            if (result == xco::TimedHistoryResult::not_closed)
                return motor_reject(d, MOTOR_DRIVE_NO_DATA, "scheduled command watermark not closed");
            if (result != xco::TimedHistoryResult::ok) {
                d.invalidate_sync_execution(result == xco::TimedHistoryResult::deadline_missed
                    ? "scheduled command deadline missed" : result == xco::TimedHistoryResult::no_candidate
                    ? "scheduled command missing or stale" : "scheduled command history invalid or retired",
                    timing_history_reason(result));
                return motor_reject(d, MOTOR_DRIVE_STALE, execution.invalid_reason.data());
            }
            timing.flags |= XNET_CO_TIMING_HAS_BOUNDARY | XNET_CO_TIMING_HAS_SOURCE
                | XNET_CO_TIMING_HAS_SOURCE_TIME | XNET_CO_TIMING_HAS_AVAILABLE_TIME;
            timing.boundary = selected.value.sync;
            timing.source = {selected.value.command.epoch, selected.value.command.command_seq,
                selected.value.command.action_seq};
            timing.source_us = selected.source_time;
            timing.available_us = selected.available_time;
            if (!d.test_sync_source_valid(selected.value.sync.timestamp_100ns, selected.source_time)) {
                d.invalidate_sync_execution("ordered SYNC source clock mapping changed", XNET_CO_TIMING_REASON_CLOCK);
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "ordered SYNC source clock mapping changed");
            }
            command = selected.value.command;
            if (command.action_seq != d.drive_action_seq
                || ((command.flags & 1U) && !d.drive_402->status().output_allowed))
                return motor_reject(d, MOTOR_DRIVE_STALE, "scheduled command was superseded by a safety transition");
            const auto& sync = selected.value.sync;
            cycle.sync_domain_id = plan.sync_domain_id;
            cycle.sync_domain_epoch = sync.epoch;
            cycle.sync_cycle_id = sync.sequence;
            cycle.sync_counter = sync.counter_present ? sync.counter : 0;
            cycle.rx_time = {plan.clock_domain_id, MOTOR_DRIVE_TIME_VALID, selected.source_time / 1000000.0};
            publish_time = {plan.clock_domain_id, MOTOR_DRIVE_TIME_VALID, selected.available_time / 1000000.0};
            processed_seq = selected.value.processed_feedback_seq;
            processed_result = selected.value.processed_result;
            scheduled_source = selected.source_time;
            scheduled_epsilon = plan.epsilon_us;
            scheduled_target = target;
            scheduled_cutoff = cutoff;
            scheduled_raw = sync.timestamp_100ns;
            const uint64_t age = plan.max_age_us - (target - cutoff) - plan.epsilon_us;
            scheduled_oldest = cutoff > age ? cutoff - age : 0;
        } else {
            if (!d.drive_command_seq) return static_cast<int32_t>(MOTOR_DRIVE_NO_DATA);
            const auto& record = d.drive_command_history[d.drive_command_seq % d.drive_command_history.size()];
            require(record.seq == d.drive_command_seq, XNET_CO_INTERNAL_ERROR, "missing command publication record");
            command = record.command;
            publish_time = {1, MOTOR_DRIVE_TIME_VALID,
                std::chrono::duration<double>(record.published.time_since_epoch()).count()};
            processed_seq = record.processed_feedback_seq;
            processed_result = record.processed_result;
        }
        const bool safety = command.action == CO_402_ACTION_QUICK_STOP || command.action == CO_402_ACTION_FAULT_REACTION
            || command.action == CO_402_ACTION_DISABLE_VOLTAGE;
        require(request->flags != MOTOR_DRIVE_READ_SAFETY || safety, XNET_CO_INVALID_STATE, "no priority safety command");
        require(request->flags != MOTOR_DRIVE_READ_BOOTSTRAP || !(command.flags & 1U),
            XNET_CO_INVALID_STATE, "bootstrap read cannot return an enabled motion command");
        const auto& adapter = *d.model->drive_adapter;
        motor_drive_view_v1 output{};
        output.abi_version = MOTOR_DRIVE_ABI_VERSION; output.struct_size = sizeof(output);
        output.token = {command.epoch, command.command_seq, command.action_seq};
        output.source_cycle = cycle;
        if (scheduled) output.target_step = request->target_step;
        output.publish_time = publish_time;
        output.action = command.action;
        output.flags = command.flags | MOTOR_DRIVE_SAFETY_DECELERATION_VALID;
        output.mode = command.mode;
        output.demand_position_rad = adapter.position_rad(command.demand_position);
        output.demand_velocity_rad_s = adapter.velocity_rad_s(command.demand_velocity);
        const auto deceleration = safety ? command.quick_stop_deceleration : command.profile_deceleration;
        output.safety_deceleration_rad_s2 = std::abs(adapter.velocity_rad_s(deceleration));
        output.processed_feedback_seq = processed_seq;
        output.processed_result = processed_result;
        if (scheduled) {
            if (p.test_issue_count == p.test_issues.size()) {
                d.invalidate_sync_execution("scheduled command assignment history overflow", XNET_CO_TIMING_REASON_HISTORY_OVERFLOW);
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "scheduled command assignment history overflow");
            }
            if (d.sync_execution->active_test_auto_retire
                && d.sync_execution->commands.retire_before(d.drive_epoch, scheduled_oldest)
                    != xco::TimedHistoryResult::ok) {
                d.invalidate_sync_execution("scheduled command history retirement failed", XNET_CO_TIMING_REASON_HISTORY_INVALID);
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "scheduled command history retirement failed");
            }
            p.test_issues[p.test_issue_count++] = {output, request->source_cutoff,
                scheduled_source, scheduled_epsilon, scheduled_target, scheduled_cutoff,
                d.sync_execution->active_test_command_plan->max_age_us, scheduled_raw};
            auto& record = d.drive_command_history[output.token.command_seq % d.drive_command_history.size()];
            if (record.seq == output.token.command_seq && record.command.epoch == output.token.epoch)
                record.test_scheduled = true;
            p.pcl_epoch = request->target_step.pcl_epoch;
            auto& timing = *diagnostic.get();
            timing.flags |= XNET_CO_TIMING_HAS_SOURCE_CYCLE;
            timing.source_cycle = output.source_cycle;
            d.record_timing_event(XNET_CO_TIMING_COMMAND_ASSIGNED);
        }
        *view = output;
        return static_cast<int32_t>(MOTOR_DRIVE_OK);
    });
}
int32_t motor_drive_submit_v1(motor_drive_port_handle port, const motor_drive_feedback_v1* feedback, uint32_t size) {
    return motor_call(port, [&](xnet_canopen_device& d, motor_drive_port& p) {
        require(feedback != nullptr, XNET_CO_INVALID_ARGUMENT, "motor feedback is required");
        require(size == sizeof(*feedback) && feedback->abi_version == MOTOR_DRIVE_ABI_VERSION
            && feedback->struct_size == sizeof(*feedback), XNET_CO_INVALID_ARGUMENT, "invalid motor feedback ABI/size");
        const auto& f = *feedback;
        require(f.feedback_seq && (f.flags & MOTOR_DRIVE_POSITION_VALID) && !(f.flags & ~0x17U)
            && f.action_result <= MOTOR_DRIVE_RESULT_ERROR && valid_step(f.executed_step)
            && std::isfinite(f.actual_position_rad)
            && std::isfinite(f.actual_velocity_rad_s), XNET_CO_ACCESS_ERROR, "invalid motor feedback fields");
        xco::TimingDiagnosticScope diagnostic(d.sync_execution && d.sync_execution->active_test_external_capture
            && !empty_cycle(f.source_cycle) ? d.sync_execution.get() : nullptr);
        if (auto* context = diagnostic.get()) timing_feedback(*context, f, nullptr);
        if (f.source.epoch != d.drive_epoch)
            return motor_reject(d, MOTOR_DRIVE_STALE, "motor feedback belongs to an old node epoch");
        if (p.submitted && f.feedback_seq == p.submitted->feedback_seq)
            return same_feedback(f, *p.submitted) ? static_cast<int32_t>(MOTOR_DRIVE_OK)
                : motor_reject(d, MOTOR_DRIVE_INVALID_FEEDBACK, "motor retry changed content");
        if (d.drive_feedback_pending) return motor_reject(d, MOTOR_DRIVE_BUSY, "motor feedback mailbox is occupied");
        if ((p.pcl_epoch && f.executed_step.pcl_epoch != p.pcl_epoch)
            || (p.submitted && (!sequence_newer(f.feedback_seq, p.submitted->feedback_seq)
                || !advancing_step(f.executed_step, p.submitted->executed_step))))
            return motor_reject(d, MOTOR_DRIVE_STALE, "motor feedback step/sequence did not advance; restart on PCL epoch change");
        std::optional<xco::TestScheduledIssue> timed_source;
        size_t timed_source_index = 0;
        if (!empty_cycle(f.source_cycle)) {
            if (!d.sync_execution || !d.sync_execution->active_test_external_capture || d.sync_execution->invalid)
                return motor_reject(d, MOTOR_DRIVE_INVALID_FEEDBACK, "scheduled feedback is not available");
            if (!d.validate_test_sync_watermark())
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "ordered SYNC watermark clock mapping changed");
            const auto& hold = d.sync_execution->active_test_hold_plan;
            for (size_t i = 0; i < p.test_issue_count; ++i) {
                const auto& issue = p.test_issues[i];
                if (hold) {
                    if (issue.view.target_step.model_step_seq <= f.executed_step.model_step_seq) {
                        timed_source = issue;
                        timed_source_index = i;
                    }
                } else if (f.source.epoch == issue.view.token.epoch && f.source.command_seq == issue.view.token.command_seq
                    && f.source.action_seq == issue.view.token.action_seq && same_cycle(f.source_cycle, issue.view.source_cycle)
                    && same_step(f.executed_step, issue.view.target_step)) {
                    timed_source = issue;
                    timed_source_index = i;
                    break;
                }
            }
            if (!timed_source)
                return motor_reject(d, MOTOR_DRIVE_INVALID_FEEDBACK, "scheduled feedback source or execution step was not issued");
            diagnostic.get()->flags |= XNET_CO_TIMING_HAS_TARGET;
            diagnostic.get()->target_step = timed_source->view.target_step;
            if (!d.test_sync_source_valid(timed_source->raw_source_timestamp, timed_source->source_us)) {
                d.invalidate_sync_execution("ordered SYNC source clock mapping changed", XNET_CO_TIMING_REASON_CLOCK);
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "ordered SYNC source clock mapping changed");
            }
            if (f.source.epoch != timed_source->view.token.epoch
                || f.source.command_seq != timed_source->view.token.command_seq
                || f.source.action_seq != timed_source->view.token.action_seq
                || !same_cycle(f.source_cycle, timed_source->view.source_cycle))
                return motor_reject(d, MOTOR_DRIVE_INVALID_FEEDBACK, "scheduled feedback did not use latest applicable command");
            uint64_t now = 0, target = 0;
            if (!d.sync_now(now)) {
                d.invalidate_sync_execution("scheduled feedback clock invalid", XNET_CO_TIMING_REASON_CLOCK);
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "scheduled feedback clock invalid");
            }
            if (!d.sync_execution->active_clock->map_model_us(f.executed_step.sample_time, target)) {
                d.invalidate_sync_execution("scheduled feedback step clock mapping unavailable", XNET_CO_TIMING_REASON_CLOCK);
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "scheduled feedback step clock mapping unavailable");
            }
            uint64_t issued_target = 0;
            if (!d.sync_execution->active_clock->map_model_us(timed_source->view.target_step.sample_time, issued_target)
                || issued_target != timed_source->target_us) {
                d.invalidate_sync_execution("scheduled feedback clock mapping changed", XNET_CO_TIMING_REASON_CLOCK);
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "scheduled feedback clock mapping changed");
            }
            if (hold) {
                if (!test_hold_step(*hold, timed_source->view.target_step, issued_target, f.executed_step, target)
                    || f.executed_step.model_step_seq - timed_source->view.target_step.model_step_seq > hold->max_hold_steps)
                    return motor_reject(d, MOTOR_DRIVE_INVALID_FEEDBACK, "scheduled feedback violates test hold plan");
                if (timed_source->epsilon_us > timed_source->max_age_us || target < timed_source->source_us
                    || target - timed_source->source_us > timed_source->max_age_us - timed_source->epsilon_us)
                    return motor_reject(d, MOTOR_DRIVE_STALE, "held command exceeded scheduled age budget");
            } else if (target != timed_source->target_us) {
                d.invalidate_sync_execution("scheduled feedback clock mapping changed", XNET_CO_TIMING_REASON_CLOCK);
                return motor_reject(d, MOTOR_DRIVE_INVALID_STATE, "scheduled feedback clock mapping changed");
            }
            if (now < target || now < timed_source->source_us)
                return motor_reject(d, MOTOR_DRIVE_STALE, "scheduled feedback step is in the future");
            if (timed_source->epsilon_us > d.model->drive_command_max_age_us
                || now - timed_source->source_us > d.model->drive_command_max_age_us - timed_source->epsilon_us)
                return motor_reject(d, MOTOR_DRIVE_STALE, "scheduled feedback source command is too old");
        } else if (d.sync_execution && d.sync_execution->active_test_external_capture) {
            const auto& record = d.drive_command_history[f.source.command_seq % d.drive_command_history.size()];
            if (record.seq == f.source.command_seq && record.test_scheduled)
                return motor_reject(d, MOTOR_DRIVE_INVALID_FEEDBACK, "scheduled feedback omitted source cycle");
            for (size_t i = 0; i < p.test_issue_count; ++i) {
                const auto& issued = p.test_issues[i].view.token;
                if (f.source.epoch == issued.epoch && f.source.command_seq == issued.command_seq
                    && f.source.action_seq == issued.action_seq)
                    return motor_reject(d, MOTOR_DRIVE_INVALID_FEEDBACK, "scheduled feedback omitted source cycle");
            }
        }
        const auto& record = d.drive_command_history[f.source.command_seq % d.drive_command_history.size()];
        if (!f.source.command_seq || f.source.action_seq != d.drive_action_seq
            || (!timed_source && (record.seq != f.source.command_seq || record.action_seq != f.source.action_seq
                || xco::Clock::now() - record.published > std::chrono::microseconds(d.model->drive_command_max_age_us))))
            return motor_reject(d, MOTOR_DRIVE_STALE, "motor feedback source command is obsolete");
        auto& adapter = *d.model->drive_adapter;
        CO_402_feedback_frame_t frame{};
        frame.epoch = f.source.epoch; frame.command_seq = f.source.command_seq; frame.action_seq = f.source.action_seq;
        frame.feedback_seq = f.feedback_seq; frame.action_result = static_cast<uint8_t>(f.action_result);
        frame.flags = static_cast<uint8_t>(f.flags | (f.limit_flags ? 0x08U : 0));
        frame.actual_position = adapter.position_counts(f.actual_position_rad);
        frame.actual_velocity = (f.flags & MOTOR_DRIVE_VELOCITY_VALID) ? adapter.velocity_counts(f.actual_velocity_rad_s) : 0;
        (void)adapter.raw(xco::DriveQuantity::actual_position, frame.actual_position);
        if ((f.flags & MOTOR_DRIVE_VELOCITY_VALID)
            && adapter.bindings[static_cast<size_t>(xco::DriveQuantity::actual_velocity)].value)
            (void)adapter.raw(xco::DriveQuantity::actual_velocity, frame.actual_velocity);
        try { frame.fault_code = adapter.fault_code(f.fault_class, f.fault_detail); }
        catch (const Error&) { d.drive_adapter_fault = true; throw; }
        std::array<uint8_t, CO_402_FEEDBACK_FRAME_SIZE> bytes{};
        require(CO_402_feedback_pack(&frame, bytes.data(), bytes.size()), XNET_CO_ACCESS_ERROR, "cannot encode motor feedback");
        const auto staged = d.stage_drive_feedback(frame, bytes.data());
        require(staged == ODR_OK, XNET_CO_ACCESS_ERROR, "motor feedback staging rejected");
        d.drive_pending_motor_feedback = f;
        d.drive_pending_timed_source = timed_source;
        if (timed_source) {
            size_t kept = 0;
            for (size_t i = 0; i < p.test_issue_count; ++i)
                if (i == timed_source_index || p.test_issues[i].view.target_step.model_step_seq > f.executed_step.model_step_seq)
                    p.test_issues[kept++] = p.test_issues[i];
            p.test_issue_count = kept;
        }
        p.submitted = f; p.pcl_epoch = f.executed_step.pcl_epoch;
        return static_cast<int32_t>(MOTOR_DRIVE_OK);
    });
}
int32_t motor_drive_heartbeat_v1(motor_drive_port_handle port, const motor_drive_token_v1* source,
                               const motor_drive_model_step_v1* progress, uint32_t heartbeat_seq) {
    return motor_call(port, [&](xnet_canopen_device& d, motor_drive_port& p) {
        require(source && progress && heartbeat_seq && valid_step(*progress),
            XNET_CO_INVALID_ARGUMENT, "valid motor heartbeat progress/source are required");
        if (source->epoch != d.drive_epoch)
            return motor_reject(d, MOTOR_DRIVE_STALE, "motor heartbeat belongs to an old node epoch");
        if (p.heartbeat_seq == heartbeat_seq)
            return same_step(*progress, p.heartbeat_step) ? static_cast<int32_t>(MOTOR_DRIVE_OK)
                : motor_reject(d, MOTOR_DRIVE_INVALID_ARGUMENT, "motor heartbeat retry changed progress");
        if ((p.pcl_epoch && progress->pcl_epoch != p.pcl_epoch)
            || (p.heartbeat_seq && (!sequence_newer(heartbeat_seq, p.heartbeat_seq)
                || !advancing_step(*progress, p.heartbeat_step))))
            return motor_reject(d, MOTOR_DRIVE_STALE, "motor heartbeat has no new model progress");
        d.drive_last_heartbeat = heartbeat_seq; d.drive_has_heartbeat = true;
        d.drive_heartbeat_changed = xco::Clock::now();
        p.heartbeat_seq = heartbeat_seq; p.heartbeat_step = *progress; p.pcl_epoch = progress->pcl_epoch;
        return static_cast<int32_t>(MOTOR_DRIVE_OK);
    });
}
int32_t motor_drive_close_v1(motor_drive_port_handle port) {
    if (!port) return MOTOR_DRIVE_INVALID_ARGUMENT;
    int32_t result = XNET_CO_OK;
    if (const auto device = port->endpoint->device.load()) {
        result = call(device, [&](xnet_canopen_device&, std::unique_lock<std::recursive_mutex>&) {
            port->endpoint->writer_open = false;
            return XNET_CO_OK;
        }, true, true);
    }
    if (result == XNET_CO_OK) delete port;
    return motor_result(result);
}

int32_t co_device_create(co_device_handle* device) {
    if (!device) return XNET_CO_INVALID_ARGUMENT;
    *device = nullptr;
    auto facade = std::unique_ptr<co_device>(new (std::nothrow) co_device);
    if (!facade) return XNET_CO_INTERNAL_ERROR;
    const auto result = xnet_canopen_create(&facade->backend);
    if (result == XNET_CO_OK) *device = facade.release();
    return result;
}
int32_t co_device_destroy(co_device_handle device) {
    if (!device) return XNET_CO_INVALID_ARGUMENT;
    const auto result = xnet_canopen_destroy(device->backend);
    if (result == XNET_CO_OK) delete device;
    return result;
}
int32_t co_device_initialize(co_device_handle device, const char* interface_name,
                             const char* model_path, const char* persistence_path) {
    return xnet_canopen_initialize(device ? device->backend : nullptr, interface_name, model_path, persistence_path);
}
int32_t co_device_start(co_device_handle device) {
    return xnet_canopen_start(device ? device->backend : nullptr);
}
int32_t co_device_stop(co_device_handle device) {
    if (!device) return XNET_CO_INVALID_ARGUMENT;
    const auto result = xnet_canopen_stop(device->backend);
    if (result != XNET_CO_OK) return result;
    std::lock_guard<std::recursive_mutex> lock(device->backend->mutex);
    return device->backend->worker_result;
}
int32_t co_device_uninitialize(co_device_handle device) {
    return xnet_canopen_uninitialize(device ? device->backend : nullptr);
}
int32_t co_device_get_status_v1(co_device_handle device, xnet_canopen_system_status_v1* status, uint32_t capacity) {
    return xnet_canopen_get_system_status_v1(device ? device->backend : nullptr, status, capacity);
}
int32_t co_device_get_capabilities_v1(co_device_handle device, xnet_canopen_capabilities_v1* caps, uint32_t capacity) {
    return xnet_canopen_get_capabilities_v1(device ? device->backend : nullptr, caps, capacity);
}
int32_t co_device_get_diagnostic_v1(co_device_handle device, xnet_canopen_diagnostic_v1* diagnostic, uint32_t capacity) {
    return xnet_canopen_get_diagnostic_v1(device ? device->backend : nullptr, diagnostic, capacity);
}
int32_t co_device_clear_diagnostic(co_device_handle device) {
    return xnet_canopen_clear_error(device ? device->backend : nullptr);
}
int32_t co_device_open_motor_port(co_device_handle device, motor_drive_port_handle* port) {
    return co_device_open_motor_port_from_xnet(device ? device->backend : nullptr, port);
}
int32_t co_device_open_motor_port_from_xnet(xnet_canopen_handle device, motor_drive_port_handle* port) {
    if (port) *port = nullptr;
    return call(device, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) -> int32_t {
        require(port != nullptr, XNET_CO_INVALID_ARGUMENT, "motor port output is required");
        const auto result = open_motor_port(d, d.drive_endpoint.get(), port);
        require(result == MOTOR_DRIVE_OK || result == MOTOR_DRIVE_BUSY, XNET_CO_INTERNAL_ERROR,
            "unexpected motor port open result");
        if (result == MOTOR_DRIVE_BUSY) return CO_DEVICE_BUSY;
        return XNET_CO_OK;
    }, false, true);
}
int32_t co_device_bind_batch(co_device_handle device, const uint16_t* indices, const uint8_t* subindices,
                             uint32_t count, uint32_t* types, uint32_t* sizes, uint32_t* buffer_size, uint32_t* batch_id) {
    return xnet_canopen_bind_batch(device ? device->backend : nullptr, indices, subindices, count,
        types, sizes, buffer_size, batch_id);
}
int32_t co_device_read_batch(co_device_handle device, uint32_t batch_id, uint8_t* buffer,
                             uint32_t buffer_size, int32_t* entry_status, uint32_t entry_count) {
    return xnet_canopen_read_batch(device ? device->backend : nullptr, batch_id, buffer, buffer_size, entry_status, entry_count);
}
int32_t co_device_write_batch(co_device_handle device, uint32_t batch_id, const uint8_t* buffer,
                              uint32_t buffer_size, int32_t* entry_status, uint32_t entry_count) {
    return xnet_canopen_write_batch(device ? device->backend : nullptr, batch_id, buffer, buffer_size, entry_status, entry_count);
}
int32_t co_device_read_od(co_device_handle device, uint16_t index, uint8_t subindex,
                          uint8_t* buffer, uint32_t buffer_size, uint32_t* value_size) {
    return xnet_canopen_read_od(device ? device->backend : nullptr, index, subindex, buffer, buffer_size, value_size);
}
int32_t co_device_write_od(co_device_handle device, uint16_t index, uint8_t subindex,
                           const uint8_t* buffer, uint32_t buffer_size) {
    return xnet_canopen_write_od(device ? device->backend : nullptr, index, subindex, buffer, buffer_size);
}
int32_t co_device_request_tpdo(co_device_handle device, uint16_t tpdo_number) {
    return xnet_canopen_request_tpdo(device ? device->backend : nullptr, tpdo_number);
}
int32_t co_device_set_fault(co_device_handle device, uint8_t error_bit, uint16_t error_code,
                            uint32_t info_code, uint8_t active) {
    return xnet_canopen_set_fault(device ? device->backend : nullptr, error_bit, error_code, info_code, active);
}
int32_t co_device_get_drive_unit_report(co_device_handle device, char* buffer, uint32_t capacity, uint32_t* required_size) {
    return xnet_canopen_get_drive_unit_report(device ? device->backend : nullptr, buffer, capacity, required_size);
}
int32_t co_device_read_sync_event(co_device_handle device, uint64_t epoch, uint64_t sequence,
                                  xnet_canopen_sync_event_t* event) {
    return xnet_canopen_read_sync_event(device ? device->backend : nullptr, epoch, sequence, event);
}
int32_t co_device_read_timing_event_v1(co_device_handle device, uint64_t sequence,
                                       xnet_canopen_timing_event_v1* event, uint32_t capacity) {
    return xnet_canopen_read_timing_event_v1(device ? device->backend : nullptr, sequence, event, capacity);
}
} // extern "C"
