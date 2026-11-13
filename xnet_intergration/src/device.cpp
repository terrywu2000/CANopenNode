/**
 * @file
 * Public ABI, device lifecycle and the single protocol-processing worker.
 * The lifecycle mutex serializes host calls; the recursive runtime mutex protects
 * OD/service state shared with the worker. Application/drive behavior remains in
 * a JSON-enabled DS402 core and ordinary application OD values.
 */
#include "internal.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <set>

using xco::Error;
using xco::require;

namespace {
constexpr uint8_t drive_fault_startup = 0x01;
constexpr uint8_t drive_fault_heartbeat = 0x02;
constexpr uint8_t drive_fault_feedback = 0x04;
constexpr uint8_t drive_fault_action = 0x08;
constexpr uint8_t drive_transport_error_bit = CO_EM_MANUFACTURER_START;
/**
 * C ABI gateway: serialize the operation and translate typed/standard exceptions
 * to operation results plus a per-handle diagnostic. The lifecycle lock stays
 * owned even when an operation unlocks the runtime mutex to join the worker.
 * This cannot make a fabricated handle or concurrent destroy safe.
 */
template<class Function> int32_t call(xnet_canopen_handle d, Function function, bool allow_shutdown = false) noexcept {
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
        d->fail(e.code, e.what());
        return e.code;
    } catch (const xco::Json::exception& e) {
        std::lock_guard<std::recursive_mutex> lock(d->mutex);
        d->fail(XNET_CO_MODEL_ERROR, e.what());
        return XNET_CO_MODEL_ERROR;
    } catch (const std::exception& e) {
        std::lock_guard<std::recursive_mutex> lock(d->mutex);
        d->fail(XNET_CO_INTERNAL_ERROR, e.what());
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
}

void xnet_canopen_device::fail(int32_t code, std::string_view message) noexcept {
    // Fixed-capacity diagnostics remain usable while reporting allocation errors.
    // Successful operations do not clear this snapshot; callers clear explicitly.
    const auto size = std::min(message.size(), error_message.size() - 1);
    const bool changed = error_code != code || std::strlen(error_message.data()) != size
        || std::memcmp(error_message.data(), message.data(), size) != 0;
    error_code = code;
    std::memcpy(error_message.data(), message.data(), size);
    error_message[size] = 0;
    // Repeated identical queue/driver errors must not flood the file each cycle.
    if (changed) {
        log(xco::LogLevel::error, "error code={}: {}", code, error_message.data());
    }
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
    outgoing.clear();
    // Extensions point into the old stack allocation; detach them before rebuilding services.
    for (auto& entry : model->entries) entry.extension = nullptr;
    drive_mode_od = {};
    drive_feedback_od = {};
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
            if (model->drive_402_external) {
                drive_feedback_od.device = this;
                auto* feedback_entry = OD_find(&model->od, 0x2f03);
                require(OD_getSub(feedback_entry, 0, &drive_feedback_od.original, false) == ODR_OK,
                    XNET_CO_STACK_ERROR, "cannot bind external DS402 feedback OD handler");
                drive_feedback_od.extension.object = &drive_feedback_od;
                drive_feedback_od.extension.read = read_drive_feedback;
                drive_feedback_od.extension.write = write_drive_feedback;
                require(OD_extension_init(feedback_entry, &drive_feedback_od.extension) == ODR_OK,
                    XNET_CO_STACK_ERROR, "cannot install external DS402 feedback OD handler");
            }
        }
    }
    log(xco::LogLevel::info, "CANopen services initialized");
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
    const auto result = binding.original.write(&original, buffer, count, transferred);
    stream->dataOffset = original.dataOffset;
    if (result == ODR_OK) {
        // Preserve accepted EMCY routing, including its disable bit, across a
        // stop/start. An explicit communication/node reset restores JSON defaults.
        if (binding.index == 0x1014) d.model->emcy_reconfigured = true;
        if (binding.index == 0x1005) d.model->sync_reconfigured = true;
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

ODR_t xnet_canopen_device::read_drive_mode(
    OD_stream_t* stream, void* buffer, OD_size_t count, OD_size_t* transferred) {
    auto& binding = *static_cast<DriveModeOD*>(stream->object);
    auto original = *stream;
    original.object = binding.original.stream.object;
    const auto result = binding.original.read(&original, buffer, count, transferred);
    stream->dataOffset = original.dataOffset;
    return result;
}

ODR_t xnet_canopen_device::read_drive_feedback(
    OD_stream_t* stream, void* buffer, OD_size_t count, OD_size_t* transferred) {
    auto& binding = *static_cast<DriveFeedbackOD*>(stream->object);
    auto original = *stream;
    original.object = binding.original.stream.object;
    const auto result = binding.original.read(&original, buffer, count, transferred);
    stream->dataOffset = original.dataOffset;
    return result;
}

ODR_t xnet_canopen_device::write_drive_feedback(
    OD_stream_t* stream, const void* buffer, OD_size_t count, OD_size_t* transferred) {
    auto& binding = *static_cast<DriveFeedbackOD*>(stream->object);
    auto& d = *binding.device;
    if (!buffer || !transferred || stream->subIndex != 0 || count != CO_402_FEEDBACK_FRAME_SIZE
        || count != stream->dataLength || stream->dataOffset != 0) return ODR_TYPE_MISMATCH;
    CO_402_feedback_frame_t frame{};
    const auto* bytes = static_cast<const uint8_t*>(buffer);
    if (!CO_402_feedback_unpack(&frame, bytes, count)) return ODR_INVALID_VALUE;
    if (d.drive_feedback_pending) {
        if (frame.feedback_seq == d.drive_pending_feedback.feedback_seq
            && std::memcmp(bytes, d.drive_pending_feedback_bytes.data(), count) == 0) {
            *transferred = count;
            stream->dataOffset += count;
            return ODR_OK;
        }
        return ODR_DATA_DEV_STATE;
    }
    if (d.drive_has_last_submitted && frame.feedback_seq == d.drive_last_submitted_seq
        && std::memcmp(bytes, d.drive_last_submitted_bytes.data(), count) == 0) {
        *transferred = count;
        stream->dataOffset += count;
        return ODR_OK;
    }
    if (d.drive_has_last_submitted && !sequence_newer(frame.feedback_seq, d.drive_last_submitted_seq))
        return ODR_DATA_DEV_STATE;
    d.drive_pending_feedback = frame;
    d.drive_pending_submitted_at = xco::Clock::now();
    d.drive_last_submitted_seq = frame.feedback_seq;
    d.drive_last_submitted_bytes = {};
    std::memcpy(d.drive_last_submitted_bytes.data(), bytes, count);
    d.drive_has_last_submitted = true;
    std::memcpy(d.drive_pending_feedback_bytes.data(), bytes, count);
    d.drive_feedback_pending = true;
    std::memcpy(d.model->value(0x2f03, 0).data(), bytes, count);
    *transferred = count;
    stream->dataOffset += count;
    return ODR_OK;
}

void xnet_canopen_device::start_drive_epoch() {
    if (!model || !model->drive_402_external) return;
    require(drive_epoch != UINT64_MAX, XNET_CO_STACK_ERROR, "DS402 external epoch exhausted");
    ++drive_epoch;
    drive_command_seq = 0;
    drive_action_seq = 1;
    drive_last_feedback_seq = 0;
    drive_last_submitted_seq = 0;
    drive_command_history = {};
    drive_feedback_pending = false;
    drive_has_last_feedback = false;
    drive_has_last_submitted = false;
    drive_pending_submitted_at = drive_feedback_accepted_at = {};
    drive_pending_feedback = {};
    drive_last_feedback = {};
    drive_pending_feedback_bytes = {};
    drive_last_feedback_bytes = {};
    drive_last_submitted_bytes = {};
    drive_has_heartbeat = false;
    drive_heartbeat_baseline = read_drive_value<uint32_t>(*model, 0x2f01);
    drive_last_heartbeat = drive_heartbeat_baseline;
    drive_heartbeat_changed = {};
    drive_started_at = xco::Clock::now();
    drive_startup_complete = false;
    drive_timed_action_seq = 0;
    drive_action_started = {};
    drive_action_timed_out = false;
    drive_transport_faults = drive_reported_transport_faults = 0;
    drive_latched_fault = 0;
    drive_current_fault = 0;
    drive_emcy_active = false;
    drive_previous_pds = xco::d402::Pds::switch_on_disabled;
    drive_has_previous_pds = false;
    drive_pp_ready = false;
    drive_reset_permitted = false;
    write_drive_value(*model, 0x2f00, uint16_t{0});
    write_drive_value(*model, 0x2f04, uint32_t{0});
    write_drive_value(*model, 0x2f05, uint16_t{0});
}

void xnet_canopen_device::process_drive_402(uint32_t dt_us, bool force_non_operational) {
    if (!model || !model->drive_402_enabled) return;
    require(drive_402 != nullptr, XNET_CO_INTERNAL_ERROR, "DS402 profile is enabled without a drive instance");
    const int8_t requested_mode = read_drive_value<int8_t>(*model, 0x6060);
    const bool unsupported_mode = !xco::d402::Drive::mode_supported(requested_mode);

    xco::d402::Inputs input;
    input.controlword = read_drive_value<uint16_t>(*model, 0x6040);
    if (unsupported_mode)
        input.controlword = static_cast<uint16_t>((input.controlword | 0x0002U) & ~0x0004U);
    input.mode_requested = requested_mode;
    input.nmt_operational = !force_non_operational && !stop_requested
        && CO_NMT_getInternalState(co->NMT) == CO_NMT_OPERATIONAL;
    input.target_position = read_drive_value<int32_t>(*model, 0x607a);
    input.profile_velocity = read_drive_value<uint32_t>(*model, 0x6081);
    input.profile_acceleration = read_drive_value<uint32_t>(*model, 0x6083);
    input.profile_deceleration = read_drive_value<uint32_t>(*model, 0x6084);
    input.quick_stop_deceleration = model->drive_has_quick_stop_deceleration
        ? read_drive_value<uint32_t>(*model, 0x6085) : input.profile_deceleration;
    input.position_tolerance = model->drive_position_tolerance;
    input.target_reached_time_us = model->drive_target_reached_time_us;
    uint16_t external_fault_code = 0;
    const auto now = xco::Clock::now();
    const auto previous_status = drive_402->status();
    if (model->drive_402_external) {
        const uint32_t heartbeat = read_drive_value<uint32_t>(*model, 0x2f01);
        if (heartbeat != 0 && ((!drive_has_heartbeat && heartbeat != drive_heartbeat_baseline)
            || (drive_has_heartbeat && sequence_newer(heartbeat, drive_last_heartbeat)))) {
            drive_last_heartbeat = heartbeat;
            drive_has_heartbeat = true;
            drive_heartbeat_changed = now;
        }
        const bool heartbeat_expired = drive_has_heartbeat
            && now - drive_heartbeat_changed >= std::chrono::microseconds(model->drive_heartbeat_timeout_us);
        const auto action_deadline = drive_action_started + std::chrono::microseconds(model->drive_action_timeout_us);
        bool action_expired = previous_status.action_pending && drive_timed_action_seq == previous_status.action_seq
            && now >= action_deadline;

        if (drive_feedback_pending) {
            const auto frame = drive_pending_feedback;
            const auto& record = drive_command_history[frame.command_seq % drive_command_history.size()];
            const bool identity_valid = frame.epoch == drive_epoch && frame.action_seq == drive_action_seq
                && record.seq == frame.command_seq && record.action_seq == frame.action_seq;
            const bool command_expired = identity_valid
                && now - record.published > std::chrono::microseconds(model->drive_command_max_age_us);
            const bool late_completion = previous_status.action_pending
                && (frame.action_result == CO_402_ACTION_RESULT_DONE || frame.action_result == CO_402_ACTION_RESULT_ERROR)
                && drive_timed_action_seq == frame.action_seq && drive_pending_submitted_at >= action_deadline;
            const bool command_valid = identity_valid && !command_expired && !late_completion;
            uint16_t result = command_valid ? 1 : command_expired ? 2 : 3;
            if (command_valid) {
                if (frame.action_result == CO_402_ACTION_RESULT_DONE && frame.action_seq == drive_timed_action_seq)
                    action_expired = false;
                const bool velocity_valid = (frame.flags & 0x02U) != 0;
                xco::d402::Feedback feedback{frame.actual_position,
                    velocity_valid ? frame.actual_velocity : drive_402->status().actual_velocity,
                    frame.fault_code != 0 || frame.action_result == CO_402_ACTION_RESULT_ERROR};
                drive_402->post_feedback(feedback);
                drive_pp_ready = (frame.flags & 0x04U) != 0;
                drive_reset_permitted = (frame.flags & 0x10U) != 0;
                drive_402->set_external_permissions(drive_pp_ready, drive_reset_permitted);
                drive_last_feedback = frame;
                drive_last_feedback_seq = frame.feedback_seq;
                drive_last_feedback_bytes = drive_pending_feedback_bytes;
                drive_has_last_feedback = true;
                drive_feedback_accepted_at = now;
                drive_current_fault = frame.fault_code
                    ? frame.fault_code
                    : frame.action_result == CO_402_ACTION_RESULT_ERROR ? CO_EMC_GENERIC : 0;
                input.action_response_seq = frame.action_seq;
                input.action_result = frame.action_result;
            }
            write_drive_value(*model, 0x2f04, frame.feedback_seq);
            write_drive_value(*model, 0x2f05, result);
            drive_feedback_pending = false;
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
        drive_402->set_external_fault(faults != 0);
        external_fault_code = drive_current_fault ? drive_current_fault : faults ? 0x8130 : 0;
        input.model_ready = drive_pp_ready;
        input.reset_permitted = drive_reset_permitted;
    }
    drive_402->step(input, dt_us);
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
        std::array<uint8_t, CO_402_COMMAND_FRAME_SIZE> command_bytes{};
        require(CO_402_command_pack(&command, command_bytes.data(), command_bytes.size()),
            XNET_CO_INTERNAL_ERROR, "cannot encode DS402 command frame");
        std::memcpy(model->value(0x2f02, 0).data(), command_bytes.data(), command_bytes.size());
        drive_command_history[command_seq % drive_command_history.size()] =
            DriveCommandRecord{command_seq, status.action_seq, xco::Clock::now()};
        const bool fault_active = status.model_fault || external_fault_code != 0;
        const uint16_t current_fault = external_fault_code;
        require(!status.model_fault || current_fault != 0, XNET_CO_STACK_ERROR, "DS402 model fault has no classified cause");
        write_drive_value(*model, 0x2f00, current_fault);
        const bool reset_completed = drive_has_previous_pds
            && drive_previous_pds == xco::d402::Pds::fault
            && status.pds == xco::d402::Pds::switch_on_disabled && !fault_active;
        if (fault_active) {
            if (drive_latched_fault == 0) drive_latched_fault = current_fault;
            if (drive_current_fault && !drive_emcy_active && co && co->em) {
                CO_errorReport(co->em, CO_EM_GENERIC_ERROR, drive_current_fault, static_cast<uint32_t>(drive_epoch));
                drive_emcy_active = true;
            }
            for (uint8_t source = 0; source < 4; ++source) {
                const uint8_t mask = static_cast<uint8_t>(1U << source);
                if ((drive_transport_faults & mask) && !(drive_reported_transport_faults & mask) && co && co->em) {
                    CO_errorReport(co->em, static_cast<uint8_t>(drive_transport_error_bit + source), 0x8130, mask);
                    drive_reported_transport_faults |= mask;
                }
            }
        } else if (reset_completed) {
            drive_latched_fault = 0;
            if (drive_emcy_active && co && co->em) {
                CO_errorReset(co->em, CO_EM_GENERIC_ERROR, static_cast<uint32_t>(drive_epoch));
                drive_emcy_active = false;
            }
            for (uint8_t source = 0; source < 4; ++source) {
                if ((drive_reported_transport_faults & (1U << source)) && co && co->em)
                    CO_errorReset(co->em, static_cast<uint8_t>(drive_transport_error_bit + source), 0);
            }
            drive_reported_transport_faults = 0;
        }
        if (model->drive_has_error_code) write_drive_value(*model, 0x603f, drive_latched_fault);
        drive_previous_pds = status.pds;
        drive_has_previous_pds = true;
    } else if (model->drive_has_error_code) {
        write_drive_value(*model, 0x603f, static_cast<uint16_t>(status.model_fault ? 0x1000 : 0));
    }
    write_drive_value(*model, 0x6041, status.statusword);
    write_drive_value(*model, 0x6061, status.mode_display);
    write_drive_value(*model, 0x6064, status.actual_position);
    if (unsupported_mode) {
        log(xco::LogLevel::warning, "unsupported 0x6060 value {} received through PDO; requesting quick stop and restoring PP",
            int(requested_mode));
        write_drive_value(*model, 0x6040, input.controlword);
        write_drive_value(*model, 0x6060, xco::d402::mode_profile_position);
    }
}

void xnet_canopen_device::drain_drive_402() {
    if (!drive_402) return;
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
        require(xco::Clock::now() < deadline, XNET_CO_STACK_ERROR,
            "DS402 did not reach a terminal state before the shutdown/reset deadline");
        worker_lock->unlock();
        std::this_thread::sleep_for(std::chrono::microseconds(model->worker_period_us));
        worker_lock->lock();
    }
}

void xnet_canopen_device::reset_stack(bool reset_all_objects) {
    log(xco::LogLevel::info, reset_all_objects ? "Reset Node requested" : "Reset Communication requested");
    drain_drive_402();
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
    start_drive_epoch();
    setup_stack();
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
                receive();
                // Receive dispatch can request a reset, stop, or bitrate switch in a callback.
                if (switching == Switch::none && !stop_requested) {
                    auto reset = CO_process(co, false, dt, nullptr);
                    if (switching == Switch::none && !stop_requested) {
                        if (reset != CO_RESET_NOT) reset_stack(reset == CO_RESET_APP);
                        else if (!co->nodeIdUnconfigured) {
                            // Apply RPDO commands before packing values for this cycle's TPDOs.
                            bool_t sync = CO_process_SYNC(co, dt, nullptr);
                            CO_process_RPDO(co, sync, dt, nullptr);
                            process_drive_402(dt);
                            CO_process_TPDO(co, sync, dt, nullptr);
                        }
                        if (!stop_requested) transmit();
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
        drain_drive_402();
    } catch (const Error& e) {
        fail(e.code, e.what()); state = XNET_CO_RUNTIME_FAILED; stop_requested = true;
    } catch (const std::exception& e) {
        fail(XNET_CO_INTERNAL_ERROR, e.what()); state = XNET_CO_RUNTIME_FAILED; stop_requested = true;
    }
    try { adapter.stop_flush(); }
    catch (const Error& e) { fail(e.code, e.what()); state = XNET_CO_RUNTIME_FAILED; }
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
    can_state = {};
    outgoing.clear();
    if (co || model) log(xco::LogLevel::info, "device uninitialized");
    if (co) { CO_delete(co); co = nullptr; }
    drive_402.reset();
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
        try {
            // A new stack restarts protocol state without resetting live OD
            // values. Launch only after sessions and CANnormal are ready.
            if (d.drive_402) d.drive_402->reset();
            d.start_drive_epoch();
            d.setup_stack();
            d.adapter.start();
            CO_CANsetNormalMode(d.co->CANmodule);
            d.stop_requested = false;
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
        if (!handle->worker.joinable()) {
            runtime.unlock();
            lifecycle.unlock();
            return call(handle, [](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>& lock) {
                lock.unlock(); d.stop(); return XNET_CO_OK;
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
        return XNET_CO_OK;
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
        if (d.shutdown_joining) {
            for (const auto& binding : batch.bindings) {
                const bool allowed = is_write
                    ? (binding.index == 0x2f01 || binding.index == 0x2f03)
                    : ((binding.index >= 0x2f00 && binding.index <= 0x2f05)
                        || binding.index == 0x6040 || binding.index == 0x6041
                        || binding.index == 0x6061 || binding.index == 0x603f);
                require(allowed, XNET_CO_INVALID_STATE,
                    "only DS402 external model exchange OD traffic is allowed during shutdown");
            }
        }
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
    }, true);
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
        require(!d.shutdown_joining || (index >= 0x2f00 && index <= 0x2f05)
                || index == 0x6040 || index == 0x6041 || index == 0x6061 || index == 0x603f,
            XNET_CO_INVALID_STATE, "only DS402 external model exchange reads are allowed during shutdown");
        require(value_size && (buffer || buffer_size == 0), XNET_CO_INVALID_ARGUMENT, "invalid OD read arguments");
        const auto& v = d.model->value(index, subindex);
        *value_size = v.size;
        // A size query must not replace the latest diagnostic.
        if (!buffer) return XNET_CO_BUFFER_TOO_SMALL;
        require(buffer_size >= v.size, XNET_CO_BUFFER_TOO_SMALL, "OD read buffer too small");
        const auto result = entry_io(d, index, subindex, v, buffer, nullptr, false);
        if (result < 0) d.fail(result, "OD " + std::to_string(index) + ":" + std::to_string(subindex) + " read failed");
        return result;
    }, true);
}
int32_t xnet_canopen_write_od(xnet_canopen_handle handle, uint16_t index, uint8_t subindex,
                             const uint8_t* buffer, uint32_t buffer_size) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) -> int32_t {
        require_od_access(d);
        require(!d.shutdown_joining || index == 0x2f01 || index == 0x2f03,
            XNET_CO_INVALID_STATE, "only DS402 heartbeat and feedback writes are allowed during shutdown");
        require(buffer != nullptr, XNET_CO_INVALID_ARGUMENT, "OD write buffer is required");
        const auto& v = d.model->value(index, subindex);
        require(buffer_size == v.size, XNET_CO_INVALID_ARGUMENT, "OD write size must equal the OD length");
        const auto result = entry_io(d, index, subindex, v, nullptr, buffer, true);
        if (result < 0) d.fail(result, "OD " + std::to_string(index) + ":" + std::to_string(subindex) + " write failed");
        return result;
    }, true);
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
int32_t xnet_canopen_get_status(xnet_canopen_handle handle, xnet_canopen_status_t* status) {
    return call(handle, [&](xnet_canopen_device& d, std::unique_lock<std::recursive_mutex>&) {
        require(status != nullptr, XNET_CO_INVALID_ARGUMENT, "status output is required");
        *status = {};
        status->runtime_state = d.state;
        // These survive stack teardown and remain readable after worker failure
        // or uninitialize. call() holds the same mutex used to complete resets.
        status->reset_node_count = d.reset_node_count;
        status->reset_communication_count = d.reset_communication_count;
        status->can_state_raw = d.can_state.raw;
        status->can_state_valid = d.can_state.valid ? 1U : 0U;
        if (d.co) {
            status->nmt_state = CO_NMT_getInternalState(d.co->NMT);
            status->can_error_status = d.co->CANmodule->CANerrorStatus;
            status->active_node_id = d.active_node; status->pending_node_id = d.pending_node;
            status->active_bitrate_kbps = d.active_bitrate; status->pending_bitrate_kbps = d.pending_bitrate;
            status->node_guarding_timeout = d.co->NGslave->lifeTimeTimeout ? 1U : 0U;
        }
        return XNET_CO_OK;
    });
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
} // extern "C"
