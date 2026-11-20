/**
 * @file
 * NI-XNET session adapter and CANopenNode's target-driver boundary.
 * CANopenNode queues immutable transmit copies; only the device worker performs
 * frame I/O. Received raw-stream records are translated into standard CAN/RTR
 * messages and dispatched through stack-owned software subscriptions.
 */
#include "internal.hpp"
#include "can_state.hpp"

#include <algorithm>
#include <cstring>

namespace {
struct TestRpdoWrite {
    OD_IO_t* io = nullptr;
    decltype(OD_IO_t::write) original = nullptr;
    void* object = nullptr;
    bool called = false;
    bool failed = false;
};

ODR_t test_rpdo_write(OD_stream_t* stream, const void* data, OD_size_t count, OD_size_t* written) {
    auto& tracked = *static_cast<TestRpdoWrite*>(stream->object);
    stream->object = tracked.object;
    *written = 0;
    const auto result = tracked.original(stream, data, count, written);
    stream->object = &tracked;
    tracked.called = true;
    tracked.failed = tracked.failed || result != ODR_OK || *written != count;
    return result;
}

class TestRpdoWrites {
public:
    TestRpdoWrites(CO_t& co, uint16_t count, uint32_t mask) {
        for (uint16_t pdo = 0; pdo < count; ++pdo) {
            if (!(mask & (1U << pdo))) continue;
            auto& common = co.RPDO[pdo].PDO_common;
            for (uint8_t i = 0; i < common.mappedObjectsCount; ++i) {
                auto& io = common.OD_IO[i];
                auto& tracked = entries_[count_++];
                tracked.io = &io;
                tracked.original = io.write;
                tracked.object = io.stream.object;
                io.stream.object = &tracked;
                io.write = test_rpdo_write;
            }
        }
    }

    ~TestRpdoWrites() {
        for (size_t i = 0; i < count_; ++i) {
            auto& tracked = entries_[i];
            tracked.io->stream.object = tracked.object;
            tracked.io->write = tracked.original;
        }
    }

    bool ok() const {
        for (size_t i = 0; i < count_; ++i)
            if (!entries_[i].called || entries_[i].failed) return false;
        return true;
    }

private:
    std::array<TestRpdoWrite, 8 * CO_PDO_MAX_MAPPED_ENTRIES> entries_{};
    size_t count_ = 0;
};

bool test_rpdo_applied(const CO_RPDO_t& rpdo) {
    const uint8_t buffer = rpdo.SYNC && !rpdo.SYNC->CANrxToggle ? 1 : 0;
    size_t offset = 0;
    for (uint8_t i = 0; i < rpdo.PDO_common.mappedObjectsCount; ++i) {
        const auto& io = rpdo.PDO_common.OD_IO[i];
        auto stream = io.stream;
        const size_t size = stream.dataOffset;
        if (!io.read || !size || size != stream.dataLength || size > CO_PDO_MAX_SIZE - offset) return false;
        std::array<uint8_t, CO_PDO_MAX_SIZE> value{};
        OD_size_t read = 0;
        stream.dataOffset = 0;
        if (io.read(&stream, value.data(), static_cast<OD_size_t>(size), &read) != ODR_OK || read != size)
            return false;
#ifdef CO_BIG_ENDIAN
        if (stream.attribute & ODA_MB) std::reverse(value.begin(), value.begin() + size);
#endif
        if (std::memcmp(value.data(), rpdo.CANrxData[buffer] + offset, size) != 0) return false;
        offset += size;
    }
    return offset == rpdo.PDO_common.dataLength;
}
} // namespace

namespace xco {
void Adapter::check(nxStatus_t status, const char* operation) {
    // Treat warnings as observable failures too; do not assume partially
    // successful NI calls have left a usable session/frame transaction.
    if (status == nxSuccess) return;
    char message[2048]{};
    nxStatusToString(status, sizeof(message), message);
    throw Error(XNET_CO_XNET_ERROR, std::string(operation) + " (" + std::to_string(status) + "): " + message);
}

/**
 * Open raw frame streams on the same physical interface, without a CAN database.
 * Leave input auto-start enabled as required by XNET; no reads occur until start.
 * Disable output auto-start so initialization does not start bus communication.
 * Partial session acquisition is retained in this object for release() to clean
 * up or retry; this method does not hide an error by discarding a live reference.
 */
void Adapter::open(const std::string& name, uint16_t kbps, std::optional<bool> termination) {
    check(nxCreateSession(":memory:", "", "", name.c_str(), nxMode_FrameInStream, &input), "create input");
    check(nxCreateSession(":memory:", "", "", name.c_str(), nxMode_FrameOutStream, &output), "create output");
    // XNET boolean properties use a one-byte u8, not the stack's uint_fast8_t.
    u8 off = 0;
    check(nxSetProperty(output, nxPropSession_AutoStart, sizeof(off), &off), "disable output auto-start");
    check(nxSetProperty(input, nxPropSession_IntfEchoTx, sizeof(off), &off), "disable transmit echo");
    u32 timing = nxOutStrmTimng_Immediate;
    check(nxSetProperty(output, nxPropSession_IntfOutStrmTimng, sizeof(timing), &timing), "immediate output timing");
    if (termination.has_value()) {
        u32 value = *termination ? nxCANTerm_On : nxCANTerm_Off;
        check(nxSetProperty(input, nxPropSession_IntfCANTerm, sizeof(value), &value), "set CAN termination");
    }
    // Use NI-XNET's default queue sizes. Uncomment to override with 1024 classic CAN frames.
    // Frame-stream queue sizes are bytes, including each frame's header and padding.
    // u32 queue_size = 1024 * sizeof(nxFrameCAN_t);
    // check(nxSetProperty(input, nxPropSession_QueueSize, sizeof(queue_size), &queue_size), "input queue size");
    // check(nxSetProperty(output, nxPropSession_QueueSize, sizeof(queue_size), &queue_size), "output queue size");
    bitrate(kbps);
}

void Adapter::bitrate(uint16_t kbps) {
    // Bitrate is interface-wide although XNET exposes the property via a session.
    // Use NI's 64-bit bits/second property, not the public API's kbit/s unit.
    u64 rate = uint64_t(kbps) * 1000;
    check(nxSetProperty(input, nxPropSession_IntfBaudRate64, sizeof(rate), &rate), "set bitrate");
}

void Adapter::start() {
    check(nxStart(input, nxStartStop_Normal), "start input");
    check(nxStart(output, nxStartStop_Normal), "start output");
    sessions_started = true;
}

void Adapter::stop_flush() {
    nxStatus_t first = 0;
    auto retain = [&](nxStatus_t status) { if (first == 0 && status != 0) first = status; };
    // Stop preserves XNET queues. Attempt every cleanup step, then report the first failure.
    if (input) retain(nxStop(input, nxStartStop_Normal));
    if (output) retain(nxStop(output, nxStartStop_Normal));
    if (input) retain(nxFlush(input));
    if (output) retain(nxFlush(output));
    sessions_started = false;
    check(first, "stop/flush sessions");
}

void Adapter::close() {
    nxStatus_t first = 0;
    if (output) {
        auto status = nxClear(output);
        // Keep failed references so a later uninitialize can retry their release.
        if (status == 0) output = 0;
        else first = status;
    }
    if (input) {
        auto status = nxClear(input);
        if (status == 0) input = 0;
        else if (first == 0) first = status;
    }
    sessions_started = false;
    check(first, "clear sessions");
}
} // namespace xco

extern "C" {
/**
 * Lock the device's recursive OD/runtime mutex for CANopenNode's CO_LOCK_* hooks.
 * context is CO_CANmodule_t::CANptr, set to the owning device by CO_CANmodule_init().
 * Recursion permits stack calls to re-enter the driver while the worker holds it.
 */
void xnet_co_lock(void* context) {
    if (context) static_cast<xnet_canopen_device*>(context)->mutex.lock();
}
/** Release the mutex acquired by the corresponding CO_LOCK_* hook. */
void xnet_co_unlock(void* context) {
    if (context) static_cast<xnet_canopen_device*>(context)->mutex.unlock();
}
/**
 * Configuration-mode hook required by CANopenNode; no hardware action here.
 * The device lifecycle explicitly stops/flushes XNET sessions before stack resets.
 */
void CO_CANsetConfigurationMode(void*) {}
/** Mark the stack ready for normal CAN operation; XNET sessions are started separately. */
void CO_CANsetNormalMode(CO_CANmodule_t* module) { if (module) module->CANnormal = true; }

/**
 * Attach a newly allocated stack CAN module to its device and initialize its buffers.
 * context is the owning xnet_canopen_device; rx/tx are stack-owned arrays with
 * nr/nt slots. The driver borrows these arrays for the lifetime of this stack.
 * bitrate is in kbit/s and is validated here, but XNET session setup applies it.
 * Clear subscriptions, transmit state, and error flags; remain outside normal mode
 * until CO_CANsetNormalMode(). No session creation or frame I/O occurs here.
 * Returns CO_ERROR_ILLEGAL_ARGUMENT for invalid inputs, otherwise CO_ERROR_NO.
 */
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t* m, void* context, CO_CANrx_t rx[], uint16_t nr,
                                  CO_CANtx_t tx[], uint16_t nt, uint16_t bitrate) {
    if (!m || !context || !rx || !tx || !xco::valid_bitrate(bitrate)) return CO_ERROR_ILLEGAL_ARGUMENT;
    m->CANptr = context; m->rxArray = rx; m->rxSize = nr; m->txArray = tx; m->txSize = nt;
    m->CANerrorStatus = 0; m->CANnormal = false;
    std::memset(rx, 0, nr * sizeof(*rx)); std::memset(tx, 0, nt * sizeof(*tx));
    return CO_ERROR_NO;
}
/**
 * Mark the module inactive when CANopenNode disables/deletes it.
 * Session shutdown belongs to the device lifecycle, not to the stack allocation:
 * the same XNET sessions can be retained while rebuilding CANopenNode services.
 */
void CO_CANmodule_disable(CO_CANmodule_t* m) { if (m) m->CANnormal = false; }

/**
 * Install one software receive filter and its protocol callback in a stack slot.
 * ident is the expected standard CAN ID; mask selects which ID bits must match.
 * A mask bit of 1 means "compare this bit"; 0 means "accept either value".
 * rtr selects remote frames (true) or data frames (false); it is always compared.
 * object is passed unchanged as the callback's first argument when receive()
 * dispatches a matching frame. These are software filters, not XNET filters.
 */
CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t* m, uint16_t index, uint16_t ident, uint16_t mask,
                                   bool_t rtr, void* object, void (*callback)(void*, void*)) {
    if (!m || index >= m->rxSize || !object || !callback) return CO_ERROR_ILLEGAL_ARGUMENT;
    // Internal packed identifier: bits 0..10 = standard CAN ID (0x7ff),
    // bit 11 = remote-transmission-request flag (0x800), not an extra CAN ID bit.
    // Strip higher caller bits, then encode the expected RTR value in ident.
    // Force bit 11 into the mask so data and remote frames never match each other,
    // even when the caller's ID mask is 0 (accept every ID of the requested type).
    m->rxArray[index] = {static_cast<uint16_t>((ident & 0x7ff) | (rtr ? 0x800 : 0)),
        static_cast<uint16_t>((mask & 0x7ff) | 0x800), object, callback};
    return CO_ERROR_NO;
}
/**
 * Configure and return one stack-owned transmit slot, or nullptr for invalid input.
 * ident is an 11-bit CAN ID, length is 0..8 bytes, and rtr selects a remote frame.
 * sync marks synchronous PDO traffic that may need cancellation outside its
 * allowed SYNC window. The caller fills data before calling CO_CANsend().
 */
CO_CANtx_t* CO_CANtxBufferInit(CO_CANmodule_t* m, uint16_t index, uint16_t ident, bool_t rtr, uint8_t length, bool_t sync) {
    if (!m || index >= m->txSize || ident > 0x7ff || length > 8) return nullptr;
    auto& b = m->txArray[index];
    // Use the same packed ID/RTR representation as receive filters; XNET expects
    // the ID and frame type separately, so transmit() splits them again.
    b.ident = ident | (rtr ? 0x800U : 0U); b.DLC = length; b.syncFlag = sync; b.bufferFull = false;
    return &b;
}
/**
 * Queue an immutable frame copy for the worker; success means queued, not sent.
 * bufferFull reserves the originating stack slot until transmission/cancellation.
 * A second send on an occupied slot reports overflow rather than replacing data.
 * The queue borrows the slot pointer, so it must be drained/discarded before the
 * stack is freed. Actual nonblocking XNET writes are performed by transmit().
 */
CO_ReturnError_t CO_CANsend(CO_CANmodule_t* m, CO_CANtx_t* b) {
    if (!m || !m->CANptr || !b) return CO_ERROR_ILLEGAL_ARGUMENT;
    auto& d = *static_cast<xnet_canopen_device*>(m->CANptr);
    std::lock_guard<std::recursive_mutex> lock(d.mutex);
    if (b->bufferFull) {
        m->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        d.fail(XNET_CO_STACK_ERROR, "CANopen transmit buffer overflow");
        return CO_ERROR_TX_OVERFLOW;
    }
    try {
        xnet_canopen_device::Pending pending{};
        pending.buffer = b;
        pending.frame.ident = static_cast<uint16_t>(b->ident);
        pending.frame.DLC = b->DLC;
        std::memcpy(pending.frame.data, b->data, 8);
        // One immutable pending copy per stack buffer bounds the queue and avoids later mutation.
        d.outgoing.push_back(pending);
        b->bufferFull = true;
        return CO_ERROR_NO;
    } catch (const std::exception& e) {
        d.fail(XNET_CO_INTERNAL_ERROR, e.what());
        return CO_ERROR_OUT_OF_MEMORY;
    }
}
/**
 * Cancel queued synchronous PDOs when CANopenNode closes the valid SYNC window.
 * Leave asynchronous traffic intact. Frames already accepted by XNET cannot be
 * recalled here; only copies still in the driver's outgoing queue are removed.
 */
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t* m) {
    auto& d = *static_cast<xnet_canopen_device*>(m->CANptr);
    std::lock_guard<std::recursive_mutex> lock(d.mutex);
    // Only unsent adapter copies are cancellable; flushing XNET would discard unrelated traffic.
    for (auto it = d.outgoing.begin(); it != d.outgoing.end();) {
        if (it->buffer->syncFlag) {
            it->buffer->bufferFull = false;
            it = d.outgoing.erase(it);
            m->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
            d.fail(XNET_CO_STACK_ERROR, "late synchronous PDO removed from adapter queue");
        } else ++it;
    }
}
/**
 * Poll XNET controller state for CANopenNode's periodic error processing.
 * Map TX/RX error counters to warning/passive flags and detect bus-off. Queue-loss
 * flags remain latched until module reset; physical-state flags are recomputed
 * so recovery can clear them. An XNET query failure marks the device FAILED and
 * requests worker shutdown instead of propagating an exception into the C stack.
 */
void CO_CANmodule_process(CO_CANmodule_t* m) {
    auto& d = *static_cast<xnet_canopen_device*>(m->CANptr);
    u32 state = 0; nxStatus_t fault = 0;
    try {
        d.adapter.check(nxReadState(d.adapter.input, nxState_CANComm, sizeof(state), &state, &fault), "read CAN state");
        d.adapter.check(fault, "CAN state fault");
        // AND keeps only these historical loss bits and clears old warning,
        // passive, and bus-off bits. Each following OR adds a currently active
        // condition without erasing the other flags (these are bit flags, not codes).
        uint16_t flags = m->CANerrorStatus & (CO_CAN_ERRTX_OVERFLOW | CO_CAN_ERRRX_OVERFLOW | CO_CAN_ERRTX_PDO_LATE);
        auto tx = nxCANComm_Get_TxErrCount(state), rx = nxCANComm_Get_RxErrCount(state);
        if (tx >= 96) flags |= CO_CAN_ERRTX_WARNING;
        if (rx >= 96) flags |= CO_CAN_ERRRX_WARNING;
        if (tx >= 128) flags |= CO_CAN_ERRTX_PASSIVE;
        if (rx >= 128) flags |= CO_CAN_ERRRX_PASSIVE;
        if (nxCANComm_Get_CommState(state) == nxCANCommState_BusOff) flags |= CO_CAN_ERRTX_BUS_OFF;
        d.record_can_state(state, flags);
        if (flags != m->CANerrorStatus && flags != 0)
            d.fail(XNET_CO_XNET_ERROR, "CAN communication error flags: " + std::to_string(flags));
        m->CANerrorStatus = flags;
    } catch (const xco::Error& e) {
        d.fail(e.code, e.what());
        d.state = XNET_CO_RUNTIME_FAILED;
        d.stop_requested = true;
    }
}
} // extern "C"

void xnet_canopen_device::record_can_state(uint32_t raw, uint16_t flags) noexcept {
    const auto previous = can_state.raw;
    const bool first = !can_state.valid;
    const bool significant = first || ((raw ^ previous) & 0xFFFFU) != 0 || flags != can_state.logged_flags;
    const bool changed = first || raw != previous;
    can_state.raw = raw;
    can_state.valid = true;
    const auto now = xco::Clock::now();
    if (!significant && now - can_state.last_log < std::chrono::seconds(1)) {
        if (changed) ++can_state.suppressed_changes;
        return;
    }
    if (!significant && !changed && can_state.suppressed_changes == 0) return;

    const auto unhealthy = [](uint32_t state) {
        const auto communication = nxCANComm_Get_CommState(state);
        return (communication != nxCANCommState_ErrorActive && communication != nxCANCommState_Init)
            || nxCANComm_Get_TxErrCount(state) >= 96 || nxCANComm_Get_RxErrCount(state) >= 96
            || nxCANComm_Get_TcvrErr(state) != 0;
    };
    const bool recovered = !first && unhealthy(previous) && !unhealthy(raw)
        && nxCANComm_Get_CommState(raw) == nxCANCommState_ErrorActive && !nxCANComm_Get_Sleep(raw);
    log(unhealthy(raw) ? xco::LogLevel::warning : xco::LogLevel::info,
        "CAN controller {}: raw=0x{:08x} comm={}({}) tx_errors={} rx_errors={} "
        "last_error={}({}) transceiver_error={} sleep={} canopen_flags=0x{:04x} counter_changes_suppressed={}",
        first ? "sample" : recovered ? "recovered" : "changed",
        raw, xco::can_communication_name(raw), unsigned(nxCANComm_Get_CommState(raw)),
        unsigned(nxCANComm_Get_TxErrCount(raw)), unsigned(nxCANComm_Get_RxErrCount(raw)),
        xco::can_last_error_name(raw), unsigned(nxCANComm_Get_LastErr(raw)),
        unsigned(nxCANComm_Get_TcvrErr(raw)), unsigned(nxCANComm_Get_Sleep(raw)), flags,
        can_state.suppressed_changes);
    can_state.logged_flags = flags;
    can_state.last_log = now;
    can_state.suppressed_changes = 0;
}

/**
 * Submit pending snapshots in FIFO order while holding the runtime mutex.
 * XNET queue-full is backpressure: retain the front frame and its busy stack slot
 * for a later iteration. Other NI failures propagate to the worker's failure
 * handler; accepting a frame here is not an on-wire delivery acknowledgment.
 */
void xnet_canopen_device::transmit() {
    while (!outgoing.empty()) {
        xco::TimingDiagnosticScope diagnostic(sync_execution && outgoing.front().buffer->syncFlag
            ? sync_execution.get() : nullptr);
        if (auto* context = diagnostic.get()) {
            context->flags |= XNET_CO_TIMING_HAS_FRAME;
            context->frame_can_id = outgoing.front().frame.ident & 0x7ff;
            if (sync_execution->last_sync_event) {
                context->flags |= XNET_CO_TIMING_HAS_BOUNDARY;
                context->boundary = *sync_execution->last_sync_event;
            }
        }
        if (sync_execution && outgoing.front().buffer->syncFlag) {
            uint64_t now = 0;
            const auto& execution = *sync_execution;
            const bool valid_clock = !execution.invalid && execution.last_sync_us && sync_now(now)
                && now >= *execution.last_sync_us;
            if (execution.invalid || !valid_clock || now - *execution.last_sync_us >= execution.active_tpdo_window_us) {
                if (execution.invalid) CO_CANclearPendingSyncPDOs(co->CANmodule);
                else invalidate_sync_execution("synchronous TPDO expired before submission",
                    valid_clock ? XNET_CO_TIMING_REASON_DEADLINE : XNET_CO_TIMING_REASON_CLOCK);
                continue;
            }
        }
        const auto& pending = outgoing.front();
        const bool timed_sync = sync_execution && pending.buffer->syncFlag;
        nxFrameCAN_t frame{};
        // Remove our internal RTR bit from the on-wire ID; translate it into
        // XNET's separate frame type instead. Only the low 11 ID bits are sent.
        frame.Identifier = pending.frame.ident & 0x7ff;
        frame.Type = (pending.frame.ident & 0x800) ? nxFrameType_CAN_Remote : nxFrameType_CAN_Data;
        frame.PayloadLength = pending.frame.DLC;
        std::memcpy(frame.Payload, pending.frame.data, 8);
        // Submit one frame per call so failure cannot ambiguously accept part of a multi-frame write.
        auto status = nxWriteFrame(adapter.output, &frame, sizeof(frame), nxTimeout_None);
        if (status == nxErrOutputQueueOverflow) {
            co->CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
            fail(XNET_CO_XNET_ERROR, "XNET output queue full; frame remains in bounded adapter queue");
            return;
        }
        if (status != nxSuccess && sync_execution)
            invalidate_sync_execution("XNET write failed during SYNC execution", XNET_CO_TIMING_REASON_TRANSPORT);
        adapter.check(status, "write CAN frame");
        pending.buffer->bufferFull = false;
        outgoing.pop_front();
        if (timed_sync) {
            diagnostic.get()->flags |= XNET_CO_TIMING_XNET_ACCEPTED;
            record_timing_event(XNET_CO_TIMING_TPDO_ACCEPTED);
            uint64_t now = 0;
            const auto& execution = *sync_execution;
            if (!execution.last_sync_us || !sync_now(now) || now < *execution.last_sync_us) {
                co->CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
                invalidate_sync_execution("synchronous TPDO accepted but clock invalid; frame cannot be recalled",
                    XNET_CO_TIMING_REASON_CLOCK);
            } else if (now - *execution.last_sync_us >= execution.active_tpdo_window_us) {
                co->CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
                invalidate_sync_execution("synchronous TPDO accepted after deadline; frame cannot be recalled",
                    XNET_CO_TIMING_REASON_DEADLINE);
            }
        }
    }
}

/**
 * Process at most one nonblocking 8 KiB stream read per worker iteration.
 * A boundary handler consumes each received SYNC and its RPDOs before the next
 * frame; the caller owns timer advancement and TPDO deadlines. The default worker
 * does not install a handler. On input overflow, flush and expose the loss.
 */
void xnet_canopen_device::receive(SyncBoundaryHandler boundary, void* context) {
    std::array<uint64_t, 1024> storage{};
    u32 bytes = 0;
    auto status = nxReadFrame(adapter.input, storage.data(), static_cast<u32>(sizeof(storage)), nxTimeout_None, &bytes);
    if (status == nxErrInputQueueOverflow) {
        co->CANmodule->CANerrorStatus |= CO_CAN_ERRRX_OVERFLOW;
        fail(XNET_CO_XNET_ERROR, "XNET input queue overflow; unread traffic lost");
        start_sync_observation_epoch();
        invalidate_sync_execution("XNET input queue overflow invalidated SYNC timing", XNET_CO_TIMING_REASON_TRANSPORT);
        adapter.check(nxFlush(adapter.input), "flush overflowed input");
        return;
    }
    adapter.check(status, "read CAN frames");
    xco::require(bytes <= sizeof(storage), XNET_CO_XNET_ERROR, "XNET returned an invalid byte count");
    size_t offset = 0;
    const auto* raw = reinterpret_cast<const uint8_t*>(storage.data());
    while (offset < bytes) {
        // XNET stream records have variable padded lengths, even when unsupported frame types arrive.
        xco::require(bytes - offset >= nxSizeofFrameHeader, XNET_CO_XNET_ERROR, "truncated XNET frame header");
        nxFrameCAN_t frame{};
        std::memcpy(&frame, raw + offset, nxSizeofFrameHeader);
        const auto size = static_cast<size_t>(nxFrameSize(nxFrameGetPayloadLength(&frame)));
        xco::require(size <= bytes - offset, XNET_CO_XNET_ERROR, "truncated XNET frame payload");
        if ((frame.Type == nxFrameType_CAN_Data || frame.Type == nxFrameType_CAN20_Data || frame.Type == nxFrameType_CAN_Remote)
            && frame.Identifier <= 0x7ff && frame.PayloadLength <= 8 && !(frame.Flags & nxFrameFlags_TransmitEcho)) {
            xco::TimingDiagnosticScope diagnostic(boundary ? sync_execution.get() : nullptr);
            if (auto* timing = diagnostic.get()) {
                timing->flags |= XNET_CO_TIMING_HAS_FRAME | XNET_CO_TIMING_HAS_FRAME_TIMESTAMP;
                timing->frame_can_id = frame.Identifier;
                timing->frame_timestamp_100ns = frame.Timestamp;
            }
            CO_CANrxMsg_t message{};
            // Pack XNET's separate ID/type into the representation registered by
            // CO_CANrxBufferInit(): low 11 bits are the ID, bit 11 marks RTR.
            message.ident = static_cast<uint16_t>(frame.Identifier | (frame.Type == nxFrameType_CAN_Remote ? 0x800 : 0));
            message.DLC = frame.PayloadLength;
            // Some stack callbacks copy all eight bytes; keep bytes beyond the actual payload zeroed.
            if (frame.Type != nxFrameType_CAN_Remote) std::memcpy(message.data, raw + offset + nxSizeofFrameHeader, message.DLC);
            for (uint16_t i = 0; i < co->CANmodule->rxSize; ++i) {
                auto& subscription = co->CANmodule->rxArray[i];
                // XOR marks differing ID/RTR bits; AND discards differences in
                // "don't care" positions. Zero means every selected bit matches.
                // Example: ident=0x180, mask=0x780 accepts IDs 0x180..0x1ff;
                // the additional 0x800 mask bit still enforces the requested RTR.
                if (subscription.CANrx_callback && ((message.ident ^ subscription.ident) & subscription.mask) == 0) {
                    const bool sync = co->SYNC && subscription.object == co->SYNC;
                    const bool old_toggle = sync && co->SYNC->CANrxToggle;
                    if (boundary && sync_execution && sync_execution->active_test_rpdo_plan
                        && !sync_execution->invalid && CO_NMT_getInternalState(co->NMT) == CO_NMT_OPERATIONAL) {
                        auto& execution = *sync_execution;
                        const auto& plan = *execution.active_test_rpdo_plan;
                        for (uint16_t pdo = 0; pdo < model->rpdo_count; ++pdo) {
                            const uint32_t bit = 1U << pdo;
                            auto& rpdo = co->RPDO[pdo];
                            if (!(plan.required_mask & bit) || subscription.object != &rpdo) continue;
                            uint64_t source = 0, now = 0;
                            const uint64_t guard = plan.epsilon_us * 2;
                            if (!rpdo.PDO_common.valid || !rpdo.synchronous
                                || message.DLC != rpdo.PDO_common.dataLength) {
                                invalidate_sync_execution("required RPDO invalid or wrong length", XNET_CO_TIMING_REASON_RPDO);
                            } else if (execution.test_rpdo_seen & bit) {
                                invalidate_sync_execution("duplicate required RPDO before SYNC", XNET_CO_TIMING_REASON_RPDO);
                            } else if (!execution.active_clock->map_rx_us(frame.Timestamp, source) || !sync_now(now)
                                || !execution.started_us || source < *execution.started_us || source > now
                                || (execution.last_sync_us && (source <= *execution.last_sync_us
                                    || source - *execution.last_sync_us <= guard))
                                || (execution.test_rpdo_last_us && (source <= *execution.test_rpdo_last_us
                                    || source - *execution.test_rpdo_last_us <= guard))) {
                                invalidate_sync_execution("required RPDO timestamp invalid or unordered", XNET_CO_TIMING_REASON_RPDO);
                            } else {
                                execution.test_rpdo_seen |= bit;
                                execution.test_rpdo_last_us = source;
                            }
                            break;
                        }
                    }
                    subscription.CANrx_callback(subscription.object, &message);
                    if (sync && old_toggle != bool(co->SYNC->CANrxToggle)) {
                        xco::require(sync_observation_sequence != UINT64_MAX, XNET_CO_STACK_ERROR,
                            "SYNC observation sequence exhausted");
                        const auto seq = ++sync_observation_sequence;
                        auto& event = sync_observations[(seq - 1) % sync_observations.size()];
                        event = {sync_observation_epoch, seq, frame.Timestamp, frame.Identifier,
                            co->SYNC->counterOverflowValue ? message.data[0] : 0U,
                            co->SYNC->counterOverflowValue ? 1U : 0U, 0};
                        if (auto* timing = diagnostic.get()) {
                            timing->flags |= XNET_CO_TIMING_HAS_BOUNDARY;
                            timing->boundary = event;
                        }
                        if (boundary && !co->nodeIdUnconfigured) {
                            const bool_t sync = CO_process_SYNC(co, 0, nullptr);
                            if (sync && sync_execution && sync_execution->active_test_rpdo_plan
                                && !sync_execution->invalid) {
                                auto& execution = *sync_execution;
                                const auto& plan = *execution.active_test_rpdo_plan;
                                const uint64_t guard = plan.epsilon_us * 2;
                                uint64_t source = 0;
                                if (execution.test_rpdo_seen != plan.required_mask) {
                                    invalidate_sync_execution("required RPDO missing at SYNC boundary", XNET_CO_TIMING_REASON_RPDO);
                                } else if (!execution.active_clock->map_rx_us(event.timestamp_100ns, source)
                                    || !execution.test_rpdo_last_us || source <= *execution.test_rpdo_last_us
                                    || source - *execution.test_rpdo_last_us <= guard) {
                                    invalidate_sync_execution("required RPDO timestamp overlaps SYNC boundary", XNET_CO_TIMING_REASON_RPDO);
                                } else {
                                    execution.test_rpdo_seen = 0;
                                    execution.test_rpdo_last_us.reset();
                                }
                            }
                            const bool allow_sync_rpdo = !sync_execution || !sync_execution->active_test_rpdo_plan
                                || !sync_execution->invalid;
                            bool required_writes_ok = true;
                            if (sync && allow_sync_rpdo && sync_execution && sync_execution->active_test_rpdo_plan) {
                                TestRpdoWrites writes(*co, model->rpdo_count,
                                    sync_execution->active_test_rpdo_plan->required_mask);
                                CO_process_RPDO(co, true, 0, nullptr);
                                required_writes_ok = writes.ok();
                            } else {
                                CO_process_RPDO(co, sync && allow_sync_rpdo, 0, nullptr);
                            }
                            if (sync && sync_execution && sync_execution->active_test_rpdo_plan
                                && !sync_execution->invalid) {
                                const auto mask = sync_execution->active_test_rpdo_plan->required_mask;
                                for (uint16_t pdo = 0; pdo < model->rpdo_count; ++pdo) {
                                    if ((mask & (1U << pdo))
                                        && (!required_writes_ok || !test_rpdo_applied(co->RPDO[pdo]))) {
                                        invalidate_sync_execution("required RPDO OD application failed or changed", XNET_CO_TIMING_REASON_RPDO);
                                        break;
                                    }
                                }
                            }
                            if (sync) boundary(*this, event, context);
                        }
                    }
                    break;
                }
            }
            // Drain general-protocol mailboxes without advancing timers again.
            auto reset = process_network(0);
            if (reset != CO_RESET_NOT) { reset_stack(reset == CO_RESET_APP); return; }
            if (sync_execution && CO_NMT_getInternalState(co->NMT) != CO_NMT_OPERATIONAL) {
                sync_execution->test_rpdo_seen = 0;
                sync_execution->test_rpdo_last_us.reset();
            }
            if (switching != Switch::none || stop_requested) return;
            if (boundary && !co->nodeIdUnconfigured) CO_process_RPDO(co, false, 0, nullptr);
        } else if (frame.Type != nxFrameType_CAN_BusError && !(frame.Flags & nxFrameFlags_TransmitEcho)) {
            fail(XNET_CO_XNET_ERROR, "unsupported CAN frame type, identifier, or payload ignored");
        }
        offset += size;
    }
}
