#ifndef CO_DEVICE_H
#define CO_DEVICE_H

/** System-facing facade for one NI-XNET CANopen slave.
 * The existing xnet_canopen_* and motor_drive_* APIs remain available.
 * Except for CO_DEVICE_BUSY, results are XNET_CO_*; model port operations
 * themselves continue to return MOTOR_DRIVE_*.
 */
#include "xnet_canopen_extensions.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct co_device* co_device_handle;

enum { CO_DEVICE_BUSY = -1000 };

/** Create a facade and its private backend. On failure *device is NULL. */
XNET_CANOPEN_API int32_t co_device_create(co_device_handle* device);
/** Do not overlap destroy with any call. A cleanup failure retains the handle
 * and its diagnostics for retry. Stop the device and join/close model users
 * first; destroy does not close caller-owned motor ports.
 */
XNET_CANOPEN_API int32_t co_device_destroy(co_device_handle device);
XNET_CANOPEN_API int32_t co_device_initialize(co_device_handle device, const char* interface_name,
                                              const char* model_path, const char* persistence_path);
XNET_CANOPEN_API int32_t co_device_start(co_device_handle device);
/** Stop is blocking, not NMT Stop. Keep external model exchange active until it
 * returns, including on failure; then stop/join model users and close the port.
 * Serialize lifecycle changes and stop attempts; do not mistake a concurrent
 * stop rejection for completion of the active stop. Do not block a PCL that
 * must supply the safety response. A worker or safety-handshake failure is
 * returned even when cleanup succeeds; inspect the device diagnostic.
 */
XNET_CANOPEN_API int32_t co_device_stop(co_device_handle device);
/** After stop, join model users and close ports before releasing the backend. */
XNET_CANOPEN_API int32_t co_device_uninitialize(co_device_handle device);
XNET_CANOPEN_API int32_t co_device_get_status_v1(co_device_handle device,
    xnet_canopen_system_status_v1* status, uint32_t capacity);
XNET_CANOPEN_API int32_t co_device_get_capabilities_v1(co_device_handle device,
    xnet_canopen_capabilities_v1* capabilities, uint32_t capacity);
XNET_CANOPEN_API int32_t co_device_get_diagnostic_v1(co_device_handle device,
    xnet_canopen_diagnostic_v1* diagnostic, uint32_t capacity);
XNET_CANOPEN_API int32_t co_device_clear_diagnostic(co_device_handle device);

/** Atomically acquire the exclusive external model writer for an initialized
 * external 402 device. Sets *port to NULL on failure. CO_DEVICE_BUSY means
 * another writer is open; the motor-domain diagnostic retains the original
 * MOTOR_DRIVE_BUSY. The caller owns the port and must close it using
 * motor_drive_close_v1, without overlapping close with port calls. Port calls
 * continue during stop; uninitialize invalidates but does not close the port.
 * The facade does not expose or transfer ownership of the backend endpoint.
 */
XNET_CANOPEN_API int32_t co_device_open_motor_port(co_device_handle device, motor_drive_port_handle* port);

/** Open a model port on an existing legacy device without creating another
 * backend. The legacy caller retains ownership of device and its lifecycle.
 * The port and result contracts match co_device_open_motor_port. Stop the
 * device while model exchange is active, then join model users and close the
 * port before uninitialize/destroy. Never overlap destroy with any call.
 */
XNET_CANOPEN_API int32_t co_device_open_motor_port_from_xnet(
    xnet_canopen_handle device, motor_drive_port_handle* port);

/** Advanced operations retain the corresponding xnet_canopen_* contracts,
 * including per-entry batch failures, permissions and lack of rollback.
 */
XNET_CANOPEN_API int32_t co_device_bind_batch(co_device_handle device, const uint16_t* indices,
    const uint8_t* subindices, uint32_t count, uint32_t* types, uint32_t* sizes,
    uint32_t* buffer_size, uint32_t* batch_id);
XNET_CANOPEN_API int32_t co_device_read_batch(co_device_handle device, uint32_t batch_id,
    uint8_t* buffer, uint32_t buffer_size, int32_t* entry_status, uint32_t entry_count);
XNET_CANOPEN_API int32_t co_device_write_batch(co_device_handle device, uint32_t batch_id,
    const uint8_t* buffer, uint32_t buffer_size, int32_t* entry_status, uint32_t entry_count);
XNET_CANOPEN_API int32_t co_device_read_od(co_device_handle device, uint16_t index, uint8_t subindex,
    uint8_t* buffer, uint32_t buffer_size, uint32_t* value_size);
XNET_CANOPEN_API int32_t co_device_write_od(co_device_handle device, uint16_t index, uint8_t subindex,
    const uint8_t* buffer, uint32_t buffer_size);
XNET_CANOPEN_API int32_t co_device_request_tpdo(co_device_handle device, uint16_t tpdo_number);
XNET_CANOPEN_API int32_t co_device_set_fault(co_device_handle device, uint8_t error_bit,
    uint16_t error_code, uint32_t info_code, uint8_t active);
XNET_CANOPEN_API int32_t co_device_get_drive_unit_report(co_device_handle device, char* buffer,
    uint32_t capacity, uint32_t* required_size);
XNET_CANOPEN_API int32_t co_device_read_sync_event(co_device_handle device, uint64_t epoch,
    uint64_t sequence, xnet_canopen_sync_event_t* event);
XNET_CANOPEN_API int32_t co_device_read_timing_event_v1(co_device_handle device, uint64_t sequence,
    xnet_canopen_timing_event_v1* event, uint32_t capacity);

#ifdef __cplusplus
}
#endif
#endif
