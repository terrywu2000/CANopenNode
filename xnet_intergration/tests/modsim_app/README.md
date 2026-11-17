# External motor simulation slave

`xnet_canopen_modsim` dynamically loads the same library as
[the passive runner](../app/README.md), but owns an independent model thread.
The library exclusively implements PDS, Statusword and PP profile generation.
The model tracks **demand**, not target, using velocity feed-forward, position
feedback, a first-order velocity response, acceleration/velocity saturation and
position limits. It never writes standard drive objects or reinterprets Controlword.

This is a software functional simulator, not a calibrated motor model, a
CANopen master, a hard-real-time system or a hardware safety mechanism.
Use exclusive access to the CAN interface and persistence file.

## Build and run

From the integration directory, build just this app without NI-XNET:

```sh
cmake -S tests/modsim_app -B build-modsim -DCMAKE_BUILD_TYPE=Release
cmake --build build-modsim
./build-modsim/xnet_canopen_modsim \
  --lib /absolute/path/libxnet_canopen.so \
  --config tests/modsim_app/device.config.json \
  --motor tests/modsim_app/motor.json --duration 30
```

The integration build also includes the executable by default; disable it with
`-DXNET_CANOPEN_BUILD_MODSIM_APP=OFF`. The app uses C++17 and the current 48-byte
status ABI. Cross-compiling for NI Linux RT requires its toolchain and a matching
library. Native log configuration and runtime dependencies are unchanged.

CLI paths are relative to the working directory. The runner JSON has the same
three required fields as the passive runner; model/persistence paths are relative
to that JSON. Persistence is never deleted; saved LSS settings can override node
ID and bitrate. Omitting `--motor` uses the defaults in [motor.json](motor.json).
SIGINT/SIGTERM or the optional positive duration request orderly shutdown.

## Model exchange and scheduling

The example [device.json](device.json) explicitly selects external, single-axis PP.
It provides the required standard OD objects, five timeouts and a counts-to-SI
`device_adapter` with an explicit 10000 counts/revolution example resolution.
Ordinary/internal models are rejected, never silently substituted.
The model uses only the dedicated [motor_drive_ interface](../../include/motor_drive.h):

- read: SI demand/action/token and processed feedback sequence/result;
- submit: complete SI feedback, actual executed model step and generic fault;
- heartbeat: independent actual model progress.

No local exchange OD or batch binding is used. API results and processing
results are checked. At most one submission is unconfirmed;
the model continues stepping and sending heartbeat while awaiting consumption.
Rejected submissions are diagnosed and recalculated from fresh snapshots.
Epoch changes discard pending feedback and action timers, **not physical
position/velocity**; sequence zero is skipped. There is no unbounded retry queue.

Each step uses the configured fixed `period_us` (100–10000 us, default 2000).
Late steps increment `overruns` and reschedule without catch-up. Injection time
uses elapsed monotonic wall time. This is not an assertion of real-time accuracy.
Commands are sampled at each model step and rechecked before committing plant
state; changed epoch/action/output permission discards the old calculation.
This bounds simulated response but cannot guarantee synchronous physical
preemption across threads. Stop keeps the model thread available
for the library's new-epoch safety handshake, then joins it and closes the port before uninitialize,
destroy or library unload. Actual motion is not forced to zero on reset/fault.

Status is printed approximately every 500 ms: native runtime/NMT and CAN fields,
plus model epoch/action/sequence, SI demand/actual/velocity, current model
fault class, accepted/rejected counts and overruns. Standard drive diagnostics
can be read separately, not as model execution inputs. These are sampled diagnostics, not
a lossless bus or transition trace. Protocol/model/API errors and cleanup
failures return nonzero; injected PDS faults themselves are reported, not treated
as application crashes.

## Parameters and injection

All [motor.json](motor.json) fields are optional; unknown fields, invalid integer
ranges, nonpositive dynamics, inconsistent bounds or a velocity time constant
smaller than one step are rejected.

| Field | Meaning |
|---|---|
| `period_us`, `action_delay_us` | Fixed model step and minimum BUSY duration for each action |
| `position_gain` | Position tracking gain, 1/s |
| `velocity_time_constant_s` | Velocity response time constant, seconds |
| `max_velocity`, `max_acceleration` | Plant limits, rad/s and rad/s² |
| `stop_deceleration` | No-output braking, rad/s²; Quick Stop uses valid command deceleration capped by the plant acceleration limit |
| `min_position`, `max_position`, `initial_position` | Finite motor-axis positions in rad; backend owns direction/origin conversion |
| `fault_class`, `fault_detail`, `fault_after_us`, `fault_duration_us` | Generic model condition from elapsed start time; zero duration holds indefinitely, zero class disables injection |
| `freeze_feedback_after_us` | Stop submitting feedback after this elapsed time; heartbeat/stepping continue; zero disables |
| `freeze_heartbeat_after_us` | Freeze heartbeat only; zero disables |
| `error_action` | Return ERROR for action enum 1–8; zero disables |

For example, `fault_class: 2`, `fault_detail: 7`, `fault_after_us: 2000000`,
`fault_duration_us: 3000000` reports overcurrent/detail 7 from second 2 through
second 5. The example device adapter maps that pair to `0x2310`; the plant
does not know device fault codes.
Clearing the condition does not clear the core's latched `0x603F`: a master must
send a new Fault Reset bit-7 edge after health/reset permission is restored.
Action delays longer than `action_timeout_us` test the core's action watchdog.
Setting `freeze_feedback_after_us: 2000000` independently exercises feedback
timeout while heartbeat continues. No fault is injected with `set_fault`.

Older motor parameter files used counts and `fault_code`. Convert their motion
values to SI using the configured resolution, replace fault codes with mapped
class/detail, and remove old exchange OD extensions/bindings from the device/host.
The supplied SI defaults preserve the former plant scale at 10000 counts/rev.

## Master-side verification

Example defaults: node 10, 125 kbit/s, Pre-operational at boot.
The master sends NMT Start (`0x000`, bytes `01 0A`), then Controlword shutdown
`0x0006`, switch on `0x0007`, enable operation `0x000F`, waiting for Statusword
state masks `0x0021`, `0x0023`, `0x0027` respectively.

RPDO1 is CAN ID `0x20A`, seven bytes:
Controlword U16 LE, mode I8 (=1), target I32 LE. Target 1000 and new-set-point
can be sent as `1F 00 01 E8 03 00 00`; wait for acknowledge bit 12, then send
`0F 00 01 E8 03 00 00`. TPDO1 (`0x18A`, every 20 ms Operational) reports
Statusword U16 LE, mode display I8, actual position I32 LE. Target reached
requires error ≤5 counts for 20 ms. Profile velocity/acceleration/deceleration
are master-owned SDO parameters (`0x6081/83/84`); no model profile is generated.

Exercise negative/relative moves, Halt, Quick Stop (`0x000B`), disable voltage,
fault/reset, NMT changes, Reset Communication and Reset Node from the master.
Resets change epoch; neither automatically restores operation.

## Hardware-independent validation

```sh
cmake -S . -B build-modsim-tests \
  -DXNET_CANOPEN_BUILD_LIBRARY=OFF -DXNET_CANOPEN_BUILD_TESTS=ON
cmake --build build-modsim-tests
ctest --test-dir build-modsim-tests -R 'modsim|app_' --output-on-failure
```

Tests run the same plant and model-thread implementation against the real stack
with fake XNET: tracking/direction/limits, delayed actions and preemption,
RPDO→PP→feedback→TPDO, Quick Stop, both protocol resets, stop/start with preserved
plant state, persistent fault/clear/reset, independent heartbeat/feedback
timeouts, invalid motor parameters, and dynamically loaded timed
and signal shutdown. Missing feedback while moving is also exercised during
native stop and Reset Communication; both complete their bounded fault path and
retain an explicit unconfirmed-stop diagnostic.
They do not certify NI-XNET hardware, motor accuracy or safety.

Feedback-freeze testing now verifies the first latched `0x603F` is `0x8130`,
Fault Reaction Active is published with output revoked, and its action deadline
enters FAULT without a fabricated DONE. Last accepted actuals remain unchanged;
this state transition is not proof that a physical drive stopped. Heartbeat can
continue while feedback is missing. Recovery requires fresh valid traffic and a
new Fault Reset edge/acknowledgement; merely returning traffic never re-enables
operation. See the integration [watchdog semantics](../../README.md#ds402-pp-configuration).
