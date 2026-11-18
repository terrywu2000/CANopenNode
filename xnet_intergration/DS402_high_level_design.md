# DS402 High-Level Design

This document introduces the architecture of the CANopen slave drive and its external model integration. 

## 1. Purpose and scope

The system simulates a single-axis CiA 402 drive on a CANopen network. A CANopen master controls the drive through standard object dictionary (OD) entries, while the slave implements the drive state machine and exchanges motion data with a motor model.

The first supported operating mode is **Profile Position (PP)**. The drive core generates the position profile; the motor model follows the resulting demand and reports its behavior. Multiple axes are represented by independent slave instances, not multiple axes inside one slave.

DS402 is optional: a device configuration without `drive_402` remains a normal CANopen device. Enabling DS402 requires a compatible JSON configuration; missing or conflicting required objects cause initialization to fail.

## 2. System context

The intended HIL deployment has one robot ECU acting as CANopen master and SYNC producer. Each slave has its own node ID, OD, drive state, motor model, and NI-XNET port. The ports connect to the same physical bus.

```text
                  Robot ECU: CANopen master and SYNC producer
                                      |
                              Shared CAN bus
                       /              |              \
                 XNET port A     XNET port B     XNET port C
                      |               |               |
                  Slave A         Slave B         Slave C
                  402 + OD        402 + OD        402 + OD
                      |               |               |
                  Model A         Model B         Model C
```

In the intended VeriStand deployment, asynchronous Custom Devices (CDs) bridge each slave to its own model using signals. VeriStand's Primary Control Loop (PCL) schedules the models at a fixed step. The CANopen worker and PCL are separate execution domains; a SYNC does not directly step a model. Nodes do not wait for one another's feedback.

**Deployment status:** the native slave and asynchronous model interface exist. The VeriStand CD/signal bridge and production SYNC-to-PCL timing pipeline are not yet implemented or validated.

## 3. Components and responsibilities

| Component | Responsibility |
|---|---|
| CANopen master | Writes drive commands and parameters; configures network behavior and produces SYNC. |
| CANopenNode | Implements communication services, including NMT, SDO, PDO, SYNC, heartbeat and EMCY. |
| DS402 drive core | Owns the power-drive-system (PDS) state machine, PP profile, drive actions, Statusword and fault decisions. It does not depend on CAN or OD access. |
| Device adapter and worker | Bind configured OD entries, convert units, run the drive core, and publish its outputs and accepted model feedback. |
| Model exchange | Provides a per-device `motor_drive_` interface and private mailbox for commands, complete feedback, action responses and heartbeat. |
| Motor model | Tracks demand and reports actual values, action results, limits and fault conditions. It does not own PDS state, write the Statusword or generate a second PP profile. |
| VeriStand CD/PCL | Intended host-side signal bridge and fixed-step model scheduler; this integration remains to be implemented. |

The existing internal model is an explicitly selected ideal-following alternative for functional testing. An unavailable external model is not silently replaced by the internal model.

## 4. Data and control flow

```text
Master                         Slave worker                       Motor model
  |                                 |                                  |
  |-- Controlword, target, -------> |                                  |
  |   profile parameters (OD)       |                                  |
  |                                 |-- DS402 state + PP profile       |
  |                                 |-- SI demand, action, token ----> |
  |                                 |       private mailbox            |-- model step
  |                                 | <--- feedback and heartbeat -----|
  |                                 |-- validate and accept feedback   |
  |                                 |-- update actual values/status    |
  | <--- Statusword, actual values -|                                  |
  |       (OD / PDO / SDO)          |                                  |
```

1. The master owns the target and profile parameters. SDO or configured RPDOs write permitted OD entries.
2. The worker reads the commands, advances the DS402 core and publishes a model demand and any drive action. The adapter converts between device OD units, core counts and the model interface's SI units.
3. The model executes independently and submits a complete feedback snapshot. A successful submit means the snapshot was staged, not that the worker has accepted it or completed an action.
4. The worker validates the feedback, publishes accepted actual values, and updates drive state and Statusword. The master observes these through configured PDOs or SDO.

The exchange is a private in-process mailbox, **not an OD or batch interface**. Standard device OD entries remain available according to their configured permissions; they are not a model feedback channel.

## 5. Timing and PDO behavior

The default worker processes received SYNC boundaries in order. At each boundary it applies available synchronous RPDO data, processes the drive and handles eligible synchronous TPDOs. It also advances normal drive and safety processing using elapsed worker time; asynchronous PDOs do not depend on SYNC.

CANopen does **not** require one RPDO and one TPDO for every SYNC. For example, a synchronous TPDO may be event-driven or configured to transmit after multiple SYNCs; asynchronous PDOs work without SYNC. PDO communication and mapping are currently fixed by the startup JSON and are not writable through SDO.

The planned deterministic pipeline will retain ordered command and accepted-feedback history, relate CAN and PCL time domains, and select complete snapshots against explicit deadlines. It must detect missing timing evidence, late data and capacity violations rather than treating the latest value as automatically valid.

Native tests exercise boundary processing, history and timed selection with injected clocks and timing evidence. These tests are **not** a production timing source: the deployed interface does not support scheduled reads, and real clock mapping, signal transport and PCL sampling have not been validated. No fixed-cycle, physical transmission or hard-real-time guarantee follows from the tests.

## 6. State, safety and lifecycle

Three states must not be confused: the library runtime state, the CANopen NMT communication state and the DS402 PDS drive state. NMT manages communication; the DS402 core decides drive actions and PDS transitions based on its inputs and policy. NMT Operational alone does not mean the drive is enabled.

Drive actions are non-blocking. The model reports their result, and the core accepts only a matching response. Quick stop, disable voltage and fault reaction can preempt normal actions. Model heartbeat, complete feedback and action completion are monitored independently; stale or missing feedback cannot be used to claim that motion stopped or a target was reached. Fault reset requires a new master request and satisfied recovery conditions.

Each model session uses an epoch and sequence identifiers to reject stale commands, feedback and action responses after reset or restart. During an external-model stop handshake, the existing model exchange must remain active until stop completes. In the standalone threaded simulator, stop the drive first, then join the model thread and close the port before uninitializing or destroying the backend. A VeriStand integration must likewise keep signal exchange available during stop and handle an unavailable model with a bounded, reported failure.

These are software behavior and failure-handling mechanisms, not a physical safety guarantee or safety certification.

## 7. Configuration and developer entry points

- **Configuration:** device JSON defines the OD and PDO configuration, and explicitly selects DS402 and the internal or external model. External models require device-unit adaptation and timeout configuration. Device-specific unit assumptions must be reported rather than treated as verified hardware behavior.
- **Protocol and drive core:** [301/](../301/) and [402/](../402/).
- **Native integration:** [model.cpp](../xnet_intergration/src/model.cpp), [device.cpp](../xnet_intergration/src/device.cpp) and [drive_adapter.cpp](../xnet_intergration/src/drive_adapter.cpp).
- **Public model interface:** [motor_drive.h](../xnet_intergration/include/motor_drive.h); backend configuration and usage are described in the [integration README](../xnet_intergration/README.md).
- **Examples and tests:** [integration tests](../xnet_intergration/tests/test_integration.cpp), [drive-core tests](../402/test_drive402.cpp) and the [external model simulator](../xnet_intergration/tests/modsim_app/README.md).

