# DS 402 (CiA 402) 驱动 Profile 需求与开发指南

> 适用范围：当前仓库 CANopenNode 核心，以及 `xnet_intergration/` NI-XNET CANopen 从站集成。
> 参考资料：`reference/canopennode-rtt`（Pure-C CiA 402 Device 核心 + RT-Thread 适配）、`reference/EtherCAT_Slave`（EtherCAT 侧 CiA 402 示例应用）。
> 本文是后续开发的需求基线。标注 **[待确认]** 的条目需要在开发前由负责人决策。

---

## 1. 背景与目标

### 1.1 现状

- 当前 CANopenNode（根 `README.md`）是 ANSI C、MISRA C:2012 风格的协议栈，分层为 `301/`、`303/`、`304/`、`305/`、`309/`、`storage/`、`extra/`，**不包含** CiA 402 实现。
- `xnet_intergration/` 把 CANopen 从站通过 NI-XNET 暴露给 VeriStand 模型，**只做通信**：
  - `include/xnet_canopen.h` 文件头明确声明："No function interprets a CiA 402 controlword or generates the application's statusword."，并声明 NMT 状态、CANopen 状态、应用驱动状态是三个独立概念。
  - `xnet_intergration/src/model.cpp:280` 注释："adding 0x6040 does not give the library a CiA 402 controlword interpreter."。即 0x6040 等对象目前只是 OD 存储与访问元数据。
  - `xnet_intergration/tests/app/README.md:113` 说明测试应用不实现 CiA 402。
- 已有 Kinco FD124 的 EDS（`xnet_intergration/eds/Kinco_FD_20250417_V1.2.eds`）与 JSON 配置（`examples/kinco_FD124.json`、`tests/app/kinco_FD124.config.json`，目前为未跟踪文件）。

### 1.2 目标

在不破坏 xnet 通信边界的前提下，提供一个**可复用、与硬件解耦**的 CiA 402 PDS（Power Drive System）状态机与模式处理模块，使上层（xnet 模型层或应用）只需实现驱动回调即可获得标准的 Controlword/Statusword 行为，并满足以下目标：

1. 按 IEC 61800-7-201 / CiA 402 规定实现 PDS 状态机、Controlword 解析、Statusword 生成。
2. 支持多轴（`logicalDevice` 偏移），每轴 OD 布局符合 CiA 402 标准索引。
3. 支持 PP、PV、HM、CSP、CSV、CST 模式，编译期可裁剪。
4. 驱动动作通过回调（DriveIF）抽象，核心模块不包含任何硬件或 CAN 代码。
5. 核心为 Pure C，无堆分配，回调非阻塞，可在 RT 环境运行。
6. 与 xnet 的 OD 访问、批量 API、复位语义（Reset Node / Reset Communication）保持一致。

### 1.3 非目标

- 不实现厂商私有对象（如 Kinco 的 0x2FF0 系列），仅预留扩展点。
- 不实现运动规划器（轨迹生成）本身；DriveIF 由应用或驱动适配层提供。
- 不修改 `xnet_canopen.h` 已发布的 ABI，除非另行批准（见 §9.3）。
- 不替代 EtherCAT 侧的 `cia402appl.c`，两者仅作为行为参考。

---

## 2. 参考实现对比

| 维度 | canopennode-rtt `profile/cia402` | EtherCAT_Slave `cia402appl.c` | xnet_intergration（当前） |
|---|---|---|---|
| 定位 | 独立 Pure-C 设备核心，RT-Thread 适配 | 示例应用，嵌入 EtherCAT 从站状态机 | 通信栈 + 外部电机驱动模型（Slave 侧），不解释 402 |
| 状态机入口 | `CO_402_device_axisProcess()`（`device/CO_402_device_fsa.h`） | `CiA402_StateMachine()`（`cia402appl.c:317`） | 无 |
| Controlword 来源 | OD 0x6040 绑定到 `objControlWord`，经 PDS 逐步处理 | OD 0x6040 映射到 `objControlWord`（`cia402appl.c:202`） | 无 |
| Statusword 生成 | 核心按 PDS 状态生成 | `StatusWord &= ~(STATUSWORD_STATE_MASK|REMOTE)` 后按状态或位设置（`cia402appl.c:333-336`） | 无 |
| 驱动动作 | `CO_402_drive_t` 回调 | `CiA402_DummyMotionControl()`（`cia402appl.c:567`） | 无 |
| 安全所有权 | QuickStop、DisableVoltage、FaultReaction 可抢占 BUSY 动作 | 无显式所有权模型 | 无 |
| 故障复位 | 0x6040 bit7 上升沿（edge-triggered） | 固定掩码 `CONTROLWORD_COMMAND_FAULTRESET_MASK`（`cia402appl.h:57`） | 无 |
| 多轴 | 支持，OD 基址 `0x6000 + n*0x0800` | 支持（`LocalAxes[]`） | 无 |
| 模式 | 编译期开关 `CO_402_CONFIG_MODE_{PP,PV,HM,CSP,CSV,CST}` | 按 0x6502 与 PDO 映射 | 无 |
| 堆使用 | 无堆，应用持有所有存储 | 静态 | 有 C++ 动态对象 |

**结论**：canopennode-rtt 的 Pure-C 核心在结构上最适合 xnet 复用（无堆、回调边界清晰、有安全所有权模型）。EtherCAT 示例更接近"验证用法"，状态转换表可用于交叉核对。

---

## 3. 术语与状态

### 3.1 三个独立状态（必须区分）

1. **运行时状态**：`xnet_canopen_status_t.runtime_state`（UNINITIALIZED / STOPPED / RUNNING / FAILED）。
2. **CANopen NMT 状态**：`nmt_state`。
3. **CiA 402 PDS 状态**：本模块维护，通过 Statusword bit0-3、5、6 表示。

PDS 状态不得因 NMT 切换而隐式改变，除非规范或本模块策略明确要求（见 §6.4）。

### 3.2 PDS 状态（CiA 402 标准）

`NOT_READY_TO_SWITCH_ON`、`SWITCH_ON_DISABLED`、`READY_TO_SWITCH_ON`、`SWITCHED_ON`、`OPERATION_ENABLED`、`QUICK_STOP_ACTIVE`、`FAULT_REACTION_ACTIVE`、`FAULT`。

参考：`reference/canopennode-rtt/profile/cia402/common/CO_402_state.h`；EtherCAT 对应常量 `cia402appl.h:90-97`。

---

## 4. 功能需求（FR）

每条需求给出验收标准。编号用于测试追溯。

### 4.1 OD 与对象

| ID | 需求 | 验收标准 |
|---|---|---|
| FR-OD-01 | 每个 CiA 402 轴必须提供以下对象：0x603F（U16 错误码）、0x6040（U16 Controlword，RPDO）、0x6041（U16 Statusword，TPDO）、0x6060（I8 Modes of Operation）、0x6061（I8 Modes of Operation Display）、0x6064（I32 Position Actual）、0x606C（I32 Velocity Actual）、0x607A（I32 Target Position）、0x60FF（I32 Target Velocity）、0x6502（U32 Supported Drive Modes） | 启动时校验存在性、标量 VAR、长度、访问属性与 PDO 映射；缺失则拒绝启动并返回 `XNET_CO_MODEL_ERROR`（或核心的对应错误码） |
| FR-OD-02 | 多轴时，轴 n 的对象基址为 `0x6000 + n * 0x0800`（canopennode-rtt 约定） | 同一设备 0x6000、0x6800、0x7000 可同时工作，互不覆盖 |
| FR-OD-03 | 核心不得假定对象有符号语义；符号/无符号由 OD/EDS 决定 | 0x6060 为 INTEGER8，0x6061 为 INTEGER8，读写不做错误的符号扩展 |
| FR-OD-04 | 对象访问须经过 OD 锁（与 xnet 的批量 API 锁序一致，见 §8.2） | 通过 SDO 写 0x6040 与批量写 API 并发时无数据竞争 |

**实现说明**：`canopennode-rtt` 的 OD 扩展通过 `CO_402_device_bindOD()` 绑定，并在 Communication Reset 时重新绑定（见 `docs/en/cia402-device-rtt.md`）。xnet 中对应的是 `Model` 的 `resolve_node` / `configure` 流程（`xnet_intergration/src/model.cpp:427`、`:494`），绑定点应放在同一流程里。

### 4.2 PDS 状态机

| ID | 需求 | 验收标准 |
|---|---|---|
| FR-PDS-01 | 按 CiA 402 状态转换表实现 PDS，所有状态转换由 Controlword 命令驱动 | 单元测试覆盖全部合法转换（至少 1–15 号转换）及非法转换忽略 |
| FR-PDS-02 | Statusword 由 PDS 状态派生，bit0-3、5、6 与状态一一对应 | 每个状态下 Statusword 的状态掩码 `0x006F` 与参考值一致（见 `cia402appl.h:90-97`） |
| FR-PDS-03 | 上电后必须处于 `NOT_READY_TO_SWITCH_ON`，经内部自检后自动进入 `SWITCH_ON_DISABLED` | 上电后 Statusword 状态位为 0x0040 |
| FR-PDS-04 | Controlword 0x000F 视为 Switch On + Enable Operation 的单条命令，但仍按状态逐步处理，不得跳过 `SWITCHED_ON` 的驱动回调 | 由 `READY_TO_SWITCH_ON` 直接写 0x000F，回调顺序为 switchOn → enableOperation |
| FR-PDS-05 | 驱动动作必须非阻塞，返回 BUSY / DONE / ERROR，核心据此推进 | 回调 BUSY 时状态保持，DONE 后提交新状态 |
| FR-PDS-06 | Shutdown、Disable Voltage、Quick Stop 必须按 CiA 402 规定的优先级生效 | Quick Stop 可抢占 BUSY 的 enableOperation；Disable Voltage 可抢占任何 BUSY |
| FR-PDS-07 | Quick Stop Active 的退出行为按规范：Controlword bit2=1 后回到 `OPERATION_ENABLED`（若 bit2 恢复），否则保持 | 单元测试覆盖 0x0002 与 0x0006 的两种退出路径 |

### 4.3 Controlword / Statusword 位定义

| ID | 需求 |
|---|---|
| FR-CW-01 | Controlword 位 0–3、7 参与 PDS 命令；位 4–6、8 作为模式相关位（set-point、change immediately、relative、halt） |
| FR-CW-02 | 故障复位为边沿触发：仅在 FAULT 状态下 bit7 由 0 变 1 时生效（canopennode-rtt 行为） |
| FR-CW-03 | 不得把 Statusword 的保留位置为非零值；bit11（internal limit）与 bit12（模式相关）仅由对应模式写入 |
| FR-CW-04 | Statusword bit10（target reached）、bit12（setpoint ack / HM attained / drive follows command）、bit13（following error / HM error）由模式模块维护 |

### 4.4 模式

| ID | 需求 |
|---|---|
| FR-MODE-01 | 模式通过编译开关 `CO_402_CONFIG_MODE_{PP,PV,HM,CSP,CSV,CST}` 启用，默认全部关闭 |
| FR-MODE-02 | 模式值：PP=1、PV=3、HM=6、CSP=8、CSV=9、CST=10 |
| FR-MODE-03 | 0x6502 位映射：PP=bit0、PV=bit2、HM=bit5、CSP=bit7、CSV=bit8、CST=bit9，必须与编译开关一致 |
| FR-MODE-04 | 写 0x6060 为不支持的模式时拒绝写入（SDO Abort 或返回错误），0x6061 保持原值 |
| FR-MODE-05 | 模式切换仅在 `OPERATION_ENABLED` 之外或经过 `modeExit` 回调完成后生效 **[待确认]**：是否允许运行中切换 |
| FR-MODE-06 | 模式接口（modeEnter / modeExit / profilePosition / profileVelocity / homing）为 **追加式** 扩展，不破坏已有 DriveIF ABI |

### 4.5 DriveIF（驱动回调）

| ID | 需求 |
|---|---|
| FR-DRV-01 | 必须提供以下回调：`shutdown`、`switchOn`、`enableOperation`、`disableOperation`、`quickStop`、`faultReaction`、`faultReset`、`disableVoltage` |
| FR-DRV-02 | 回调返回值为 `CO_402_drive_result_t`（BUSY/DONE/ERROR） |
| FR-DRV-03 | 安全相关回调（quickStop、disableVoltage、faultReaction）可以抢占 BUSY 的普通动作；抢占时必须同步撤销被抢占动作的物理效果（见 `CO_402_device_fsa.h` 中 `CO_402_device_axisControlwordReadFailed` 注释） |
| FR-DRV-04 | 所有回调禁止阻塞、禁止调用 CAN/OD 访问函数、禁止在回调中获取 OD 锁 |

### 4.6 故障与复位

| ID | 需求 |
|---|---|
| FR-FLT-01 | 任何内部故障（PDS 操作失败、Controlword 读取失败、产品故障）都进入 `FAULT_REACTION_ACTIVE`，再进入 `FAULT` |
| FR-FLT-02 | 故障源必须被锁存并通过诊断接口（DiagIF）暴露，缺失映射时仍按失败关闭（fail-closed） |
| FR-FLT-03 | 0x603F 必须反映最近一次锁存的故障代码 |
| FR-FLT-04 | Reset Node 恢复 OD 默认值；Reset Communication 保留 OD 值并重新绑定扩展（与 xnet `status.reset_node_count` / `reset_communication_count` 的语义保持一致） |
| FR-FLT-05 | Reset Communication 期间，若驱动处于 OPERATION_ENABLED，必须先经过 quickStop 或 faultReaction，不得直接丢弃 |

### 4.7 运行与集成

| ID | 需求 |
|---|---|
| FR-RUN-01 | 提供周期入口 `CO_402_device_axisProcess()`，建议周期 1–10 ms，由调用方决定 |
| FR-RUN-02 | 支持两种调度方式：与 CANopen 主循环共用线程；或独立 worker 线程（参考 `PKG_CANOPENNODE_CIA402_DEVICE_RTT_DEDICATED_WORKER`） |
| FR-RUN-03 | 诊断与日志通过 `CO_402_LOG_*` 宏输出，允许通过 `CO_402_LOG_CUSTOM_HEADER` 覆盖 |
| FR-RUN-04 | Demo 模式只做软件仿真，不得驱动硬件 |

---

## 5. 非功能需求（NFR）

| ID | 需求 | 验证方式 |
|---|---|---|
| NFR-01 | 核心模块为 C99 或 C11，无堆分配 | 代码审查、`grep malloc` 为空 |
| NFR-02 | 与现有 xnet 代码风格一致（xnet 为 C++ 实现，对外 C ABI） | 代码审查 |
| NFR-03 | MISRA C:2012 风格（参照根 README）；允许偏离需在注释中说明 | 静态检查 |
| NFR-04 | 单轴 `axisProcess` 最坏执行时间需在目标平台实测并记录 **[待确认]**（建议 < 100 µs，不含回调） | 基准测试 |
| NFR-05 | 不得在 `xnet_canopen.h` 已有函数的签名或语义上做破坏性修改 | ABI 对比 |
| NFR-06 | 所有新文件包含版权与模块说明头，与 `CO_402_*` 风格一致 | 代码审查 |
| NFR-07 | 只在需要注释处添加注释（仓库约定） | 代码审查 |

---

## 6. 行为规范（实现约束）

### 6.1 Controlword 处理顺序

1. 读取 0x6040 快照（一次性读取，避免撕裂）。
2. 若读取失败，调用 `CO_402_device_axisControlwordReadFailed()`，进入 FAULT 路径。
3. 判定 Quick Stop / Disable Voltage 等安全命令是否优先。
4. 按当前 PDS 状态处理普通命令。
5. 更新 Statusword，写入 0x6041。

### 6.2 Statusword 派生

Statusword 的状态位由 PDS 当前状态决定，模式相关位由模式模块写入。Statusword 写入应在 OD 锁内完成，避免 TPDO 读到不一致的组合。

### 6.3 Fault Reset

- 只在 FAULT 状态且 bit7 上升沿时调用 `faultReset`。
- 连续保持 bit7=1 不得重复触发。
- 复位成功后回到 `SWITCH_ON_DISABLED`。

### 6.4 NMT 与 PDS 的关系

- NMT 进入 Stopped 时，应先将 PDS 转到安全状态（经 quickStop 或 disableVoltage，具体见 **[待确认]** §9.2）。
- NMT 重新进入 Operational 不得自动恢复 `OPERATION_ENABLED`，必须由主站重新发送命令序列。

---

## 7. 推荐架构

### 7.1 分层

```
VeriStand / 应用模型
        │  (xnet_canopen.h 批量 OD API，不变)
        ▼
xnet_intergration/src/model.cpp        ← OD 描述、EDS/JSON 生成
        │
        ▼
xnet_intergration/src/cia402/   [新增，Pure C]
        ├─ CO_402_device.{h,c}          轴与管理器
        ├─ CO_402_device_fsa.{h,c}      PDS 状态机
        ├─ CO_402_device_od.{h,c}       OD 绑定与校验
        ├─ CO_402_device_sync.{h,c}     RPDO/TPDO 同步
        ├─ CO_402_drive.h               DriveIF（回调）
        └─ 驱动适配层（本项目实现）     将回调映射到 Kinco FD124 等
        │
        ▼
CANopenNode 核心（301/305/309 等）
```

### 7.2 复用策略 **[待确认 — 需负责人决策]**

| 方案 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| A. 直接引入 | 把 `reference/canopennode-rtt/profile/cia402/{common,device}` 复制到 `xnet_intergration/src/cia402/` | 代码已成熟，测试少 | 需确认许可证；需去除 RT-Thread 依赖；与 xnet 的锁模型需适配 |
| B. 仅移植行为 | 以 canopennode-rtt 为行为规范，重新实现 Pure-C 核心 | 许可与依赖清晰 | 工作量大，需重新验证 |
| C. 只在模型层实现 | 在 `model.cpp` 中实现 402 解释器 | 改动小 | 违背 xnet 通信边界声明；难以复用到其他平台 |

**推荐方案 A（若许可证允许）或 B**。不推荐 C，因为它直接违背 `xnet_canopen.h` 的声明，并使模型层承担状态机逻辑。

### 7.3 与 xnet 集成点

1. **OD 描述**：在 `model.cpp` 的 `load()` 中，当 JSON 配置中声明 `cia402` 段时，自动生成 §4.1 所需的对象（参考 `model.cpp:280` 之后的对象创建流程）。
2. **绑定**：在 `Model::configure()`（`model.cpp:494`）或 `device.cpp` 中，于 OD 初始化后调用 `CO_402_device_bindOD()`。
3. **周期调用**：`device.cpp` 只有一个协议处理 worker（文件头注释，`src/device.cpp:3`）。建议在该 worker 的 `CO_process` 循环中调用 `CO_402_device_axisProcess()`，避免新增线程。
4. **Reset**：`xnet_canopen_device::reset_stack()`（`src/device.cpp:230`）在 `model->restore()`（`:243`）之后完成复位。402 重新绑定应放在该流程中，并在成功完成后才发布计数（参考 `device.cpp:247-249` 的注释）。
5. **状态快照**：`xnet_canopen_status_t` 目前不包含 402 状态；若需暴露，应新增独立 API（`xnet_canopen_cia402_status_t`），不修改已有 40 字节结构（见 `xnet_canopen.h` 注释关于 ABI 的约束）。

### 7.4 DriveIF 对 Kinco FD124 的适配 **[待确认]**

- 需确认 FD124 的 0x6040/0x6041 行为是否与标准一致（尤其 bit4、bit5、bit8 的含义）。
- 需确认 FD124 EDS 中是否声明 0x6502、0x603F，以及它们的 PDO 映射属性。
- 需确认 FD124 的 quickStop 选项码（0x605A）与 shutdown 选项码（0x605B）默认值。

---

### 7.5 外部电机驱动模型接口

> 本节所称“外部电机驱动模型”（下文简称“模型”）位于 **Slave 侧**，与 xnet 从站设备同侧，不在主站侧。它负责按模式计算电机驱动的实际值（位置、速度、力矩），不是电机本体模型。

**结论：不新增任何 C API 函数。** 模型与 402 核心统一通过 OD 对象和现有批量接口（`xnet_canopen_bind_batch`、`read_batch`、`write_batch`）交互。PDS 状态机与 Statusword 在库内运行，模型只看到 OD。

#### 7.5.1 职责分工

| 角色 | 负责的 OD | 访问通道 |
|---|---|---|
| 主站 | 0x6040 Controlword、0x6060 模式 | SDO / RPDO |
| 402 核心（库内 device worker，每周期） | 0x6041 Statusword、0x6061 mode_display、demand（0x607A、0x6081、0x60FF、0x6071）、0x603F 错误码，执行 FaultReaction | 直接读写 OD（worker 内加锁，不经过批量 API） |
| 外部电机驱动模型（写） | 0x6064、0x606C、0x6077 实际值；0x2F00 故障请求；0x2F01 心跳 | `write_batch`（需要 local_write） |
| 外部电机驱动模型（读） | 状态、命令、模式、对应模式的 demand | `read_batch`，每个模式一组，单次调用读取 |

#### 7.5.2 模型自有对象（厂商自定义区 0x2000–0xBFFF）

| 索引 | 名称 | 类型 | 写入方 | 语义 |
|---|---|---|---|---|
| 0x2F00 | 模型故障请求 | UNSIGNED16 | 模型 | 非零表示请求故障，值为要写入 0x603F 的错误码。模型保持非零直到核心已进入 FAULT；核心不回写。 |
| 0x2F01 | 模型心跳 | UNSIGNED32 | 模型 | 模型每个周期递增。核心检测到计数不再变化即判定模型超时。 |

- 故障请求用 OD 对象，而不用 `xnet_canopen_set_fault`。后者只产生 EMCY 与厂商故障位，不会使 PDS 进入 FAULT。
- 心跳计数用于超时判定，不依赖跨机器的墙钟时间。

#### 7.5.3 批量分组（模型侧绑定，启动时完成）

模型在 setup 阶段一次性绑定所有分组，之后按 0x6061 选择分组。绑定 ID 在 stop/start 后保持。

| 分组 | 方向 | 内容 | 状态 |
|---|---|---|---|
| `view_pp` | 读 | 0x6040、0x6041、0x6061、0x603F、0x2F01、0x607A、0x6081 | 首期 |
| `view_pv` | 读 | 0x6040、0x6041、0x6061、0x603F、0x2F01、0x60FF、0x6083、0x6084 | 首期 |
| `feedback_pp` | 写 | 0x6064、0x606C、0x2F01（最后写入） | 首期 |
| `feedback_pv` | 写 | 0x606C、0x2F01（最后写入） | 首期 |
| `fault_req` | 写 | 0x2F00 | 首期 |
| `demand_tq` / `feedback_tq` | 读 / 写 | 0x6071 / 0x6077 + 0x2F01 | 预留 |

- 状态、模式与 demand 放在同一分组，由一次 `read_batch` 读取，保证三者属于同一周期（见 §7.5.7）。
- 模式扩展方式：新增模式只需新增分组和核心模式表的一行，不新增 API 函数，也不修改状态机。
- 不支持的模式：mode_display 为 0，模型不得写运动分组。

#### 7.5.4 运行阶段流程

1. 主站发送 Controlword，核心在 worker 中按 PDS 合法转换表推进状态，并更新 0x6041、0x6061 与 demand。
2. 模型用一次 `read_batch(view_pp)` 或 `read_batch(view_pv)` 读取状态、模式与对应 demand，三者属于同一周期。
3. 模型按模式计算实际值。仅在 `OPERATION_ENABLED` 或 `QUICK_STOP_ACTIVE` 下写 feedback 分组。
4. 模型需要故障时写 `fault_req`，核心在下一周期锁存故障并走 FaultReaction。
5. 核心每周期在一次锁内读取实际值与心跳。心跳未变化时，按 §7.5.6 处理超时。

#### 7.5.5 所有权与权限

- **Controlword（0x6040）**：只由主站写。模型只读。
- **Statusword（0x6041）、mode_display（0x6061）、0x603F**：只由核心写。模型 local_write 为 false。
- **Demand（0x607A、0x6081、0x60FF、0x6071）**：只由核心写。模型只读。
- **实际值（0x6064、0x606C、0x6077）、0x2F00、0x2F01**：JSON 中 local_write 为 true，只由模型写。SDO 访问可以是只读，本地写权限与 SDO 权限相互独立。
- 批量写入不运行状态机，不产生 EMCY。状态与故障的执行只由核心完成。
- 回调中不得获取锁（见 §8.2）。

#### 7.5.6 超时与缺省模型

- **超时（主判据）**：核心连续 N 个周期未观察到 0x2F01 变化，视为模型故障。N 由 `feedback_timeout_us` 与 worker 周期换算得到。超时前未收到任何心跳，同样视为故障。超时期间实际值冻结。
- **缺省模型（InternalModel）**：未配置外部电机驱动模型时，实际值等于 demand，行为与早期版本一致。
- **启用**：仅当 JSON 中声明 `drive_402` 段时启用 402 与外部电机驱动模型接口，未声明时不改变现有行为（§9 决策 1）。

#### 7.5.7 批量接口的已知约束与应对

| 约束（来自 `xnet_canopen.h`） | 对本设计的影响 | 应对 |
|---|---|---|
| 单次 `read_batch` 在一次加锁内读取全部条目，worker 仅在周期之间释放锁 | 可作为同一周期的一致快照。这是实现保证，头文件尚未写明 | 同一模式的状态与 demand 放入同一分组，一次读取（§7.5.3）。在 §7.5.9 中补充头文件说明 |
| 跨调用不保持锁，`read_batch` 与 `get_status` 之间可能发生复位 | 不存在联合的 OD + 状态快照 API，不能把状态与 OD 作为一个原子快照 | 在 `get_status` 之间夹住 `read_batch`，复位计数变化则丢弃并重读 |
| `write_batch` 逐条尝试，无回滚 | 反馈分组可能只写入部分条目 | 0x2F01 心跳放在最后写入。心跳条目失败时核心视为未更新，部分写入的实际值由下一周期覆盖 |
| `write_batch` 不产生 TPDO，不运行状态机 | 模型写入不会立即通知主站 | 按设计使用：TPDO 由核心的 TPDO 周期发送 |
| 本地读取要求对象可 SDO 读取 | 0x2F00、0x2F01 必须声明为可读 | 在 JSON 中声明 |
| 批量绑定只校验地址与大小，不校验后续权限 | 权限错误在调用时返回 | 在 setup 阶段检查 local_write 配置 |

#### 7.5.8 与当前核心实现的差距（待开发）

`402/drive402.hpp` 当前的 `ExternalModel` 是纯 C++ 接口，不关心 OD，与本设计有以下差距：

1. `post(Feedback{position, velocity, fault})` 没有力矩字段，也不区分模式。
2. 没有读取命令位与状态的入口。
3. 超时基于核心的时间戳，应改为心跳计数。
4. 没有模型故障请求入口，应改为读取 0x2F00。

开发方案：`402/` 核心保持不依赖 OD 的纯逻辑，新增一个 OD 适配层（位于 `xnet_intergration/src/`），负责把 OD 快照映射为核心的输入，并把核心输出写回 OD。

#### 7.5.9 对外部接口文档的影响

`xnet_canopen.h` 文件头声明"No function interprets a CiA 402 controlword"。库内运行 402 核心后，这句话需要修订为：模型不解释 Controlword，库内 402 核心按 JSON 声明启用后解释它。开发时需同步修改头文件注释。

同时，`xnet_canopen_read_batch` 的注释需补充快照语义：单次调用内所有条目在同一次加锁中读取，对应一个 worker 周期结束时的状态。跨调用的一致性仍须通过 `get_status` 复位计数判断。

#### 7.5.10 外部电机驱动模型的 Slave 侧整合评估

外部电机驱动模型与 xnet 从站位于同一 Slave 侧，通过本地批量 OD 接口交互，不经过 CAN 总线。评估结论与待确认项如下：

| 评估项 | 结论 | 依据 / 处理 |
|---|---|---|
| 数据通道 | 本地 C API，无 CAN 报文开销 | `xnet_canopen.h` 批量接口（~177–242）。反馈延迟约为一个 worker 周期，往返约两个周期，需在文档中注明 |
| 心跳 0x2F01 语义 | 由“链路检测”变为“模型存活检测” | 超时仍为 `feedback_timeout_us` 对应的 N 个 worker 周期（`402/drive402.hpp:50–75`，`ExternalModel` 的 `age_us_` 计时） |
| 故障隔离 | 模型崩溃或挂起不会被掩盖 | 0x2F00 为 OD 故障请求，不等同于 `xnet_canopen_set_fault`；后者仅发 EMCY，不进入 FAULT（§7.5.7） |
| 锁与抖动 | 批量调用持有 runtime 递归锁，模型的批量大小和频率会影响 worker 抖动 | `device.cpp` 的 `call()`/`batch_io`。要求单次批量保持小规模（控制 WCET）。锁顺序：生命周期锁 → runtime 锁（§8.2）；回调中不得再取锁 |
| 访问权限 | 0x2F00、0x2F01 需 `local_write: true` 且 SDO 为只读 | `allows_local_write`（`internal.hpp:72`）与 SDO 访问权限相互独立（`device.cpp:590`）。须验证网络 SDO 写 0x2F00 被拒绝 |
| 批量原子性 | 无回滚；部分写入时返回 `XNET_CO_PARTIAL_FAILURE` | 心跳放在批量写的最后一项，保证实际值先于心跳更新。跨调用的快照一致性不保证（`xnet_canopen.h` ~274–277） |
| 复位与生命周期 | 批量 ID 在 stop/start 与主站复位后保留，uninitialize 后失效 | 模型须在 uninitialize 后重新 `bind_batch`（`xnet_canopen.h` ~181） |
| ABI | 无需新增 C 函数 | 仅需修订 `xnet_canopen.h` 头注释中关于 CiA 402 的表述（§7.5.9） |
| 宿主进程 / 线程 | **待确认**：模型是否与 xnet 设备在同一进程、同一线程 | `xnet_canopen.h` 头注释将“VeriStand model”描述为独立方。若同进程，批量调用跨线程；若不同进程，需另行确定 IPC 方式，本文档暂不覆盖 |
| 驱动模型的反馈字段 | 反馈结构不含力矩字段 | `ExternalModel` 的 `Feedback` 仅有位置、速度、故障（`402/drive402.hpp`）。力矩或电流反馈需先扩展结构体，见 §7.5.8 |

**结论**：外部电机驱动模型可以在不改变 402 核心和 CAN 协议的前提下，以本地批量接口整合到 Slave 侧。需要解决的问题是宿主进程与线程模型（上表“待确认”项）、批量调用的 WCET 限制，以及力矩反馈字段的扩展。

#### 7.6 系统架构框图

**组件关系**（箭头表示数据或调用方向）

```
  ┌──────────────────────┐
  │   CANopen 主站       │  Controlword 0x6040、模式 0x6060、目标值
  │ (PLC / 上位机)       │  状态与实际值的读取方
  └──────────┬───────────┘
             │ CAN 总线：SDO、RPDO/TPDO、NMT、EMCY、心跳
             ▼
  ┌────────────────────────────────────────────────────────────────────┐
  │  Slave 侧（同一台设备内）                                           │
  │                                                                    │
  │  ┌──────────────────────────────────────────────────────────────┐  │
  │  │ xnet 设备（CANopen 从站，每设备一个 device worker 线程）        │  │
  │  │  301/  CANopenNode 核心：NMT、SDO、PDO、EMCY、对象字典 (OD)     │  │
  │  │  xnet_intergration/src/  OD 适配层（只做映射，不含状态机）      │  │
  │  │  402/  DS 402 核心（纯逻辑，与 xnet、CAN 端口无关）             │  │
  │  └──────────────────────────────────────────────────────────────┘  │
  │          ▲ 批量 OD 接口：bind_batch / read_batch / write_batch      │
  │          │ 模型读取 PDS 状态与命令；写实际值、0x2F00、0x2F01        │
  │  ┌───────┴──────────────────────────────────────────────────────┐  │
  │  │ 外部电机驱动模型（Slave 侧应用模型，可选）                     │  │
  │  │  · 读取 PDS 状态与命令，决定运行或请求故障                     │  │
  │  │  · 按模式计算实际值（位置、速度、力矩）                        │  │
  │  │  · 未连接时由库内 InternalModel 代替（实际值 = demand）        │  │
  │  └──────────────────────────────────────────────────────────────┘  │
  └────────────────────────────────────────────────────────────────────┘
```

**分层约定**

| 层 | 目录 | 可见内容 | 不可做的事 |
|---|---|---|---|
| 主站 | 外部 | CAN 报文与 OD | 不直接访问库内状态 |
| 301 | `301/`（CANopenNode 核心） | OD、PDO、SDO、NMT、EMCY | 不解释 Controlword |
| 402 核心 | `402/`（纯逻辑） | 结构化输入输出，无 OD、无 CAN | 不访问 xnet、CAN 或 OD |
| OD 适配层 | `xnet_intergration/src/` | OD 与核心输入输出的映射 | 不实现状态转换规则 |
| 外部电机驱动模型 | Slave 侧（与 xnet 从站同侧） | 批量接口（OD 视图） | 不请求状态转换，只能请求故障 |

- 模型与 402 核心之间没有直接调用，只通过 OD 与批量接口交互，因此模型可以替换为 InternalModel，而不影响 402 核心。
- 402 核心依赖的是纯结构体，因此可以脱离 xnet 做单元测试（§10.1）。

#### 7.7 系统工作流程

**启动阶段**

1. 加载 JSON。若声明 `drive_402`，`model.cpp` 生成 0x6040–0x6084 对象，以及 0x2F00、0x2F01。
2. 402 核心绑定 OD（§7.3）。
3. 模型在 setup 阶段调用 `bind_batch`，创建 §7.5.3 中的各分组。若未连接模型，则启用 InternalModel。
4. 设备进入 Pre-operational 或 Operational（NMT 不影响 PDS 的故障状态，§6.4）。

**运行阶段（每个 worker 周期）**

| 步 | 执行者 | 动作 |
|---|---|---|
| 1 | 主站 | 通过 SDO 或 RPDO 写入 Controlword、模式、目标值 |
| 2 | 402 核心（worker） | 读取 0x6040 与 0x2F00。0x6040 通过轮询读取，因为 RPDO 写入直接落到 OD 存储，不经过 OD 扩展回调 |
| 3 | 402 核心 | 检查 0x2F01 心跳，判断是否超时（§7.8） |
| 4 | 402 核心 | 若 0x2F00 非零，进入故障处理（§7.8）；否则按 PDS 合法转换表推进状态（§6.1） |
| 5 | 402 核心 | 更新 0x6041、0x6061、demand，并在 `FAULT` 时写入 0x603F |
| 6 | 301 核心 | 按 TPDO 周期发送 0x6041、0x6064 等 |
| 7 | 模型 | `read_batch(view_pp/view_pv)` 读取一致快照（§7.5.7） |
| 8 | 模型 | 计算实际值，仅在 `OPERATION_ENABLED` 或 `QUICK_STOP_ACTIVE` 下写 feedback 分组，最后写 0x2F01 |
| 9 | 402 核心 | 下一周期读取实际值，更新反馈 |

**正常运动示例（PP 模式，示意，省略上电与 Switch On 的中间步骤）**

```
主站        核心(worker)                   模型
 │ CW=0x000F   │                               │
 │────────────▶│ 0x6040 → OPERATION_ENABLED    │
 │             │ demand 0x607A 已写入          │
 │             │───── read_batch(view_pp) ────▶│ 读取状态与 demand
 │             │◀──── write_batch(feedback) ───│ 写 0x6064、0x606C、0x2F01
 │◀── TPDO ────│ 0x6041 target_reached 置位    │
```

**模型与主站的分工**

- 状态转换只由主站 Controlword 发起。模型只能读取状态，并在需要时请求故障。
- 模型的写入只影响实际值和故障请求，不影响 PDS 状态。

#### 7.8 模型故障机制

**故障来源**

| 来源 | 触发条件 | 核心动作 | 0x603F 写入值 |
|---|---|---|---|
| 模型故障请求 | 0x2F00 非零 | 锁存，进入 FaultReaction，最后进入 FAULT | 0x2F00 的值 |
| 心跳超时 | 连续 N 个周期 0x2F01 未变化，或从未收到心跳 | 同上 | 0x8130（心跳错误，标准 EMCY 码） |
| 本地故障 | PDS 操作失败、Controlword 读取失败（FR-FLT-01） | 同上 | 核心内部代码 |

- 仅在 JSON 声明 `drive_402`、且外部电机驱动模型已连接时，心跳超时才生效。InternalModel 无心跳，不触发超时。
- 0x2F00 的编码规则（已确认）：UNSIGNED16，值即为写入 0x603F 的故障码，原样使用，不做转换。0 表示无请求。模型自行负责码值的取值，应与设备故障码表保持一致。

**故障处理流程**

```
 模型写 0x2F00 = code
        │
        ▼  下一周期 worker
 ┌──────────────────────────────┐
 │ 锁存 code 到 0x603F（FR-FLT-03）│
 │ 通过 301 发送 EMCY             │
 │ 状态 → FAULT_REACTION_ACTIVE  │
 │ 调用 faultReaction 回调       │  ← 可抢占 BUSY 动作（FR-DRV-03）
 └──────────────┬───────────────┘
                ▼ 反应完成
        状态 → FAULT（Statusword bit3=1）
                │
                ▼
 模型读到 FAULT 后停止写 feedback，把 0x2F00 清零（模型自行清零，核心不回写）
```

- **锁存规则**（已确认）：同一故障周期内，只锁存第一个故障码到 0x603F。已处于 FAULT 时，新的故障请求只更新 EMCY（发送携带新故障码的 EMCY），0x603F 保持首次锁存的值，直到故障复位。
- **EMCY**（已确认）：故障锁存时，由 301 核心发送 EMCY。EMCY 的错误码与 0x603F 一致。
- **超时期间**：实际值冻结（§7.5.6）。进入 FAULT 后，核心不再使用 feedback 计算 demand 的结果。
- **模型的安全输出**：进入 FAULT 后，模型应把实际值归零或保持安全值。核心无权修改模型的输出。

**故障复位**

- 只能由主站发起：在 FAULT 状态下 Controlword bit7 由 0 变 1 时，核心调用 `faultReset` 并回到 `SWITCH_ON_DISABLED`（§6.3）。
- 模型不能复位故障，也不能清除 0x603F。
- **复位拒绝**（已确认）：若 0x2F00 仍为非零，核心忽略该次复位请求，状态保持 FAULT，0x603F 不变。
- 由于复位依赖 bit7 的上升沿（§6.3），模型清零 0x2F00 后，主站必须重新产生一次 0→1 上升沿才能复位。主站若一直保持 bit7 为 1，则需先清零 bit7 再置 1。

**安全抢占**

- quickStop 与 faultReaction 可以抢占 BUSY 的普通动作（FR-DRV-03）。
- 故障处理期间，核心忽略 Controlword 中的普通运动命令，直到进入 FAULT。

---

## 8. 线程、锁与并发

### 8.1 线程模型

- CANopen 接收线程、定时线程、主循环与 402 处理之间的关系见根 `README.md` 流程图。
- 402 周期处理在定时线程或独立 worker 中执行，二者不能同时执行 `axisProcess`。

### 8.2 锁序

- canopennode-rtt 固定锁序：**生命周期锁 → OD 锁**（参考 `cia402-device-rtt.md`）。
- xnet 中公共调用先获取生命周期锁，再获取运行时互斥锁，worker 只获取互斥锁（`src/internal.hpp:164`）。402 模块加入后必须保持这一顺序。
- OD 批量调用（`xnet_canopen.h:201`、`:233`）各自独立加锁，不跨调用保持锁。
- 回调中不得获取任何锁。

### 8.3 数据竞争注意点

- 0x6040 由 SDO/RPDO 写入，由 402 线程读取：需在 OD 锁内读取快照。
- 0x6041 由 402 线程写入，由 TPDO 读取：需在 OD 锁内写入。

---

## 9. 待决问题（开发前必须确认）

1. **许可证与复用**：canopennode-rtt 的许可证是否允许复制到本仓库？（影响 §7.2）
2. **NMT Stopped 时的 PDS 策略**：转到 quickStop 还是 disableVoltage？需与 Kinco 硬件行为对齐。
3. **xnet ABI 扩展**：是否需要新增 402 状态查询 API？若需要，接受新增独立函数还是新 struct 版本？
4. **模式范围**：首期只做 PP + PV，还是 PP/PV/HM/CSP/CSV/CST 全部？
5. **运行中切换模式**：是否允许 OPERATION_ENABLED 下写 0x6060？
6. **周期与 WCET**：目标平台 CPU、周期要求、单轴最坏执行时间预算？
7. **测试硬件**：是否有 Kinco FD124 实物用于验收？还是只做仿真验收？
8. **EtherCAT 行为对照**：是否需要保证与 EtherCAT 示例 `cia402appl.c` 的 Statusword 在所有状态下逐位一致？
9. **外部电机驱动模型的控制权边界（已决定）**：模型不允许直接请求 PDS 状态转换，只允许请求故障；状态转换全部由主站 Controlword 发起（见 §7.5.2）。
10. **模型故障机制（已决定）**：0x2F00 值即为 0x603F 故障码；同一故障周期只锁存第一个码，FAULT 中的新请求只更新 EMCY；故障锁存时由 301 发送 EMCY；0x2F00 非零时拒绝复位（见 §7.8）。

---

## 10. 测试计划

### 10.1 单元测试（纯软件，无硬件）

| 测试组 | 覆盖内容 |
|---|---|
| T-PDS | 全部 PDS 合法转换；非法转换忽略；0x000F 回调顺序 |
| T-CW | Controlword 位解析；边沿复位；保留位处理 |
| T-SW | 每个 PDS 状态下 Statusword 的状态位 |
| T-MODE | 0x6060 写入校验；0x6061 回读；0x6502 与编译开关一致性 |
| T-SAFE | QuickStop、DisableVoltage 抢占 BUSY 动作 |
| T-FLT | 故障路径、故障锁存、0x603F 更新、fail-closed |
| T-RST | Reset Node / Communication 对 OD 与绑定的影响 |
| T-MULTI | 多轴 OD 基址与隔离 |

建议以 canopennode-rtt 的 `examples/cia402_multi_axis_device/` 与 `examples/demo_device/` 为参考，编写 host 侧（x86）测试，不依赖 RT-Thread。

### 10.2 集成测试（xnet）

- 使用 `xnet_intergration/tests/fake_xnet` 与 `tests/test_integration.cpp` 扩展 402 场景。
- 通过 SDO / PDO 写 0x6040，检查 0x6041 状态序列。
- 覆盖 Reset Node、Reset Communication、NMT 切换、Node Guarding 超时场景。

### 10.3 硬件验收（Kinco FD124，待确认）

- 上电 → Shutdown → Switch On → Enable Operation → Quick Stop → Fault Reset 的完整序列。
- 记录每一步 Statusword 与驱动实际状态的对应关系。

### 10.4 验证命令

本文档为需求文档，本身无需构建验证。后续实现时使用仓库现有构建与测试入口（以 `xnet_intergration/README.md` 与 `tests/` 为准），**不得新增构建工具**，除非任务明确要求。

---

## 11. 里程碑

| 阶段 | 内容 | 出口标准 |
|---|---|---|
| M0 | 决策：§9 全部待确认项 | 文档更新，决策写入 §9 |
| M1 | Pure-C 核心骨架：DriveIF、PDS 状态机、Controlword/Statusword | T-PDS、T-CW、T-SW 通过 |
| M2 | OD 绑定、多轴、0x6502 与模式开关 | T-MODE、T-MULTI 通过 |
| M3 | 故障、复位、安全抢占 | T-SAFE、T-FLT、T-RST 通过 |
| M4 | xnet 集成：JSON 配置生成 402 对象、周期调用、Reset 重绑 | 集成测试通过，ABI 无破坏 |
| M5 | 模式扩展（PP/PV 首期，其余按 §9.4） | 对应模式测试通过 |
| M6 | Kinco FD124 硬件验收 | §10.3 记录完整 |

---

## 12. 开发检查清单

- [ ] 新文件使用 C 或 C++ 与 xnet 风格一致，无堆分配（核心部分）。
- [ ] 所有回调非阻塞，不获取锁。
- [ ] Statusword 状态位与 PDS 状态一一对应。
- [ ] Fault Reset 为边沿触发。
- [ ] 0x6502 与编译开关一致。
- [ ] Reset 后重新绑定 OD 扩展。
- [ ] `xnet_canopen.h` 未破坏式修改。
- [ ] 注释仅用于需要澄清之处。
- [ ] 提交信息包含 Co-authored-by trailer。

---

## 附录 A：参考文件索引

| 主题 | 路径 |
|---|---|
| 设计边界说明 | `reference/canopennode-rtt/docs/en/cia402.md` |
| Device 核心契约 | `reference/canopennode-rtt/docs/en/cia402-device-core.md` |
| RT-Thread 集成 | `reference/canopennode-rtt/docs/en/cia402-device-rtt.md` |
| 控制器角色 | `reference/canopennode-rtt/docs/en/cia402-controller.md` |
| 状态与对象常量 | `reference/canopennode-rtt/profile/cia402/common/CO_402_state.h`、`CO_402_objects.h`、`CO_402_mode.h`、`CO_402_defs.h` |
| DriveIF | `reference/canopennode-rtt/profile/cia402/device/CO_402_drive.h` |
| 状态机绑定 | `reference/canopennode-rtt/profile/cia402/device/CO_402_device_fsa.h` |
| EtherCAT 参考状态机 | `reference/EtherCAT_Slave/src/cia402appl.c`（`CiA402_StateMachine` 317 行起） |
| EtherCAT 掩码与常量 | `reference/EtherCAT_Slave/src/cia402appl.h`（47–140 行） |
| xnet 公共 API | `xnet_intergration/include/xnet_canopen.h` |
| xnet 模型层 | `xnet_intergration/src/model.cpp`（280、427、494 行附近） |
| xnet 设备层 | `xnet_intergration/src/device.cpp`、`driver.cpp` |
| xnet 测试 | `xnet_intergration/tests/`、`tests/app/README.md` |
| Kinco FD124 | `xnet_intergration/eds/Kinco_FD_20250417_V1.2.eds`、`examples/kinco_FD124.json` |
