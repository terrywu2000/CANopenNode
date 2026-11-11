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
3. 支持 PP、PV、HM、CSP、CSV、CST 模式，编译期可裁剪；力矩模型可选扩展 PT，首期模式范围见 §9。
4. 驱动动作通过回调（DriveIF）抽象，核心模块不包含任何硬件或 CAN 代码。
5. 核心为 Pure C，无堆分配，回调非阻塞，可在 RT 环境运行。
6. 与 xnet 的 OD 访问、批量 API、复位语义（Reset Node / Reset Communication）保持一致。

### 1.3 非目标

- 不实现厂商私有对象（如 Kinco 的 0x2FF0 系列），仅预留扩展点。
- 不实现通用运动规划器；PP/PV 及可选 PT 的 profile/斜坡处理统一由核心的模式处理模块负责，模型只实现驱动跟踪、限幅和动态响应，不重复生成轨迹。
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
| FR-OD-05 | 主站独占写入 target 与 profile 参数，核心独占发布 demand、状态和已接受实际值；模型只写本地交换对象 | 覆盖命令、反馈的网络/本地权限；核心不得覆盖 0x607A、0x60FF、0x6071 |

**实现说明**：`canopennode-rtt` 的 OD 扩展通过 `CO_402_device_bindOD()` 绑定，并在 Communication Reset 时重新绑定（见 `docs/en/cia402-device-rtt.md`）。xnet 中对应的是 `Model` 的 `resolve_node` / `configure` 流程（`xnet_intergration/src/model.cpp:427`、`:494`），绑定点应放在同一流程里。

### 4.2 PDS 状态机

| ID | 需求 | 验收标准 |
|---|---|---|
| FR-PDS-01 | 按 CiA 402 状态转换表实现 PDS，普通转换由 Controlword 发起，内部就绪、故障及驱动完成条件由核心处理 | 单元测试覆盖全部合法转换（至少 1–15 号转换）、异步完成及非法转换忽略 |
| FR-PDS-02 | Statusword 由 PDS 状态派生，bit0-3、5、6 与状态一一对应 | 每个状态下 Statusword 的状态掩码 `0x006F` 与参考值一致（见 `cia402appl.h:90-97`） |
| FR-PDS-03 | 上电后必须处于 `NOT_READY_TO_SWITCH_ON`，经内部自检后自动进入 `SWITCH_ON_DISABLED` | 上电后 Statusword 状态位为 0x0040 |
| FR-PDS-04 | Controlword 0x000F 视为 Switch On + Enable Operation 的单条命令，但仍按状态逐步处理，不得跳过 `SWITCHED_ON` 的驱动回调 | 由 `READY_TO_SWITCH_ON` 直接写 0x000F，回调顺序为 switchOn → enableOperation |
| FR-PDS-05 | 驱动动作必须非阻塞，返回 BUSY / DONE / ERROR，核心据此推进 | 回调 BUSY 时状态保持，DONE 后提交新状态 |
| FR-PDS-06 | Shutdown、Disable Voltage、Quick Stop 必须按 CiA 402 规定的优先级生效 | Quick Stop 可抢占 BUSY 的 enableOperation；Disable Voltage 可抢占任何 BUSY |
| FR-PDS-07 | Quick Stop Active 的完成与退出同时考虑 0x605A、有效反馈和 Controlword，不得仅以 bit2 恢复判定重新使能 | 覆盖支持的选项码、减速未完成、命令撤销与重新使能路径 |

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
| FR-MODE-01 | 模式通过编译开关 `CO_402_CONFIG_MODE_{PP,PV,PT,HM,CSP,CSV,CST}` 启用，默认全部关闭；PT 为可选扩展 |
| FR-MODE-02 | 模式值：PP=1、PV=3、PT=4、HM=6、CSP=8、CSV=9、CST=10；PT 与 CST 不可互换 |
| FR-MODE-03 | 0x6502 位映射：PP=bit0、PV=bit2、PT=bit3、HM=bit5、CSP=bit7、CSV=bit8、CST=bit9；只公布编译启用、OD 完整且模型声明支持的交集 |
| FR-MODE-04 | 写 0x6060 为不支持的模式时拒绝写入（SDO Abort 或返回错误），0x6061 保持原值 |
| FR-MODE-05 | 模式切换仅在 `OPERATION_ENABLED` 之外或经过 `modeExit` 回调完成后生效 **[待确认]**：是否允许运行中切换 |
| FR-MODE-06 | 模式接口（modeEnter / modeExit / profilePosition / profileVelocity / homing）为 **追加式** 扩展，不破坏已有 DriveIF ABI |
| FR-MODE-07 | 0x6061 只公布模型已确认就绪的模式，不直接回显 0x6060；待确认时禁止应用新模式的非零 demand，拒绝或超时须诊断 |

### 4.5 DriveIF（驱动回调）

| ID | 需求 |
|---|---|
| FR-DRV-01 | 必须提供以下回调：`shutdown`、`switchOn`、`enableOperation`、`disableOperation`、`quickStop`、`faultReaction`、`faultReset`、`disableVoltage`；动作请求及结果枚举与之对应并在协议冻结时明确 |
| FR-DRV-02 | 回调返回值为 `CO_402_drive_result_t`（BUSY/DONE/ERROR） |
| FR-DRV-03 | 安全相关回调（quickStop、disableVoltage、faultReaction）可以抢占 BUSY 的普通动作；同步撤销旧动作的逻辑输出权限，异步模型按新 action_seq 执行安全反应；不承诺跨线程同步撤销物理效果（§7.5.4） |
| FR-DRV-04 | 所有回调禁止阻塞、禁止调用 CAN/OD 访问函数、禁止在回调中获取 OD 锁 |
| FR-DRV-05 | 外部模型通过带代次和动作序号的反馈确认 BUSY/DONE/ERROR；回调只发布动作或检查缓存响应，不执行模型计算、不等待模型线程 |

### 4.6 故障与复位

| ID | 需求 |
|---|---|
| FR-FLT-01 | 任何内部故障（PDS 操作失败、Controlword 读取失败、产品故障）都进入 `FAULT_REACTION_ACTIVE`，再进入 `FAULT` |
| FR-FLT-02 | 故障源必须被锁存并通过诊断接口（DiagIF）暴露，缺失映射时仍按失败关闭（fail-closed） |
| FR-FLT-03 | 0x603F 反映当前故障周期首次锁存的故障代码，直到成功故障复位；后续故障独立报告 EMCY |
| FR-FLT-04 | Reset Node 恢复普通应用 OD 默认值；Reset Communication 保留普通应用 OD 并重新绑定扩展；两者均使模型交换会话失效，更新 epoch，不复用旧动作/反馈（§7.5.7） |
| FR-FLT-05 | Reset Communication 期间，若驱动处于 OPERATION_ENABLED，必须先经过 quickStop 或 faultReaction，不得直接丢弃 |
| FR-FLT-06 | 区分当前故障条件、锁存记录与复位许可；仅在条件清除、模型健康且有匹配完成响应后允许复位，FAULT 中仍持续反馈与心跳 |

### 4.7 运行与集成

| ID | 需求 |
|---|---|
| FR-RUN-01 | 提供周期入口 `CO_402_device_axisProcess()`，建议周期 1–10 ms，由调用方决定 |
| FR-RUN-02 | 可移植核心由宿主单线程周期调用；xnet 中固定使用已有 device worker 处理 402，外部模型在应用独立线程执行，二者不得同时调用 axisProcess |
| FR-RUN-03 | 诊断与日志通过 `CO_402_LOG_*` 宏输出，允许通过 `CO_402_LOG_CUSTOM_HEADER` 覆盖 |
| FR-RUN-04 | Demo 模式只做软件仿真，不得驱动硬件 |
| FR-RUN-05 | xnet 首期采用同进程、独立模型线程；计算在库锁外，短时批量交换在锁内；核心只消费完整提交且代次有效的反馈 |
| FR-RUN-06 | 心跳新鲜度和反馈新鲜度独立监测，以本机单调时钟的实际经过时间判超时；不得用名义 worker 周期数替代时间 |

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
| NFR-08 | 分别记录模型周期/WCET、批量 API 持锁时间、worker 抖动、反馈年龄与命令到反馈/TPDO 延迟；不以轴处理耗时替代端到端验收 | 按 §8.4 在目标平台测试，预算见 §9.6 |

---

## 6. 行为规范（实现约束）

### 6.1 Controlword 处理顺序

1. 在 worker 锁内读取 Controlword、模式和 target 快照，并消费通过代次校验的完整模型反馈。
2. 若读取失败，调用 `CO_402_device_axisControlwordReadFailed()`，进入 FAULT 路径。
3. 判定 Quick Stop / Disable Voltage 等安全命令是否优先。
4. 按当前 PDS 状态处理普通命令。
5. 更新 Statusword，写入 0x6041。

### 6.2 Statusword 派生

Statusword 的状态位由 PDS 当前状态决定，模式相关位由模式模块根据已接受反馈维护；模型不直接写 Statusword。bit10 的容差/持续时间、bit11 的限幅来源及各模式 bit12/13 的判据须写入模式配置和测试。实际反馈无效或过期时，不得报告新的 target reached。Statusword 与实际值应在 OD 锁内发布，避免 TPDO 读到未提交反馈。

### 6.3 Fault Reset

- 只在 FAULT 状态且 bit7 上升沿时调用 `faultReset`。
- 连续保持 bit7=1 不得重复触发。
- 当前故障条件清除、模型心跳和完整反馈均有效、模型允许复位后，才可启动 faultReset 动作；匹配的 DONE 返回后回到 `SWITCH_ON_DISABLED`。
- 条件不满足则拒绝本次请求并诊断，不排队保留边沿。主站须在条件满足后重新发送可被采样的 0→1 脉冲；FAULT_REACTION_ACTIVE 中的提前边沿不得延迟执行。

### 6.4 NMT 与 PDS 的关系

- NMT 进入 Stopped 时，应先将 PDS 转到安全状态（经 quickStop 或 disableVoltage，具体见 **[待确认]** §9.2）。
- NMT 重新进入 Operational 不得自动恢复 `OPERATION_ENABLED`，必须由主站重新发送命令序列。

---

## 7. 推荐架构

### 7.1 分层

```
CANopen 主站 → CANopenNode / OD
                        │
        xnet_intergration/src/ OD 适配层
                        │ 结构化输入输出
                 402/ 独立逻辑核心
                        │ 动作、模式、demand / 已提交反馈
        xnet_intergration/src/ 模型交换适配层
                        │ 现有批量 C API
        Slave 驱动模型（同进程、独立线程，锁外计算）
```

### 7.2 复用策略 **[待确认 — 需负责人决策]**

| 方案 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| A. 直接引入 | 把 `reference/canopennode-rtt/profile/cia402/{common,device}` 的独立逻辑移植到 `402/`，OD/平台代码留在适配层 | 可复用现有行为与测试 | 需确认许可证、去除 RT-Thread 依赖，并验证锁与异步模型契约 |
| B. 仅移植行为 | 以 canopennode-rtt 为行为规范，重新实现 Pure-C 核心 | 许可与依赖清晰 | 工作量大，需重新验证 |
| C. 只在模型层实现 | 在 `model.cpp` 中实现 402 解释器 | 改动小 | 违背 xnet 通信边界声明；难以复用到其他平台 |

**推荐方案 A（若许可证允许）或 B**。不推荐 C，因为它直接违背 `xnet_canopen.h` 的声明，并使模型层承担状态机逻辑。

### 7.3 与 xnet 集成点

1. **OD 描述**：在 `model.cpp` 的 `load()` 中，当 JSON 配置中声明 `drive_402` 段时，按启用模式生成 §4.1 标准对象及 §7.5 本地交换对象；配置不完整则拒绝启动。
2. **绑定**：在 `Model::configure()`（`model.cpp:494`）或 `device.cpp` 中，于 OD 初始化后调用 `CO_402_device_bindOD()`。
3. **周期调用**：402 只在现有 device worker 中处理，插入 RPDO 处理之后、TPDO 处理之前；另设应用拥有的模型线程，不在 worker 内执行模型计算。
4. **Reset**：`xnet_canopen_device::reset_stack()`（`src/device.cpp:230`）在 `model->restore()`（`:243`）之后完成复位。402 重新绑定应放在该流程中，并在成功完成后才发布计数（参考 `device.cpp:247-249` 的注释）。
5. **状态快照**：402 与模型诊断通过只读 OD 对象暴露，不新增 C 函数，不修改已有 40 字节 `xnet_canopen_status_t`；复位代次随模型命令快照发布。

### 7.4 DriveIF 对 Kinco FD124 的适配 **[待确认]**

- 需确认 FD124 的 0x6040/0x6041 行为是否与标准一致（尤其 bit4、bit5、bit8 的含义）。
- 需确认 FD124 EDS 中是否声明 0x6502、0x603F，以及它们的 PDO 映射属性。
- 需确认 FD124 的 quickStop 选项码（0x605A）与 shutdown 选项码（0x605B）默认值。

---

### 7.5 外部电机驱动模型接口

> 本节假定已有 **Slave 侧驱动行为模型**（下文简称“模型”），负责跟踪、动态响应、限幅及故障条件，可在其内部组合电机和负载模型。借鉴 Beckhoff Torque Controller 的强类型端口、固定步长、模式门控与诊断方式，不直接接入其主站控制逻辑。

**结论：保留现有 C ABI，新增本地交换对象和 OD 扩展。** 模型通过 `bind_batch`、`read_batch`、`write_batch` 交互。PDS 状态机与 Statusword 只有库内一份；已有模型若包含 PDS，应拆出或关闭该部分。以下为待实现协议，不表示现有库已支持。

#### 7.5.1 职责分工

| 角色 | 负责的 OD | 访问通道 |
|---|---|---|
| 主站 | 0x6040、0x6060、target（0x607A、0x60FF、可选 0x6071）及 profile 参数（如 0x6081、0x6083、0x6084） | SDO / RPDO，按设备配置开放 |
| 402 核心 | 0x6041、0x6061、0x603F；profile/斜坡后的 demand；经验证的实际值 | worker 内发布，绝不覆盖主站 target |
| 模型交换适配层 | 动作/模式请求、命令代次、暂存与完整帧校验 | OD 扩展只做有界校验和复制，不推进 PDS |
| 模型 | 实际位置/速度/力矩、动作结果、模式就绪、限幅、当前故障条件与复位许可 | 通过本地完整反馈帧提交，不直接写标准实际值或 Statusword |

首期 profile/斜坡生成归核心，模型不得重复生成。CSP/CSV/CST 的周期目标按模式规则直接形成 demand。后续若要由模型负责 profile，必须另行修订该边界，不得同时启用两套轨迹处理。

#### 7.5.2 本地交换对象（厂商自定义区 0x2000–0x5FFF）

| 索引 | 名称 | 类型 | 写入方 | 语义 |
|---|---|---|---|---|
| 0x2F00 | 当前模型故障 | UNSIGNED16 | 核心发布 | 来自最新已接受反馈帧，0 表示故障条件已消失，不是锁存记录 |
| 0x2F01 | 模型心跳 | UNSIGNED32 | 模型 | 独立于反馈提交；模型存活时持续递增 |
| 0x2F02 | 命令快照 | 固定长度 OCTET_STRING | 核心 | 带版本、epoch、command_seq、action_seq、动作、模式请求、已接受模式、有效输出使能、demand 和参数 |
| 0x2F03 | 完整反馈提交 | 固定长度 OCTET_STRING | 模型 | 带版本、epoch、feedback_seq、源 command_seq、action_seq、动作结果、模式响应、实际值、有效掩码、限幅、当前故障码、reset_permitted |
| 0x2F04 | 处理确认 | UNSIGNED32 | 核心 | 最后被 worker 完成处理的 feedback_seq，0 表示本代次尚无结果；须同时检查 0x2F05 |
| 0x2F05 | 处理结果 | UNSIGNED16 | 核心 | 0=无结果、1=已接受、2=命令过期、3=内容无效；epoch 失效通过新命令快照通知 |

- 上述索引均为每轴 ARRAY，subindex `n+1` 对应轴 n，subindex 0 为只读元素数；不套用标准 0x6000 区的轴偏移。轴数须在 subindex 范围内并与设备配置一致。
- command_view 中的标准 402 对象仍按 FR-OD-02 加轴偏移；交换对象按上述 subindex 选择轴，禁止混用两种寻址规则。
- 0x2F01、0x2F03 为 SDO 只读、`local_write: true`，禁止 RPDO 写入；其余为核心只读发布、`local_write: false`。启动时验证权限及索引冲突。
- 旧草案中的模型直接写 0x2F00、标准实际值以及“心跳最后写”方案被本协议替代；不改变现有通用批量 API 的逐条尝试语义。
- 快照和反馈帧采用显式小端字段编码、固定长度、无隐式填充，不直接 memcpy 编译器结构体。实现前在接口头文件冻结版本 1 的逐字节偏移、长度、枚举、保留位和有效掩码；未知版本、错误长度、非法枚举/保留位须返回错误并诊断，不得截断或默认成功。
- 位置为 I32 counts，速度为 I32 counts/s，力矩为 I16 额定力矩千分比；U16 故障码原样对应设备故障表。额定力矩、正方向、转换比例、模式限幅、容差和持续时间由 JSON 明确，初始化时拒绝缺失或不一致配置。模型内部可用浮点，适配层负责有界转换，溢出不得静默回绕。
- 首期 PP/PV 必须提供位置和速度；PT/CST 启用时必须提供力矩，且配置对应的斜坡/限幅参数与 bit10/11 判据。仅支持 PP/PV 的模型不要求 PT；PT=4 与 CST=10 分别声明能力。
- 模型故障经完整反馈进入核心，不使用 `xnet_canopen_set_fault` 代替 PDS 故障路径。心跳计数仅指示更新，超时使用 worker 本机单调时钟，不依赖模型墙钟。

#### 7.5.3 批量分组（模型侧绑定，启动时完成）

模型在 setup 阶段按轴一次性绑定分组。首期统一命令/反馈帧，通过版本与有效掩码扩展字段，避免先读模式再跨调用读取不同模式 demand。

| 分组 | 方向 | 内容 | 状态 |
|---|---|---|---|
| `command_view` | 读 | 本轴 0x2F02、0x6040、0x6041、0x6061、0x603F、0x2F04、0x2F05 | 首期 |
| `feedback_submit` | 写 | 本轴 0x2F03，单一条目、完整帧 | 首期 |
| `model_alive` | 写 | 本轴 0x2F01，独立条目 | 首期 |

- 模型每次检查调用返回值及每条状态，只使用全部成功的快照。绑定返回的类型和长度必须与帧契约一致。
- 新增模式须补充核心模式处理、模型能力、OD、参数及测试，不假定只增加模式表一行即可完成。
- 不支持的模式按 FR-MODE-04 拒绝，0x6061 保持先前已接受值；初始无模式时为 0。模型仍更新健康和实际反馈，但不得施加未接受模式的输出。

#### 7.5.4 运行阶段流程

1. worker 在 RPDO 后消费已提交反馈，检查心跳和反馈年龄，处理安全命令，再推进 PDS、模式和 profile，发布下一命令快照及 TPDO 数据。
2. 模型线程一次 `read_batch(command_view)` 取快照，释放 API 内部锁后计算；在待机、QuickStop、FaultReactionActive、FAULT 中也持续执行和报告实际值。
3. 模型一次写入完整 `feedback_submit`，逐项检查结果；成功只表示帧已校验并进入待消费缓存，不表示 PDS 已转换。由 0x2F04/0x2F05 判断处理确认与结果；拒绝时诊断并读取新命令后重新计算，不无限等待已被拒绝帧。
4. `model_alive` 单独更新。独立反馈超时可检测“心跳仍更新但计算无有效结果”，不得用心跳代替计算进展。

**动作与模式握手**：核心先发布动作请求而不是先提交目标 PDS 状态。`action_seq` 对每个新动作或抢占递增；模型以同 epoch、同 action_seq 返回 BUSY/DONE/ERROR。模型计算只执行有效命令，不自行重解原始 Controlword。模式请求也使用新的 action_seq；只有匹配的模式就绪响应才能更新 0x6061。模式未就绪不得使用新模式的非零 demand。匹配动作的旧响应或被抢占动作的延迟 DONE 均不得生效。

安全命令使核心立即撤销旧动作的逻辑输出权限，并发布优先安全动作；模型每次施加输出前检查最新动作/代次，发现抢占则丢弃旧计算。PDS 的完成仍等待匹配反馈。异步交换只能提供有界响应，不能承诺同步撤销物理效果；不得用于真实硬件安全保证。`action_timeout_us` 到期须锁存故障并切入失败关闭策略；若安全反应本身无反馈，则撤销输出许可、保留故障诊断并按已定义的无反馈反应进入 FAULT，不伪造 DONE。

#### 7.5.5 所有权与权限

- **Target / profile 参数**：只由主站写，模型与核心只读；demand 使用命令帧中的独立字段。
- **Statusword、mode_display、0x603F**：只由核心按状态规则发布，包括模型无反馈时的故障路径。**标准实际值、0x2F00**：只由核心从已接受反馈发布；无新反馈时保持并标记年龄。
- **模型本地写**：仅 0x2F01、0x2F03，网络不能写；应用保证每轴只有一个模型写入者。
- OD 扩展回调只校验/复制完整反馈，不运行状态机、不产生 EMCY、不获取额外锁；状态、故障和 TPDO 由 worker 执行。

#### 7.5.6 超时与缺省模型

- **配置**：仅 `drive_402` 启用 402；其中显式选择 internal 或 external 模型、支持模式、周期及超时。未声明时保持现有通信行为。
- **启动握手**：external 模式在 `startup_timeout_us` 内必须收到当前 epoch 的有效反馈及心跳；此前禁止使能，不能静默启用 InternalModel。到期未就绪须诊断并故障。
- **运行超时**：分别以 `heartbeat_timeout_us`、`feedback_timeout_us`、`action_timeout_us` 监测心跳变化、完整反馈消费及动作完成。使用 worker 本机单调时钟的实际经过时间，不以名义周期数换算；模型故障条件仍在时持续报告。
- **失联**：已连接 external 模型超时后失败关闭，不自动切到 InternalModel。对外实际值保留最后有效值但标记陈旧，不能当作安全状态或新的 target reached。
- **InternalModel**：仅显式选择 internal 时启用，理想跟随 demand，并按相同动作/模式契约给出就绪响应；它不模拟真实动态，不需外部心跳。

#### 7.5.7 批量接口的已知约束与应对

| 约束（来自 `xnet_canopen.h`） | 对本设计的影响 | 应对 |
|---|---|---|
| 单次调用内加锁，跨调用不保持锁 | 锁可保证不被 worker 插入，但不能证明数据年龄 | command_view 含 worker 发布的 command_seq 与 epoch；诊断 get_status 不视作同一原子快照 |
| `write_batch` 逐条尝试、无回滚 | 后面的心跳可能成功，前面的实际值却失败 | 用单条目完整帧扩展，长度/内容全部校验后才提交缓存；旧有效反馈不被失败写入污染 |
| 批量写不推进状态机、不请求 TPDO | 提交与消费分开 | worker 消费后统一发布实际值与状态 |
| 绑定不校验后续权限 | 配置错误可能到运行时才暴露 | 启动校验全部权限、类型、大小；调用时检查每条结果，错误显式诊断 |

**完整帧提交规则**

1. 每轴一个模型写入者，最多一个未确认反馈帧。0x2F03 的 OD 写扩展在现有 runtime 锁内先校验整个帧，再一次性复制为 pending；禁止分段提交、禁止先覆盖已接受标准实际值。pending 未处理时拒绝新提交并诊断；模型仍持续本地步进和心跳，收到确认后才提交下一帧，不能累积无界队列。相同序号的重试只允许完全相同的帧，且不得刷新反馈新鲜度。
2. 校验版本、固定长度、当前 epoch、必需有效字段及源命令年龄；动作结果必须关联已发布 action_seq。反馈允许来自有界历史 command_seq，而非强求等于最新 worker 周期；超过 `command_max_age_us` 则拒绝，防止异步模型永远追不上快照。
3. worker 再校验 epoch、命令年龄和动作相关性，合法时在同一锁内发布完整实际值及模式反馈，更新处理确认及结果；过期帧丢弃并发布拒绝结果，不改变 PDS、故障条件或反馈年龄。旧代次通过更换 epoch 清空确认，使模型放弃等待并重新握手。抢占前帧可保留有效实际测量，但旧动作完成响应不得提交状态。
4. epoch 为句柄内 U64 单调代次，在 initialize、协议复位、stop/start 的模型会话失效点更换，并清除 pending、动作响应及握手状态；Reset Communication 仍保留普通应用 OD，交换会话数据例外。复位/停止开始即撤销旧代次，任何跨越该点的计算结果均拒绝。单次安全停止的完成握手使用新代次；重启需重新握手，不自动恢复 OperationEnabled。
5. command_seq、action_seq、feedback_seq 为 U32，0 保留为无值，按模序比较且有效窗口小于半范围；回绕行为须测试。epoch 不允许回绕复用，耗尽则拒绝启动并诊断。uninitialize 后批量 ID 失效，销毁前模型线程必须退出；新句柄不得接收旧线程反馈。

#### 7.5.8 与当前核心实现的差距（待开发）

`402/drive402.hpp` 当前的 `ExternalModel` 是纯 C++ 接口，不关心 OD，与本设计有以下差距：

1. `Feedback` 需扩展力矩、有效性、模式就绪、限幅、持续故障、复位许可及动作响应。
2. 建立有效动作/模式请求与 demand 输出，不能只向模型提供最终 PDS 状态。
3. 将存活、反馈及动作超时分开；心跳计数判断更新，本机单调时间判断超时。
4. 增加完整帧 OD 扩展、pending/消费缓存、代次与序号校验、初始化权限检查。

开发方案：`402/` 核心保持不依赖 OD 的纯逻辑，新增一个 OD 适配层（位于 `xnet_intergration/src/`），负责把 OD 快照映射为核心的输入，并把核心输出写回 OD。

#### 7.5.9 对外部接口文档的影响

`xnet_canopen.h` 文件头声明"No function interprets a CiA 402 controlword"。库内运行 402 核心后，这句话需要修订为：模型不解释 Controlword，库内 402 核心按 JSON 声明启用后解释它。开发时需同步修改头文件注释。

同时补充单次批量调用的锁语义、跨调用不原子及无数据年龄保证；不能把一次 read_batch 宣称为“所有字段必定同一产生周期”。帧协议的版本、布局、提交/消费、权限、代次和错误语义须在接口头文件及 xnet 中英文 README 中同步说明。保留通用 write_batch 无回滚语义，不修改其行为来实现事务。

#### 7.5.10 外部电机驱动模型的 Slave 侧整合评估

外部电机驱动模型与 xnet 从站位于同一 Slave 侧，通过本地批量 OD 接口交互，不经过 CAN 总线。评估结论与待确认项如下：

| 评估项 | 结论 | 依据 / 处理 |
|---|---|---|
| 数据通道 | 同进程本地 C API，不产生额外模型 CAN 报文 | 延迟取决于两个线程的相位、计算与锁等待，不保证一周期反馈或两周期往返 |
| 故障检测边界 | worker 存活且可取得锁时，能检测模型无锁挂起和无有效反馈 | 同进程崩溃可终止整个服务；持锁挂起可阻止超时检测，不宣称进程级故障隔离 |
| 锁与抖动 | 短时交换、锁外计算 | 回调只校验/复制固定大小帧，实测 WCET，不执行文件 I/O 或模型步进 |
| 访问与完整性 | 仅心跳、完整帧允许本地模型写 | 网络写交换对象、直接写标准实际值均被拒绝；原子性来自单帧 OD 扩展，不来自 write_batch |
| 复位与生命周期 | 批量 ID 可保留，模型会话必须失效 | uninitialize 后重新绑定；Reset/stop/start 更新 epoch 并重新握手 |
| ABI | 不新增 C 函数，不改变已有结构及通用批量语义 | 必须开发帧扩展、适配器、核心响应接口并更新说明，绝非只改头注释 |
| 宿主进程 / 线程 | 首期同进程、独立模型线程 | 若要求崩溃隔离，应另立 IPC、独立看门狗与进程恢复需求，不在本期隐含承诺 |
| 驱动响应 | 模型不拥有 PDS，但提供动作完成、模式就绪和实际动态 | 核心保留状态生成权，避免双状态机及“先转换再等待完成”的循环依赖 |

**结论**：按上述契约实现适配层和核心扩展后可集成 Slave 驱动模型；当前库不因此自动具备这些能力。先使用理想跟随且可注入延迟、限幅和故障的模型验收，再替换为复杂模型。

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
  │  Slave 侧（首期同进程、两个执行线程）                               │
  │                                                                    │
  │  ┌──────────────────────────────────────────────────────────────┐  │
  │  │ xnet 设备（CANopen 从站，每设备一个 device worker 线程）        │  │
  │  │  301/  CANopenNode 核心：NMT、SDO、PDO、EMCY、对象字典 (OD)     │  │
  │  │  xnet_intergration/src/  OD 适配层（只做映射，不含状态机）      │  │
  │  │  402/  DS 402 核心（纯逻辑，与 xnet、CAN 端口无关）             │  │
  │  └──────────────────────────────────────────────────────────────┘  │
  │          ▲ 批量 OD 接口：bind_batch / read_batch / write_batch      │
  │          │ 模型读有效动作/模式/demand；提交完整反馈帧与独立心跳     │
  │  ┌───────┴──────────────────────────────────────────────────────┐  │
  │  │ 外部电机驱动模型（Slave 侧应用模型，可选）                     │  │
  │  │  · 独立线程、库锁外计算，不重复实现 PDS 或 profile             │  │
  │  │  · 反馈实际值、动作结果、模式就绪、限幅及持续故障             │  │
  │  │  · InternalModel 仅显式选择，external 失联不得自动回退        │  │
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
| 外部电机驱动模型 | Slave 侧（与 xnet 从站同侧） | 有效动作、模式、demand 及批量接口 | 不生成 PDS/Statusword，不重复 profile；只返回动作响应、实际值与故障条件 |

- 模型与 402 核心之间没有直接调用，只通过 OD 与批量接口交互，因此模型可以替换为 InternalModel，而不影响 402 核心。
- 402 核心依赖的是纯结构体，因此可以脱离 xnet 做单元测试（§10.1）。

#### 7.7 系统工作流程

**启动阶段**

1. 加载 `drive_402`，按模式生成标准对象及每轴交换对象，校验模式交集、类型、单位、权限和全部超时配置。
2. 绑定 OD 扩展、初始化核心和新 epoch，默认禁止输出；根据配置显式选择 internal 或 external。
3. 应用 setup 绑定分组、初始化模型，启动后以当前 epoch 反馈与心跳完成握手；external 缺失不得回退。
4. NMT 与 PDS 独立管理；即便 NMT Operational，也须完成模型就绪及主站使能序列。

**运行阶段（两个线程异步执行，不保证表内步骤同周期完成）**

| 步 | 执行者 | 动作 |
|---|---|---|
| 1 | 主站 | 通过 SDO 或 RPDO 写入 Controlword、模式、目标值 |
| 2 | worker | 接收与 RPDO 完成后快照命令，消费合法 pending 帧；过期/旧代次帧拒绝并诊断 |
| 3 | 402 核心 | 以实际经过时间检查心跳、反馈和动作超时，处理持续故障及安全抢占 |
| 4 | 402 核心 | 根据合法命令与匹配动作结果推进 PDS、模式和 profile；不覆盖 target |
| 5 | worker | 发布下一有效动作/模式/demand 快照、状态与已接受实际值，更新处理确认及结果 |
| 6 | 301 核心 | TPDO 处理和发送；402 必须位于 RPDO 与 TPDO 处理之间 |
| 7 | 模型线程 | 读取 command_view，在库锁外按固定步长计算；有未确认帧时先检查处理结果，不阻止本地步进 |
| 8 | 模型线程 | 在所有 PDS 状态持续反馈；施加输出前检查动作/代次，提交完整帧，独立更新心跳 |
| 9 | worker | 后续周期消费反馈；只有满足模式判据且反馈有效才更新 target reached 等状态 |

**正常运动示例（PP 模式，示意，省略上电与 Switch On 的中间步骤）**

```
主站        核心(worker)                   模型
 │ CW=0x000F   │                               │
 │────────────▶│ 发布 enableOperation 动作     │
 │             │──── command_view ────────────▶│ 库锁外计算
 │             │◀── feedback_submit ───────────│ 匹配动作 DONE
 │             │ 消费后进入 OPERATION_ENABLED │
 │             │──── command_view ────────────▶│ 跟踪独立 demand
 │             │◀── feedback_submit ───────────│ 完整实际值与模式响应
 │◀── TPDO ────│ 按有效反馈与容差更新状态       │
```

**模型与主站的分工**

- 普通状态转换由主站发起，核心根据命令、内部条件、故障和匹配动作响应决定转换；模型不能直接指定 PDS 状态。
- 模型响应会影响转换的完成时机和模式就绪，但不拥有状态转换规则或 Statusword 写权限。

#### 7.8 模型故障机制

**故障来源**

| 来源 | 触发条件 | 核心动作 | 0x603F 写入值 |
|---|---|---|---|
| 持续模型故障 | 已接受反馈中的 fault_code 非零，发布到 0x2F00 | 锁存，执行 FaultReaction，完成或无反馈失败关闭后进入 FAULT | 0x2F00 的值 |
| 心跳超时 | 运行时心跳实际年龄超限，或启动握手到期 | 同上 | 0x8130，并在诊断载荷区分本地模型超时与网络心跳 |
| 完整反馈/动作超时 | 心跳可正常但无完整反馈或动作未完成 | 同上 | 按设备故障表配置的非零模型错误码，启动时校验 |
| 本地故障 | PDS 操作失败、Controlword 读取失败（FR-FLT-01） | 同上 | 核心内部代码 |

- external 从启动握手开始即受超时管理，握手完成后持续监测；InternalModel 不触发外部存活超时。
- 0x2F00 为当前条件，不是锁存记录；模型只在对应条件真正消失后，在完整帧中报告 0。故障源仍存在时不得因读到 FAULT 就清零；事件型故障须在模型中保持至收到已接受确认（0x2F05=1），拒绝结果不算已报告。

**故障处理流程**

```
 模型提交完整帧 fault_code = code
        │
        ▼  下一周期 worker
 ┌──────────────────────────────┐
 │ 发布 0x2F00，首次锁存到 0x603F │
 │ 通过 301 发送 EMCY             │
 │ 状态 → FAULT_REACTION_ACTIVE  │
 │ 调用 faultReaction 回调       │  ← 可抢占 BUSY 动作（FR-DRV-03）
 └──────────────┬───────────────┘
                ▼ 反应完成
        状态 → FAULT（Statusword bit3=1）
                │
                ▼
 模型持续反馈和心跳；故障条件消失后报告 fault_code=0 与复位许可
```

- **锁存规则**：同一故障周期首次码保留到成功复位；FAULT 中后续新故障报告 EMCY，不改首次锁存码。相同持续条件不逐周期重复发 EMCY，变化/重新出现时才报告。
- **EMCY**：首次故障码与 0x603F 相同；后续故障 EMCY 可与首次锁存码不同，须保留当前条件和诊断历史。
- **超时期间**：标准实际值保留最后有效值并标记陈旧，不是“电机已停”。核心撤销运动许可，不使用陈旧反馈推断动作完成；诊断与无反馈故障反应按 §7.5.4 执行。
- **模型安全响应**：停止施加普通驱动力或执行规定减速，继续计算故障反应及惯性响应；实际位置/速度不因 FAULT 被强制清零。核心负责命令许可，模型负责动态响应，均不提供真实硬件安全保证。

**故障复位**

- 只能由主站在 FAULT 下以 bit7 上升沿发起；通过 §6.3 健康与许可检查后发布 faultReset，匹配 DONE 后才清除 0x603F 并回到 `SWITCH_ON_DISABLED`。
- 模型不能复位故障，也不能清除 0x603F。
- **复位拒绝**：0x2F00 非零、反馈/心跳过期、模型未允许复位或仍有核心故障条件时均拒绝，诊断原因且保留 FAULT 和锁存码。
- 条件清除不自动复位；主站必须重新产生可被采样的 0→1。参考 Beckhoff 的主站控制器若持续保持 bit7=1，应在主站适配中改成受控脉冲，不改变从站边沿规则。

**安全抢占**

- quickStop 与 faultReaction 可以抢占 BUSY 的普通动作（FR-DRV-03）。
- 故障处理期间，核心忽略 Controlword 中的普通运动命令，直到进入 FAULT。

---

## 8. 线程、锁与并发

### 8.1 线程模型

- 可移植核心由宿主保证单线程执行；xnet 首期由已有 device worker 独占 402，应用另外拥有一个模型线程，不创建第二个 402 worker。
- 模型线程在库锁外初始化/步进；API 调用只短时交换固定大小数据。启动 external 时按 §7.7 握手；停止/复位先撤销旧会话和输出权限，必要安全动作使用新 epoch 完成有界握手，不得持有 runtime 锁阻塞等待模型。
- stop 完成后 worker 不再执行；应用暂停模型 API 调用，start 后以新 epoch 重新握手。uninitialize/destroy 前先停止并 join 模型线程，禁止失效句柄访问；destroy 不与任何模型调用重叠。

### 8.2 锁序

- canopennode-rtt 固定锁序：**生命周期锁 → OD 锁**（参考 `cia402-device-rtt.md`）。
- xnet 中公共调用先获取生命周期锁，再获取运行时互斥锁，worker 只获取互斥锁（`src/internal.hpp:164`）。402 模块加入后必须保持这一顺序。
- OD 批量调用（`xnet_canopen.h:201`、`:233`）各自独立加锁，不跨调用保持锁。
- 回调中不得获取任何锁。

### 8.3 数据竞争注意点

- 0x6040 由 SDO/RPDO 写入，由 402 线程读取：需在 OD 锁内读取快照。
- 0x6041 由 402 线程写入，由 TPDO 读取：需在 OD 锁内写入。
- pending 帧的校验、复制与消费由 runtime 锁保护；模型计算结果只通过帧发布，不共享未同步的模型内部指针。

### 8.4 周期与时延

- 模型使用固定步长，可从 2 ms 验证；worker 使用实际 dt。两者不要求同相，也不保证同一 SYNC 内反馈。
- 当前 worker 的 `wait_for(worker_period_us)` 是处理后等待，不是绝对周期截止期限。晚到时记录 overrun，禁止无界补步；模型步进策略及最大允许延迟须在配置中明确。
- 反馈迟于产生它的命令是正常行为，通过 command_seq 和本机发布时刻计算年龄；超限则诊断并拒绝。心跳、反馈、动作超时预算须覆盖模型周期、worker 周期、计算和锁等待，并留出可测裕量。
- 验收至少记录周期分布/最大抖动、模型 WCET、API 持锁时长、命令到反馈与 TPDO 的延迟、反馈最大年龄和丢帧/超时计数。目标阈值须在 §9.6 决定，不把平均值或“一两个周期”的估算当作最坏上界。

---

## 9. 设计决策与待决问题

1. **许可证与复用**：canopennode-rtt 的许可证是否允许复制到本仓库？（影响 §7.2）
2. **NMT Stopped 时的 PDS 策略**：转到 quickStop 还是 disableVoltage？需与 Kinco 硬件行为对齐。
3. **xnet ABI（本次确定）**：不新增 C 函数、不改现有结构和通用批量语义；扩展每轴本地 OD 帧协议及诊断对象。
4. **模式范围（待确认）**：PP/PV 为首期基线；是否加入 PT 或其他模式由模型能力与验收范围决定。若接入力矩模型，必须单独选择 PT=4 或 CST=10 并补全对象/参数，不因参考 Torque Controller 就强制所有模型支持 PT。
5. **运行中切换模式**：是否允许 OPERATION_ENABLED 下写 0x6060？
6. **周期与 WCET（待确认）**：确定目标 CPU、worker/模型周期、各 WCET、最大持锁/抖动/端到端延迟阈值，以及 startup/heartbeat/feedback/action 超时、command_max_age_us 和超周期步进策略。未填写不得宣称实时验收通过。
7. **测试硬件**：是否有 Kinco FD124 实物用于验收？还是只做仿真验收？
8. **EtherCAT 行为对照**：是否需要保证与 EtherCAT 示例 `cia402appl.c` 的 Statusword 在所有状态下逐位一致？
9. **控制权边界（本次确定）**：PDS 和 Statusword 归核心；普通转换由主站发起，模型只提供动作结果/模式就绪、实际值、限幅及故障条件。动作完成影响转换时机，模型不能指定目标 PDS。
10. **故障与恢复（本次修订）**：0x2F00 为完整反馈导出的当前故障条件，不直接由模型写；首次码锁存到 0x603F，后续新条件独立 EMCY。仅故障源消失且健康、复位许可成立时接受主站新边沿，并等待匹配动作完成。
11. **执行与轨迹所有权（本次确定）**：同进程、独立模型线程、库锁外计算；核心负责 profile/斜坡，模型负责跟踪动态与限幅，不实现第二份 PDS。
12. **交换与生命周期（本次确定）**：单条目完整帧校验提交，独立心跳，worker 统一消费；代次/序号防旧帧，external 缺失或失联不回退 internal。
13. **实现前协议冻结（待完成）**：按 §7.5 冻结帧版本 1 的逐字节布局、全部动作/模式枚举、错误码表、配置 schema 和诊断字段，提供编码往返测试；不得边联调边猜测结构体布局。

本次确定项是后续开发约束，不代表实现完成或硬件安全认证；§1.1 与 §7.5.8 描述的现有实现差距仍需开发验证。

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
| T-MODEL-ACTION | 模型延迟完成、ERROR、动作超时、安全抢占与迟到 DONE；模式未就绪不输出新模式非零 demand |
| T-MODEL-FRAME | 编解码、长度/版本/枚举/有效位错误、部分写失败、pending 占用、重复提交不刷新年龄、拒绝确认与重算；失败不得污染实际值 |
| T-MODEL-LIFE | 启动无反馈、待机/FAULT 心跳、心跳正常但反馈超时、外部失联不回退、单调时钟判断及计数回绕 |
| T-MODEL-RESET | 计算期间 Reset/stop/start、旧代次/旧动作拒绝、uninitialize 后重绑与线程退出；持续故障不允许复位，条件消失后必须新边沿 |
| T-MODEL-UNITS | Target 不被覆盖、profile 不重复、正负方向、单位缩放/溢出、限幅及 target reached 容差/持续时间；按启用模式覆盖 |

建议以 canopennode-rtt 的 `examples/cia402_multi_axis_device/` 与 `examples/demo_device/` 为参考，编写 host 侧（x86）测试，不依赖 RT-Thread。

### 10.2 集成测试（xnet）

- 使用 `xnet_intergration/tests/fake_xnet` 与 `tests/test_integration.cpp` 扩展 402 场景。
- 通过 SDO / PDO 写 0x6040，检查 0x6041 状态序列。
- 覆盖 Reset Node、Reset Communication、NMT 切换、Node Guarding 超时场景。
- 使用 fake XNET 加同进程模型线程验证 RPDO → 402 → TPDO 顺序、帧提交与消费、多轴独立序号/代次及本地/网络权限。
- 注入单帧写失败、批量其他条目失败、模型延迟/无锁挂起、持续故障和复位竞争；验证失败不被心跳成功掩盖，拒绝帧可恢复重算。
- 理想跟随模型先验收，再加入可配置延迟、斜坡跟踪误差、限幅、故障源；同一契约下替换复杂模型。若采用 Simulink 生成代码，验证 initialize/step/terminate、端口类型与固定步长；TwinCAT TcCOM 或 FMU 不是本地 C API 的隐式运行时。
- 在目标平台按 §8.4 记录时序指标，对照 §9.6 阈值逐项验收；仅桌面测试通过不代表满足实时预算。

### 10.3 硬件验收（Kinco FD124，待确认）

- 上电 → Shutdown → Switch On → Enable Operation → Quick Stop → Fault Reset 的完整序列。
- 记录每一步 Statusword 与驱动实际状态的对应关系。

### 10.4 验证命令

本文档为需求文档，本身无需构建验证。后续实现时使用仓库现有构建与测试入口（以 `xnet_intergration/README.md` 与 `tests/` 为准），**不得新增构建工具**，除非任务明确要求。

---

## 11. 里程碑

| 阶段 | 内容 | 出口标准 |
|---|---|---|
| M0 | 决策：§9 待确认项、帧布局/配置/错误码冻结 | 模式与目标时序预算明确，接口编码往返测试可执行 |
| M1 | Pure-C 核心骨架：DriveIF、PDS 状态机、Controlword/Statusword | T-PDS、T-CW、T-SW 通过 |
| M2 | OD 绑定、多轴、0x6502 与模式开关 | T-MODE、T-MULTI 通过 |
| M3 | 故障、复位、安全抢占 | T-SAFE、T-FLT、T-RST 通过 |
| M4 | xnet 与模型适配：每轴帧/OD 权限、握手、独立线程、消费与 Reset 重绑 | T-MODEL-* 与 fake XNET 集成测试通过，ABI 无破坏，故障/旧帧不污染有效状态 |
| M5 | 模式扩展（PP/PV 首期，其余按 §9.4） | 对应模式测试通过 |
| M6 | 目标平台时序与 Kinco FD124 硬件验收（若纳入范围） | §8.4 及 §9.6 阈值通过，§10.3 记录完整 |

---

## 12. 开发检查清单

- [ ] 新文件使用 C 或 C++ 与 xnet 风格一致，无堆分配（核心部分）。
- [ ] 所有回调非阻塞，不获取锁。
- [ ] Statusword 状态位与 PDS 状态一一对应。
- [ ] Fault Reset 为边沿触发。
- [ ] 0x6502 与编译开关一致。
- [ ] Reset 后重新绑定 OD 扩展。
- [ ] `xnet_canopen.h` 未破坏式修改。
- [ ] Target / Demand / Actual 唯一所有权，模型无第二份 PDS 和 profile。
- [ ] 完整帧布局已冻结，失败无污染；处理确认包含拒绝结果，模型可恢复。
- [ ] 模型在待机、故障反应和 FAULT 中持续反馈/心跳，不把实际值强制清零。
- [ ] 动作/模式握手、epoch 与序号校验、旧帧和旧 DONE 拒绝均有测试。
- [ ] 故障条件、锁存记录、复位许可分离；external 超时不切换 InternalModel。
- [ ] 模型计算在锁外，线程退出与句柄释放有序；时序实测达标且说明进程隔离限制。
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
