# DS 402 (CiA 402) 驱动 Profile 需求与开发指南

> 适用范围：当前仓库 CANopenNode 核心，以及 `xnet_intergration/` NI-XNET CANopen 从站集成。
> 参考资料：`reference/canopennode-rtt`（Pure-C CiA 402 Device 核心 + RT-Thread 适配）、`reference/EtherCAT_Slave`（EtherCAT 侧 CiA 402 示例应用）。
> 本文是后续开发的需求基线。首期范围已确定为 PP 位置控制；仅将仍需负责人决策的事项标注为 **[待确认]**。
> 实现进度：单轴 PP 的 internal 路径及 external v1 帧交换已接入 CANopenNode worker；JSON 启动校验、反馈提交、epoch/序号/超时与异步动作确认已有实现和 fake-XNET 覆盖。本文其余硬件语义、诊断覆盖和实物验收仍需逐项验证；功能验证不构成硬实时或安全认证。

---

## 1. 背景与目标

### 1.1 现状

- 当前 CANopenNode（根 `README.md`）是 ANSI C、MISRA C:2012 风格的协议栈，分层为 `301/`、`303/`、`304/`、`305/`、`309/`、`storage/`、`extra/`，**不包含** CiA 402 实现。
- `xnet_intergration/` 把 CANopen 从站通过 NI-XNET 暴露给 VeriStand 模型。当前已增加显式 JSON 启用的单轴 PP internal/external 路径；普通配置仍只做通信与 OD 存储。NMT 状态、CANopen 状态、DS402 驱动状态仍是三个独立概念。
  - `include/xnet_canopen.h` 说明库可按 JSON 显式启用 DS402 PP；未新增公共 ABI。
  - `src/model.cpp` 对启用配置校验必需对象、权限、类型和已启用 PDO 映射；不自动补建对象。
  - external 帧交换通过本地 OD API 接入；模型仍由宿主应用运行，不由库隐式创建线程或模型实例。
  - `xnet_intergration/tests/app/README.md:113` 说明测试应用不实现 CiA 402。
- 设备 OD 由 JSON 配置描述；JSON 由工具从目标真实驱动设备的 EDS 转换生成。普通 CANopen 设备配置可以不包含 CiA 402 对象，电机驱动设备配置则包含其实际提供的 402 对象与 PDO 配置。
- 仓库已有 Kinco FD124 的 EDS（`xnet_intergration/eds/Kinco_FD_20250417_V1.2.eds`）与 JSON 配置样例（`examples/kinco_FD124.json`、`tests/app/kinco_FD124.config.json`）。

### 1.2 目标

在不破坏 xnet 通信边界的前提下，提供一个**可复用、与硬件解耦**的 CiA 402 PDS（Power Drive System）状态机与模式处理模块，使上层（xnet 模型层或应用）只需实现驱动回调即可获得标准的 Controlword/Statusword 行为，并满足以下目标：

1. 按 IEC 61800-7-201 / CiA 402 规定实现 PDS 状态机、Controlword 解析、Statusword 生成。
2. 当前仅支持单轴电机驱动，使用设备 JSON/EDS 中该轴的标准 OD 索引；多轴及 `logicalDevice` 偏移属于未来扩展。
3. 首期只支持 PP（Profile Position）位置控制。PV、PT、HM、CSP、CSV、CST 等模式作为未来扩展，不属于本次实现与验收范围。
4. 驱动动作通过回调（DriveIF）抽象，核心模块不包含任何硬件或 CAN 代码。
5. 核心为 Pure C，无堆分配，回调非阻塞，可在 RT 环境运行。
6. 与 xnet 的 OD 访问、批量 API、复位语义（Reset Node / Reset Communication）保持一致。

### 1.3 非目标

- 不实现厂商私有对象（如 Kinco 的 0x2FF0 系列），仅预留扩展点。
- 不实现通用运动规划器；首期 PP 的 profile/斜坡处理由核心负责，模型只实现位置跟踪、限幅和动态响应，不重复生成轨迹。
- 不修改 `xnet_canopen.h` 已发布的 ABI，除非另行批准（见 §9 第 3 项）。
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
| 多轴 | 参考实现支持，OD 基址 `0x6000 + n*0x0800` | 参考示例支持（`LocalAxes[]`） | 当前范围为单轴 |
| 模式 | 首期仅 PP（1） | 0x6502 存在时校验 PP 位，PDO 按 JSON 校验 | 无 |
| 堆使用 | 无堆，应用持有所有存储 | 静态 | 有 C++ 动态对象 |

**结论**：canopennode-rtt 的 Pure-C 核心在结构上最适合 xnet 复用（无堆、回调边界清晰、有安全所有权模型）。EtherCAT 示例更接近"验证用法"，状态转换表可用于交叉核对。

---

## 3. 术语与状态

### 3.1 三个独立状态（必须区分）

1. **运行时状态**：`xnet_canopen_status_t.runtime_state`（UNINITIALIZED / STOPPED / RUNNING / FAILED）。
2. **CANopen NMT 状态**：`nmt_state`。
3. **CiA 402 PDS 状态**：本模块维护，通过 Statusword bit0-3、5、6 表示。

CANopen NMT 是底层通信层状态机，DS402 PDS 是上层电机控制状态机。NMT 状态本身不直接改变 PDS；DS402 可以读取 NMT 状态作为自己的输入，并由 DS402 决定电机控制动作、PDS 转换及相应 OD 发布（见 §6.4）。

### 3.2 PDS 状态（CiA 402 标准）

`NOT_READY_TO_SWITCH_ON`、`SWITCH_ON_DISABLED`、`READY_TO_SWITCH_ON`、`SWITCHED_ON`、`OPERATION_ENABLED`、`QUICK_STOP_ACTIVE`、`FAULT_REACTION_ACTIVE`、`FAULT`。

参考：`reference/canopennode-rtt/profile/cia402/common/CO_402_state.h`；EtherCAT 对应常量 `cia402appl.h:90-97`。

---

## 4. 功能需求（FR）

每条需求给出验收标准。编号用于测试追溯。

### 4.1 OD 与对象

| ID | 需求 | 验收标准 |
|---|---|---|
| FR-OD-01 | 配置启用 `drive_402` 时，核心对象为 0x6040、0x6041、0x6060、0x6061；当前 PP 运行还依赖 0x6064、0x607A、0x6081、0x6083、0x6084。0x603F、0x6502、0x6085 等为可选对象。 | 初始化校验必需对象的类型、子索引、长度、访问属性；可选对象存在时也校验。PDO 映射不强制，已有映射须与 JSON/EDS 中的方向、对象和长度一致；未映射对象可通过 SDO 访问。必需项缺失或配置冲突时返回 `XNET_CO_MODEL_ERROR`。 |
| FR-OD-02 | 当前仅支持单轴，使用 JSON/EDS 描述的该轴标准对象索引，不做轴索引偏移或多轴实例化 | 配置中的单个 PP 轴绑定且运行；声明多个 DS402 轴的配置必须拒绝初始化，不得被误解释为多个 DS402 轴 |
| FR-OD-03 | 核心不得假定对象有符号语义；符号/无符号由 OD/EDS 决定 | 0x6060 为 INTEGER8，0x6061 为 INTEGER8，读写不做错误的符号扩展 |
| FR-OD-04 | 对象访问须经过 OD 锁（与 xnet 的批量 API 锁序一致，见 §8.2） | 通过 SDO 写 0x6040 与批量写 API 并发时无数据竞争 |
| FR-OD-05 | 主站独占写入 target 与 profile 参数，核心独占发布 demand、状态和已接受实际值；模型只写本地交换对象 | 覆盖命令、反馈的网络/本地权限；核心不得覆盖 0x607A 或 PP profile 参数 |

JSON 是 OD 与 PDO 配置的唯一运行时来源，初始化阶段不根据 `drive_402` 自动补建对象，也不在运行时动态添加 PDO 映射。普通设备 JSON 未声明 `drive_402` 时按普通 CANopen 设备初始化，不执行 402 对象校验或状态机；声明启用时，若 EDS 转换得到的 JSON 缺少当前 PP 实现必需的对象或其他必要配置，必须作为配置错误拒绝初始化，不得静默降级为普通设备或自动补默认对象。0x603F 不存在时不发布该对象，但 external 故障仍通过状态机和 EMCY 处理。若使用 external 模型，所需本地交换对象必须作为本设备 JSON 的本地扩展项提供并通过同一初始化校验；不得由运行时绕过 JSON 创建。

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
| FR-CW-04 | 首期 PP 的 Statusword bit10（target reached）、bit12（set-point acknowledge）及 bit11（internal limit）由 PP 模式逻辑维护；其他模式相关位不在首期实现范围 |

### 4.4 模式

| ID | 需求 |
|---|---|
| FR-MODE-01 | 首期只启用 PP（模式值 1）；其他模式的运行逻辑不属于首期 |
| FR-MODE-02 | 0x6502 可缺省；存在时须包含 PP 能力（bit0），允许保留源设备声明的其他位。其他位不表示模拟器支持这些模式。 |
| FR-MODE-03 | 0x6060 写入 1 表示 PP；写入任何其他模式值必须拒绝（SDO Abort 或返回错误），不得改变当前模式；0x6061 仅在 PP 初始化/就绪后发布 1 |
| FR-MODE-04 | 首期不支持运行时模式切换；0x6060 不得从 PP 切换到其他模式。该能力作为未来需求另行设计 |
| FR-MODE-05 | PP profile 位置命令按 Controlword 的 new-set-point、change-immediately、relative 等相关位处理，并由核心生成 profile demand；同一命令不得重复触发，确认及 target reached 判据必须有单元测试 |

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
| NFR-04 | 首期以 CANopen 毫秒级基本控制为目标，当前只做功能验证；记录处理周期与延迟作为诊断信息，不设硬实时 WCET、抖动或端到端性能验收门槛，也不据此宣称硬实时能力 | 功能测试与可选诊断记录 |
| NFR-05 | 不得在 `xnet_canopen.h` 已有函数的签名或语义上做破坏性修改 | ABI 对比 |
| NFR-06 | 所有新文件包含版权与模块说明头，与 `CO_402_*` 风格一致 | 代码审查 |
| NFR-07 | 只在需要注释处添加注释（仓库约定） | 代码审查 |
| NFR-08 | 可记录模型周期、批量 API 持锁时间、worker 周期、反馈年龄与命令到反馈/TPDO 延迟，供功能调试使用；这些观测值不是首期性能通过/失败门槛 | 功能测试中的诊断记录 |

---

## 6. 行为规范（实现约束）

### 6.1 Controlword 处理顺序

1. 在 worker 锁内读取 Controlword、模式和 target 快照，并消费通过代次校验的完整模型反馈。
2. 若读取失败，调用 `CO_402_device_axisControlwordReadFailed()`，进入 FAULT 路径。
3. 判定 Quick Stop / Disable Voltage 等安全命令是否优先。
4. 按当前 PDS 状态处理普通命令。
5. 更新 Statusword，写入 0x6041。

### 6.2 Statusword 派生

Statusword 的状态位由 PDS 当前状态决定，PP 相关位由核心根据已接受反馈维护；模型不直接写 Statusword。bit10 的位置容差/持续时间、bit11 的限幅来源及 bit12 的 set-point acknowledge 判据须写入配置和测试。实际反馈无效或过期时，不得报告新的 target reached。Statusword 与实际值应在 OD 锁内发布，避免 TPDO 读到未提交反馈。

### 6.3 Fault Reset

- 只在 FAULT 状态且 bit7 上升沿时调用 `faultReset`。
- 连续保持 bit7=1 不得重复触发。
- 当前故障条件清除、模型心跳和完整反馈均有效、模型允许复位后，才可启动 faultReset 动作；匹配的 DONE 返回后回到 `SWITCH_ON_DISABLED`。
- 条件不满足则拒绝本次请求并诊断，不排队保留边沿。主站须在条件满足后重新发送可被采样的 0→1 脉冲；FAULT_REACTION_ACTIVE 中的提前边沿不得延迟执行。

### 6.4 NMT 与 PDS 的关系

- NMT 状态机只管理底层 CANopen 通信状态；NMT 状态变化不直接调用或改写 PDS 状态，也不直接触发电机控制动作。
- DS402 周期逻辑可以读取 NMT 状态，并据此决定自身是否接受普通控制命令、撤销输出许可、执行 disableVoltage/quickStop 等动作，以及如何更新 PDS 和 0x6041。所有电机控制行为变化必须经 DS402 状态机完成并由其发布相应 OD。
- NMT 状态到 DS402 行为的具体策略属于 DS402 产品策略，须在 DS402 内定义并测试；NMT 重新进入 Operational 不得自动恢复 `OPERATION_ENABLED`，需要重新满足 DS402 条件并由主站发送使能命令序列。

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

1. **OD 来源与校验**：启动时加载 JSON；该 JSON 由设备 EDS 转换工具生成并决定 OD 与 PDO，不由 402 核心补建对象。未声明 `drive_402` 时按普通设备初始化；声明启用时，校验 PP 必需对象、存在的可选对象及配置中的 PDO 映射；不强制指定 PDO 映射，必需对象缺失或配置不匹配时拒绝启动。
2. **绑定（当前实现）**：`Model::load()` 完成 PP 必需 OD 与已配置 PDO 校验；显式启用时由 `device.cpp` 在 `setup_stack()` 安装 0x6060 SDO 写扩展，拒绝非 PP 模式。当前是 C++ `Drive` 直接读写已校验 OD，尚无通用 `CO_402_device_bindOD()` API。
3. **周期调用**：402 只在现有 device worker 中处理，插入 RPDO 处理之后、TPDO 处理之前；另设应用拥有的模型线程，不在 worker 内执行模型计算。
4. **Reset**：Reset Node/Communication 与 worker stop 在重建或退出前请求 DS402 安全动作；external 路径等待匹配动作反馈，并通过新 epoch 使旧帧失效。NMT 层不直接修改 PDS。
5. **状态快照**：402 与模型诊断通过只读 OD 对象暴露，不新增 C 函数，不修改已有 40 字节 `xnet_canopen_status_t`；复位代次随模型命令快照发布。

### 7.4 DriveIF 对 Kinco FD124 的适配 **[待确认]**

- 需确认 FD124 的 0x6040/0x6041 行为是否与标准一致（尤其 bit4、bit5、bit8 的含义）。
- 需确认 FD124 EDS 中是否声明 0x6502、0x603F，以及它们的 PDO 映射属性。
- 需确认 FD124 的 quickStop 选项码（0x605A）与 shutdown 选项码（0x605B）默认值。

---

### 7.5 外部电机驱动模型接口

> 本节假定已有 **Slave 侧驱动行为模型**（下文简称“模型”），负责跟踪、动态响应、限幅及故障条件，可在其内部组合电机和负载模型。借鉴 Beckhoff Torque Controller 的强类型端口、固定步长、模式门控与诊断方式，不直接接入其主站控制逻辑。

**结论：保留现有 C ABI，新增本地交换对象和 OD 扩展。** 宿主模型通过 `bind_batch`、`read_batch`、`write_batch` 或等价的单对象 API 交互。PDS 状态机与 Statusword 只有库内一份；已有模型若包含 PDS，应拆出或关闭该部分。以下 v1 帧契约已实现为当前接口基线，模型计算仍由宿主应用负责。

#### 7.5.1 职责分工

| 角色 | 负责的 OD | 访问通道 |
|---|---|---|
| 主站 | 0x6040、0x6060（仅 PP）、0x607A 及 PP profile 参数（0x6081、0x6083、0x6084） | SDO / RPDO，按设备配置开放 |
| 402 核心 | 0x6041、0x6061、0x603F；PP profile demand；经验证的实际位置值 | worker 内发布，绝不覆盖主站 target |
| 模型交换适配层 | 动作/模式请求、命令代次、暂存与完整帧校验 | OD 扩展只做有界校验和复制，不推进 PDS |
| 模型 | 实际位置（可包含设备配置的其他遥测量）、动作结果、PP 就绪、限幅、当前故障条件与复位许可 | 通过本地完整反馈帧提交，不直接写标准实际值或 Statusword |

首期 PP profile/斜坡生成归核心，模型不得重复。PV、PT、HM、CSP、CSV、CST 等模式及相应 demand 处理均为未来扩展。

#### 7.5.2 本地交换对象（厂商自定义区 0x2000–0x5FFF）

| 索引 | 名称 | 类型 | 写入方 | 语义 |
|---|---|---|---|---|
| 0x2F00 | 当前模型故障 | UNSIGNED16 | 核心发布 | 来自最新已接受反馈帧，0 表示故障条件已消失，不是锁存记录 |
| 0x2F01 | 模型心跳 | UNSIGNED32 | 模型 | 独立于反馈提交；模型存活时持续递增 |
| 0x2F02 | 命令快照 | OCTET_STRING，56 字节 | 核心 | 版本 1 命令帧；字段布局见下表 |
| 0x2F03 | 完整反馈提交 | OCTET_STRING，36 字节 | 模型 | 版本 1 反馈帧；字段布局见下表 |
| 0x2F04 | 处理确认 | UNSIGNED32 | 核心 | 最后被 worker 完成处理的 feedback_seq，0 表示本代次尚无结果；须同时检查 0x2F05 |
| 0x2F05 | 处理结果 | UNSIGNED16 | 核心 | 0=无结果、1=已接受、2=命令过期、3=内容无效；epoch 失效通过新命令快照通知 |

- 当前单轴配置中，上述交换对象各表示唯一实例，不使用每轴 ARRAY 或轴 subindex 寻址；数据类型、长度与访问属性须由设备 JSON 的本地扩展描述。
- command_view 中的标准 402 对象使用设备 JSON/EDS 中的实际索引；不应用 `0x6000 + n * 0x0800` 轴偏移，也不允许一个模型帧被解释为多个轴。
- 0x2F01、0x2F03 为 SDO 只读、`local_write: true`，禁止 RPDO 写入；其余为核心只读发布、`local_write: false`。启动时验证权限及索引冲突。
- 旧草案中的模型直接写 0x2F00、标准实际值以及“心跳最后写”方案被本协议替代；不改变现有通用批量 API 的逐条尝试语义。
- 快照和反馈帧采用显式小端字段编码、固定长度、无隐式填充，不直接 memcpy 编译器结构体。实现前在接口头文件冻结版本 1 的逐字节偏移、长度、枚举、保留位和有效掩码；未知版本、错误长度、非法枚举/保留位须返回错误并诊断，不得截断或默认成功。
- **协议 v1 已冻结**（`402/CO_402_protocol.h`）：所有整数均为小端，二补数 I32；固定长度，不允许尾随字节。版本为 1，长度字段必须精确匹配；保留位必须为 0。

命令帧（0x2F02，56 字节）：

| 偏移 | 宽度 | 字段 | 约束 |
|---:|---:|---|---|
| 0 | U8 | version | 必须为 1 |
| 1 | U8 | length | 必须为 56 |
| 2 | U8 | action | 0 none、1 shutdown、2 switchOn、3 enableOperation、4 disableOperation、5 quickStop、6 faultReaction、7 faultReset、8 disableVoltage |
| 3 | U8 | flags | bit0 output_allowed、bit1 pp_ready；bit2..7 保留 |
| 4 | U64 | epoch | 非零 |
| 12 | U32 | command_seq | 非零 |
| 16 | U32 | action_seq | 非零；无动作时仍为当前代次动作序号 |
| 20 | U16 | controlword | 最近接受的原始主站值，仅供诊断/模型上下文，不授权模型重解析 |
| 22 | U16 | statusword | 核心当前值 |
| 24 | U8 | pds | 枚举 0..7，顺序同 `CO_402_PDS_*` |
| 25 | I8 | mode | 0 未显示、1 PP；其他值非法 |
| 26 | U16 | reserved | 必须为 0 |
| 28 | I32 | demand_position | PP 核心 profile 输出，counts |
| 32 | I32 | target_position | 最近接受的主站目标，counts |
| 36 | U32 | profile_velocity | counts/s |
| 40 | U32 | profile_acceleration | counts/s² |
| 44 | U32 | profile_deceleration | counts/s² |
| 48 | U32 | quick_stop_deceleration | counts/s² |
| 52 | I32 | demand_velocity | counts/s |

反馈帧（0x2F03，36 字节）：

| 偏移 | 宽度 | 字段 | 约束 |
|---:|---:|---|---|
| 0 | U8 | version | 必须为 1 |
| 1 | U8 | length | 必须为 36 |
| 2 | U8 | action_result | 0 none、1 BUSY、2 DONE、3 ERROR |
| 3 | U8 | flags | bit0 position_valid（必需）、bit1 velocity_valid、bit2 pp_ready、bit3 limited、bit4 reset_permitted；bit5..7 保留 |
| 4 | U64 | epoch | 必须等于当前 epoch 且非零 |
| 12 | U32 | feedback_seq | 非零、模序递增 |
| 16 | U32 | command_seq | 必须引用保留历史内且未超 `command_max_age_us` 的核心命令 |
| 20 | U32 | action_seq | 必须匹配当前动作序号 |
| 24 | I32 | actual_position | 当 position_valid 时有效，counts |
| 28 | I32 | actual_velocity | 仅 velocity_valid 时有效，counts/s |
| 32 | U16 | fault_code | 0 无当前故障，非零为设备故障表代码 |
| 34 | U16 | reserved | 必须为 0 |

序号 0 保留为无值；U32 比较使用模序且有效窗口小于 2³¹。相同 `feedback_seq` 的完全相同重试可幂等应答但不刷新新鲜度；同序号不同内容、旧序号、未知命令或不匹配 action_seq 均拒绝。0x2F03 一次写入必须正好 36 字节；pending 未消费时只允许同帧重试，否则返回设备状态错误。0x2F04/0x2F05 分别发布被消费的 feedback_seq 和结果（0 none、1 accepted、2 stale、3 invalid）。
- 位置为 I32 counts；U16 故障码原样对应设备故障表。PP 的位置比例、正方向、速度/加速度/减速度单位、限幅、target-reached 容差及持续时间由 JSON 明确，初始化时拒绝缺失或不一致配置。模型内部可用浮点，适配层负责有界转换，溢出不得静默回绕。
- 首期 PP 必须具备有效位置反馈及 profile 参数；速度、力矩对象和 PT/CST 能力仅在未来模式扩展时纳入要求。
- 模型故障经完整反馈进入核心，不使用 `xnet_canopen_set_fault` 代替 PDS 故障路径。心跳计数仅指示更新，超时使用 worker 本机单调时钟，不依赖模型墙钟。

#### 7.5.3 批量分组（模型侧绑定，启动时完成）

模型在 setup 阶段为唯一轴实例一次性绑定分组。首期统一命令/反馈帧仅承载 PP 所需的动作、位置 demand、位置反馈与诊断字段；通过版本与有效掩码为未来模式扩展保留空间。

| 分组 | 方向 | 内容 | 状态 |
|---|---|---|---|
| `command_view` | 读 | 本轴 0x2F02、0x6040、0x6041、0x6061、0x603F、0x2F04、0x2F05 | 首期 |
| `feedback_submit` | 写 | 本轴 0x2F03，单一条目、完整帧 | 首期 |
| `model_alive` | 写 | 本轴 0x2F01，独立条目 | 首期 |

- 模型每次检查调用返回值及每条状态，只使用全部成功的快照。绑定返回的类型和长度必须与帧契约一致。
- 首期只接受 PP；其他模式值按 FR-MODE-03 拒绝。模型仍更新健康和实际反馈，但不得施加未接受模式的输出。
- 未来新增模式须补充核心模式处理、模型能力、OD、参数及测试，不假定只增加模式表一行即可完成。

#### 7.5.4 运行阶段流程

1. worker 在 RPDO 后消费已提交反馈，检查心跳和反馈年龄，处理安全命令，再推进 PDS 与 PP profile，发布下一命令快照及 TPDO 数据。
2. 模型线程一次 `read_batch(command_view)` 取快照，释放 API 内部锁后计算；在待机、QuickStop、FaultReactionActive、FAULT 中也持续执行和报告实际值。
3. 模型一次写入完整 `feedback_submit`，逐项检查结果；成功只表示帧已校验并进入待消费缓存，不表示 PDS 已转换。由 0x2F04/0x2F05 判断处理确认与结果；拒绝时诊断并读取新命令后重新计算，不无限等待已被拒绝帧。
4. `model_alive` 单独更新。独立反馈超时可检测“心跳仍更新但计算无有效结果”，不得用心跳代替计算进展。

**动作与 PP 握手**：核心先发布动作请求而不是先提交目标 PDS 状态。`action_seq` 对每个新动作或抢占递增；模型以同 epoch、同 action_seq 返回 BUSY/DONE/ERROR。模型计算只执行有效命令，不自行重解原始 Controlword。PP 就绪通过匹配的动作/反馈确认；匹配动作的旧响应或被抢占动作的延迟 DONE 均不得生效。模式进入/退出握手属于未来模式扩展。

安全命令使核心立即撤销旧动作的逻辑输出权限，并发布优先安全动作；模型每次施加输出前检查最新动作/代次，发现抢占则丢弃旧计算。PDS 的完成仍等待匹配反馈。异步交换只能提供有界响应，不能承诺同步撤销物理效果；不得用于真实硬件安全保证。`action_timeout_us` 到期须锁存故障并切入失败关闭策略；若安全反应本身无反馈，则撤销输出许可、保留故障诊断并按已定义的无反馈反应进入 FAULT，不伪造 DONE。

#### 7.5.5 所有权与权限

- **Target / profile 参数**：只由主站写，模型与核心只读；demand 使用命令帧中的独立字段。
- **Statusword、mode_display、0x603F**：只由核心按状态规则发布，包括模型无反馈时的故障路径。**标准实际值、0x2F00**：只由核心从已接受反馈发布；无新反馈时保持并标记年龄。
- **模型本地写**：仅 0x2F01、0x2F03，网络不能写；应用保证该单轴模型对象只有一个模型写入者。
- OD 扩展回调只校验/复制完整反馈，不运行状态机、不产生 EMCY、不获取额外锁；状态、故障和 TPDO 由 worker 执行。

#### 7.5.6 超时与缺省模型

- **配置**：仅 `drive_402` 启用 402；其中显式选择 internal 或 external 模型、首期模式 PP、周期及超时。未声明时保持现有通信行为。
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

1. 单轴模型仅允许一个写入者，最多一个未确认反馈帧。0x2F03 的 OD 写扩展在现有 runtime 锁内先校验整个帧，再一次性复制为 pending；禁止分段提交、禁止先覆盖已接受标准实际值。pending 未处理时拒绝新提交并诊断；模型仍持续本地步进和心跳，收到确认后才提交下一帧，不能累积无界队列。相同序号的重试只允许完全相同的帧，且不得刷新反馈新鲜度。
2. 校验版本、固定长度、当前 epoch、必需有效字段及源命令年龄；动作结果必须关联已发布 action_seq。反馈允许来自有界历史 command_seq，而非强求等于最新 worker 周期；超过 `command_max_age_us` 则拒绝，防止异步模型永远追不上快照。
3. worker 再校验 epoch、命令年龄和动作相关性，合法时在同一锁内发布完整实际值及模式反馈，更新处理确认及结果；过期帧丢弃并发布拒绝结果，不改变 PDS、故障条件或反馈年龄。旧代次通过更换 epoch 清空确认，使模型放弃等待并重新握手。抢占前帧可保留有效实际测量，但旧动作完成响应不得提交状态。
4. epoch 为句柄内 U64 单调代次，在 initialize、协议复位、stop/start 的模型会话失效点更换，并清除 pending、动作响应及握手状态；Reset Communication 仍保留普通应用 OD，交换会话数据例外。复位/停止开始即撤销旧代次，任何跨越该点的计算结果均拒绝。单次安全停止的完成握手使用新代次；重启需重新握手，不自动恢复 OperationEnabled。
5. command_seq、action_seq、feedback_seq 为 U32，0 保留为无值，按模序比较且有效窗口小于半范围；回绕行为须测试。epoch 不允许回绕复用，耗尽则拒绝启动并诊断。uninitialize 后批量 ID 失效，销毁前模型线程必须退出；新句柄不得接收旧线程反馈。

#### 7.5.8 与当前核心实现的差距（待开发）

`402/drive402.hpp` 当前的 `ExternalModel` 是纯 C++ 接口，不关心 OD，与本设计有以下差距：

1. `Feedback` 需扩展位置有效性、PP 就绪、限幅、持续故障、复位许可及动作响应。
2. 建立有效动作请求与 PP position demand 输出，不能只向模型提供最终 PDS 状态。
3. 将存活、反馈及动作超时分开；心跳计数判断更新，本机单调时间判断超时。
4. 增加完整帧 OD 扩展、pending/消费缓存、代次与序号校验，以及基于 JSON 的 OD/PDO 初始化校验。

开发方案：`402/` 核心保持不依赖 OD 的纯逻辑，新增一个 OD 适配层（位于 `xnet_intergration/src/`），负责把 OD 快照映射为核心的输入，并把核心输出写回 OD。

#### 7.5.9 对外部接口文档的影响

`xnet_canopen.h` 文件头已修订为：普通应用行为由外部模型提供，配置可显式启用库内单轴 PP 核心；NMT 与 DS402 状态仍分层。后续完成 external 接口时继续同步更新中英文说明。

同时补充单次批量调用的锁语义、跨调用不原子及无数据年龄保证；不能把一次 read_batch 宣称为“所有字段必定同一产生周期”。帧协议的版本、布局、提交/消费、权限、代次和错误语义须在接口头文件及 xnet 中英文 README 中同步说明。保留通用 write_batch 无回滚语义，不修改其行为来实现事务。

#### 7.5.10 外部电机驱动模型的 Slave 侧整合评估

外部电机驱动模型与 xnet 从站位于同一 Slave 侧，通过本地批量 OD 接口交互，不经过 CAN 总线。评估结论与待确认项如下：

| 评估项 | 结论 | 依据 / 处理 |
|---|---|---|
| 数据通道 | 同进程本地 C API，不产生额外模型 CAN 报文 | 延迟取决于两个线程的相位、计算与锁等待，不保证一周期反馈或两周期往返 |
| 故障检测边界 | worker 存活且可取得锁时，能检测模型无锁挂起和无有效反馈 | 同进程崩溃可终止整个服务；持锁挂起可阻止超时检测，不宣称进程级故障隔离 |
| 锁与抖动 | 短时交换、锁外计算 | 回调只校验/复制固定大小帧，不执行文件 I/O 或模型步进；耗时仅作诊断记录 |
| 访问与完整性 | 仅心跳、完整帧允许本地模型写 | 网络写交换对象、直接写标准实际值均被拒绝；原子性来自单帧 OD 扩展，不来自 write_batch |
| 复位与生命周期 | 批量 ID 可保留，模型会话必须失效 | uninitialize 后重新绑定；Reset/stop/start 更新 epoch 并重新握手 |
| ABI | 不新增 C 函数，不改变已有结构及通用批量语义 | 必须开发帧扩展、适配器、核心响应接口并更新说明，绝非只改头注释 |
| 宿主进程 / 线程 | 首期同进程、独立模型线程 | 若要求崩溃隔离，应另立 IPC、独立看门狗与进程恢复需求，不在本期隐含承诺 |
| 驱动响应 | 模型不拥有 PDS，但提供动作完成、PP 就绪和实际位置动态 | 核心保留状态生成权，避免双状态机及“先转换再等待完成”的循环依赖 |

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

1. 加载设备 JSON；若声明 `drive_402`，校验其 OD、PDO、PP 对象类型/权限/单位、external 模型所需本地扩展对象及全部超时配置。对象由 JSON 提供，不由运行时生成或补齐。
2. 校验成功后绑定 OD 扩展、初始化核心和新 epoch，默认禁止输出；根据配置显式选择 internal 或 external。
3. 应用 setup 绑定分组、初始化模型，启动后以当前 epoch 反馈与心跳完成握手；external 缺失不得回退。
4. NMT 与 PDS 分层管理；NMT Operational 只是 DS402 可查询的通信状态之一，仍须完成 PP 模型就绪及主站使能序列。

**运行阶段（两个线程异步执行，不保证表内步骤同周期完成）**

| 步 | 执行者 | 动作 |
|---|---|---|
| 1 | 主站 | 通过 SDO 或 RPDO 写入 Controlword、PP 模式值和目标位置 |
| 2 | worker | 接收与 RPDO 完成后快照命令，消费合法 pending 帧；过期/旧代次帧拒绝并诊断 |
| 3 | 402 核心 | 以实际经过时间检查心跳、反馈和动作超时，处理持续故障及安全抢占；可读取 NMT 状态作为策略输入 |
| 4 | 402 核心 | 根据合法命令与匹配动作结果推进 PDS 和 PP profile；不覆盖 target |
| 5 | worker | 发布下一有效动作/PP demand 快照、状态与已接受实际值，更新处理确认及结果 |
| 6 | 301 核心 | TPDO 处理和发送；402 必须位于 RPDO 与 TPDO 处理之间 |
| 7 | 模型线程 | 读取 command_view，在库锁外按固定步长计算；有未确认帧时先检查处理结果，不阻止本地步进 |
| 8 | 模型线程 | 在所有 PDS 状态持续反馈；施加输出前检查动作/代次，提交完整帧，独立更新心跳 |
| 9 | worker | 后续周期消费反馈；只有满足 PP 判据且反馈有效才更新 target reached 等状态 |

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

### 8.4 周期与时延（功能验证）

- 当前目标是 CANopen 毫秒级基本控制；模型可使用固定步长（例如 2 ms）开展功能验证，worker 使用实际 dt。两者不要求同相，也不保证同一 SYNC 内反馈。
- 当前 worker 的 `wait_for(worker_period_us)` 是处理后等待，不是绝对周期截止期限。晚到时记录 overrun，禁止无界补步；模型步进策略及最大允许延迟须在配置中明确。
- 反馈迟于产生它的命令是正常行为，通过 command_seq 和本机发布时刻计算年龄；超限则诊断并拒绝。功能测试使用配置的心跳、反馈、动作超时检查异常路径。
- 可记录周期分布、模型处理时间、API 持锁时长、命令到反馈与 TPDO 的延迟、反馈年龄和丢帧/超时计数，用于解释功能测试结果；首期不设性能合格阈值，也不据此声明实时或硬实时保证。

---

## 9. 设计决策与待办

1. **许可证与复用 [待确认]**：确认 canopennode-rtt 的许可证是否允许将其实现移植到本仓库；若不允许，按其公开行为重新实现。
2. **目标设备配置与轴数范围（已确定/实现前核对）**：当前仅支持一台设备上的单轴电机驱动，标准对象与本地交换对象各只有一个轴实例，不使用轴偏移或按轴数组。核对首期目标设备 EDS 转换得到的 JSON，确认 PP 必需 OD/PDO 项满足 FR-OD-01。若不满足，应修正转换/配置或报告配置不兼容，不得自动补项。多轴及 `logicalDevice` 轴偏移属于未来需求。
3. **xnet ABI（已确定）**：不新增 C 函数、不改现有结构和通用批量语义；使用本地交换帧协议及诊断对象。
4. **首期模式（已确定）**：只实现 PP；PV、PT、HM、CSP、CSV、CST 等均为未来扩展。首期不支持运行时模式切换。
5. **功能验证目标（已确定）**：以 CANopen 毫秒级基本控制为目标，只做功能验证；周期/WCET/抖动/延迟记录不设性能验收门槛，不作硬实时声明。
6. **NMT/DS402 边界（已确定）**：NMT 管理底层通信；DS402 查询 NMT 状态并独占决定电机控制动作、PDS 转换及相关 OD 更新。各 NMT 状态下 DS402 的具体命令接受与输出策略须在 DS402 行为表和测试中定义。
7. **控制权边界（已确定）**：PDS 和 Statusword 归核心；普通转换由主站发起，模型只提供动作结果、PP 就绪、实际值、限幅及故障条件。动作完成影响转换时机，模型不能指定目标 PDS。
8. **故障与恢复（已确定）**：0x2F00 为完整反馈导出的当前故障条件，不直接由模型写；首次码锁存到 0x603F，后续新条件独立 EMCY。仅故障源消失且健康、复位许可成立时接受主站新边沿，并等待匹配动作完成。
9. **执行与轨迹所有权（已确定）**：同进程、独立模型线程、库锁外计算；核心负责 PP profile/斜坡，模型负责跟踪动态与限幅，不实现第二份 PDS。
10. **交换与生命周期（已确定）**：单条目完整帧校验提交，独立心跳，worker 统一消费；代次/序号防旧帧，external 缺失或失联不回退 internal。
11. **实现前协议冻结（实现前完成）**：按 §7.5 冻结帧版本 1 的逐字节布局、首期动作/反馈枚举、错误码表、JSON schema 和诊断字段，提供编码往返测试；不得边联调边猜测结构体布局。
12. **实现状态**：协议 v1 已冻结，JSON external 配置校验、帧编解码/提交、心跳与命令新鲜度校验、反馈消费、动作 BUSY/DONE/ERROR、PP readiness/reset permission、故障发布和 reset/stop epoch 更新已接入。仍须持续验证设备 EDS 的实际语义、故障码映射、驱动时序和硬件行为；不得据此宣称硬实时或硬件安全认证。

本次确定项是后续开发约束，不代表实现完成或硬件安全认证；§1.1 与 §7.5.8 描述的现有实现差距仍需开发验证。

---

## 10. 测试计划

### 10.1 单元测试（纯软件，无硬件）

| 测试组 | 覆盖内容 |
|---|---|
| T-PDS | 全部 PDS 合法转换；非法转换忽略；0x000F 回调顺序 |
| T-CW | Controlword 位解析；边沿复位；保留位处理 |
| T-SW | 每个 PDS 状态下 Statusword 的状态位 |
| T-MODE | 只接受 PP 的 0x6060 写入校验；0x6061 回读；0x6502 存在时包含 PP 位，允许其他位 |
| T-CONFIG | 普通设备 JSON 不触发 402 校验；可选对象及 PDO 映射缺失时仍可初始化；必需对象缺失、类型或映射冲突时初始化失败 |
| T-SAFE | QuickStop、DisableVoltage 抢占 BUSY 动作 |
| T-FLT | 故障路径、故障锁存、0x603F 更新、fail-closed |
| T-RST | Reset Node / Communication 对 OD 与绑定的影响 |
| T-NMT | NMT 状态改变本身不直接改写 PDS；DS402 按自身策略读取 NMT 状态并更新 PDS/OD |
| T-AXIS | 单轴 JSON/OD 绑定；不得将其他 OD 项解释成额外的 402 轴 |
| T-MODEL-ACTION | 模型延迟完成、ERROR、动作超时、安全抢占与迟到 DONE；PP 未就绪时不输出 PP demand |
| T-MODEL-FRAME | 编解码、长度/版本/枚举/有效位错误、部分写失败、pending 占用、重复提交不刷新年龄、拒绝确认与重算；失败不得污染实际值 |
| T-MODEL-LIFE | 启动无反馈、待机/FAULT 心跳、心跳正常但反馈超时、外部失联不回退、单调时钟判断及计数回绕 |
| T-MODEL-RESET | 计算期间 Reset/stop/start、旧代次/旧动作拒绝、uninitialize 后重绑与线程退出；持续故障不允许复位，条件消失后必须新边沿 |
| T-MODEL-UNITS | Target 不被覆盖、PP profile 不重复、正负方向、单位缩放/溢出、限幅及 target reached 容差/持续时间 |

建议以 canopennode-rtt 的 `examples/cia402_multi_axis_device/` 与 `examples/demo_device/` 为参考，编写 host 侧（x86）测试，不依赖 RT-Thread。

### 10.2 集成测试（xnet）

- 使用 `xnet_intergration/tests/fake_xnet` 与 `tests/test_integration.cpp` 扩展 402 场景。
- 使用一份普通设备 JSON 和一份启用 402 的 EDS 转换 JSON 验证初始化分支；后者缺少 PP 必需 OD 或 PDO 配置不合法时必须启动失败。
- 通过 SDO / PDO 写 0x6040，检查 0x6041 状态序列。
- 覆盖 Reset Node、Reset Communication、NMT 切换和 Node Guarding 超时；验证 NMT 层不直接执行电机控制动作，PDS/Statusword 变化只由 DS402 产生。
- 使用 fake XNET 加同进程模型线程验证单轴 RPDO → 402 → TPDO 顺序、帧提交与消费、序号/代次及本地/网络权限；验证当前配置不创建或隐式处理额外 402 轴。
- 注入单帧写失败、批量其他条目失败、模型延迟/无锁挂起、持续故障和复位竞争；验证失败不被心跳成功掩盖，拒绝帧可恢复重算。
- 理想跟随模型先验收，再加入可配置延迟、斜坡跟踪误差、限幅、故障源；同一契约下替换复杂模型。若采用 Simulink 生成代码，验证 initialize/step/terminate、端口类型与固定步长；TwinCAT TcCOM 或 FMU 不是本地 C API 的隐式运行时。
- 以功能正确性及 CANopen 毫秒级基本控制目标为验收方向；可按 §8.4 记录周期与延迟，但功能验证不构成硬实时性能门槛或实时性认证。

### 10.3 可选设备行为对照（Kinco FD124）

- 如有 Kinco FD124 实物，可对比 EDS 转换 JSON 与设备实际支持的对象/PDO 配置，并检查上电 → Shutdown → Switch On → Enable Operation → Quick Stop → Fault Reset 的功能序列。
- 此项为可选功能对照，不是首期必需验收，也不包含实时性能或硬件安全认证。

### 10.4 验证命令

本文档为需求文档，本身无需构建验证。后续实现时使用仓库现有构建与测试入口（以 `xnet_intergration/README.md` 与 `tests/` 为准），**不得新增构建工具**，除非任务明确要求。

---

## 11. 里程碑

| 阶段 | 内容 | 出口标准 |
|---|---|---|
| M0 | 决策：§9 待确认项、PP 帧布局/JSON 配置/错误码冻结 | EDS 转换配置与 PP 必需对象/PDO 校验规则明确，接口编码往返测试可执行 |
| M1 | Pure-C 核心骨架：DriveIF、PDS 状态机、Controlword/Statusword | T-PDS、T-CW、T-SW 通过 |
| M2 | JSON OD/PDO 初始化校验、单轴 OD 绑定与 PP 能力声明 | T-CONFIG、T-MODE、T-AXIS 通过 |
| M3 | 故障、复位、安全抢占 | T-SAFE、T-FLT、T-RST 通过 |
| M4 | xnet 与单轴模型适配：帧/OD 权限、握手、独立线程、消费与 Reset 重绑 | T-MODEL-* 与 fake XNET 集成测试通过，ABI 无破坏，故障/旧帧不污染有效状态 |
| M5 | 未来模式扩展（不属于首期） | 另行定义范围与需求后启动 |
| M6 | 可选 Kinco FD124 行为对照 | §10.3 的对象/PDO 与功能序列记录完成；非首期出口条件 |

---

## 12. 开发检查清单

- [ ] 新文件使用 C 或 C++ 与 xnet 风格一致，无堆分配（核心部分）。
- [ ] 所有回调非阻塞，不获取锁。
- [ ] Statusword 状态位与 PDS 状态一一对应。
- [ ] Fault Reset 为边沿触发。
- [ ] 0x6502 存在时包含 PP 位；即使声明其他模式，模拟器仍只接受 PP。
- [ ] 当前只绑定一个 DS402 轴；多轴配置被明确拒绝，不做轴偏移或每轴数组处理。
- [ ] 设备 JSON 在初始化阶段完成 OD/PDO 校验；缺项或冲突明确报配置错误。
- [ ] Reset 后重新绑定 OD 扩展。
- [ ] `xnet_canopen.h` 未破坏式修改。
- [ ] Target / Demand / Actual 唯一所有权，模型无第二份 PDS 和 profile。
- [ ] 完整帧布局已冻结，失败无污染；处理确认包含拒绝结果，模型可恢复。
- [ ] 模型在待机、故障反应和 FAULT 中持续反馈/心跳，不把实际值强制清零。
- [ ] 动作握手、epoch 与序号校验、旧帧和旧 DONE 拒绝均有测试；首期无运行时模式切换。
- [ ] 故障条件、锁存记录、复位许可分离；external 超时不切换 InternalModel。
- [ ] 模型计算在锁外，线程退出与句柄释放有序；功能验证通过并说明进程隔离限制，不宣称硬实时达标。
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
