# 阶段 C：传输边界与 KR260 存储接口

日期：2026-11-02，Asia/Singapore。依据本地已安装 SDK、现有部署源码及 AMD 官方资料作设计核对；本次没有打开设备、连接远端、发送命令或运行 Vivado。本说明只定义后端与时序口径，不指定物理执行频率，也不恢复已移除的阶段 A 任务。

## 1. 当前实际通路

现有项目通路是 **PC → SSH → KR260 Linux → SocketCAN → USB-CAN → PiPER**。配置中的接收器驱动为 `gs_usb`，产品为 `candleLight USB to CAN adapter`，设备路径经过 USB 控制器。它不是已经接入 PL 的 CAN 控制器。配置只描述已有设备，不能证明本次实时连接状态。来源：[io_config.json](/media/aup/UbuntuData/PIPER机械臂/official_adapter/lerobot_robot_piper_shared/io_config.json:48)。

后续root的[2026-11-02板端只读采集](../docs/stage_d_board_readonly_20261102/can_interface/verification.json)实际确认`can0`存在，驱动`gs_usb`、USB父设备`1-1.3:1.0`；当时`DOWN/STOPPED`、报告RX/TX packets均0，details没有配置bitrate字段。该次未启用/改动接口、未打开SocketCAN数据socket或发帧；它仍属于PS/USB路径，不证明PL物理CAN后端、线路或执行效果已验证。

远端桥逐帧调用非阻塞 `socket.send()`；返回 `accepted_sequence` 表示桥端检查和 socket 提交完成，不能解释为驱动器已接受整个关节组、按时执行或到位。桥端序号位于 SSH RPC 中，没有编码进下列 CAN 目标载荷。来源：[target()](/media/aup/UbuntuData/PIPER机械臂/deployment/lerobot_robot_piper_executor/remote_bridge.py:246)、[send_frame()](/media/aup/UbuntuData/PIPER机械臂/deployment/lerobot_robot_piper_executor/remote_bridge.py:357)。

因此，保留现有 USB-CAN 的方案可消费 PL 产生的描述符，但 Linux 调度、驱动队列和 USB 传输仍在路径内；不能据 PL 时钟把这个方案称为“CAN 发送时刻已由 PL 保证”。

## 2. 已核实的 CAN 字节契约

下表是当前位置控制路径；每帧均为 classic CAN、11 位标准 ID、8 字节数据。六关节绝对位置必须用三个帧描述，夹爪另一个帧；它们在 CAN 上串行传输，没有共同的原子提交字段。

| ID | 字节 | 当前语义及量化 |
|---|---|---|
| `0x155` | 0–3 / 4–7 | J1 / J2，大端 signed int32，1 单位 = 0.001° |
| `0x156` | 0–3 / 4–7 | J3 / J4，同上 |
| `0x157` | 0–3 / 4–7 | J5 / J6，同上 |
| `0x159` | 0–3 | 夹爪宽度，大端 signed int32，1 单位 = 1 µm；当前路径不是角度模式 |
| `0x159` | 4–5 | 力度，实际 TX encoder 使用大端 unsigned 16 位，1 单位 = 1 mN；当前部署 1 N = 1000 |
| `0x159` | 6 / 7 | 当前固定 `0x01`：使能、宽度模式；`0x00`：不设置零点 |

来源：[项目实际编码](/media/aup/UbuntuData/PIPER机械臂/deployment/lerobot_robot_piper_executor/sdk_codec.py:73)、[SDK 关节编码](/home/aup/piper_sdk/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/piper/default/parser.py:420)、[SDK 夹爪编码](/home/aup/piper_sdk/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/effector/agx_gripper/default/parser.py:67)、[标准帧构造](/home/aup/piper_sdk/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/core/table_driven.py:61)。

SDK 夹爪消息的说明文字把力度标成 int16，而实际 TX encoder 使用 unsigned 16 位；此处按实际编码记录。当前 1 N 位于两种解释共同的正值范围内，本说明不据此扩大允许力度。来源：[消息字段说明](/home/aup/piper_sdk/pyAgxArm/pyAgxArm/protocols/can_protocol/msgs/effector/agx_gripper/default/transmit/arm_gripper_ctrl.py:22)。

阶段 C 描述符的 96 字节分成 32 字节元数据和四个 16 字节记录；每条记录的 ID 按 little-endian uint16 保存，DLC/flags 各一字节，随后是保持上述线序的八字节 payload 和四字节 reserved。记录头的字节序不改变 payload 内各 CAN 字段的大端编码。这里记录旧路径的 `status_code=0x01`，不授权新后端自动使能：夹爪力度、模式/使能状态和 `set_zero` 必须来自显式审阅的命令参数，缺失时拒绝生成对应描述符，不自行补 enable、清错或置零。`0x151` 模式/速度、`0x470` 关节使能与 `0x150` 急停/重置不属于此四帧目标组。

反馈 `0x2A5/0x2A6/0x2A7` 分别携带对应两轴角度，同样按大端 signed int32 毫度解码；`0x2A8` 是夹爪实际反馈，`0x2A1` 是模式、运动和故障状态。它们不携带上述 RPC 的命令序号。`0x476` 是设置类命令的应答，不能当作三个位置目标帧的逐目标接受回执。来源：[项目反馈解析](/media/aup/UbuntuData/PIPER机械臂/official_adapter/lerobot_robot_piper_shared/protocol.py:12)、[设置应答字段](/home/aup/piper_sdk/pyAgxArm/pyAgxArm/protocols/can_protocol/msgs/piper/default/feedback/arm_feedback_set_instruction_response.py:10)。

PL 内部七轴 Q16.16 仍分别表示六轴 degree 和夹爪 metre。CAN 编码前必须转换：关节 `round(q16 × 1000 / 65536)`，夹爪 `round(q16 × 1000000 / 65536)`；不能直接把 Q16.16 字重新解释成 CAN 整数。BRAM 的主机字节序与 CAN 的大端字段应分别定义，编码后的整数所代表的物理值也应检查范围。上述转换只是后续实现契约，本阶段没有修改原编码器。

## 3. 120 周期的准确含义

100 MHz 的 120 周期等于 **1.2 µs**，只能作为 PL 内部 `tick → 受限目标的完整四帧描述符有效` 的设计预算。它不包含描述符消费者等待、PS 调度、USB 传输、CAN 仲裁/位填充/重发，也不包含电机跟随。

仅四帧各 8 字节的数据载荷就有 `4 × 8 × 8 = 256 bit`。即使按现有配置的 1 Mbit/s、完全忽略帧头、CRC、ACK、帧间隔和争用，载荷串行化也需要 **256 µs**。这是便于核算的理想下界，不是总帧时延估计或最坏时延承诺；已经大于 1.2 µs。可复算命令：`/usr/bin/python3 -c 'print(4*8*8/1000000, 120/100000000)'`。

建议分别统计 `tick_to_descriptor_cycles`、`descriptor_to_backend_accept`、`first_can_sof_to_last_can_eof`；现有 USB-CAN 路径不能为后两项给出固定周期保证。若后端背压，不能把 `valid && ready` 的可变等待混入固定流水线预算：空间不足时走明确的停止/拒绝路径，不重写源时刻或默默积压旧目标。`descriptor_valid`、描述符入队完成、CAN 帧完成和执行器到位应使用不同事件名。

## 4. AXI BRAM Controller 与真双口存储

AMD PG078 v4.1 明确支持控制器占用一个或两个 BRAM 口；单口模式将 AXI 读写仲裁到一个存储口，双口模式由控制器同时占用读口和写口。其 BRAM 读延迟可配置，必须与实际存储输出寄存级数匹配。[AMD AXI BRAM Controller PG078，Overview / BRAM Interface / Read Latency](https://docs.amd.com/v/u/en-US/pg078-axi-bram-ctrl)。

本设计建议 **AXI BRAM Controller 单 BRAM 口（`C_SINGLE_PORT_BRAM=1`）→ 真双口 BRAM 的 A 口；PL chunk 读取器独占 B 口**。`single port` 在这里描述控制器的口占用，不是把 BRAM 配成单口存储。不要先让控制器连接 A/B 两口，再让 PL 共驱 B 口。BMG 的 True Dual Port 为两口提供独立操作能力，碰撞行为仍需遵守器件规则。[AMD Block Memory Generator PG058，Memory Types / Collision Behavior](https://docs.amd.com/v/u/en-US/pg058-blk-mem-gen)。

本地 Vivado 2025.2 的 IP 定义也可只读核对：

- [axi_bram_ctrl component.xml](/home/aup/Xilinx/2025.2/Vivado/data/ip/xilinx/axi_bram_ctrl_v4_1/component.xml:594)：仅在 `C_SINGLE_PORT_BRAM=0` 且 external 模式时启用控制器的第二 BRAM 接口。
- [blk_mem_gen component.xml](/home/aup/Xilinx/2025.2/Vivado/data/ip/xilinx/blk_mem_gen_v8_4/component.xml:2827)：存在 `True_Dual_Port_RAM` 类型；不是已经为本项目生成的 IP 实例。

为控制时序，建议初版 A/B 同用约定的 100 MHz 时钟，ECC 关闭并明确两口读延迟；这是可回退的设计选择，不是已验证的板上配置。PS 仅写 inactive bank，完成写入及必要的可见性屏障后发布提交信息；PL 只能接纳完整且时效有效的块，active bank 未释放前禁止覆盖。真双口 RAM 本身不能解决同地址并发读写或半块可见性问题。PS 的 AXI 装载延迟不属于 tick 流水线的 120 周期。

## 5. KR260 资源与未布线接口

K26 使用 XCK26，公开容量为：117,120 LUT、234,240 FF、144 个 36 Kb BRAM、64 个 UltraRAM、1,248 个 DSP。容量是器件事实，不是本设计的利用率、可用余量或时序结果。[AMD K26 数据手册 DS987，Programmable Logic，2026-03-25 rev 1.6](https://docs.amd.com/r/en-US/ds987-k26-som/Programmable-Logic)。

KR260 数据手册列出四个 Pmod 接口和 USB host，但不能据此推断当前机械臂 CAN 已连接到 PL。AMD 自己的 KR260 CAN 示例另外使用 Pmod CAN 设备；这也是可选外设示例，不证明本项目拥有该硬件或所需连线。[AMD KR260 DS988，Product Details](https://docs.amd.com/r/en-US/ds988-kr260-starter-kit/Product-Details)、[AMD KR260 CAN 示例，CAN Communication Setup and Demonstration](https://xilinx.github.io/kria-apps-docs/kr260/ros2_multinode_communication_via_tsn/0_2/build/html/docs/app_deployment.html#can-communication-setup-and-demonstration)。

当前阶段 C 的 `tx_backend` 应止于受保护的描述符接口。将其连接到真正的 CAN 控制器/收发器属于后续物理集成；本文件不选择扩展板，不分配封装引脚，也不提供未经接线核实的 CANH/CANL、时钟或电平约束。若沿用 PS/USB-CAN 消费描述符，应如实称为“PL 目标生成 + PS/USB 发送后端”，把确定性承诺限制在 PL 描述符输出之前。任何最终物理发送路径都只能从 `limit_gate` 后取得目标；PS 不应有绕过该门的目标注入入口。

## 本地证据指纹

```text
SDK commit: d90e234ba8e408cfa964b38aa9120b3131fc9f85
deployment/lerobot_robot_piper_executor/sdk_codec.py
  SHA256 62bfa3a83606f6608ce214ee0c08927d86418a6119cb69c06eacc08e6b0b686c
deployment/lerobot_robot_piper_executor/remote_bridge.py
  SHA256 32fc6e9af3285cc89f3b9f3b187ee573991058b29cdbdaed9991c0027893a502
official_adapter/lerobot_robot_piper_shared/io_config.json
  SHA256 3f7a8ed07c65095f24f17d628ac91fa1b75302f69648d387eef1d68223241c6b
SDK drivers/piper/default/parser.py
  SHA256 b1262c12c15aeb8d2e43cfb4685fc57403a22472bfe846a422fba29628521df0
SDK drivers/effector/agx_gripper/default/parser.py
  SHA256 34ff0be7ae0e315b4163331d32a608be8979db0576cdb0db86d6a3b6f84885a1
```
