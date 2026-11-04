# D1-T 更新频率、接口语义与验收边界

日期：2026-10-27（Asia/Singapore）。本文件只汇总此前已读取的本地源码及官方公开资料；没有新增硬件操作、重新检索或运行厂商程序。

## 结论

没有一手证据证明“D1-T 硬件最高只能 10 Hz”，也不能据此认定 10 Hz 是系统的最大瓶颈。已保存机载源码能证明的，是角度查询线程配置了 100 ms 等待；命令接收、反馈扫描、电机内部闭环是不同层次。软件应先修正已确认的时间语义和执行问题，当前证据不支持直接提高频率或承诺消除静态残差。

## 三层频率必须分开

| 层次 | 当前能确认的内容 | 不能据此推断的内容 |
| --- | --- | --- |
| 上位机目标生成与发送 | 轨迹发送频率由应用配置；DDS 回调另行接收命令 | 发出 10 次/秒不代表电机只控制 10 次/秒，也不证明每次目标都被采用 |
| 机载查询与反馈发布 | 保存代码每轮先等 100 ms，再依次查询七个关节并发布 | 不是各轴同时采样，也不是严格固定 10 Hz；实际周期还包含查询和发布耗时 |
| 舵机内部控制 | D1-T 官方产品页说明具备位控、速控、力控功能 | 未找到本型号、本固件的内部伺服频率、最大有效目标更新率、编码器采样率或可保证控制带宽 |

保存源码的 `Timer_` 在每轮执行 `sleep_for(interval_)` 后调用回调，因此角度扫描周期约为 `100 ms + callback 耗时`，不是截止时间式固定周期。`pubServoAngle_callback()` 内按轴调用 `queryRawAngle()`。命令订阅通过另一个回调处理，不能把反馈 Timer 的设置当成命令处理上限。

- [Timer 实现，第 47 行](/media/aup/UbuntuData/ACT训练/04_执行日志与原始采集/D1_Control_logs/gripper_recovery_20261023_201607/controller_patch/marm_controller_node.cpp:47)
- [逐轴查询，第 105 行](/media/aup/UbuntuData/ACT训练/04_执行日志与原始采集/D1_Control_logs/gripper_recovery_20261023_201607/controller_patch/marm_controller_node.cpp:105)
- [命令订阅与 100 ms 配置，第 247 行](/media/aup/UbuntuData/ACT训练/04_执行日志与原始采集/D1_Control_logs/gripper_recovery_20261023_201607/controller_patch/marm_controller_node.cpp:247)

这些是保存源码的证据，不是本轮对当前机载二进制的部署核验。综合状态话题与角度话题也不能简单合并计数，称为更高频的独立关节采样。

## 队列长度 10 不是 10 Hz

`InitChannel(callback, 10)` 的第二个参数在宇树官方 SDK2 中明确是 `int64_t queuelen`，并存入 `mQueueLen`。它控制接收队列长度，不规定控制周期。

- [宇树官方 channel_subscriber.hpp，第 19 行](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/channel/channel_subscriber.hpp#L19)
- [保存的机载订阅调用，第 248 行](/media/aup/UbuntuData/ACT训练/04_执行日志与原始采集/D1_Control_logs/gripper_recovery_20261023_201607/controller_patch/marm_controller_node.cpp:248)

官方 D1 SDK 的反馈示例通过异步订阅回调打印消息；主线程的 `sleep(10)` 只保持程序存活，不是反馈采样率或发布率。

- [反馈示例订阅和主线程，第 26 行](/media/aup/UbuntuData/ACT训练/02_程序与源码/D1/D1_SDK/src/get_arm_joint_angle.cpp:26)

## delay 与 ACK 的证据边界

官方 D1 示例通过 `rt/arm_Command` 发送单次 JSON 位置命令，包含 `delay_ms=0`。消息定义把 `delay_ms` 表示为有符号 16 位整数，但该字段本身没有定义最高更新率，也没有绝对开始时间、速度、加速度或完整轨迹缓存协议。

- [SDK 单关节示例，第 17 行](/media/aup/UbuntuData/ACT训练/02_程序与源码/D1/D1_SDK/src/joint_angle_control.cpp:17)
- [消息字段，第 25 行](/media/aup/UbuntuData/ACT训练/02_程序与源码/D1/D1_SDK/src/msg/SetServoAngle_.hpp:25)
- [SDK 多关节示例，第 17 行](/media/aup/UbuntuData/ACT训练/02_程序与源码/D1/D1_SDK/src/multiple_joint_angle_control.cpp:17)
- [厂商域名的 D1 SDK 分发地址](https://unitree-firmware.oss-cn-hangzhou.aliyuncs.com/tool/d1_sdk.zip)
- [本地保存的下载清单和 SHA256](/media/aup/UbuntuData/ACT训练/06_技术文档与交接/Piper_LeRobot_ACT技术评估_20261027/_evidence/d1_ecosystem/official_sdk_zip_inventory.json)

保存的机载控制回调将 `delay_ms=0` 改为 1，根据当前角差计算并限制速度、加速度，再计算 `testdelay_ms` 传给 `setRawAngle()`。所以 `delay_ms=1` 不能解释成“1 ms 控制周期”，也不能假定它就是最终运动用时。此代码还存在零角差计算风险；不能靠反复发送相同目标测吞吐。

- [时间参数重计算，第 182 行](/media/aup/UbuntuData/ACT训练/04_执行日志与原始采集/D1_Control_logs/gripper_recovery_20261023_201607/controller_patch/marm_controller_node.cpp:182)
- [向舵机传入重算时间，第 207 行](/media/aup/UbuntuData/ACT训练/04_执行日志与原始采集/D1_Control_logs/gripper_recovery_20261023_201607/controller_patch/marm_controller_node.cpp:207)

保存的通信节点对单关节命令先转发内部 DDS 消息，再返回接收回执。这里的 ACK 不能证明舵机已采用目标、已到位或与其他轴同步。逐轴收齐 ACK 也不能生成“同步执行”证据；应分别记录发送时差、回执时差和实际状态，并明确缺少驱动采用时间。

- [内部转发与接收回执，第 153 行](/media/aup/UbuntuData/ACT训练/04_执行日志与原始采集/D1_Control_logs/gripper_recovery_20261023_201607/controller_source/marm_communication_node.cpp:153)

## 官方 digital_servo 不能直接视为本机驱动

宇树 `digital_servo` 官方仓库明确面向 J288/S288，给出 6 Mbps 半双工 TTL、20 字节控制包与 26 字节反馈包。该波特率既不是整臂控制频率，也没有证明这台 D1-T 使用相同舵机与协议。保存的本机代码使用 `FSUS_Protocol` 和 `FSUS_Servo`；目前没有已核验的适配关系。

- [官方 digital_servo README](https://github.com/unitreerobotics/digital_servo)
- [官方 J288/S288 协议](https://github.com/unitreerobotics/digital_servo/blob/main/specs/protocol.md)
- [本机保存代码的 FSUS 类，第 36 行](/media/aup/UbuntuData/ACT训练/04_执行日志与原始采集/D1_Control_logs/gripper_recovery_20261023_201607/controller_patch/marm_controller_node.cpp:36)
- [D1-T 官方产品页](https://www.unitree.com/D1-T/)

本次已查的 D1-T 产品页没有 Hz 或 frequency 指标。其他型号、社区工程或厂商独立舵机的参数不能替代本型号、本固件的接口合同。

## 静态残差如何界定验收

1. 先区分动态滞后与静态残差。运动中误差较大、停止后收敛，可能是时序或带宽问题；固定目标保持后，真实位置仍稳定偏离，是另一项验收结果。两者需要分别报告，不能通过相位对齐或时间延长抹去终点误差。
2. 稳态角度反馈只证明软件读数稳定，不能独立证明输出轴准确。应确认单位、映射、零位、反馈有效性，以及固定目标被正确送入相应接口；若要给硬件定位精度结论，还需要独立物理测量和明确的测量不确定度。
3. 相同固定目标在厂商原生接口下也存在稳定残差，才支持把问题范围收缩到驱动/反馈/伺服/机构/负载链。它仍不能单独区分硬件损坏、摩擦死区、回差、固件行为或标定误差。若只有应用执行路径出现残差，应先排查应用与传输实现。
4. 硬件验收应先固定姿态、负载、供电、目标范围、接近方向、稳定窗口、重复次数和误差限，再评价最终误差、重复性与方向依赖性。阈值来自厂家适用规格及任务误差预算；没有可靠规格时，应写“达到/未达到本项目要求”，不能伪称符合/不符合厂家标准。
5. 当前的停止阈值、轨迹容差和最终抓取精度是不同概念。历史文档明确说明某次 1° 是测试中止门槛，不是精度合格线；不能把它当作 D1-T 的标准精度。提高 Hz、重复发相同目标、放宽门槛或写入经验零偏，都不构成静态精度通过的证据。

- [项目文档关于中止门槛与精度的区别](/media/aup/UbuntuData/ACT训练/06_技术文档与交接/D1_Control项目文档/README.md:197)

在硬件能力尚未核验前，软件可交付的边界是：保留统一计划时间、量化逐轴发送差异、限制可实现的共同推进、可靠拒绝反馈故障、如实报告未到位。不能把这些软件改进表述为已经解决机载伺服或机械残差。
