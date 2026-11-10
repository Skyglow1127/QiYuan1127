# KR260 高低电平触觉采集

2026-10-26：已完成真实传感器 → PL → PS → 电脑曲线及触觉 API 的采集流程。当前接入一个传感器，经用户按压、放开三次确认接在 DO1：按下为 0，松开为 1。默认启用 DO1，DO2 保留原接口但不参与接触判断。软件提供数字接触状态，不测量连续力度；尚未执行机械臂实物抓取。

## 数据经过 PL 和 PS

传感器 DO1 / DO2 → J21 → PL 两级同步 → 两位全输入 AXI GPIO（0xA0000000）→ PS AXI 访问及 Linux xgpio 驱动 → board_reader.py → SSH JSONL → app.py → 电平图、CSV、只读触觉 API。

| 模块端 | J21 物理针脚 | FPGA / PL |
| --- | --- | --- |
| VCC | 1，3.3 V | 电源 |
| GND | 6 | 地 |
| DO1 | 11 | AB14 → bit 0 |
| DO2 | 12 | Y14 → bit 1 |

这是已核实的原接口。沿用 r2_aux_reset_and_afi_20261025、25 MHz C3 时钟及配套设备树，本轮未重新综合。已通过现有部署入口恢复运行时 FPGA，得到 REAL_GPIO_ACQUISITION_PASS；压力控制器本次编号为 /dev/gpiochip3。

真实模式要求 PL AXI GPIO。启动时核实 /axi/gpio@a0000000 的 compatible、地址、宽度、全输入属性、线路名称、固件身份，以及 sysfs 与字符设备的主次设备号。chip=auto 自动匹配动态编号；找不到就失败，不改读 PS 普通 GPIO，不切换演示。检查与读取在同一个板端进程执行，映射回执保存为 .pl.json。这是运行时身份核验，不是对比特流内容的重新测量。

## 启动与使用

进入 /media/aup/UbuntuData/抓取实验/压力测试程序，运行 ./start.sh，打开 http://127.0.0.1:8765/。已有服务运行时直接打开地址。

默认 tactile_config.json：pressure-kr260、auto、offset 0/1、100 Hz、30 ms 消抖、250 ms 失效；单路 DO1、低电平触发、DO1 响应已核实。当前后台已运行真实采集。程序重新启动后需点击“开始采集”；仅打开网页不会启动机械臂。

1. 保留当前接线，使用“单路 · DO1”，点击“开始采集”（已运行时无需重复点击）。
2. 查看 DO1 原始电平和时间曲线；按下为 0，松开为 1。“接触”显示消抖后的状态。
3. “机械臂触觉输入”显示 CONTACT、NO_CONTACT 或 HOLD，数据同时写入 records/。
4. 更换接线或增加传感器时，先停止并取消对应“响应已核实”标记，重新按压确认通道及极性，再启用该路。只要求启用的通道核实；DO2 未使用不会阻塞 DO1。
5. 停止后保留曲线及 CSV；关闭网页不停止后台采样，终端 Ctrl+C 结束服务。

传感器拆除时，停止采集并取消连接标记。程序不能从一个静止高电平自动判断传感器是否仍接着。演示模式仅用于界面检查，标为 simulated，不能获得有效机械臂触觉输入。

板端部署路径为 /home/ubuntu/kr260-pressure-20261025。需要恢复 r2 时使用 authenticate_and_deploy.sh，该操作需要板端管理员身份并改变运行时 FPGA；日常采样无需重载或 sudo。采样器通过 SSH 直接发送执行。本次恢复前后两路机械臂相机服务 PID 未改变、帧数继续增长、约 29.9 fps 且无错误；未向机械臂发出运动指令。

## 机械臂只读接口

GET http://127.0.0.1:8765/api/tactile，schema 为 kr260-tactile/v1。

| 字段 | 意义 |
| --- | --- |
| source / through_pl_ps | 真实来源 / 已核实 PL → PS 映射 |
| sensor_connected | 本次声明传感器接回，不能自动诊断所有物理断线 |
| valid / fresh | 所有启用通道的接触有效 / 数据未过期 |
| channels[].enabled | 该路是否参与触觉判断；当前仅 DO1 为 true |
| channels[].level | 新鲜原始 0/1，不等同于已核实接触 |
| channels[].raw_contact | 按极性解释的原始触发 |
| channels[].contact / valid | 消抖后的接触及有效性，无效时为 null |
| receive_age_ms | 主机最后接收到响应时的间隔 |
| transport_lag_growth_ms | 相对首帧的延迟增长，非绝对端到端延迟 |
| decision | HOLD、CONTACT、NO_CONTACT |
| stop_closing | 接触或输入无效时为 true，供外部控制器使用 |
| both_contact | 双路启用时表示均稳定接触；单路为 null，不能单独证明抓稳 |
| grasp_verified / robot_motion_enabled | 固定 false，本软件不验证抓稳、不发运动指令 |

默认候选电平连续保持 30 ms 后更新稳定接触；启动、采样缺口或接收中断后重新积累稳定时间。已就绪通道的原始接触也会令 stop_closing=true，避免等待消抖时继续挤压。输入无效同样建议停止继续闭合，不把缺数据当成没接触。参数须按实际安装、抖动和运动速度验证。

NO_CONTACT 不能单独授权运动；已有控制器仍负责限位、碰撞、反馈和闭合超时。Linux 轮询可能漏短脉冲，不替代硬件急停。物理断线可能保持某个电平，DO 本身无法识别所有此类故障。

Python 接入：从 tactile_client 导入 TactileClient，在已有控制器的决策循环调用 touch.read()，检查返回的 stop_closing。客户端不会执行动作，只提供判断；连接失败、过期或重复旧样本时返回 HOLD，不复用缓存。

只读运行示例：

    python3 tactile_client.py --once
    python3 tactile_client.py --seconds 10

客户端含超时、响应大小、时间、会话与序号检查，不连接 D1 控制端口。实际抓取仍需具体目标、路径、关键参数及单独批准。

## 文件、记录和验证

| 文件 | 用途 |
| --- | --- |
| start.sh / app.py | 默认界面、记录与 HTTP API |
| board_reader.py | PS 读取，先核实 PL 设备树与字符设备 |
| tactile.py | 消抖及接触判断 |
| tactile_client.py | 机械臂程序只读接入示例 |
| tactile_config.json | 当前已核实的 DO1 单路默认参数 |
| web/index.html | 电平、接触状态及阶梯图 |
| firmware/ | 既有 r2 PL 工程及部署文件 |
| records/ / evidence/ | 原始记录、配置、映射及验证证据 |

CSV 保留原始 do1/do2、时间、极性、来源及 gap，追加 contact1/2、sensor_connected、pl_verified、transport_lag_growth_ms。消抖不覆盖原码。间隔超过 2.5 / 采样率标记 gap，曲线不跨缺口连接；历史曲线保留，过期或停止后的触觉输入失效。未核实的接触字段留空。

原连续幅值文件保留为历史，不再作为默认入口，也不继续开发。

本次软件测试 68 项通过，其中 19 项新增数字触觉测试，17 项为保留的历史连续采集测试。覆盖单路选择、传感器取出、极性、消抖、缺口、过期、队列延迟、模拟隔离、PL 拒绝与客户端失败处理。运行 python3 -m unittest discover -s tests -v。

离线回放既有实测 CSV 的 868–882 秒窗口：1400 样本、DO1 10 个原始边沿，原始值逐项保留，产生 HOLD / CONTACT / NO_CONTACT。这是历史数据软件回放，未作为实时数据发布。

r2 六个固定文件哈希及设计元数据校验通过。历史 FPGA 验证 524 项、实机快照 97,319 条 real 数字样本仍保留。

本次单路识别会话 20261026T123327_419411Z_ssh 保存了 36,753 条真实样本。DO1 低电平持续约 2.020、1.780、1.640 秒，共 6 个边沿，对应用户三次按压；DO2 一直为 1。快照为 records/20261026_DO1单路三次按压_原始实测.csv，SHA256 为 b992e6e8a7e75eec5b91351be2215eee8ab0db51a3dfb90b5a1a0b08d1727ef7。

随后切换 DO1 单路，实时 API 已确认 source=real、through_pl_ps=true、valid=true；只读客户端连续取得递增序号，松开时 NO_CONTACT。证据见 evidence/single_channel_identification_result.json、do1_single_live_start.json、tactile_client_live.json、digital_delivery_status.json 及数字触觉测试日志。三次按压原始记录与后续单路实时会话分别保存，未改写原始数据。

技术文档：/media/aup/UbuntuData/技术文档/压力传感器（FPGA)/压力传感器采集程序技术文档.docx。
