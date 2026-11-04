# KR260 DESC_ONLY 工程与冒烟检查

本文件是阶段 D 的人工执行说明。构建进展与实际报告见 REPORT.md；没有加载板卡，也没有板上验证结果。工程只有内部描述符接收器；`disable_req=1`、`motion_permit=0`，无 CAN、GPIO 或其他 fabric 引脚输出。不得把 FPGA manager 的 `operating`、只读检查通过或离线仿真称为机械臂执行成功。

## 已核对的本地输入

| 项目 | 已安装文件中的事实 |
|---|---|
| Vivado | `/home/aup/Xilinx/2025.2/Vivado`，脚本使用该安装，不安装新组件 |
| Board part | `xilinx.com:kr260_som:part0:1.1` |
| FPGA part | `xck26-sfvc784-2LV-c` |
| PS preset | `mpsoc_preset_vsom`；请求PL0=100MHz，33.333MHz参考实际派生99,999,001Hz |
| 原生 RAM | BMG `True_Dual_Port_RAM`，32768×32bit，共 128KiB；A/B 读取延迟各 1 周期 |
| 控制器 | AXI BRAM Controller `CONFIG.SINGLE_PORT_BRAM=1`，仅占 A 口；B 口归 player |

依据：[board.xml](/home/aup/Xilinx/2025.2/Vivado/data/xhub/boards/XilinxBoardStore/boards/Xilinx/kr260_som/1.1/board.xml:536)、[preset.xml](/home/aup/Xilinx/2025.2/Vivado/data/xhub/boards/XilinxBoardStore/boards/Xilinx/kr260_som/1.1/preset.xml:17)、[PL0 频率](/home/aup/Xilinx/2025.2/Vivado/data/xhub/boards/XilinxBoardStore/boards/Xilinx/kr260_som/1.1/preset.xml:316)。缺失这些输入时脚本中止，不能退化成猜测的 PS 配置。

BD 为 `PS HPM0 → SmartConnect → cfg / ownership guard → AXI BRAM Controller → RAM A`；RAM B 接 player。Controller 模式使用 BYTE_ADDRESS；player 的 17bit **字节地址**零扩展为 BMG 的 32bit 地址，不右移两位。两个口同为派生PL0时钟99,999,001Hz，B 口写数据和写使能由 RTL 固定为零。`bank_writable` 与 `write_busy` 显式连接，COMMIT 必须等待已接收 AXI 写事务 drain，不能仅凭 CPU 已发出写操作判定完成。

## 人工生成工程

以下命令在开发机运行。新输出目录必须不存在；脚本不覆盖既有工程。`--check-inputs` 只是静态检查，不启动 Vivado。

```bash
cd /media/aup/UbuntuData/PIPER机械臂/robot_rt
tclsh tcl/create_bd.tcl --check-inputs
tclsh tcl/build.tcl --help
/home/aup/Xilinx/2025.2/Vivado/bin/vivado -mode batch \
  -source tcl/build.tcl -tclargs --out /tmp/robot_rt_kr260_build --jobs 1
```

若只需生成 BD/XPR，不综合，改用 `-source tcl/create_bd.tcl`。Vivado 需要成功推断 `robot_rt_top_bd/s_axi_cfg` 与 guard 的 AXI 接口；找不到端口、地址段或预设会直接报错，需修正工程后重新选择新输出目录，不能忽略报错继续上板。

建议的 cfg 基址为 `0xA0000000`（4KiB），chunk 为 `0xA0020000`（128KiB）。这些是**新设计分配值**，不是已探测到的板上地址。脚本在 BD 校验后核实实际映射，生成 `addresses.json` 和 `address_map.txt`。guard 收到并转发完整绝对地址；控制器取 RAM 窗口内的低 17bit 字节地址。

`build.tcl` 先完成 synth/route，保留 `reports/utilization_{synth,route}.rpt`、`reports/timing_{synth,route}.rpt`、`reports/check_timing.rpt` 和 `reports/drc_route.rpt`。只有生成时钟与PS实际派生值和RTL CLOCK_HZ一致、setup/hold 非负、关键未约束检查为零且无 DRC Error 后才写 `robot_rt.bit`，再转换 `robot_rt.bit.bin`，并导出 `robot_rt.xsa`、`artifacts.sha256.json`。不能用本说明中的资源估计代替这些实际报告；112 周期内部延迟也仍须 RTL 仿真及实现后核验。

当前构建将general.maxThreads设1，默认逐个完成IP综合。模块引用core另有独立OOC约束集，实际生成脚本必须出现`read_xdc -mode out_of_context`，输入aclk目标不弱于10ns，并排除implementation；检查不满足即停止。这个约束只帮助独立综合，不会在最终顶层给core/aclk另造时钟，最终仍以PS生成时钟签核。OOC的频率元数据或脚本预检通过均不能写成STA通过。

生成 BIF 是 PL bitstream 项：

```text
all:
{
    [destination_device=pl] robot_rt.bit
}
```

转换使用 `bootgen -arch zynqmp -image robot_rt.bif -process_bitstream bin -w on`，生成的是 PL 加载文件，不是含 FSBL/PMU/内核的启动镜像。参见 AMD [destination_device](https://docs.amd.com/r/en-US/ug1283-bootgen-user-guide/destination_device)、[process_bitstream](https://docs.amd.com/r/2024.2-English/ug1283-bootgen-user-guide/process_bitstream)；时序覆盖的判断参见 [UG906 Check Timing](https://docs.amd.com/r/2025.2-English/ug906-vivado-design-analysis/Check-Timing-Section)。

## 板上平台前提

**仅加载 `.bit.bin` 不执行 PS preset、不配置 DDR/MIO、不启用 HPM0，也不保证PS输出匹配的FCLK。** KR260 Linux 的启动平台必须事先与本工程导出的 XSA 匹配，启用 HPM0、PL0的实际标称频率与addresses.json一致（本预设99,999,001Hz）并正确释放复位。要先由现场工程人员核对该平台和设备树地址占用，并停止占用原 PL 设计的驱动/应用。此仓库没有制作或安装 Linux 启动镜像、设备树 overlay，也不会自动卸载现有平台。

`fpga_manager` 的 `operating` 仅表示管理器报告 PL 已配置，不能证明 PS 时钟、地址译码或本设计 ID 正确。对未映射地址读取 `/dev/mem` 可能触发总线异常；不应在原有未知平台上试读建议地址。先核对匹配平台，再使用显式 `--read-status`。内核若禁用 `/dev/mem` 或不提供 manager `flags/firmware`，脚本中止，不修改内核策略或尝试其他编程后端。

本设计 `hard_fault=0` 是**合成测试常量**，不是已连接的硬故障反馈；`backend_ready=1` 仅表示内部只读描述符副本可抓拍，不表示 CAN 可用。这两者都不构成物理执行许可。XDC 无自创时钟、fabric pin LOC 或时序豁免；物理后端的 pin、电平、收发器和故障链路仍未布线。

后续[板端只读盘点](stage_d_board_readonly_20261102/can_interface/verification.json)确认Linux `can0`为`gs_usb`/USB `1-1.3:1.0`，当时`DOWN/STOPPED`、报告RX/TX packets均0，未取得配置bitrate字段。本次未打开SocketCAN数据socket、配置或启用接口、发送CAN帧；这一状态记录不作故障诊断，也不改变PL物理CAN后端尚未实现的边界。

## 人工加载与只读 smoke

把成功构建的整个输出目录复制到目标 KR260 后，先进行文件校验。以下仅为现场命令，本轮未执行：

```bash
bash robot_rt/sw/load.sh --check --build /path/to/robot_rt_kr260_build
sudo bash robot_rt/sw/load.sh --load --build /path/to/robot_rt_kr260_build --read-status
```

默认 `--check` 只验证构建 SHA256、地址契约并读取 manager 状态，不安装固件、不写 sysfs、不打开 `/dev/mem`。显式 `--load` 才把校验后的固件写入 `/lib/firmware`，设置 full-bitstream flags=0 并写 manager 的 firmware 属性；随后等待 `operating`。`--firmware-dir` 仅用于已由该 Linux 固件查找路径配置支持的目录，不会自动修改查找路径。AMD 的加载接口见 [ZynqMP PL Programming](https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/18841847/Solution+ZynqMP+PL+Programming)，管理器状态定义见 [Linux FPGA manager](https://cdn.kernel.org/doc/html/latest/driver-api/fpga/fpga-mgr.html)。

额外 `--read-status` 以 `O_RDONLY`、`PROT_READ` 映射 cfg，验证 ID=`0x5254504c` 和 CAPS bit0 DESC_ONLY，核对RO NOMINAL_CLOCK_HZ（0xa8/ac）与构建manifest一致，输出 STATE、STATUS、FAULT/REJECT、CYCLE、TICK_COUNT 和已有 SNAP_* 计数。64bit 活计数用 high/low/high 重读；CYCLE 应持续增加。脚本不写 `SNAPSHOT`，所以 SNAP_* 可能仍为旧值。复位后尚未显式配置时 TICK_COUNT=0、描述符全零均属预期，不能当 tick 或 player 已通过验收。

阶段 D 增加的 `0x600..0x65f` 是 **RO 描述符快照**：只有显式 `SNAPSHOT` 才与计数器一起锁存最近一组96B描述符，之后保持到下一次快照，不是执行FIFO，也不代表保留全部样本。load脚本不写SNAPSHOT，只打印已经存在的快照；尚未快照时可以全零，不能据此判断没有生成过描述符。

`sw/pl_bank.py`负责离线打包；新增`sw/board_desc_smoke.py`提供默认只检查、显式`--run`才写寄存器的DESC_ONLY冒烟入口。后者仍未在真实PL上执行：必须先完成原构建签核和匹配的PS平台加载，只允许明确虚构的静止输入，不接CAN或机器人。原`sw/load.sh`继续只读状态，不会自动调用该入口。

记录现场的板卡型号、启动平台/XSA 哈希、构建 manifest、Vivado timing/utilization、manager state 和只读输出后，才能报告相应检查结果。100MHz 的 112/120 周期分别约为 1.12/1.2µs，只指内部描述符边界；CAN 总线四帧传输及机械响应时延另见 [TRANSPORT_NOTES.md](../spec/TRANSPORT_NOTES.md)，不能混同。

## Vivado 2025.2模块引用说明

BD不直接引用SystemVerilog顶层文件；使用纯Verilog-2001薄封装 `robot_rt_top_bd.v` / `robot_rt_axi_guard_bd.v`，内部实例化原SV。RAM B以完整PLAYER_BRAM接口独占连接，32bit字节地址由原17bit零扩；PS的AWUSER/ARUSER由guard封装显式接收并丢弃，不参与所有权或地址判断。复位极性由PS接口派生，验证后断言，不能写只读派生参数。

## 内存受限时的综合调度

本机多轮实际综合触发内存保护后，新增controller退出后再运行worker的方式；小工程验证通过，主工程状态见REPORT，使用边界与命令见[EXTERNAL_BUILD](EXTERNAL_BUILD.md)。该方式只更换构建调度，不绕过原约束、最终STA/DRC或加载前检查。


## D5 纯离线事务计划与新快照解析

`sw/desc_smoke.py`提供可复核的离线计划/解析闭环；新增板端入口见本页下一节。它只读普通 JSON 文件、生成新目录内的 bank 与事务说明；不打开 `/dev/mem`、不导入机器人/相机/CAN 后端，也不执行输出中的 CMD。当前实现尚未通过完整工程容量/时序验收，不能用此工具绕过构建和板上平台前提。

以下是虚构静止 seed 的纯离线示例。`275/2 Hz` 是明确的设计参数，`1000` 等为示例 PL 周期，不是实测 `f_exec`、现场反馈或主机时间映射。所有频率、源时间、初态和限值来自显式 `pl_bank` 输入；工具没有执行频率默认值。输出目录必须不存在。

```bash
cd /media/aup/UbuntuData/PIPER机械臂/robot_rt
python3 sw/desc_smoke.py --help
python3 sw/pl_bank.py --design-example --clock-hz 99999001 \
  --f-num 275 --f-den 2 --origin-cycle 1000 --first-index 8 \
  --output-dir /tmp/desc_smoke_input
python3 sw/desc_smoke.py plan --input /tmp/desc_smoke_input/input.json \
  --arm-by-cycle 2000 --begin-cycle 2200 --end-cycle 9000000 \
  --output-dir /tmp/desc_smoke_plan
python3 sw/desc_smoke.py verify --plan /tmp/desc_smoke_plan/plan.json \
  --capture /path/to/exported_capture.json --output /tmp/desc_smoke_result.json
```

`plan.json` 复用 `sw/pl_bank.py` 的整数打包和 CRC，绑定工具/规范 SHA，依次列明：身份、DESC_ONLY 与标称时钟核对 → 静态配置和独立 seed → CONFIG_COMMIT → 读回实际 CYCLE_ORIGIN → 先 payload 后 header → Device-memory 屏障及读回 → bank sequence/COMMIT 并等待 READY → 独立 KICK → ARM → 首 tick 前 STOP → begin/end 两次 SNAPSHOT → DISARM 并核对空闲状态。仅接受首 bank、静止初态和零 jerk 的冒烟记录，不生成实际运动路径。

CONFIG_COMMIT 会重新锁存 DDS origin。示例 origin 不能用于盲目重放：外部实现必须在提交后读回实际 origin；不符就停止并基于该真实值重新打包后续 bank，不能再次 CONFIG_COMMIT 后继续使用旧 bank。`validation_at_cycle` 是输入中的显式 PL 周期假设，不能拿 PC monotonic 纳秒替代；该工具不会建立物理时钟映射。ARM 上界及两个快照目标也都用 PL 周期。目标周期仅供审查，解析器按真实锁存周期计算，不能假定 Linux 恰好在某一指定拍完成事务。

`capture_template.json` 是不可通过的空模板，必须由独立采集方填写。捕获契约要求 `source_kind=synthetic|external_capture`、来源说明、run ID、对应 plan ID、五个前置 CMD 的 OKAY 回执和按序递增的活 CYCLE 边界。每次快照需先读活 CYCLE、写 SNAPSHOT 并确认 OKAY、读取模板所列全部32bit寄存器词，再读活 CYCLE；`command_ack` 的周期边界可直接使用这两次读数。要求 `before < SNAP_CYCLE < after` 且两次区间互不重叠，禁止在中间 reset、重配或发出其他 SNAPSHOT。只给 `load.sh` 的旧 SNAP_*、省略命令回执、复用旧锁存或模板 null 均拒绝。采集方须保留原始顺序记录，该 JSON 不具备硬件来源认证能力。

解析器先按实际 ARM 回执的保守区间检查 seed/source 非未来及未过期：seed 和 produced+map_error 不晚于区间起点，区间终点严格早于 seed+source_age 和 normal_deadline−map_error。COMMIT 回执不等于异步 validator 完成时刻，结果不声称已认证 READY 发生时间。然后核对相同 session/config/原点/频率/时钟，begin 在首 tick 前、end 已包含完整 brake 尾；用两次实际 SNAP_CYCLE 与 DDS 整数公式计算 tick/emit 差及名义时钟计数率，不使用主机等待秒数冒充计数来源。它还核对零拒绝/故障/背压/缺块、量化事件数、最终96B描述符及 reported latency min=max=112，最后要求 DISARM 回执与 DISARMED/free/unconfigured 读回。latency 仅核对快照报告的极值，不能替代独立逐事件边沿测量；计数率也不是实际振荡器、CAN 或电机频率。成功结果名为 `offline_capture_consistent`，始终保持 `board_acceptance=false`、`physical_f_exec_hz=null`。空模板或独立 synthetic fixture 通过不等于板上验收。


## D5 板端静止描述符入口（真实PL尚未运行）

`sw/board_desc_smoke.py`默认`--check`，只校验真实成功构建的manifest、四个产物SHA、地址和显式静止输入，输出目录必须为新目录。没有成功构建文件时直接拒绝，不生成假bit或假签核结果。`--run`额外要求little-endian Linux/AArch64、root、现有cc、manager operating、明确确认匹配PS平台的manifest SHA及Device MMIO映射；这两个确认参数是操作人的声明，不能代替前述时钟、AFI、地址和复位核验。当前板sudo需现场输入密码。

输入为`pl_bank.py`显式design-only JSON，另须`source_kind="synthetic_stationary"`，只允许两条normal和两条brake、零v/a/jerk、previous_sequence=0、同PL时基及零映射误差。没有默认执行频率。再次运行必须给新的显式session_nonce：RTL保留last_nonce，不能重复使用旧会话；程序不自动刷新真实反馈或旧策略TTL。

执行顺序是：只写静态配置及CRC并清零其保留字 → 只CONFIG一次 → 读取真实origin/current cycle → 在同一配置下生成带明确synthetic标签的seed/source时刻和未来bank → 写seed及其CRC → RAM payload、header、Device屏障和逐词读回 → sequence/COMMIT并等待READY → KICK/ARM/首tick前STOP → 两次新SNAPSHOT → DISARM。后半段不会再CONFIG；输入的绝对时间字段会明确替换为本次虚构静止测试的PL时间，不能用于真实反馈重打时间戳。异常后尽力DISARM；原硬故障、来源、lease、限值和112拍路径未放宽。

命令模板如下，当前未进行实际PL运行；输入的时钟/设计率、lead、window和horizon须彼此兼容。例如99,999,001Hz名义clock下，20,000,000/100,000,000拍约0.2/1秒，不是实测机械臂频率。

```bash
python3 sw/board_desc_smoke.py --help
python3 sw/board_desc_smoke.py --check --build /REAL/VERIFIED_BUILD \
  --input /path/to/explicit_synthetic_stationary.json \
  --lead-cycles 20000000 --window-cycles 100000000 --output-dir /NEW/check
# 匹配平台加载和权限准备完成后，才在KR260运行：
sudo python3 sw/board_desc_smoke.py --run --build /REAL/VERIFIED_BUILD \
  --input /path/to/explicit_synthetic_stationary.json \
  --lead-cycles 20000000 --window-cycles 100000000 --output-dir /NEW/run \
  --platform-manifest-sha256 ACTUAL_MANIFEST_SHA --device-mmio-mapping
```

native后端用现有cc编译小型volatile LE32 + `dsb sy`辅助代码；不安装组件，也不用`mmap.flush`冒充Device屏障。实际SO与源SHA、编译命令和输出保留。回执`kind=mmio_readback`记录完成后的状态/故障/拒绝码和PL周期区间，**不填写或推断AXI BRESP**；离线解析器仍支持原始raw_axi回执，并分别报告观测范围。当前17项新旧定向测试通过，使用fake-MMIO，见[原始证据](stage_d_board_desc_smoke_20261102/verification.json)；辅助C代码另已在真实KR260现有cc13.3.0下编译，并只在普通进程RAM通过8个32bit模式检查，没有打开`/dev/mem`，不是PL寄存器测试。

采集结果保存实际配置读回、重包输入、bank、计划、命令回执、原始快照词及清理结果。按真实SNAP_CYCLE核对DDS计数和reported min=max112；仍不认证物理时钟、CAN、机械臂动作或独立逐事件延迟。当前设计没有导出dbg引脚，示波器测量/独立内部探针仍TODO(run)，不能编造管脚或用自报极值替代测量。完整PL构建与上板结果见REPORT。
