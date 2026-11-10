# KR260 四线 DO 采集固件

此固件把已核实的两路比较器数字输出送入 Linux AXI GPIO。它没有 ADC，也不能恢复模拟压力幅值或 kPa/N。最终构建结果以 `output/manifest.json` 及 `evidence/summary.json` 为准；`hardware_status: NOT_TESTED` 表示该版本尚未通过实板验证。

当前修订为 `r2_aux_reset_and_afi_20261025`。原 r1 的辅助复位低有效但固定接 0，真实 IP 仿真确认不能释放复位，禁止再次加载。旧源代码、固件、构建与日志保存在 `revisions/r1_aux_reset_held_20261025`。修正版显式指定外部复位低有效、辅助复位高有效；辅助复位及调试复位固定 0。AFI 驱动节点拥有 116–119 四路 fabric reset，与 AMD 官方设备树生成器保持一致。

数据路径为 J21.11/AB14（DO1）、J21.12/Y14（DO2）→ 两级触发器同步 → AXI GPIO 单通道 2 bit、全部输入 → SmartConnect → ZynqMP HPM0 → Linux `gpio-xilinx` → `board_reader.py`。0/1 保留电平含义，触发极性由应用中逐通道配置。两根信号互相独立，不保证两路压力事件的物理同时性。

本设计使用官方 KR260 board preset，并将 GPIO 地址固定到 0xA0000000，范围 64 KiB。地址必须由 Vivado 回读后写入 `address_map.json`；DTS 从该回读结果产生。整个 PL AXI 总线使用 C3 外部 25 MHz 时钟，A12 保持低电平使风扇开启。PS HPM0 接口为 128 bit，由 SmartConnect 转为 AXI GPIO 的 AXI4-Lite；overlay 的 AFIFS 配置与该宽度匹配。没有依赖 Linux 动态启用 PL0_REF 时钟。

板端只读设备树证据确认 `fpga_full` 对应 `/fpga-region`、`amba` 对应 `/axi`，reset-controller 支持 116–119 四路复位。新 overlay 将复位声明放在 `fsr-afi` 驱动节点，并使用一个外部固定时钟节点。此匹配不能代替实际加载验证；只有加载成功并检查 GPIO 电平随实际按压变化后，才能称为硬件通过。

构建入口是 `python3 build_firmware.py`。它只经随附的 `tools/ross_mcp.py` 使用已安装 AMD Ross `--stdio-bridge`，不直接执行 batch Vivado。依赖本机 Vivado 2025.2、AMD Ross 扩展及其 `vivado-ip-configurator` 技能；技能目录可用 `--skill` 指定。由 Ross 返回的唯一 session log 路径作为配置证据源。启动前要求至少 8 GiB 可用 RAM，各综合/实现/bitstream 阶段至少 6 GiB；仅新 Vivado 进程树放入验证过的 5 GiB RAM + 1 GiB swap systemd scope，2 个线程、1 个构建 job。scope 创建或读回失败即停止。所有 Vivado 会话始终保留，用户手动关闭。

运行中断后可执行 `python3 build_firmware.py --resume latest_build.json` 复用同一会话与已完成阶段。配置失败后应保留 evidence 和 IP 尝试计数；不要删除 ledger、重命名 IP 或重建会话来绕过修复上限。首次配置调试记录会包含已经修正的问题；最终阶段通过证据与这些历史记录分别保留。

验证包含：原工程 baseline 完整 240 次检查、独立两通道 CDC 完整 240 次检查，以及真实生成 reset/GPIO IP 的 44 项总线集成检查（两次复位释放、两次 TRI 写入、16 次 GPIO DATA 读取）。三个正式测试均要求唯一 TEST_PASS 以及 compile/elaborate/simulate 三份完整日志无 ERROR/FATAL。旧 r1 的预期失败对照单独保留在 `negative_r1_reset_held`，不作为通过证据。总线测试覆盖真实复位 IP、GPIO AXI4-Lite 和 CDC，不模拟运行中 PS/SmartConnect 的 Linux 集成。随后完成单任务综合、布局布线、timing summary、DRC、CDC 报告、实现后 I/O 回读、bitstream 与 XSA 导出。仿真不能模拟真实亚稳态概率、电气阈值或压力标定。

离线输出中的六个校验项为 `fsr_top.bit`、`kr260-fsr.bit.bin`、`kr260-fsr.xsa`、`kr260-fsr.dtbo`、`shell.json`、`address_map.json`。`kr260-fsr.dts` 为附带源文件。`../install_board.sh` 先校验这些哈希与设计信息，默认只安装，不自动切换当前 FPGA。重配 PL 会替换当前运行的加速器，须先按板端流程停止旧应用。不要在板卡仍运行旧固件时直接访问新的 AXI 地址。

来源：

- GPIO 参数语义：AMD PG144 [User Parameters](https://docs.amd.com/r/en-US/pg144-axi-gpio/User-Parameters)。
- AFIFS 位宽配置：AMD Linux [xlnx,afi-fpga binding](https://raw.githubusercontent.com/Xilinx/linux-xlnx/xlnx_rebase_v6.6_LTS/Documentation/devicetree/bindings/fpga/xlnx,afi-fpga.txt)。
- 复位归属：AMD [SDT 设备树生成器](https://github.com/Xilinx/system-device-tree-xlnx/blob/xlnx_rel_v2025.2/device_tree/data/device_tree.tcl#L859-L895) 与 [AFI 驱动](https://github.com/Xilinx/linux-xlnx/blob/master/drivers/fpga/xilinx-afi.c)。
- Linux GPIO binding：[xlnx,gpio-xilinx.yaml](https://kernel.googlesource.com/pub/scm/linux/kernel/git/stable/linux-stable/+/56d5e32929ee8e772922242f37cc234c437c89c0/Documentation/devicetree/bindings/gpio/xlnx%2Cgpio-xilinx.yaml)。
- 官方 board 文件版本、来源及哈希：`boards/SOURCES.json`；C3/A12 沿用本仓库已有 KR260 约束；AB14/Y14 的 SOM/载板 I/O 标准在 vendored board XML 中交叉核对。
