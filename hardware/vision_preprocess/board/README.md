# KR260 VPR1 板级集成与实测包

2026-10-27：PS、双向 AXI DMA、VPR1 缩放核及原压力 GPIO 已完成综合、布局布线和 BIT/XSA/DTBO 生成。目标 6.8.0-1036-xilinx 内核驱动已编译、加载并实际运行，19 组共 380 帧 DMA 测试均通过，逐字节比较 315,598,240 字节，透传和缩放零误差。完整路径比较已经完成；测试驱动已卸载，原 `k26-starter-kits` 已恢复并核实 FPGA 处于 operating 状态。实际结果见 `reports/vision_efficiency_latest.json` 和 `vision_preprocess/board/evidence/target_v4/deployment_result.json`。

2026-10-27 连接已恢复：KR260 为 10.42.0.55，Linux 6.8.0-1036-xilinx。重启后基础固件为 k26-starter-kits。历史连接检查确认两台 Orbbec DaBai DC1 直接接入 KR260 USB，当前测试使用冻结帧，不假定重启后的相机服务仍在运行。`ubuntu` 用户的 sudo 需要交互密码，本工具不读取或收集密码。目标机已安装编译器、匹配内核头文件、NumPy 和 Pillow。

## 硬件结构与已完成的检查

路径为 KR260 DDR → AXI DMA MM2S → VPR1 → AXI DMA S2MM → KR260 DDR。主机软件默认路径保持独立。帧号、比例、wall-clock 时延由软件管理，不要求相机先将图像送到电脑再回传 FPGA。

- 器件：xck26-sfvc784-2LV-c，沿用固定版本官方 SOM board preset。
- 时钟：C3 25 MHz；风扇 A12 固定低。没有 Linux PS 时钟改频或 PS 初始化脚本。
- 控制：HPM0 128 bit，经 SmartConnect 到 GPIO、DMA 和 VPR1。GPIO 0xA0000000、DMA 0xA0040000、VPR1 0xA0050000；这些地址已经由 Vivado 回读。
- 数据：DMA/流均 32 bit，SmartConnect 转为 HP0 128 bit；40 bit DMA 地址，26 bit 长度，simple mode、双向各单通道、16 beat burst、无 DRE。内核分配的页对齐 DMA 缓冲满足地址对齐要求。
- 原压力输入仍为 AB14、Y14，两级同步、input-only。复位沿用已修正的 polarity，辅助及调试复位固定 0，AFI 节点拥有 fabric reset 116–119。
- 尺寸：编译最大 1920×1080；运行时各轴 1≤输出≤输入≤编译上限；透传要求输入输出一致。像素及寄存器 ABI 同上层 VPR1 README。
- 实现结果：4900 LUT、7637 FF、2 BRAM、0 DSP。25 MHz 下 WNS +34.783 ns、WHS +0.010 ns；无 Error/Critical DRC。保留 2 条 vendor FIFO BRAM 碰撞提示和 1 条 SmartConnect 无可布线负载警告，没有降级或屏蔽。
- CDC 报告只列出两路压力输入同步器，均有 ASYNC_REG。实现后管脚回读为 C3/A12/AB14/Y14。
- 此前处理核 109 帧、2,514,355 输出像素和 240 项原基线均已通过；本次另有真实 DMA、驱动状态、像素计数和原始输出文件证据。380 帧的 core/DMA 状态及 19 个输出文件哈希均已复核。

首次复用的旧视觉会话在综合期间触及隔离内存上限并被 OOM killer 终止。已保留日志和中断产物；随后在可用 RAM 超过 8 GiB 时启动干净会话，保留相同 5 GiB RAM + 1 GiB swap scope，完成构建。构建始终 2 个线程、1 个 job，各阶段检查 6 GiB 可用 RAM；原压力工程会话未关闭，新会话也保持打开。

## 板端 DMA 驱动

`driver/vpr_dma.c` 是此原型的单帧专用驱动，拥有两个 MMIO 资源和两个 1920×1080×4 字节缓冲。使用 Linux `dma_alloc_coherent` / `dma_mmap_coherent`；HP0 本身为非一致性通路，DT 不设置 `dma-coherent`。用户程序不猜测物理地址，不使用 `/dev/mem` 或任意保留地址。该 DT 实例由本驱动独占，不同时绑定 generic xilinx_dma 驱动。

每帧先复位 DMA 和核、启动 S2MM，再启动 MM2S；等待核心完成后再次确认两方向 DMA idle、实际 RX 字节数、核心错误位和输入输出计数。硬件轮询有超时；异常时复位 DMA 并 abort 核。重置、配置、提交和等待都计入 DMA 阶段耗时。驱动仅允许一个打开者，设备权限默认 root 0600。

## 恢复连接后执行

先确认板卡供电和有线连接恢复，再将本交付目录复制到 KR260 用户目录。它只部署独立测试，不修改相机或机械臂程序，不执行机械臂动作。

板端管理员在终端交互执行：

```bash
cd /home/ubuntu/kr260-vision-board-20261027-v4
sudo python3 software/admin_test.py --repeats 20 --serve-seconds 600
```

`--install-deps` 只安装本测试需要的编译器、与运行内核相同版本的 headers、NumPy、Pillow。可以在这些依赖已具备时省略。驱动编译发生在切换 PL 之前；编译失败时保留旧固件。

脚本先检查完整包哈希、板卡型号和当前 slot-0 应用。只允许暂时替换已知 kr260-fsr 或 k26-starter-kits；未知设计拒绝替换。它会停止明确匹配的旧压力 stdin 读数进程，释放 GPIO 后切换；相机和机械臂进程不停止。测试结束或失败时默认卸载测试驱动并恢复原已知应用，记录恢复是否成功。压力采集的上位程序可能需要重新连接，不能把一段固件切换期间的读数当成连续实测。

结果保存在 `results_时间戳/`。每组输出逐字节比较并保存 RGBX 文件、哈希、DMA 状态、像素计数和每帧耗时。`hardware_results.json` 只有实际 DMA 测试通过才写 HARDWARE_PASS；`deployment_result.json` 另行记录旧应用恢复结果。真实在线相机帧可通过 `--camera-url` 接入已有服务的已确认 JPEG 端点；默认使用交付包中冻结的两张历史相机图像和可复现随机数据，不伪装为现场实时画面。

测试进入 `Benchmark ready` 后，主机使用同一已有网络连接转发输出测试端口：

```bash
ssh -i ~/.ssh/id_ed25519_kr260 -o ExitOnForwardFailure=yes -N \
  -L 127.0.0.1:18765:127.0.0.1:18765 ubuntu@10.42.0.55
# 在另一个主机终端，仓库根目录：
python3 vision_preprocess/board/software/host_benchmark.py \
  --output vision_preprocess/board/evidence/network_benchmark.json
```

服务只监听 KR260 loopback，输入始终为板端已有帧，不接受主机上传图像或物理地址。软件和 PL 返回相同 RGB888 字节格式。主机实测 request→板端计算→输出传输→主机复制的完整 wall-clock，并记录板端打包、复制和 DMA 阶段。冻结帧测试不包含相机曝光/采集或模型推理；记录这些边界，不能把模块时延当成整套视觉系统时延。只有实测完成后才能判断是否值得让主程序使用 PL。

## 复现构建

在本机仓库根目录运行 `python3 scripts/run_vision_board.py --resume reports/vision_board_latest.json` 可检查/继续同一工程；不要重建会话来重置 IP 配置尝试计数。首次建设脚本为 `create_design.tcl`，`continue_setup.tcl` 和 `finish_setup.tcl` 仅保留本次有证据的恢复步骤，不是从空工程启动的入口。所有 Vivado 命令通过 Ross MCP，禁止直接 batch Vivado 替代。

参考：[AMD AXI DMA 参数](https://docs.amd.com/r/en-US/pg021_axi_dma/User-Parameters)、[PS HP 非一致性接口](https://docs.amd.com/r/en-US/pg201-zynq-ultrascale-plus-processing-system/Slave-Interface)、[AFI binding](https://raw.githubusercontent.com/Xilinx/linux-xlnx/xlnx_rebase_v6.6_LTS/Documentation/devicetree/bindings/fpga/xlnx,afi-fpga.txt)。

## CPU/NPU 比较补充（v2）

板端加入单线程 `-O3` 原生 C 基线，输入、输出及整数 floor 映射与 PL 一致，计时包含输出分配和坐标生成。NumPy 只作为独立参考和补充基线。主机端另比较 OpenCV/原生 C 和真实 NPU 算子执行，需保存 profile 与算子分配报告；回退到 CPU 的 Resize 不算 NPU 成绩。NPU 浮点/量化转换和必要网络传输应计入端到端，编译启动与稳态分别报告。

## 已测结果：当前选择 CPU

以冻结的腕部 640×480 RGB 图像缩到 320×240、输出为独立 RGB888 缓冲为例：

| 范围 / 路径 | p50 ms | p95 ms |
|---|---:|---:|
| 板端本地原生 C，20 次 | 0.916 | 0.937 |
| 板端本地 PL，含打包、两侧复制、DMA、拆包，20 次 | 33.760 | 33.877 |
| 板端 CPU 缩放后传到主机，30 次交错 | 7.500 | 8.112 |
| 板端 PL 缩放后传到主机，30 次交错 | 40.244 | 40.857 |
| 原图传到主机，再由原生 C 缩放，30 次交错 | 16.412 | 17.612 |
| 原图传到主机，再由 NPU 缩放，含转换，30 次交错 | 30.074 | 31.806 |

网络路径使用现有有线连接上的 SSH 转发，没有主机上传帧再回传的步骤。输入在板端，所有路径返回相同 RGB888 输出；每次都检查像素一致性。全局相机冻结帧也得到相同排序。现场两台 DC1 仍接在 KR260 USB；重启后没有监听原 18760/18761 相机端口，所以本轮明确使用冻结帧，计时不包含现场曝光、采集、JPEG 解码或模型推理。实际视觉主程序若使用 JPEG/其他数据格式，需要另计相应编解码，不能直接套用这里的裸 RGB 网络耗时。

当前 25 MHz、每拍最多 1 像素的流核，消费一帧 640×480 输入至少需要 12.288 ms。实测 DMA 重置/提交/等待中位约 12.321 ms；RGBX 打包约 15.129 ms，输出 RGB 拆包约 3.713 ms。各阶段是独立中位数，不以其和冒充端到端时延。CPU 只计算输出所需采样点，因此单独缩放更适合原生 C；PL 保留为已验证的接口和基础能力，不建议把本原型接入主程序作为加速器。

主机 Ryzen AI 9 HX 370 上，同一随机帧的 100 次交错试验：原生 C 0.238 ms、OpenCV 0.206 ms、真实 NPU 含转换 13.848 ms（均为 p50、输出零误差）。这是独立 Resize 比较，不推断整网推理性能。NPU profile、VAIML 分配和 hw runner 证据已保存；冷编译和会话创建另计约 19.8 秒。

640×480→224×224 的 NPU 输出有 224 个像素、671 个字节不一致，均在输出第 119 行，采到了前一个源图像行。最大字节误差 248，平均绝对误差约 0.389；该尺寸不作为合格的 NPU 性能方案。相同尺寸的 PL 和 CPU 输出零误差。量化编译崩溃、显式 BF16 Cast 方案崩溃的日志同样保留，没有写成 NPU 成绩。

本次验证的是时延、数据一致性和接口；没有测量功率或每帧能耗。完整数据位于 `vision_preprocess/board/evidence/`、`vision_preprocess/benchmark/` 和 `reports/vision_efficiency_latest.json`。
