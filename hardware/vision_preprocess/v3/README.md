# KR260 VPR3：π0.5 图像预处理

VPR3 为现有 LeRobot π0.5 提供可选择的图像预处理路径。PL 执行 Vitis Vision 双线性 resize、居中补边和紧凑 RGB888 打包；主机管理相机角色、帧号、缩放参数和时间信息。π0.5 权重、微调、动作推理仍使用原来的主机计算路径。本模块不控制机械臂。

2026-11-01 已完成 KR260 实物和 ROS 2 回放验证。功能通过，但当前640×480→224×224任务中，PL含复制和DMA的处理比板端PS慢约75%，ROS交付总延迟增加约13%–16%。当前主机相机和π0.5生产路径继续使用CPU。完整实测见文末；实时相机、NPU和策略动作回归尚未测试。

## 数据路径与边界

```text
已有图像源 → PS/主机解码 → RGB888 → C 打包 RGBX32 → DMA → Vitis resize + border
                                                           → PL RGB888打包
π0.5 官方 Camera/processor ← owned RGB888 ← 一次连续复制 ← DMA ←┘
```

优先在图像已位于 KR260 的路径上使用 PL。若 USB 摄像头在主机，保留 CPU 路径，先测量增加的网络往返再决定部署。服务仅复用显式配置的已有相机 URL，不移动 USB 设备、重新标定或改变生产相机服务。

| 项目 | 合同 |
|---|---|
| 器件与时钟 | xck26-sfvc784-2LV-c；C3 25 MHz，经 MMCM 到 100 MHz |
| 编译时最大输入 | 1920×1080，RGBX32 AXIS |
| 编译时最大画布 | 640×640，RGB888；32-bit AXIS 带逐字节 TKEEP |
| 运行时输入尺寸 | 宽16..1920、高16..1080 |
| 运行时输出画布 | 两轴16..640；实际内容两轴至少2 |
| 缩小限制 | 每轴最多4倍；1920→224不支持，明确报错 |
| 默认 π0.5 几何 | 224×224；ratio=max(iw/224,ih/224)，内容尺寸int截断，居中黑边0 |
| 普通输出字节数 | ow×oh×3；224²为150,528字节，比VPR2 RGBX少25% |
| 原样旁路 | 保留全部32位；输入输出同尺寸，字节数iw×ih×4 |
| ABI | tag 0x56505233；0x34=1；/dev/vpr3dma；ioctl 0xc0405603 |
| 计数器 | 输入/输出计数均为像素数；done包括最后一个紧凑输出beat被接收 |

寄存器沿用 VPR2：0x00启动/中止，0x04状态，0x08输入宽高，0x0c画布宽高，0x10旁路，0x14错误，0x18/0x1c像素计数，0x20输入上限，0x24版本，0x28内容宽高，0x2c填充值，0x30画布上限，0x34紧凑输出能力。宽在低16位，高在高16位。

`rtl/vpr3_stream.sv` 复用原 `../v2/rtl/vpr2_stream.sv` 和已经综合的 Vitis HLS RTL；不重新声明 HLS cosimulation 通过。`rtl/rgb888_axis_packer.sv` 处理1..3个尾部有效字节、反压及abort。驱动使用 Linux coherent DMA API，接收先于发送启动，并有状态、字节数和超时检查。

DMA缓冲起始地址保持对齐。AMD PG021 对关闭DRE时限制的是起始地址对齐；S2MM_LENGTH按字节配置且必须覆盖接收包。2026-11-01板测已确认3种非整word输出长度，每种20帧，接收字节数及图像正确。[AMD AXI DMA PG021，第71–72、81页](https://www.amd.com/content/dam/xilinx/support/documents/ip_documentation/axi_dma/v7_1/pg021_axi_dma.pdf)

## π0.5 接入

正式采集仍使用原相机和官方 `lerobot-record`，保持原始数据集。下例是可选择的推理相机适配接口；不修改官方 LeRobot 包：

```python
import sys
sys.path.insert(0, "/home/aup/文档/ChatGPT/VIVADO/vision_preprocess/v3/software")
from pi05_adapter import Pi05Camera

# existing_camera 是当前项目已经配置好的官方 Camera 实例。
# 默认软件路径；保持原相机名称和观测字典键。
camera = Pi05Camera(existing_camera, backend="software")
# 完全原生路径：backend="raw"；或 existing_camera/read_raw() 供原始采集。

# 仅在 KR260 已加载匹配固件、驱动且当前图像本就在板端时使用：
# from transport import PL
# transport = PL()
# camera = Pi05Camera(existing_camera, backend="pl", transport=transport)
```

输出为 HWC uint8 RGB。官方 processor 执行一次 `/255`，π0.5 再执行一次 `*2-1`。输出已为224×224时，官方模型跳过内部resize。Camera接口会引入 uint8 量化，因此并不与原模型浮点resize逐位相同；量化路径保留独立质量评估。适配层没有偷偷回退到CPU，PL失败会明确报错。现有主程序的启动配置尚需在真实连通后做集成验收，不能把离线API测试当成策略闭环完成。

## 重建与验证

从工作区 `/home/aup/文档/ChatGPT/VIVADO` 运行：

```bash
python3 scripts/run_vpr3_board.py
# 中断或失败后继续同一个尚存的专用会话：
python3 scripts/run_vpr3_board.py --resume reports/vpr3_board_latest.json
python3 scripts/package_vpr3.py
```

VS Code Tasks 中对应 `Ross: Build VPR3 Pi05 Board`、`Ross: Resume VPR3 Pi05 Board`、`VPR3: Package Verified Board Firmware`。构建命令通过 AMD Ross MCP stdio bridge 调用 Vivado 2025.2，Tcl模式；启动8 GiB、综合/实现6 GiB可用内存门槛，专用进程树8 GiB RAM＋1 GiB swap限制，单job、两线程。任务结束后保留 Vivado 会话。

依赖当前工作区已经冻结的 Vitis HLS 输出和 `build/vpr3_pi05_vectors_20261031`；重建脚本记录其哈希。独立工作站需迁移这些依赖，板端测试包不是独立 Vivado SDK。

验证包含：37个packer帧；旧14例和π0.5新14例；4种尾部模式、2个32位旁路帧、invalid/abort/早发下一帧，总计34个完整全核帧。必须出现完整TEST_PASS且编译、展开、仿真日志无error/fatal。π0.5质量要求uint8参考max≤2、MAE≤0.5；官方float参考max≤2.5字节等效值、MAE≤1，填充精确。模型动作回归单独标记为NOT_RUN。

最新状态和完整证据索引：`reports/vpr3_board_latest.json`、`reports/vpr3_software_latest.json`、`reports/vpr3_delivery_latest.json`。综合完成、时序收敛、硬件通过分别记录，不能互相替代。

## 板端与效率对比

生成的 `vision_preprocess/delivery/kr260-vpr3-日期-v1` 包含匹配固件、驱动源码、测试程序、冻结float参考和历史JPEG。拷到板端后，在**板端终端**运行：

```bash
sudo python3 software/admin_test.py --repeats 20 --saved-images --serve-seconds 600
```

命令在包目录下执行。去掉 `--saved-images` 才读取已有18760/18761相机服务；可用 `--wrist-url`、`--global-url`显式覆盖。默认不安装依赖，确实缺少时可加 `--install-deps`。脚本核对板卡型号和全部包哈希、编译板端驱动、检查PL占用、暂时加载固件，退出时恢复原应用；不会停止其他应用，遇到PL占用即退出。需要系统sudo认证时只在终端输入密码。

板端报告区分：原样数据一致性、官方浮点/uint8图像误差、输入打包、DMA reset/提交/等待、输出复制、处理总延迟、CPU占用和顺序吞吐率。PS基线是板上单线程OpenCV，明确区别于主机PyTorch/模型推理。ROS节点另外测量JPEG取图/解码、网络传输、消息发布和接收的完整路径。历史静态JPEG与实时相机单独标记。

主机 ROS 2 Jazzy 的重建与测试入口（板端出现 `Benchmark ready` 后运行对比）：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base build/vpr3_ros_logs build --base-paths vision_preprocess/v3/ros2 \
  --build-base build/vpr3_ros_build --install-base build/vpr3_ros_install
source build/vpr3_ros_install/setup.bash
python3 scripts/benchmark_vpr3_ros.py \
  --package /home/ubuntu/kr260-vpr3-20261031-v1 --source recorded
# 实时模式必须与板端admin的source一致，使用 --source live。
```

默认两轮交替PS/PL顺序，两个相机角色每种路径各60帧/轮。命令自建SSH loopback隧道，不改现有相机连接。结束后通过测试包的stop文件请求admin正常退出，并检查原固件已恢复。`--url`模式可以测试已有服务，软件单路径须显式 `--backends software`，不得冒充PS/PL对比。

缩小输出和减少软件解包开销是本版的优化机制；实际收益以同一输入、同一尺寸、同一设备的实测为准。不能把像素测试通过等同于机器人动作成功率保持。

## 2026-11-01 实物验证结果

实物为KR260 revB，内核6.8.0-1036-xilinx。主机10.42.0.1的现有`KR260-Shared`连接到板端USB RTL8153网卡`enx00e04c4d19a8`（10.42.0.55）。复用已连通配置，未更改其他网口、现有相机服务或生产程序。两路USB相机仍在主机；本次板端使用预先保存的global/wrist图像，**不是实时相机测试**。

- 硬件380个缩放帧（14个官方参考、3个尾字节、2个JPEG配置，各20帧）及60个旁路帧通过。旁路累计190,530,000字节完全一致。
- 官方uint8参考最大误差2，最差画布MAE 0.5；官方float参考最大误差0.00647933（归一化到0..1），最差MAE 0.00199115。黑边精确。实际保存的RGB文件长度、哈希及图像误差另行复核一致。
- ROS两轮交换PS/PL顺序，共8次运行、480/480条消息；每角色、每后端120条。相同角色的输入JPEG SHA跨后端一致，每帧输出150,528字节。
- 测试结束已恢复`k26-starter-kits`；独立SSH复查确认VPR3驱动和设备节点消失、ready文件删除，FPGA状态为`operating`。

板内预处理延迟p50，从内存RGB输入到独立RGB输出；包含PL输入打包、DMA和输出复制，不含JPEG解码、网络和策略推理。两种CPU均为单线程OpenCV，主机版本4.13.0、板端版本4.11.0，属于实际部署环境比较。

| 640×480→224×224 | 主机CPU | KR260 PS CPU | KR260 PL |
|---|---:|---:|---:|
| global | 0.0491 ms | 2.8760 ms | 5.0478 ms |
| wrist | 0.0489 ms | 2.8815 ms | 5.0540 ms |

PL中位阶段耗时约：输入打包0.76–0.77ms、DMA reset/提交/等待3.26ms、输出复制0.95ms，其余为调用开销。DMA阶段包含传输和内核执行，不是单独计算核时间。PL进程CPU时间约3.53ms，PS约2.82ms，本轮也未减少CPU时间。19个配置中仅640×480→416×416的一个合成案例板内获得1.041倍小幅收益，不能外推为224²或完整链路加速。

ROS计时从主机请求到订阅端收到图像，包括板端取文件、JPEG解码、复制/DMA、HTTP/SSH网络传输及ROS；排除预先上传文件、相机曝光和模型推理。吞吐率取实际订阅时间间隔，表示保存图像回放消息率。

| 角色 | PS总延迟 p50 / p95 | PL总延迟 p50 / p95 | PS消息/s（两轮范围） | PL消息/s（两轮范围） |
|---|---:|---:|---:|---:|
| global | 21.88 / 25.22 ms | 24.66 / 26.55 ms | 43.83–45.95 | 39.19–40.66 |
| wrist | 20.82 / 22.62 ms | 24.24 / 63.40 ms | 46.97–47.75 | 37.80–38.84 |

腕部PL的尾延迟仍包含网络/HTTP/调度抖动，该部分p95为45.59ms，不能归因于PL计算核或视为纯网线延迟。板内交替微基准未单独测连续吞吐，不将延迟倒数标成实测FPS。没有同期板端频率/负载记录，也没有功耗测量，不能据此宣称能效改善。

结论：保留可复用PL模块及显式接口，当前主机USB相机＋π0.5流程继续采用软件预处理。后续若优化PL，应优先减少coherent缓冲复制并评估连续融合处理；在图像本就到达KR260的连接方式下重新测量。实时图像接入、所选微调checkpoint的动作回归、NPU对照均为`NOT_RUN`，未改变机械臂行为。

证据路径（相对工作区根目录）：

- 总索引：`reports/vision_v3_latest.json`；详细对比：`reports/vpr3_validation_latest.json`。
- 实物原始结果及RGB：`vision_preprocess/v3/evidence/ros_20261101_181010/board_results/`。
- ROS逐帧和恢复证据：`vision_preprocess/v3/evidence/ros_20261101_181010/`。
- 主机CPU逐次记录：`vision_preprocess/v3/evidence/board_20261101_171139/host_cpu_baseline.json`。

报告审核发现旧摘要误把DMA字节数、状态和计数也按纳秒转为毫秒；现已只汇总真实duration字段。`raw_before_units_fix/`保留原始摘要，480条逐帧记录未变，所有真实时间分布保持一致，`report_units_check.json`记录核验结果。此次只修正报告聚合并重建本地主机ROS包，已验证的冻结交付包和比特流未改动。
