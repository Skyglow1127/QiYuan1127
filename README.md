# QiYuan1127｜PiPER 具身智能与 FPGA 工程

本项目围绕 **PiPER 平台上的 π0.5 + LoRA 微调与本地推理方案**，开展示教数据采集、策略训练、机械臂执行接口和 FPGA 动作执行模块开发。工程从 D1-T 初始机械臂测试起步，以 ACT 完成数据训练与验证实验，逐步形成 π0.5 模型对照、本地推理优化和 FPGA 工程交付成果。

仓库提供工程源码、模型配置与下载入口、官方 SDK、实验输入和结果，以及 FPGA 工程和上板产物。发布版本：v1。

## 当前成果

| 成果 | 已完成内容 | 查看入口 |
| --- | --- | --- |
| π0.5 + LoRA 本地推理 | LoRA 020000 本地加载、低内存推理、同步测速与单线程异步预取对照 | [本地推理实验](experiments/pi05_lora_local_inference_v10/) |
| FPGA 工程与板端验证 | Vivado 工程、上板固件、KR260 加载、数据测试及 ACT 录制轨迹回放 | [工程目录](hardware/fpga_project_20261104/) |
| PiPER 采集与执行软件 | 双相机服务、官方示教录制适配、ACT 启动、共享执行后端与 CAN 编码接口 | [PiPER 源码](src/piper/) |
| 数据与模型训练 | 52 条示范、20488 帧的训练数据记录，ACT 020000 模型与 π0.5 专家 B 训练成果 | [训练入口](src/lerobot_workspace/) · [模型下载](models/DOWNLOADS.md) |
| SDK 与复现配套 | pyAgxArm 1.0.0 源码、依赖说明、路径准备、下载校验和离线检查工具 | [官方 SDK 副本](third_party/pyAgxArm/) · [复现说明](docs/REPRODUCING.md) |

**已有结果亮点：**

- AMD Ryzen AI 9 HX 370 AIPC 上（使用内置 Radeon 890M 核显），π0.5 + LoRA 在 10 步流匹配设置下生成完整 50×7 动作块，中位耗时 **1.3833 秒**。
- 固定录制输入的同步/异步对照中，20 步消费窗口的模拟有效动作频率由 **9.02–9.03 Hz** 提升至 **14.62–14.83 Hz**；50 步窗口下异步模拟吞吐约 **20 Hz**。该指标对应离线队列实验；计时范围与参数见原报告。
- FPGA 的 174451 条可布线网络全部完成，路由错误为 **0**，完整工程及上板文件已提供下载。
- **KR260 实际上板测试已完成**：固件加载和静止数据测试通过，静止测试产生 44 条输出；ACT 录制轨迹的 56 条输出比对通过，留存 57 个板端快照（含初始快照）。[上板测试证据](hardware/fpga_project_20261104/上板测试_20261104/)覆盖板卡描述符回放；实时相机连续换块与机械臂执行的状态见结果说明。
- 已有 **171 项选定离线测试通过**；引入仓库 SDK 副本后，PiPER 的 **74 项测试复核通过**。后者属于前述套件的一部分，不重复累计。

详细指标、对应版本与验证范围见 [当前成果与历史迭代](docs/RESULTS.md)。

## 项目推进路线

| 阶段 | 目的 | 留存成果 |
| --- | --- | --- |
| D1-T 初始测试 | 验证机械臂控制、感知规划与轨迹执行方法 | 接口代码、轨迹候选、回归测试及历史分析 |
| PiPER + ACT | 开展训练集有效性测定训练，建立采集、训练和执行基础 | 录制适配、训练脚本、020000 权重及历史运行记录 |
| π0.5 模型对照 | 探索动作专家训练和部署接口 | 专家 B 完整模型、验证损失及推理服务代码 |
| π0.5 + LoRA 本地化 | 推进最终技术路线的本地推理与调度优化 | 本地加载器、同步测速、异步预取程序和原始结果 |
| FPGA 工程交付 | 实现动作执行模块的硬件设计与构建输出 | RTL、工程配置、时序报告、上板产物及真实板卡测试记录 |

D1-T、ACT 和专家 B 的阶段资料保留用于说明技术演进。专家 B 是完整权重训练分支；本地 LoRA 实验使用独立的 LoRA 020000 适配器，两者模型身份和结果分别记录。

## 仓库导航

| 路径 | 内容与定位 |
| --- | --- |
| [experiments/](experiments/) | π0.5 + LoRA 本地推理程序、输入样本、性能对照与历史实验记录 |
| [hardware/fpga_project_20261104/](hardware/fpga_project_20261104/) | 当前 FPGA 工程、上板固件、板端配置与实际板卡测试结果 |
| [hardware/vision_preprocess/](hardware/vision_preprocess/) | 视觉预处理 RTL、驱动、ROS 2 接口及 HLS 实验 |
| [src/piper/](src/piper/) | PiPER 共用采集与执行实现 |
| [src/lerobot_workspace/](src/lerobot_workspace/) | LeRobot 官方录制和 ACT 训练入口 |
| [src/pi05/](src/pi05/) | π0.5 专家 B 训练、推理服务与执行桥 |
| [src/d1/](src/d1/) | D1-T 初始测试与轨迹工程 |
| [models/](models/) | 模型配置、处理器统计、下载地址与 SHA-256 |
| [third_party/pyAgxArm/](third_party/pyAgxArm/) | 当前 PiPER 执行链使用的官方 SDK 源码 |
| [configs/](configs/) · [examples/](examples/) | 资产路径配置与数据元信息示例 |
| [scripts/](scripts/) · [tests/](tests/) | 工作副本准备、模型下载、文件检查与测试说明 |
| [docs/RESULTS.md](docs/RESULTS.md) | 当前成果、阶段结果和原始证据索引 |
| [docs/history/](docs/history/) | 早期 FPGA 构建记录 |
| [hardware/robot_rt_50mhz/](hardware/robot_rt_50mhz/) | 早期 FPGA 源码和离线回归基线 |

早期 FPGA 实现阶段记录已归入历史目录；当前 FPGA 成果以 `hardware/fpga_project_20261104/` 为入口。

## 下载与环境准备

- [FPGA 工程与上板产物](hardware/fpga_project_20261104/)：当前源码、工程配置、固件和校验清单。
- [PiPER SDK 源码](third_party/pyAgxArm/)：当前执行链使用的 pyAgxArm；使用方法见 [SDK 说明](docs/sdk/README.md)。
- [模型下载清单](models/DOWNLOADS.md)：已发布 ACT 020000 和专家 B best004000，下载脚本固定版本并校验文件。本地 LoRA 实验对应的适配器获取状态见 [模型说明](models/README.md)。完整数据集另行保存。

```bash
python -m pip install -r requirements-hub.txt
python scripts/download_models.py --list
python scripts/download_models.py
```

从已安装对应依赖的 Python 环境准备独立工作副本：

```bash
python scripts/prepare_workspace.py --output .local/workspace-with-assets --assets .local/downloaded-assets.json
```

已有本机资产可参照 [配置示例](configs/assets.example.json) 提供路径。工作副本脚本使用原选定源码和历史 FPGA 回归基线；当前 FPGA 交付工程及本地 LoRA 实验按各自目录使用，详见 [复现说明](docs/REPRODUCING.md)。

软件检查入口（Python 3.12）：

```bash
python -m pip install -r requirements.txt
python scripts/check_package.py
python scripts/run_offline_checks.py --suite core
```

设备地址、相机标定和运行环境按现场配置。实物操作须完成对应预检；D1 实物抓取须先确认具体目标、路径和关键参数。

## 来源与工程记录

- [当前成果与历史迭代](docs/RESULTS.md)
- [收录规则与版本组织](docs/PACKAGING.md)
- [原始来源和 SHA-256](docs/source_manifest.json)
- [第三方来源与许可](THIRD_PARTY_NOTICES.md)

历史报告保留各次实验的原始状态、参数和数值，可沿项目路线追溯从初始测试、模型训练到本地推理及 FPGA 工程交付的推进过程。
