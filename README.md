# QiYuan1127｜PiPER 具身智能与 FPGA 工程

本项目以 **PiPER 平台上的 π0.5 + LoRA 微调与本地推理**为核心，整合双相机示教采集、策略训练、动作数据传输与 FPGA 轨迹处理。计算平台采用 **AMD Ryzen AI 9 HX 370 AIPC**，板端采用 **AMD KR260**。

工程从 D1-T 初始机械臂平台起步，经由 ACT 数据训练与模型迭代，形成 π0.5 + LoRA 本地推理、动作专家模型对照和 FPGA 板端轨迹回放成果。

## 快速执行

[进入项目 Skill 仓库：fpga-contest-skills](https://github.com/Skyglow1127/fpga-contest-skills/tree/main)

## 已完成成果

### π0.5 + LoRA 本地推理

- 完成 π0.5 基座与 LoRA 020000 适配器的本地加载，提供低内存加载、同步推理和单线程异步预取实现。
- 在 Ryzen AI 9 HX 370 AIPC 的 Radeon 890M 核显上，使用录制观测、10 步流匹配生成完整 **50 × 7 动作块**，单次推理中位耗时 **1.3833 秒**。
- 保存输入样本、动作输出数组、运行参数和性能报告，提供相应程序与配置。

[本地推理程序与结果](experiments/pi05_lora_local_inference_v10/) · [推理结果数据](experiments/pi05_lora_local_inference_v10/01_同步推理/结果/run_v10_r33/report.json)

### 示教采集与模型训练

- 整理 **52 条示范、20,488 帧、20 Hz** 的双相机示教数据记录，统一六关节角度与夹爪宽度的七维状态、动作格式。
- 完成 ACT 020000 模型训练与发布，提供第一轮训练入口及第二轮低学习率训练代码。
- 完成 π0.5 动作专家 B 的 **20,000 步训练**，发布 best004000 模型及配套处理器。
- 提供官方 LeRobot 录制入口适配、双相机采集、CAN 编码、策略启动和动作执行接口。

[ACT 训练代码](src/lerobot_workspace/) · [ACT 第二轮训练](src/piper/training/act_round2_v6/) · [π0.5 专家 B 训练](src/pi05/pi05_expert_training_batch8_v6/) · [PiPER 采集与执行源码](src/piper/)

### FPGA 工程与 KR260 上板成果

- 完成 **50 MHz FPGA 工程**的综合、布局与布线，提供 RTL、工程配置、时序报告及上板固件。
- **174,451 条可布线网络全部完成，路由错误为 0**。
- 固件已加载至 KR260，完成板卡身份和寄存器读取。
- 完成 ACT 录制画面到动作轨迹、再到真实 FPGA 描述符回放的处理流程；保存 **56 条输出和 57 个板端快照**，输出数值、序号、时间戳与描述符逐条一致。
- 提供 `.bit`、`.bit.bin`、`.xsa`、设备树、地址配置及文件校验值。

[FPGA 工程与固件](hardware/fpga_project_20261104/) · [ACT 轨迹板端回放结果](hardware/fpga_project_20261104/上板测试_20261104/03_ACT轨迹上板测试_通过/result.json)

## 项目组成

| 目录 | 内容 |
| --- | --- |
| [src/d1/](src/d1/) | D1-T 初始平台的控制、感知与轨迹处理代码 |
| [src/piper/](src/piper/) | PiPER 采集适配、相机服务、ACT 执行接口与第二轮训练代码 |
| [src/lerobot_workspace/](src/lerobot_workspace/) | ACT 训练、官方录制及运行入口 |
| [src/pi05/](src/pi05/) | π0.5 专家训练、本地推理、云端服务与动作传输 |
| [experiments/](experiments/) | π0.5 + LoRA 本地推理程序、输入样本与结果 |
| [hardware/](hardware/) | FPGA 工程、轨迹处理模块、视觉预处理与板端产物 |
| [models/](models/) | 模型配置、处理器参数及下载清单 |
| [third_party/pyAgxArm/](third_party/pyAgxArm/) | 项目使用的官方 PiPER SDK 源码 |
| [configs/](configs/) | 资产与环境配置示例 |
| [scripts/](scripts/) | 模型下载、工作副本准备及文件校验工具 |
| [复现需要的材料包/](复现需要的材料包/) | 按功能分类的源码、配置、SDK 与代码压缩包 |

## 模型与材料入口

- [ACT 020000 模型](https://huggingface.co/echoma1127/QiYuan1127-PiPER-ACT-020000)
- [π0.5 专家 B best004000 模型](https://huggingface.co/echoma1127/QiYuan1127-PiPER-PI05-best004000)
- [模型版本、下载地址与 SHA-256 清单](models/manifest.json)
- [模型下载脚本](scripts/download_models.py)
- [独立复现材料包](复现需要的材料包/) · [代码 ZIP](复现需要的材料包/代码文件.zip)

第三方来源与许可见 [THIRD_PARTY_NOTICES.txt](THIRD_PARTY_NOTICES.txt) 及各组件附带的许可证。
