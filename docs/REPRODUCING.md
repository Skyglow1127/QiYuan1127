# 环境与复现入口

先完成 README 中的离线检查。所有模型、数据和 SDK 均应使用与选定源码匹配的版本。这里给出准备方法，不自动安装驱动、部署远端或触发实物动作。

## 环境

| 模块 | 必要环境 |
| --- | --- |
| 最小离线套件 | Python 3.12、根目录 requirements.txt |
| PiPER ACT | LeRobot 0.6.1、匹配的 PyTorch / torchvision、pyAgxArm、相机 SDK |
| π0.5 | LeRobot 0.6.1、配置中的 PyTorch/Transformers 环境、完整模型、原 tokenizer 与处理器；按 CPU/GPU 分开配置 |
| D1 完整规划与执行 | ROS 2、NumPy、SciPy、OpenCV、Unitree SDK2、Boost ≥1.75、C++17；根据设备配置网口及标定 |
| FPGA | AMD Vivado / Vitis 2025.2、KR260 板卡支持文件；VPR3 另需 Vitis Vision 与对应生成 HLS IP |

`src/piper/requirements-lerobot.lock.txt` 是原环境包版本快照，可能含本机路径和平台相关软件，供核对，不建议不加区分地在其他机器全量安装。GPU 环境记录见 `docs/results/pi05/environment.json`。

PiPER 插件安装示例（在独立、已经匹配好 PyTorch 的环境中）：

```bash
python -m pip install 'lerobot==0.6.1'
python -m pip install ./third_party/pyAgxArm
python -m pip install -e src/piper/official_adapter -e src/piper/deployment
```

随仓库提供的 SDK 为 pyAgxArm 1.0.0，原提交及完整归档见 [SDK 说明](sdk/README.md)。

最初整理检查复用了原有 LeRobot / SDK 环境；新增 pytest 与 SciPy 只安装在忽略提交的 `.local/qa-deps` 中，没有修改原工程虚拟环境。

## 工作副本

当前 FPGA 交付目录为 `hardware/fpga_project_20261104/`，其文件按原目录关系保留。下述工作副本脚本仍使用 `hardware/robot_rt_50mhz/` 作为历史回归基线，不会自动替换成新 FPGA 工程。

运行 `scripts/prepare_workspace.py` 会复制选定源码到新的 `.local/workspace` 并替换原本机路径。目标已存在会拒绝覆盖，需指定新输出目录。准备脚本使用调用它的 Python，因此完整 PiPER 使用时应从已安装上述依赖的环境运行。

- `lerobot/projects/piper/`：PiPER 项目。
- `lerobot/`：官方录制和 ACT 训练包装脚本、独立的 activate.sh。
- `pi05/`：各训练、推理目录；保持原来的兄弟目录关系。
- `d1/`：D1 工程。
- `hardware/`：硬件源码。

脚本不修改数值控制参数或模型身份校验。它不代替编译、相机标定和设备配置。历史 SHA 指向原文件；路径变化后需要新的部署审查，不能把旧 approval 或 trial 复制过来继续执行。

## 入口定位

| 任务 | 仓库入口 | 说明 |
| --- | --- | --- |
| PiPER 示范录制 | src/lerobot_workspace/record_piper_official.sh | 调用官方 lerobot-record；共用 CAN 观察适配不作为执行器 |
| ACT 本地训练 | src/lerobot_workspace/train_act_rad_place_v5.sh | 默认 dry-run；有完整合并数据时复用，不覆盖它 |
| ACT 已有云训练方案 | src/lerobot_workspace/cloud_act_v5/ | 原 020000 模型的训练资料；需改为自己的云端环境 |
| ACT 修复配置入口 | src/piper/deployment/launch_act_contact_repair.py | 当前 30/.7 配置；调用前读已知限制 |
| π0.5 B 训练 | src/pi05/pi05_expert_training_batch8_v6/train_expert_checked.py | batch8 独立完整专家训练；云端路径须自行配置 |
| π0.5 本地推理 | src/pi05/pi05_expert_inference_v6/runtime.py | full-native PI05，需完整权重、原 tokenizer 与处理器 |
| π0.5 无损服务 | src/pi05/pi05_cloud_inference_v7/lerobot_lossless_service/ | 18814 方案；GPU、地址和远端目录均为部署配置 |
| π0.5 设备桥接 | src/pi05/pi05_cloud_inference_v7/cloud_grasp_act_bridge/ | 官方策略队列与 PiPER 执行接口，不加载 ACT 权重 |
| D1 轨迹 | src/d1/app/closed_loop_runner.py | demo / prepare / execute 分开；本次只检查离线部分 |
| FPGA 当前工程 | hardware/fpga_project_20261104/工程源码/ | 已提供工程配置、源码、时序报告；上板产物从同目录的上板文件子目录获取 |
| 50 MHz 历史基线 | hardware/robot_rt_50mhz/tcl/build_50mhz.py | 早期源码与离线回归基线；构建阶段记录在 docs/history/fpga_50mhz/ |

如果运行入口依赖远端脚本、设备驱动、相机 SDK `.so`、未收录的历史日志或板卡文件，请按其配置补充；不要跳过模型身份、反馈新鲜度或关节限位检查。

## 视觉预处理的外部依赖

VPR3 保留 RTL、Linux DMA 驱动、ROS 2 节点与软件参考；V2 HLS 源码作为生成依赖保留。Vitis Vision 整库、生成的 HLS IP 和 Vivado 项目缓存不随提交复制。重新生成 IP 时需使用与原设计一致的接口与参数。

VPR3 的板卡生成 Tcl 还使用 AMD Ross 插件中的 `lib/ipcfg.tcl`，由 `VPR_SKILL` 指定；该工具不属于本项目源码。原机器的 VPR 构建约定使用 Ross MCP、串行构建及内存限制；本次未执行任何构建或改写流程。

## π0.5 + LoRA 本地离线推理实验

[独立实验目录](../experiments/pi05_lora_local_inference_v10/README.md) 保留同步测速、异步预取对照和 Radeon 890M/ROCm 结果。该目录整体自带相对路径关系，不由 `prepare_workspace.py` 自动迁移。先运行其 `--self-test` 可验证假模型调度；正式测速还需对应 LoRA 020000、基座、tokenizer 和本地 ROCm 环境，并调整归档脚本中的原机器路径。已发布专家 B 完整权重不是这份 LoRA 适配器。
