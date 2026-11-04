# PiPER 官方 SDK 与工作区调用清单

整理日期：2026-11-04。依据本机可访问的历史会话、源代码、Git 信息和包元数据整理；未连接机械臂或 KR260，未运行运动程序。

## 一、首先区分两代 SDK

| 内容 | 本机原始位置 | 版本与用途 |
|---|---|---|
| 目前 ACT / π0.5 执行链使用的新版 SDK | [/home/aup/piper SDK/pyAgxArm](</home/aup/piper SDK/pyAgxArm>) | 官方 `pyAgxArm 1.0.0`，本机 Git 提交 `d90e234ba8e408cfa964b38aa9120b3131fc9f85`。项目的 `sdk_codec.py` 实际导入其关节和夹爪 CAN 编码器。 |
| 同一目录的无空格别名 | [/home/aup/piper_sdk](/home/aup/piper_sdk) | 软链接到 `/home/aup/piper SDK`；不是另外一份 SDK，也不是旧版 Python 包。 |
| 新版 Python wheel | [pyagxarm 1.0.0 wheel](</home/aup/piper SDK/wheels/common/pyagxarm-1.0.0-py3-none-any.whl>) | SDK 安装包；本机 SDK 独立虚拟环境为 `/home/aup/piper SDK/.venv`。 |
| 旧版 `piper_sdk` Python 包 | [/home/aup/miniconda3/lib/python3.14/site-packages/piper_sdk](/home/aup/miniconda3/lib/python3.14/site-packages/piper_sdk) | 官方旧版 `piper_sdk 0.6.2`；官方 GUI 通过 `C_PiperInterface_V2` 导入。不能与上述路径别名混为一谈。 |
| 官方上位机 | [/home/aup/Piper_sdk_ui](/home/aup/Piper_sdk_ui) | 实际代码 [piper_ui.py](/home/aup/Piper_sdk_ui/piper_ui.py) 导入旧版 `piper_sdk`；历史会话曾在本机打开 GUI。 |

官方来源：[pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)、[旧版 piper_sdk](https://github.com/agilexrobotics/piper_sdk)、[Piper_sdk_ui](https://github.com/agilexrobotics/Piper_sdk_ui)。本机版本与官方仓库最新版本可能不同，以本地提交为准。

## 二、新版 SDK 源码和资料

| 目录或文件 | 内容 |
|---|---|
| [SDK 顶层说明](</home/aup/piper SDK/先看这里.txt>) | 本机安装、ROS2 和 KR260 安装包说明；含历史状态。 |
| [来源与提交](</home/aup/piper SDK/sources.json>) | SDK、ROS2、模型仓库的地址、提交及 wheel 哈希。 |
| [PiPER API](</home/aup/piper SDK/pyAgxArm/docs/piper/piper_api.md>) / [固件参考](</home/aup/piper SDK/pyAgxArm/docs/piper/firmware_reference.md>) | 机械臂接口与固件档位。 |
| [夹爪 API](</home/aup/piper SDK/pyAgxArm/docs/effector/agx_gripper/agx_gripper_api.md>) / [CAN 手册](</home/aup/piper SDK/pyAgxArm/docs/can_user.md>) | 夹爪及 CAN 通信资料。 |
| [工厂 API](</home/aup/piper SDK/pyAgxArm/pyAgxArm/api/agx_arm_factory.py>) | 创建机型、固件和通信配置。 |
| [PiPER CAN 编码器](</home/aup/piper SDK/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/piper/default/parser.py>) | 项目复用的关节目标和模式编码。 |
| [夹爪 CAN 编码器](</home/aup/piper SDK/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/effector/agx_gripper/default/parser.py>) | 项目复用的夹爪目标编码。 |
| [PiPER 消息定义](</home/aup/piper SDK/pyAgxArm/pyAgxArm/protocols/can_protocol/msgs/piper/default>) / [夹爪消息定义](</home/aup/piper SDK/pyAgxArm/pyAgxArm/protocols/can_protocol/msgs/effector/agx_gripper/default>) | 关节、模式、夹爪及反馈消息。 |
| [项目早期 SDK 工具](</home/aup/piper SDK/tools>) | 状态、使能和运动测试相关脚本；脚本能力不等同于本次已执行。 |

## 三、涉及 SDK 的工作区与实际调用

### 1. PiPER / LeRobot 采集与 ACT 执行

原始项目：[/home/aup/Lerobot/projects/piper](/home/aup/Lerobot/projects/piper)。`/media/aup/UbuntuData/PIPER机械臂` 是指向它的软链接，不是独立项目副本。

| 文件或目录 | 与 SDK 的关系 |
|---|---|
| [sdk_codec.py](/home/aup/Lerobot/projects/piper/deployment/lerobot_robot_piper_executor/sdk_codec.py) | **直接导入新版 pyAgxArm 的 Codec 和消息类型**，生成 `0x155/0x156/0x157` 六关节及 `0x159` 夹爪命令字节。编码模块本身不打开 CAN。 |
| [backend.py](/home/aup/Lerobot/projects/piper/deployment/lerobot_robot_piper_executor/backend.py) | 调用项目 SDK 编码模块，并与远端桥接通信。 |
| [robot.py](/home/aup/Lerobot/projects/piper/deployment/lerobot_robot_piper_executor/robot.py) / [guard.py](/home/aup/Lerobot/projects/piper/deployment/lerobot_robot_piper_executor/guard.py) | LeRobot 执行器和目标、反馈检查，属于项目代码。 |
| [remote_bridge.py](/home/aup/Lerobot/projects/piper/deployment/lerobot_robot_piper_executor/remote_bridge.py) | KR260 侧自写的 Python SocketCAN 桥；负责总线收发，本文件不直接导入官方 SDK。 |
| [rollout_act_piper.py](/home/aup/Lerobot/projects/piper/official_adapter/rollout_act_piper.py) | 将本机 `pyAgxArm` 源码目录加入路径，并在 ACT 执行中准备 SDK 编码器。 |
| [preflight_j_hold.py](/home/aup/Lerobot/projects/piper/deployment/preflight_j_hold.py) | 直接引用官方编码器，用于相关检查和模式帧构造。 |
| [protocol.py](/home/aup/Lerobot/projects/piper/official_adapter/lerobot_robot_piper_shared/protocol.py) | 采集端依据官方协议自行解析被动 CAN 数据；此文件并不直接导入 SDK。 |
| [record_piper.py](/home/aup/Lerobot/projects/piper/official_adapter/record_piper.py) | 项目录制包装入口；官方 `lerobot-record` 才是录制引擎。 |
| [SDK 技术参考](/home/aup/Lerobot/projects/piper/docs/PIPER_SDK技术参考.md) / [API 索引](/home/aup/Lerobot/projects/piper/docs/PIPER_SDK_API索引.md) | 该工作区此前整理的接口、源码和固件说明。 |
| [KR260 状态示例](/home/aup/Lerobot/projects/piper/docs/kr260_examples/read_status.py) / [回零工具源码](/home/aup/Lerobot/projects/piper/docs/kr260_examples/home_follower.py) | 项目编写的新版 SDK 调用示例。历史会话曾使用板端 `/home/ubuntu/piper_sdk/` 和 `/home/ubuntu/piper_tools/`；本次未连接板卡核验这些远端路径当前是否存在。 |

### 2. π0.5 项目

| 原始目录或文件 | 与 SDK 的关系 |
|---|---|
| [早期云端 LoRA 执行工作区](/home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_grasp) | `client.py`、`trial.py` 等添加新版 SDK 源码路径，复用 PiPER 项目的执行后端和 SDK 编码器。 |
| [动作专家本地执行目录](/home/aup/Documents/Codex/v5/wo-y/outputs/pi05_expert_inference_v6/local_grasp) | 动作专家模型的本地执行相关代码，复用同一 PiPER 执行后端。 |
| [云端 π0.5—ACT 执行桥](/home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/cloud_grasp_act_bridge) | [trial.py](/home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/cloud_grasp_act_bridge/trial.py) 加入新版 SDK 路径；[client.py](/home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/cloud_grasp_act_bridge/client.py) 通过项目后端间接使用编码器。 |
| [其他历史云端执行版本](/home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7) | 包含 `cloud_grasp_lerobot`、`cloud_grasp_controlled`、`cloud_grasp_controlled_v2` 等版本；是同一 SDK 后端的不同实验入口，不是不同 SDK。 |

模型推理本身与 PiPER SDK 编码分属不同环节；加载 π0.5 权重不等于直接调用机械臂 SDK。

### 3. FPGA 动作块接口

[robot_rt 通信说明](/home/aup/Lerobot/projects/piper/robot_rt/spec/TRANSPORT_NOTES.md) 对照官方 `pyAgxArm` 编码、报文格式和项目 `sdk_codec.py`。当前离线 π0.5→PL bank 编译器不是直接使用 SDK 向机械臂发送命令；不能把协议对照写成 PL 已经运行 SDK 或物理 CAN 链路已打通。

### 4. D1 项目的 PiPER 迁移准备

[PiPER 迁移准备](</media/aup/UbuntuData/ACT训练/10_Piper迁移准备>) 中的 [reader.py](</media/aup/UbuntuData/ACT训练/10_Piper迁移准备/piper_prepare/reader.py>) 有新版 `pyAgxArm` 工厂和设备读取代码。[sources.json](</media/aup/UbuntuData/ACT训练/10_Piper迁移准备/sources.json>) 记录其当时只读检查官方源码、未安装或连接设备；不应把准备代码当作已完成实机运行。

### 5. 第三方 LeRobot 参考代码

- [WEGO 参考的 piper_motors_bus.py](/home/aup/Lerobot/docs/piper_references/wego/lerobot_robot_piper/motors/piper_motors_bus.py)：导入旧版 `C_PiperInterface_V2`。
- [Agrobotics 参考的 piper_sdk_interface.py](/home/aup/Lerobot/docs/piper_references/agrobotics/lerobot_robot_piper/piper_sdk_interface.py)：同样导入旧版 `C_PiperInterface_V2`。

这些是项目保留的第三方参考，不是证据表明当前 PiPER 采集或 ACT 部署正在使用这些实现；来源和许可仍应按各自仓库核查。

### 6. GitHub 整理工作区

[/media/aup/UbuntuData/GITHUB/project](/media/aup/UbuntuData/GITHUB/project) 保存了项目代码副本和 [第三方依赖说明](</media/aup/UbuntuData/GITHUB/project/THIRD_PARTY_NOTICES.md>)。其中 PiPER 的 `sdk_codec.py`、录制协议 `protocol.py` 和 π0.5 桥 `client.py` 三个文件，本次核对与上述原项目对应文件的 SHA-256 相同；这不表示整个目录所有文件都相同。该工作区未复制完整 SDK，需另行准备 `pyAgxArm`。

## 四、ROS2、URDF 和 KR260 包

- [官方 ROS2 驱动完整源码](</home/aup/piper SDK/ros2_ws/src/agx_arm_ros>)：其中 [agx_arm_ctrl_single_node.py](</home/aup/piper SDK/ros2_ws/src/agx_arm_ros/src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py>) 直接导入 `pyAgxArm`；另有消息、描述和 MoveIt 包。
- [官方 URDF 模型](</home/aup/piper SDK/ros2_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf>)：机械臂模型和网格资源。
- [KR260 安装包](</home/aup/piper SDK/KR260安装包.tar.gz>)：已有的较大安装包，包含 SDK/ROS2 源码、模型和 ARM64 离线依赖；历史记录不代表当前板端状态。
- 本次源码快照的 `08_ROS2驱动节选` 只含控制与消息两个包及顶层说明/许可，不含完整 URDF 网格和 MoveIt；完整内容见以上原始目录或 KR260 包。

## 五、快照范围与使用

本次 ZIP 包含：新版 SDK 源码与 wheel、旧版已安装 Python 包和版本元数据、官方 UI、PiPER 项目主要采集/执行源码、π0.5 各主要实验入口、D1 的 PiPER 迁移准备、第三方参考，以及 ROS2 控制/消息源码节选。

未打包虚拟环境、模型权重、训练数据、历史运行日志、Git 数据库和较大的完整 ROS2 网格；历史备份及 GitHub 整理副本在上方给出原路径。快照不承诺可直接迁移运行：源代码中的绝对路径、板卡、CAN、模型与环境配置仍以各原工程为准。源文件未修改。

每个已复制文件的原路径、快照相对路径、字节数和 SHA-256 见 `原始路径与哈希.tsv`。文件中包含控制和使能程序，以上清单仅用于查找与阅读；本次没有执行这些程序。
