# PiPER SDK 与工程调用资料

本工程当前 ACT / π0.5 执行链使用 **pyAgxArm 1.0.0** 的关节和夹爪 CAN 编码器。本机来源提交为 `d90e234ba8e408cfa964b38aa9120b3131fc9f85`，官方来源为 [agilexrobotics/pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)。

## 浏览与下载

- [当前使用的 SDK 源码](../../third_party/pyAgxArm)：从用户整理的快照复制，共 286 个文件，逐文件保持原始字节。
- [分类说明和原始目录](original-reading-guide.zh-CN.md)：原整理说明，保留本机路径与当时状态。其中“GitHub 工作区未复制 SDK”描述的是本次补充前的状态。
- [逐文件原始路径与 SHA-256 清单](original-paths-sha256.tsv)：完整 707 文件的来源、大小和哈希；绝对路径是原机器的溯源记录。
- [SDK 入库及压缩包校验清单](manifest.json)：286 个入库文件的校验值及原归档包的校验记录（原归档包不公开）。

原归档包校验记录（非公开下载入口）：

```text
1177241d3e7704b3e656d212cc967bf2e93adabf305c78d2aae97dc4ea5745ce
```

## 版本与用途

| 内容 | 当前作用 | 获取位置 |
| --- | --- | --- |
| pyAgxArm 1.0.0 | ACT / π0.5 共用执行后端直接复用的 CAN 编码器 | 仓库 `third_party/pyAgxArm`；完整包第 01 类另含 wheel 和早期工具 |
| piper_sdk 0.6.2 | 旧版官方接口，提供 `C_PiperInterface_V2` | 完整包第 02 类 |
| Piper SDK UI | 使用旧版接口的官方上位机源码 | 完整包第 03 类 |
| PiPER / LeRobot、π0.5、D1 迁移准备 | 项目调用源码与历史实验入口 | 完整包第 04–06 类；选定部署源码仍以仓库 `src/` 为准 |
| 第三方 LeRobot 参考 | 引用旧版接口的历史参考资料 | 完整包第 07 类 |
| ROS2 驱动节选 | 控制和消息包，调用新版 SDK | 完整包第 08 类；不含完整 URDF 网格和 MoveIt |

原机器的 `/home/aup/piper_sdk` 是指向 `/home/aup/piper SDK` 的路径别名，不是旧版 `piper_sdk` Python 包。录制端 `protocol.py` 自行解析官方 CAN 协议；执行端 `sdk_codec.py` 才直接导入新版 SDK 编码器。π0.5 执行桥通过同一后端间接使用 SDK。

## 安装与验证

在已准备好的独立 PiPER 环境中，从仓库根目录安装随仓库提供的版本：

```bash
python -m pip install ./third_party/pyAgxArm
```

SDK 声明依赖 `python-can` 和 `typing-extensions`。模型、LeRobot、相机 SDK 和设备配置仍需按 [复现说明](../REPRODUCING.md) 准备；完整源码包是查阅归档快照，不是完整运行环境。

本次将仓库 SDK 目录置于 Python 导入路径首位，确认实际从该目录导入，再运行已有 PiPER 离线套件，**74 项通过**。记录见 [offline_validation.json](offline_validation.json)。707 个快照文件哈希、压缩包全部条目与目录的一致性、286 个入库 SDK 文件哈希均已核对；未运行机械臂控制或连接板卡。

## 许可

pyAgxArm 的原 [LGPL-3.0-only LICENSE](../../third_party/pyAgxArm/LICENSE) 与版权声明保持原样；其引用的 GPL v3 文本补附于 [GPL-3.0.txt](GPL-3.0.txt)。完整压缩包保留各来源已有的许可证，不能统一视为项目原创代码或采用同一种许可。历史脚本和目录说明不构成当前设备的执行授权。
