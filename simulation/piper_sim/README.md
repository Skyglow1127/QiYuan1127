# 松灵 PiPER · 独立 Gazebo 仿真

采用 AgileX 官方标准 PiPER：6 个旋转关节、2 个真实移动夹指。沿用 D1-T 的照片工作台、AMD KR260、MINISFORUM AIPC、相机与用户提供的 T 形支架 STL；环境清单 `config/environment.json` 与 D1-T 项目逐字节相同。

## 启动与停止

```bash
cd /home/aup/Documents/Codex/2026-10-15/xian/outputs/piper_sim
bash run.sh
# 控制页面：http://127.0.0.1:8768/
bash stop.sh
```

`run.sh` 启动本项目的原生 Gazebo 窗口与浏览器控制服务。`stop.sh` 只关闭 `piper-sim`、`piper-gui` 两个用户服务。服务不会随聊天结束退出，未设置开机自启。

端口由 `config/runtime.json` 管理，默认 **8768**，可用 `PIPER_PORT` 覆盖；更改前先停止本项目。ROS domain 为 **62**，Gazebo partition 为 **`piper_codex_sim`**，世界名为 `piper_lab`。本项目与 D1-T 各自使用独立控制接口，不连接或操作实机。

复用现有 ROS 2 Jazzy / Gazebo Harmonic 运行库 `../../work/runtime`；日志在 `../../work/piper_run`。保留项目与 `work` 目录的相对位置。

## 模型与参数

模型来自 [AgileX 官方 agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf)，固定提交 `680fa925cd00db7e2628563d2edc5540963045de`，采用 **MIT** 许可，许可证保存在 `upstream/agx_arm_urdf/LICENSE`。使用 `piper/` 标准型号和官方 `agx_gripper/`，没有混用 H、L、X 变体。

官方夹爪中的无质量虚拟 `gripper_link` / `gripper` 控制关节已移除；两个实际夹指的 mimic 标签改为显式 PID 耦合。为了让 DART 中的有限接触面更稳定，两指各加一个 42 × 26 × 1.5 mm 的仿真接触补面，向内突出 0.3 mm；它是接触近似，不是实测零件。官方视觉模型、质量与惯性未改变。旋转关节为 `joint1`–`joint6`；夹指为 `gripper_joint1`、`gripper_joint2`，张开位置分别为 **+0.05 m、−0.05 m**，闭合均为 0。按官方网格计算，模型最大净开口为 100 mm；这不是实物测量。

所有实体的官方质量、质心和完整惯性张量均保留。11 个实体惯性正定且满足主惯量三角不等式，质量合计约 4.709554 kg；部分上游惯量在网格包围盒假设下存在物理一致性疑点，详见 `upstream/official_physical_validation.json`。尚未进行实机动力学校准。PID、速度和力矩设置为仿真参数，集中在 `config/robot.json`。

## 场景与腕部相机

工作台、周边设备与照片布局来自原 D1-T 环境。未实测的尺寸及摆放位置仍是近似值；KR260 和 AIPC 表示外观及碰撞占位，不模拟 FPGA、CPU 或 NPU 计算性能。

保留现有相机外观及 T 形支架 STL。相机在 PiPER 腕部的安装位姿是仿真估计，不是手眼标定结果。图像来自 Gazebo 场景中的 RGB 传感器，可通过 `/api/wrist_camera.jpg` 查看；它不是实物相机画面。

## 抓放演示与验证

PiPER 抓取姿态已按官方网格重新求解；没有沿用 D1-T 的关节角。流程为复位方块 → 抓取 → 抬起约 8 cm → 放回工作台。放置按 Gazebo 返回的方块相对夹爪姿态重新求解关节角，把当前倾斜的方块下沿移到桌面上方 6 mm 后松爪，再检查真实落地。每轮抓取前先复位，抓取只针对当前固定点，不做任意目标识别。放置验证要求方块真实接触桌面、中心高度误差 ≤3 mm、水平落点距目标 ≤5 cm；释放后的落点可能偏移，不表示精密原位装配。

抓取期间只使用物理接触、摩擦与关节 PID，不附加固定关节或写方块位置。显式“复位方块”在夹爪张开退离后调用 Gazebo set_pose。当前验收结果由 `verification.json` 与 `grasp-verification.json` 记录。

```bash
source scripts/env.sh
python3 tests/verify_sim.py
python3 tests/verify_grasp.py
```

两脚本会移动本项目的仿真机械臂；应顺序执行，不要同时点击控制。

来源、环境清单校验值、适配内容和证据路径见 [SOURCES.json](SOURCES.json)。

## 本轮验收（2026-10-27）

已通过 11 项控制/反馈/限位/保持与相机检查，以及两轮物理抓取—保持 3 仿真秒—放回—复位。两轮抬升分别约 7.94 cm、7.93 cm。环境清单与 D1-T 字节一致，11 个实体的官方惯性值保持不变；D1-T 原仿真仍在线。完整结果见 [acceptance-summary.json](acceptance-summary.json)。
