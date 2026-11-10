# 宇树 D1-T 从臂：URDF / Xacro / Gazebo

本项目仅包含 D1-T 的**从臂（执行臂）**。6 个旋转关节，加 1 个双指夹爪控制自由度；夹爪在物理引擎中是两个移动关节。仿真控制仍只连接 Gazebo；2026-10-22 新增独立实机只读反馈页，没有主臂、实机运动控制或双臂遥操作。

## 现在操作

2026-10-26 已按照片加入白色工作台、蓝色隔板与铝框、MINISFORUM AIPC 和 AMD KR260，并按用户要求移除连接线、棋盘格、灰色立柱、蓝色小板和黑色电源盒。布局和未实测尺寸为近似值；新增腕部相机的尺寸和安装位姿为仿真估计，实际相机型号与光学参数尚未标定。照片中没有明确目标物的尺寸，保留原有 40 mm 红色方块作为演示目标。调整流程见 [PHOTO_WORKSPACE.md](PHOTO_WORKSPACE.md)。

控制台：http://127.0.0.1:8767 。拖动 J1–J6 滑块和夹爪闭合量；可使用「准备姿态」「回到零位」「保持当前位置」「恢复控制」。左侧三维姿态依据 Gazebo 经 ROS 2 返回的 `/joint_states` 更新，数值显示实际值，而非目标值。拖动视图旋转、滚轮缩放、右键平移。

原生 Gazebo 窗口也会显示同一个从臂。它与浏览器控制台共用同一个物理仿真。浏览器不支持 WebGL 时自动使用 Canvas 兼容渲染及简化显示网格；Gazebo 仍使用完整官方 STL。

## 启动与停止

本机已配置 ROS 2 Jazzy + Gazebo Harmonic 8.15.0。新增 Debian 依赖提取到当前工作区 `work/runtime`，不需要 sudo。运行时请保留本项目与该目录的相对位置。

```bash
cd /home/aup/Documents/Codex/2026-10-15/xian/outputs/d1t_sim
bash run.sh
# 再打开 http://127.0.0.1:8767
bash stop.sh
```

`config/runtime.json` 的 `simulation_port` 默认 **8767**；旧版默认 8765 现被独立压力测试服务占用。`D1T_PORT` 可覆盖默认值，例如先停止仿真，再运行 `D1T_PORT=8877 bash run.sh`；桌面面板也须用同一变量启动。启动脚本会检查 Gazebo 反馈结构，其他程序的 JSON 响应不会被识别成仿真。实机只读页保持 8766。

`run.sh` 使用用户级 systemd 运行，不随聊天结束退出；未设置开机自启。直接终端运行可用 `bash scripts/start.sh`，按 Ctrl+C 结束；加 `--headless` 可省略原生窗口。日志保存在工作区 `work/run`。

其他 Ubuntu 24.04 / ROS Jazzy 机器可先安装 `ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-xacro ros-jazzy-robot-state-publisher python3-yaml python3-numpy`，之后使用系统模式 `D1T_USE_SYSTEM=1 bash scripts/start.sh`。需存在用户图形会话，或使用 `--headless`。

## 模型来源与参数边界

- [宇树 D1-T 产品页](https://www.unitree.com/cn/mobile/D1-T/)：6 轴 + 夹爪、臂展 550 mm（不含夹爪）/670 mm（含夹爪）、约 2.37 kg、额定负载 500 g。
- [宇树官方开发文档](https://support.unitree.com/home/en/developer/D1Arm_services)：D1-T 由两套 D1 组成；“Robotic Arm Download → Gripper version”提供适用于夹爪仿真的资源。
- [实际采用的官方模型 ZIP](https://oss-global-cdn.unitree.com/static/90b2525be5d84531ab9814f48b2f86a7.zip)：原始 SolidWorks 导出 URDF 和 9 个 STL。源 ZIP 与原始 URDF 保存在 `upstream`，校验值见 `upstream/SHA256SUMS`。

本模型采用上述 D1 带夹爪下载包，**没有混用单独的 D1-550 下载包**。官方开发页面当前参数表写的是 D1-550（3152 g），与 D1-T 产品页版本不同。按来源分别记录，不能当成同一个硬件版本的已验证规格。

官方 CAD 导出连杆质量合计只有 **0.719136 kg**，与产品页整机质量不符。保留原始质量、质心、完整惯性张量、关节轴、原点和角度限位，没有凭空分配缺失质量。因此本模型用于结构展示、关节控制和仿真接口开发，**未校准实机动力学，也未验证 500 g 负载、实机力控或通用碰撞规划**。新增的抓取演示仅验证当前 20 g 方块、当前摩擦参数和固定地面场景。

原始文件的 effort/velocity 均为 0，不能直接用于控制。`config/model.yaml` 补入可调仿真值：前两关节力矩上限 3.3 N·m，其余旋转关节 1.7 N·m；这些数值参考官方 D1-550 表，仅为本项目初值，不宣称是 D1-T 的已核实电机额定值。夹指为 20 N 仿真力上限；旋转速度上限 0.6 rad/s、夹指 0.03 m/s，控制台目标变化另限速至 0.45 rad/s、0.025 m/s。PID 增益也属于仿真调参。

夹爪按原始 URDF 的 `Joint7_1: [0, 0.03] m`、`Joint7_2: [-0.03, 0] m` 联动，控制台 0% 为张开（Joint7_1=+0.03 m、Joint7_2=-0.03 m），100% 为模型零位闭合（两指均为 0）。本次按 CAD 轴向修正了旧版反向标注。没有把它错误标成可保证的 65 mm 净开口。两指由命令节点耦合，并非机械闭环；当前抓取结果仅适用于本仿真。

## Xacro 结构

- `urdf/d1t.urdf.xacro`：从臂顶层，固定基座，参数 `root`、`mesh_dir`。
- `urdf/d1_macros.xacro`：`cad_link` 统一 visual/collision/inertial；`cad_joint` 统一关节/限位/阻尼/PID；`d1_arm` 可复用模型宏，支持前缀。
- `config/model.yaml`：集中管理官方几何、惯性与仿真控制参数。
- `generated/d1t.urdf`：Xacro 生成的完整、可解析 URDF。
- `worlds/lab.sdf`：重力、DART 物理、照片工作台环境、150 × 230 × 20 mm 安装垫块、灯光。
- `config/environment.json` / `scripts/build_environment.py`：照片布局参数与环境生成器。
- `scripts/build.py`：生成 URDF、ROS–Gazebo 桥接配置、状态发布参数和显示用拓扑。

生成：

```bash
source scripts/env.sh
python3 scripts/build.py
# 或直接：
xacro urdf/d1t.urdf.xacro root:="$PWD" mesh_dir:="$PWD/meshes" -o generated/d1t.urdf
```

完整官方网格用于 visual 和 collision；9 个 link 均启用物理自碰撞，采用 DART + FCL，物理步长 0.5 ms。直接相连的父子 link 按关节装配关系排除碰撞；其他连杆与地面均检查。机械臂基座以世界固定关节安装在 20 mm 高的垫块上，基座与垫块的安装接触被排除。模型未加入不明来源的外形尺寸或惯性估算。

控制台另以原始 CAD 三角网格进行 FCL 检查：在发出运动前，按实际逐轴限速规律采样整条路径，要求 2 mm 几何间距；每次控制更新也检查候选指令。拒绝时保持原目标，并明确显示碰撞 link，前端滑块恢复已接受目标。它不是绕障规划器，也不是连续碰撞检测证明；包括工作台面、安装垫块、照片环境的简化外包围盒与红色方块。仅双指与方块允许接触。检测到双指夹持时，根据实测的方块相对夹爪变换预测携物轨迹，检查方块与环境的穿透（容许 1 mm 接触误差）；该预测不会移动 Gazebo 方块。板卡上细小电子元件仅作视觉表示；若新增障碍物须同步扩展保护场景。实机仍需独立安全系统。

本次复现并拦截了 `Link2` 与 `Link5` 的穿模姿态，以及终点无碰撞但中途接近障碍物的轨迹。结果见 `collision-verification.json`。

## 控制链路与接口

浏览器 → 本机 HTTP（127.0.0.1:8767）→ rclpy Float64 目标 → ros_gz_bridge → Gazebo JointPositionController PID → DART → Gazebo JointStatePublisher → ROS `/joint_states` → 控制台与 robot_state_publisher（TF）。

- ROS 指令：`/d1t/arm_Joint1/cmd_pos` 至 `/d1t/arm_Joint6/cmd_pos`，类型 `std_msgs/msg/Float64`，单位 rad。
- 双夹指：`/d1t/arm_Joint7_1/cmd_pos` 和 `/d1t/arm_Joint7_2/cmd_pos`，单位 m；控制台统一驱动。
- 状态：`/joint_states`、`/clock`、`/tf`、`/tf_static`。
- ROS_DOMAIN_ID=61；Gazebo 分区 `d1t_codex_sim`；ROS 发现仅本机。控制台持续发布设定值，外部 ROS 控制前应停止控制台节点，避免两个发布者争用。
- API：GET `/api/state`，POST `/api/control`。示例 `{"values":{"arm_Joint1":0.3,"gripper":0.5}}`；夹爪为 0–1 闭合比例。服务端验证限位与有限数值，离线时拒绝动作。
- “保持”中止自动动作并保持手臂角度，同时保留夹爪挤压力目标，以免立即掉落；并不是实机急停。

## 验证与复现

`tests/verify_sim.py` 检查官方惯性数值未变化、惯性张量正定，并对正在运行的 Gazebo 执行六轴运动、夹爪开合、越界拒绝、保持/恢复和准备姿态验证。旋转误差门槛 0.003 rad、夹指 0.0005 m；它会移动仿真机械臂。实际结果见 `verification.json`。

```bash
source scripts/env.sh
python3 tests/verify_sim.py
python3 tests/verify_collision.py
python3 tests/verify_grasp.py
```

第三方模型版权归宇树；该下载包未附明确的再分发许可证，保留出处与原始文件，不为其另行授予许可。Three.js 及其加载器/控制器使用 MIT 许可证，见 `web/vendor/THREE-LICENSE.txt`。

## 实机动力学校准

详细步骤、数据条件、惯性坐标系转换与验收方法见 [DYNAMICS_CALIBRATION.md](DYNAMICS_CALIBRATION.md)；实测记录模板为 `config/calibration_template.yaml`。尚未采集实机数据，模板的 null 不代表零值，不得直接加载为模型。

碰撞保护新增依赖：NumPy 与 python-fcl。本机位于工作区 `work/mesh_deps`，`scripts/env.sh` 自动加载。其他机器请在兼容 ROS Python 的环境中安装 `requirements-control.txt`。

## 桌面启停面板（2026-10-19）

2026-10-22 新增 **连接并查看实机（只读）** 和 **断开实机监视**。实机页为 `http://127.0.0.1:8766/`，仿真页现为 `http://127.0.0.1:8767/`。实机监视不依赖 Gazebo；关闭仿真不改变实机供电、使能或监视状态。`./run_real.sh` / `./stop_real.sh` 可单独启停实机监视。

实机只读接入：`hardware/feedback_reader.cpp` 在 enp99s0、DDS domain 0 订阅 `rt/arm_Feedback`，只创建 DataReader，无命令 publisher；HTTP 所有 POST 返回403。复用本机 `/usr/local` 的 Unitree SDK2 和 `/home/aup/D1/Control/msg/ArmString_`，不修改原 SDK、Camera 或已有抓取项目。角度与状态分别判断 0.6 s 帧龄，过期或读进程退出显示断线，不以回执代替实际角度。

姿态显示沿用 `config/hardware_monitor.json` 指向的2026-10-22局部经验候选：SDK J0..J5乘符号 `[-1,+1,+1,-1,+1,+1]` 后转弧度，映射到 `arm_Joint1..6`。六轴原点、轴向、上下限与候选使用的URDF一致；本模型额外20 mm安装高度仅作建模参考。候选未完成全域物理验证，仅供预览，不用于实机下发。SDK J6保留原始值，不转为毫米或双指开度，不画未标定夹爪。真实物体位置没有接入此页。

实物抓取继续使用已有 `/home/aup/D1/首次抓取项目链_2026-10-22.txt` 记录的项目链；其轨迹、夹爪、TCP和目标仍须按实际条件审查，并在用户明确批准具体目标、路径与关键参数后执行。监视连接不会启动已有运动桥接或重放轨迹。

双击桌面的 **D1-T 仿真控制台**，或运行本目录的 `./console.sh`。

- **启动仿真**：启动 Gazebo 和 8767 控制服务，收到机械臂反馈后自动打开控制页面。
- **打开机械臂控制页面**：重新打开关节控制和方块抓放界面。
- **关闭仿真**：只停止本项目的 `d1t-sim`、`d1t-gui` 用户服务，并确认仿真反馈已关闭。
- 关闭面板窗口仅退出面板，不会停止仿真。面板可在仿真关闭时独立打开。

状态每两秒更新，启停时按钮暂时禁用以避免重复操作。重启仿真会重新加载初始场景。原有 `run.sh` / `stop.sh` 仍可使用，启停操作共用锁；修改前脚本保存在 `launcher-backup`。

## 4 cm 红色方块抓取（2026-10-17）

先前版本已备份在 `../backups/d1t_sim_before_red_cube_2026-10-17.tar.gz`；项目本地运行依赖另存 `../backups/d1t_runtime_2026-10-17.tar.gz`，均附 SHA-256。恢复说明见 `../backups/RESTORE.txt`。依赖备份不包含系统已有的 `/opt/ros/jazzy`。

方块为 **40 × 40 × 40 mm** 均匀立方体，模拟质量 **0.020 kg**，三个主惯量均为 **5.333333 × 10⁻⁶ kg·m²**；质量和摩擦系数 0.8 是演示假设。使用参数化 `urdf/red_cube.sdf.xacro`，生成 `models/red_cube/model.sdf`。方块初始中心 (0.32, 0, 0.02) m，底面 z=0；机械臂安装于用户指定的 150 × 230 × 20 mm 垫块上，本体底面 z=0.02 m。现照片场景以工作台面为 z=0；方块下方没有独立垫台。

控制台新增「一键抓取」「放回方块」「复位方块」：

1. 一键抓取：张开 → 移到上方 → 下降 → 夹紧 → 检查双指接触 → 抬起约 8 cm → 保持。
2. 放回方块：下降到地面 → 张开 → 退回上方。
3. 复位方块：张开并退离，再通过 Gazebo 显式复位服务将方块放回地面中央。手中持有抬起的方块时需先放回。
4. 自动动作期间禁用手动滑块；「保持当前位置」可中止。失败会显示具体原因，不会报告为抓取成功。

方块始终由重力、碰撞、夹指 PID 力和摩擦推动。**没有吸附插件、固定关节、跟随动画或抓取期间的方块位置写入**。浏览器中方块的位置和朝向来自 Gazebo PosePublisher；双指接触来自方块 Contact sensor。成功须满足双指有接触、方块至少升高 6 cm，并稳定保持 1.2 仿真秒。地面接触不算夹持。验收脚本额外检查两轮抓放与 2 仿真秒保持，并输出 `grasp-verification.json`。

`scripts/grasp_scene.py` 管理序列与反馈；`config/grasp_scene.json` 保存当前场景及离线求解的抓取/上方姿态。这里是固定抓取点的示例，不会自动识别或追踪任意摆放的方块；改变摆放位置后须重新求解可达姿态并验证路径。

抓取 API：POST `/api/control`，`{"action":"grasp"}` / `{"action":"place"}` / `{"action":"reset_cube"}`。`/api/state` 增加 `cube`（实际姿态、双指接触）、`grasp`（阶段、结果）和 `scene`。速度推进采用仿真时钟，低帧率时也不会超前快进指令。

实现参考：[Gazebo PosePublisher](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1PosePublisher.html)、[Gazebo 接触传感器文档](https://gazebosim.org/docs/harmonic/sensors/)及已安装 Harmonic `worlds/contact_sensor.sdf` 示例。Python 新增依赖为 SciPy、Gazebo transport13/msgs10 Python 模块及 Protobuf，本机均已配置。

### 安装垫块与地面高度

机械臂下方的垫块按用户尺寸建模：X=150 mm、Y=230 mm、Z=20 mm，中心位于 (0, 0, 0.01) m，底面在地面上。机械臂安装原点为 (0, 0, 0.02) m。红色方块仍放在地面，中心高度 0.02 m，因此方块底面与垫块底面平齐。抓取和抬升姿态已按新的机械臂安装高度重新计算。

固定基座与支撑面使用互斥碰撞位掩码（基座 2，垫块和地面 1），避免安装面重合的多余接触计算。其他连杆、夹爪和方块保留默认全位掩码，仍与地面、垫块及基座发生碰撞。参考本机 Gazebo 官方 `worlds/shapes_bitmask.sdf` 示例。

### 腕部 RGB 相机

控制台已加入随 `arm_Link6` 运动的照片近似相机模型和 Gazebo 实时 RGB 画面，可用“腕部近景”查看外观、“腕部 RGB”放大传感器画面。配置位于 `config/wrist_camera.json`，详细范围和参数见 `PHOTO_WORKSPACE.md`。当前 RGB 为 640 × 480 / 10 Hz；尺寸、安装角与光学参数均为未标定的仿真设置。

### 官方高清参考与设备比例修正

当前 AIPC/KR260 外观已按官方高清图及机械资料优化，可在控制页点击“官方高清参考图”对照查看。KR260 采用 DS988 的 132 × 140 × 36 mm 整机包络；AIPC 按照片/CPU/DMI 比对推定的铭凡 EliteMini AI370 采用 130 × 127 × 46.9 mm。摆位与局部细节仍为近似，具体来源和推定边界见 `PHOTO_WORKSPACE.md` 与 `references/official`，验证记录为 `official-reference-verification.json`。
