# 照片工作台环境（2026-10-26）

以用户提供的四张照片搭建：白色工作台、后方与右侧蓝色隔板/铝框、D1-T 从臂、银色 MINISFORUM AIPC 和绿色 AMD KR260。根据用户后续要求，已移除所有连接线、棋盘格、KR260 旁灰色立柱、前方蓝色小板和黑色电源盒；保留桌面穿线孔。照片保存在 `/home/aup/文档/ChatGPT/演示视频/环境照片`。未转录设备上的私人标签。

采用已有宇树官方 D1 带夹爪模型表示 D1-T 执行臂。AIPC 按用户指定配置标识为 **AMD Ryzen AI 9 HX 370**，板卡为 **AMD Kria KR260**；它们在此处用于外观与占位，不运行 CPU/NPU/FPGA 的硬件级计算仿真。

照片只有一个整体视角，其余为设备特写，不能唯一恢复真实尺度与深度。桌面、隔板和设备摆位仍为可调整的照片估计；主机与板卡的外包络已按下述官方资料修正，局部细节仍近似制作，均不代表实机标定结果。新增腕部相机的外形尺寸和安装位姿也采用仿真估计，实际相机型号、内参与畸变等光学参数尚未标定。保留已验证的机械臂、150 × 230 × 20 mm 安装垫块和 40 mm 红色演示方块。照片未给出实际待抓物体的尺寸与位姿。

坐标约定：安装垫块中心的桌面投影为原点，**桌面 z=0，+Z 向上，+X 指向后方蓝色隔板，−Y 指向整体照片右侧**。机械臂安装高度仍为 z=0.02 m，方块初始中心仍为 (0.32, 0, 0.02) m。工作台及设备使用静态碰撞几何；碰撞保护使用简化外包围盒。KR260 上细小电子元件仅作视觉表示。

## 启停与调整

```bash
cd /home/aup/Documents/Codex/2026-10-15/xian/outputs/d1t_sim
bash run.sh                  # 浏览器：http://127.0.0.1:8767/
bash stop.sh                 # 只停本项目两个仿真服务
```

修改 `config/environment.json` 后生成环境并重启：

```bash
bash stop.sh
source scripts/env.sh
python3 scripts/build_environment.py
bash run.sh
```

每次生成都会重建环境输出，持久修改应放在 `config/environment.json`。若调整机械臂安装、桌面高度或演示方块位置，还须同步调整抓取场景并重新检查路径；仅移动周边设备后先检查预览和碰撞提示。

默认端口在 `config/runtime.json` 的 `simulation_port`；`D1T_PORT` 可覆盖。改端口前先停止仿真，启动脚本和桌面面板须使用相同变量。8765 保留给现有压力测试服务，8766 保留给独立实机只读监视。启动使用用户级 systemd，无开机自启。

照片场景修改前完整备份：`../backups/d1t_sim_before_photo_workspace_2026-10-26.tar.gz`；本次简化环境与新增腕部相机前完整备份：`../backups/d1t_sim_before_wrist_camera_2026-10-26.tar.gz`。本次只修改仿真；实机抓取仍须用户明确批准具体目标、路径与关键参数后执行。

## 腕部相机 RGB 仿真（2026-10-26）

腕部相机按新增三张照片建立白色横向机壳、黑色多镜头面和 T 形支架，固定在 `arm_Link6`，随 Gazebo 的实际腕部姿态运动。几何、安装位姿和光学设置统一放在 `config/wrist_camera.json`；生成入口为 `scripts/build_wrist_camera.py`，由 `scripts/build.py` 自动调用。相机的机壳碰撞盒和用户 STL 支架网格同时进入 Gazebo 与 FCL 路径检查。固定几何并入原腕部刚体，不凭照片添加未经测量的质量和惯性。

当前为真实 Gazebo RGB 传感器输出：640 × 480、10 Hz、水平视场 90°，近远裁剪 0.01–4 m。相机为展示抓取区向夹爪方向倾斜约 20°；这些都是仿真初值，不是实机内参或手眼标定结果，也不代表实际相机型号。未实现深度图。

控制页新增“腕部近景”和“腕部 RGB”。`GET /api/wrist_camera.jpg` 返回最新 JPEG，`GET /api/state` 的 `wrist_camera` 字段提供帧龄（秒）、帧计数和分辨率。帧超过 2 秒后不再作为实时图像显示，接口返回 503；图像编码在独立线程完成。后台使用 Ogre2 无窗口渲染，不需要连接实物相机。

本轮验证见 `wrist-camera-verification.json`；`photo-workspace-verification.json` 是新增相机前的历史记录，不能代表本轮删减后的场景计数。

## 首轮照片细化与用户 STL 支架（2026-10-26）

AIPC 现有圆角分层外壳、接缝、44 条侧通风槽、金属接口边框与 USB 内部舌片、音频/电源细节和橡胶脚；KR260 现有开口端口与触点、SFP 笼孔、PCB 走线/焊盘、小元件与 IC 引脚、散热鳍片、扫掠风扇叶片及开放环栅。布局和主要设备碰撞包络未变，细节依照片近似制作，不代表精确电路板设计文件。可用“AIPC 近景”和“KR260 近景”查看。

用户 `摄像头.STL` 已冻结为 `upstream/wrist_camera_mount_original.STL`，SHA-256 记录在 `config/wrist_camera.json`。该文件包含 1276 个三角面，内容为 T 形安装支架、斜面及通孔；机壳与镜头继续采用照片外观近似。源 STL 无单位声明，按部件尺度推定毫米后转换为米，完整保留原网格，未简化或变形。`scripts/import_wrist_camera_mesh.py` 依据配置的刚体矩阵生成 `meshes/wrist_camera_mount.STL`，横梁孔中点对齐机壳背面，供 Gazebo、网页与 FCL 共用。实时 RGB 的光学设置保持原值。

本次运行验证为 `detail-refinement-verification.json`：相机新网格的抓放路径检查通过，Gazebo 一轮物理抓取/放回通过，RGB 持续在线且方块可见。较早的验证文件为相应旧版本记录。

## 官方图片与机械尺寸修正（当前版本）

参考入口为控制台的“官方高清参考图”：`/web/official-references.html`。已保存 10 张官方图片及元数据，位于 `references/official`；AIPC 来源含 1946 × 1946 产品图和官方支持规格图，KR260 来源含 1200 × 1200 产品图与高清机械图。未使用电商转引图作为建模依据。

- AMD KR260：按 [DS988 机械章节](https://docs.amd.com/r/en-US/ds988-kr260-starter-kit/Mechanical) v1.1（2024-07-02），整机包络为 132 × 140 × 36 mm，裸 PCB 为 119 × 140 mm，SOM/散热组件为 60 × 77 × 27 mm。132 mm 包含接口外伸，与旧摘要 119 mm 口径区分。端口按 [AMD 官方产品图](https://www.amd.com/en/products/system-on-modules/kria/k26/kr260-robotics-starter-kit.html) 修正为 4 个 RJ45、4 个 USB-A、DisplayPort、SFP+、4 组 Pmod 和 40 针接口。
- AMD 平台 AIPC：由用户照片、HX 370 处理器及本机 DMI 的 EliteMini 系列信息，比对推定为 MINISFORUM EliteMini AI370。DMI 并未直接声明具体零售型号。外壳按 [官方支持页](https://www.minisforum.com/pages/product-info) 链接的 AI370 规格图采用 130 × 127 × 46.9 mm，接口、斜向侧通风、后排气和品牌字样按 [MINISFORUM 官方产品图库](https://refurbished.minisforum.com/products/minisforum-elitemini-ai370-refurbished) 重建。该尺寸以 AI370 机型推定正确为前提，不是用户设备的物理测量值。

场景设备中心保持不变，新外包围盒均包含于上一版已验证的设备外包围盒。局部零件、走线和丝印是外观近似，不构成精确 PCB CAD 或电气仿真。品牌文字几何在 Gazebo 和网页共用 `models/photo_workspace/markings.json` 对应的生成数据。

本轮验证为 `official-reference-verification.json`：SDF、静态障碍同步、抓放路径、关节反馈、腕部 RGB 和官方参考图片访问均已核对。机器人及相机 URDF 哈希与上一版一致，已有物理抓放通过证据沿用 `detail-refinement-verification.json`；没有重复执行无必要的完整抓取验收。修正前完整备份为 `../backups/d1t_sim_before_official_reference_2026-10-26.tar.gz`。
