# 机械臂仿真

- [D1-T](d1t_sim/)：官方模型适配、Xacro、双指、固定方块抓放、碰撞检查、网页控制。
- [PiPER](piper_sim/)：官方标准PiPER模型、接触抓放、桌面和相机配置。

各目录的SOURCES.json记录官方来源、版本和本地改动；官方网格/惯性参数不属于原创算法。原验收JSON保存历史仿真结果。

新机器采用ROS 2 Jazzy/Gazebo Harmonic系统环境：运行入口前设置 `D1T_USE_SYSTEM=1` 或 `PIPER_USE_SYSTEM=1`，安装对应目录requirements-control.txt，然后按目录README重新build生成带有本机路径的URDF/SDF。原私有运行镜像未收录，generated中的绝对路径属于历史输出。控制台和run_real.sh是不同用途，不能把仿真批准当实机批准。

本轮只归档与检查源文件，没有开启仿真或机械臂运动。
