# VPR3 ROS 2 Jazzy 接口

节点连接已启动的 VPR3 HTTP 服务，发布 `/vpr3/image`（`sensor_msgs/Image`、紧凑 `rgb8`）和 `/vpr3/geometry`（JSON）。默认画布 224×224，尺寸向下取整，居中黑边；保留源图像尺寸和实际缩放比例。`--source live|recorded` 必填，后端仅接受显式 `software|pl`，服务缺失、源类型错配或 PL 不可用都会失败。

在仓库根目录构建：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base build/vpr3_ros_log build --base-paths vision_preprocess/v3/ros2 --build-base build/vpr3_ros_build --install-base build/vpr3_ros_install --executor sequential
source build/vpr3_ros_install/setup.bash
```

板端管理员测试已输出 `Benchmark ready` 后，使用实际部署目录：

```bash
python3 scripts/benchmark_vpr3_ros.py --package /home/ubuntu/实际VPR3包目录 --source recorded
```

脚本读取板端 ready 文件，仅转发其 loopback 服务端口（默认 18768），以两轮交替 PS/PL 顺序测试两个样本角色。结束时写入该包的 `benchmark_stop`，让管理员进程恢复先前固件；核对恢复回执后复制结果。它不会自行加载固件、输入 sudo 密码或停止已有应用。`--source live` 必须使用实际 live 服务，不能替代回放模式。

已经存在的软件服务可用直连方式检查 ROS，避免任何 SSH 或固件操作：

```bash
python3 scripts/benchmark_vpr3_ros.py --url http://127.0.0.1:18768 --source recorded --backends software --frames 12 --rounds 1
```

所有测试使用单独的 ROS domain 和可靠 QoS。验证器按主机请求时间戳配对图像/元数据，检查字节数、SHA-256、几何、全部填充字节和帧身份，并保存首帧 `.rgb`。实时模式要求每条消息的源帧号唯一；回放模式明确记录实时帧数为 0，单独统计请求序号。

结果包含服务端各阶段、HTTP 请求总往返、消息复制、请求到 ROS 图像订阅的 p50/p95，以及每轮实际接收吞吐。`transport_and_http_overhead` 为请求总往返减去服务端工作时间，包含调度/HTTP等开销，不能当作孤立的网络传输延迟。ROS stamp 表示主机请求开始，不是曝光时间。样本上传、相机曝光、模型推理及机器人动作不在本次范围；软件服务基线为 OpenCV uint8，官方 Pi05 torch 浮点精度由其他验证流程核对。
