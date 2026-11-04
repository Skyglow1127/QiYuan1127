> 来源文档快照：下文的“当前”、路径和命令对应原记录时间；本次选定版本与验证边界见 [提交状态](../../../docs/RESULTS.md)。

# π0.5 动作专家 B：本机与云端推理

日期：v6，Asia/Singapore。训练实验 `pi05_expert_only_v6_r09` 已完成，最终模型、最佳模型及完整续训状态均已本地备份，逐文件 SHA256 校验通过。原 LoRA 对照模型 A 保留不变。

默认推理候选为验证损失最低的 **step 4000**：loss `0.0672206492`，完整权重 SHA256 `583ff741239b111053dc969fe5ba69eb70475e888c7ece92b04a78d3319386cf`。最终 step 20000（loss `0.0724199404`）另行保留，没有替换或删除。最低离线验证损失不代表已经证明实物抓取更好。

## 已配置内容

- 本地：AMD Radeon 890M，已有 ROCm 7.14.1 / PyTorch 2.11 环境，常驻服务 `127.0.0.1:18807`。
- 云端：RTX 5090 D，已有 CUDA / PyTorch 2.8 环境，新 B 独立服务经 SSH 转发到 `127.0.0.1:18806`。
- 两端都严格加载 B 的完整 813 个原生参数、6 个非持久 buffer，以及 B 保存的处理器；不挂载 A 的 LoRA，不修改官方 LeRobot 0.6.1 源码。
- 推理 batch **1**，每次真正预测 `[50,7]` 动作块，固定 **10 次去噪**；训练 batch 8 不限制推理必须 batch 8。
- 本机采用逐 tensor 普通文件读取，避免 mmap/HIP 统一内存导致加载时驻留两份权重；不量化、不修改权重或 dtype。
- 最终配置使用 `denoise_step` 编译，Runtime 构造、预热与所有预测在同一常驻线程；收到并发请求即拒绝，不积压过期观测。

## 测速结果

同一段已录制的双相机观测、相同 state/task、相同保存的 float32 初始噪声；GPU 同步计时。主要比较均为 3 次预热后 10 次测量，中位数如下：

| 路径 | 模型计算 | 输入已准备到结果校验 |
|---|---:|---:|
| 本机普通模式 | 1469 ms | 1473 ms |
| 本机编译模式，直接 Runtime | 1405 ms | 1408 ms |
| 本机最终常驻服务，localhost IPC | 1359 ms | 1368 ms |
| 云端普通模式 | 197 ms | 253 ms，含网络 |
| 云端编译模式 | 73 ms | 123 ms，含网络 |

最终常驻本机服务的实际 localhost IPC 复测另存 `results/local_service_final/report.json`。这里的耗时不包括相机采集、机械臂反馈、CAN 发送和执行器运动；不能把动作块长度当成实时控制频率。样本数有限，p95 仅描述本次短测。

原始报告、逐次计时及动作输出见 `results/`，汇总为 `benchmark_summary.json`。模型加载与首次调用单列，不计入暖态中位数。AOTriton 候选设置只做过独立离线试验，未改善本机速度，未用于默认服务。

## 本地终端命令

启动或复用已运行的本机模型（首次启动需加载及预热）：

```bash
cd /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_expert_inference_v6
bash start_local.sh
```

使用录制输入预测一次，结果保存到新建的 `predictions/<时间>/actions.json`，没有机械臂动作：

```bash
bash /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_expert_inference_v6/predict_local.sh
```

PiPER 现场只读预览：

```bash
bash /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_expert_inference_v6/local_grasp/preview.sh
```

模型在本机 AMD GPU 上计算；PiPER 反馈与相机继续使用已有 KR260 硬件适配。上面的机器人网络连接不等于云端模型推理。

停止仅本任务的本机模型服务：

```bash
bash /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_expert_inference_v6/stop_local.sh
```

状态检查：`python3 /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_expert_inference_v6/local_service.py status`。

## 实机抓取当前未就绪

本机加速后仍约 1.4 秒，超过现有的 **400 ms 原始观测至动作时效门槛**。本次没有放宽门槛、刷新或伪造源时间，没有发送机械臂命令。当前入口能预测、预览和记录拒绝原因，不能据此直接执行抓取。慢速停稳单步本地方案需要另行设计及审阅；云端约 123 ms 的结果也只是离线链路证据，实机仍须现场预览。

本次现场预览还因 KR260 SSH RPC 超时未取得反馈：SSH 别名当前指向 `192.168.26.2`，旧 IO 资料记录 `10.42.0.55`；电脑两张有线网卡均 `NO-CARRIER`，没有本机 CAN 接口。没有自动修改 IP、路由、接口身份或机器人配置去绕过这些问题。接好网络后应重新读取真实设备身份，再生成新预览。

现场诊断保存在 `live_preview_initial.log` 与 `local_grasp/previews/initial_v6/`。`local_grasp/README.md` 说明当前 5 秒、J5%、夹爪 1 N、每轮新动作块取 action0 的候选试跑范围；它不代表本次抓取已获批准。具体图像、目标、路径与参数展示并取得新批准之后，才可执行符合新方案的实物测试。此前 A 或空载微动的批准不能复用。

## 云端配置位置

新 B 服务目录：`/root/autodl-tmp/piper_pi05/expert_inference_v6`；当前服务使用 `cloud_compiled_config.json`，实际模型位于训练实验的 `best_model/`，不占用或覆盖原 A 部署目录。

本机 `cloud_compiled_config.json` 对应 `http://127.0.0.1:18806`，仅在已有 SSH master 和转发存活时可用。测速命令：

```bash
/home/aup/Lerobot/envs/piper-cpu/bin/python /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_expert_inference_v6/client.py --url http://127.0.0.1:18806 --config /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_expert_inference_v6/cloud_compiled_config.json --input /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_expert_inference_v6/canonical_input.npz --output /tmp/pi05_B_cloud_speedcheck --warmup 3 --repeats 10
```

输出目录必须是新目录。原 A 模型及部署配置保留，原 A 的推理进程在本次资源隔离测速中暂停。
