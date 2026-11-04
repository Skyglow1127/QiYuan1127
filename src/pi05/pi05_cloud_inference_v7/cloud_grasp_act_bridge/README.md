> 来源文档快照：下文的“当前”、路径和命令对应原记录时间；本次选定版本与验证边界见 [提交状态](../../../../docs/RESULTS.md)。

# π0.5 闭爪协调修复（v8）

当前仍为动作专家B best004000，既有18814无损云端服务，模型与官方LeRobot0.6.1不变。

最新失败的直接证据：视觉门放行时实际开口66.5mm，实际尚未收至示范夹持平台就被原手臂时序带走，下一块又张开；允许闭合阶段实际最低47.9mm。52段示范稳定夹持反馈42.5～45.4mm，已有成功ACT反馈44.5mm。

本次新增`closure_sync.py`与运行接线：允许闭合后保持当时实测六轴，锁存模型最小闭合意图并单独保存原来源；每步额外只读新反馈但不刷新相机或原模型时间。实际42.5～45.5mm连续0.30秒、极差≤0.5mm，且上次ACK目标比实际小至少2mm，才判夹持候选。4秒未完成、空闭<35mm、锚点偏离>3°或反馈/故障异常即停。候选目标ACK后丢弃旧FIFO，必须凭之后的新观测重新预测。携带上限固定为确认时ACK+1mm与实际−2mm的较小值，防小步累计松开；仅实际进入既有完整六轴放置窗才允许释放。反馈力只记录，平台不证明真实抓起。

2°/3mm相对反馈限幅、1N、5%J模式、400ms原槽位、TTL与双端Guard保留。阶段只在ACK后提交，未知结果不重发。默认启动仍只展示；旧批准因代码/参数变化失效。目标仍为红块→左黑框→示范末尾撤回，不自动home。

82项离线检查通过（24纯函数、14接线、23官方队列、21原视觉）；包含真实SDK与双Guard合成发送。完整实机抓放仍待批准验证。最新035901只读预检失败：ctrl_mode/mode_feedback均0、夹爪未使能，J2−1.91°/J3+1.923°超出现有范围；未发送运动。

恢复原可执行状态和范围内起点后，先运行`preview.sh --duration 90`，再运行`start_trial.sh`展示新计划。只能使用新计划所列绑定ID批准命令；不得重用旧命令。

---

以下为历史排查记录，其中旧端口、方案ID和“当前”字样仅指对应历史轮次。

# π0.5 官方FIFO与ACT执行适配：当前18812编译服务

当前使用π0.5动作专家B batch8 best4000完整模型，不加载ACT权重。官方 `PI05Policy.select_action/reset` 保持原方法；运行时消费20步，每次仍预测50步。原checkpoint消费值10、权重、LeRobot安装、原ACT工程均不改。

云端连接改为独立18812服务 `lerobot_fast_service`：仅启用 `denoise_step` 编译（reduce-overhead），精度仍highest。旧18810 eager目录完整保留，新服务未复制旧runtime/PID/results。新配置已完成云端加载、预热、真实测速及现场只读预览；旧服务/代码指纹的批准不能迁移。

当前新待审方案ID：`24280e444a87584885804e6e3f73a989cbed4f3383de7920eaa675293387d3e4`，对应`previews/v8_r28438/report.json`。此前编译实机接受69条后因相机来源失鲜退出；相机修复后另有3条被接收端接受，第3条随后本地commit过期。本次异步日志修复仅离线和只读验证，未启动新实机运动；不得重发已接受的第3条。

```bash
bash /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/lerobot_fast_service/start.sh
bash /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/cloud_grasp_act_bridge/preview.sh --duration 0
bash /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/cloud_grasp_act_bridge/start_trial.sh
```

最后一条默认只展示计划。新现场预览生成计划后，现场操作者审阅当前红块→桌面左侧黑框目标、路径及参数，确认现场条件，再使用展示的绑定ID命令批准并运动。

## 运行参数及来源

- 官方每次预测50步，FIFO执行前20步后重新预测；保存checkpoint10不改。20Hz下最后索引19的原来源派生槽位为采集时刻+0.95秒。
- duration0按官方语义持续至Ctrl+C或保护停止。可重新预览选择1至300秒有限时长，不能执行时临时覆盖。结束不自动回初始位。
- J模式速度5%，夹爪1N；当前供电和使能方式不变。
- 仅复用已有ACT的四项硬件适配：关节边界向内2°、夹爪向内1mm，相对本周期输入反馈的目标限幅2°/3mm，目标速率比例0.9。没有alpha平滑、抓放续步或原始8°预测拒绝。
- 2°/3mm限制针对本周期输入反馈，不是累计位移限制。接收端仍对真正当前反馈实施8°/15mm跟踪保护。
- VLA硬范围、6°/8mm实际单步、45°/s及50mm/s速率、反馈新鲜度、故障、设备身份、租约、120ms发送TTL及400ms来源/槽位检查全部保持。

## 编译预热与超时

任何实时来源构造/采集、机器人构造、arm之前，用服务目录内真实录制 `canonical_input.npz` 连续推理3次。零输入服务预热不足以证明真实录制路径已编译，因此仅这3次启动录制请求临时使用30秒超时。保留同一HTTP连接；finally恢复原client、connection及socket超时（默认预览3秒、执行0.4秒）。失败原异常回抛，无自动重试，live请求不放宽。

三个输出全部丢弃，不入FIFO、不混入现场预览requests、不刷新真实来源时间。`recorded_remote_warmup.discarded_predictions` 单独记录3个request ID、每次timing及总耗时；现场报告仍只包含指定数量的真实新动作块。launch与client同时严格核验compile=true、denoise_step、reduce-overhead、highest；模型SHA、处理器/tokenizer SHA、813张量完整加载证明和配置SHA检查保持。

## VLA独立异步诊断日志

`v8_r27`第3条接收端已接受，本地`commit`观测ACK时却晚于原期限55.93ms；真正RPC接收完成和本地ACK时间之间有同步INFO输出，旧记录无法把I/O与调度耗时精确分开。本轮未出现相机拒绝。原错误的“Disarmed, foreign or expired”在这里有明确过期证据，不表示第3条未发送。

`control_logging.py`覆盖官方构造、arm、运行及全部既有清理。控制线程仅复制日志记录及内置容器参数并放入4096项有界队列，格式化/原handler在后台执行；保留原记录时间、handler级别与过滤器调用。父`trial.py`先持久化收到的原始行，再异步镜像终端。满队列仅丢诊断记录或终端镜像，不重放动作、不改源时间、真实commit时间、Guard或TTL；动作trace仍走原路径。原handler身份在scope结束后恢复。

43项定向回归通过（日志10＋原20＋相机13）。200ms合成阻塞sink配合原Guard/Robot/backend代码：旧同步发送耗时200.789ms且晚53.582ms拒绝，新异步耗时1.207ms且提前145.811ms接受；仅一次target请求，原来源和期限不变。真实晚ACK及未知传输仍拒绝，不自动重试。

run目录内的`control_logging_statistics.json`、`terminal_mirror_statistics.json`记录队列丢弃、关闭丢弃、sink错误、pending及daemon状态。scope的0.25秒join有界；永久阻塞handler仍可能使Python logging/stdio进程退出flush等待，不保证整个进程有界退出。测试最终释放人为阻塞sink，未增加atexit/os._exit绕过。诊断统计写入失败保留原控制异常，父完整日志写失败请求子进程既有清理。

固定证据：`control_logging_verification_result.json`、`../control_logging_repair_v8/repair_receipt.json`。本次日志修复没有实机复测，改动前文件及失败trace保留在`before_control_logging_v8/`。

## VLA独立相机重采集

最新编译实机run`v8_r26`接受69条目标后，因全局相机`板端帧年龄 + 完整HTTP往返 >150ms`停止；失败时两项数值没有进入旧日志，不能据此诊断USB故障或侧边预览导致。19.2Hz警告本身不会退出。

`camera_recovery.py`只替换VLA的取帧backend，预览与执行共用。每组共享200ms截止、最多4轮，超时或配对过旧时只重取需要的一侧；读取future全部完成后才重试。读取、解码和RGB转换后再次检查期限，失败帧不提交元数据。接受帧保留原板端年龄＋完整HTTP往返的保守来源估计，不刷新旧帧时间；身份、HTTP503、帧号倒退及解码错误仍失败。原150ms年龄、60ms配对、400ms来源/槽位和TTL保持，不改共享ACT适配、官方LeRobot库或相机服务。

13项定向检查通过，验证记录在`camera_recovery_verification_result.json`；原20项离线检查也通过。原适配和新适配各30秒HTTP-only读取均600/600通过，新适配最大年龄42.53ms、最大时差25.91ms；这些读取不连接CAN/RPC，不证明真实超时已在现场恢复。该轮5组完整只读预览通过后，短实机有3条被接收端接受且无相机报错，随后第3条本地commit过期；本次日志修复保持此相机模块不变。记录在`../camera_recovery_v8/repair_receipt.json`；改动前文件与失败run保存在`before_camera_recovery_v8/`。

## 验证与当前限制

20项离线测试通过：包含40次官方FIFO选择仅2次预测、slot19来源时序、40个SDK及双端Guard合成目标、严格compiled/模型健康pins、3次录制预热先于live采集且丢弃输出，以及0.4/3秒超时在成功和失败后恢复、失败不重试。离线测试不连接网络或硬件。随后新服务真实测速（3次预热、10次测量）模型中位75.23ms、p95 80.42ms；HTTP上传至动作校验中位136.86ms、p95 145.78ms、最大147.08ms。报告：`../lerobot_fast_service/results/speed_v7T181251_973195Z/report.json`。该HTTP统计排除采集、图像编码和机械臂执行。

当前日志修复后的5组现场只读预览全部通过，原始来源至动作校验191.21–298.19ms，首命令静态计算最小剩余时效101.8ms、等待均为0。该预览只检查独立新动作块及首命令数值可行性，没有验证整块实机执行、未来反馈、完整抓放或无限运行尾延迟。10次测速p95仅描述该样本，不构成长时间无超时保证。

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 /home/aup/Lerobot/envs/piper-cpu/bin/python -B /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_inference_v7/cloud_grasp_act_bridge/offline_verification.py
```

历史5秒试跑62条目标全部接受，正常达到时长。无限50步方案第47条事务发送过期；随后20步方案在新块返回时超过来源派生槽位期限，日志 `trial/runs/v8_r24/rollout.log` 显示模型178.2ms、总388.9ms，采集耗时另计。因此本次保留20步，优化计算路径，不继续仅靠缩步或放宽Guard。

改动前生产文件、计划、批准和完整trial小日志在 `before_fast_runtime_v8/`，附38个文件SHA256清单。更早的5/60秒、runtime10/50/20配置及历史计划仍保留在既有baseline、before_queue20、plan_history目录。原模型与ACT均未删除。

双相机实时预览：http://127.0.0.1:18761/ 。Ctrl+C请求现有清理流程，不是物理急停；现场保持可立即断电。
