一键抓取程序隔离主机 CLI

来源：02_程序与源码/D1/Control 的 CMake、arm_control.cpp、src、msg 及主机测试。
此副本只用于 09_一键抓取程序；原归档源码和 runtime/Control/build 符号链接未修改。
独立输出：../host_build/arm_control。

新增显式 --bootstrap-position-once：仅正常单轴 funcode 1 非零位置目标，可在新鲜（250 ms 内）power=1、enable=0、error=0 时启动一次。
必须由上层绑定到用户已确认的具体路径。此开关不会产生开启使能指令，不会生成同位、回零、阻尼或微调目标。
实际全部关节范围、J6 机载 0..50 范围、目标范围、三位小数量化后非零位移均须通过。
发送后要求本 seq 回执与发送后的新 enable=1/power=1/error=0 及实际角度帧；既有位置闭环和保护轴漂移检查仍生效。
启动机会在第一个正常位置发布时消耗，即使当时 enable 已为 1，也不允许在中途掉使能后再次启动。
非零检查复用实际 encode 格式，避免浮点半界值在判断和线上小数之间不一致。
启动态确认失败后本进程停止运动，无自动重发。缺省 CLI 行为不变。

构建：cmake -S host_src -B host_build -DBUILD_TESTING=ON
      cmake --build host_build --target arm_control protocol_tests dds_test_peer
离线测试：ctest --test-dir host_build --output-on-failure
隔离回环测试：cmake --build host_build --target position_bootstrap_integration_test
                cmake --build host_build --target dds_integration_test
隔离测试仅用网卡 lo、DDS domain 231 和 d1_test_command/d1_test_feedback 话题。

2026-10-26 用户授权反馈抖动容差：
J0..J5 实际反馈在原限位外最多 0.2 度（加 float32 表示量级误差）可通过；
这只适用于读取到的实际反馈，原始值不改写。J6 无新增容差，bootstrap J6<=50 检查不变。
正常运动前、运动中的新反馈及首次位置启动统一检查；超过容差会报告轴、原值、原限位与容差并停止后续命令。
checkAngle/命令目标限位保持原值，超限反馈不能直接生成超限同位目标。hold-current 仍禁用。
定向隔离测试：cmake --build host_build --target feedback_jitter_integration_test

2026-10-26 用户授权逐轴角度反馈过滤（默认关闭）：
显式 --feedback-max-speed <正有限 deg/s> 使用执行计划的最大关节速度。
J0..J5 各自比较 abs(当前原始角度-该轴上一有效角度) > 3*设置速度*该轴 dt。
仅拒绝异常轴，其它有效轴继续更新；被拒轴的位置、接收时间、序号不变。J6 不参与角速度判别。
初始各轴必须有限且通过原反馈限位；已有基线后 J6 有限越限值、arm 原 cone 内缓慢越限仍由原硬限位失败。
某轴 pending 时，连续两条有界 raw 值若满足该轴 3*速度*raw dt（dt>0），允许重捕获；超限/无效值清除该轴候选。
原 [rx] 与 marker.message 原始报文保留，不生成同位保持命令，不裁剪或改写被拒绝值。

State 保存七轴独立 jointTimes/jointSequences/jointPendingSince。angles 仅全部轴初始化后存在，
angleTime 为最新有效轴的接收时间，angleFrames 任一轴接受即增加；这些汇总字段不代替逐轴运动检查。
所有轴均 <=0.6 秒且无 pending 才可发送已批准轨迹的新目标。单轴超时只记录并暂停整条轨迹，
只有全部七轴的有效反馈年龄都达到 --feedback-recovery-timeout-ms（默认 2000，范围 601..60000）才整体超时。
没有任何有效轴时初始等待仍有限；部分轴从未初始化、其它轴健康时保持暂停，等待恢复/取消/明确硬故障。
状态、ACK 和拒绝值不能刷新任何有效轴时间。power/enable/error 不允许值、真实限位及否定 ACK 仍立即失败。
[joint-feedback-stale]/[joint-feedback-recovered] 仅在单轴状态迁移时记录；回调和等待循环均检查，避免每帧刷屏。
首次位置启动与 hold-current-check 仍要求七轴各自 <=250 ms；位置闭环到位计数仅消费七轴均在命令后独立更新的样本，
每轴值不能重复攒到位次数，保护关节漂移仍在每次读取时检查。

每条角度 raw 对应一个 [feedback-accepted]/[feedback-rejected] JSON wrapper；任一轴接受即 accepted 标签，
因此必须读取 accepted_joints/rejected_joints/joint_pending/joint_rejections，不能凭标签断定整包有效。
joint_times_s[7]/joint_sequences[7]/merged_angles[7] 为融合后的逐轴状态；未知时间/位置为 null、序号为0。
received_monotonic_s 为该 raw 的真实本机接收时间；rejected 轴时间绝不会改成该时刻。
pause_total_s 为连续任一 pending 段结束后的累计暂停秒数；gap 日志仅诊断等待，不伪造 q_time。
all_feedback_gap_count 持久累计相邻任意有效轴更新之间超过整体硬超时的次数；
last_all_feedback_gap_start_s/end_s 保存最近一次该间隔的源接收时间（未知为 null）。
等待期间计数增加即整体超时，避免等待线程晚醒后被新鲜帧掩盖已发生的全轴中断；持续有效流不会误记。
makeFeedbackMarker 与真实 Controller 共用，支持不启动 DDS 的跨层离线夹具。
