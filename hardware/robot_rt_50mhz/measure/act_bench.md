# ACT本机十分钟推理性能

已实际连续运行 **600.056秒 / 3691次**，退出成功。加载和3次预热不计入该时长。

| 项目 | 实测 |
|---|---|
| 处理器/后端 | Ryzen AI 9 HX 370；Torch2.10.0+cpu；FP32；CPU8线程 |
| 前向延迟P50/P99 | 159.765 / 193.070ms |
| 内存中录制观测到物理动作P50/P99 | 160.833 / 194.250ms |
| 周期采样RSS峰值 | 774.42MiB |
| 末60秒/首60秒前向P50比 | 0.9964 |
| 模型chunk/当前消费 | 100 / 30 |

20Hz数据时间轴下30步为1.5秒，半周期预算750ms；本轮P99低于该预算，保持30步和关闭temporal ensemble。单次前向仍超过50ms，不能据此配置20Hz逐步新推理。没有改动当前部署消费步数。

输入是episode41的四组已保存RGB/state，反复推理以测持续负载。观测到动作口径包含RAM转换、官方预处理、前向、官方反归一化，排除磁盘读取、相机曝光/传输、网络、CAN和机械响应。该测量是CPU结果，未测890M GPU或NPU。

周期频率/温度快照与延迟趋势不能证明完全没有热/功耗降频。资源检查包括初始主机只读核查、旧名称规则的周期检查和约360/480秒的精确名称抽查，不是全面持续监控；原有相机和其他服务未停止。

原始记录在 `bench_original_cpu_600s/timings.csv`、`telemetry.jsonl`，统计、来源哈希和实际加载的源码快照在同目录 `results.json`、`act_at_start.py`、`act_runtime_at_start.py`。基准结束后仅修正了不在计时路径上的Q16端点编码；最终代码B5已另行重跑，见 `alignment_original_ep41_q16_fixed/verification.json`。

在robot_rt根目录复现，须使用新输出目录：

```bash
/home/aup/Lerobot/envs/piper-cpu/bin/python measure/benchmark_act.py --duration 600 --threads 8 --output-dir measure/bench_cpu_rerun
```
