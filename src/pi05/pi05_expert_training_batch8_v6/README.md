> 来源文档快照：下文的“当前”、路径和命令对应原记录时间；本次选定版本与验证边界见 [提交状态](../../../docs/RESULTS.md)。

# π0.5 动作专家模型 B：batch=8 训练配置

实验版本：v6。实验 ID：`pi05_expert_only_v6_r09`。

## 训练方法与固定配置

| 项目 | 新模型 B |
|---|---|
| 初始模型 | 与 A 相同的原始 `pi05_base`，不加载 A 或旧 B 的权重/优化器 |
| 更新范围 | 原始动作专家，以及 `action_in_proj`、`action_out_proj`、`time_mlp_in`、`time_mlp_out` |
| 冻结范围 | 完整 PaliGemma VLM |
| PEFT / LoRA | 禁用且不加载 |
| 框架 | LeRobot 0.6.1，保留官方安装源码，使用独立包装入口 |
| Batch / seed | **8 / 1000** |
| 正式步数 | **20,000**，独立 smoke 和续训检查通过后从 base 新启动 |
| 学习率 | 峰值 `2.5e-5`，衰减目标 `2.5e-6` |
| 调度 | 配置 warmup 1000 / decay 30000；官方实现对正式 20k 自动缩放到 **666 / 20000** |
| 精度 | BF16，保留官方指定的 FP32 模块；gradient checkpointing 开启，compile 关闭 |
| 动作 | 绝对动作，chunk 50，`n_action_steps=10`，推理去噪 10 步 |
| 验证 / 保存 | 正式计划每 2000 步，验证全部留出帧 |

按 batch × 步数估算，20k 约处理 160,000 个训练起点，相当于 160000 / 18231 ≈ **8.776 轮**。这个数不表示任务成功率或训练效果已得到验证。

名义可训练参数为 693,422,112（209 张量）。其中专家 `lm_head.weight` 在官方动作前向中未被调用，预期没有梯度与优化器状态；有效动作计算分支约 430,098,464 参数。入口审计准确的参数范围、优化器集合、有限损失/梯度、原始专家更新及完整 VLM 冻结 hash；严格加载检查全部模型张量和非持久 buffer。batch=8 的真实 GPU 短测未出现 OOM 或非有限数值；观察到 CUDA reserved 峰值约 15.53 GB，完整 checkpoint 为 11,549,234,497 字节。原模型 A、base、部署文件和数据元信息与旧实验启动前的完整清单一致。

## 数据、单位与模型 A

沿用原 LeRobot v3 数据集：52 条轨迹、20,488 帧、20 Hz。训练 episode 固定为 **0–45，18,231 帧**；验证为 **46–51，2257 帧**。入口会检查实际划分和帧数，并保存元信息校验值。

七维 state/action 顺序为 J1–J6（度）和夹爪（米）；state 是执行臂反馈，action 是绝对目标。相机为 `observation.images.global` 和 `observation.images.wrist`。沿用 A 的统计、tokenizer、图像方向和任务文字，不改单位、相对动作或标签对齐。

任务文字：`Pick up the red cube and place it inside the black square on the left side of the table.`

模型 A 云端目录：`/root/autodl-tmp/piper_pi05/runs/pi05_train_v5_r06`。

A 本地完整备份：`/home/aup/Documents/Codex/v5/wo-y/outputs/piper_pi05_cloud_backup_v5`。

A 最终 adapter SHA256：`8e064a68c5991f1b1321e726d901968c516ec728971aa4b9eb1fc6d4213f14a1`。原始 base SHA256：`0eb11ca9587678c1d2ef8cf32807c29f8ce53a2bfdfc1aa4a4c96f16fca59b0f`。新编排继续保存 A、base、部署文件及数据元信息的前后校验，不覆盖或合并 A。

## 目录与存储策略

本地新入口：

```text
/home/aup/Documents/Codex/v5/wo-y/outputs/pi05_expert_training_batch8_v6/
  train_expert.sh
  train_expert_checked.py
  pi05_low_memory.py
  remote_job.py
  watch_and_backup.py
  check_status.py
  launch_receipt.json
```

云端共享只读资产：

```text
基座：/root/autodl-tmp/piper_pi05/models/pi05_base
Tokenizer：/root/autodl-tmp/piper_pi05/models/paligemma_tokenizer
数据：/root/autodl-tmp/lerobot_act/datasets/rad_place_merged_v5
```

新 B 的目标路径（创建和完成状态以启动后记录为准）：

```text
数据盘：/root/autodl-tmp/piper_pi05/experiments/pi05_expert_only_v6_r09
  bin/                    独立入口
  job_status.json         当前阶段与子进程/日志位置
  logs/                   smoke、resume、formal 日志
  best_model/             最佳验证 checkpoint 的完整 pretrained_model
  best_model_receipt.json 最佳 step、验证指标和身份

系统盘：/root/pi05_expert_runs/pi05_expert_only_v6_r09
  smoke/                  独立 100 步和续训到 101 步的检查
  train/                  正式训练
    audit/                配置、split、参数范围、实际 LR 和保存校验
    checkpoints/last      最近完整 checkpoint 的软链接
    checkpoints/020000/   正式计划的最终 checkpoint
```

B 保存的是完整策略：模型约 9.35 GB，连同优化器等续训状态约 11.55 GB，实际以本次保存测量为准。系统盘仅保留最近一份完整 checkpoint，先写入并校验下一份，再删除已验证的旧 B checkpoint；因此须保留同时容纳两份的空间。best 完整模型放数据盘，原子替换也需要新旧双份余量。`last` 是软链接，不额外复制权重。

smoke 与续训检查通过后，保留审计/hash 清单，清理仅本实验自产的 smoke 大权重，正式运行从 base 重启。新备份 watcher 已启动，PID `724911`，绑定本实验 ID；独立备份目录为 `/home/aup/Documents/Codex/v5/wo-y/outputs/piper_pi05_expert_cloud_backup_v6/pi05_expert_only_v6_r09`。每 180 秒只读查询，云端完成后下载最终完整 checkpoint、best 模型、日志与审计，逐文件验证 SHA256。当前仍在等待训练完成，不能当作备份已完成；须以 `watcher_status.json` 的 `complete_verified` 和 `verification.json` 为准。电脑和现有 SSH master 需保持在线。

## 恢复与对比限制

续训需同一 checkpoint 的 `pretrained_model/` 与 `training_state/`，后者包含优化器、参数组、scheduler、RNG 和 step。使用该 checkpoint 的 `train_config.json` 与 `--resume=true`，恢复本实验环境和 batch=8；仅 `best_model/` 不含独立续训状态。恢复前确认没有同实验训练子进程或同卡推理占用，避免重复启动 supervisor/fresh 脚本。

100→101 续训测试改变了总步数，官方调度闭包会重新缩放，因此验证的是能加载并继续，不是与未中断 101 步训练逐值等价。故障时先保留日志及最新已验证 checkpoint；新编排不会自动恢复 A 推理，后续需要使用 A 时另行明确启动并检查服务状态。

B 相对 A 同时改变了训练方法、batch、学习率和样本曝光量（约 160k 对 40k），属于两套完整方案对比，不能将效果差异全部归因于是否使用 LoRA。保持相同的验证输入、目标窗口、处理器、10 步推理和预先记录的噪声，保存原始输出，分别报告关节度误差、夹爪毫米误差及延迟。报告最终模型及最佳验证模型；验证损失不是实机抓取成功率。

现有 A 云服务按 LoRA 身份加载；B 完整权重需要独立加载/部署验证，训练完成不自动替换 A。训练和备份不授权机械臂运动；实机抓取仍须展示目标、路径和关键参数并获得本次明确批准。

## 本地只读状态命令

```bash
python3 /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_expert_training_batch8_v6/check_status.py
```

此命令复用已有 SSH master，只读取当前阶段、最新优化步、日志尾部和 GPU 使用情况；输出同时保存为本地 `latest_cloud_status.json`。

实时查看正式训练日志（在本机终端）：

```bash
ssh -S /home/aup/Documents/Codex/v5/wo-y/outputs/pi05_cloud_grasp/runtime/cloud.sock -p 29596 root@116.172.66.186 'tail -n 2 -f /root/autodl-tmp/piper_pi05/experiments/pi05_expert_only_v6_r09/logs/formal_training.log'
```

进度条中 `[已用时间<预计剩余时间]` 可查看训练耗时，验证和保存另需时间。Ctrl+C 只退出日志查看，不停止云端训练。
