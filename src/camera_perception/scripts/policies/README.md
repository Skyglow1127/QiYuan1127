# D1 ACT 离线初版

2026-10-24。当前新增的是可运行的 ACT 离线训练、Qwen 任务解析和历史画面回放链路。已有实机抓取程序、SDK、相机配置和 PREACTION 保持原样。**完整抓放仍由现有几何系统承担；ACT 目前只训练抬升片段，未部署实机。**

## 已实现的输入输出

- 输入：全局和腕部 RGB，固定 global/wrist 顺序；命令前可得的七维实际关节状态。
- 输出：10×7 SDK 绝对位置，5 Hz 对应约 2 秒预测。内部可学 `target - current_q`，再加回观察时的实际 q；这与把未来反馈当标签不同。J0～J5 为度，J6 为未标定 SDK 位置量。
- 训练：冻结 ImageNet ResNet18，复用 ACT 作者的 CVAE、Transformer、动作 query；D1 适配七维输入投影，hidden=128、两层编码/解码、latent=32、L1+10×KL。不是原论文 80M 模型/50 Hz 复现。
- 部署预测：z=0，不传未来动作、不传未来图像；模型常驻回放进程。冻结特征按 float16 缓存后转回 float32，与回放一致。
- OpenCV：图像解码、固定缩放、RGB 转换、红色组件诊断。它不生成 ACT 的动作标签或动作目标。
- Qwen：现有本地模型产生 TaskSpec，严格解析后只路由已登记阶段。模型没有微调，未把 Qwen 包装成原生 VLA。ACT 路由仅给 lift 的 shadow；执行来源仍标记 geometry。

## 可重复运行

在 `/home/aup/Camera` 执行：

```bash
bash scripts/run_act_pilot.sh output/act_pilot_new_run
```

输出目录必须不存在。配置为 `configs/policies/act_lift_pilot.json`。环境用已有 `.venv-models`，本次只新增 IPython 9.10.0 及其依赖（作者模块导入它），没有升级 torch/torchvision。训练只用 CPU；没有使用 NPU、FPGA，也没有自动提升线上模型。

单独回放：

```bash
.venv-models/bin/python -m scripts.policies.shadow \
  --dataset output/act_lift_pilot_20261024/data \
  --checkpoint output/act_lift_pilot_20261024/train_residual/best.pt \
  --output output/act_lift_new_shadow --count 20
```

连续历史观察接口审查（不训练、不发送动作）：

```bash
.venv-models/bin/python -m scripts.policies.continuous_shadow \
  --dataset output/act_lift_pilot_20261024/data \
  --checkpoint output/act_lift_pilot_20261024/train_residual/best.pt \
  --split all --output output/act_lift_new_continuous_shadow
```

该入口按原始连续顺序逐帧预测，在缺样、时钟间隙和段边界清空衔接历史。分别审查当前实际 q 到候选、此前真实已发送目标到候选、相邻未发送模型候选之间的速度／加速度；计算使用各自实际时间间隔。没有先前速度时，加速度标为不可判定，不补零通过。J6 保持原始 SDK 位置语义，不套用机械臂角速度单位。

候选调度**另作明确离线假设**：在历史命令时刻触发推理，保留原始动作时间，选原始第 1 点（下一 0.2 秒 tick）作为单步候选。第 0 点的原始截止时间已经到达，单独记录其过期；第 1 点若赶不上截止时间也拒绝，不平移整块时间、不追赶、不裁剪。该索引选择与配置 `execute_prefix_steps=1` 所表示的“执行步数”不同，不修改线上配置。历史老师轨迹上的后续 q 和图像只作为下次记录观察，绝不称为执行 ACT 后得到的反馈。`protocol.json`、`predictions.jsonl` 和 `summary.json` 保留这一假设、原始预测、拒绝及不可判定原因；即使数值诊断通过，`live_eligible` 仍为 false。

`summarize_continuous_shadow.py --dataset ... --review ... --output ...` 可只读保存的预测，追加按 train/val 分列的教师目标对照，无需再次推理。未来教师目标仅用于事后对照；不足两步或 padding 分别计为不可用，不当成通过。首步所需位移／时间也受教师命令与实际 q 的跟踪差影响，因此拒绝意味着当前直接接入调度假设未通过，不能单凭它认定实物危险或 ACT 能力失败。第 0 点过期是原时间定义导致的结构性结果。现有 policy contract 的 J6 范围仍为 [0,65]；已保存机载目标范围与其交集为 [0,50]，补充报告单列此差别和候选实际范围，不把旧诊断称为完整机载范围验证。

本地 Qwen 高层解析（仅保存图像）：

```bash
.venv-models/bin/python -m scripts.policies.task \
  --image /absolute/path/to/global.jpg \
  --instruction '抓起红色方块，抬高10厘米，再放回原位' \
  --output output/act_new_task
```

## 数据和评估的边界

数据为 10 月 23 日抬升段训练、10 月 22 日抬升段验证，按整个操作会话分组。连续窗口会重叠，不能把 781/223 个窗口当成独立抓取次数。实际完整抓取示范数为 0。缺帧、无效反馈、超时、未命令轴、故障和分段间隙不补成专家目标。现有原始命令、实际反馈、图像 SHA 和标签证据保留在 causal 和 manifest 中。

首轮绝对目标模型跨姿态误差较大，保留在 `train/`；随后调整为相对当前 q 的内部动作表示，保留在 `train_residual/`。第二种表示使用了同一验证集，属于开发比较，不是独立测试。不能再将这组验证表现当新测试成功率。最佳 checkpoint 由验证集部署先验（z=0）误差选择，测试/真机未参与。

`shadow.py` 只检查数值、限位和内部速度/加速度，明确记录 `collision_checked=false`、`live_eligible=false`，不伪装为完整碰撞验证。不会导入 SDK 或发送位置。未来真实接入还需要连续轨迹检查、单一 SDK 调度、反馈健康、实时新帧以及当次具体路径批准；不能直接把 predictions.jsonl 重放到机械臂。

## 下一轮实际改进

当前模型对平均动作基线没有稳定优势，打乱视觉也几乎不变。先补同一任务不同初态/不同方向的连续成功示范，特别是接近、闭爪、已持物、释放的状态转换；不是继续对现有两段加训练步数。起步预算仍为约 20 条完整示范、按整次操作划分训练/验证，另留新物理执行评估。该数目是项目预算，不是保证成功的下限。

最新实机采样记录存在 J4 合法 JSON 异常跳变，STOP 仍保留。本工具不宣称排除了该故障，也不为离线训练执行运动。实机抓取遵循当前长期约定：展示目标、路径、参数后取得明确批准。

## 来源和许可证

- 本地 Skill：`/home/aup/TEAM Skills/具身智能知识库检索与方法复用/SKILL.md`。
- 本地 ACT 笔记：`/home/aup/具身论文及技术/批次归档/01-具身基础十篇/papers/P06-ACT.md`；沿用已核验笔记，本轮没有重读整篇 PDF。
- 作者实现：https://github.com/tonyzhaozh/act ，固定提交 `742c753c0d4a5d87076c8f69e5628c79a8cc5488`。`vendor/act/upstream.json` 保存下载原件 SHA。上游文件未修改；D1 适配在 vendor 外。MIT 与 DETR Apache 许可证均保留。
- 相比作者配置：冻结骨干、双相机、七维单臂、小 Transformer、5 Hz、可选相对当前状态的内部输出；没有主从遥操作或论文全任务复现。
