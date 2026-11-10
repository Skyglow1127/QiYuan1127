# PREACTION：ACT 动作块输入

本项目提供完整动作块接口、离线图像输入适配器和序列网络。2026-10-25 已进行首轮真实离线训练，使用保存的几何计划及对应执行图像，产物位于 `output/preaction_sequence_round1_20261025`。训练候选仅覆盖所记录的数据分布；原 ACT 执行审查结论保持原样。

## 新旧入口

| 入口 | 输入与用途 |
| --- | --- |
| `core.py` / `predict.py` / `train.py` | 保留旧接口：当前图像、当前 q、单个终点变化量和预测时长。原有权重与历史对照仍使用它。 |
| `sequence_contract.py` | 接收 ACT `ActionChunk` 或其字典，保存完整 10×7 绝对目标、10 个时间戳和显式掩码。 |
| `act_sequence.py` | 校验当前图像并生成模型输入；不生成未来图像。 |
| `sequence_model.py` | 冻结视觉编码器＋空间特征＋GRU，逐步消费完整序列，输出每步 4 个 ROI 平移/缩放参数。 |
| `audit_act_sequence.py` | 检查已有 ACT 离线提案的动作接口，保留原审查状态和原因；不逐条复核图片。 |
| `prepare_sequence_data.py` | 从哈希绑定的前置计划、真实执行前缀和图像构建逐步标签；原始非命令轴保留 `None`。 |
| `train_sequence.py` | 同预算训练完整序列与同容量终点条件模型，按有效标签计算损失。 |
| `evaluate_sequence.py` | 对照保持当前画面、终点模型及动作消融，保存逐时距指标和图像。 |

ACT 自己的双 RGB 输入保持原有设计。这里的 PREACTION 每次使用一个明确选择的相机视角，可以是 `global` 或 `wrist`；本轮未实现双视角融合。

## 调用逻辑

1. ACT 产生完整 `ActionChunk`，调用方同时提供该观察对应的实际 q、观察 ID、q 可用时间、预测开始和结束时间。
2. 调用 `build_act_sequence()`，显式传入 `valid_steps[10]` 和 `commanded_joint_mask[10][7]`。
3. 调用 `prepare_observation(packet, current_image)`，从保存的当前图像生成张量。
4. 将 `rgb_mask`、`q_features`、`sequence_features` 交给 `PreActionSequence`。新建对象的预测头仍是未训练状态；训练候选须通过 `train_sequence.load_trained_sequence()` 加载并校验其来源和 `conditioning_mode`。终点模式还必须应用 `conditioning_features(..., 'endpoint')`，不能只加载权重而跳过输入转换。

### 动作与时间字段

`chunk` 必须包含 `commands[10][7]`、`target_times[10]`、`observation_id`、`model_sha256`、`action_type="sdk_joint_position"`、`coordinate_frame="sdk_native"`。

- J0–J5 使用度；J6 使用未标定的 SDK 原始位置量。原始目标不裁剪。
- 时间使用同一单调时钟域的秒数，原值以双精度保存；10 个目标时间严格递增。
- `q_observed_at` 严格早于预测开始，年龄不超过 0.6 秒；预测结束不得早于开始。
- `valid_steps` 表示预测步本身有效，不能从历史教师标签的 `is_pad` 推导。
- `commanded_joint_mask` 表示该提案包含哪些轴的目标更新；不代表命令已发送，也不表示未更新轴实际静止。
- `future_at_completion_mask` 表示目标时间严格晚于预测结束。`valid_future_steps` 是它与 `valid_steps` 的交集，不是可执行许可。
- 已过期的点保留，不平移时间、不删除第一点；有效的过期提案仍进入序列网络，但不作为已执行动作或实测状态。
- 无效步不更新 GRU 状态且输出零；未指定轴的动作变化特征为零，原始值仍保存在包中。全无效和全过期均有明确掩码。

每步网络特征共 17 维：`目标减当前q[7]`、`轴掩码[7]`、`相对开始时间[1]`、`步有效[1]`、`结束时仍在未来[1]`。动作变化按 `[5,5,5,5,5,5,20]` 缩放；q 按 `[180,180,180,180,180,180,100]` 缩放，不裁剪。浮点特征为 float32，原始动作与时间另存为 float64。

### 当前图像字段

请求的 `current_image` 需要 `path`、`sha256`、`role`、`serial`、`available_t`、`board_age_at_fetch_ms`、`capture_age_estimate_s`。

- `global` 对应 `CC1365302K0`，`wrist` 对应 `CC1365301EY`；输入必须是原始 640×480 RGB 文件。
- `available_t` 早于开始，接收年龄不超过 0.25 秒；拍摄年龄估计同样不超过 0.25 秒。
- 拍摄年龄应等于 `开始时间 - available_t + board_age_at_fetch_ms / 1000`，一致性容差为 1 微秒。这是已有采集记录的估计口径，不是硬件曝光时钟测量。
- 同一份文件字节先校验 SHA，再解码，避免校验与读取之间内容变化。
- 本适配器用当前帧 HSV 最大红色区域裁切 96×96 ROI，未调用 Qwen/YOLO；没有可用红色区域时拒绝。不得把该区域当作物体身份或真值分割。

## 离线运行

在 `/home/aup/Camera` 执行。每次选择不存在的新输出目录，避免覆盖历史结果。

```bash
.venv-models/bin/python -m scripts.preaction.audit_act_sequence \
  --dataset output/act_lift_pilot_20261024/data \
  --review output/act_continuous_shadow_20261024_round1 \
  --output output/preaction_sequence_next_audit

.venv-models/bin/python -m scripts.preaction.act_sequence \
  --request output/preaction_sequence_next_audit/example_request.json \
  --output output/preaction_sequence_next_input
```

`example_request.json` 是完整请求范例，包含 schema `d1-preaction-act-request-v1`、上述动作/观察/时间/掩码字段及 `current_image`。历史回放的结束时间是“历史开始时间＋另行测得的推理耗时”，不能当作新实机观测。

适配输出包括 `sequence.json`、`inputs.npz`、`current_roi.png`、`summary.json`。`inputs.npz` 的主要形状为 `[1,4,96,96]`、`[1,7]`、`[1,10,17]`，并另存完整动作、时间和布尔掩码。

下面只做未训练网络的数据通路检查，不显示或使用它的随机预测结果：

```python
import numpy as np
import torch
from scripts.preaction.sequence_model import PreActionSequence

model = PreActionSequence()
legacy = torch.load(
    "output/preaction_unheld_round2_20261023/forecast/best.pt",
    map_location="cpu", weights_only=True)
model.load_legacy_vision(legacy)  # 仅复用 vision.*；新预测头需要另行训练
model.eval()
with np.load("output/preaction_sequence_next_input/inputs.npz", allow_pickle=False) as data:
    inputs = [torch.from_numpy(data[k]) for k in
              ("rgb_mask", "q_features", "sequence_features")]
with torch.no_grad():
    result = model(*inputs)
assert result["motion"].shape == (1, 10, 4)
print(model.training_status, result["future_mask"].tolist())
```

新检查点使用独立 schema；旧终点权重不能整体装入序列网络。保存函数拒绝覆盖已有文件，默认记录 `untrained_contract_test`。声明训练过也不等于预测质量已验证。

## 首轮序列训练与重跑

首轮使用全局相机 `CC1365302K0`。原腕部方案的持物图像在底边截断，无法提供完整框/尺度标签；原失败诊断保留在 `data/build_diagnostics.json`。`camera_amendment.json` 记录了首次训练前改用同一日志的全局图像，其他时序阈值、切分、预算均未变化。不能把这组权重声称为已验证的腕部相机预测器。

数据为 362 个训练窗口、22 个开发验证窗口，3443 个有效逐步标签；10点间隔0.2秒，覆盖至1.8秒，step0不提供未来标签。验证来自10月22日抬升，该会话以前也参与过ACT开发，因此没有新的独立测试集。原有历史val/test来源仍排除。更多相邻窗口不等于更多独立示范。

两种模式各训练300步，使用同一冻结视觉编码器、随机种子、数据顺序及14,332参数结构。终点对照重复最后一个有效动作目标，保留时间和有效标记；旧8秒终点权重没有被当作1.8秒基线。

重新运行相同来源的完整离线实验：

```bash
cd /home/aup/Camera
bash scripts/run_preaction_sequence_pilot.sh output/preaction_sequence_repeat_new
```

输出必须是新目录。该命令依次构建数据、训练两个模式、生成评估结果；使用现有 `inventory.json` 和 `protocol_global.json`，不会创建新采集数据。

最终候选：`train_sequence/best.pt`；终点对照：`train_endpoint/best.pt`；指标：`evaluation/evaluation.json`；图像：`evaluation/validation_examples.png`。这些路径相对于本轮结果目录。

## 数据边界与后续补采

训练样本必须把**当时已知的完整计划**与该计划实际执行后的图像逐步对齐，记录执行偏差、中断和有效标签；按完整采集会话拆分训练与验证。当前保存的 ACT 提案均未执行，其后历史教师画面不能充当这些提案的效果标签。

本轮保留了每步计划、实际命令时间及图像标签时间，缺失/截断标签有独立 `label_mask`，不改写输入动作有效性。未来图像只作弱监督，当前ROI固定用于所有未来帧。单轴未命令值不会补成七轴保持目标；几何来源也不会伪装成ACT模型生成的提案。

训练结果需结合动作消融和数据覆盖理解。当前全局视角的接近、空夹爪阶段红方块几乎不动，持物阶段短时变化也很小；目标ROI的平移/尺度预测不能替代夹持、接触、碰撞或整臂状态判断。补采清单由本轮结果报告给出，真实动作需另行制定具体计划。
