# 当前成果与历史迭代

本页区分原工程已有证据与 发布版本 v1 的副本检查。没有在本次整理中启动相机、机器人、云端训练、推理服务或 Vivado 构建。


## 成果导航与迭代顺序

| 阶段 | 已有成果 | 对应材料 |
| --- | --- | --- |
| D1-T 初始测试 | 控制接口、感知规划、轨迹执行方案与回归用例 | `src/d1/` 与下方历史说明 |
| ACT 数据验证 | 数据录制与训练链路、020000 权重、执行后端和历史任务记录 | `src/piper/`、`src/lerobot_workspace/`、`results/piper/` |
| π0.5 方案探索 | 专家 B 20000 步训练及选定完整模型 | `results/pi05/` 与模型下载清单 |
| π0.5 + LoRA 本地化 | LoRA 020000 本地推理、低内存加载与同步/异步离线对照 | `experiments/pi05_lora_local_inference_v10/` |
| FPGA 工程交付 | 已完成布线、时序通过、固件加载与两项真实板卡测试 | `hardware/fpga_project_20261104/` |

历史记录用于追溯具体版本的试验与改进；当前 FPGA 交付和本地 LoRA 实验分别由各自的新报告支撑。

## 已有软件回归检查

选定测试共 171 项通过，机器可读结果见 [packaging_validation.json](results/packaging_validation.json)。

| 套件 | 数量 | 验证内容 |
| --- | ---: | --- |
| D1 | 38 | 虚拟时钟与机械臂共同进度、反馈与命令边界区分、原失败片段回放 |
| PiPER | 74 | 目标投影、目标速率、SDK 编码、Guard、ACT 动作块来源时序 |
| π0.5 | 24 | 闭爪协调状态机、异常输入、超时与状态提交条件 |
| 无损协议 | 13 | 图像和数值精确还原、畸形请求拒绝、模型配置与既有服务一致性 |
| FPGA 软件参考 | 22 | 定点与整数运算、舍入、轨迹参考与测试向量 |

这些检查针对选择的接口和回归案例。其余随源码保留的测试可能需要完整 SDK、ROS、模型、历史夹具或板卡环境，未声称全部通过。

## 历史阶段：D1-T 初始机械臂测试

收录 `shared_progress_v1` 及 `timed_group_v1` 候选。原交接指出：共同进度版本未通过本版本实机运动验证，带时整组下发未编译、测试或部署。本次选择的 Python 回归不改变 C++ 和机载候选的状态。

历史方法一报告记录原参考跨度 12 秒、发送跨度约 21.4505 秒及 J4 约 1.218° 稳态残差。它用于说明改进动机，不是当前新算法的实机成绩。

- [软件验证记录](../src/d1/validation/software_validation.json)
- [历史方法一报告](../src/d1/validation/historical_20261026_215128_2e42b2/report.html)
- [硬件与历史故障分析](../src/d1/validation/hardware_assessment.md)

## 数据验证阶段：PiPER ACT

本次读取的 `act_launch_contact_repair.json` 为原 020000 权重、30 步、alpha 0.7、默认 30 秒。该版本来自用户指定的历史恢复，包含原 60 mm / 150 ms / 2 秒开口确认逻辑；它不是后来 50 步、alpha 0.5、加速度软整形版本。

- [选定配置](../src/piper/deployment/act_launch_contact_repair.json)
- [恢复来源清单](results/piper/selected_baseline_manifest.json)
- [30 步离线验证](results/piper/act_action_steps30_v7/verification.json)

历史运行记录包含用户确认的整段任务完成，但仍有抖动、放置和回撤碰块问题；没有稳定成功率统计。后来 50 步版本有 2826 条 accepted、相机 HTTP503 停止的记录，**该记录不作为当前 30 步副本的验收结果**。

- [后续版本历史运行审查](results/piper/act_run_review_v8/latest_run_review.json)
- [旧开口确认超时问题分析](results/piper/act_release_timeout_fix_v7/verification.json)

## 模型对照阶段：PiPER π0.5 专家 B

专家 B 从原 π0.5 基座训练，冻结 VLM、训练动作专家，未使用 LoRA 适配器。保存记录确认 batch8、20000 步完成；best004000 留出验证损失约 0.06722。best 与 final 是不同完整模型，不能混配预处理统计。

52 条示范共 20488 帧、20 Hz；B 的训练 episode 为 0–45，验证为 46–51。ACT 原训练使用全部 52 条，没有同样的留出划分，不能直接把两者损失作为公平成功率对比。

- [训练完成记录](results/pi05/latest_remote_status.json)
- [best004000 选择记录](results/pi05/best_model_receipt.json)
- [训练方案差异](results/pi05/comparison_manifest.json)

已有实机动作尝试，未证实完整抓放成功。闭爪协调和无损服务是各自有明确边界的模块；原说明中的端口、方案 ID、时间和“当前”字样属于当时记录。当前线上服务是否仍存在没有在本次核验。

## π0.5 + LoRA 本地离线推理

新增 [本地推理实验归档](../experiments/pi05_lora_local_inference_v10/README.md)，使用 π0.5 基座 + LoRA 020000，与上节专家 B 完整权重实验分别记录。

已有 Radeon 890M 同步测速：10 步流匹配预测中位时间 1.3833 秒，输出完整 50×7 动作块；每种设置预热 3 次、计时 10 次。完整两轮同步/异步对照中，20 步消费窗口的模拟有效动作频率为同步 9.02–9.03 Hz、异步 14.62–14.83 Hz；50 步窗口下异步约 20 Hz。

这些数据使用重复的历史录制观察，计时不含采图、输入预处理和模型加载；有效频率排除初次预测等待。没有机器人通信、RTC 前缀指导或过期动作裁剪，不作为实机闭环频率和任务成功率。具体参数、报告、少量样本试跑及历史未完成记录在实验目录中分别保留。本次仅做归档检查与假模型调度自检，未重跑 GPU 测速。

## FPGA：当前交付版本

当前工程入口为 [FPGA 工程文件](../hardware/fpga_project_20261104/)，源码、工程配置与上板产物均按目录提供。

- [产物清单](../hardware/fpga_project_20261104/上板文件/artifacts.sha256.json) 记录 `timing_passed=true`，并给出 `.bit`、`.bit.bin`、`.xsa` 和地址配置的 SHA-256。
- [布线报告](../hardware/fpga_project_20261104/上板文件/route_status.rpt) 记录 174451 条可布线网络全部完成，路由错误为 0。
- [时序检查](../hardware/fpga_project_20261104/上板文件/reports/check_timing.rpt) 记录各列出检查项均为 0；[布线后时序报告](../hardware/fpga_project_20261104/上板文件/reports/timing_route.rpt) 随工程保留。
- [综合后算术检查](../hardware/fpga_project_20261104/仿真结果/综合后算术检查/verification.json) 记录 `pass=true`、`complete=true`。这些是已有结果，本次整理没有重跑仿真。

### 后续真实 KR260 上板测试

新增 [上板测试成果](../hardware/fpga_project_20261104/上板测试_20261104/)，其固件与仓库中现有 `.bit/.bit.bin/.xsa` 的哈希一致。静止测试和 ACT 回放记录中的构建清单 SHA-256 均为 `d9fbcfcd53bf80ed9026b3f5a2b361d5c0d47c4603dbaf9a82170a92cd6e9f91`，对应当前上板文件。

| 测试 | 结果 | 原始证据 |
| --- | --- | --- |
| 固件加载与身份检查 | `load_test_passed=true`；Linux 报告 PL0 为 50000000 Hz，RTL 名义参数为 49999500 Hz | [加载与静止测试状态](../hardware/fpga_project_20261104/上板测试_20261104/02_上板加载和静止测试_通过/status.json) |
| 静止数据板端测试 | `stationary_test_passed=true`、`hardware_access=true`；输出计数增加 44 | [静止测试结果](../hardware/fpga_project_20261104/上板测试_20261104/02_上板加载和静止测试_通过/stationary/result.json) |
| ACT 录制轨迹板端回放 | `pass=true`、`hardware_access=true`；10 个正常动作段、44 个减速段和 2 条保持输出，共 56 条输出；含初始状态共 57 个快照 | [ACT 回放结果](../hardware/fpga_project_20261104/上板测试_20261104/03_ACT轨迹上板测试_通过/result.json) · [板端快照](../hardware/fpga_project_20261104/上板测试_20261104/03_ACT轨迹上板测试_通过/captures.json) |

因此，当前可以明确记录“**FPGA 已成功上板，静止数据和 ACT 录制轨迹板端回放测试通过**”。本节验证范围为固件加载、静止数据处理及 ACT 录制轨迹的 FPGA 描述符回放。

原固件清单中的 `board_tested=false` 是构建时的历史状态，保留原文以维持产物身份；后续板测状态以上表为准。静止检查工具中的 `board_acceptance=false` 对应其整体验收范围，不否定单项加载与静止测试通过的记录。此次入库只核对文件哈希、既有快照序号、故障码及输入身份，未重新上板或仿真。


### FPGA 历史构建阶段

早期 [50 MHz 构建状态](history/fpga_50mhz/status.json) 记录 `stage=implement`、`pass=false`。该阶段资料现归档在 `docs/history/fpga_50mhz/`，用于保留从实现阶段问题到后续布线、时序及产物输出的迭代过程，不作为当前交付版本的状态。

原 `hardware/robot_rt_50mhz/` 保留为早期源码与离线回归基线；最新交付从 `hardware/fpga_project_20261104/` 获取。两份源码不混用已有验收记录。

### 视觉预处理实验

[VPR3 保存验证](results/vision/vpr3_validation_latest.json) 对应独立图像预处理实验。相关 RTL、驱动、ROS 2 接口和 V2 HLS 源码已保留；构建依赖外部 Vitis Vision 和冻结 HLS IP，见复现说明。
