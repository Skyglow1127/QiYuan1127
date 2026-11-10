# 时延感知的智能驾驶避险系统

这是一个可在电脑上运行的**结构化观测＋纵向控制仿真原型**。它研究：周边车辆的信息已经过时，系统应如何估计风险、控制加减速，并同时衡量碰撞与行驶进度。

当前场景是双车道直路、1 辆自车和 **1 辆目标车**。自车保持车道；目标车按脚本执行正常跟车、急刹或切入。输入是带时间戳的位置、速度和稳定目标编号，支持延迟及噪声。首版固定 **50 Hz 仿真、10 Hz 观测与控制**，其他频率会被显式拒绝。该原型不是自动驾驶全栈，没有摄像头感知、真实目标关联、地图定位、实车执行或道路安全验证。

本文件说明怎样运行代码。具体开发范围见 [AI开发任务书.md](AI开发任务书.md)，面向使用者的推进方式见 [开发流程指南.md](开发流程指南.md)，当前已完成事项与实际结果以 [STATUS.md](STATUS.md) 和原始实验文件为准。本文不预设学习模型优于解析方法，也不承诺比赛晋级或获奖。

## 10月24日新增成果

新增四目标IDM/MOBIL交互场景，96回合验证无碰撞/越界，36项测试通过。报告和演示见 [今日工作成果](deliverables/20261024/今日工作成果.md)。推荐控制仍为ca_recovery。

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_demo.py --scenario traffic --method ca_recovery --delay .2 --calibration runs/release_v1/model/calibration.json --video
```

固定验证已运行于`runs/traffic_20261024`；`scripts/validate_traffic.py`会拒绝覆盖该目录。后续新验证使用新脚本/输出目录，不重写已有证据。

## 当前推荐版本

第三版已完成自行核查、修订及600回合新测试，34项测试通过。推荐方法 `ca_recovery`：常规和压力场景的jerk指标分别下降约56%和47%，平均行驶进度基本保持。结果及运行命令见 [修订后结果报告](deliverables/修订后结果报告.md)，演示见 [新版急刹回放](runs/revision_v3/videos/brake.mp4)。学习模型尚未显示实际有意义的额外驾驶收益。

## 已完成的首版（历史）

正式工作目录为 `/home/aup/AIC/aic 初版`。当前已完成 1,800 回合主实验、360 回合压力实验，31 项测试通过，且在本目录重新安装环境、复跑三类场景并核对了指标。

直接查看 [当前结论](STATUS.md)、[技术报告初稿](deliverables/技术报告初稿.md) 或 [急刹演示视频](runs/migration_check/brake/replay.mp4)。学习预测误差显著下降，但闭环进度基本相同；首版尚未证明学习方案更有竞争力。

旧实验内的绝对路径保留原始值；在新目录定位其文件时，按 [迁移说明](迁移说明.md) 映射路径，不继续向旧目录新增开发。

## 先运行一个例子

所有命令在项目根目录执行。已有 `.venv` 且依赖完整时，可以直接运行：

```bash
cd '/home/aup/AIC/aic 初版'
env -u PYTHONPATH .venv/bin/python scripts/run_demo.py \
  --scenario brake --method ca_fixed --seed 0 \
  --delay 0.2 --duration 30 --video
```

命令结束后会打印结果和 `run_dir`。打开对应目录里的 `replay.mp4`、`run_plot.png` 和 `summary.json`：视频展示车辆行为，曲线展示速度、加速度及观测年龄，JSON 保存机器可读的指标。

不指定 `--output` 时，每次演示自动创建新的运行目录。需要切换场景时，将 `brake` 改为 `follow` 或 `cutin`。视频标记为 **STRUCTURED-STATE SIMULATION**；画面中的车辆位置是仿真参考状态，控制器接收的仍是经过延迟处理的观测。

## 首次安装

项目声明需要 Python 3.11 或更高版本，当前锁定环境使用 Linux x86_64 与 Python 3.12，复现优先采用这一组合。基础仿真与解析方法不要求 GPU；学习模块采用 CPU 小模型。已有可用环境无需重新安装。

```bash
cd '/home/aup/AIC/aic 初版'
python3.12 -m venv .venv
env -u PYTHONPATH .venv/bin/python -m pip install --upgrade pip
env -u PYTHONPATH .venv/bin/python -m pip install -r requirements.lock
env -u PYTHONPATH .venv/bin/python -m pip install --no-deps -e .
```

要采集学习数据、训练、标定或使用 `ml_*` 方法，再单独安装 PyTorch CPU 版本：

```bash
env -u PYTHONPATH .venv/bin/python -m pip install -r requirements-torch.lock
```

命令统一使用 `env -u PYTHONPATH`，避免外部环境中的同名包路径干扰本项目虚拟环境。基础测试命令如下；请以实际输出判断是否通过：

```bash
env -u PYTHONPATH .venv/bin/python -m pytest -q
```

## 一条命令运行全流程

依赖就绪后，快速检查整条管线：

```bash
env -u PYTHONPATH .venv/bin/python scripts/reproduce.py --output runs/quick_v1
```

默认 quick 使用较少回合、5 轮训练，主要用于查依赖、接口和产物是否连通。完整规模使用：

```bash
env -u PYTHONPATH .venv/bin/python scripts/reproduce.py \
  --output runs/release_v1 --full
```

完整模式计划采集 90 个回合、用独立调参种子运行 270 个回合、主评测 1800 个回合、压力评测 360 个回合。固定安全裕度先在调参集选择，然后应用于两种固定裕度方法；最终评测不参与选择。

每阶段日志在输出目录的 `logs/`，进度与退出状态在 `pipeline.json`，还有数据、模型、调参记录、主评测、压力评测及各自的报告。`pipeline.json` 的 `status: completed` 才表示整条管线结束；还应检查各阶段结果。`source.zip` 和 `source_manifest.json` 保存启动时的代码与依赖信息，运行中修改源码会使冻结检查失败。

`runs/release_v1` 是完整运行的示例输出位置，不能仅凭这个目录名认为已完成。目录已存在时查看既有结果；新实验使用 `release_v2` 等新目录。一键流程会自行采集和训练，不会使用下面示例的 `data/pilot_v1`；已准备好数据和模型时，可以按后面的单步命令继续，避免重复计算。

## 完整操作顺序

顺序是：**跑通场景 → 采集数据 → 训练 → 独立标定 → 四组评测 → 生成报告**。先确认上一阶段成功并留下文件，再进入下一阶段。

下面使用 `data/pilot_v1`、`models/pilot_v1` 举例。采集、训练、标定和评测会拒绝覆盖对应的已有成果；若目录或文件已经存在，就读取和使用它们。需要新实验时使用 `pilot_v2` 等新名称，并同步修改下游路径。不要为了重跑而直接删除旧证据。

### 1. 采集带延迟、噪声的历史观测

```bash
env -u PYTHONPATH .venv/bin/python scripts/collect_data.py \
  --output data/pilot_v1 --episodes-per-scenario 30 \
  --duration 15 --seed-start 1000
```

这会为三类场景各生成 30 个回合，留下 `dataset.npz`、`metadata.json` 和每个回合的原始日志。脚本要求每类至少 20 回合，按完整回合划分训练、验证、标定和测试集；同一条轨迹不会拆到不同集合。输入来自在线观测，未来仿真真值只用于离线训练标签。

成功标志是数据文件、样本计数和回合记录完整；不能仅凭程序开始打印进度就认定完成。这是起步数据规模，不等于足够证明泛化能力。

### 2. 训练轻量残差预测器

```bash
env -u PYTHONPATH .venv/bin/python scripts/train_predictor.py \
  --dataset data/pilot_v1/dataset.npz \
  --output models/pilot_v1 --epochs 80 --seed 0
```

模型学习修正恒加速度预测的偏差，输出 `model.pt`、`metadata.json`、`heldout_errors.json` 与误差明细。模型选择使用验证集；测试集结果用于报告，不能反复看测试成绩后调参。

先看学习预测是否真的改善误差，再看闭环驾驶效果。训练损失下降不等于碰撞减少，预测更准也不保证驾驶更好。

现有 pilot 的离线预测结果可查看 [预测误差简报](runs/prediction_report_pilot_v1/report.md) 和 [误差图](runs/prediction_report_pilot_v1/prediction_errors.png)。这些结果仅描述预测误差，不能代替闭环验证。需要从模型目录生成一份新报告时：

```bash
env -u PYTHONPATH .venv/bin/python scripts/prediction_report.py \
  --model-dir models/pilot_v1 --output runs/prediction_report_pilot_v2
```

### 3. 用独立数据标定误差范围

```bash
env -u PYTHONPATH .venv/bin/python scripts/calibrate.py \
  --dataset data/pilot_v1/dataset.npz \
  --model models/pilot_v1/model.pt \
  --output models/pilot_v1/calibration.json
```

这一步只使用标定集，为解析预测与学习预测分别估计经验误差范围，并保存模型、数据的对应关系。逐坐标的经验 95% 范围不是联合覆盖或无碰撞保证。更换模型或数据后，重新生成匹配的标定文件。

### 4. 跑四组相同条件的闭环对比

| 方法名 | 预测 | 安全裕度 |
|---|---|---|
| `ca_fixed` | 恒加速度解析预测 | 固定 |
| `ca_adaptive` | 恒加速度解析预测 | 自适应 |
| `ml_fixed` | 解析预测＋学习残差 | 固定 |
| `ml_adaptive` | 解析预测＋学习残差 | 自适应 |

```bash
env -u PYTHONPATH .venv/bin/python scripts/evaluate.py \
  --output runs/pilot_eval_v1 \
  --scenarios follow brake cutin \
  --methods ca_fixed ca_adaptive ml_fixed ml_adaptive \
  --fixed-margin 2 \
  --delays 0 0.2 0.4 --delay-modes fixed \
  --seeds 5 --seed-start 50000 --duration 30 \
  --profile development --position-noise 0.1 --velocity-noise 0.1 \
  --model models/pilot_v1/model.pt \
  --calibration models/pilot_v1/calibration.json
```

这是 3 场景 × 3 延迟 × 5 种子 × 4 方法，共 **180 个回合**的开发性评测。种子从 50000 开始，与上述采集数据的种子范围分开。手动示例为两种固定裕度方法统一使用 2 米；正式比较应先在独立调参集选择并冻结参数，一键流程已经包含这一步。脚本保存计划清单、逐回合目录和持续写入的 `episodes.csv`。

`--seeds 5` 表示每个条件用 5 个种子，并非全实验只有 5 回合。需要先查管道时，可用新输出目录、单场景、单延迟和 1 个种子运行；这种小规模检查不能用来证明算法优势。

当前评测脚本不提供自动续跑。若中途失败，保留已有目录、查看报错及日志，再决定如何恢复；不要将部分 CSV 描述为完成了计划中的全部回合。

冻结方法与参数后，可以另建目录，将 `--profile` 设为 `holdout`、`--delay-modes` 设为 `fixed bursty`，并改用未使用过的种子，检查更紧迫场景和延迟突增。`holdout` 是另一组脚本化场景参数，不是道路数据；用它调过参后，就不能继续把同一批数据当作未见测试。

### 5. 生成分组图表和简报

```bash
env -u PYTHONPATH .venv/bin/python scripts/make_report.py \
  --input runs/pilot_eval_v1/episodes.csv \
  --output deliverables/pilot_report_v1
```

输出 `report.md`、`summary.json`、`groups.csv` 和分组 PNG 图。它们来自输入 CSV，不会自动补齐未完成实验，也不会替你作“已达国赛水平”的结论。

每次为一个评测目录生成报告，不要把不同模型、控制参数或调参批次的 CSV 混并后当成同一实验。

若需要学习方法的演示回放：

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_demo.py \
  --scenario cutin --method ml_adaptive --seed 50000 \
  --delay 0.4 --duration 30 --video \
  --model models/pilot_v1/model.pt \
  --calibration models/pilot_v1/calibration.json
```

## 怎样看结果才不容易误判

- **碰撞：**看“碰撞回合数／总回合数”和 Wilson 95% 区间。`0/N` 只说明这批样本无碰撞，不等于绝对安全。
- **进度：**`progress_m` 是完整预算内累计进度，碰撞提前终止后不再增加。`progress_rate = progress_m / budget_s`，不能改用碰撞前的短时间作分母。`duration_s` 保存实际运行时长。
- **效率：**碰撞率相同时还要看进度，防止依靠过度停车获得表面安全。图中的进度误差线是样本标准差。
- **计算开销：**报告中的“平均回合 P95”是每个回合 P95 的平均值，不是全部决策耗时合并后的 P95。原始 `decision_ms` 在轨迹日志中。
- **制动：**`emergency_count` 是处于紧急制动状态的决策次数；`emergency_events` 才是进入该状态的次数，两者不能混用。
- **可比性：**相同场景、种子、噪声、预算和控制设置下比较。图表会将不同场景、profile、延迟模式、噪声和预算分开；发现缺失或不匹配时先检查数据。

若四组都没有碰撞，就如实报告这批场景无法建立碰撞率差异，继续分析进度和代价。若学习模块没有稳定收益，保留解析方案和负面结果。

## 项目文件在哪里

```text
AI开发任务书.md / 开发流程指南.md   开发范围、验收与协作方法
STATUS.md                         实际进度与证据入口
src/aic_drive/                    场景、传感时序、预测、控制、执行、绘图
scripts/                          演示、采集、训练、标定、评测、报告入口
tests/                            时序、接口、控制及学习相关检查
data/pilot_v1/                    数据集、划分说明、采集回合
models/pilot_v1/                  模型、误差记录、标定文件
runs/                            演示与评测的原始证据
deliverables/                     生成的图表与报告
```

单次运行目录保存 `config.json`、`scenario.json`、`provenance.json`、`trajectory.jsonl`、`observations.jsonl` 和成功结束后的 `summary.json`；失败时会尽可能留下 `error.json`。`provenance.json` 包含源码和依赖信息，便于追溯结果。只有加上 `--video` 的演示命令才会自动生成回放与诊断图。

## 以后怎样与 AI 继续开发

让 AI 先读取任务书和 `STATUS.md`，检查代码与记录，再完成下一项未完成工作。要求它实际运行、保留失败证据、更新状态，不重复未受改动影响且已通过的验证。

你每天主要检查：场景有没有跑通、今天改了什么、有什么真实结果、下一步最值得做什么。报名资格、队伍、费用、账号及最终提交由你确认；AI 可准备材料，但不能把仿真结果写成实车能力或已获比赛认可。

遇到报错，保留完整命令、终端输出和相关目录，不需要先自行理解所有代码。最有价值的下一步，是让系统更可复现、比较更公平、结论更有依据。

## 2026年10月25日 模型优化

新残差集成与平滑CA基线已实现，40项测试通过。新模型共同≤2.4秒的位置误差较冻结旧模型降低37.3%，但闭环收益很小且名义95%误差矩形仍欠覆盖。稳定演示继续使用ca_recovery；新学习方法为ml4_recovery，平滑解析为fit_recovery，相同新数据重训旧网络为ml_retrained_recovery。

请先阅读[优化与证据说明](deliverables/20261025/优化与证据说明.txt)和[6页佐证PDF](deliverables/20261025/模型优化与实验佐证_20261025.pdf)。模型及标定位于runs/model_optimization_v4/models；训练及冻结评估入口scripts/optimize_model_v4.py拒绝覆盖已有实验。原始结果、源码快照与独立核查均保留。
