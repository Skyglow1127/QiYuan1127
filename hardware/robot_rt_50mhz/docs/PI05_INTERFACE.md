# π0.5 + LoRA → PL 离线接口（2026-11-04）

新增接口已完成 **服务响应 → 物理动作校验 → 原观测时间调度 → 实际 DDS 间隔规划 → 整数停止尾 → bank 打包 → 二进制回读**。本入口只做离线计算，无模型加载、网络、相机、CAN、MMIO、ARM 或构建命令。现有 ACT、部署、RTL、构建文件未改。

非静态通过样例是明确标注的**模拟 50×7 数据**。找到了真实 π0.5+LoRA 归档响应，保留其原始动作与本地时刻做拒绝验证；本次没有宣称真实模型端到端通过或实机闭环。既有样本不能在当前限值与时效下通过，不据此放宽校验、修改预测或重写来源时间。

## 文件与运行

- `sw/rt_adapter/pi05.py`：已后处理响应的专用契约适配器，不导入 ACT 或任何归一化器。
- `sw/rt_adapter/integer_planner.py`：复用 `C2Tracker` 生成候选 jerk，复用 `player_reference.advance` 传播和认证全部整数状态；求解精确整数停止尾。
- `sw/pi05_to_pl.py`：单块离线编译入口，复用原 `ChunkScheduler`、`DDSConfig`、`build_bank`。
- `sw/verify_pl_bank.py`：独立解析输出 bytes，复核 CRC、配置、DDS、段接缝、连续区间包络、Q16/CAN 整数量化与停止终值。
- `tests/test_pi05_pl.py`：17 项针对性测试，含多组子用例。
- `examples/pi05/`：完整运行配置、模拟输入，以及三份有真实来源 SHA256 的拒绝夹具。
- `docs/pi05_offline_20261104/`：本轮测试记录、真实拒绝原因和生成工件。

使用已有系统 Python/NumPy，无需安装依赖。输出目录必须尚不存在，防止覆盖。

```bash
cd /home/aup/Lerobot/projects/piper/robot_rt
/usr/bin/python3 sw/pi05_to_pl.py \
  --input examples/pi05/simulated.json \
  --config examples/pi05/offline_config.json \
  --output-dir /tmp/pi05-bank-new
/usr/bin/python3 -m unittest discover -s tests -p 'test_pi05_pl.py' -v
```

生成 `bank0.bin`（65536 bytes）、32 位小端字的 `bank0.mem`、`config.bin`、`config_writes.mem`、可重新交给原 `sw/pl_bank.py --input` 的 `bank_input.json`、完整输入/配置、报告和源码 SHA256。配置写列表不含 CMD/ARM。工件可供设计测试台/未来板端加载器读取，**不能直接当成当前板上的运动执行文件**。

## 输入契约和调用方式

已检查指定服务：`predict_action_chunk` 返回后调用其自己的 `postprocessor`，响应正好 `actions[50,7]`；`units=['degree']*6+['metre']`，`representation='absolute joint targets and gripper width'`。检查点为 `type=pi05`、`chunk_size=50`、`n_action_steps=10`、`use_relative_actions=false`。训练使用的本地 `rad_place_merged_v5/meta/info.json` 标明 `fps=20`。

服务本身没有轴名/采样率字段，不能从浮点数猜测。因此调用方必须显式提供 `Pi05Contract`：

```python
from sw.rt_adapter.pi05 import Pi05Contract, AXES
contract = Pi05Contract(
    axis_order=AXES, sample_hz=20, use_relative_actions=False,
    postprocessed=True, model_id='pi05-piper-red-cube-020000')
chunk = contract.adapt(
    response, sequence=sequence,
    observation_at=original_observation_monotonic,
    available_at=response_validated_monotonic,
    request_id=original_request_id, input_sha256=original_input_sha256)
# 交给已有、有状态 ChunkScheduler.accept/pop_due/reference_at。
```

轴序由检查点 `action_feature_names` 确认：`joint_1.deg` … `joint_6.deg`、`gripper.m`。契约拒绝轴序、单位、表示、模型/请求/输入身份不匹配，以及错误形状、bool/string、NaN/Inf。全部50槽检查位置限值，不能因槽未被消费就藏起越界预测。默认整块拒绝，无投影/夹断/二次反归一化。

观测采集端与响应接收端必须使用**同一调用方单调时钟域**。`observation_at` 是该观测最早的原始来源时间；`available_at` 是本地完成响应验证的时刻，映射到通用 `TimedChunk.produced_at`，不是云端模型运行时刻。服务 `server_timing_s` 不参与调度，不拿接收时间替换来源时间。响应适配本身是纯函数；序号重放保护由持久的 `ChunkScheduler` 实例负责。

本 CLI 是一个**初始 bank 编译器**，每次创建新的离线调度场景，不应当作为运行中不断重新初始化 session 的实时循环使用。它拒绝非零 `previous_sequence` 和非静止初始导数；没有偷偷把运动中的 v/a 清零。后续连续 bank 必须从已提交整数状态/序号续接，不能重复本初始入口冒充无缝切换。

## 本轮参数与时间轴

| 参数 | 本轮值/来源 |
|---|---|
| 模型采样率 | 20 Hz，独立于执行率与 FPGA 时钟 |
| 消费窗口 | 显式 10 槽；没有把完整 50 槽全部执行 |
| `f_num/f_den` | 275/2 = 137.5 Hz，离线测试率，非实测机械臂频率 |
| `clock_hz` | 49,999,500，来自50MHz构建 `compiled_clock_hz`，非实测振荡器频率 |
| 来源寿命/槽迟到 | 0.4 s / 0.01 s |
| 规划响应时间/预留 | 0.4 s / 0.001 s，预留只属于离线时间模型，没有实时耗时保证 |
| 停止尾预算/超时/lease | 最多160段 / 1.5 s / 0.75 s |
| 关节位置下界 | [-60,0,-120,-45,-60,-80] deg |
| 关节位置上界 | [30,120,0,45,65,60] deg |
| 夹爪位置 | [0,0.1] m |
| v / a / j / DQ（六关节） | 45 deg/s / 160 deg/s² / 2000 deg/s³ / 6 deg |
| v / a / j / DQ（夹爪） | 0.05 m/s / 0.5 m/s² / 5 m/s³ / 0.008 m |
| 夹爪编码 | 显式 width mode/status=1、set_zero=0、1000 mN；仅编码检查 |

位置、速度与 DQ 值沿用已有 `pi05_execution_defaults.json`；a/j、响应时间为本轮显式数值场景，**没有机器人能力或碰撞安全验证**。所有量来自配置，不写回已有部署。

第 i 槽保持 `observation_at+i/20`；丢弃过期前缀，不把10槽解释成“收到后再执行10槽”。模型消费窗口为0.5秒，但来源寿命0.4秒先截止。模拟输入原观测100.0、本地可用100.02，因此槽0丢弃，最终正常参考用槽1～7。插值读取原时间轴的邻近目标，不外推窗口。

离线 PL 映射以显式 `offline_affine_not_hardware_synchronized` 标记，保留本地时间并提供模拟 cycle anchor/DDS origin。观测 cycle 向下取整，可用/校验 cycle 向上取整，截止时间保守取整。normal输出连同112拍描述符延迟必须严格早于原截止时刻。首段起点不得早于可用/规划时刻；实际积分使用 DDS 相邻 tick 的整数差，不能使用名义 `1/f` 代替。

该映射**不是已完成的 PS/PL 校时**。编译器没有把实际 CPU 规划耗时反馈到真实当前 cycle，因此输出仅适用于离线验证。接入实时硬件前必须测量并预留规划/上传/预检耗时，在真实提交时重新验证期限和时钟误差；不能原样重放历史时戳或模拟 cycle。

## 整数规划与停止尾

每个候选段以真实整数终点的 q/v/a 重建浮点规划状态，再将 jerk 编码为 Q32.32，调用原整数参考认证完整 Bernstein 包络、位置/DQ、v/a/j、溢出及 Q16/CAN 量化。下一段保存的 q/v/a 等于前一整数终值逐 bit 相等。浮点规划只是候选生成器，不能绕过整数门。

旧代码只有打包时的“末端必须精确 v=a=0”检查，没有求解非静态整数停止尾。本次新增有界阻尼制动，并在每一步尝试两段精确停止解。给定实际 `h1=d1/C`、`h2=d2/C`，搜索理想中间加速度附近的整数格点，为两段求 jerk；分别执行现有 RNE 积分，只有最终 **整数 v=a=0** 且全部包络/量化检查通过才接纳。失败则保留原状态继续有界制动，预算/限值不允许时拒绝。没有阈值 snap、直接改终值、清状态或放宽原校验。

搜索是有界、保守的，不保证每个可制动状态都能找到解；极高测试率超出搜索预算、bank容量不足、位置边界无可行轨迹等均明确失败。停止尾可延伸到正常来源窗口之后，但不再取模型目标，且整bank必须满足配置的 stop timeout 与保守 lease 检查。单bank不能容纳任意50槽高频轨迹，容量不足时拒绝而非静默截短。

整数接缝的逐 bit 连续不等于物理 CAN 输出严格 C2；每段无限精度端点与整数端点仍有不超过0.5 Q32 LSB的舍入误差，沿用原规格表述。

## 验证结果与真实数据限制

本轮94项测试通过：新增17、原通用调度/轨迹21、整数/tick参考32、bank9、ACT15。命令与结果见 `pi05_offline_20261104/verification.json`，新增测试完整输出在 `pi05_tests.log`。

```bash
/usr/bin/python3 -m unittest discover -s tests -p 'test_chunk_adapter.py' -v
/usr/bin/python3 -m unittest discover -s tests -p 'test_*reference.py' -v
/usr/bin/python3 -m unittest discover -s tests -p 'test_pl_bank.py' -v
/home/aup/Lerobot/envs/piper-cpu/bin/python -m unittest discover -s tests -p 'test_act*.py' -v
```

模拟演示：47个正常段+19个停止段；实际间隔363632/363633 cycles；最终七轴整数 v/a全零；bank SHA256：`4cb5177313b2ddf00fcf55541046fa5331b286424b544aff2772eb359c923d47`。从打包 bytes 再逐段积分，header/payload/config/seed CRC与全部接缝通过；重新计算CRC也无法使坏接缝、保留位、错误配置或非零停止终值通过。另验证20、160、200 Hz显式执行率。

模拟跟踪误差最大值依次约 `[0.315894,0.315894,0.252715,0.157947,0.189536,0.221126] deg` 与 `0.000631787 m`；这是参考轨迹滞后，不是机械臂实测误差。

三份真实归档拒绝证据（完整文件路径/源SHA/原时间保存在夹具与 `real_rejections.json`）：

| 输入 | 结果 |
|---|---|
| live_preview/request_000 | 来源年龄0.444996s超过0.4s，拒绝 |
| live_preview/request_004 | 完整预测块J5最大约66.6553°超过原65°上界，拒绝；不能只验消费前缀掩盖后段 |
| previews/20261030_234137_792200250/request_000 | 当时有效首槽相对原输入状态超过6° DQ，拒绝 |

保存的 recorded_smoke样本标记capture time unknown，不能为它编造来源时刻称作真实时效验证。另检查了本地归档报告中的真实响应，未找到能在上述限值/时效与原始起点下完整通过的样本。因此本轮**通过的非静态 bank 是模拟数据，真实模型输出只完成契约/拒绝验证**。下一步需要取得仍在原来源寿命内、整块合法、起点连续的真实响应；不能以放宽限值或洗时间戳替代。

## 后续上板条件

当前50MHz构建 `status.json` 仍 `pass=false`、stage=implement；日志明确 `Synthesis incomplete: synth_1 Out-of-date`。本轮仅读取，未重启/干扰构建。没有布线完成、bitstream签核或上板通过证据。

此接口可作为后续 PL **设计数据/离线参考**使用，但不能直接接管实机。还需要：完成构建与布局布线/时序签核；以实际 NOMINAL_CLOCK_HZ、DDS origin、session/epoch和已校准时钟误差生成新bank；接入可信反馈seed及实际规划/上传/预检期限；实现inactive bank所有权、READY/提交与后继bank精确状态衔接；先做DESC_ONLY描述符比对，再验证物理传输、反馈保护与实际执行率。原 `board_desc_smoke.py`/`desc_smoke.py` 仅支持静止数据，不能拿这份非静态bank绕过其静止检查。

本次没有发送运动指令。后续若涉及D1实物抓取，仍须先展示具体目标、路径和关键参数并取得用户明确批准；本工件、历史使能或标定许可均不是抓取授权。
