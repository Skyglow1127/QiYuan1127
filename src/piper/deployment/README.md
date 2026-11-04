> 来源文档快照：下文的“当前”、路径和命令对应原记录时间；本次选定版本与验证边界见 [提交状态](../../../docs/RESULTS.md)。

# ACT 执行适配（默认只读，KR260 仅 RX 接通）

**当前抓取回退与放下后回撤碰块修正（v8）：** 用户再次澄清夹爪偏右，要求撤销上一轮抓取改动；放置现象确认为“方块已经放下，回撤时又碰到它”。已实际执行上一轮回退，再保留用户此前要求的duration0无限；当前`policy_approach_refresh`已移除，抓取恢复原50步/alpha0.5/160及原ACT，没有固定左右角度补偿。放置坐标仍由ACT决定，关节窗无法证明接桌。

最新031728原ACT边开爪边移动，夹爪实际跟随滞后；追加20槽还包含回撤和再次闭爪（首段原开口70.6→28.9mm）。新增显式`policy_release_opening_prefix=True`，只延续已解码物理预测中>=55mm、未比最后成功原开口小超过3mm且仍大于实际开口的连续前缀；原归一化队列与来源时间保持。旧profile缺字段默认False，保留原20槽行为，支持准确回退。

新增显式`policy_release_settle`：先有抓取侧J1<0闭爪提议成功ACK，随后在放置窗、原ACT要求>=55mm且比actual多6mm、actual>=35mm、过去150ms观测样本各轴跨度<=0.75度时，每次松爪短暂保留上一ACK六轴参考，夹爪继续本tick ACT原提议。actual相对起点增开5mm或最多350ms后放行；撤回开爪、离窗、原关节提议离锚>5度、旧/重复/未来/断档采样等取消。只在成功ACK后提交history，失败/disarm/rearm清空；下次抓取侧闭合才再武装。下游cap/EMA/限速/加速度/双Guard继续处理参考，不能把参考保持当成实测TCP完全静止或释放成功。没有固定60mm门槛/2秒超时停止/强制爪宽/盲目下压。

369项离线部署回归、56项定向检查通过，含10项真实SDK/双端Guard/9键图像集成。历史8210帧固定反馈分类有13次缓冲触发，11次相对开口放行、2次模型撤回开爪取消；4段续步80原槽中候选前缀共43槽（首段20→6）。这些不模拟新物理反馈，回撤无碰撞及成功率仍需现场验收。5种回退入口临时副本校验通过；原模型7文件SHA及原硬Guard/时序保持。证据`logs/act_stable_placement_v8/verification.json`。本轮没有驱动实物。

启动仍为`bash /home/aup/Lerobot/projects/piper/run_act_piper_repair.sh --run --duration 0`。撤掉本轮放置协调：`bash /home/aup/Lerobot/projects/piper/restore_act_piper_before_stable_placement.sh`，恢复`baselines/placement_parent_50_no_refresh_v8/`的50/.5/160/无限时长，保留相机及启动字段修复，不恢复已被用户拒绝的新图重预测。`before_stable_placement_v8_r29/`是被撤销版本的诊断快照，不是推荐回退源。下文是历史记录。


**当前下落偏左/持续运行修正（v8）：** 用户更正为下落时夹爪位于红块左侧。最新两次正常运行均到60秒结束，闭爪所用视觉源年龄约0.78～2.70秒，首次约1.64/2.03秒；原每槽时效仍合格，不能混同为Guard过期。闭爪时关节跟随原策略较近，加速度整形在首次闭爪六轴改动均0；没有对应抓取图像或桌面/相机到base标定，不能直接归因加速度或施加固定左右角度偏置。

当前原模型、普通50步、alpha0.5、160软整形、1N和原保护保持；新增`policy_approach_refresh`可选一次接近新图重预测：实际六轴在已有抓取窗、夹爪>=35mm、尚未开始闭合、已消费>=10槽且该预测视觉源>0.8秒时，先验证旧待执行槽仍有效、新观测来源更新且新鲜，再丢弃旧缓存，使用同tick真实双图batch重新预测。每次接近只刷新一次，新预测正常执行50步；闭爪、携带、放置和回程不反复刷新，不reset机器人/模型或加速度历史。只有窗外J2<=90且已ACK开口>=55mm的新接近事件才可再武装。普通50步不变，局部旧块会提前截断，不能将其描述为全程每50步才重预测。

修复入口默认`default_duration_s=0`，沿用LeRobot0.6.1官方无限运行语义；命令`bash /home/aup/Lerobot/projects/piper/run_act_piper_repair.sh --run --duration 0`，Ctrl+C走原shutdown/disarm/teardown，保护故障仍可停止。正值覆盖仍<=60秒，旧profile默认30秒不变；每次生成显式时长review，旧配置不默认升级无限。原0.75秒lease、0.12秒TX TTL、反馈/图像/每槽时效和所有硬限位均保持。

327项离线回归通过，包括官方0时长跨183秒后shutdown、65秒连续命令后原时效仍拒绝、绝对122秒位置短lease仍过期、官方ACT队列一次刷新后正常50步以及不重标旧动作。两段固定历史状态下，各在首闭爪前识别一次刷新；不能据此推断新策略的物理反馈。32项临时副本回退验证通过，原模型7文件SHA通过；证据`logs/act_right_bias_v8/verification.json`。本轮未驱动实物，左偏根因与改善仍待现场验收，新增同步重预测仍有CPU停顿，不承诺消除振动。

修改前12文件及SHA在`baselines/before_right_bias_unlimited_v8/`。回退本轮：`bash /home/aup/Lerobot/projects/piper/restore_act_piper_before_left_bias_fix.sh`，恢复50/.5/160/默认30秒且无新刷新，保留相机和启动字段修复。旧30及其他50回退继续保留。下文是历史记录。


**启动首条动作失败修复（v8）：** 最近三次`021918_729415`、`021943_633948`、`022013_869218`均完成接管和首块50步推理，却在首条发送前报`Expected exactly six joint_*.deg fields and gripper.m`，0条accepted。上一轮新接近模块误收含global/wrist两图的9键完整观测，而helper严格要求7标量；仅在robot.py调用处显式选取ACTION_KEYS，保留完整图像观测和Guard/helper严格校验，不改模型、profile或任何参数。

测试夹具现包含两张真实形状uint8 RGB数组，并用三次实际首条raw/反馈/来源时序走get_observation→send_action→SDK编码→双Guard；旧源码三例均重现，修后全部通过。297项部署回归、入口及7模型文件完整性、三条回退--check通过。证据`logs/act_startup_observation_fix_v8/verification.json`；修改前6文件保存在该目录backup.json所列快照（含已知错误，仅供诊断）。50步、alpha0.5、160目标加速度整形、原模型、1N及全部硬保护不变。未发送实物运动指令，现场重启效果仍需用户运行确认。下文是先前记录。


**当前接近提前闭爪与关节目标尖峰修正（v8）：** 用户澄清是在靠近红块前提前闭合。最新两轮50步日志seq151的实际J2约82.2/85.8度，而正常闭爪段约102.2度；上轮抓取续步期间，实际夹爪到35mm需约1.46秒，机械臂仍明显搬运，不能视为原地等夹稳。因此撤掉当前profile的抓取续步，普通50步、原模型、alpha0.5和原放置条件续20步保持。

修复入口新增显式`policy_approach_gripper`：已观测到开爪或已成功发送明确开爪意图后，在六轴实际反馈尚未进入记录的抓取姿态窗时，任何比上一成功夹爪目标小的闭合提议只保持该旧夹爪目标（包括每帧微小闭合的累积）；六关节不冻结、不强制开爪。进入窗内开始闭合后，允许搬运、放置及回程；在抓取侧出现新一轮大开爪提议时重置分类。姿态窗只是本任务阶段判断，不是图像对准、距离或接触证明，不适合作为任意摆放位置的泛化抓取保证。

新增六关节目标加速度软整形`policy_joint_acceleration_deg_s2=160`，在原反馈cap、EMA和限速后与硬区间/CAN网格求交；夹爪不参加加速度整形。只以成功ACK后的历史计算，发送失败/重新arm清空。原硬区间冲突时该轴保留原硬投影并记录软约束降级，不增加等待或修改deadline。它约束的是目标平均速度变化，不是硬件实际加速度保证。

296项部署回归通过，含47项新检查；两轮672条固定历史反馈经真实SDK编码及双Guard回放通过，无软降级或硬拒绝。160候选把目标加速度峰值约461/588降至160，目标二差RMS仅降6.4%/7.6%，主要削减尖峰。历史反馈固定、ACK代理时间及预测缓存不随新轨迹变化，不能将其当作实机消抖或一次抓取成功率；约200ms同步推理换块停顿仍存在。证据`logs/act_approach_jitter_v8/`。原模型、1N、2度/3mm cap、0.9限速比例、全部源时效/反馈/双Guard及相机不改。本轮未连接CAN或发送运动指令；下次启动修复入口生效。

修改前12文件和SHA256保存在`baselines/before_approach_jitter_v8_r25/`，回退该版用`bash /home/aup/Lerobot/projects/piper/restore_act_piper_before_approach_jitter.sh`。更早无抓取续步50步与原30步回退也兼容当前版本。下文均为历史记录。


**当前抓取闭爪换块衔接（v8）：** 用户确认已对准但未夹住；30/50步日志均出现实际夹爪仍约51～56mm时，新块将闭爪提议反向改为62～68mm。普通50步/alpha0.5保持，仅新增抓取窗口内、曾开爪且原模型已要求闭爪时的连续闭合prefix续用，最多20个原预测槽，同一块与放置续步不叠加，总计最多70步。反归一化副本只用于分类，原normalized队列及source/deadline保留；原关节轨迹继续，不是原地等夹稳。原cap2度/3mm、1N及全部Guard不改，无强制爪或固定开口/2秒停止。249项离线回归及临时副本回退验证通过，证据`logs/act_grasp_continuation_v8/verification.json`。原模型、相机及旧profile保持；现场成功率待试跑。回退本轮修复：`bash /home/aup/Lerobot/projects/piper/restore_act_piper_before_grasp_fix.sh`，恢复原50步配置；原30步回退也保留。下文是历史版本记录。

**当前再次使用50步（v8）：** 用户要求从30改50，当前修复入口配置及匹配源码已恢复到已保存的50步版本；其余参数保持。chunk100、alpha0.5、条件放置续20步（最多70步）、原模型及全部保护不变。原30步回退仍可用，24项定向离线检查及入口/模型校验通过，证据`logs/act_action_steps50_v8_r21/verification.json`；下次启动生效，本轮未启动或停止实物。以下保留旧版本记录。

**当前已恢复改70步之前的30步版本（v7晚）：** 按用户要求实际回退，当前普通30步、alpha0.5、原20000步模型，条件松爪续20步（最多50步）；保留相机修复及原保护，无固定开口确认停止。配置和匹配入口与`baselines/before_70steps_v7_r17/`逐文件哈希一致。24项定向离线检查和入口/模型校验通过，证据`logs/act_restore_before70_v7_r20/verification.json`。下次启动生效，未启动/停止实物。以下70/50/100段落是历史试跑记录。

**当前100步试跑（v7晚）：** 用户要求50步改100步，当前修复入口配置/校验/提示已同步。完整使用同次100步预测后重新预测，不再有可追加的放置缓存；名义20Hz间隔约5秒，另加推理耗时。原模型、alpha0.5、速度、夹爪及原保护保持。50步版本保存于`baselines/before_100steps_v7_r19/`；原30步回退命令仍可用并兼容100/50/70，备份和原清单均保留。38项离线检查及临时副本回退验证通过，证据`logs/act_action_steps100_v7/verification.json`；未发实物运动，效果待现场试跑。下文保留先前版本记录。

**当前50步试跑（v7晚）：** 用户随后要求70步改50步，当前修复入口已同步修改，其他配置保持。普通50步，原放置续步条件满足时最多70步；模型预测100步，alpha0.5、速度、夹爪、相机和全部原保护不变。70步版本保存于`baselines/before_50steps_v7_r18/`。30步回退命令仍为`bash /home/aup/Lerobot/projects/piper/restore_act_piper_30steps.sh`，兼容50/70版本，保留原30步备份及原70步清单。24项定向离线检查和临时副本回退验证通过，证据`logs/act_action_steps50_v7/verification.json`；未发送实物运动，效果待现场试跑。

**当前70步试跑（v7晚）：** 用户要求把当前连续执行30步改70步，其余参数保持。`run_act_piper_repair.sh`现在普通阶段70步，原放置条件满足时仍可续用同次预测后20步，最多90步；模型预测100步、原20000步权重、alpha0.5、速度、夹爪及原时效/双端Guard保持。普通阶段20Hz名义间隔约3.5秒，另加推理耗时。修改前30步/alpha0.5、已移除开口确认停止的版本保存于`baselines/before_70steps_v7_r17/`；一键恢复：`bash /home/aup/Lerobot/projects/piper/restore_act_piper_30steps.sh`，加`--check`可只验证。恢复不启动/停止机械臂，下次启动生效，且保留相机修复；以前的30步/alpha0.7备份继续保留。24项定向检查及临时副本回退检查通过，证据`logs/act_action_steps70_v7/verification.json`。未发送实物运动，效果待现场试跑。下文记录先前版本历史。

**当前松爪2秒停止修复（v7晚）：** 用户截图与`v7_r14`/`v7_r13`确认，异常由此前新增的全关节冻结＋60mm/150ms/2s确认逻辑触发。前一轮实际开口停在46.7mm，虽ACT提议达到73mm，最终目标受3mm反馈cap限在49.7mm；另一轮达到58.3mm后被新块闭合。固定60mm并不能证明成功放置，冻结关节又阻止原ACT协调轨迹，因此移除`gate_release_motion`及其2秒应用层停止，而不是放宽原硬Guard。普通30步、alpha0.5、原模型、速度、夹爪1N与原限位/反馈/限速/源时效继续保持；松爪条件满足时仍最多续用同次预测后20步，60mm只作缓存续步的分类上界。无强制开爪/盲降，关节按原ACT输出经原保护连续执行。225项离线检查及两段历史80帧真实编码器/双端Guard回放通过，实机效果未验收。用户特别指定的“上一版”是消抖前30步/alpha0.7，完整相关配置/源码和哈希保存在`baselines/previous_30steps_alpha07_v7_r15`；不是alpha0.5失败快照。证据见`logs/act_release_timeout_fix_v7/verification.json`。

**当前消抖参数调整（v7晚）：** 最新两次30步运行中，固定历史反馈及已记录的反馈cap后目标，比较α0.7/0.6/0.5。将修复入口`policy_joint_smoothing_alpha`从0.7降为0.5，仅修改这一项运行参数：1106帧重放均无约束失败或新增等待，目标二阶差分RMS约降34%、>0.05°相邻反向修正次数约降45%，最大目标步长由2.30°降为1.44°（另一轮2.23°降为1.37°）。20Hz下滤波名义斜坡延迟增加约29ms；这是固定反馈下的命令指标，不能当实机振动或成功率验收。30步、模型、速度、夹爪及放置等待保持。约194ms的换块间隔依然存在；固定关节目标时J5实际反馈仍约0.5°范围波动，底层控制/负载/接触/反馈噪声因素未定因。备份在`baselines/contact_release_alpha07_v7/`；21项定向回归及入口检查通过，证据`logs/act_jitter_tuning_v7/verification.json`。下次启动修复入口生效，本次未发实物运动。

**放置松爪与回撤衔接修正（v7）：** 用户确认会出现不松爪，以及松爪后回撤碰到/带走方块。最近30步失败日志出现4次松爪提议，均在slot29结束，下一块立即变回闭爪，实际仅张到45.9～50.9mm。独立修复入口普通阶段仍30步；在记录的放置关节窗口内，模型已要求开口至少55mm且实际未达60mm时，允许沿用同次100步预测的后续最多20步（总计最多50步），保留原观测时间及每步deadline。松爪开始后暂存上一成功关节目标，实际开口达到60mm并持续150ms后才放行后续关节动作；超过2秒仍未确认则Guard拒绝并停止。夹爪目标仍来自ACT，原模型、0.7平滑、速度、1N、反馈cap和双端Guard保持。关节窗口不代表桌面接触确认，未增加强制开爪或下降动作。变更前30步版本见`baselines/contact_30_before_placement_v7/`；224项离线检查通过，证据`logs/act_placement_release_fix_v7/verification.json`。实机成功率待用户试跑。

**当前抓放策略修复入口（v7）：** 用户反馈第二轮模型重复抓取、放置后策略卡住。最新日志仍持续accepted，末段夹爪原提议约18～19mm、反馈44.2mm、受3mm cap后的目标41.2mm，策略一直在闭爪；用户手动终止。新增`bash /home/aup/Lerobot/projects/piper/run_act_piper_repair.sh --run --duration 60`，独立配置`act_launch_contact_repair.json`回用已完成整段任务的原ACT，默认连续执行30步后重新预测，并以0.7新目标权重轻度平滑六关节；夹爪不滤波。滤波在反馈cap之后、最终限速/网格求交和双端Guard之前，用上一成功commit目标作锚点，不改变原来源时间，失败不推进滤波状态。原两份模型、两份旧配置都保留，原配置缺字段时保持旧行为；局部适配源码增加显式可选分支。218项离线测试、真实SDK字节/双端Guard的合成跟随和CPU加载预热通过，实机一次抓取及放置回程尚未验收。原10步方案离线证据见`logs/act_grasp_release_fix_v7/verification.json`。用户反馈10步效果更差后，按要求只将修复入口消费步数改为30，其余参数不变；30步实机效果待试跑，变更验证见`logs/act_action_steps30_v7/verification.json`。

**第二轮ACT独立调用（v7）：** 用户要求调用新训练模型后，新增`bash /home/aup/Lerobot/projects/piper/run_act_piper_round2.sh --run --duration 60`入口，使用`act_launch_round2.json`。新权重已完成追加10000步并下载到`training/act_round2_v6/result/pretrained_model/`，SHA256为`2062da502e72ef4351a079e7e542d3390f1d21751c36a3a50465a8a77eb7243c`。相对成功基线只改模型路径及哈希，控制参数保持20步、已使能10%/冷启动5%、夹爪1N及原双端保护；原59个源码/配置文件、原入口和旧权重均保留。模型7文件完整性、234张量有限、CPU严格加载及3次预热通过，17项入口/启动器测试通过；本轮未调用实物运动。证据见`logs/act_round2_invocation_v7/`。同一新入口用`--check`仅做硬件就绪检查。

**用户确认成功的配置已保留（v6）：** 用户确认当前配置能完成红块抓取、放到左侧黑框及回程，但抓取/放置抖动仍大。完整配置、59个源码/配置文件和启动入口、逐文件SHA256及固定模型校验已保存在[任务成功基线](baselines/task_success_v6/README.md)。关键参数为20步、已使能10%/冷启动5%、8线程、六关节2°余量、夹爪1～99mm/1N；当前默认未修改。后续优化保留该基线用于回退，不把一次任务成功解释为可靠性或消抖验收。

**抖动分析与待试方案：** 最新`v6_r11`的825个已接受目标中，普通目标间隔中位数50.52ms，新块间隔187.37ms；新块原始关节最大轴变化中位数6.10°，最终单轴最大变化3.74°。这些是目标流数据，支持同步推理停顿和块间衔接跳变是贡献因素，不能排除接触或机械因素。固定历史反馈进行受原Guard约束的离线候选比较，六关节在反馈cap后、最终限速/网格求交前，取70%新目标和30%上一已commit目标，超过0.05°的指令反向变化从1078次降到573次（约47%），825帧无新增等待/拒绝。夹爪不参与滤波，20步/10%保留；固定50ms采样下滤波的名义斜坡滞后约21ms。这只是待试候选，未写入当前执行逻辑或默认配置，也未发实物运动指令。分析及适用限制见`logs/act_jitter_analysis_v6/analysis.json`。之后可评估后台提前推理以消除停顿；本机每次推理约180ms，直接启用每帧ACT时间集成不能维持20Hz。

**当前放置后回程修复：** 用户确认物块已放到左侧目标；`v6_r10`接受553条目标，第554条因原始夹爪预测`-0.0006396882236003876m`（−0.6397mm）被拒绝。当次真实夹爪反馈40.9mm，未越界；19.6Hz警告不是这次停止原因。冻结示范统计中的夹爪action最低−5.2mm，因此负闭合提议不是仅靠微小数值容差能覆盖的情况。

新默认显式配置`policy_gripper_target_margin_m=0.001`：有限夹爪提议映射到现有0～100mm工作范围内侧的1～99mm目标，零位附近和负提议对应1mm，原始数据、零点、1N力度及真实反馈0～100mm检查均保留。最终目标继续求反馈相对3mm、目标限速和CAN网格交集；不直接跳到1mm。记录中第554条离线回放得到37.901mm，SDK编码37901µm，主机和独立接收端Guard通过，原来源/反馈截止时刻不延长。合成连续跟随可逐步收敛到1mm；这不证明实机超调或回程成功。NaN/Inf、真实负宽度/过宽反馈、过期和无可行交集仍拒绝。字段只可与带原ACT时序、cap和限速的统一关节投影一起使用，旧配置缺字段时仍严格拒绝夹爪越界提议。完整208项离线测试通过，本轮未发硬件运动指令，下一次原入口生成新配置生效。证据见`logs/act_return_gripper_projection_v6/verification.json`。

**当前放置悬停修复：** `v6_r08`运行接受1039条目标，60秒正常结束，但放置阶段J1、J2持续被此前设置的28°、118°策略上界截断。最后10秒原预测中位数约30.372°、122.813°，真实反馈中位数约28.057°、117.972°。最后一帧仅把这两轴恢复到原预测，用SDK标称正运动学估算法兰下降约45.4mm，与用户看到的悬停高度吻合；这是相对法兰估算，未使用桌面/TCP标定，不能作为已完成放置或无碰撞的证明。

当前软件工作范围将J1上界从30°改为40°、J2从120°改为135°，主机与板端同步；控制器机械限位未修改。两侧仍留2°策略余量，六关节策略目标范围（deg）依次为`[-58,38]、[2,133]、[-118,-2]、[-43,43]、[-58,68]、[-78,58]`，覆盖示范J1/J2动作最大值34.823°/128.253°。启动器先核对六轴软件工作范围均在已查询的控制器限位内，超出则拒绝生成配置；这些限位不证明桌面路径无碰撞。20步、健康已使能10%、冷启动5%、反馈相对cap、目标限速、源时效及双端Guard保持。离线记录目标回放已能越过旧J1/J2上界；随后用户确认已完成放置，本次处理回程中止。证据见`logs/act_placement_workspace_v6/verification.json`。

`policy_joint_target_margin_deg=2.0`继续将六关节有限策略提议投影到各自软件工作范围内部，并在反馈cap、速率及CAN网格求交时约束最终目标。原始观测不投影；真实反馈超出当前软件工作范围、NaN/Inf、过期或无可行目标交集继续拒绝。旧配置未显式启用上方夹爪余量时，夹爪预测越界仍拒绝。该配置仅作用带原始ACT时序的输出，要求反馈cap和目标限速启用，不能与旧J5专用字段混用。逐帧记录全部投影前后值、目标范围与按轴累计次数；控制台按秒摘要，避免逐帧多轴刷屏。旧run缺新字段时沿用原有行为，新的顶层启动会生成新配置；重用旧execute_config不会获得此次范围调整。余量不代表已证明所有物理超调小于余量。


入口在下一次运行时重新核对新鲜反馈、相机序列号及机器/USB-CAN物理身份，并生成新一轮配置；仅允许物理身份一致时更新易变的 ifindex。`--run` 必要时通过终端sudo准备正常1Mbps CAN、恢复原相机服务，再上传独立适配文件并使用官方rollout。不会保存密码、安装依赖、自动reset/标零或沿用旧姿态。重新上电导致夹爪失能时，受限预检按当前宽度、1N使能夹爪，并检查新鲜反馈偏移≤2mm；只允许经核对的payload，故障时尝试失能，未做实机动作验证。

此目录包含独立 stdlib KR260 CAN RPC 桥、AIPC Backend 和 LeRobot Robot 适配，默认 **`execute=false`、unarmed、只读**。已复制到 KR260 独立目录并完成一次仅 RX 实机接通：7 维 `.pos` 别名、双相机 640×480、身份和源时钟均通过，样本年龄约 18.7ms；`execute=false`、`armed=false`、未发 CAN 命令。该次检查未切模式、使能、标零、reset 或实机执行策略，接口为 UP + LISTEN-ONLY + 1Mbps。原有 `official_adapter` 与只读 SSH 桥保持不变；官方 LeRobot 0.6.1 源码与数据未改动。

证据：[executor_rx_check.json](/home/aup/Lerobot/outputs/preview_act_v5/live_plan/executor_rx_check.json)。接通后补充的 angle/width 标签修正已通过离线测试并同步，KR260 `python3 -S --help` 通过，未增加实机 TX。当前源码还增加了“正常 CAN 模式下仍允许只读”的兼容更新；部署时应核对 bridge SHA256 `32fc6e9af3285cc89f3b9f3b187ee573991058b29cdbdaed9991c0027893a502`，guard SHA256 `c4ea080e542be9a0c233f369957c02031634387c25224b2be4f49d9ea25509ec`。

### 后续实机核查进度（v5）

用户已通过原主从随动把从臂摆回示范起点，随后明确确认主臂已断电、从臂仍保持姿态。KR260 `can0` 已设置为正常 UP / 1Mbps，为从臂提供 CAN ACK；这不改变机械臂控制模式。新部署桥仍 `execute=false`、unarmed，只读接通正常，六轴和夹爪反馈健康，见 [leader_off_rx_check.json](/home/aup/Lerobot/outputs/preview_act_v5/live_plan_after_manual_start/leader_off_rx_check.json)。原采集适配仍要求 LISTEN-ONLY，此时不能直接开始录制。

单独向从臂发送过一次非运动固件查询 `0x4AF`、DLC=1、payload=`01`，得到硬件 `H-V1.2-1`、固件 **`S-V1.8-2`**，见 [firmware_query.json](/home/aup/Lerobot/outputs/preview_act_v5/live_plan_after_manual_start/firmware_query.json)。该查询阶段没有发送 reset 或运动指令；之后单独获批执行的 reset 见下文。

该固件低于 `S-V1.8-5`；[官方说明](</home/aup/piper SDK/pyAgxArm/docs/piper/piper_api.md:4522>)指出从 Follower/MIT 切位置速度控制需要先 reset，reset 会失电。用户随后明确确认“已稳固支撑且空载，允许仅复位并检查失能”。该批准仅覆盖本次 reset 和失能检查；不覆盖重新使能、模式切换或策略运动。

用户调整起点后的纯被动 ACT [预测报告](/home/aup/Lerobot/outputs/preview_act_v5/live_plan_after_manual_start/report.html)仍未下发目标：首步 J6 差约 0.015°，J1 差约 −5.267°，夹爪差约 +5.974mm。随后断主臂时又取得新的反馈；这些旧预测不能作为之后的实际目标。当前仍没有完成实机抓取验证。

复位阶段已准备 [reset_piper_once.py](/home/aup/Lerobot/projects/piper/deployment/reset_piper_once.py)，并复制到 KR260 同名独立目录。默认 `--check-only` 不开 CAN，板端 dry run 与 SHA 核对通过（`4eaf58ae6aaccd2aed5db9c5d7d436b90f9e0c7e4d76580993134059bca0ca25`）；6 项离线模拟及独立源码审查通过。唯一可执行报文为一次 `0x150 / 02 00 00 00 00 00 00 00`，其后只核验 reset 后新鲜六轴失能反馈和实际姿态，失败不重试，不串联模式、使能或模型动作。复位前另一次被动检查确认六轴均有健康使能反馈，见 [motor_enable_before_reset.json](/home/aup/Lerobot/outputs/preview_act_v5/live_plan_after_manual_start/motor_enable_before_reset.json)。

收到用户明确批准后已执行一次 reset（`send_attempts=1`、`automatic_retry=false`），[reset_once_result.json](/home/aup/Lerobot/outputs/preview_act_v5/live_plan_after_manual_start/reset_once_result.json)确认六轴 reset 后新鲜使能位全部为 0，六轴状态字均为 0；反馈年龄约 0.1～20.2ms。没有继续发送模式、enable、关节/夹爪目标或标零。**当前停在六轴失能状态，主臂保持断电，用户须维持可靠承托。** reset 后一次新鲜姿态为 `[3.053,-2.879,-0.278,7.666,11.232,-10.737,0.0097]`（deg/m）；与 reset 前相比姿态发生变化，不能使用旧预测或旧位置作为重新上电目标。后续低速 J / 使能步骤尚未执行或获批。

### 用户新请求 ACT 试跑后的进度

用户随后明确要求“直接调用 ACT 看抓取效果”，并确认从臂已手动摆稳、手退出、现场可随时断电。已只读查询实际控制器限位和夹爪配置：[controller_limits.json](/home/aup/Lerobot/outputs/act_local_trial_v5/controller_limits.json)。J2/J3 在失能姿态仍约 −2.075°/+2.507°；已明确展示低速恢复到 +1°/−1°、其余轴保持，随后最长 30s、5% J/POS_VEL、1N 夹爪的 ACT 试跑计划。该阶段由新请求授权，不沿用“仅 reset”的旧批准。

已准备独立 [preflight_j_hold.py](/home/aup/Lerobot/projects/piper/deployment/preflight_j_hold.py)（最终 SHA `2f0050a7bd8a2f8ba1799edc199489299b9554f7375bdbd93e33b60845116c1f`）：显式每轴目标偏移≤4°、目标严格入控制器界限、仅 J2/J3 指定初始反馈例外、恢复偏移≤4.5°且目标误差≤0.5°；故障立即拒绝，支撑下逐轴失能。已审查及离线模拟，尚未实际执行。官方启动器在 RX 连接并 setup 后才调用一次该预检，待预检成功及静默 1.05s 再 arm，确保接收端观察到真实 0x151。后端发送 TTL 和源观测时效分离，39 项执行适配测试通过。

实际启动在 SSH 接收连接阶段中止，`startup_result.json` 未创建，**尚未调用原地预检、使能或 ACT action**。随后直接在 KR260 被动监听 0.6s 为 0 帧；接口仍正常 UP/1Mbps/ERROR-ACTIVE、无错误，但 RX 累计不再增加。当前不能确认机械臂最新状态，已暂停动作，等待用户核查从臂供电、急停和 CAN 线；主臂继续断电。[尝试记录](/home/aup/Lerobot/outputs/act_local_trial_v5/attempt_summary.json)，[启动日志](/home/aup/Lerobot/outputs/act_local_trial_v5/rollout.log)。反馈恢复前不得绕过 gate 或发送模式/使能目标。

## 已实现

- `guard.py`：默认 disarmed；只接受训练对应的 7 个绝对目标。6 轴为 deg，夹爪为 signed width m。各轴限位、单步变化、目标变化速度、反馈跟踪差与时限必须显式提供；不从示范极值推断硬件限位，不静默 clamp。
- arm 时以当前反馈为目标锚点，不移动到 episode 起点。第一个模型目标也必须通过步长与速度检查。
- 拒绝缺字段、非有限值、越界、过期、逆行时钟、重复提交及上一批发送结果不明。只有整批目标被接收端接受后才 commit。
- `LocalOwner` 只排除同一电脑遵守该锁的执行进程，**不能隔离主臂或其他 CAN 发送者**。
- `sdk_codec.py` 直接调用本机官方 pyAgxArm 的纯编码器，不创建 SDK driver。关节度转 0.001deg；夹爪米转 signed µm，force 为显式 N 参数，set_zero 始终 0。
- MotionCtrl2 离线编码固定为 **CAN + J + POS_VEL(0x00)**，低速比例需显式指定为 1～10%，安装方向需显式指定；不是 JS/MIT(0xAD)。这些字节没有被发送。
- `remote_bridge.py` 在 KR260 上只需 Python ≥3.10 stdlib：复用现有 `remote_can_source.py` 的身份核验、接口状态与内核接收时间工具，不修改原桥。新的 RPC 无任意 CAN 报文、模式、enable、reset、zero 操作。只有独立本地 execution policy 全部确认、显式 arm 后，才能接受按顺序的 0x155/156/157/159 四帧目标事务。
- 新部署桥的 RX/hello 门只要求接口已为 UP + 1Mbps，允许 LISTEN-ONLY 或正常 CAN；默认 `execute=false` 在两种模式下均不会调用 CAN send，也不会改接口。正常模式的 CAN 控制器可能由硬件自动 ACK，这不等于发送运动命令。`execute=true` 的 arm 门仍强制正常模式并通过全部审阅/健康门。原采集桥仍严格要求 LISTEN-ONLY，未放宽。
- 远端独占 flock、随机 lease、连续序列、接收端单调时钟 TTL、反馈/模式/夹爪故障检查再次独立执行；看到其他控制发送源立即撤销，不与主臂争抢。输入断联/lease 超时且反馈健康时，只有经明确确认的 `fresh_feedback_target_once` 策略可以把最新反馈目标发送一次；反馈不健康时不发送、不 reset/disable。
- 服务循环的 SSH stdout 非阻塞且有队列上限；接口查询在后台，不能阻塞主循环的 lease 看门狗。优雅 SIGTERM 会执行 finally 撤销；SIGKILL、板卡断电或内核挂死无法由该 Python 进程保证停止，须有人工/控制器停止方案。
- `backend.py` 复用现有四时间戳 `SourceClock`，以 offset 下界和漂移余量将 AIPC 截止时刻转换成更早的远端截止时刻；不直接比较两台机器的单调时钟。相机仍通过原 HTTP Camera 类核验角色/序列号，并检查 150ms 年龄、60ms 配对差。
- `robot.py` 是独立 LeRobot `Robot` 接口。`connect()` 延迟载入 `observe_config.json`，只读连接，自动标定/模式切换均禁用；普通 `send_action()` 拒绝。配置固定了本次经 root 核实的新 ifindex=15，原采集配置 ifindex=14 不改。未来接口重建需重新核对身份。

## 仍须补齐，不能跳过

1. 固件已确认为 `S-V1.8-2`，主臂已由用户断电；仍须完成获批准的旧固件模式切换与上电验证，并核对安装方向。主臂断电不能自动证明从臂已切到 POS_VEL。
2. 核对该设备及桌面场景的限位、夹爪 signed width 的物理零位/允许范围与力度。训练数据负夹爪值可以被编码，不等于允许发送；不擅自加偏置或截断。
3. 独立目录 `/home/ubuntu/kr260-camera/piper_policy_executor/` 已部署 `remote_bridge.py` 与 `guard.py`，复用板上既有 `/home/ubuntu/kr260-camera/remote_can_source.py`。仅 RX 首测已通过；执行配置和获批准模式下的停止/TTL 行为仍未在实机验证，不能把离线测试视为实机动作批准。
4. 只有明确批准且物理条件满足后，才建立单独 receiver execution policy 和 AIPC execution config，填入经过核实的参数与证据。这里没有实机执行配置，`observe_config.json` 的执行路径、限位和夹爪力度均为空。CAN 仍须人工按获批准步骤设置；脚本从不调用 `ip link set`，也不发 0x151 模式指令。`Readiness` 不是物理条件证明。
5. 向用户展示首次抓取的目标、轨迹/活动范围、速度、夹爪参数和停止方法，取得明确批准后才能 arm。`Readiness` 字符串只是审阅门，不是传感器证明；程序无法验证物理隔离或代替人工批准。

执行反馈也不能直接包一层现有观察 Robot：录制用 `checked_snapshot()` 还要求主臂 action 分组可用，隔离主臂后不满足其原契约。真实执行后端应独立消费 state/status，保留双相机身份、新鲜度和软件配对检查；不放宽原采集器。

## 已确认的录制模式旁证

有限检索旧 CAN 诊断发现 [can.jsonl:340648](/home/aup/Lerobot/logs/kr260_can_verification/v5_r01/can.jsonl:340648) 的 0x151 为 `01 01 64 AD 00 00 00 00`：CAN/J、速度比例 100%、MIT 直通（SDK `move_js` 一类语义），**不是本部署候选的低速 POS_VEL=0x00**。同段 [can.jsonl:340654](/home/aup/Lerobot/logs/kr260_can_verification/v5_r01/can.jsonl:340654) 的 0x159 为 `00 00 1A F4 03 E8 01 00`，即 signed width 0.0069m、1N、width enabled、无 zero。

这是旧录制随动模式的有限样本证据，不证明全部采集时刻或当前控制器模式，也不授权使用 MIT 运行模型。0x2A1 `mode_feedback=1` 只说明 J 反馈，不能区分 POS_VEL 与 J/MIT 直通。执行 arm 门还必须匹配最近实际观察到的完整 0x151 与已审阅低速 POS_VEL 配置；捕获 `...AD...` 会被拒绝。固件未知时不尝试 0x470 全零“退出”，也不自动 reset/stop；已有 SDK 文档提示旧 follower 切 J 可能涉及掉电，需人工程序核实。

## 官方 rollout 接入方式

当前固定 0.6.1 中，部署入口是 `lerobot-rollout`，不是 `lerobot-record`。独立插件使用 type `piper_policy_executor`。官方 context 只保留 `.pos/.vel` 标量后缀，所以 Robot 暴露以下固定顺序键别名；**只是字段别名，不转换单位，不改变训练数据**：

| Robot/rollout 别名 | backend/guard/训练数据字段 | 数值单位 |
|---|---|---|
| `joint_1.pos`～`joint_6.pos` | `joint_1.deg`～`joint_6.deg` | deg |
| `gripper.pos` | `gripper.m` | signed width m |

相机仍为 `global`、`wrist`，RGB 640×480。官方默认 processor 按该固定顺序组装 7 维 state/action 张量，与训练维序相同；ACT 仍使用 checkpoint 中的预处理/后处理与训练统计。动作返回 Robot 后映射回 canonical 字段，由 guard 校验，再经 Backend 发送。不另写策略控制循环或修改官方框架。

参数启动/预检入口通过 PYTHONPATH 加入 deployment 并显式 import `lerobot_robot_piper_executor.robot`，构造官方 rollout config，调用官方 `build_rollout_context()`（它通过 RobotConfig 工厂创建并以 unarmed 状态连接 Robot），再从 `ctx.hardware.robot_wrapper.inner` 取得该实例。官方 strategy 先 setup，保持 unarmed；随后读取当前状态、检查获批准的就绪证据、调用 `arm_from_current_state()`，立即运行官方 strategy 的 run，最后 teardown。该执行分支及失败清理仍待实机验证；当前默认仅 check-only。先使用同步推理，显式设置 `return_to_initial_position=false`，避免退出时自动回程；`strategy=base` 不做 episode 间自动复位。

本地参数启动器 [rollout_act_piper.py](/home/aup/Lerobot/projects/piper/official_adapter/rollout_act_piper.py) 默认 `--check-only`，不构造 Robot、不连接硬件。执行分支须同时给出 `--execute --deployment-config PATH --readiness PATH --readiness-reviewed`，并通过配置与接收端的独立门。顶层入口仍为 `run_act_piper.sh`；默认每次执行20步后重新预测，目标20Hz，运行时长仍由原有 `--duration` 控制。

**当前 ACT 动作块配置**：40步/5%试跑后仍提前闭爪，`act_launch_defaults.json` 已恢复 `n_action_steps=20`；模型仍预测100步，只消费前20步，再用最新观测重新预测。名义视觉更新窗口从2秒恢复为1秒，另加实际推理耗时。此前20步/10%有用户报告抓起过方块的证据，但并不证明稳定成功或抖动已消除。时间集成仍继承模型的 `temporal_ensemble_coeff=null`。直接调用模型启动器而不传参数时仍继承checkpoint，官方源码和模型文件未改。`--n-action-steps N` 显式覆盖优先于默认，范围1到100；旧命令若带40仍会使用40。

**当前接管速度与抓取时序**：40步/5%三次运行的实际跟随进度均比此前20步/10%慢，因此健康已使能从臂的默认CAN/J/POS_VEL速度比例恢复为10%。接管先保持实时六轴姿态，确认健康稳定后才进入ACT；冷启动恢复和该次ACT仍为5%，不会自动升速。速度由review、实际0x151记录、本机执行配置及板端policy共同绑定，任何不一致拒绝，不能用结果自报替代精确报文检查。保留反馈相对2°/3mm、90%目标速率约束、1N夹爪及原双端硬保护；J5仍−60°～70°。这次回退用于恢复跟随进度，不能保证消抖；未新增平滑滤波或延时，以免进一步加重闭爪早于到位。

**逐帧诊断**：顶层启动器为新run的`execute_config.json`写入`policy_target_trace_path`，本机适配记录到同目录`policy_targets.jsonl`。每条包含原预测、J5修正后、反馈相对限制后、最终目标、当次实际七维反馈及采样时刻、原ACT块时序、命令创建与RPC返回时刻、接受或拒绝结果。ACK只表示请求返回，最终目标也不表示实际到位；不保存图像、不逐帧刷盘。旧配置缺该字段时关闭诊断。它不改变目标、缓存来源时间、等待预算或任何保护。

**同步 ACT 缓存的时间来源**：`act_chunk_timing.py` 保留官方 `select_action` 与动作队列，只附加一次性时间元数据。每个动作的计划时刻固定为“原始观测时刻 + 块内索引/20Hz”；后续新观测不刷新旧块来源。执行适配以该计划时刻检查原有允许迟到窗口，并独立检查最新反馈年龄、接收端TTL、租约、关节和夹爪限制。队列用完后才重新预测。此前5～6Hz数据来自每步重新推理的旧配置；21:31运行中缓存段的目标接受间隔约50ms，块间仍有慢推理停顿，不能据此认定全程稳定20Hz。

**v5 运行后的发送间隔修复**：该次已接受101条目标；第100→101条间隔402ms，第101条J2目标44.488°，下一条46.152°却在约14ms内进入检查，触发45°/s保护。官方同步循环在慢推理轮后会立即开始下一轮，反馈相对2°限幅无法保证连续目标的变化速度。`robot.py` 现对带ACT时间元数据的目标，在准备命令前等待上一成功命令创建时刻之后至少`1/fps`（20Hz为50ms），RPC耗时包含在周期内。等待后重新检查原slot和反馈年龄，原硬限速及接收端检查继续生效；超过一个周期仍变化过大的目标仍会拒绝。此修复尚未实机验收。

**21:40 停止后的后续修复**：启动器现在先在任何硬件连接之前，用保存的模型和处理器执行3次离线预热，随后清空策略队列及处理器状态。维持实测最快的8线程；本次真实模型检查的3次耗时为145～155ms，另两轮201帧离线FIFO检查的6个新块为171～185ms。此前出现过约371ms长尾但未稳定复现，不能将它归因为GC，也不能保证预热消除全部长尾；400ms原始有效期继续严格执行。每个新块记录推理耗时和原观测年龄；发送前过期会区分编码后与时钟映射后，并记录命令创建时剩余窗口。

新生成的执行配置显式设置`policy_target_rate_fraction=0.9`。策略目标先经过下述显式J5小幅上界修正，再经过完整绝对范围检查和反馈相对2°/3mm限制；随后`target_rate.py`按上一成功host commit后的真实间隔，将连续目标限制在原硬速度的90%以内，并向内保留一个CAN量子。关节使用0.001°网格、夹爪使用1µm网格，主机与接收端比较同一编码解码值。反馈距离和速率区间无交集时仅作下述有界等待，无法满足仍拒绝；原步长、跟踪误差、时效、租约以及接收端硬保护均继续检查。此项会修改过快的目标，使其逐步接近策略值；不是提高速度阈值。旧配置未设置该字段时仍按原流程硬拒绝超速。

**历史J5专用饱和及余量（旧配置兼容）**：训练集`rad_place_merged_v5`的20,488帧中，J5 action最大70°，3,057帧（14.92%）恰为70°；ACT线性回归头没有输出有界激活。旧配置`j5_upper_prediction_saturation=true`仅作用于带原始ACT时序的策略提议。目标上界为`min(70°, guard_J5_upper) - j5_upper_target_margin_deg`，余量2°时为68°；高于它的有限J5提议压回该值，低于它的提议不减去余量。此前70°目标出现了板端70.072°实际反馈，因此不能将目标贴在硬上界。新默认已由上方六关节统一配置取代，以下J5字段只保留用于旧run的兼容。

该余量还约束最终执行目标：`shape_target_rate`与`target_rate_wait_s`可选`target_bounds`，求反馈相对范围、原目标速率范围、内部目标范围的CAN网格交集。若相对范围和内部目标范围本身无交集，拒绝；仅速率暂不允许时沿用原最多50ms等待及原source/sample/gap期限检查。不会在最后直接截断目标而违反限速，也不刷新反馈时间。旧配置缺余量字段时保持原行为；余量必须为0～5°的JSON数值，并且要求J5饱和、反馈cap和目标限速都启用。新run启动计划明确打印J5目标上限68°和反馈硬上限70°。

每次饱和保存raw、投影值、累计次数及连续饱和诊断，结束时汇总。该flag必须为JSON布尔值；与旧`j5_upper_prediction_tolerance_deg`同时配置拒绝。旧run显式1°容差继续只接受(70°,71°]，两字段都缺失时保持原始越界即拒绝。既有`--run`入口会为新run生成新配置；重用旧run的execute_config不会自动获得此变更。固定checkpoint的路径、权重SHA、deg/m单位和官方框架均沿用原检查。

如果重新预检时J5真实反馈仍超过70°，现有ACT接管会在发初始化命令前拒绝；没有把越界反馈归一化或加入自动复位、回零。v6本轮RX-only检查J5仍70.026°、六轴使能且电机故障位为0，当前持物姿态不能直接重启ACT。该快照不是后续执行的实时证据。

为此另准备`recover_j5_boundary_once.py`，默认`--check-only`只输出本地计划，ACT入口不会调用它。实际执行需要用户针对当前持物回退的明确批准及现场条件review：新CAN/J/POS_VEL5%模式帧仅一次，0x157将J5先退到69°、真实反馈到位后再68°；J6从每次新鲜反馈保持，其他关节和夹爪不发送目标，不enable/disable/reset/zero。复用独占CAN owner，原始起点只允许J5在(70°,70.1°]的窄恢复域，其余轴、故障、新鲜度、外部控制和持物宽度均检查；每个目标最多尝试一次，失败不继续、不重试，也不声称已安全物理停止。结束仅确认回到有效范围，不自动开始ACT或完整抓取。准备中的`recovery_review_pending.json`所有授权和现场条件字段均为false，不能用于执行。

**22:16 首命令编码初始化修复**：22:15和22:16两次运行均接受0条ACT目标；模型推理169～174ms，命令创建时尚余193～210ms，但首次编码耗时228～235ms，导致发送前过期。此前只预热模型，遗漏了`encode_target`内SDK导入和编码器初始化。完整模型进程的离线检查已复现首次编码232ms、准备后0.026ms。`prepare_target_codec()`现于任何Robot构造前完整编码并核对关节/夹爪四帧字节，丢弃结果，缓存纯编码器及消息类；执行backend也在连接前准备，并在取得租约前兜底。只读backend不进行此准备。新增独立冷进程回归，避免测试套件已加载SDK掩盖冷启动问题；原编码后截止时间检查继续保留。此修复尚未实机验收。

**22:55 反馈距离与目标限速交集修复**：上一夹爪目标63.817mm，下一反馈约58.6mm，3mm反馈距离限制要求新目标不高于61.6mm；提交后约47ms时，90%目标限速尚只能退到约61.70mm，形成约0.1mm空交集。这不能单凭日志判为夹爪硬件故障。适配器现仅在约束需要额外时间时，等待纯函数计算的必要间隔（该例约2.31ms）；最多等一个控制周期且不超过50ms。正常帧不额外等待，避免逐帧叠加RPC时间。原slot、原反馈时间和命令间隔在等待前后继续检查；超等待预算或过期仍拒绝，反馈cap、90%速率约束、CAN量化与双端硬保护均未放宽。此修复尚未实机验收。

readiness JSON 的字段均为非空证据引用字符串：`leader_isolation`、`firmware_and_j_mode`、`hardware_and_fixture_limits`、`signed_gripper_mapping`、`verified_stop_strategy`、`robot_side_lease_and_watchdog`、`approved_goal_path_parameters`。AIPC 的 `Readiness(**json)` 必须与接收端 execution policy 完全一致；不能用测试占位文字填实机配置。

## 离线验证

离线测试覆盖 guard、官方 Codec、假 stdio RPC（包括 LISTEN-ONLY/正常模式 hello）、源时钟截止时刻、默认只读、过期、断联、状态陈旧、未知 CAN ID、部分事务、外来控制源、并发 owner、夹爪 angle/width 区分、官方 Robot 工厂、7维特征后缀、预热状态清理，以及连续目标限速和双端量化一致性。下面只运行假数据测试，不连接硬件、不安装依赖。所有测试限位均为合成值，不能用于实机：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH='/home/aup/Lerobot/projects/piper/deployment:/home/aup/piper SDK/pyAgxArm' \
/home/aup/Lerobot/envs/piper-cpu/bin/python -m unittest discover \
  -s /home/aup/Lerobot/projects/piper/deployment -p 'test_*.py' -v
```

本地依据：

- SDK `protocols/can_protocol/drivers/piper/default/driver.py:137`：关节目标到 0.001deg；`:993`：`move_j()`；`:1020`：JS 的无规划/机械冲击警告。
- SDK `protocols/can_protocol/drivers/piper/default/parser.py:400`：0x151 编码；`:425` 起：0x155～157 编码。
- SDK `protocols/can_protocol/msgs/piper/default/transmit/arm_mode_ctrl.py:130`：POS_VEL=0x00、MIT=0xAD。
- SDK `protocols/can_protocol/drivers/effector/agx_gripper/default/driver.py:368`：米、N 转原始控制量；其 `parser.py:69` 起为 0x159 编码。
- 官方 `lerobot/rollout/strategies/core.py:303`：经过官方 robot action processor 后调用 `send_action()`。本机库位置 `/home/aup/Lerobot/envs/piper-cpu/lib/python3.12/site-packages/`。

SDK 根目录：`/home/aup/piper SDK/pyAgxArm/pyAgxArm/`。以上源码证据只说明本地 SDK 协议，未确认实机固件一致。


# 用户指定恢复30步alpha0.7历史版本（v8）

- 已按用户指定的 `deployment/baselines/previous_30steps_alpha07_v7_r15/manifest.json` 成套恢复11文件，逐项SHA256一致。30步、alpha0.7、默认30秒、原ACT；同时恢复当时放置续20步以及60mm/150ms/2秒开口确认逻辑。此为精确历史版本恢复，未宣称解决后续已发现的问题。
- 恢复前50步/alpha0.5/a160/无限时长及放置prefix、短暂settle版本的11个切换文件与97个支持文件，保存于 `deployment/baselines/before_restore_alpha07_v8_r31/`，分别有manifest与support_manifest。原模型未改，新增helper留存但基线入口不启用。
- 一键恢复本次操作前版本：`bash /home/aup/Lerobot/projects/piper/restore_act_piper_before_alpha07_restore.sh`；重新切到用户指定alpha0.7：`bash /home/aup/Lerobot/projects/piper/restore_act_piper_alpha07.sh`。加`--check`只校验。工具先验证所有源文件、当前版本及依赖；未知后续修改拒绝覆盖。旧`restore_act_piper_30steps.sh`指向另一alpha0.5快照，不能代替此次精确恢复。
- 本条记录操作历史；后续切换后，以实际profile与11文件SHA为准。仅影响下次启动，未启动/停止服务或发送机械臂动作；历史execute_config、姿态与抓取授权不可复用。
