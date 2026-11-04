# 阶段 C：参数化 PL 执行器规格 v1

本规格供阶段 D 编写 RTL。2026-11-04起当前工程请求时钟为50 MHz，Vivado按20 ns检查；KR260预设派生的标称 `CLOCK_HZ=49,999,500 Hz`同时用于积分和DDS。112周期内部描述符延迟约2.24 µs。动作频率仍由原来的 `f_exec=num/den` 配置，降频不改变这个比值。50 MHz构建结果见指定交付目录中的实际报告。阶段 A 已按用户要求删除；B 的真实权重、预处理和模型对齐证据继续使用。

## 1. 设计范围与数据通路

```text
模型 → B适配器/原时间调度 → PS的C2规划及整数校核 → inactive chunk_ram
                                                       ↓ 校验/冻结
CLOCK_HZ → tick_gen → player → limit_gate → tx_backend → 描述符消费者
           ↓          ↑          ↑            ↓
          stats ← safety_fsm ← lease/硬故障   stats
```

PS 不能写 TX FIFO、运行中状态或绕过 `limit_gate`。`tx_backend` 的唯一目标输入来自该门；BRAM 内是候选段，不是可以直接发送的帧。另一个 Linux 进程仍可操作现有 USB-CAN，因此这里的“唯一通路”是新 PL 核内部约束，不冒充整台电脑的权限隔离。

现有物理路径含 PS/Linux/USB-CAN，未接到 PL。v1 的确定性边界止于 **完整 96 字节描述符 valid 脉冲**。112 周期预算不包括 CAN 仲裁、四个串行帧、PS/USB 或电机运动。物理线上的“环频率=出帧频率”尚未实现；见 [TRANSPORT_NOTES.md](TRANSPORT_NOTES.md)。不据此宣称已经完成任务书的物理出帧验收。

## 2. 顶层接口与时钟

所有内部端口同用 `aclk=CLOCK_HZ`，复位同步释放。PS AXI 域由 BD 的时钟转换器接入该域；不直接跨域传递多位总线。AXI BRAM Controller 只占真双口 BRAM 的 A 口（single-port controller），PL 独占 B 口；两口读延迟均配置为 1。PS 窗口写入必须先过 bank 所有权防火墙，不能把原始无保护 BRAM 从机额外映射给 PS。

| 端口 | 方向/宽度 | 契约 |
|---|---|---|
| `s_axi_cfg` | AXI4-Lite，32bit | 4KiB 寄存器窗口，详见 `regmap.yaml`；非法/RO写返回SLVERR |
| `s_axi_chunk` | AXI4，32bit | 128KiB 候选BRAM窗口；只允许 FREE bank，其他写/读改写返回SLVERR |
| `hard_fault` | in，1 | 同步器后最高优先级；锁存故障，禁止valid并请求断使能 |
| `backend_ready` | in，1 | 只在第112拍采样，消费者保证该拍能接完整描述符；不能等valid再组合生成ready |
| `tx_desc_valid/data` | out，1/768 | 合法tick固定第112拍给一拍valid；背压时不补发、不积压旧目标 |
| `disable_req` | out，1 | 高有效故障请求，复位即为1；不是已接线的驱动断电证明 |
| `motion_permit` | out，1 | 只有已ARM且无故障才为1；不能直接解释为夹爪使能CAN字段 |
| `dbg_tick/dbg_desc` | out，1/1 | 一拍脉冲；引脚均留 `TODO(pin)`，不是已接示波器 |

没有物理后端的版本 `CAPS=DESC_ONLY`。此版本可以验证描述符，`disable_req` 始终为1，不提供真实执行使能。以后有物理后端时，必须另外验证收发器、使能极性及故障输入；不得把内部请求改称已断电。

| 模块 | 单一功能与关键内部端口（均在aclk域） |
|---|---|
| tick_gen | cfg_num/den[63:0]、config_commit → tick、tick_index/cycle[63:0]、实际interval[31:0]；计数/相位只由该模块写 |
| chunk_ram | AXI防火墙、冻结/CRC/时序/包络预检及预取；req_bank/index[0:0]/[7:0] → record_valid/data[2047:0]及cert_epoch[31:0]；只有预检成功能置READY |
| player | 已冻结record与tick → candidate_q/v/a[7×64]、overflow、九组只读未RNE乘积[9×7×192]；只计算，不直接更新已提交状态或TX |
| limits_product_cache | PL校验后的raw limits/epoch → 五组精确边界乘积[5×7×192]；35项双partial乘法流水计算，保留完整162位乘积，E38原子发布 |
| limit_gate | 十五拍按轴流水消费同job精确项、RNE端点、raw limits/cache与previous_wire → checked_q16[7×32]、线单位目标、allow/reject；第112拍仍须核对本job完成及最终ready |
| safety_fsm | 每拍hard_fault/lease、tick边界bank事件、gate结果 → RUN/RAMP/HOLD选择、permit/disable及粘滞故障；所有valid最终与无故障相与 |
| tx_backend | 只接limit_gate结果 → 96B descriptor；第112拍ready握手，其他来源不接入 |
| stats | tick/allow/reject/fault事件 → uint64计数及原子snapshot；不参与目标计算 |

当前预检与player共用一套player_math和一套limit_gate_pipe，tick计算/预取优先；未完成预检不能抢占固定输出流水线。验证缓存中保存的证书绑定bank_sequence、config_epoch与CRC，配置失效即全部作废。共享模块与gate的通用默认均保留E0输入快照；当前仅顶层在§20的完整job稳定合同下使用`STABLE_CACHE_INPUTS=1`并将其传给gate的`STABLE_JOB_INPUTS=1`。该模式旁路gate的重复输入快照，不旁路检查或S0流水寄存。

## 3. tick_gen：精确有理分频

`RATE_NUM`、`RATE_DEN` 为正 uint64，令 `T=CLOCK_HZ*den`；乘积必须可表示为uint64。每个时钟计算65位 `sum=phase+num`，若 `sum>=T` 则 `phase=sum-T` 并发tick，否则 `phase=sum`。只允许 `floor(T/num)>=256`。这是内部流水线/预取约束，不是执行器能力。

CONFIG_COMMIT只允许DISARMED且两bank FREE；CRC/raw校验通过后原子锁存静态配置，原 `tick_commit` 拍清DDS phase/tick_index并启动配置cache；中途改频返回错误。自由运行CYCLE不清零，DDS在原提交拍锁存只读CYCLE_ORIGIN，**不移动到cache完成或AXI ACK拍**。cache在自身E38原子发布，`C_ACTIVATE`等待其done及epoch匹配后才回复配置成功，较旧E36缓存版增加2clock初始化等待；不延长来源TTL/lease，也不重置或暂停已锚定的DDS时间。首tick在 `origin+ceil(T/num)`，第n个tick在 `origin+ceil(n*T/num)`，n从1开始；header/record全部使用这个绝对cycle，Python DDS的结果为相对origin的偏移。每个tick间隔只能是floor/ceil(T/num)，长期相位误差 `[0,1)` 时钟。还必须满足 player 的最长间隔 `ceil(T/num)<=2^32-1`，否则拒绝整个配置。周期计数器uint64不回绕运行：加法先扩宽，无法表示未来段/112拍延迟时拒绝，运行溢出则锁存FAULT，硬复位后重新初始化session。块切换/HOLD/STARVE不得重置DDS相位。

`sim/tick_reference.py` 是整数golden。`275/2 Hz` 仅作非整数测试；物理频率字段保持null。tick在已配置时持续计数；RUN/RAMP/HOLD每个合法tick恰好一个描述符，DISARMED/FAULT没有目标valid。没有第二个隐藏的1kHz控制环。

## 4. 模型到原生段的映射

模型仍输出七维100步、数据20Hz；B证据的运行覆盖为30步。当前旧部署由独立任务管理，本规格不覆盖其后续改动。PL协议没有硬编码ACT、100、20或30/50。实际消费量必须从所选profile读取，不能拿B历史快照覆盖当前部署。

PS先按 B 的原观测时间选择有效模型槽，再在DDS的**实际整数clock间隔**求解常jerk段。B的 `C2Tracker` 提供浮点规划算法；PS还须按下述整数golden重新传播和留量化裕量。不能把 B 的均匀浮点 `1/f` 结果直接当作整数DDS的bit一致输入。

每段存起点 `q0,v0,a0,j`，都是signed64 **Q32.32**，六轴单位deg及其秒导数，夹爪m及其秒导数。最终位置才转Q16.16。禁止直接在Q16.16反复积分。设实际段长 `d=end_cycle-start_cycle`，`C=CLOCK_HZ`：

```text
a1 = RNE((a0*C + j*d) / C)
v1 = RNE((2*v0*C^2 + 2*a0*d*C + j*d^2) / (2*C^2))
q1 = RNE((6*q0*C^3 + 6*v0*d*C^2 + 3*a0*d^2*C + j*d^3) / (6*C^3))
```

RNE是对完整有理式做一次nearest/ties-to-even，负值对称处理；不是每项先舍入。系数按全参数上界精确存储，signed64状态按低32无符号补0成signed33、高32有符号拆分，与完整signed98系数分别形成signed131/signed130寄存partial；先符号扩展到162位再把高项左移32并相加，完整signed162乘积再扩展到signed192，累加和商保留192位，余数按正分母严格小于2^96保存96位；没有中间舍入或数据截断，最终回到64位前检查溢出。下一段q/v/a必须等于本段计算终值逐bit相等，不允许用容差掩盖接缝；换bank同样检查。即便如此，相对无限精度多项式仍有每段不超过0.5个Q32 LSB的终点舍入误差，不能宣称物理输出严格C2或实际jerk已证明。

player在tick i计算覆盖 `[tick(i-1),tick(i)]` 的段终值，输出代表tick i的位置，描述符延迟112cycles固定。首段起点来自显式静止seed（v=a=0），不能默认全零关节。seed位置、采样时刻、配置和session一起提交；其真实性来自未来接入的反馈端，本规格不伪造PL独立测量反馈。

**插值/时间缩放**：常jerk多项式在任意段内时刻按同式求值，是三次插值。PS可显式重规划 `s>=1` 的慢速版本：时长乘s，v/a/j分别除s/s²/s³，重新量化并以真实DDS时间生成记录。截止时刻仍为原观测截止；超期拒绝，不能用缩放复活旧来源。v1不在PL内动态改s，也不在STARVE时拉伸旧段。

这是 B 规划算法的硬件实现分解，**尚未完成“B录制轨迹→本格式→RTL”的一致性验证**。阶段D必须同时生成同整数算法的PS baseline；原B浮点baseline继续保留作精度/跟踪对照。不能将新整数算法的变化全归因于PL时钟。

## 5. limit_gate：连续包络与最终编码

入队前计算每轴精确有理 Bernstein 控制点，按floor/ceil向外包络到Q32.32整数；越界拒绝整个bank，允许保守误拒，不能向内舍入。常jerk段长h=d/C：

| 量 | 控制点（凸包覆盖整段） |
|---|---|
| q，三次 | q0；q0+v0·h/3；q0+2v0·h/3+a0·h²/6；精确q(h) |
| v，二次 | v0；v0+a0·h/2；精确v(h) |
| a，一次 | a0；精确a(h) |
| j，常数 | j |

q还须处于 `[q0-DQ_MAX,q0+DQ_MAX]`，v/a/j分别处于对称限值域。所有限值都是显式Q32.32配置，复位0无效；B回放场景的200/1000/20000等数**不是部署默认限值**。每tick再检查溢出、状态连续、时效、当前位置/导数和DQ；预检证书只对应冻结bank与同一config_epoch。

当前实现不在运行gate重复求d²/d³及上述宽乘积。player只读导出九组未RNE的signed192精确项，每组七轴：

```text
Q=6*C^3, V=2*C^2
qp=[q0*Q, 6*v0*d*C^2, 3*a0*d^2*C, j*d^3]
vp=[v0*V, 2*a0*d*C, j*d^2]
ap=[a0*C, j*d]
Nq=[3*qp0, 3*qp0+qp1, 3*qp0+2*qp1+qp2, 3*(qp0+qp1+qp2+qp3)]
Rq=[0, qp1, 2*qp1+qp2, 3*(qp1+qp2+qp3)]
Nv=[2*vp0, 2*vp0+vp1, 2*(vp0+vp1+vp2)], Na=[ap0, ap0+ap1]
```

Nq/Nv/Na的公共正分母分别为3Q/2V/C；Rq=Nq-q0×3Q。物理边界检查 `3*lower*Q<=Nq<=3*upper*Q`，相对DQ检查 `-3*DQ_MAX*Q<=Rq<=3*DQ_MAX*Q`，与原max/min交集严格等价。乘2/3使用精确192位shift/add，不整数除以2或3。全合法参数域下Nq/Nv/Na最多需164/131/97个signed bit，3Q分母本身最多需99个signed bit，统一192不回绕。

PL的 `limits_product_cache`准备每轴lower×Q、upper×Q、DQ_MAX×Q、vmax×V、amax×C，共35项。当前7cd18版本把完整signed64×signed98分为两路精确partial乘积，再按完整signed162重组；E0锁存raw配置/epoch，E1..E35寄存选定输入，E2..E36寄存partial，E3..E37写私有scratch，E38一次发布五组7×signed192及cached epoch（详见§18）；raw在job内冻结，epoch变化或无效不能发布/复活cache。使用时乘3或2，不能从PS接收一个未复核的乘积/许可位。配置、cache与record都必须属于同一epoch；validator再锁自己的raw配置快照，ARM、bank接纳和输出提交都核对匹配。

原生乘积接口仍为192位；门内按signed162处理。默认0模式在E0快照，当前顶层专用1模式使用§20稳定视图；两者在E0均检查全部98项高30位是bit161的符号扩展且无X/Z，否则锁存本job拒绝。hull寄存器signed166，Q32区间65、Q16码49、缩放85、wire码54位；每次算术/比较前显式符号扩展回192，完整raw64域含非法边界也不会截断清除拒绝。数学证明见[位宽说明](../docs/stage_d_compact_arithmetic_20261102/gate_width_proof.md)，源级位数变化不等于综合FF利用率。

`limit_gate_pipe`的`STABLE_JOB_INPUTS`默认0，在start E0快照同job输入；值1只在§20证明的共享owner合同下以同位宽稳定视图代替重复数据快照。E0仍锁存bad_job及previous-valid、清旧许可并核高位/XZ/valid，固定E15输出done。E1..E7逐轴注入同一条算术流水线，轴6在E13完成，E14归约全部七轴状态，E15原子发布。默认0模式下busy期间外部扰动不改变快照，所有模式忙时start均不重采样；模式1要求生产者直到job排空前保持完整数据不变。拒绝时allow/clamp/q16/wire全零。来源时间与控制权限仍由顶层检查，15拍本身不是STA结果。组合 `limit_gate`继续作golden，seed仅选STATIONARY专用路径：v0/a0/j/v1/a1必须全零且q1=q0，保留其他全部位置、正限值与量化门；运行段不能借它绕过连续包络。

Q16位置为 `RNE(q32/65536)`，收回到 `[ceil(lower32/65536),floor(upper32/65536)]`，只允许最终量化带来的至多1LSB修正，并记CLAMP。真实物理越界不可当普通钳位继续跑。量化后再检查 `abs(next_q16-previous_q16)*65536<=DQ_MAX_q32`；内部小步长不代表跨舍入门槛的输出也小。CAN整数再次进行保守合法域量化：关节单位0.001deg，夹爪µm。转换后的位置和相邻CAN目标DQ仍须处于限值域，空整数域/溢出拒绝；不能在边界四舍五入出界。abs(INT64_MIN)、q±DQ及比较先扩宽，禁止64bit回绕。

## 6. chunk_ram：双bank、字节布局、原子交接

BRAM总128KiB，bank0相对0x00000，bank1相对0x10000，每个64KiB。每bank的首256B为header，后255条256B记录；normal_count≥1、brake_count≥1、两者之和≤255。容量是原生执行段数，不是模型槽数；高执行率时需更频繁补bank，不能假定一个模型chunk一定装得下。PS必须有足够的剩余horizon供规划/校验，来不及就走停止尾，不覆盖当前bank。

数值均little-endian，保留字节必须0。header字段见 `regmap.yaml.bank_header`。`payload_crc32`覆盖实际使用记录；`header_crc32`覆盖256B header，但自身四字节置0。CRC为CRC-32/ISO-HDLC（reflected 0xEDB88320，init/xorout全1，与zlib.crc32一致）；它检错，不是安全认证。

记录布局如下（轴顺序J1..J6、夹爪）：

| 相对偏移 | 字段 |
|---|---|
| 0x00/0x08 | start_cycle/end_cycle uint64 |
| 0x10 | 执行终点tick_index uint64 |
| 0x18/0x1c | kind uint32（NORMAL=1、BRAKE=2）/reserved=0 |
| 0x20+axis·0x20 | q0/v0/a0/j，四个signed64 Q32.32 |

bank状态为 `FREE→VALIDATING→READY→ACTIVE→FREE`，失败 `VALIDATING→REJECTED→FREE`（PS显式RELEASE，错误原因保留）。不自动释放正在使用的bank。PS写FREE bank，先payload后header，执行Device-memory写屏障并读回，再写门铃快照sequence并COMMIT。header.first_cycle是首记录start_cycle，first_tick_index是首记录end_cycle所对应tick。

写防火墙只接收32bit对齐、INCR、最多64beat的burst，同时至多一个未完成写事务；跨bank/越界burst在AW拒绝。COMMIT接收后立即关闭该bank的新AW；此前已握手的burst必须完成所有W及BRAM写、B应答和两拍存储settle，随后才进入VALIDATING并返回CMD成功。读CRC期间没有任何晚到写；异常缺W由独立lease使系统停止，不能无限等待时放开bank。READY/ACTIVE/VALIDATING的新写均SLVERR；双口RAM自身不能替代这个防火墙。PL读口仲裁优先当前/下一段预取，其次后台校验；后台未完成就不能READY。

session_nonce/config_epoch/config_crc必须匹配；bank_sequence严格递增（失败不消耗已接受序号）；所有normal段严格按DDS相邻ticks排列；第一段的起点接前一bank正常轨迹终点，`previous_sequence`必须匹配；第一bank用previous=0与已审查seed接合。正常运行只在当前bank的normal终点转入READY successor。若没有及时READY successor，立即选择当前bank的brake尾，不能中途回到正常轨迹；brake末端v=a必须**精确为0**，否则不成为READY。

READY不保证以后永不过期：每次选择及输出还检查时效，过期bank拒绝，不能沿用上次validation的时间判断。两bank忙时PS等待；不能用新块序号重写ACTIVE/READY块。完成normal_count后才允许切successor；brake记录紧接其后，tick索引和q/v/a也必须连续。

CONFIG_CRC32的规范字节串为影子寄存器窗口 `[0x030,0x074)`、`[0x100,0x2c0)`、`[0x460,0x468)` 按地址顺序拼接，共524B；CONFIG_CRC32自身四字节视为0，axis行未使用的16B必须0。所有字段little-endian，不能依赖C结构体padding。seed、bank门铃、ARM_FIRST_TICK和lease kick不在这个CRC内；SEED_CRC32独立覆盖 `[0x400,0x448)` 的72B。CRC与nonce仅检错/防混session，不是对恶意主机的认证。

## 7. 时间域、lease与故障

CONFIG_COMMIT只锁存CONFIG_CRC32覆盖的静态配置；门铃、ARM、KICK、seed是独立邮箱，不在运行中改写静态配置。CONFIG_COMMIT同时校验Q16及CAN整数合法域非空、所有正限值、MAX_MAP_ERROR≤uint32最大值、非零session/epoch、gripper审阅字段；无效配置不部分生效。

PL的uint64 CLOCK_HZ周期计数是唯一执行时间域。PS/PC观测的monotonic时间不是同一个时钟：未来后端需通过带往返界的相关采样得到映射 `(ps_anchor_ns,pl_anchor_cycle,误差界,有效期)`；跨网络还要计入主机间偏移/漂移。当前尚未实现该物理映射。每块提供保守 `map_error_cycles<=MAX_MAP_ERROR`；不能直接把PC纳秒除10当PL周期或收到块时重打来源时间。

接收normal段须满足：`observation+error<=produced-error`、`produced+error<=now`；`now+error<normal_deadline<=observation-error+SOURCE_MAX_AGE`；每个normal输出的 `end_cycle+112+error<normal_deadline`。所有加减法先扩宽到signed192检查，负时间/uint64溢出即拒绝，不允许回绕。误差检查向不利方向取值，可能保守误拒。brake尾是停止程序，不是继续执行过期观测目标，但同样受位置/导数和独立stop_timeout保护。

lease由当前session严格递增的KICK序号刷新，写影子值后专门CMD提交；普通读寄存器/写BRAM/推理新序号不刷新。lease超时与hard_fault都在**每个PL时钟**检查，不等下一低频tick。时钟回绕、配置非法、后台写ACTIVE、链不连续、算术溢出均可锁存故障。

lease在ARMED/RUN/RAMP/HOLD中生效，ARM之前须至少一次有效KICK；DISARMED不因未KICK或backend_ready低而故障。hard_fault始终锁存。backend_ready仅对本tick计划输出的第112拍生效；背景校验失败只拒绝新bank，不无故破坏当前合法bank。

| 优先级 | 条件 | 唯一动作与恢复 |
|---|---|---|
| 最高 | hard_fault、lease过期、内部溢出、背压/本tick缺记录、正常段早期过期 | `FAULT_DISABLED`，立刻撤销permit/请求断使能；清本tickvalid，不补帧 |
| 次高 | 正常bank用完且无READY successor，或对齐边界的STOP | `RAMP`，只进入与当前状态匹配的已校验brake尾；没有可用尾就FAULT_DISABLED |
| 其次 | RAMP精确到v=a=0 | `HOLD`，每tick只发静止目标；不能用阈值瞬间归零冒充连续制动 |
| 最低 | 仅位置输出量化的≤1LSB回收 | `CLAMP`事件，保留RUN；任意实际越界走最高优先级 |

复位进入DISARMED（disable_req=1），CONFIG_COMMIT通过后仍DISARMED。ARM要求合法配置、完整初始bank、未来first_tick、静止seed、有效lease及backend模式。SEED_SEQUENCE非零，SEED_CYCLE不得未来且按SOURCE_MAX_AGE检查；首record的q/v/a必须与seed/0/0一致。到first_tick进入RUN。正常STOP置pending，禁止选择successor，在当前bank的normal结束才进入该bank已对齐的brake尾；从STOP开始累计STOP_TIMEOUT，入队还要求整bank时长不大于STOP_TIMEOUT，因此不能无限排队等停止。每拍急停不受该等待限制。

DISARM无条件撤销permit/valid并废弃预取，待内部BRAM事务结束后将bank回FREE、清configured，进入DISARMED；不能把运行中DISARM称为平滑停止。CLEAR_FAULT仅在DISARMED、两bank FREE、hard_fault低时清锁存，不设置permit也不保留旧session；下次CONFIG_COMMIT必须使用新nonce。RELEASE只允许REJECTED或DISARMED下的READY，不释放ACTIVE/VALIDATING。HOLD/FAULT都禁止自动因新块恢复：重新DISARM、清故障、重建session/seed/bank再ARM。

**四级停机不是固定依次经过四步。** 有安全停轨时减速再保持；没有可行轨迹或硬故障直接断使能请求。v1不在线求解任意时刻的最优制动，不把断使能请求冒充物理制动距离已验证。

FAULT_CODE固定编码（同拍按编号小者优先，第一故障锁存）：0 NONE、1 HARD_FAULT、2 LEASE_EXPIRED、3 CLOCK_OVERFLOW、4 ARITH_OVERFLOW、5 RECORD_MISS、6 SOURCE_EXPIRED、7 BACKPRESSURE、8 CHAIN_MISMATCH、9 STOP_TIMEOUT、10 INTERNAL_ERROR。REJECT_CODE：0 NONE、1 BUSY/OWNERSHIP、2 LAYOUT/RESERVED、3 CRC、4 SESSION/CONFIG、5 SEQUENCE、6 TIMESTAMP/DDS、7 LIMIT/QUANTIZATION、8 CHAIN、9 BRAKE_NOT_ZERO、10 CAPACITY；入队拒绝保持运行bank，不伪造已接纳。拒绝新块导致后续无successor时仍按正常STARVE规则停止。

## 8. tx_backend 与描述符

96B=32B元数据+四个16B CAN记录。元数据0x00 emit_sequence u64、0x08 tick_cycle u64、0x10 tick_index u64、0x18 config_epoch u32、0x1c flags u32（bit0 RAMP，bit1 HOLD，bit2 CLAMP，其余0）。emit_sequence是本session成功valid/ready传递的逐组序号，从1递增，拒绝/背压不消耗，HOLD照常递增；SNAP_LAST_SEQUENCE对应它。header.bank_sequence另为非零严格递增的bank序号，0仅作首bank的previous哨兵。两类序号禁止回绕，不能混用去重。CAN记录=id u16 LE、dlc u8=8、flags u8=0、payload[8]按线序、reserved u32=0。ID顺序0x155/0x156/0x157/0x159；端序/单位见TRANSPORT_NOTES。CAN转换以已受限Q16为输入，RNE后向合法整数域内收，再对相邻实际CAN解码位置复核DQ，初始previous来自seed相同编码。

夹爪0x159含使能字段，因此不能静默照抄旧码0x01。`GRIPPER_POLICY`必须显式确认status、set_zero=0和审阅位，力度亦明确给出；未配置时不生成整组描述符。DESC_ONLY的例子是数据编码验证，不授权外部消费者执行。v1不生成0x151/0x470/0x150、不清错、不设零点。

只在tick后的第112拍检查 `backend_ready`：为1且无故障就同时valid并完成一次传递；为0则同拍锁存FAULT_DISABLED，丢弃该组，计BACKPRESSURE，不拉长流水线、不突发补发。此前ready改变不预留、不承诺以后空间。外部消费者必须能在第112拍接受完整96B组，内部序列化/总线背压自行另外统计。描述符消费者即使是PS，也不得有额外的raw目标写入该核FIFO；PS自行调用系统CAN不在本核硬件保证范围内。

## 9. player时序与112周期设计预算

入队CRC/全段包络校验、bank装载、PS传输不在tick流水线内。预先从冻结bank的PL口读入当前及下一条记录，32bit口读64words=64cycles；每个最短256cycle间隔留足112cycle计算及64cycle预取。miss不等待，走故障。当前TDM实现保持7轴并行；每轴按同一九项调度复用低signed33×signed98及高signed32×signed98两路完整partial表达式，先注册全部131/130位，再扩162/移位重组及扩192，不进行中间舍入。每轴另有一个共享radix-8除法器，q/v/a依次各用22个迭代边沿；这是七轴并行、轴内三个商时分复用，E78完成不变。

C初始提出的四条record提前缓存不是当前D实现组织。当前只在无效的`cur_record`内原地填充64个word，第63字返回后才原子置`cur_valid`；READY表示整bank已通过校验，首record或切换后的record仍须在目标tick前完成预取。第63字返回与tick同沿时，tick只见旧的invalid，仍报MISS，不旁路接收半发布记录。最后normal记录的tick第0拍决定下一tick接successor还是本bank的brake；该拍未READY的successor不再追赶本次切换。RAMP一旦选择不被随后新块打断，所有选择都不改DDS相位。其他内部状态只在成功输出第112拍提交；故障时丢弃candidate，下一tick不会沿未发送状态继续走。

| tick上升沿为0 | 周期数 | 完成的工作 |
|---|---:|---|
| 0→4 | 4 | 锁存record/时效/安全输入（不预留外部ready） |
| +4 | 1个边沿 | math接受start E0 |
| +5..+17 | 13个边沿，与下一行重叠 | E1..E9寄存乘法输入，E2..E10寄存完整partial，E3..E11重组九项；q/v/a分子与种子分别在E9/10、E11/12、E12/13准备 |
| +15..+80 | 66个迭代边沿 | 每轴同一除法器在E11..32求q、E33..54求v、E55..76求a；每商22轮共66位，99位trial比较1D..7D，无按数值早退 |
| +81..+82 | 2个边沿 | E77精确RNE/int64溢出检查，E78给done及q/v/a或全零无效结果 |
| +82 / +83 | 1 | math done后，下拍gate接受同job精确项、raw/cache及RNE端点 |
| +83→+98 | 15 | gate固定E15 done，精确hull/相对DQ、Q16与CAN逐级舍入及实际wire门 |
| +99 | 1 | top锁存本job gate_ready；每个新job先清ready，禁止沿用旧allow |
| +99→+112 | 13 | 保持本job结果并打包，第112拍按全部ready/allow、cache epoch、时效、故障与backend_ready提交或拒绝 |
| **外部合计** | **112** | 固定功能延迟，余8拍到任务书120拍上限；不等于STA通过 |

上述排程的当前证据为[按轴gate实际HDL验证](../docs/stage_d_axis_gate_20261102/verification.json)：99,999,001参数下pipe、原生乘积域故障及top均实际PASS，pipe为738向量、15处复位中止及2个单位/尾轴拒绝定向用例，top为42,377项且外部仍112拍；最大正int32参数的pipe/domain也通过。该历史快照中的math E78、cache E36及其他模块沿用[前版九套验证](../docs/stage_d_compact_arithmetic_20261102/verification.json)，该历史归档的gate是E8，不能改称E15。最大整数参数只是数值域测试，不表示GHz运行能力。该历史记录的78/15/36/112均为功能仿真周期，当前cache E38见§18；实际标称CLOCK_HZ=99,999,001、目标10ns的每拍逻辑仍须真实STA确认。完整核心曾多次因主机内存保护停止，尚无最终利用率、实现签核或bitstream；[实际独立子模块综合](../docs/stage_d_partition_20261102/children_runtime/verification.json)也不是完整设计容量或布线后STA。若实现不能满足时序，必须改架构/规格并重新验证。fault可在流水线任何拍撤销valid；正常目标的valid延迟始终112，不为等待ready延长。

## 10. stats与资源估计

SNAPSHOT在一个clk锁存cycle/tick/emit/reject/clamp/starve/backpressure、latency_min/max、last_seq、fault_cycle；PS只读snapshot，避免32bit总线撕裂64bit读数。tick→descriptor应min=max=112；计数只表示内部事件，CAN/电机指标另列。

| 资源 | **估计范围** | 依据/不确定性 |
|---|---:|---|
| chunk双bank | 32～36 BRAM36 | 128KiB，按32bit宽4096深分片及端口拼接估计，非只用总bits向上取整 |
| 预取/元数据/寄存器 | 2～8 BRAM36，3k～8k FF | 当前/下一record、冻结配置、计数及验证缓存 |
| 积分/并行常量除法/包络 | 30k～80k LUT、15k～60k FF、150～700 DSP | 7轴与3商并行宽算术；取决于共享/常量优化/流水线，最大风险项 |
| 总计 | 34～44 BRAM36，35k～90k LUT，20k～75k FF，150～700 DSP，URAM 0 | **C初始设计估计，非当前cache/流水优化实测**；不包括未来CAN IP和PS平台 |

K26公开容量和来源见TRANSPORT_NOTES。估计不证明放得下或100MHz闭合，更不证明驱动器接受该频率。阶段D必须以实际report_utilization/report_timing替换；超预算不能隐去。

## 11. 阶段D实现验收清单

按 `regmap.yaml` 实现地址/权限、AXI写防火墙和单口controller+TDP；RTL对照两个整数golden；验证CRC、半写/重放/过期/背压/每拍watchdog/接缝/制动尾/量化/复位。仿真覆盖所有bank与FSM边，断使能请求不经过TX目标通路。实际运行Vivado之前所有时序/资源均只能称设计目标。现有USB-CAN只可作为可变时延消费者，不替代PL物理CAN出帧证据。此清单不包含已删除A的补测/采购待办。

## 实际构建时钟修订（2026-11-02）

本地KR260 preset的参考时钟为33.333MHz，请求PL0为100MHz，Vivado派生标称值实际为99,999,001Hz。保持预设参考与PLL配置，使用该派生值设置RTL `CLOCK_HZ`、AXI/BRAM接口元数据、DDS阈值及秒导数积分常数；不得仅修改界面标签或伪造更精确参考频率。100MHz仍是参考仿真的默认profile，二者不得混用bank。RO `NOMINAL_CLOCK_HZ` 位于0xa8/ac，加载器核对它和生成的addresses.json；不是测得振荡器频率或f_exec。编译参数范围1..2147483647，所需192bit位宽按参数上界验证。

## 后台校验时延修订

资源精简后，HCHECK与每个RCHECK以共享130周期128/64无符号整数ceil除法做两次全局tick映射，仍检查uint64 origin相加溢出。相对组合版，每个header/record额外265周期，4记录块额外1325、255记录块额外67840；不改运行侧112周期、不刷新来源时间、不延长TTL/lease。256周期最短tick预算不代表任意bank组织下持续供块吞吐保证；后台不能及时准备时沿用原STARVE/停止规则。


## 后台gate等待时延修订

validator保留原0..15枚举值，仅以5bit状态追加GSTART/GWAIT。math done且无溢出后启动本validator的流水gate，冻结record、raw limits/epoch与chain_wire，待gate done后才进入原GCHECK更新链。每record较原组合门额外17clock；它与共享ceil时间映射带来的每header/record额外265clock独立相加（header不加gate等待）。运行侧112拍、DDS origin、来源时间、TTL及lease均不因此延长。后台来不及READY时仍走原STARVE/停止尾策略。

## 影子寄存器存储修订

外部寄存器表与时序不变，物理RW_SHADOW存储按130个合法word密排；旧1,024word只读视图中894个洞恒零。shadow_slot非法返回−1并检查上下界，不别名slot0。CRC静态131word流含28个只读零reserved，保持原序列；配置复制/原子性、WSTRB及配置后静态拒写均保持。[最新top验证](../docs/stage_d_sparse_shadow_20261102/verification.json)通过1,004,686项，涵盖真实AXI130合法与893非法aligned地址、字节mask/无别名以及原112拍流程。该规模推导不等于FF/LUT实测，尚无完整BD时序签核。

## 15. 当前共享计算与静止seed（2026-11-02）

`shared_math_gate`在math的E0原子锁存所获客户端的raw、limits、previous-wire及cache-valid；默认0还锁存五组cache，顶层专用1使用PL全局cache的稳定视图。整job独占math及gate，完整稳定输入合同见§20。运行请求直接接受、不排队，保持外部tick+4启动math、+83启动gate、+98完成、+99置ready、+112提交。各客户端done只回给当前owner，结果分别保存；后台结果不会改运行输出。后台MSTART/GSTART请求保持到grant，math E78、gate E80接受/E95完成、E96消费，真实top独立观测整job小于100拍。

后台只在`configured && dds_quiet && !dds_tick && !pipe_active`时取得首次math grant。DDS原phase不改，用unsigned128的`phase+128*num<T`保证未来128次DDS步均不跨阈值；已注册tick另行排除，运行窗口保留至112拍。quiet只约束首次grant，不能阻止同一owner随后gate继续。取消屏蔽事件并排空当前子核，DISARM同步清共享状态；碰撞/协议超时锁存FAULT，不延长来源有效期。等待中的bank仍受原FINISH当前时间检查。

`stationary_seed_gate`是静止编码的组合参考，只覆盖原STATIONARY调用中固定q0=q1、其他运动状态零、dt=1和无previous-wire的情况。当前顶层改用下节的顺序服务；其他段仍须经过完整math及gate，不得借静止门放行运行轨迹。

功能证据见[共享顶层](../docs/stage_d_shared_execution_20261102/top/verification.json)、[seed](../docs/stage_d_shared_execution_20261102/seed/verification.json)；当前共享版资源与最终STA以REPORT最新记录为准。此前隔离核心默认opt实测208,920 LUT已超过K26容量；这已推翻C初始估计，不能用35k～90k的初始表宣称放得下。

## 16. 启动检查分拍与运行结果保存（2026-11-02）

顶层只实例化一个 `stationary_seed_gate_seq`，供 CONFIG、首次 COMMIT 和 ARM 互斥使用。服务 E0 锁存七轴完整 raw64 输入，E1..35 每轴五拍完成原两级保守量化和检查，E36 原子发布结果。`bounds_valid` 独立于初始位置和导数是否合法；失败的完整编码不发布位置或 allow。复位、DISARM 和硬故障取消服务；忙时不接受新 start。

证书绑定当前位置、配置 epoch 和 session nonce，每次新任务先使旧证书失效。CONFIG 只消费本次 bounds 证书，仍等待原131字 CRC，原 DDS 提交边沿不变。首次 COMMIT 保持原写入排空和两拍 settle，取得新证书才启动 validator；后继 COMMIT 使用已确认的父块，父块在等待中退休则拒绝。ARM 同时逐字计算原18字 seed CRC，并在最终边沿重新检查原来源时效、lease、首 tick 和 bank 条件。以顶层接受命令为 P0，正常 P1 快照、P37 发布、P38 保存证书、P39 响应；首次 COMMIT 的额外写入等待另计。64拍内部看门狗只使缺失结果被拒绝，不给来源或 lease 增加宽限。

数学单元提前一拍寄存除法控制和完整192位正分母；当前分拍乘积版在E10/32/54预译码，q/v/a实际除法分别在E11..32、E33..54、E55..76，E77舍入、E78完成，详见§20。共享单元新增 `rt_q1_latched/rt_v1_latched/rt_a1_latched`，直接读取既有运行结果寄存器；顶层第112拍的位置提交及制动末端导数判据使用该视图。原 done/data 接口、gate 和 validator 不变，没有新增结果寄存级或运行周期。该视图必须结合本次任务的 math/gate ready、取消及失败检查使用，不能把旧保存值当作新任务完成。

真实功能验证见[顺序单元](../docs/stage_d_capacity_reduction_20261102/sequential_seed_validation/verification.json)、[启动协议](../docs/stage_d_capacity_reduction_20261102/sequential_seed_top_validation/verification.json)、[除法控制](../docs/stage_d_capacity_reduction_20261102/registered_division_decode_validation/verification.json)、[运行结果视图](../docs/stage_d_capacity_reduction_20261102/latched_runtime_validation/verification.json)和[100MHz名义参数回归](../docs/stage_d_capacity_reduction_20261102/sequential_adopted_nominal_validation/verification.json)。它们不代替完整工程布线后时序或上板验收；物理输出仍为 DESC_ONLY。

## 17. 后台时间映射乘积寄存（2026-11-02）

当前`bank_validator`为026560版本，保留原状态0..17的枚举值，追加`THRESHOLD_PREP=18`与`DIV_PRODUCT=19`。接受bank任务的E0仍快照原参数；下一拍独立寄存完整unsigned128的`T = den_reg * CLOCK_HZ`，再开始读header。HCHECK仍拒绝`T[127:64] != 0`，不截断合法性判据。每次全局tick映射先用新增状态寄存完整unsigned128的`index64 * T[63:0]`，随后才由原DIV_REQ启动原130周期128/64 ceil除法；上一索引和当前索引仍分别计算，origin相加溢出等原检查不变。两个乘法不再同拍级联进入divider寄存器，没有改舍入、位宽或时刻定义。

对总record数N（normal与brake合计），一次启动准备加每个header/record两次乘积准备，较前版新增`1 + 2*(N+1)`拍；N=4新增11拍，N=255新增513拍。这与前述共享ceil除法每header/record的265拍及gate等待独立相加，不能替换或重复合并。MSTART/gate共享等待仍按原规则处理，因此这些数字只表示本次固定增量，不承诺任意输入下的后台完成总时长。HCHECK、RCHECK、FINISH的原判据保持，FINISH仍以消费时的当前时间检查；新增等待可能使本来接近截止的bank被拒绝，不延长source、TTL或lease。运行侧math/gate/描述符的112拍不变，DDS origin/phase不变。

99,999,001Hz数值profile的[真实功能验证](../docs/stage_d_capacity_reduction_20261102/validator_mapping_pipeline_validation/verification.json)包括10个定向job、17,853项检查，覆盖完整128位最高位、新状态18/19复位及新增等待后TIMESTAMP/DDS拒绝码6；组合顶层实际1,593,148项检查，52个后台job中4个并行、1,394拍等待，原112拍断言保持。采用时仅本validator RTL相对第21轮改变，math/shared/seed/gate的数据运算与各自既有证据不变。

[隔离综合](../docs/stage_d_capacity_reduction_20261102/validator_mapping_pipeline_synthesis/verification.json)采用K26、RuntimeOptimized、EXTERNAL_EXECUTION=1与10ns OOC时钟，估计WNS +2.239ns、WHS +0.109ns、脉宽余量 +4.725ns；接口延迟未约束、未布局，且本次没有时钟中和/DCP重开，不能当完整平台STA或生产child验收。第21轮完整综合通过但placement由本任务主动停止，详见[综合证据](../docs/stage_d_capacity_reduction_20261102/managed21_full_synthesis/verification.json)和[停止记录](../docs/stage_d_capacity_reduction_20261102/managed21_stopped_placement/verification.json)。该节采用时新第22轮完整结果尚未取得；最终PS/RAM同钟、setup/hold/脉宽、未约束路径和DRC签核要求均不改变。


## 18. 配置cache的精确拆分乘法流水（2026-11-02）

当前`limits_product_cache`为7cd18版本，端口、五组35项整数乘积及raw合法域不变。每项原signed64输入`x`与完整signed98系数`k`按下式计算；系数仍为QDEN、VDEN或C，不缩窄、不预舍入：

```text
lo = signed33({1'b0, x[31:0]})
hi = signed32(x[63:32])
low_product  = signed131(lo * k)
high_product = signed130(hi * k)
product162 = sext162(low_product) + (sext162(high_product) << 32)
```

`x = lo + (hi << 32)`覆盖全部signed64，包括INT64_MIN。必须先把high_product符号扩展到完整162位再移32位，不能在130位寄存器内先移位。两路partial保存全部131/130位，重组保留全部162位，发布时才符号扩展为192位；没有中间RNE、截断或饱和。原raw64守卫、负数上取整及int32编码域、正限值检查全部保留。

| 以cache接受start为E0 | 当前工作 |
|---|---|
| E0 | 锁存全部raw配置/epoch，清两级valid及旧公开结果 |
| E1..E35 | 寄存完整signed64输入、signed98系数及group/axis标签 |
| E2..E36 | 按前拍输入/标签寄存完整low/high partial |
| E3..E37 | 按前拍partial标签重组完整162位并写私有scratch |
| E38 | 两级valid都已排空，原子发布五组7×signed192、epoch及done |

最后一项遵循E35输入→E36 partial→E37 scratch→E38发布；不能在E36或E37提前读完scratch。group/axis标签随两级valid传递，复位或新E0先清valid，旧job的数据不能被新job消费。无效请求也在E38完成并清零公开结果。每个busy边沿（含E38）仍永久记录epoch变化/未知，最终再次核对当前epoch；回到旧epoch不能复活cache。忙时start与外部raw扰动不重采样。

相较原E36缓存，当前只增加2拍CONFIG初始化等待；`C_ACTIVATE`继续等done且epoch匹配。原`tick_commit`同时启动DDS和cache，DDS origin/phase不移到完成或ACK拍，配置CRC及寄存器布局不变。来源时间、TTL、lease不刷新，运行math/gate及描述符固定112拍不变。

本轮实际cache定向验证覆盖五个CLOCK_HZ数值profile，每个165个job、39个复位位置、114个busy epoch异常、10个raw X/Z与2个E0 epoch X/Z；组合顶层实际1,593,194项检查，52个后台job中4个并行、1,378拍共享等待，原外部112拍断言保持。这里只更新已采用版本的功能排程与证据口径；未改模块按各自源码哈希沿用既有证据，不声称14套all在本轮全部重跑。独立综合的正余量也不能替代完整平台布局布线、未约束路径及DRC签核，最终结果仍见docs/REPORT.md。


## 19. 历史：第24轮CONFIG乘积、共享cache与原地预取（2026-11-02）

以下保存第24轮当时的top05bf/shared02bd/mathc335/TBb5a来源合同；本节的旧gate a5及E10首轮排程不代表当前版本。CONFIG乘积和原地预取保持不变；当前源码tuple、gate稳定输入及E11首轮排程见§20。

CONFIG接受沿P0冻结完整num64、den64及nonce/epoch/期望CRC标记，先清旧fresh；P1计算并寄存完整96位`T=den*CLOCK_HZ`与`MG=(num<<32)-num`。两式均先扩展到96位，不能只保留T低64位。原131字CRC同时继续；原C_CHECK只消费fresh且全部输入标记仍一致的本次结果，并保留T高位、最短/最长间隔及原全部静态判据。C_CHECK消费或拒绝、复位、DISARM与故障都清prepare/fresh，旧任务不得复活。这项流水没有增加CRC等待、移动原DDS提交/origin或延长来源/lease；E38配置cache的等待另按§18。

`shared_math_gate.STABLE_CACHE_INPUTS`默认0：每job在E0快照两客户端选定的五组乘积。仅`robot_rt_top`设为1，前提是两客户端乘积端口逐位连接同一份PL生成的全局cache，且该cache在整个owner job中不变；使用rt端口作为同一份数据的规范视图，不接受PS提供的许可或乘积替身。raw limits、previous-wire及cache_valid仍按job快照，epoch、全宽高位守卫、取消排空和第112拍最终检查不变。只省共享层冗余的一份乘积存储；原`limit_gate_pipe` a5b8版本仍在自己的E0快照完整输入，端口、stub与child合同均未修改。默认0的通用服务仍支持客户端cache不同或外部扰动，不能把顶层专用合同推广为通用免快照模式。

原地预取只在`cur_valid=0`时逐字写`cur_record`，完整第63字与bank/index一起原子发布；下一record发布前，已发起的math只读取此前锁存的pipe上下文。tick不能读取填充中的记录，也不能在第63字同沿借用新valid；MISS、DISARM、硬故障及复位仍撤销预取/提交许可，不等待补齐，不改变112拍提交或源时效。

数学分母寄存与原预译码共用边沿：E9准备Q_DEN供E10首q轮，E31当拍仍用原Q_DEN完成末q轮，同时以NBA准备V_DEN供E32；E53同理完成末v轮并准备C供E54。完整192位分母不截断，也不改变原66位商、96位余数、RNE、int64拒绝或E77/E78。复位/空闲使控制无效，忙时start仍不重入。该寄存只移除迭代组合路径内的kind→分母选择，不增加运行边沿。

[密度修复功能验证](../docs/stage_d_capacity_reduction_20261102/density_repair_validation/verification.json)保存默认0共享服务及顶层专用1的实际HDL，顶层1,593,194项、62个cache生命周期和5,928个稳定边沿；旧编译失败记录保留原原因。[完整分母寄存功能验证](../docs/stage_d_capacity_reduction_20261102/divisor_register_validation/verification.json)保存99,999,001与最大正int32 CLOCK_HZ各4,112个数学向量/E78，以及shared和同一顶层112拍检查。本轮没有重跑14套all，也未据这些功能结果宣称物理FF/LUT减少或布线收敛；c335独立OOC的RuntimeOptimized与Default两profile均已完成结构检查、时钟中和和DCP真实重开：Runtime为34,626 LUT /25,323 FF /198 DSP、WNS +0.426ns、WHS +0.106ns；Default为37,443 LUT /24,901 FF /198 DSP、WNS +1.985ns。各gate复用自身directive下原a5已验证工件，不是本轮重综合，见[双profile实际OOC](../docs/stage_d_capacity_reduction_20261102/divisor_children_synthesis/verification.json)。第24轮采用Runtime；本节保存的是当时隔离综合证据，不是当前新tuple的结果。上述隔离估计不能代替完整setup/hold、未约束路径、DRC或DESC_ONLY边界检查。


## 20. 当前：完整partial乘积与稳定job输入（2026-11-02）

当前精确安装的tuple为top `05bf08b660621772e7a545e10f6400d6566a6171a6e5bb87353763eb1a0cadcd`、math `0b943a9a1a9f0dc3de9dd97ac7d7f8459075148489649a1ac6535a7866585cf3`、gate `43b7a8591aa2269a2c3bad3895fd4bd5e5cc9c45259b34f04489af3da8b189ac`、shared `d2c9e6d061799a478e2a9d08b2505f72a0512a651d2120f00f876d298a5157de`及monitor TB `5b0d5717276432b73f9f2b07eec8451f2bc8e9c2ec51a4a8c44a041180d201d0`。寄存器/描述符协议仍为v1；下述child工件参数契约单独升级为v2。

每轴原完整signed64×signed98状态乘积现在按§18相同恒等式拆分：低32无符号补0为signed33，乘积完整signed131；高32为signed32，乘积完整signed130。两partial先寄存，重组时分别符号扩展到162位，再将高项左移32并相加，最后扩192。不能在130位内先移位；负数及INT64_MIN均在同一全宽恒等式中，不截位、不增加RNE、不降低原溢出门槛。

| math start为E0 | 当前边沿工作 |
|---|---|
| E1..E9 / E2..E10 / E3..E11 | 锁九项操作数 / 锁完整131、130位partial / 重组并保存九项192位精确term |
| E9 / E10 / E11..E32 | q分子 / 绝对值及商余种子 / 22轮radix-8 |
| E11 / E12 / E33..E54 | v分子 / 种子 / 22轮radix-8 |
| E12 / E13 / E55..E76 | a分子 / 种子 / 22轮radix-8 |
| E10 / E32 / E54 | 提前一拍寄存完整Q_DEN、V_DEN、C和控制；E32/E54当拍仍消费旧q/v分母，NBA新值只供下一拍 |
| E77 / E78 | 原精确RNE/int64检查 / 固定done；溢出或无效请求同延迟并输出全零 |

系数在E3更新后才由E4的可变系数launch使用；qp3在E8已保存供E9求和，vp2在E10供E11，ap1在E11供E12，均不依赖同沿NBA新值。66轮除法于E76完成，E77/E78及公开九term接口不变。gate只在math完成后消费这些term；无需保证它们在math计算过程中已经稳定。

`limit_gate_pipe.STABLE_JOB_INPUTS`默认0，保留原E0完整数据快照。只有值1旁路九组精确term、五组cache、13组raw64状态/限值及previous-wire的重复快照，共21,924个声明位；各视图保留相同位宽和signed属性。值2、负数或X/Z不进入有效生成分支。E0的98项高部符号扩展/XZ、terms/cache有效及math overflow检查、bad_job和previous-valid锁存全部保留；逐轴S0数据寄存、量化/范围/DQ守卫、E14归约、E15发布、拒绝全零不变。声明位旁路不是综合FF/LUT节省测量。

顶层保持`STABLE_CACHE_INPUTS=1`，shared精确转发为gate的`STABLE_JOB_INPUTS(STABLE_CACHE_INPUTS)`。许可依赖三个生产者的不变性：shared在math E0锁raw/limits/previous及有效位并持有owner；math直到gate完成排空前不再接受新start，九term及q/v/a终值在E78后不变；两客户端接同一份PL全局cache，CONFIG/epoch不得在有效owner期间改写它。gate接受至E15的全部输入因此稳定。取消只屏蔽事件并排空旧owner，不提早重用上下文；DISARM/硬故障的公共复位使owner和子核一起失效，不能把旧allow或旧term接到新job。通用默认0仍支持外部扰动和不同客户端cache，不能无合同开启1。

child工件必须使用`robot_rt.partition.v2`与每child的精确`hdl_generics`/input_contract。math只带`CLOCK_HZ=99999001`；当前gate另带`STABLE_JOB_INPUTS=1`，通用源/stub仍默认0。生成的实际synth Command、源码/快照/DCP及parameter_evidence的SHA、复用origin均绑定该map；父prepare还须独立显式传入`--partition-gate-stable-job-inputs 1`并核对top常量和shared转发，不能从child manifest反推期望值。v1 gate或flag0 DCP不充当flag1；原a5回退只与其完整旧源码/工具/profile一起使用。此合同不放宽maxThreads1、setup/hold、未约束路径、DRC或最终发布顺序。

[已闭合功能验证](../docs/stage_d_capacity_reduction_20261102/stable_job_gate_validation/verification.json)在归档时仍标为私有未采用；其后安装了上述相同SHA，历史归档不改写。math在实际与最大正int32参数各4,112向量/E78通过；默认0的pipe738向量/reset15及domain通过，通用shared模式0也通过。当前top为1,593,194项、52个后台job/4个并发/1,378拍等待；新增稳定gate monitor覆盖76个job、1,216个稳定边沿，完整数据宽24,864bit。专用模式1生命周期测试另有20,655项、8个取消与32个复位场景、1,010个稳定和62个取消边沿；该单元测试不实现或证明外部112拍，112断言来自完整top。功能证据不等于形式化全状态证明，也没有重跑本轮14套all。

当前新tuple的真实child OOC已闭合：RuntimeOptimized、CLOCK_HZ=99999001、10ns下，math0b为37,264 LUT /26,303 FF /198 DSP，WNS +3.869ns、WHS +0.106ns、脉宽余量 +4.725ns；gate43b/flag1为18,956 LUT /9,418 FF /15 DSP，WNS +3.713ns、WHS +0.105ns、脉宽余量 +4.238ns。guard rc0/293.569秒/最低3,844,172KiB，真实双generic brace命令、时钟中和及重开检查通过。接口I/O delay未约束，这些隔离OOC结果不能替代完整容量或布线后STA。root已冻结133输入（c9151a5c…）并真实启动parent25，完整综合进行中，尚无本轮完整平台综合/布局/最终STA/bit或板端验收。旧c335/a5数值仍仅为历史；运行DESC_ONLY、来源时间、DDS origin、TTL/lease及外部112拍均不变。见[本轮闭合child OOC及参数证据](../docs/stage_d_capacity_reduction_20261102/stable_job_children_synthesis/verification.json)。
