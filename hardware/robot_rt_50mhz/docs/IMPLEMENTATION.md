# 阶段D：RTL、离线仿真与实际构建

本实现为 **DESC_ONLY**：只生成受限CAN描述符数据，没有PL CAN收发器，也不连接现有USB-CAN。RTL把 `disable_req` 固定为1、`motion_permit` 固定为0；即使仿真中的ARM成功，也不代表驱动器被使能。本任务没有修改旧ACT入口、模型或现有运行保护。

## 文件与规格对应

| 规格模块 | 实现 |
|---|---|
| tick_gen | `rtl/tick_gen.sv`，精确有理DDS、参数化CLOCK_HZ计数和全局origin |
| chunk_ram | `rtl/robot_rt_axi_guard.sv` 保护PS写口；`bank_validator.sv` 冻结块校验；top仲裁PL只读B口与预取 |
| player | `rtl/player_math.sv`，七轴Q32.32三阶积分，每轴TDM复用两路完整partial乘法及一个radix-8除法器；轴内q/v/a分时、七轴并行、精确RNE/E78；top按tick驱动 |
| 配置乘积缓存 | `rtl/limits_product_cache.sv`，完整64×98拆分为low131/high130两路寄存partial，精确162位重组，35项在E38原子发布 |
| limit_gate | `rtl/limit_gate_pipe.sv`复用math精确项，15拍按轴完成包络/编码，默认0快照、top专用1稳定job视图；`stationary_seed_gate_seq.sv`供启动检查，`stationary_seed_gate.sv`与`limit_gate.sv`保留组合golden |
| 共享执行 | `rtl/shared_math_gate.sv`在运行与后台之间调度一套math及gate，锁存上下文、隔离结果并处理取消 |
| safety_fsm | `rtl/robot_rt_top.sv`，来源/lease/序号/状态/停止尾、故障锁存与输出提交 |
| tx_backend | top中的96B打包器，只接门后目标，ready失败拒绝，不接物理CAN |
| stats | top中的原子SNAPSHOT，计数器及描述符快照只读 |
| Vivado与加载 | `tcl/create_bd.tcl`、`build.tcl`、`constraints/robot_rt.xdc`、`sw/load.sh` |

所有RTL、testbench和工具在本目录从零编写，没有复制旧部署源码；旧SDK只用于核对CAN字节契约。`sw/pl_bank.py` 仅打包离线输入、配置CRC、原生段和停止尾，不读设备、不写CMD/ARM，没有实时相机/时钟映射/ACT接管后端。

## 可复现命令

在robot_rt根目录运行，输出目录须不存在：

```bash
/usr/bin/python3 -m unittest discover -s tests -p 'test_pl_bank.py' -v
/usr/bin/python3 sim/run_hdl.py --suite all --clock-hz 99999001 --output-dir /tmp/robot_rt_hdl_new
/usr/bin/python3 sw/pl_bank.py --design-example --clock-hz 99999001 --f-num 275 --f-den 2 --origin-cycle 0 --first-index 8 --output-dir /tmp/robot_rt_bank_new
tclsh tcl/create_bd.tcl --check-inputs
tclsh tcl/build.tcl --check-inputs
bash sw/load.sh --help
```

runner使用已安装2025.2的xvlog/xelab/xsim，不安装工具、不跑综合、不连接板卡。除退出码外还检查Fatal/Error和PASS，避免xsim在`$fatal`后返回0造成假通过。当前`SUITES`字典实际为14套：ceil、quiet、tick、guard、math、gate、cache、pipe、domain、bounds、seed、seed_seq、shared、top；这里只读计数字典，没有重跑all。

E38 cache的五个CLOCK_HZ数值profile各通过165个job、39个复位位置、114个busy epoch异常、10个raw X/Z及2个E0 epoch X/Z，独立核对E38原子发布。最新采用top仍为1,593,194项，52个后台job中4个并行、1,378拍共享等待，另新增62个cache生命周期/5,928个稳定边沿；外部112拍断言保持。当前0b943数学单元在实际参数和最大正int32参数各通过4,112向量/E78，并与新stable-job gate/shared/top同批复核；top新增76个gate job/1,216个稳定边沿，见文末当前章节。未改的组合gate、流水gate等按相应源码哈希沿用既有证据。历史[按轴gate验证](stage_d_axis_gate_20261102/verification.json)的pipe为738向量、旧cache为38个job/E36；[九套历史验证](stage_d_compact_arithmetic_20261102/verification.json)和更早E8 gate记录保留原口径，不能冒充当前E38 cache或本轮14套all全量重跑。

## 对C规格的实现说明

实际RAM预取为每clock一个32bit word的流水读取，当前段计算期间准备下一段；首brake或successor在切换后预取。C原先提出的四条record提前缓存并非本版组织方式，必须用最短256clock间隔的顶层仿真验证无miss，不能只凭慢频率例子推断。后台validator可以逐word握手慢读，优先级低于运行预取；未完整CRC/包络/链验证不会READY。内存受限构建后改为单共享128/64精确ceil除法器，130周期，每header/record的两次时间映射比原组合实现多265周期。停止/来源TTL及lease没有延长；最短256周期是运行/预取预算，不保证每种bank长度都能持续后台供块。

新增只读 `0x600..0x65f` 描述符快照，只有SNAPSHOT才与计数器一起锁存，不能当TX写入口或实时消费者FIFO。D寄存器表包括该24word扩展；C的90寄存器原始规格与25项参考证据由提交 `be8ac9b` 保存。

积分模块固定78clock计算；运行及validator使用15clock按轴流水gate，复用math已算的九组未RNE精确项，不重复构造宽多项式乘法。启动seed使用顺序stationary_seed_gate_seq，组合stationary_seed_gate保留golden；运行与validator时分共用一套math和gate。顶层仍在112clock提交；当前配置乘积缓存E38、math E78、gate E15和顶层112都只是已核对的功能周期，不能证明组合级在目标10ns收敛。实际Vivado构建正在推进，最终利用率/STA以REPORT记录为准；目前尚无完成签核的报告，C资源表仍只是初始估计；构建脚本对setup、hold、未约束路径及DRC不通过时停止，不生成可部署成功清单。

PS规划必须使用相同整数积分/实际DDS间隔与量化门，且提供精确零速零加速度停止尾。现有B浮点录制回放没有被改成硬件成功证据。完整的实时ACT→PL feeder、主机到PL时钟相关映射、物理CAN时刻、反馈闭环与任务成功率尚未验证。

## 人工Vivado与板端步骤

按 [BOARD_SMOKE.md](BOARD_SMOKE.md) 运行。本轮继续指令后已实际运行本机Vivado：BD校验及生成通过，历史完整综合及部分布局已有记录；当前新tuple尚无完整route/bootgen签核，未加载bitstream。加载也不能初始化PS；目标Linux必须匹配生成XSA的HPM0、与addresses.json一致的派生FCLK和复位配置。

阶段E需要实际PS/PL同频数据；当前没有这类硬件数据，不能生成“实测消抖”表或成功率。没有重新加入阶段A频扫、IMU或采购传感器待办。现有USB-CAN的发送时间仍由Linux/USB/总线决定。

## 历史：合成资源精简（2026-11-02）

原384位乘积高半溢出检测在合法signed64状态、uint32段长及正int32 CLOCK_HZ下恒为假：全参数绝对和上界q/v/a仅需163/130/97个signed bit。保留192位精确乘积/累加，系数改98/98/97/65/65/33位signed；末端int64溢出、invalid_dt、商余数及RNE不改。数学证明与重新仿真记录在stage_d_resource_optimization_20261102。真实/最大clock的math各528向量通过，顶层41887项通过；不能凭这些功能结果宣称已缩减到某一DSP数量或已过STA。

## 当前TDM状态乘法排程（2026-11-02）

每个player_math实例保留7轴并行，轴内依次处理九项状态乘积。当前0b943每轴复用低signed33×signed98和高signed32×signed98两路表达式，先寄存全部131/130位partial，再分别扩162、高项移32相加，完整162位结果扩192；没有中间舍入、截断或饱和。该恒等式覆盖signed64负数及INT64_MIN。表达式个数不等于实际DSP数量，也不证明面积或时序收益。当前validator和运行player经shared共用此实例，原两独立实例为历史组织。

| math接受start为E0 | 当前工作 |
|---|---|
| E0 | 锁存七轴q/v/a/j及d |
| E1..E9 | 寄存九项操作数/系数；E3系数更新后由E4可变系数launch使用 |
| E2..E10 / E3..E11 | 捕获完整131/130位partial / 先扩162重组、再扩192保存九term |
| E9/10、E11/12、E12/13 | 分别完成q/v/a分子及绝对值/商余种子 |
| E10、E32、E54 | 提前一沿准备完整192位分母与控制；E32/E54仍消费旧商的末轮，NBA新分母只供下一拍 |
| E11..32、E33..54、E55..76 | 每轴同一除法器分别求q/v/a，每商22轮radix-8，七轴并行 |
| E77 / E78 | 原精确RNE及int64范围检查 / done；无效输入同延迟且输出全零 |

qp3 E8→qsum E9、vp2 E10→vsum E11、ap1 E11→asum E12均读取前拍term；最后a轮E76先于E77 RNE。九term只在math完成后供gate读取，未新增外部运行边沿。

[历史TDM功能仿真归档](stage_d_tdm_20261102/verification.json)保存当时三套实际PASS：99,999,001Hz参数math 528向量、顶层41,887项检查，以及2,147,483,647参数math 528向量。源哈希与归档时源码一致；只复制命令日志和向量metadata，没有复制大型golden或编译缓存。顶层仍用外部边沿核对112拍，math独立核对78拍。最大正int32只是整数参数边界测试，不是GHz运行频率证据。

上述TDM归档为此前快照。attempt08与09均因主机内存保护中止，没有形成器件容量或STA判定；attempt10预检启动问题已修复，attempt11因本机内存限制中止，尚无综合/实现PASS。功能PASS不能代替综合利用率、布线后setup/hold或真实RAM时钟报告，也不改变未上板、未接物理CAN的状态。

## 历史：配置cache与8拍gate集成（2026-11-02）

以下为当时E8 gate/E36 cache记录；当前gate为E15、cache为E38，分别见后文当前章节。

`player_math`新增九组只读packed精确乘积：qp0..3、vp0..2、ap0..1，各为7×signed192，轴0占低位。这些值没有提前RNE，也不是PS可写证书。`limit_gate_pipe`接受同一math job的初态、段长、RNE端点、上一实际wire、raw limits及缓存，E0整体快照。它以3QDEN/2VDEN/C公共分母和整数shift/add重构Bernstein与相对DQ，不执行除以2或3；Q16→CAN逐级RNE、最多1LSB内收、真实前后wire范围/DQ及拒绝全零语义保留。

`limits_product_cache`锁存PL已校验的raw lower/upper/dq/vmax/amax及epoch，单个signed64×signed98表达式依次算每轴五项：lower×QDEN、upper×QDEN、dq×QDEN、vmax×VDEN、amax×C，共35项。E1..E35写私有scratch，E36一次发布五组7×signed192及cached_cfg_epoch；无早期cache_valid。raw输入在job内冻结，epoch变化/无效会拒绝或作废，ARM、bank接纳及运行提交均核对当前epoch。不能以PS给出的乘积或单一valid位替代这条PL计算路径。

配置raw锁存后的原 `tick_commit` 同时启动DDS和cache，DDS origin仍在原拍锚定，不移动到缓存完成/AXI ACK。`C_ACTIVATE`等待cache完成再回复配置成功，较此前ACK多37clock；来源截止、TTL、lease和DDS相位均不延长。

| 外部tick为0 | 当前实际功能时序 |
|---|---|
| +4 / +82 | math接受start / 固定E78 done |
| +83 | 流水gate接受同一job的快照 |
| +91 / +92 | gate固定E8 done / top锁存gate_ready |
| +112 | 核对本job的math/gate ready、allow、cache epoch、故障与backend_ready后提交；否则拒绝 |

每个新job清 `pipe_gate_ready`，不把上一job的旧allow当成当前许可。fault、DISARM及来源/停止时限仍由原优先级处理，不能为等待gate推迟第112拍。validator原状态编码0..15不变，扩成5bit枚举后仅追加GSTART/GWAIT；math后等待自己的gate done再进入原GCHECK，期间record/raw配置/上一wire保持冻结。每record较此前额外10clock，与共享ceil除法带来的每header/record额外265clock独立相加，不刷新时间或保证任意bank组织的供块吞吐。

seed仅使用 `STATIONARY=1` 的组合门：要求v0/a0/j/v1/a1全零、q1=q0且位置合法；保留正限值、端点、Q16、CAN及实际wire/DQ门，不重新计算整套hull宽乘法。该专用参数不能用于任意运行段。

[本轮功能归档](stage_d_pipeline_20261102/verification.json)记录CLOCK_HZ=99,999,001的八套实际HDL PASS，top为42,349项；最大正int32参数下math 528向量、pipe 738向量、cache 38 job均通过。最大参数是整数范围测试，不是GHz运行能力。未据此填写DSP/LUT节省数字或宣称STA通过；attempt10预检状态问题已修复，attempt11因本机内存限制中止，精简算术版本的attempt12同样因本机内存保护中止；分阶段小工程验证通过，第13轮因新进程未继承线程参数而启用7进程并触发内存保护；现已修正，第15轮实际worker确认最大1进程，尚无最终STA。

## 历史：精确缩窄与64轮radix-8选择（2026-11-02）

正int32 CLOCK_HZ下三个分母均小于2^96。除法余数保存96位；每轮99位trial与1D..7D并行比较，选择最大合法商位并只减一次，64轮和最终RNE保持。保护门原生signed162乘积快照必须通过192位高部符号扩展检查，98项任一不符则拒绝；其余寄存器按完整raw64域缩窄，算术前显式扩回192，防止差值/移位在窄域回绕。两套gate声明数组各少20860位，不代表综合FF节省数量。

前版[九套HDL及边界验证](stage_d_compact_arithmetic_20261102/verification.json)全部通过，包括四组flags全有效、raw记录由旧gate判合法的高位损坏检查。math仍78拍、gate仍8拍、top仍112拍。actual profile及最大正int32 profile均只是仿真参数，不构成物理频率或STA结论。

## 当前：十五拍按轴保护门（2026-11-02）

旧七轴并行gate的实际独立综合用量为83,460 LUT、48,615 FF、15 DSP。随后七轴改为共用一条算术流水线：默认0在E0快照全部job，当前top专用1在同job稳定合同下使用直接视图；E1..E7逐轴注入；每轴经过六级相同精确检查，轴6在E13写回；E14归约，E15统一发布。端口、原整数计算/舍入/限值、高位符号扩展检查和失败全零保持。不同轴的raw、单位、previous-wire与状态随token对齐。

[五套实际HDL验证](stage_d_axis_gate_20261102/verification.json)通过：实际参数pipe/domain/top及最大整数参数pipe/domain；pipe738向量、reset15处和2项解析定向检查，domain4 jobs/80项，top42,377项。该轮math/cache未改，沿用当时E78/E36证据；当前cache E38见下文。gate在tick+83接受、+98 done、+99 ready，外部仍在+112核全部时效/许可后提交；validator相对原组合门每record额外17拍，来源deadline与lease不延长。旧E8归档与成功DCP保留其原SHA，不冒充当前版本。

独立子模块综合及分层保存流程见[低内存构建说明](EXTERNAL_BUILD.md)；它们不等于完整BD资源、布线后STA或板卡验收。

## 当前：稀疏影子寄存器（2026-11-02）

仅130个合法RW_SHADOW地址需要物理存储。`shadow_words[0:129]`保存这些word，旧`shadow[0:1023]`保留只读wire视图，894个洞恒零。非法/非对齐地址映射−1并先检查范围，不别名到slot0；原地址、WSTRB、配置后静态拒写、CRC序列、28个reserved静态零、配置原子复制及读写握手保持，无新增总线拍。

[最新顶层实际验证](stage_d_sparse_shadow_20261102/verification.json)为1,004,686项，包含独立显式地址表驱动真实AXI的130合法、893非法aligned写、每字节及零WSTRB、全量跨寄存器不别名回读与复位，以及原完整top/112拍流程。新增HDL穷举是configured=0的aligned空间；不能称全部非对齐或配置后情况也被本次HDL穷举。静态映射模型另核0..4095全部byte、configured和16种mask。声明存储少28,608bit仍不是测得FF节省；实际完整core资源/STA以REPORT为准。此前42,377项top记录是未做此存储改动的快照。

## 历史：共享版的定向验证

以下为当时共享版的定向记录，使用其保存的math/cache/gate来源哈希；当前cache E38及最新top见下一节。新增quiet、seed、shared及当时组合top已实际运行：quiet82,070项（含弱调度反例）、seed1,406向量及98未知输入、shared21,306项（含非零v/a/j）、top1,411,416项（含最短非整数间隔时4个后台任务与运行并发）。见[REPORT](REPORT.md)及stage_d_shared_execution_20261102。资源缩减幅度须等新默认opt报告，功能周期不能代替100MHz STA。


## 当前：E38配置cache拆分乘法（2026-11-02）

当前cache源码为`7cd18fd36511ef1eb8eac2c878282d5e55bdc22c2a9095f4c797de1df8fd6005`。原E36版本仍作为历史保留。每个完整signed64输入先分为低32无符号（补0成signed33）和高32有符号，两者分别乘完整signed98系数，寄存全部signed131与signed130 partial；先符号扩展到完整162位，再将高项移32位相加。重组与原64×98逐bit相等，含负数和INT64_MIN；不缩系数、不增加RNE或截断，公开端口仍为7×signed192。

E0锁raw/epoch并清两级valid；E1..35锁输入和目的标签；E2..36锁partial及对应标签；E3..37写完整162位scratch；最后两级valid排空后只在E38统一发布五组35项、epoch与done。最后一项不能被默认输入覆盖，也不能提前发布。复位/新job使旧partial失效，忙时start不重采样；每个busy拍和最终发布拍的epoch拒绝、raw全域守卫、无效公开全零与旧epoch不复活均保持。

这比原E36 cache增加2拍初始化等待，顶层`C_ACTIVATE`已有done/epoch检查，不改其协议。DDS origin仍在原`tick_commit`边沿锚定，不改为cache完成或ACK时刻；运行侧112拍、配置CRC/寄存器布局及来源/TTL/lease门槛不变。cache五profile及组合top的实际定向计数见本文“可复现命令”后的当前说明；独立OOC或功能PASS不代表完整工程STA通过。


## 历史：第24轮CONFIG乘积流水、共享cache与完整分母寄存（2026-11-02）

以下保留第24轮的top05bf/shared02bd/mathc335/TBb5a历史合同，gate为a5。其E10首轮与gate独立快照描述不能作为当前说明；CONFIG乘积及原地预取保持不变。当前tuple、E11首轮和稳定job合同见下一节及PL_SPEC §20。

CONFIG P0冻结num64/den64及nonce、epoch、期望CRC；P1寄存完整96位`den*CLOCK_HZ`和`(num<<32)-num`。原131字CRC并行继续，C_CHECK只比较标记一致且fresh的本次结果，保留原高位/频率/nonce/其余静态判据，成功或拒绝后消费fresh。复位、DISARM和故障清prepare/fresh，旧乘积不能满足新CONFIG；原CRC长度、DDS提交/origin及E38 cache等待不变，没有新增运行延迟或TTL宽限。

共享模块新增`STABLE_CACHE_INPUTS`，默认0仍在job E0完整快照五组cache。只有top设为1：运行与validator接同一份PL cache，job全程不变、epoch有效且取消路径仍生效；共享层直接将该全局乘积送gate，raw/previous-wire/cache_valid仍逐job锁存。原a5b8 `limit_gate_pipe`自身的E0快照、全宽检查、15拍、端口/stub和child合同均不变，不能把这个顶层专用不变性假设用于通用客户端。

预取去掉独立buffer→record整块复制，在invalid的`cur_record`内原地写64字；第63字到达后才连同bank/index发布valid。tick始终消费边沿前的valid，因此第63字与tick同沿仍MISS，不能因NBA顺序放行半条record。pipe已锁存前一段上下文，故下一段填充不改其计算；故障/DISARM/复位仍取消预取与输出，112拍不等待补齐。

数学单元沿原E9/E31/E53预译码拍寄存完整192位分母。E31/E53本拍分别仍完成旧q/v的末轮，新分母只供下一拍v/a首轮；E10..31、E32..53、E54..75、E77 RNE和E78 done均保持。复位、空闲和忙时start的原行为不变，不缩分母精度、不新增结果寄存级。

[密度修复实际HDL](stage_d_capacity_reduction_20261102/density_repair_validation/verification.json)覆盖默认0共享服务及top专用1；[分母寄存实际HDL](stage_d_capacity_reduction_20261102/divisor_register_validation/verification.json)进一步在99,999,001与最大正int32参数各核4,112向量/E78，并通过shared和top外部112拍检查。top生命周期monitor核62个cache job、5,928个稳定边沿；这是已测轨迹上的功能证据，不是形式化全状态证明或STA。未改模块沿用对应SHA的证据，没有本轮14套all全量重跑；c335独立OOC的RuntimeOptimized及Default均已完成结构/时钟中和/真实重开：分别为34,626 LUT /25,323 FF /198 DSP、WNS +0.426ns /WHS +0.106ns，以及37,443 LUT /24,901 FF /198 DSP、WNS +1.985ns。gate各自复用同directive下原a5验证工件，未在该轮重综合，见[双profile实际OOC](stage_d_capacity_reduction_20261102/divisor_children_synthesis/verification.json)。第24轮采用Runtime；以上是旧tuple的隔离OOC结果，不代表当前版本或完整平台签核，没有板卡或物理CAN验收。


## 当前：分拍完整乘积、stable-job gate与v2 child合同（2026-11-02）

当前已安装top `05bf08b660621772e7a545e10f6400d6566a6171a6e5bb87353763eb1a0cadcd`、math `0b943a9a1a9f0dc3de9dd97ac7d7f8459075148489649a1ac6535a7866585cf3`、gate `43b7a8591aa2269a2c3bad3895fd4bd5e5cc9c45259b34f04489af3da8b189ac`、shared `d2c9e6d061799a478e2a9d08b2505f72a0512a651d2120f00f876d298a5157de`及TB `5b0d5717276432b73f9f2b07eec8451f2bc8e9c2ec51a4a8c44a041180d201d0`。math精确partial和E11/33/55首轮排程见本文当前TDM表；E77 RNE、E78 done、gate E15及运行外部112拍保持。

`STABLE_JOB_INPUTS=0`仍是gate通用默认：原E0数据快照保留，busy外部输入扰动不影响本job。顶层shared的`STABLE_CACHE_INPUTS=1`现精确转发到gate同名专用模式，gate使用同位宽直接视图，旁路九term10,206位、五cache5,670位、13组raw状态/限值5,824位及previous-wire224位，共21,924个重复声明位；不是已测得21,924 FF节省。模式1只允许完整job不变：shared E0冻结raw/limits/previous/有效位并持有owner，math E78后直到gate排空无新start、term/终值不变，PL全局cache在owner期间同epoch不改写。取消屏蔽结果但不提前复用owner；公共复位同时中止shared/math/gate，旧许可不能进入新job。

两模式均保留E0全部98项高位/XZ检查及terms/cache/math有效判据、bad_job/previous-valid锁存、逐轴S0寄存、完整包络与两级RNE/1LSB内收/DQ门、E14归约/E15原子发布及拒绝全零。非法2/负数/X/Z参数不进入有效生成分支。不能把top不变性推广为任意客户端的免快照许可。

[当前SHA的已闭合功能验证](stage_d_capacity_reduction_20261102/stable_job_gate_validation/verification.json)包含实际与最大正int32参数math各4,112向量/E78、模式0通用shared、默认0 pipe738向量/reset15及domain；组合golden gate的722例另列，不充当流水模式0测试。top为1,593,194项、52后台job/4并发/1,378拍等待，新增monitor实核76个gate job/1,216个稳定边沿、完整输入24,864bit。专用模式1生命周期另通过20,655项、8个取消与32个复位场景、1,010稳定和62取消边沿；该单元不测外部112，112由完整top独立断言。归档保留采集时“private_not_adopted”原记录；随后采用相同SHA，不把历史metadata重写成新工具结果。

新工具只接受参数绑定的`robot_rt.partition.v2`：math的map仅`CLOCK_HZ=99999001`，gate另有`STABLE_JOB_INPUTS=1`，源/stub通用默认仍0。实际synth Command、源/快照/DCP、parameter_evidence及reuse origin的SHA必须吻合；父prepare独立传`--partition-gate-stable-job-inputs 1`并核top常量/shared转发，不从子manifest反推期望。原v1/flag0 gate DCP不能替代flag1；原a5回退需配套完整旧源码/工具/profile。原maxThreads1、最终setup/hold/check_timing/DRC/发布顺序不变。

新tuple的真实RuntimeOptimized child OOC已闭合：math0b为37,264 LUT /26,303 FF /198 DSP，10ns WNS +3.869ns /WHS +0.106ns /脉宽余量 +4.725ns；gate43b/flag1为18,956 LUT /9,418 FF /15 DSP，WNS +3.713ns /WHS +0.105ns /脉宽余量 +4.238ns。guard rc0、293.569秒、最低3,844,172KiB；真实双generic brace命令及中和/重开检查通过，接口I/O delay仍未约束。root已通过生成器准备133输入（c9151a5c…），parent25完整综合在运行，尚无本轮完整平台综合、布局、最终STA/bit或板端验收。本次不是14套all全量重跑；隔离OOC不证明全平台FF/LUT收益或STA通过，旧c335/a5仍只作历史。DESC_ONLY、DDS origin、来源时间、TTL/lease及112预算不放宽。见[本轮闭合child OOC及参数证据](stage_d_capacity_reduction_20261102/stable_job_children_synthesis/verification.json)。
