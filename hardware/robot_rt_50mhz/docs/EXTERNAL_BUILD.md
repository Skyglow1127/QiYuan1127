# 低内存构建调度

原 `tcl/build.tcl` 仍是完整构建入口。新 `tcl/split_synthesis.py` 把综合拆成“生成脚本并退出controller → 完整runme.sh → 新controller核验”，避免父Vivado与综合worker同时常驻。两种方式调用相同OOC约束、综合报告、route签核及导出函数，没有放宽时序或保护。

## 当前第27轮：原生POST容量门拒绝，构建已停止

第27轮已被真实容量门拒绝，后台已停止。root于07:07:44核实guard932829已在04:24:23闭合，rc1、921.812秒、最低可用内存10,000,160KiB。实际place完成，phys_opt_design算法完成后POST真实执行，因CLB 14,611/14,640（99.80%）超过95%阈值拒绝；physopt.error已出现，route未启动。旧26为14,422/14,640（98.51%），本轮反多189个CLB；综合LUT/FF/DSP减少没有转化为CLB打包改善。没有最终STA、bit/XSA或上板验收；后续先诊断实际packing，再决定下一候选。

### 第27轮此前阶段观察（截至04:12:25；保留当时记录，当前状态以上文为准）

第27轮最初选择并安装math97f6bc1d、容量driver88365fe7及新增hook5d2128b8，其他19份原源文件当时逐SHA保持，变更前21份源与hook此前不存在的记录已备份。math仍保留完整精度、原192位guard、RNE/E78和top112；七套正常测试与四个非法CLOCK原time0拒绝已实际通过，见[功能证据](stage_d_capacity_reduction_20261102/threebank_frontacc_validation/verification.json)。

[闭合97f6 OOC](stage_d_capacity_reduction_20261102/threebank_frontacc_ooc_diagnostic/verification.json)的guard rc0、229.497秒、174项fresh SHA通过。同设置raw为32,223 LUT /27,546 FF /102 DSP，相对7b分别减少1,430 /5,226 /96；相对6411为−4,697 LUT /+595 FF /DSP不变。自己的defaultopt另列32,195 LUT。raw/opt WNS分别+3.869/+4.177ns，两者WHS+0.105ns、WPWS+4.725ns、setup/hold/pulse失败端点0；未约束I/O delay，不能据此认定完整STA、CLB packing或布线改善。[后续选择记录](stage_d_capacity_reduction_20261102/threebank_frontacc_ooc_diagnostic/selection/source_selection.json)与早期准备/未采用字段分开保存。

DIRECT child已闭合，新math97f6重综合、原flag1 gate按完整证据复用。第27轮原111项保护清单SHA `62c2ed1c66ca8536cebf432244475be36670c1726c0a68d4b8d5de473a9b363c`与49行冻结输入保留原字节；prepare、七个IP及TOP的9阶段和独立controller均已返回0。[整机综合证据](stage_d_capacity_reduction_20261102/managed27_full_synthesis/verification.json)为95,237 LUT /107,728 FF /184 DSP /32 BRAM，相对26同阶段少1,430 LUT、5,226 FF、96 DSP；core单列90,754 LUT /101,876 FF /184 DSP。未布局setup WNS +1.524ns/TNS0，hold WHS −0.087ns/THS −1285.353ns、55,402失败端点、no_clock8；不能当布局容量、route或最终STA通过。

初次implementation在约20秒的静态预检中返回1：旧parser不识别Vivado固定POST hook的nested catch格式，原生implementation worker和POST均未执行，没有place或route结果。整轮guard rc1、1126.380秒、最低可用内存8,508,980KiB，见[原始预检拒绝](stage_d_capacity_reduction_20261102/managed27_impl_hook_preflight_failure/verification.json)。这是脚本解析拒绝，不能写成布局、路由或内存失败；原失败记录、旧SHA和false字段保留。

root仅将helper更新为61b0c447版本，154项纯测试及ENV七个负例通过，函数外字节保持，见[修复采用与首次续跑闭合证据](stage_d_capacity_reduction_20261102/managed27_posthook_parser_fix_and_continuation/verification.json)。相对最初21份原源码，如今18份原样，math/phase_driver/split_implementation三项变更，另新增hook；旧selection的19字段保留首次采用时口径。原111项中其余110项不变，旧project的49行冻结清单不含该helper，也未被改写；首次续跑另锁142项，SHA `06265298018f3a6f3e4d08bbdf2776a53bac7fb98d9b08a1e276b8fc20524302`。随后sameout的`--phase all`续跑在17.050秒、guard rc1时被原fresh状态门拒绝：不得对已生成过脚本的run重复生成或reset缓存结果；这次没有启动native implementation。

后续[第三次显式复用续跑](stage_d_capacity_reduction_20261102/managed27_existing_scripts_continuation/verification.json)采用私有driverbb314，125项mock检查、51项负例及反向字节守恒通过；仅显式复用该27工程已生成且未执行的原脚本，核初始9文件SHA和全部冻结依赖，不reset、不改旧manifest。canonical driver883保持不变，原142项来源值全部保持，新164项清单SHA `d6ca4b50d5b92391b0f18a0ecc8e929aaa2b61e810a81c0eef07774a83426484`。root于04:12:25实际确认init与opt已结束，place已进入全局布局，guard尚未闭合；尚无布局完成、POST、route或最终STA验收，也没有bit/XSA或上板结果。[95%容量门负probe](stage_d_capacity_reduction_20261102/physopt_capacity_gate/verification.json)不替代新worker的实际POST证明，原route、严格STA/DRC及bit/export门保持；ACT部署与实物运动未由本次变更操作。

第27轮原工作目录为`/tmp/robot_rt_managed27_frontacc_flow_20261103`；首次修复续跑的闭合拒绝在`/tmp/robot_rt_managed27_posthook_fix_flow_20261103`，第三次显式复用续跑为`/tmp/robot_rt_managed27_existing_scripts_flow_20261103`。三者使用同一输出`build/kr260_20261103_attempt27_threebank_frontacc_capacity`，不重新综合HDL。第三次仍调用canonical helper61b的`--phase all`，仅以`--phase-driver`显式选择私有bb314；严格要求该27 run为Scripts Generated、0%、NEEDS_REFRESH0且9个初始文件SHA匹配、无执行产物，只返回SCRIPT_READY，无reset。DIRECT child及reuse来源保持原路径，原STA/DRC/容量门不变。

### 第25～26轮及此前私有测试历史（保留当时措辞，当前选择以上文为准）

当前阶段25的真实child OOC已全部闭合通过：guard rc0、293.569秒、最低可用内存3,844,172KiB。RuntimeOptimized下math0b为37,264 LUT /26,303 FF /198 DSP，10ns独立OOC WNS +3.869ns、WHS +0.106ns、脉宽余量 +4.725ns；gate43b/mode1为18,956 LUT /9,418 FF /15 DSP，WNS +3.713ns、WHS +0.105ns、脉宽余量 +4.238ns。真实Command的双generic brace格式已通过严格解析、发布及中和/重开检查；OOC没有约束接口I/O delay，不等于完整平台STA。见[闭合child OOC及参数证据](stage_d_capacity_reduction_20261102/stable_job_children_synthesis/verification.json)（归档SHA `868fffd42c8a930f7c32b100c0acdeb81da856a9b3f1986db48fe943b2a09850`）。

root随后用已审修复生成器`62fc9f32…`实际完成wrapper准备，冻结133份输入，清单SHA256为`c9151a5ceb432832cfa60cd19ce101991248ddcd7d63fb0e67a92391335eb1ac`，当前保持不变；这与此前28项纯测试分开记录。第25轮工作目录为`/tmp/robot_rt_managed25_flow_20261102`，输出`build/kr260_20261102_attempt25_stable_job_inputs`。prepare、七个IP与TOP共9个阶段的原生综合流程及独立controller复核均已通过；core为95,795 LUT/100,633 FF，两个真实子实例的参数、装配及发布后读回均通过，其中gate明确采用mode1。

完整平台综合报告为100,278 LUT、106,485 FF、280 DSP、32 BRAM，未布局估计setup WNS +1.524 ns、TNS 0、失败端点0；hold WHS −0.087 ns、THS −1303.724 ns、失败端点56,006，`no_clock=8`。相对第24轮完整综合增加2,428 LUT、减少20,995 FF；这是综合报告差值，尚不能证明布局CLB密度已解决。[完整综合与独立复核归档](stage_d_capacity_reduction_20261102/managed25_full_synthesis/verification.json)已闭合（verification SHA256 `41286ae96f8951200999b0ff18ccabd19fa82c41db1f402bdcecd04ef6e99698`，366文件、370项归档检查/715项原始来源检查）。

第25轮place/physopt已闭合；随后root向经PID、start时间与guard祖先链核实的owned native发送SIGINT，主动停止Global Iteration1的未完成route。guard实际rc1、6665.517秒、最低可用内存3,562,272KiB，最后overlaps 116,389；这是操作员取消，不是自然route/时序失败或内存终止，未得到routed DCP、最终STA或bit。[第25轮停止记录](stage_d_capacity_reduction_20261102/managed25_stopped_route/verification.json)保留wrapper rc0与外层严格拒绝、原始残留状态的区别。

同一opt DCP的独立`-no_psip`分支已完成[place-only对照](stage_d_capacity_reduction_20261102/no_psip_placement_comparison/verification.json)及[默认physopt诊断](stage_d_capacity_reduction_20261102/no_psip_physopt_diagnostic/verification.json)。该分支诊断资源为97,249 LUT、104,459 FF、280 DSP、32 BRAM，CLB 14,454/14,640（98.73%）、1,691 control sets；相对原25布局，控制集减少1,897但CLB增加37，不能称面积或route收益。默认physopt真实rc0、169.395秒；无setup违例，netlist未修改。实际min/max报告setup WNS +0.523ns/TNS0，hold WHS −0.083ns/THS −9.262ns、457失败，no_clock0；仍不是最终routed STA。

该独立route随后也由root向经归属核验的owned native发送SIGINT主动停止，诊断链现已闭合为取消；guard rc1、779.373秒、最低可用内存5,042,100KiB。首Global Iteration1 overlaps为283,430（原25为273,251），仍level6；中间估计WNS +0.727ns/WHS −0.073ns均非最终结果，不能据首点较高断言自然失败。224份输入停止时fresh SHA检查通过。实际body与锁定SHA相符，原签核Tcl正文仅删`open_run impl_1`和`close_design`两行、原clock/STA/DRC门保留；但取消后最终签核未执行，也没有routed DCP、bit或XSA。[独立route停止记录](stage_d_capacity_reduction_20261102/no_psip_stopped_route/verification.json)保留完整原始状态，不能当route收益或签核通过。

从原opt DCP `af0ce805…`追加一次`opt_design -resynth_area`的[AREA-only诊断](stage_d_capacity_reduction_20261102/area_only_resynth_diagnostic/verification.json)已自然闭合：guard rc0、306.678秒、最低可用内存5,238,996KiB，245项来源SHA/stat独立核对均通过。两份报告同为Optimized，LUT 97,462→94,658，减少2,804（2.877%）；FF 104,458、DSP280、BRAM32不变，control sets 1,691→1,313。这只证明该优化阶段的LUT下降，不是与旧placed混比，也不证明CLB打包、route或最终时序改善。未布局估计setup WNS +1.215ns/TNS0，hold WHS −0.087ns/THS −1278.708ns、54,451失败，no_clock0；没有hold豁免，未完成最终STA。

基于该AREA产物`8103e4b1…`的[默认布局诊断](stage_d_capacity_reduction_20261102/area_default_placement_diagnostic/verification.json)已自然闭合：guard rc0、1020.979秒，272项来源SHA/stat复核通过。实际93,066 LUT、104,559 FF、280 DSP、32 BRAM，CLB 14,431/14,640（98.57%）、2,291 control sets；相对第25轮旧布局少3,509 LUT、46 FF及1,297 control sets，但CLB多14个，不能称打包或route改善。setup WNS +0.138ns/TNS0，hold WHS −0.113ns/THS −12.630ns、557失败，no_clock0。随后[默认physopt](stage_d_capacity_reduction_20261102/area_default_physopt_diagnostic/verification.json)也已闭合，guard rc0、212.466秒、306项来源复核通过；因无setup违例，netlist未改，上述资源和时序数值相同。这些阶段均未签核，未采用为生产默认。

AREA产物的独立route已由root在核实PID/start/cwd/guard祖先链及325项输入SHA后主动SIGINT停止；guard rc1、12,054.185秒，最低可用内存4,515,048KiB。最近中间WNS −1.168ns/TNS −1272.256ns，Global Iteration2最后overlaps 13,628；这些都不是最终STA。没有routed DCP、签核通过、bit或XSA。停止是为了实测已通过功能验证的新计算结构，属于操作员取消，不能写成自然失败。见[闭合停止证据](stage_d_capacity_reduction_20261102/area_default_stopped_route/verification.json)。

独立精确倒数候选8b9已经通过三组各4,112数值向量、shared/top和55,260项四态/复位检查；[功能证据](stage_d_capacity_reduction_20261102/reciprocal_division_validation/README.md)保留其范围。随后实际隔离OOC闭合，但同设置raw synth资源从旧37,264 LUT/26,303 FF/198 DSP增加到40,477 LUT/37,666 FF/478 DSP，WNS +3.744ns；因此没有采用该候选。默认opt另列，不能与旧raw混比；见[实际OOC对照](stage_d_capacity_reduction_20261102/reciprocal_division_ooc_diagnostic/README.md)。该候选未采用，原0b完整证据保留。

第26轮采用了仅复用原乘法银行的7b34候选。三组各4,112数值向量、shared/top、55,260项四态/复位及20,655项稳定owner生命周期检查通过，非法CLOCK0/−1的基线/候选四例均实际命中原time0拒绝；参见[功能证据](stage_d_capacity_reduction_20261102/reciprocal_sharedbank_validation/README.md)。[真实同设置OOC](stage_d_capacity_reduction_20261102/reciprocal_sharedbank_ooc_diagnostic/README.md)中raw LUT 37,264→33,653（减少3,611，约9.69%），FF 26,303→32,772，DSP198不变，WNS +3.869ns；额外defaultopt为33,573 LUT/WNS +4.177ns。LUT下降伴随FF增加，不能预称CLB打包、route或最终时序改善。

新的独立child已实际闭合：guard rc0、151.290秒、最低可用内存3,940,608KiB，新math真实重综合、未改flag1 gate按原完整证据复用。已严格核DIRECT guard/argv/request、v2 generics、端口、时钟中和及重开；[child证据](stage_d_capacity_reduction_20261102/reciprocal_sharedbank_children_synthesis/README.md)。root随后真正冻结父wrapper129项输入，清单SHA `81de42652e354f92fad344e06fcb3f56d1fabda534fdb893f02779b03fb26c9d`，从全新目录启动第26轮完整构建；[准备与启动范围](stage_d_capacity_reduction_20261102/reciprocal_sharedbank_parent_preparation/README.md)不代表后续综合/布局/route/STA已经通过。仅canonical math改为7b34，原14RTL/构建工具中其余字节核对保持，变更前备份在`build/baselines/before_sharedbank_reciprocal_20261102/`。ACT部署与物理运动未由本轮改动。

第26轮九个综合阶段及各独立controller已全部闭合，完整平台为96,667 LUT、112,954 FF、280 DSP、32 BRAM；相同综合阶段比旧25少3,611 LUT、多6,469 FF。综合估计setup WNS +1.524ns/TNS0，但hold WHS −0.087ns/THS −1334.095ns、57,735失败，no_clock8，不能当最终STA通过；[闭合综合证据](stage_d_capacity_reduction_20261102/managed26_full_synthesis/README.md)保留综合完成时父guard尚未闭合的原始点时记录；本轮最终失败见下文。

随后place和默认physopt已完成：94,747 LUT、111,020 FF，CLB 14,422/14,640（98.51%）、3,362 control sets。相比旧25同阶段，LUT少1,828、FF多6,415、CLB反多5个，未证明打包改善。physopt仅有setup估计WNS +0.278ns且netlist未改，没有本阶段完整min/max报告，hold/no_clock不补造。见[闭合布局点时证据](stage_d_capacity_reduction_20261102/managed26_closed_placement/README.md)。第26轮route现已自然失败：`Route 35-2`报告37,791个node overlaps，原生route耗时15,480.7秒；guard rc1、17,627.898秒、最低可用内存3,163,188KiB，不是取消或内存停止。失败后DRC的0 Errors不抵消非法布线，`routed_error.dcp`只作诊断，不能用于bit/export。129份输入fresh SHA全部匹配；动态清单实际48行/44个唯一路径，保留重复项。见[自然布线失败原始证据](stage_d_capacity_reduction_20261102/managed26_failed_route/verification.json)。没有合法route、最终STA、bit/XSA或上板验收；新优化仅在私有候选中实测，尚未采用。

本轮完整工程候选采用math7b34ff4d/gate43b7a859/sharedd2c9e6d及TB5b0d5717，top05bf不变，四构建工具为修复版`ffb85360…`所绑定字节；validator026560、seed96a8和cache7cd18保持。数学前端与倒数校正共用原乘法银行，仍E78，顶层仍112拍；gate新参数`STABLE_JOB_INPUTS`通用默认0保留快照，仅top稳定owner合同显式选择1。child使用`robot_rt.partition.v2`及逐模块generic/证据绑定，parent独立flag1同时核top常量与shared转发；不接受v1 gate或跨flag DCP，不能用旧a5替代新gate。线程1、原10ns及setup/hold/脉宽、未约束路径、DRC和新bit门槛不变。

[上一版0b/43b功能闭合证据](stage_d_capacity_reduction_20261102/stable_job_gate_validation/verification.json)保留实际/最大整数参数各4,112数学向量、shared及top1,593,194检查；top新增完整data monitor为76个job/1,216个边沿，其cancel/reset为0，不冒充覆盖。另一次模式1专门测试真实覆盖8次cancel、32个reset目标、20,655项检查，使用同一个PL cache，并验证旧saved allow不能充当新done。首个五套runner中的`gate_default`标签实际是组合reference；新流水门默认0已另外通过pipe738向量和domain4个job/80检查。单位级cancel/reset不测外部112，112证据来自独立完整top。归档“私有/未采用”字段保留测试创建时口径，随后采用由上述静态记录单独确认；无本版物理资源或STA结论。

第24轮已经闭合：root核对自有native PID80333/start28698945后主动SIGINT停止route，guard rc1/5,150.784秒、最低可用内存3,714,592KiB；不是内存停止或工具自然失败。place15分46秒完成，94,504 LUT /125,590 FF /280 DSP /32 BRAM，CLB14,580/14,640（99.59%）、4,150 control sets；physopt WNS−1.002ns，route中间WNS−0.947ns/TNS−469.221ns/WHS−0.070ns，level6且最后126,492 overlaps。没有final route/STA/bit，见[闭合实现记录](stage_d_capacity_reduction_20261102/managed24_placement_and_stopped_route/verification.json)（31839207…，306项归档检查）。第24轮综合97,850 LUT /127,480 FF、WNS +0.204ns/hold负余量/no_clock8保留于[综合档案](stage_d_capacity_reduction_20261102/managed24_full_synthesis/verification.json)；相对23综合少9,788 LUT/7,604 FF不是布局CLB差。

第24轮选用c335 RuntimeOptimized child：math为34,626 LUT /25,323 FF /198 DSP，10ns独立OOC WNS +0.426ns、WHS +0.106ns；同源Default备选为37,443 LUT /24,901 FF /198 DSP、WNS +1.985ns，保留回退。两组均完成结构检查、时钟中和与真实DCP重开；各gate都复用自身directive下原a5验证工件，没有本次gate重综合。选定Runtime不等于已证明全平台更好，孤立估计不能代替最终STA，见[两种child实际记录](stage_d_capacity_reduction_20261102/divisor_children_synthesis/verification.json)。父core仍RuntimeOptimized、每进程线程1；PS/RAMB同钟、原10ns setup/hold/脉宽、未约束路径、DRC及bit新产物门槛均不放宽。

第23轮已闭合：place真实完成31分56秒，102,352 LUT /133,298 FF /280 DSP /32 BRAM，CLB14,611/14,640（99.80%）；physopt完成。观察到level6拥塞、372,478 overlaps，且新候选实际HDL通过后，root改变原让其自然完成的计划，向核实归属的自有native发送SIGINT，主动中止24分16秒的route。guard rc1、4,921.379秒、最低可用内存3,705,564KiB；不是内存保护或自然route失败，没有完成route/最终STA/bit。见[完整停止记录](stage_d_capacity_reduction_20261102/managed23_placement_and_stopped_route/verification.json)。

第24轮历史生成器只在两child均BOTH VERIFIED及其guard真实闭合rc0后冻结实际输入。通过manifest/literal partition_plan引用各自真实DCP、stub、signature和原验证日志；gate复用目录可能只有reuse.json，必须沿其原manifest取工件，不能把旧leaf路径前缀替换成新目录。60份是本次去重后的实际数量，不能硬编码为历次37份。显式child目录、math SHA与child synth directive必须相互一致，其他RTL/Tcl仍对原基线核SHA；run在每阶段前后保持输入barrier。原生worker发布后仍需新controller核真实实例/零黑盒、run/gen DCP相等及真实状态，最终route/signoff/bit门不变。

阶段25新CLI使用两个独立选择；下列示例默认只显示计划，不会另外创建工程或代替root已经启动的parent流程：

```bash
/usr/bin/python3 tcl/compile_children.py --repo . --out build/children_v2_new --synth-directive runtimeoptimized --gate-stable-job-inputs 1
/usr/bin/python3 tcl/split_synthesis.py --phase prepare --repo . --out build/parent_v2_new --partition-dir build/children_v2_new --core-directive runtimeoptimized --partition-gate-stable-job-inputs 1
```

真实执行需另加`--execute`且目录必须fresh。只写compiler flag1不足以证明parent flag1；后者prepare-only，源常量/转发、逐模块generics及参数证据在冻结/装配/发布后读回均须一致。v1仅可迁移同源clock-only math的完整旧证据，新math0b不能冒充c335；v1 gate绝不迁移。本次真实子综合已经确认双generic brace命令能通过严格literal/canonical token解析；缺等号、双等号、重复或未知项仍拒绝。旧解析缺口与修复测试保留，不松STA或worker检查；第25轮参数化parent的两个真实子实例已通过原生worker装配及独立controller发布后读回，gate的mode1与父级独立flag一致；此项只证明原parent装配/读回，不替代AREA-only产物选择或未来route及最终STA；已取消的诊断route和place/physopt均另有闭合证据。生成器62fc还要求闭合guard的DIRECT argv/cwd、真实plan文件SHA和JSON、child request/source/profile/generic一致；不声称认证launcher PID。

以下为通用步骤及各轮历史原文；历史“进行中”不是阶段25验收结论。

[小工程三步实际验证](stage_d_split_build_20261102/probe/verification.json)通过，Project真实状态为Complete/100%/NEEDS_REFRESH=0，实际XDC读取及DCP同时核查。主工程attempt13已完成BRAM Controller和chunk RAM的相同流程，13轮核心因新进程未继承线程限制而触发内存保护，15轮已明确每进程设置并检查；这不是主设计容量、STA或bitstream验收。

从repo根目录，输出目录必须不存在。默认只显示命令；加 `--execute` 才运行本机工具：

```bash
/usr/bin/python3 tcl/split_synthesis.py --phase prepare --repo . --out build/kr260_external_new --execute
/usr/bin/python3 tcl/split_synthesis.py --phase synth --repo . --out build/kr260_external_new --run robot_rt_bd_bram_ctrl_0_synth_1 --execute
```

按生成的 `split/plan.tcl` 中ips顺序逐个执行单run入口，最后执行 `synth_1`。当前固定工程的顺序为bram_ctrl、chunks、core、fabric、guard、ps、resets，然后顶层；新增IP依赖须重新审查顺序。每run首次验证创建不可覆盖的证据；已有证据相同才允许只检复用，内容不同拒绝覆盖；失败不自动reset或覆盖缓存。

冻结清单覆盖列明的原始RTL、构建库/driver、XDC和plan，并记录BD/XCI/综合wrapper、源/约束集成员及关键属性。DCP与明确非构建角色的生成stub/仿真网表是后续产物，不因当前缺失而冒称输入；这仍不代表整个Vivado工程或工具安装已做防篡改认证。只在新建隔离工程中使用，运行中不编辑源文件、约束集或生成工程。每个run还核source/constraint set名称、真实状态、end/error标记、非空DCP和日志；退出码0不单独构成成功。

> 历史快照：以下一段为第23轮启动时记录，现已place完成后主动停止route。

当前fresh attempt23已真实启动，37份冻结输入清单SHA256为`3f24a6657c4e28c81b20db269bb70bdb6c86dd227a565ae48a1c5014f45103e5`。相对第22轮仅采用cache7cd18的完整部分乘积分拍，E38发布；Default math/gate DCP按原来源/参数/产物合同复用，父core仍RuntimeOptimized。第23轮八项综合及独立复核已通过，完整布局布线正在运行。route/bit签核继续使用主库原检查：PS/RAMB同钟、setup/hold、未约束路径及DRC；签核和routed DCP/输入manifest绑定后才允许bit步骤。未加载板卡、未连接机械臂，没有实机效果结论。

后续入口已准备（目前尚未实际实现签核）：

```bash
/usr/bin/python3 tcl/split_implementation.py --phase route --repo . --out build/kr260_external_new
/usr/bin/python3 tcl/split_implementation.py --phase finish --repo . --out build/kr260_external_new
```

默认仅显示步骤。实际运行需加 `--execute`；本任务在综合完成后继续执行。route与bit分阶段均在controller退出后执行worker，finish先调用原STA/DRC签核并绑定routed DCP及输入manifest，拒绝使用route旧标记或旧bit文件充当新bit成功。此入口不加载板卡。

## 第16轮的模块层级编译选项

15b在单worker、controller退出后编译的条件下，仍于689.518秒触发1GiB可用内存保护；没有产出核心DCP、最终利用率或STA。第16轮在全新输出目录显式选择保留core层级：

```bash
/usr/bin/python3 tcl/split_synthesis.py --phase prepare --repo . --out build/kr260_hierarchy_new --core-flatten none --execute
```

`--core-flatten`只在prepare接受，默认仍为`rebuilt`。`none`只作用于core综合run，在OOC预检前写入并固定在plan.contracts，后续controller核对实际属性、生成Tcl及真实worker日志。原RTL、时钟/XDC、单worker和最终签核门槛均不改变；其他IP与顶层综合不改此选项。失败工程不自动重置或混用结果。

本地AMD官方`synth_design`手册84–94行说明：`none`保留模块层级并限制综合优化；它不等于保留全部寄存器，也不承诺降低内存或改善时序。模块内合并、常量和未用逻辑消除仍可发生。实际构建状态见REPORT；功能仿真证据不代替该profile的综合/实现报告。

## 独立子模块与发布前装配

当前可选流程先综合真实`player_math`与`limit_gate_pipe`，固定K26、CLOCK_HZ=99,999,001及原10ns OOC目标，再清除各子模块的临时OOC timing并重开DCP检查端口和零时钟。参数兼容stub只用于父核心的独立综合；当前共享版实际装配必须填入两个指定实例、核对原端口并拒绝任何残余黑盒。

```bash
/usr/bin/python3 tcl/compile_children.py --repo . --out build/children_new --synth-directive runtimeoptimized
/usr/bin/python3 tcl/compile_children.py --repo . --out build/children_new --synth-directive runtimeoptimized --reuse-from build/kr260_child_partitions_runtime_20261102 --execute
/usr/bin/python3 tcl/split_synthesis.py --phase prepare --repo . --out build/kr260_partition_new --partition-dir build/children_new --execute
```

输出目录须全新。`runtimeoptimized`是已安装Vivado的官方综合选项，减少部分RTL/时序优化；它不免除最终PS时钟下的STA。`--reuse-from`只复用源及各DCP/端口/脚本/日志SHA仍匹配的已验证子模块；改变的模块重新编译。当前新gate实际综合与math复用见[OOC证据](stage_d_axis_gate_20261102/ooc/verification.json)，旧两个独立综合见[历史证据](stage_d_partition_20261102/children_runtime/verification.json)。这些报告均未布线，不构成完整BD容量或STA验收。

原生`TCL.POST`位于默认checkpoint保存之后，[实际反例](stage_d_partition_20261102/managed_post_failure/verification.json)显示worker完成而重开仍有黑盒，因此当前可选流程要求该属性为空。controller生成全新core worker后，在唯一完整`synth_design`命令之后且任何`rename_ref/write_checkpoint`之前插入一次装配source。保存原文、修改文、diff、literal/JSON SHA合同，runner在执行完整原`runme.sh`前后分别核对；不改完成标记或发布后的DCP。此变换针对本机已核查的Linux LF生成文本，不承诺任意编码输入。

[小工程实际验证](stage_d_partition_20261102/managed_insert/verification.json)已确认正常worker完成、新controller的Complete/100%/NEEDS_REFRESH=0以及保存的两个真实子实例。主core还必须核对两个共享真实实例、无黑盒、正常run和gen发布DCP的SHA相同，并检查局部原10ns时钟覆盖。小工程通过不替代此主设计检查；最终PS/RAM同钟、STA/DRC及bit门槛仍使用原主库，未放宽。

当前两个子实例为`inst/core_impl/shared_execution/math`及`inst/core_impl/shared_execution/gate`，仍只接收原真实player_math/limit_gate_pipe的DCP。四独立实例流程与小工程保存证据保留为历史，不能直接冒充当前共享版的主工程验收。真实REGISTER分类及六项有效时序结构检查在装配和重开后都执行；已发布DCP重复复查只允许完整DCP/XDC/报告绑定一致的只读缓存。当前60项纯Tcl测试见[记录](stage_d_shared_execution_20261102/partition_helper/verification.json)，实际资源和STA结果见[REPORT](REPORT.md)。

## 第18轮的显式core综合选项

attempt17的core默认综合触及本机内存保护，未产生核心结果；BRAM/chunks真实完成记录保留。fresh18使用前面隔离资源构建实际运行过的官方选项：

```bash
/usr/bin/python3 tcl/split_synthesis.py --phase prepare --repo . --out build/kr260_runtime_new --partition-dir build/children_new --core-directive runtimeoptimized --execute
```

`--core-directive`仅prepare允许，省略时保留原run设置；显式时写入并回读`RuntimeOptimized`。无论是否显式，实际值都会进入冻结合同，每个后续controller检查该core属性。显式设置还要求生成worker Tcl和真实综合日志中的实际`synth_design`命令包含唯一匹配directive，不能由注释替代。实际fresh18 prepare已接受该属性；33项[纯CLI/Tcl检查](stage_d_shared_execution_20261102/core_directive_candidate/verification.json)不替代完整综合、内存或STA结果。运行中不改冻结源，旧17工程不重置。最终时钟、约束、算术精度、原保护及签核要求保持。

实际attempt18父核心综合已成功，但旧装配hook错误地要求`current_design.NAME`等于HDL顶层。真实工具探测区分了会话名NAME与只读HDL属性TOP，当前helper严格核PART+TOP并输出实际值；完整端口、两个真实实例、黑盒、原时钟和发布后重开检查保持。见[实际失败与探测](stage_d_shared_execution_20261102/managed18_actual/verification.json)。fresh19已采用修正，结果仍须实际验收。

## 第19轮的真实发布与验证器修正

managed核心已真实装配、正常worker完成并发布相同run/gen DCP，独立读回通过。实际日志含`synth_design completed successfully`，命令解析现限定其后以选项开始，排除状态提示，错误/缺失/重复directive仍拒绝。原临时核心时钟复查改用本机官方`read_xdc -no_add`，避免把临时XDC注册进project；复查结束再严格核对完整原快照。此前误登记的唯一额外member经精确删除后，原所有源/约束集、成员顺序、属性和内容以及关开工程后的快照均相同，core真实current状态保留。

为沿用同次已完成核心，修正验证器作为独立带SHA副本运行；旧frozen源与清单未改，额外manifest绑定旧/新driver、helper、diff、旧manifest及实际worker/日志/DCP，执行前后核对。没有编辑完成标记或降低原身份/时钟/资源/STA门槛。历史假拒绝和恢复过程独立保存，见[REPORT](REPORT.md)。完整平台/布线后签核仍待实际执行。

第19轮全部IP及完整顶层综合已实际通过，优化前120,329 LUT、WNS−6.442ns及no_clock8均保留为待处理结果；原生opt/place/route已启动。实现入口的静态解析最初未识别实际`set rc [catch {…} RESULT]`格式，原生worker尚未启动时被拦下；随后逐项核对真实线程设置在执行型init_design catch中且早于create_project/link，并独立绑定39个源/已完成DCP/原生入口SHA后执行完整未修改runme。结束后仍由新controller核验，最终STA未通过不生成验收结论。见REPORT和当次continuation manifest；没有reset或编辑完成标记。

## 第19轮实际布局结果及后续输入

全部IP与完整顶层综合已通过；原生opt完成后真实平台报告为115,429 LUT，但细布局Place 30-487因实际打包需求133,706组合LUT/CLB不足失败。没有route结果、STA签核或bit；见[实际证据](stage_d_shared_execution_20261102/managed19_native_implementation/verification.json)。

运行结束并归档后，已将完成行解析、read_xdc -no_add及读回后完整快照核验纳入生产driver/helper；实施worker检查同时拒绝同一命令中的可执行分号复合命令，实际native完整夹具仍通过。旧工程冻结清单不改写；本轮seed/static_valid缩位及任何新子模块DCP均须新建工程prepare。最终中央签核函数没有改动。


## 第20轮已停止的完整构建

数学子模块采用精确共享divider和乘法输入流水，实际32,387 LUT、未布局WNS +0.792ns；原15拍gate验证DCP经源/产物合同校验复用，本轮没有重编gate。见[实际子模块证据](stage_d_capacity_reduction_20261102/input_pipeline_synthesis/verification.json)。完整工程使用新输出`build/kr260_20261102_attempt20_pipeline_bounds`，绑定34份当前源码、Tcl和子工件，未重写第19轮冻结输入。

同一受管串行流程已实际完成prepare、七个IP综合/独立复核和顶层综合/独立复核，随后执行生产`split_implementation.py --phase all`。该轮原生opt/place均实际完成，place为20分31秒、102,511 LUT /128,281 FF /281 DSP /32 BRAM；post-place估计WNS −5.843ns，physopt最终估计−5.621ns；route中间−5.070ns和level6拥塞后由本任务主动SIGINT停止，保存了真实placed/physopt工件用于只读诊断。不是内存保护停止，也没有完成route/最终STA失败结论。没有route签核、bit或XSA产物。本轮完整综合优化前为107,071 LUT /129,776 FF /281 DSP /32 BRAM，未布局WNS −5.207ns、no_clock8原样保留；这两个数字不能解释为最终布局容量和时序。运行期间不改RTL/Tcl/输入合同，最终仍使用原中央PS/RAM同钟、setup/hold/脉宽、未约束路径与DRC门槛。最终结果以REPORT及原始工件为准。

第21轮开始构建时采用串行seed96a8、math提前译码3c91和寄存输出sharedf4/top826；当时完整顶层实际1,593,126检查、112拍和原保护保持。shared新增寄存监控35,290次并保留原21,306检查，math独立4,112向量仍E78，串行seed5,026个job和隔离综合证据继续保留；这些独立证据不是完整STA。原attempt20冻结源、综合与停止记录保持，新源码及child合同只进入新工程。

## 第21轮综合完成与第22轮时间映射分拍

第21轮七个IP和完整顶层综合/独立复核已通过：105,957 LUT /134,775 FF /276 DSP /32 BRAM，未布局综合WNS −3.558ns，最差路径为validator的threshold→index两个乘法级联，逻辑延迟10.514ns。原生opt已完成；随后由本任务向经身份核验的自有进程发送SIGINT，在placement Phase2.4 GlobalPlace Phase1主动停止。guard返回1、1963.815秒，最低可用内存2,944,576KiB；不是内存保护或自然布局失败，没有完成place、route或最终STA。见[完整综合](stage_d_capacity_reduction_20261102/managed21_full_synthesis/verification.json)及[主动停止布局](stage_d_capacity_reduction_20261102/managed21_stopped_placement/verification.json)。

第22轮前已采用串行seed96a8、math3c91、sharedf4/top826及validator026560，当时完整顶层1,593,148检查、112拍保持。该validator的HCHECK/RCHECK/FINISH及新增后台固定拍证据继续保留，随后cache分拍的本轮记录见下节。

第22轮采用validator026560：完整128位threshold/index乘积分开寄存，新增后台`1+2*(N+1)`拍，原130周期ceil除法、时效与运行112拍不变；其独立OOC结果不能代替完整签核。第22轮原目录、37冻结cf639081…及Default child合同保留，不改旧输入或完成标记。

## 第22轮实现前取消与第23轮cache乘积分拍

第22轮七个IP和完整顶层原生综合及独立复核已完成：107,744 LUT /134,944 FF /276 DSP /32 BRAM，综合估计WNS −1.531ns、WHS −0.087ns、no_clock8，未作最终验收。外层在TOP driver期间暂停，TOP独立controller与原生worker自然完成后，本任务在实现前取消外层；原execution保留8条已收取记录与KeyboardInterrupt，未伪造第9条记录。guard返回−2、1530.924秒、最低可用内存3,542,428KiB；无impl_1目录，没有opt/place/route、最终STA或bit。见[完整综合](stage_d_capacity_reduction_20261102/managed22_full_synthesis/verification.json)。

第22轮首次只读诊断直接打开单个顶层DCP，七个IP引用未解析，strict clock检查真实失败；第二次只读打开完整project/run后解析成功。其100条最差路径均为cache同一源寄存器到多个scratch终点，逻辑7.446ns、未布局route估计3.821～3.824ns、slack −1.531～−1.528ns；支持当前cache关键路径排查，不排除未报告的其它路径族。失败、通过及暂停/取消原始记录均保留于[诊断档案](stage_d_capacity_reduction_20261102/managed22_synthesis_diagnostics/verification.json)，不是布线后STA。

当前采用串行seed96a8、提前译码math3c91、寄存输出sharedf4/top826、时间映射分拍validator026560，以及精确分拆乘积cache7cd18。新cache快照完整signed64/98输入，分别寄存低32位无符号与高32位有符号的完整部分乘积，符号扩展后精确重组；E38原子发布，不改原始高位/epoch守卫或运行侧112拍。99,999,001Hz数值profile的组合顶层实际通过1,593,194项检查，52个后台job中4个并行、1,378拍等待。cache五个参数profile各通过165个job、39处复位和114次逐拍epoch检查；最大正int32只是数值域测试，0/−1检查非法参数拒绝，不表示相应物理频率。见[cache功能验证](stage_d_capacity_reduction_20261102/cache_split_pipeline_validation/verification.json)。

已采用的cache7cd18独立RuntimeOptimized、10ns OOC综合为7,450 LUT /13,844 FF /24 DSP，WNS +5.347ns；接口延迟未约束及HD.CLK_SRC警告仍保留，不能代替完整平台STA，见[cache隔离综合](stage_d_capacity_reduction_20261102/cache_split_pipeline_synthesis/verification.json)。fresh attempt23已由本任务真实启动，冻结37份输入，清单SHA256为`3f24a6657c4e28c81b20db269bb70bdb6c86dd227a565ae48a1c5014f45103e5`。相对第22轮只更换该cache RTL，其余控制、时效、112拍及已验证Default math/gate DCP复用合同保持，父core仍RuntimeOptimized。第23轮八项综合及独立复核已通过，完整布局布线正在运行；尚无最终STA、bit或上板结果。 新cache仍由父core源码综合，没有增加第三个分层child；原两个共享实例与最终中央签核门槛保持。
