# π0.5 类负载在 XDNA2 上的有效算力实测报告

**状态：未达标，因当前 Ryzen AI 1.8 运行时/缓存依赖故障停止。8.0 effective TFLOPS 目标尚未实现；P2 尚未通过，整块 proxy 尚无有效性能分数。** 完整主干曾编译成功，但实际执行审计出现全 CPU 节点；随后严格缓存复用在 session 构造阶段 SIGSEGV，尚未进入 warmup。具体复现、证据与恢复条件见 [BLOCKED.md](BLOCKED.md)。不得把编译成功、功能修复或 CPU fallback 结果用作 NPU 达标证据。

已完成当前机器环境勘察、无权重下载的静态 shape proxy、编译缓存与设备归属审计工具，以及完整独立视觉阶段的正式测量。视觉阶段在 20 次 warmup、20 次样本下为 p50 **723.667 ms**、p95 **724.771 ms**、**0.91131599 effective TFLOPS**。该数字只代表视觉阶段，不代表 π0.5 整块，也不能与用户提供的 FLM 4.9 TFLOPS 构成性能 A/B。[测量记录](results/P1_vision_measured_baseline.json)

## 测试环境与边界

本机为 Ubuntu 24.04.2 LTS、Linux 6.14.0-27-generic、AMD Ryzen AI 9 HX 370，NPU 为 Strix / aie2p，驱动 amdxdna `2.25.260102.56.release_20260630`，XRT 2.25.37、NPU 固件 1.1.2.64。当前独立环境的 Ryzen AI / VOE 为 1.8.0、onnxruntime-vitisai 为 1.27.0，providers 含 `VitisAIExecutionProvider` 与 `CPUExecutionProvider`；`pip check` 通过。系统电源配置为 `performance`，XRT 所报 NPU 模式为 `default`，两者分别记录，没有修改电源设置。[环境指纹](results/P0_environment.json)、[原始命令证据](evidence/p0_commands.json)、[环境说明与安装路线](environment.md)

NPU 频率、AC 状态、物理内存容量、内存速率和带宽未能读取或未测量，保持 JSON null。OS 可见内存为 28,334,220 KiB；不据此填报物理 32 GB 或约 120 GB/s 为实测值。本地 LeRobot 环境的 torch 元数据带 ROCm 7.14.1，但本次推理均未调用 GPU；ROCm 版本的这个限定范围保存在环境指纹中。FLM 和 openpi 未在限定合理目录发现，未下载模型或重复测量用户已给的 FLM 结果。

全部生成、编译与推理在目标机器执行。SDK 从已有完整 uv 缓存恢复，没有更改 vendor 代码，没有编写 IRON/MLIR-AIR kernel。OOM 防护通过本任务进程的 systemd scope 内存限制实施；它不是系统电源调整。[恢复记录](evidence/sdk_restore.json)、[取舍记录](DECISIONS.md)

## 代理与计算口径

代理权重为独立非零随机值，固定 seed 20261030，不依赖权重下载。三路视觉为单次 batch=3 调用，27 层、256 token/图、d=1152；主干为 18 层、d=2048、GQA 8/1、seq=968，导出 18 层 KV；动作专家为 18 层、d=1024、50 个动作 token、41 个输入，由 host 串行调用 10 次。专家注意力包含 968 个 prefix KV 与当前 50 个动作 KV，即 1018 个 key。[实际图与逐项 FLOP 账本](bench/proxy_active/manifest.json)

输入为预嵌入 token，覆盖视觉 Transformer、主干 prefill、动作专家与 Euler 更新，排除图像 patch stem、图像预处理、词表查找、相机和机器人 I/O；不声称真实权重策略精度或完整 pi05_base 端到端性能。图初始即采用静态 shape、MatMul、加性 mask，无 Einsum、TopK、NonZero；这些初始构图条件不能作为后续优化增益。

有效算力定义为有用的算法 GEMM FLOP 除以同步 wall time，每次乘加计 2 FLOPs。归一化、RoPE、GELU、Softmax、同步及拷贝耗时保留在延迟内，不额外估计它们的 FLOP。整块计时应包含视觉、host 数据衔接、完整主干和串行 10 次专家；整块 p50/p95 必须来自完整 block 样本，不相加各阶段分位数。正式性能至少 20 次 warmup 与 20 次样本；单次功能检查和 EP 审计没有性能分数。

| 部分 | 有用算法 GEMM FLOP | 来源 |
|---|---:|---|
| 视觉单次调用 | 659,489,292,288 | manifest `stages.vision.flops_per_call` |
| 主干单次调用 | 3,974,951,337,984 | manifest `stages.backbone.flops_per_call` |
| 专家单步 | 38,889,652,224 | manifest `stages.expert.flops_per_call` |
| 整块，含专家 ×10 | 5,023,337,152,512 | manifest `flops_per_block` |

实际图的分子为 **5.023337152512 TFLOP/块**，不以任务概算的约 4.8 TFLOP 替代。内部 KV 补齐候选额外执行 7,993,294,848 GEMM FLOP/块，独立列账，不计入有效算力分子。[补齐候选 manifest](bench/proxy_p2_padded_active/manifest.json)

当前可执行基线是官方 VAIML 的 BF16 路径：视觉源图 FP32；为降低编译内存，主干/专家从同一组随机权重一次舍入至 FP16，再请求编译为 BF16。这是可行性修复，尚无成功整块 FP32/BF16 性能 A/B。原生 INT8、per-channel 权重、per-tensor 对称激活与 P5 量化余弦验收尚未建立。

## 峰值规格与有效算力对照

| 项目 | 数值 | 来源 / 是否本次实测 |
|---|---:|---|
| XDNA2 INT8 理论峰值规格 | 50 TOPS | 用户给定规格；不转换为 TFLOPS，不作利用率分母 |
| 本机 FLM 3B Q4_1 既有有效算力 | 约 4.9 TFLOPS | 用户已知实测，直接采信；未复测，无原始 p50/p95 |
| 本任务整块目标 | ≥8.0 TFLOPS | 验收目标；尚未达到 |
| 本次完整独立视觉阶段 | 0.91131599 TFLOPS | 20 warmup +20 样本，p50=723.667 ms；仅视觉 |
| AMD IRON 融合 FlashAttention 参照 | 3.62 TFLOPS | 用户给定参照，工作负载不同 |
| Qualcomm IQ-9075 π0.5 参照 | 1111 ms/块 | 用户给定参照，平台与完整模型计量边界不同 |

给定参照的来源和限制集中在 [P0_accepted_references.json](results/P0_accepted_references.json)。FLM prefill 的 1K/2K/4K 既有数值分别为 766.4/892.9/927.7 tok/s；本次未生成这些样本。以上跨工作负载数据用于背景对照，不能据此计算本代理加速比或证明达到目标。

## 合格性能测量

| 测量范围 | warmup / 样本 | p50 ms | p95 ms | effective TFLOPS | 结论 |
|---|---:|---:|---:|---:|---|
| 完整 27 层视觉，batch=3 | 20 / 20 | 723.666984 | 724.7712084 | 0.9113159877 | 已通过纯 NPU 审计 |
| 完整 18 层主干 | 0 / 0，未形成合格样本 | — | — | — | 设备归属审计未通过；strict 缓存构造 SIGSEGV |
| 完整 18 层专家、10 步 | 未测 | — | — | — | 无有效分数 |
| 视觉 + 主干 + 专家整块 | 未测 | — | — | — | 不能报告整块性能 |

视觉原始 warmup 和样本全部保存在 [P1_vision_measured_baseline.json](results/P1_vision_measured_baseline.json)。该结果同时具有 AMD 设备报告的 1031 个 VAIML 节点、ORT 实际 Vitis EP 事件且 CPU=0、VAIML hardware runner、最终编译摘要 CPU=0，`npu_only_proven=true`。仅 providers 列表或 preliminary 支持比例不足以通过审计。

完整主干补齐候选已成功创建 session；AMD 逻辑图报告为 **1140 个 VAIML 节点、0 个 CPU 节点，即逻辑图 CPU=0/1140**，编译摘要也为 CPU=0，但 ORT 实际 profile 出现 **1441 个 CPU 节点、0 个 Vitis 节点，即实际执行图 CPU=1441/1441=100%**。两份报告的节点表示与分母不同，必须并列，不能相加分母求占比；节点占比也不是 CPU 时间占比。该结果 `npu_only_proven=false`、`performance=null`，未作为 NPU 吞吐。[完整主干审计](results/P2_backbone_kv_pad_full_audit.json)

随后同一完整主干候选执行严格缓存复用，进程以 **139 / SIGSEGV（signal 11）** 退出，失败点在首个 strict cache-hit session 构造、session metadata 检查点之前，完成 warmup/sample 均为 **0/0**。没有创建耗时终态或推理 p50/p95，根因仍未知。不能由 debug metadata 文件大小推断存在已确认的 2 GiB 限制。[缓存复用终态](results/P2_backbone_kv_pad_cached_strict.json)、[systemd 日志](evidence/P2_backbone_kv_pad_cached_strict_systemd.log)

已有 core 的离线原生回溯进一步定位顶帧为 `voe/lib/libflexmlrt.so` 中的 `flexmlrt::ryzen_ai_xrt::ryzen_ai_xrt`，调用链经 HWRunner、FlexMLModelRunner、VAIML `MyCustomOpGraphMode`、ORT `FunctionKernel::Create` 至 `InferenceSession::Initialize`，确认在首次推理前构造硬件 runner 时崩溃；该定位不等于根因诊断。[诊断 JSON](results/P2_backbone_cached_crash_diagnosis.json)、[原生回溯](evidence/P2_backbone_cached_crash_bt.txt)

## 优化 A/B 与取舍

目前没有一项具备同负载、单变量、双方 20+20 合格样本的性能 A/B，因此**没有可声称的性能提升百分比**。下表中的兼容性进展只说明功能或设备归属变化；“N/A”表示缺少可比测量，不表示 0% 增益。

| 阶段 | 单项改动 / 对照 | A p50 ms | B p50 ms | 性能增益 | 是否采纳 / 证据 |
|---|---|---:|---:|---|---|
| P1 | 修正动态库搜索顺序，使现有 XRT 2.25 优先于冲突版本 | N/A | N/A | N/A | 作为环境修复保留；不是算力优化 |
| P1 | 主干/专家源图 FP32→FP16，降低编译内存 | N/A，原路径 OOM | N/A | N/A | 仅作为可行性恢复；[baseline](results/P1_bf16_baseline.json)、[recovery](results/P1_bf16_recovery.json) |
| P1 | 独立完整视觉基线 | — | 723.666984 | 基线，无 A/B | 已记录；[20+20 测量](results/P1_vision_measured_baseline.json) |
| P2 尝试1 | K-cache singleton 轴 Transpose→等价 Reshape | N/A | N/A | N/A | CPU 功能完全一致，但后端失败并全 CPU；无效，未作为性能优化采纳 |
| P2 尝试2 | 同一图仅 optimize_level 1→2 | N/A | N/A | N/A | 同一后端 spill 错误、仍全 CPU；无效，回滚 level 1 |
| P2 尝试3 | 同一主干内部 K/V 读取分支 968→1024 补零并屏蔽，公开 shape 保持 968 | N/A | N/A | N/A | 单层零 CPU 成功；完整主干实际全 CPU，strict 缓存构造崩溃；未作为性能优化采纳 |
| P3 | IOBinding / KV 常驻 | 未执行 | 未执行 | 未测 | 未执行 |
| P4 | RMSNorm/QKV、RoPE、GeGLU、attention 胶水融合 | 未执行 | 未执行 | 未测 | 未执行 |
| P5 | 原生 INT8、量化质量与预排布局 | 未执行 | 未执行 | 未测 | 未执行 |
| P6 | 跨块视觉/专家重叠 | 未执行 | 未执行 | 未测 | 未执行 |
| P7 | 固定系统条件并连续 3 次稳定性验收 | 未执行 | 未执行 | 未测 | 只采集了当前条件，未完成该阶段 |

singleton Reshape 和 KV 补齐分别做过一次 CPU 功能对照，三项输出 bitwise 相同；这些检查没有计时，不能计入性能样本，也不能替代 P5 的量化质量验收。[singleton 功能检查](results/P2_singleton_k_functional_check.json)、[补齐功能检查](results/P2_padding_cpu_correctness.json)

磁盘缓存使用当前 SDK 的 `cache_dir`、`cache_key`、`enable_cache_file_io_in_mem=0`。同一完整视觉图曾观测到首次创建 **304.059350 s**、随后缓存创建 **0.590401 s**；正式视觉测量的缓存创建为 **0.733982 s**。这些都是单次 session 创建观测，不是 warmup 后 p50/p95，也不是推理 A/B 增益；它们只证明视觉图的缓存创建低于 10 s，不能宣布 P2 的整块验收通过。[首次编译](results/P1_bf16_baseline.json)、[后续缓存](results/P1_vision_measured_baseline.json)

## 结论与停止条件

**当前软件栈尚未提供值得采用的 π0.5 整块 NPU 部署证据，8 TFLOPS 目标未实现。** 合格的独立视觉实测为 0.91131599 TFLOPS；完整主干在实际 profile 中全 CPU，严格缓存构造又发生进程崩溃。当前停止原因是该完整负载在 Ryzen AI 1.8 运行时/缓存路径的依赖故障，根因未确定；已有视觉纯 NPU 实测证明不能概括为整套 SDK 不可用，也不能断言 NPU 硬件不适合该类负载。

由 proxy 账本可透明推导：5.023337152512 TFLOP/块若达到 8 TFLOPS，整块 p50 需不高于 **627.917144 ms**。当前视觉单阶段 p50 已为 723.666984 ms；在保留该视觉实现且串行执行的假设下，即使其余阶段耗时为零，整块也至多约 **6.941504 TFLOPS**。这只是由视觉时间和账本得到的上界，不是整块实测；它说明后续必须降低或隐藏视觉耗时。P6 跨块重叠会改变此串行假设，且尚未实现。

P2 未完成，且没有完整主干/专家/整块的有效性能样本，因此**没有触发“P2 完成后低于 6.0 TFLOPS”的停止门槛**。P3–P7 未执行，本次按无法在允许范围绕过的当前软件依赖故障停止；没有账号或许可证缺失的证据。应优先修复或替换兼容的官方运行时/缓存组件，再恢复 P2。停止决策、已安装 EP/runtime 的精确二进制 SHA256 与未支持的结论范围见 [P2_stop_decision.json](results/P2_stop_decision.json)。

对近期需要形成可用 π0.5 闭环的部署，建议有条件地优先推进现有 iGPU + PyTorch 路线，以减少对当前失败的 NPU 编译/缓存路径的依赖；其速度、精度和内存仍须独立实测。本报告未运行 iGPU 对照，不能声称其比 NPU 更快。若官方组件修复后完整主干与专家均能零 CPU 执行，再用同口径 20+20 整块样本重新判断 NPU 的部署价值。

## 复现命令

以下从任务工作目录执行；无需云端推理或模型权重。已有可用环境时跳过创建/恢复步骤；交付模型已生成，可跳过 `build_proxy.py`，它仅用于重新生成原始随机图。普通 CPU onnxruntime wheel 会覆盖 AMD wheel，应保持当前环境包来源。环境只读检查使用 `--fingerprint`，保留现有 P0 JSON，避免改动其哈希及缓存身份。

```bash
python3 -m venv work/ryzenai-venv
python3 outputs/bench/restore_cached_sdk.py --cache /home/aup/NPU/deployment/ryzenai/uv-cache --venv work/ryzenai-venv --receipt outputs/evidence/sdk_restore.json
bash outputs/bench/run_sdk.sh python -m pip check
bash outputs/bench/run_sdk.sh python outputs/bench/capture_environment.py --fingerprint
bash outputs/bench/run_sdk.sh python outputs/bench/build_proxy.py --stage all
```

复现已合格的完整视觉阶段测量（20+20），输出使用新名称以保留原始结果：

```bash
systemd-run --user --scope -p MemoryMax=16G -p MemorySwapMax=64M bash outputs/bench/run_sdk.sh python outputs/bench/bench_npu.py --mode proxy --model-dir outputs/bench/proxy_active --only-stage vision --stage P1 --optimization vision_reproduce --fingerprint outputs/results/P0_environment.json --warmup 20 --samples 20 --output outputs/results/P1_vision_reproduce.json
```

对已生成的完整主干候选只做 EP 归属审计；该命令不能替代性能测量：

```bash
systemd-run --user --scope -p MemoryMax=16G -p MemorySwapMax=64M bash outputs/bench/run_sdk.sh python outputs/bench/bench_npu.py --mode ep-report --model-dir outputs/bench/proxy_p2_padded_active --only-stage backbone --stage P2 --optimization backbone_audit_reproduce --fingerprint outputs/results/P0_environment.json --output outputs/results/P2_backbone_audit_reproduce.json
```

每份 JSON 的 `command` 保存其真实运行命令，`manifest` 保存模型 SHA、dtype、shape 与 FLOP 账本，`sessions` 和同名 `_evidence/` 保留创建、AMD 报告、ORT profile 及编译摘要。不同候选重建入口为 `convert_fp16.py`、`patch_kv_layout.py`、`pad_attention_kv.py`，各自 `--help` 列出路径；原始与派生权重/图分别保留。未来性能 A/B 使用 `--compare-to <baseline.json> --changed-variable '<唯一变量>'`；默认低于 3% 不采纳，各阶段额外门槛仍须单独满足。

## results 索引

| 记录 | 用途 / 当前状态 |
|---|---|
| [P0_environment.json](results/P0_environment.json) | 当前环境与 unknown 原因 |
| [P0_accepted_references.json](results/P0_accepted_references.json) | 用户既有实测、规格与目标，未重测 |
| [P1_vision_1layer_compatibility.json](results/P1_vision_1layer_compatibility.json) | 初始单层放置检查；fallback/未证明 |
| [P1_vision_1layer_bf16.json](results/P1_vision_1layer_bf16.json) | 动态库冲突导致硬件上下文失败 |
| [P1_vision_1layer_bf16_xrtpath.json](results/P1_vision_1layer_bf16_xrtpath.json) | XRT 修复后缓存审计证据不足；无性能 |
| [P1_fp16_input_compatibility.json](results/P1_fp16_input_compatibility.json) | FP16 输入源图单层 NPU 兼容性检查 |
| [P1_bf16_baseline.json](results/P1_bf16_baseline.json) | 视觉创建成功，主干初始化 OOM；整块无样本 |
| [P1_bf16_recovery.json](results/P1_bf16_recovery.json) | FP16 恢复后主干发现不支持的 Transpose，停止编译 |
| [P1_vision_measured_baseline.json](results/P1_vision_measured_baseline.json) | 唯一合格 20+20 独立 stage 性能记录 |
| [P2_singleton_k_functional_check.json](results/P2_singleton_k_functional_check.json) | 一次 CPU 功能等价检查，无性能 |
| [P2_singleton_k_layout_smoke.json](results/P2_singleton_k_layout_smoke.json) | 尝试1后端失败，全 CPU；无效 |
| [P2_singleton_k_opt2_smoke.json](results/P2_singleton_k_opt2_smoke.json) | 尝试2 level 2 仍失败，全 CPU；无效 |
| [P2_padding_cpu_correctness.json](results/P2_padding_cpu_correctness.json) | 一次 CPU 补齐等价检查，无性能 |
| [P2_kv_pad1024_smoke.json](results/P2_kv_pad1024_smoke.json) | 补齐候选单层纯 NPU 验证，无性能 |
| [P2_backbone_kv_pad_full_audit.json](results/P2_backbone_kv_pad_full_audit.json) | 完整主干编译成功，但实际 ORT CPU 事件导致审计失败 |
| [P2_backbone_kv_pad_cached_strict.json](results/P2_backbone_kv_pad_cached_strict.json) | strict 缓存 session 构造 SIGSEGV，exit 139；warmup/sample 0/0 |
| [P2_backbone_cached_crash_diagnosis.json](results/P2_backbone_cached_crash_diagnosis.json) | native_crash_localized：已有 core 定位至 HWRunner 构造，不运行推理，不计性能 |
| [P2_stop_decision.json](results/P2_stop_decision.json) | 当前依赖故障停止、P2 未完成、8 TFLOPS 未实现；二进制版本与 SHA256 |

完整结果索引见 [results/INDEX.md](results/INDEX.md)。所有正式数字均可回溯到以上 JSON 与同名证据目录。设备计数后处理保留原始 AMD/ORT 报告和性能值，并分别使用逻辑图与执行图的分母；修改记录保存在 JSON 的 `postprocessing_history`。`performance=null` 表示没有可报告的性能结果，不能填零、推算吞吐或当作 NPU 性能。
