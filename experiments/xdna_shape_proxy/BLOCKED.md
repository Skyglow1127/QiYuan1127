# 阻塞：完整主干的 Ryzen AI 1.8 运行时 / 缓存路径

**任务停止在 P2，未达到 8 effective TFLOPS。** 该阻塞针对当前 SDK 处理完整静态 shape proxy 的路径；已有完整视觉阶段纯 NPU 20+20 测量成功。P2 尚未完成，不能触发“P2 完成后低于 6 TFLOPS”的停止门槛，也不能据此断言 NPU 硬件上限或硬件不适合该负载。[终态决策](results/P2_stop_decision.json)

## 已确认现象

1. 完整 18 层主干内部 KV 补齐候选可创建 session。AMD 逻辑图报告 1140 个 VAIML 节点 / 0 个 CPU 节点，最终编译摘要也给出 CPU=0；但 ORT 实际 profile 为 **1441 个 CPU 节点 / 0 个 Vitis 节点**。两份报告的图表示不同，不能合并计数。完整主干的 `npu_only_proven=false`，`performance=null`。[完整审计](results/P2_backbone_kv_pad_full_audit.json)
2. 保留同一完整主干与编译缓存、改为严格禁止 CPU fallback 的后续运行，在首个 cache-hit session 构造期间以 **exit 139 / SIGSEGV，signal 11** 结束。发生在 session metadata 检查点之前，warmup 和正式样本完成数均为 **0**，没有推理 p50/p95 或有效算力。[崩溃终态](results/P2_backbone_kv_pad_cached_strict.json)、[程序日志](evidence/P2_backbone_kv_pad_cached_strict.log)、[systemd 日志](evidence/P2_backbone_kv_pad_cached_strict_systemd.log)
3. 单层 KV 补齐候选已通过纯 NPU 审计，不能据此外推完整 18 层的可执行性。原始 singleton Reshape、编译 optimize_level 2 的单变量尝试均遇到后端 spill 错误并落 CPU，已记录无效并回滚 level 1。[单层补齐审计](results/P2_kv_pad1024_smoke.json)、[尝试1](results/P2_singleton_k_layout_smoke.json)、[尝试2](results/P2_singleton_k_opt2_smoke.json)

已有 core 的离线原生回溯顶帧为 `voe/lib/libflexmlrt.so` 的 `flexmlrt::ryzen_ai_xrt::ryzen_ai_xrt`，调用链为 HWRunner → FlexMLModelRunner → VAIML `MyCustomOpGraphMode` → ORT `FunctionKernel::Create` → `InferenceSession::Initialize`，确认首次推理之前的硬件 runner 构造 SIGSEGV；根因仍未确定。[诊断 JSON](results/P2_backbone_cached_crash_diagnosis.json)、[回溯文本](evidence/P2_backbone_cached_crash_bt.txt)

现有信息不足以确定 SIGSEGV 根因。较大 debug metadata 文件仅是观察，不是已确认的 2 GiB RAI/partition 限制。不同 SDK 包中同名动态库具有不同哈希，也不能单凭这一点认定加载错误。

## 固定环境与关键文件

Ubuntu 24.04.2 / kernel 6.14.0-27；HX 370 / Strix aie2p；XRT 2.25.37；amdxdna `2.25.260102.56.release_20260630,88dda53eee336c998abca38c71d5bc79fb3633b8`；固件 1.1.2.64；Ryzen AI / VOE 1.8.0；onnxruntime-vitisai 1.27.0。系统 performance、NPU default，无电源修改。完整指纹和已测二进制大小/SHA256 见 [P2_stop_decision.json](results/P2_stop_decision.json)，SDK wheel 恢复来源见 [sdk_restore.json](evidence/sdk_restore.json)。

模型：`outputs/bench/proxy_fp16_static_kv_pad/backbone_proxy.onnx`

```text
ONNX SHA256: 817d0c7487b0e7151f29aa313a436579777f0f338dd530af4ae2ff083da910ba
external data SHA256: d980cb2f382049802bcfa5ac318250e953cc388502120137ecf2be708fafeeaa
公开输入: tokens [1,968,2048] float16
公开输出: hidden [1,968,2048]，18 层各 K/V [1,1,968,256] float16
cache_dir: work/npu-cache/backbone/60ef203689f7675111ab
cache_key: pi05_backbone_60ef203689f7675111ab
config: outputs/bench/vai_bf16_from_fp16_config.json
config SHA256: 54ab888b9f6285a8ea4cc7cd6081c8f414df58a2ddad4a60d75ea3d3345d8f56
```

模型由 `proxy_p2_padded_active/manifest.json` 指向上述实际文件。权重独立随机，不含待下载的真实策略权重。配置使用官方 VAIML、`optimize_level=1`、`preferred_data_storage=auto`、`enable_f16_to_bf16_conversion=true`；磁盘缓存参数为 `cache_dir`、`cache_key`、`enable_cache_file_io_in_mem=0`。

## 精确复现步骤

以下命令是故障复现说明，本次已停止继续运行。工作目录为 `/home/aup/Documents/Codex/2026-10-30/w`。

先只读确认环境：

```bash
bash outputs/bench/run_sdk.sh python outputs/bench/capture_environment.py --fingerprint
bash outputs/bench/run_sdk.sh python -m pip check
```

对应已完成的首次编译/实际执行审计：

```bash
systemd-run --user --scope --unit=pi05-npu-p2-backbone.scope -p MemoryMax=16G -p MemorySwapMax=64M bash outputs/bench/run_sdk.sh python outputs/bench/bench_npu.py --mode ep-report --model-dir outputs/bench/proxy_p2_padded_active --only-stage backbone --stage P2 --optimization backbone_kv_pad_full_audit --fingerprint outputs/results/P0_environment.json --output outputs/results/P2_backbone_kv_pad_full_audit.json
```

保留已有缓存后，对应崩溃的严格复用命令：

```bash
systemd-run --user --scope --unit=pi05-npu-p2-backbone-cached.scope -p MemoryMax=16G -p MemorySwapMax=64M bash outputs/bench/run_sdk.sh python outputs/bench/bench_npu.py --mode proxy --model-dir outputs/bench/proxy_p2_padded_active --only-stage backbone --stage P2 --optimization backbone_kv_pad_cached_strict --fingerprint outputs/results/P0_environment.json --output outputs/results/P2_backbone_kv_pad_cached_strict.json
```

这些命令复用记录名称，重新执行会覆盖相应 JSON；复现前应保留本次 JSON 和同名 `_evidence/`。查看既有 systemd 证据无需重新触发崩溃。

## 恢复所需文件与通过条件

需要 AMD 针对上述确切模型/图、环境、实际加载二进制和缓存复现，提供兼容本机 XRT/固件的官方 EP/VAIML/runtime 修复包，或正式文档支持的绕过方法。具体补丁文件名、版本和许可证要求尚未知，不编造安装包。现有 SDK wheel 已在本地，不存在已确认的账号或下载许可证阻塞；不要通过随机替换动态库掩盖问题。

提交给维护者的最小材料为：模型 ONNX 与 external data、对应 manifest 与配置、环境/二进制 SHA256、两个 P2 终态 JSON、AMD 报告、ORT profile、编译摘要、崩溃和 systemd 日志。只保留不含真实模型权重的随机 proxy 即可；本任务没有授权向维护者发送材料。

获得修复后，应先在同一完整主干上确认实际 ORT Vitis 执行、CPU=0、VAIML hardware runner 和最终内部 CPU=0 同时成立，再确认 strict 缓存 session 可加载。随后验证完整专家并完成整块 20 warmup +20 样本，才继续评估 P2 的 6 TFLOPS 门槛及 P3–P7。当前 P3–P7 均未执行，原生 INT8 和量化余弦要求也未验收。
