"""Summarize measured repair data, retaining the original failed run."""
import json
from common import ROOT,write_json

out=ROOT/'results/repair_final'
r=json.loads((out/'report.json').read_text())
old=json.loads((ROOT/'results/final_run/report.json').read_text())
comp=json.loads((ROOT/'results/repair_full_compile/session.json').read_text())
audit=json.loads((out/'npu_session/execution_audit.json').read_text())
f=r['feature_error'];s=r['npu_stage_seconds'];actions=r['action_error']['per_axis']
ms=lambda v: f"{v['median']*1000:.3f}"
lines=[
 'π0.5 NPU 数值问题修复验证（2026-11-04）',
 f"数值回归通过：{r['passed']}。本次仅一个真实录制样本，未验证机械臂任务成功率。",
 '',
 '实施方式：真实 SigLIP + 投影重新导出为静态 batch=1；VAIML 配置 tp_size=1。',
 'NPU 依次执行 global、wrist 两次，然后拼回 [2,256,2048] FP32 特征接入原 GPU π0.5 + 020000 LoRA。',
 'CPU 只做运行时转换、IPC、两份特征的拼接及原有管理工作；没有 CPU 视觉回退、补通道或修改动作值。',
 '这是针对当前环境已验证的编译/运行配置规避；尚未定位 AMD 内部导致 TP=2 或 batch=2 异常的具体源代码。',
 '',
 '定位证据：results/repair_configuration_matrix.json 保存同一真实卷积的 TP/batch 对照。',
 'TP=1,batch=2 消除零通道，但第二张相机仍有明显误差；TP=1,batch=1 的单层余弦为 0.9999906444。',
 '绑定 NaN 初始化输出仍得到 50% 零，说明并非 Python 未读完缓冲区；运行时返回转换前的 BF16 缓冲区已存在零。',
 'BF16/FP32 转换函数单独测试通过；替换为已有另一份 libflexmlrt 未消除问题。',
 '纯 BF16 I/O 探针编译失败：gemm_bf16xbfp16 未声明。直接 FlexML 探针缺少 aiecompiler PATH，未作为运行证据。',
 '',
 '完整视觉特征（与原 GPU FP32 比）：',
 f"修复前 cosine={old['feature_error']['cosine']:.10f}；修复后 cosine={f['cosine']:.10f}",
 f"max_abs={f['max_abs']:.9g}, mean_abs={f['mean_abs']:.9g}, RMSE={f['rmse']:.9g}",
 f"精确零比例={r['output_zero_diagnostic']['npu_zero_fraction']}; 重复特征最大差={r['npu_feature_repeat_error']['max_abs']}",
 f"数值门槛：cosine>=0.9999，relative_RMSE<=0.02，有限值，无半零异常，重复特征和动作完全一致。",
 f"这些是工程回归门槛，不代表已满足任何未指定的机械臂动作容差。",
 '',
 '同一双相机、状态、文字、seed=0 初始噪声，动作 [50,7]，六关节 degree，夹爪 metre。',
 '逐轴 |NPU-GPU|（最大值 / 平均值）：',
]
for name,v in actions.items():lines.append(f"{name}: {v['max_abs']:.9g} / {v['mean_abs']:.9g} {v['unit']}")
lines += [
 f"原 GPU 入口复现最大差={r['original_gpu_baseline_error']['max_abs']}; NPU 动作重复最大差={r['npu_repeat_error']['max_abs']}",
 '',
 f"预热每路径 {r['warmup_per_path']} 次，计时每路径 {r['samples_per_path']} 次；以下为中位数 ms：",
 f"GPU 双相机视觉 {ms(r['gpu_vision_seconds'])}",
 f"NPU 两次 session.run 合计 {ms(s['npu_ort_seconds'])}（包含内部拷贝和同步）",
 f"GPU pack/D2H/sync {ms(s['gpu_pack_sync_d2h_seconds'])}; host concat {ms(s['worker_batch_concat_seconds'])}",
 f"IPC/copy/control {ms(s['ipc_copy_and_control_seconds'])}; H2D/cast/sync {ms(s['h2d_cast_sync_seconds'])}",
 f"视觉完整往返 {ms(s['vision_roundtrip_seconds'])}",
 f"GPU 完整动作块 {ms(r['complete_chunk_seconds']['gpu'])}; NPU+GPU 完整动作块 {ms(r['complete_chunk_seconds']['npu'])}",
 f"GPU耗时/混合耗时={r['speedup_gpu_over_hybrid']:.6f}（大于1才表示混合路径更快）。",
 f"首次编译/创建 {comp['session_create_seconds']:.3f}s；首次双图像视觉 {comp['cold_run_seconds']:.3f}s。",
 f"本次有既有 Vivado 构建同时运行；未终止或修改。计时为当前负载下的小样本结果。",
 '',
 f"真实 NPU 执行审计通过：{audit['npu_only_proven']}。完整报告：{out/'npu_session/execution_audit.json'}",
 f"AMD 分配 NPU 节点={audit['amd']['npu_node_count']}，CPU 节点={audit['amd']['cpu_node_count']}；ORT 与硬件执行证据保留在同目录。",
 '原始动作、特征、初始噪声、profile、算子分配均在 results/repair_final/。',
 '旧失败结果保留于 results/final_run/，旧 REPORT.txt 是修复前的历史报告。',
 '真实权重与原文件一致性证据：results/repair_source_checks.json。',
 '',
 '复现命令（输出目录必须尚不存在）：',
 f"bash {ROOT}/run_offline.sh --vision both --warmup 2 --samples 3 --npu-model {ROOT}/models/vision_batch1.onnx --npu-config {ROOT}/code/vaiml_tp1.json --output-dir {ROOT}/results/repair_reproduce_01",
 '仅 GPU 基线：同上改 --vision gpu，使用新的输出目录。',
 '无需新增依赖、系统升级或驱动重启。NPU 仍为现有 Ryzen AI 1.8 环境，GPU 仍为原 ROCm 环境。',
 '所有执行均离线，无机械臂、CAN、串口或 PL 调用。',
]
(ROOT/'results/REPAIR_REPORT.txt').write_text('\n'.join(lines)+'\n')
write_json(ROOT/'results/repair_summary.json',dict(
    report=str(out/'report.json'),numerical_regression=r['numerical_regression'],
    feature_error=f,action_error=r['action_error'],speedup_gpu_over_hybrid=r['speedup_gpu_over_hybrid'],
    workaround=dict(tp_size=1,device_batch=1,camera_calls=2),npu_only_proven=audit['npu_only_proven']))
print('\n'.join(lines))
