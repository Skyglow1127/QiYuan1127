"""Write a compact, evidence-linked Chinese delivery report from measured JSON."""
import argparse
import json
from pathlib import Path
from common import ROOT

def read(p): return json.loads(Path(p).read_text())
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--run',type=Path,default=ROOT/'results/integrated')
    args=ap.parse_args(); run=args.run.resolve()
    r=read(run/'report.json'); assert r.get('execution_passed',r['passed']) and r['paths']=='both'
    audit=read(run/'npu_session/execution_audit.json')
    first=read(ROOT/'results/npu_first_compile/session.json')
    export=read(ROOT/'results/reference/export_equivalence.json')
    manifest=read(ROOT/'models/vision.manifest.json')
    def ms(x): return f'{x*1000:.3f}'
    text=[
        'π0.5＋020000 LoRA：真实 NPU 视觉＋ROCm GPU 动作推理实测',
        f'目录：{ROOT}',
        f'本次完整运行：{run}',
        '',
        '状态：真实 NPU→GPU 离线链路已闭环，但数值验收失败，当前不能作为合格的视觉部署使用。',
        '同一真实双相机样本、状态、文字和初始噪声，两条路径均生成有限的 50×7 原始动作块；NPU 结果存在确定的数值故障。',
        f'NPU 全视觉硬件执行证据通过：{audit["npu_only_proven"]}。全链时间比 GPU/混合路径 = {r["speedup_gpu_over_hybrid"]:.4f}，仅是本次 3 样本原始时间比；错误结果不能用于宣称可用加速。',
        '',
        '输入与语义',
        '数据集 rad_place_merged_20261030，episode 46，frame 0；相机顺序 global → wrist。',
        '保留原 LeRobot 预处理：480×640、RGB、CHW、[0,1] → 原 resize_with_pad_torch → 224×224、[-1,1]、FP32。',
        '两个真实图像 mask 均为 true；每图 256 token；送回 GPU 的投影特征为 FP32 [2,256,2048]。',
        '原文字 tokenizer、状态量化进文字、掩码、主干、动作专家、80 个 LoRA 张量、10 次去噪及动作反归一化均沿用现有实现。',
        '动作顺序 joint_1.deg … joint_6.deg，gripper.m；绝对关节目标，前六轴 degree、末轴 metre。',
        '',
        '真实权重和精度',
        f'导出 {manifest["source_tensors"]} 个真实 FP32 张量；27 层、宽 1152、16 头、patch 14；包含 patch 卷积、位置嵌入、全部编码层、post layer norm 和 1152→2048 投影。',
        '无随机代理权重；NPU 使用 VAIML BF16 编译，接口输入/输出仍为 FP32。未使用 INT8，也未做整数校准。',
        f'FP32 ONNX/原 GPU：max_abs={export["max_abs"]:.9g}，mean_abs={export["mean_abs"]:.9g}，cosine={export["cosine"]:.12f}。',
    ]
    f=r['feature_error']
    text.append(f'BF16 NPU/原 GPU：max_abs={f["max_abs"]:.9g}，mean_abs={f["mean_abs"]:.9g}，RMSE={f["rmse"]:.9g}，cosine={f["cosine"]:.12f}。')
    for name,v in f['by_camera'].items():
        text.append(f'  {name}：max_abs={v["max_abs"]:.9g}，mean_abs={v["mean_abs"]:.9g}，cosine={v["cosine"]:.12f}。')
    text.append('上述 NPU 特征与下面 npu_actions.npy 均来自第一个稳定测量样本；npu_features_samples.npy 与 npu_actions_samples.npy 按相同样本索引一一对应。')
    text += ['', '动作差异（NPU 视觉路径减原 GPU，未经修改/裁剪/平滑）', '轴 | 单位 | 最大绝对误差 | 平均绝对误差 | RMSE | 平均有符号误差']
    for name,v in r['action_error']['per_axis'].items():
        text.append(f'{name} | {v["unit"]} | {v["max_abs"]:.9g} | {v["mean_abs"]:.9g} | {v["rmse"]:.9g} | {v["signed_mean"]:.9g}')
    a=r['action_error']
    text += [f'整体数值差异：max_abs={a["max_abs"]:.9g}，mean_abs={a["mean_abs"]:.9g}，RMSE={a["rmse"]:.9g}；包含混合单位，物理解读以逐轴结果为准。',
        f'独立包装的 GPU 路径与最初原入口动作最大误差：{r["original_gpu_baseline_error"]["max_abs"]:.9g}。',
        f'相同输入/噪声的第一与第三测量样本：GPU 动作最大数值变化 {r["gpu_repeat_error"]["max_abs"]:.9g}；NPU 动作最大数值变化 {r["npu_repeat_error"]["max_abs"]:.9g}（混合单位，原始每轴样本已保存）。',
        '', '性能（单位 ms；每条路径预热 2 次、测量 3 次；报告中位数，原始每次时间在 report.json）',
        f'GPU 原视觉、双相机顺序执行：{ms(r["gpu_vision_seconds"]["median"])}',
        f'NPU 视觉 session.run：{ms(r["npu_stage_seconds"]["npu_ort_seconds"]["median"])}',
        f'GPU 打包＋同步＋D2H：{ms(r["npu_stage_seconds"]["gpu_pack_sync_d2h_seconds"]["median"])}',
        f'Unix socket 拷贝及控制：{ms(r["npu_stage_seconds"]["ipc_copy_and_control_seconds"]["median"])}',
        f'H2D＋类型转换＋同步：{ms(r["npu_stage_seconds"]["h2d_cast_sync_seconds"]["median"])}',
        f'NPU 视觉含完整往返：{ms(r["npu_stage_seconds"]["vision_roundtrip_seconds"]["median"])}',
        f'原 GPU 完整动作块：{ms(r["complete_chunk_seconds"]["gpu"]["median"])}',
        f'NPU 视觉＋GPU 完整动作块：{ms(r["complete_chunk_seconds"]["npu"]["median"])}',
        '完整块计时包含原图像预处理、视觉、必要 IPC、主干、10 次专家去噪、原后处理和动作返回 CPU；两者均排除预先完成的 tokenizer 处理。',
        'session.run 包含 ORT 内部数据搬运、设备传输和同步，不能解释为纯 NPU kernel 时间；SDK 未提供这些内部传输的独立分解。',
        '阶段中位数来自分别计时，不将它们相加作为完整块中位数。单次输入 1,204,224 字节，输出 4,194,304 字节。',
        '', '首次编译/加载/冷运行（与上述稳定样本分开）',
        f'首次 NPU session 创建（含编译）：{first["session_create_seconds"]:.6f} s；首次 session.run：{first["cold_run_seconds"]:.6f} s。',
        f'本次缓存 NPU session 创建：{r["npu_session"]["session_create_seconds"]:.6f} s；worker 启动及审计总计：{r["npu_worker_start_seconds"]:.6f} s。',
        f'GPU 基座＋LoRA 加载：{r["loading"]["base_plus_adapter_load_seconds"]:.6f} s。',
        f'本次完整路径首块：GPU {r["cold_complete_chunks"]["gpu"]["seconds"]:.6f} s，混合 {r["cold_complete_chunks"]["npu"]["seconds"]:.6f} s。',
        '', '实际执行证据',
        f'AMD 分配报告：NPU={audit["amd"]["npu_node_count"]}、CPU={audit["amd"]["cpu_node_count"]} 个逻辑节点。',
        f'ORT 实际 profile：Vitis EP={audit["ort"]["npu_node_count"]}、CPU EP={audit["ort"]["cpu_node_count"]} 个执行节点（融合后口径，与逻辑节点数不同）。',
        f'VAIML hardware runner={audit["compiler"]["hardware_runner_proven"]}；最终后端内部 CPU offload 为零={audit["compiler"]["compiler_cpu_offload_zero_proven"]}。',
        'CPU 负责 tokenizer、Python 调度、socket、NumPy 包装和结果接收/保存；原 resize/归一化、token 嵌入、语言主干、动作专家及动作反归一化随动作张量在 GPU，最后按原 device processor 返回 CPU。视觉图没有 CPU fallback。',
        f'证据：{run}/npu_session/execution_audit.json、ort_vision_*.json、compiler_summaries/；原 AMD 报告路径也记录于审计 JSON。',
        '', '交付与限制',
        f'{run}/gpu_actions.json、npu_actions.json：各 50×7 原始动作；同时提供 .npy、.csv 和全部重复样本。',
        f'{run}/gpu_features.npy、npu_features.npy、initial_noise.npy、preprocessed_pixels.npy、input.json、report.json。',
        '只有一个真实录制样本，精度差异不能代表任务成功率；此次未执行机械臂、未调用 π0.5→PL 接口。',
        '保留 GPU 视觉切换能力和全部原始预测；未修改 LeRobot 源码、原模型、ACT 或 FPGA 工程。',
        '本机已有 ROCm 与 Ryzen AI 环境已实际验证可用；本任务未安装或升级依赖。',
    ]
    text += ['', '已定位的实际阻塞与有限修复实验',
        '完整视觉输出 FP32 [2,256,2048] 的后 1024 个通道全部为精确零，GPU 参考没有该现象；同一会话多次运行复现。',
        '相同输入和初始噪声下 NPU 结果还出现重复漂移；GPU 参考逐值稳定。漂移本身也属于未解决的数值问题，不将其解释成模型随机采样。',
        '真实单算子 patch Conv 小图同样有一半通道精确归零，而 CPU 同图与 GPU 最大差异约 7.6e-6。',
        '纯 GPU BF16 完整视觉与 GPU FP32 余弦相似度为 0.999984454，远好于 NPU，不能把 NPU 故障归结为一般 BF16 舍入。',
        '已验证未修复的设置：显式 FP32→BF16、batch=1、FP16 源图/接口、unvectorized 布局、原生 BF16 权重/Conv＋边界 Cast（opset 22）。',
        '相关原始对照：results/precision_diagnostic/、stage_diagnostic/、stem_diagnostic/、stem_explicit_bf16/、stem_batch1/、stem_fp16/、stem_unvectorized/、stem_native_bf16/。',
        '等价 patchify＋MatMul 的 FP32 CPU 导出检查通过（cos≈1），但 NPU 小图也异常，完整候选没有被采纳。',
        '该候选首次完整编译在打包阶段因 original-model-signature.txt 缺失退出 134；两个任务内编译进程共享 CWD 临时文件导致冲突。包装现已改为每次调用独立 CWD，后续小图和最终缓存运行均使用隔离目录。这个编译冲突不作为数值故障的根因。',
        '原生 BF16 的 Conv 在 opset 17 被 ORT 类型检查拒绝，已改为支持 BF16 Conv 的 opset 22 后实际运行，仍复现半零输出。',
        '当前只确定故障范围为本机 Ryzen AI 1.8/ORT Vitis EP 的这些真实视觉图数值路径；底层根因未确认，不泛化为所有 NPU 模型均不可用。需要修复/替换相关编译运行组件后，重新通过真实权重数值检查。',
        '未修改或裁剪动作来缩小误差；未放宽动作限值；未对接 PL、未伪造来源时间。',
        '最终命令仍完成全部动作和性能记录，然后以 exit 2 明确报告数值验收失败；--vision gpu 正常返回。',
    ]
    (ROOT/'results/REPORT.txt').write_text('\n'.join(text)+'\n')
    readme=f'''π0.5 真实 NPU 视觉离线部署

本次结果见 {ROOT}/results/REPORT.txt
完整机器可读结果见 {run}/report.json
当前状态：真实 NPU 计算和 50×7 离线对接已运行，但当前 SDK 出现半通道归零的数值故障。精度未通过，不能视为合格替代。

一条复现命令（输出目录必须尚不存在）：
bash {ROOT}/run_offline.sh --vision both --warmup 2 --samples 3 --output-dir {ROOT}/results/reproduce_01
已知当前环境会在保存全部原始动作/特征/计时后返回 exit 2，报告 numerical_fault=true；此退出码是实际数值问题的复现结果。

切换：--vision gpu / --vision npu / --vision both。
默认复用本任务保存的 dataset episode 46 frame 0 真实录制输入；同一 seed 0，使用 torch 原 sample_noise。
NPU 是每次预测实际调用的持久进程，无特征缓存替代计算；双相机批量调用后保持 global→wrist 顺序交给原 embed_prefix。
--recorded 可指定另一个同格式 .npz（observation.state、observation.images.global、observation.images.wrist、task）。

导出另一录制帧（所有输出仍放任务目录）：
/home/aup/Lerobot/envs/piper-cpu/bin/python {ROOT}/code/export_recorded_input.py --dataset /home/aup/Lerobot/datasets/rad_place_merged_20261030 --episode 46 --frame 1 --output {ROOT}/results/another_frame.npz
然后在复现命令中添加 --recorded {ROOT}/results/another_frame.npz。

环境：
GPU Python: /home/aup/Lerobot/envs/pi05-rocm-7.14.1/bin/python
NPU Python: /home/aup/Documents/Codex/2026-10-30/w/work/ryzenai-venv/bin/python
NPU 的 XRT/VAIML 动态库路径由 code/run_npu_env.sh 单独设置，避免混入 ROCm 进程。
精确版本与设备见 results/npu_packages.json、results/xrt_examine.txt、results/environment.json。
静态 NPU 图只支持一次两张 224×224 RGB 图像；本地策略 observation batch=1。

真实模型与导出：
基座 /home/aup/Lerobot/models/PI05/pi05_base/，LoRA 路径见 code/common.py。
models/vision.onnx + vision.data 是默认真实视觉图与 FP32 权重。vision_patchify.* 是保留的未采纳候选，不是默认路径。
models/vision.manifest.json 包含全部 439 个张量映射/哈希和操作统计。
如需重导出：
bash {ROOT}/code/run_npu_env.sh python {ROOT}/code/export_vision.py
如需单独检查 FP32 导出等价性：
bash {ROOT}/code/run_npu_env.sh python {ROOT}/code/verify_export.py
code/verify_patch_stem.py 可复现单算子故障，code/diagnose_native_bf16.py 可复现原生 BF16 对照。
首次创建 NPU session 会编译；后续复用 cache/npu/。不要把首次编译计入稳定推理时间。
部署采用 AMD VAIML BF16 路线，配置参照 AMD 官方文档：https://ryzenai.docs.amd.com/en/1.8/modelrun.html；是否正确以本任务实测为准。

代码：common.py 复用原严格低内存加载和原处理器；export_vision.py 导出真实权重；npu_worker.py 执行/审计；npu_bridge.py 对接；run_offline.py 固定噪声对照。
既有加载器和审计器的逐字副本及 SHA 见 results/reused_sources.json。
离线开发，不含机械臂、CAN、串口、实时摄像头或运动发送流程。
'''
    (ROOT/'README.txt').write_text(readme)
    print(ROOT/'results/REPORT.txt')

if __name__=='__main__': main()
