π0.5 真实 NPU 视觉离线部署

最新修复结果见 /home/aup/NPU/pi05_vision_npu/results/REPAIR_REPORT.txt
完整机器可读结果见 /home/aup/NPU/pi05_vision_npu/results/repair_final/report.json
当前状态：固定 tp_size=1、静态 batch=1，两路相机依次执行；半通道归零和重复运行漂移已消除。
同一真实样本的完整 50×7 动作块已运行，预先设定的数值回归检查通过。
与原 GPU 的视觉余弦 0.999923994，最大关节差 0.655931°，夹爪差 0.156311 mm。
当前没有提速：3 次计时中位数 GPU 1.479 s，NPU+GPU 1.608 s。未验证机械臂任务成功率或动作容差。

一条复现命令（输出目录必须尚不存在）：
bash /home/aup/NPU/pi05_vision_npu/run_offline.sh --vision both --warmup 2 --samples 3 --output-dir /home/aup/NPU/pi05_vision_npu/results/repair_reproduce_01
默认选择 models/vision_batch1.onnx 和 code/vaiml_tp1.json；当前已测运行 exit 0。
数值回归失败仍保存原始结果并返回 exit 2，不通过调整预测值或放宽动作限值掩盖差异。
回归门槛：特征有限、无半零异常、cosine>=0.9999、relative RMSE<=0.02，重复特征/动作完全一致。
这是单样本工程回归门槛，不是机械臂控制精度/任务成功验收。

切换：--vision gpu / --vision npu / --vision both。
默认复用本任务保存的 dataset episode 46 frame 0 真实录制输入；同一 seed 0，使用 torch 原 sample_noise。
NPU 是每次预测实际调用的持久进程，无特征缓存替代计算；global、wrist 各调用一次，再按原顺序交给原 embed_prefix。
--recorded 可指定另一个同格式 .npz（observation.state、observation.images.global、observation.images.wrist、task）。

导出另一录制帧（所有输出仍放任务目录）：
/home/aup/Lerobot/envs/piper-cpu/bin/python /home/aup/NPU/pi05_vision_npu/code/export_recorded_input.py --dataset /home/aup/Lerobot/datasets/rad_place_merged_20261030 --episode 46 --frame 1 --output /home/aup/NPU/pi05_vision_npu/results/another_frame.npz
然后在复现命令中添加 --recorded /home/aup/NPU/pi05_vision_npu/results/another_frame.npz。

环境：
GPU Python: /home/aup/Lerobot/envs/pi05-rocm-7.14.1/bin/python
NPU Python: /home/aup/Documents/Codex/2026-10-30/w/work/ryzenai-venv/bin/python
NPU 的 XRT/VAIML 动态库路径由 code/run_npu_env.sh 单独设置，避免混入 ROCm 进程。
精确版本与设备见 results/npu_packages.json、results/xrt_examine.txt、results/environment.json。
静态 NPU 图每次输入 [1,3,224,224]；对接模块每个动作块实际调用两次；本地策略 observation batch=1。

真实模型与导出：
基座 /home/aup/Lerobot/models/PI05/pi05_base/，LoRA 路径见 code/common.py。
models/vision_batch1.onnx + vision_batch1.data 是默认真实视觉图与 FP32 权重。
models/vision_batch1.manifest.json 包含全部 439 个张量映射/哈希和操作统计；与原导出的实际权重哈希一致。
models/vision.onnx、vision_patchify.* 及 results/final_run、REPORT.txt 保留为旧失败实验，不能作为最新状态。
旧错误配置可用 --npu-model /home/aup/NPU/pi05_vision_npu/models/vision.onnx --npu-config /home/aup/NPU/pi05_vision_npu/code/vaiml_config.json 复现；预计 exit 2。
如需重导出：
bash /home/aup/NPU/pi05_vision_npu/code/run_npu_env.sh python /home/aup/NPU/pi05_vision_npu/code/export_vision.py
如需单独检查 FP32 导出等价性：
bash /home/aup/NPU/pi05_vision_npu/code/run_npu_env.sh python /home/aup/NPU/pi05_vision_npu/code/verify_export.py
code/verify_patch_stem.py 可复现单算子对照；使用 --config code/vaiml_tp1.json --batch 1 --only-conv 得到修复配置（config 需绝对路径）。
results/repair_configuration_matrix.json 保存 TP/batch 对照，results/repair_source_checks.json 保存原文件及权重一致性检查。
首次创建 NPU session 会编译；后续复用 cache/npu/。不要把首次编译计入稳定推理时间。
部署采用 AMD VAIML BF16 路线，配置参照 AMD 官方文档：https://ryzenai.docs.amd.com/en/1.8/modelrun.html；是否正确以本任务实测为准。

代码：common.py 复用原严格低内存加载和原处理器；export_vision.py 导出真实权重；npu_worker.py 执行/审计；npu_bridge.py 对接；run_offline.py 固定噪声对照。
既有加载器和审计器的逐字副本及 SHA 见 results/reused_sources.json。
离线开发，不含机械臂、CAN、串口、实时摄像头或运动发送流程。
