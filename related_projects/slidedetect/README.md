# KR260 拉莫尔信号 SFLS 测频仿真工程

**导师指定的正式论文工程请进入 [formal_reproduction](formal_reproduction/README.md)。** 该版本采用论文约 2.1 kHz 输入和 200 MHz 计数，包含含噪声模型、参数出处、独立重复试验及更细的 FPGA 读出。原文 35 dB 精度仍有未解释条件，尚未宣称全文复现成功。以下内容保留早期 70–350 kHz 工程的原有范围。

Vivado 2025.2 / XSim，输入 70–350 kHz，100 MHz 时间戳计数，1 kHz 结果更新，全 FPGA 定点实现。

**请先看结论边界：**普通窗口不保证全量程 0.001 Hz；长窗口模式在本项目静态测试中达到毫赫兹级，但需要约 6–30 秒观察时间。1 kHz 更新率不等于 1 kHz 测量带宽。实际噪声和时基准确度另计。

## 文件入口

- `reports/verification.html`：最终图文验证报告（运行汇总脚本生成）。
- `docs/DESIGN.md`：算法、位宽、接口、窗口代价、误差上界、实现边界。
- `build/vivado/sfls_kr260.xpr`：可打开的 Vivado 工程。
- `reports/end_to_end.wdb`：常规窗口的真实时钟仿真波形。
- `rtl/`：可综合 SystemVerilog，所有求频运算在硬件内完成。
- `sim/`：自检 testbench 和可重建测试向量。
- `model/`：激励与独立直接 LS oracle，不替代硬件求频。
- `reports/`：准确度、分辨率测试、动态响应、布局布线时序和资源报告。
- `scripts/`：从干净目录重建项目和重跑测试的入口。

## 环境

本次验证环境：Ubuntu 24.04.2，Vivado 2025.2（SW Build 6299465），KR260 器件 `xck26-sfvc784-2LV-c`。Python 3、NumPy、Matplotlib 用于激励、参考运算与绘图。

脚本默认安装目录为 `/home/aup/Xilinx/2025.2/Vivado`。其他机器设置：

```bash
export VIVADO_ROOT=/your/install/2025.2/Vivado
```

Python 依赖：`numpy`、`matplotlib`。没有 Vitis、PetaLinux 或实际 KR260 连接也可完成本工程仿真。

交付压缩包不包含可重建的大型时间戳文本向量、仿真器二进制缓存和路由检查点；首次运行回放测试前先执行下面两个 `generate` 命令。默认端到端 testbench 的输入由自身产生，无需这些文本向量。

## 重现步骤

从仓库根目录执行；这些脚本只操作自己的 `build/`、`reports/`、`sim/vectors/`，重跑会覆盖对应生成结果。

一键执行全部验证：`bash scripts/run_all.sh`。也可以按下面的命令分步运行。

```bash
python3 model/reference.py generate
python3 model/extended.py generate

bash scripts/run_unit.sh
bash scripts/run_replay.sh small
bash scripts/run_replay.sh stride1
bash scripts/run_replay.sh stride64
bash scripts/run_replay.sh extended1
bash scripts/run_replay.sh extended64

bash scripts/run_end_to_end.sh small
bash scripts/run_end_to_end.sh full
bash scripts/run_end_to_end.sh decimated
bash scripts/run_end_to_end.sh precision
bash scripts/run_end_to_end.sh precision_offset

python3 model/reference.py report
python3 model/extended.py report
bash scripts/run_implementation.sh
bash scripts/run_postroute.sh
bash scripts/capture_wave.sh
python3 scripts/summarize.py
```

长窗口完整时钟仿真包含约 6 亿个时钟周期，会比加速回放慢。建议串行执行重型实现任务，避免内存与交换空间压力。没有需要付费许可的专有算法 IP。

所有回归以 `PASS ...` 标记和错误扫描判定，不能只依赖 XSim 的进程返回码：某些 `$fatal` 在 XSim 外层仍可能返回 0。若脚本未成功，不应继续宣称报告为新的通过结果。

## Vivado GUI 中检查

打开 `build/vivado/sfls_kr260.xpr`，默认仿真顶层 `tb_end_to_end`。运行 Behavioral Simulation 即可执行真实时钟端到端用例。该用例默认 350 kHz、32768 点常规窗口，含输入丢失和复位检查。

工程另含 `sim_precision`（32768 点、D=64、349999.999 Hz、6.02 秒真实时钟输入）和 `sim_arithmetic`（小窗口算术回放）两个 Simulation Set，可在 Sources 中选择设为活动集合后运行。

已有波形可直接通过 Open Static Simulation 打开 `reports/end_to_end.wdb`，不用重复长时间仿真。关注 `frequency_uHz`、`result_valid`、`window_full`、`fault`、`debug_event_valid`、`debug_sample_req`、`result_last_tick`。

也可直接执行以下命令打开 GUI、工程与已保存波形：

```bash
"${VIVADO_ROOT:-/home/aup/Xilinx/2025.2/Vivado}/bin/vivado" -mode gui -source scripts/open_results.tcl
```

微赫兹数值除以 1,000,000 得 Hz，例如 350000000000 表示 350000 Hz。`result_valid` 才是结果更新脉冲；窗口填满前结果无效。

## 范围与后续工作

本项目交付可综合计算核和仿真工程，不包含实板接线、比较器、电压调理、ADC、参考时钟校准、传感器磁场标定或完整板级 bitstream。上板前必须补全这些工作。短窗口下的失败频点、动态平滑、1 ppm 时钟失配用例均保留在报告中。
