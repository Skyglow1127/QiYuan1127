#!/usr/bin/env python3
"""Build a self-contained, evidence-linked report. Fail on missing regressions."""
from pathlib import Path
import base64,csv,hashlib,html,json,re,datetime
ROOT=Path(__file__).resolve().parents[1]
REP=ROOT/'reports'
tests=['divider','faults','small','stride1','stride64','extended1','extended64',
       'e2e_small','e2e_full','e2e_decimated','e2e_precision','e2e_precision_offset','postroute','project_arithmetic']
out=[]
for test in tests:
    p=ROOT/'build'/test/'simulate.log';s=p.read_text()
    passes=re.findall(r'^PASS .+$',s,re.M)
    assert passes and not re.search(r'Fatal:|FATAL|ERROR:|Timing Violation',s),test
    out.append(dict(test=test,status='PASS',evidence=str(p.relative_to(ROOT)),message=passes[-1]))
timings={}
for suffix in ['','_precision']:
    s=(REP/f'implementation_status{suffix}.txt').read_text()
    values=dict(line.split('=',1) for line in s.splitlines())
    assert float(values['WNS'])>=0 and float(values['WHS'])>=0
    timing=(REP/f'timing_routed{suffix}.rpt').read_text()
    assert 'All user specified timing constraints are met.' in timing
    util=(REP/f'utilization_routed{suffix}.rpt').read_text()
    for name,key in [('CLB LUTs','LUT'),('CLB Registers','FF'),('Block RAM Tile','BRAM36'),('DSPs','DSP')]:
        m=re.search(r'\|\s*'+re.escape(name)+r'\s*\|\s*(\d+)',util)
        values[key]=int(m.group(1))
    timings['stride64' if suffix else 'stride1']=values

accuracy=json.loads((REP/'accuracy_summary.json').read_text())
extended=json.loads((REP/'extended_summary.json').read_text())
pairs=json.loads((REP/'resolution_pairs.json').read_text())
noise=json.loads((REP/'noise_summary.json').read_text())
e2e={}
for profile in ['full','precision','precision_offset']:
    with (ROOT/f'build/e2e_{profile}/results.csv').open() as f:rows=list(csv.DictReader(f))
    times=[int(r['time_ns']) for r in rows]
    assert all(b-a==1_000_000 for a,b in zip(times,times[1:]))
    assert all(r['frequency_uHz']==r['reference_uHz'] for r in rows)
    e2e[profile]=dict(outputs=len(rows),first_output_s=times[0]/1e9,
        latency_cycles=sorted(set(int(r['latency_cycles']) for r in rows)),
        observed_output_period_ns=1_000_000,
        frequency_min_Hz=min(int(r['frequency_uHz']) for r in rows)/1e6,
        frequency_max_Hz=max(int(r['frequency_uHz']) for r in rows)/1e6)
    target=349999999000 if profile=='precision_offset' else 350000000000
    e2e[profile]['max_abs_error_Hz']=max(abs(int(r['frequency_uHz'])-target) for r in rows)/1e6
    assert e2e[profile]['max_abs_error_Hz']<=.001,profile

manifest={}
for folder in ['rtl','sim','scripts','model','docs']:
    for p in sorted((ROOT/folder).rglob('*')):
        if p.is_file() and p.suffix in ['.sv','.py','.sh','.tcl','.xdc','.md'] and '__pycache__' not in str(p):
            manifest[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
for p in [ROOT/'README.md',ROOT/'requirements.txt']:
    manifest[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
bundle=dict(date='2026-10-20',tool='Vivado/XSim 2025.2',part='xck26-sfvc784-2LV-c',
    tests=out,timing=timings,end_to_end=e2e,accuracy=accuracy,pairs=pairs,noise=noise,
    extended=extended,source_sha256=manifest,
    limitation='OOC timing with explicit interface budgets; not board signoff. Long windows do not establish kHz bandwidth.')
(REP/'verification.json').write_text(json.dumps(bundle,indent=2,ensure_ascii=False))

def image(name):
    b64=base64.b64encode((REP/name).read_bytes()).decode()
    return f'<img alt="{name}" src="data:image/png;base64,{b64}">'
def table(headers,rows):
    return '<div class="table"><table><thead><tr>'+''.join(f'<th>{html.escape(str(x))}</th>' for x in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join(f'<td>{html.escape(str(x))}</td>' for x in row)+'</tr>' for row in rows)+'</tbody></table></div>'

peak=max(x['max_abs_error_Hz'] for x in accuracy if x['profile']=='stride64')
fast=max(x['max_abs_error_Hz'] for x in accuracy if x['profile']=='stride1')
body=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>KR260 拉莫尔 SFLS 测频验证报告</title><style>
body{{margin:0;background:#f4f6f7;color:#172c3a;font:16px/1.75 system-ui,"Noto Sans CJK SC",sans-serif}}
main{{max-width:1140px;margin:auto;padding:42px 28px 80px}}h1{{font-size:34px;line-height:1.3}}h2{{margin-top:40px;font-size:23px}}
.sub{{color:#526773}}.card{{background:white;border:1px solid #dce3e8;border-radius:12px;padding:22px 26px;margin:22px 0}}
.notice{{border-left:5px solid #d68430}}.good{{border-left:5px solid #238573}}.table{{overflow:auto}}table{{border-collapse:collapse;width:100%;font-size:14px}}
th,td{{padding:10px 12px;text-align:left;border-bottom:1px solid #dce3e8;white-space:nowrap}}th{{background:#e8f0f3}}
img{{width:100%;height:auto;margin:16px 0}}code{{background:#e9eef1;padding:2px 5px}}li{{margin:8px 0}}a{{color:#176988}}
</style><main><p class="sub">2026-10-20 · Vivado / XSim 2025.2 · KR260 / K26</p>
<h1>拉莫尔信号 SFLS 测频<br>工程仿真与实现验证报告</h1>
<div class="card good"><b>工程已完成可综合 RTL、独立参考模型、XSim 回归和器件 OOC 实现。</b><br>
输入范围 70–350 kHz；输出更新率 1 kHz；求频运算全部在 FPGA 核心中完成；输出单位 1 µHz。</div>
<div class="card notice"><b>性能结论有明确条件。</b><br>
普通窗口静态测试最大误差 {fast:.6f} Hz，不能保证 0.001 Hz。长窗口静态测试最大误差 {peak:.6f} Hz，已测频点满足 0.001 Hz；其窗口为约 6–30 秒。不能据此声称具有 1 kHz 磁场测量带宽或真实传感器的绝对精度。</div>
<h2>1. 配置与延迟</h2>
{table(['项目','普通窗口','长窗口'],[['时钟','100 MHz','100 MHz'],['保留时间戳数','32768','32768'],['周期抽取 D','1','64'],['350 kHz 窗口','93.62 ms','5.99168 s'],['70 kHz 窗口','468.10 ms','29.9584 s'],['输出周期','1 ms','1 ms'],['快照到结果','98 clocks / 0.98 µs','98 clocks / 0.98 µs']])}
<p>填窗等待、窗口平滑延迟、快照计算延迟分别统计。固定事件数导致窗口跨度随输入频率变化。长窗口抽取是本项目对论文方法的工程扩展，最终频率公式补偿抽取倍数。</p>
<h2>2. 实际执行的检查</h2>
{table(['测试','结果','日志证据'],[[x['test'],x['message'],x['evidence']] for x in out])}
<p>时间戳回放加速了输入事件的仿真时间，但所有递推、缓存和除法均由 RTL 执行。两个长窗口端到端用例还真实运行了约 6 亿时钟周期，覆盖实际输入、同步、抽取和 1 kHz 调度。门级 SDF 用例只检查短时启动和边沿路径，完整精度扫描是 RTL 级验证。</p>
<h2>3. 静态频率与毫赫兹辨识</h2>
<p>主测试含 10 个频率、3 个初相位、每个组合 17 个滑动输出，另有噪声用例。重叠输出相关，不当作独立 Monte Carlo 样本。扩展测试另覆盖 17 个均匀频点和 14 个近整数分频频点。</p>
{image('accuracy.png')}
{table(['模式','输入 Hz','最大误差 Hz','RMSE Hz','≤0.001 Hz'],[[r['profile'],f"{r['frequency_Hz']:.3f}",f"{r['max_abs_error_Hz']:.6f}",f"{r['rmse_Hz']:.6f}",'通过' if r['pass_1mHz'] else '未通过'] for r in accuracy])}
<p>使用整数 µHz 判定边界，避免浮点表示把恰好 0.001 Hz 的误差误判。整频点的零误差并非全部频点都准确，近整数分频输入与不同初相位是必要测试。</p>
{table(['模式','低频 Hz','高频 Hz','最小输出差 µHz','最大输出差 µHz'],[[r['profile'],f"{r['lower_Hz']:.3f}",f"{r['upper_Hz']:.3f}",r['minimum_measured_separation_uHz'],r['maximum_measured_separation_uHz']] for r in pairs])}
<p>差频测试采用同一初相位和窗口索引配对；结果支持这些确定性用例的辨识，不能代替未知噪声下的统计置信保证。</p>
<h2>4. 动态响应与时基风险</h2>
{image('dynamic_response.png')}
{table(['模式','信号','相对平均事件时刻最大误差 Hz','相对最新事件时刻最大误差 Hz'],[[r['profile'],r['label'],f"{r['max_error_vs_mean_Hz']:.6f}",f"{r['max_error_vs_end_Hz']:.6f}"] for r in extended if r['label']!='dense_static'])}
<p>长窗口明显平滑磁场变化，1 kHz 输出不会消除这种延迟。+1 ppm 时基失配在 350 kHz 附近产生约 0.35 Hz 误差；增加帧长不能校准时基。1 ns/10 ns 独立高斯边沿抖动用例保存在 <code>noise_summary.json</code>，属于指定模型下的结果。</p>
<h2>5. 全链路真实时钟结果</h2>
{table(['用例','输出数','首个输出 s','测得输出周期 ns','计算延迟 clocks','频率最小/最大 Hz'],[[k,v['outputs'],v['first_output_s'],v['observed_output_period_ns'],str(v['latency_cycles']),f"{v['frequency_min_Hz']:.6f} / {v['frequency_max_Hz']:.6f}"] for k,v in e2e.items()])}
<p>计数包含输入停止后、10 ms 超时告警前的保持输出。接收端应检查最新事件时间戳；超时后 valid 被抑制，复位后重新建窗。</p>
<h2>6. KR260 器件资源与时序</h2>
{table(['模式','LUT','FF','BRAM36','DSP','WNS ns','WHS ns'],[[k,v['LUT'],v['FF'],v['BRAM36'],v['DSP'],v['WNS'],v['WHS']] for k,v in timings.items()])}
<p>器件为 <code>xck26-sfvc784-2LV-c</code>，100 MHz，时钟不确定度 0.1 ns。上表是 OOC 核级实现，不是板级签核：同步 rst 输入合同 0.5–2.0 ns，外部输出数据通路预算 0.2–2.0 ns，需在实际集成时验证。</p>
<p>DRC 中的 DSP 流水线建议和 OOC 无物理接收端警告保留在报告中；异步输入端口的 CDC 分析存在工具覆盖边界，不能用“零跨时钟条目”证明模拟亚稳态安全。未生成板级 bitstream。</p>
<h2>7. 交付与复现</h2>
<ul><li>Vivado 工程：<code>build/vivado/sfls_kr260.xpr</code></li>
<li>波形数据库：<code>reports/end_to_end.wdb</code></li>
<li>完整命令：<code>README.md</code>；接口与误差推导：<code>docs/DESIGN.md</code></li>
<li>机器可读结果、日志列表和源码 SHA-256：<code>reports/verification.json</code></li>
<li>原论文在项目根目录；本项目复现 SFLS 算法并扩展为 FPGA 实现，没有原作者源码或原始实验数据。</li></ul>
<div class="card notice">若最终需求是“对快速变化的拉莫尔信号，在约 1 ms 响应内仍有 0.001 Hz 精度”，本工程没有证明该目标成立。下一步应依据传感器噪声、动态带宽和时基规格，评估更高分辨时间戳、ADC 相位估计或其他结构；不能把当前长窗口结果当作该目标已经完成。</div>
</main></html>'''
(REP/'verification.html').write_text(body)
print('Verified',len(out),'regressions; wrote reports/verification.html and verification.json')
