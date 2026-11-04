#!/usr/bin/env python3
"""Offline synchronous vs one-chunk async prefetch benchmark. No robot IO or RTC guidance."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time

EXISTING = Path(__file__).resolve().parents[2] / '01_同步推理' / '程序'


def save(path, obj):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(path)


def summary(values):
    ordered = sorted(values)
    p = .95 * (len(ordered) - 1)
    lo = int(p)
    hi = min(lo + 1, len(ordered) - 1)
    return dict(median=statistics.median(values), mean=statistics.mean(values),
                p95=ordered[lo] + (ordered[hi] - ordered[lo]) * (p - lo),
                maximum=max(values), samples=len(values))


def experiment(predict, pool, mode, consume, fps, chunks, seed):
    """Same worker and model for both modes; at most one pending future.

    A dummy consumer reads actions at a nominal fixed rate. It sends nothing.
    A chunk's first consumption begins after its inference completes AND the
    previous chunk's consumption window ends. No stale-prefix skipping is done.
    """
    began = time.perf_counter()
    pending = pool.submit(predict, seed)
    records, boundaries, ages = [], [], []
    late_ticks = 0
    for index in range(chunks):
        waited_at = time.perf_counter()
        result = pending.result()
        wait = time.perf_counter() - waited_at
        boundary = time.perf_counter()
        if index == 0:
            startup = boundary - began
        else:
            boundaries.append(wait)
        if mode == 'async' and index + 1 < chunks:
            pending = pool.submit(predict, seed + index + 1)
        for action_index in range(consume):
            deadline = boundary + action_index / fps
            time.sleep(max(0., deadline - time.perf_counter()))
            tick = time.perf_counter()
            late_ticks += int(tick - deadline > .005)
            # Materialize a scalar from the actual predicted action (no IO).
            float(result['actions'][action_index][0])
            ages.append(tick - result['started'])
        time.sleep(max(0., boundary + consume / fps - time.perf_counter()))
        records.append({k: v for k, v in result.items() if k != 'actions'})
        if mode == 'sync' and index + 1 < chunks:
            pending = pool.submit(predict, seed + index + 1)
    elapsed = time.perf_counter() - began
    return dict(mode=mode, consume_steps=consume, fps=fps, chunks=chunks,
                simulated_actions=consume * chunks, total_seconds=elapsed,
                startup_seconds=startup, post_startup_seconds=elapsed - startup,
                post_startup_effective_action_hz=consume * chunks / (elapsed - startup),
                prediction_seconds=summary([r['elapsed'] for r in records]),
                boundary_wait_seconds=boundaries, boundary_wait_total_seconds=sum(boundaries),
                boundary_wait_p95_seconds=summary(boundaries)['p95'],
                inference_start_to_consumption_age_seconds=summary(ages),
                ticks_over_5ms_late=late_ticks, predictions=records)


def load_predictor():
    # Reuse known model paths and the existing streaming loader; no framework edits.
    sys.path.insert(0, str(EXISTING))
    import benchmark as old
    torch, np = old.torch, old.np
    torch.set_num_threads(8)
    torch.cuda.set_per_process_memory_fraction(.85)
    cfg = old.PreTrainedConfig.from_pretrained(str(old.ADAPTER), local_files_only=True,
        cli_overrides=['--device=cuda', '--dtype=bfloat16', '--gradient_checkpointing=false',
                       '--compile_model=false', '--push_to_hub=false'])
    assert cfg.chunk_size == 50 and cfg.num_inference_steps == 10
    assert getattr(cfg, 'rtc_config', None) is None, 'This test measures plain async prefetch, not RTC guidance.'
    cfg.pretrained_path = str(old.BASE)
    pre, post = old.make_pre_post_processors(policy_cfg=cfg, pretrained_path=str(old.ADAPTER),
        preprocessor_overrides={'tokenizer_processor': {'tokenizer_name': str(old.TOKENIZER)},
                                'device_processor': {'device': 'cuda'}},
        postprocessor_overrides={'device_processor': {'device': 'cpu'}})
    with np.load(old.OBSERVATION, allow_pickle=False) as data:
        obs = {key: torch.from_numpy(data[key].copy()) for key in cfg.input_features}
        obs['task'] = str(data['task'].item())
    batch = pre(obs)
    print('加载 π0.5 + LoRA 020000；本机 GPU、BF16、10 步流匹配……', flush=True)
    base, loading = old.load_base_streaming(old.BASE, cfg)
    assert not loading['missing_keys'] and not loading['unexpected_keys'] and not loading['shape_mismatches']
    pcfg = old.PeftConfig.from_pretrained(str(old.ADAPTER), local_files_only=True)
    pcfg.base_model_name_or_path = str(old.BASE)
    policy = old.PeftModel.from_pretrained(base, str(old.ADAPTER), config=pcfg,
                                          local_files_only=True, is_trainable=False)
    policy.eval().requires_grad_(False)
    saved = old.load_file(str(old.ADAPTER / 'adapter_model.safetensors'), device='cpu')
    loaded = old.get_peft_model_state_dict(policy, adapter_name='default')
    assert set(saved) == set(loaded)
    for name, tensor in saved.items():
        actual = loaded[name].detach().cpu()
        assert tensor.shape == actual.shape and torch.equal(tensor.to(actual.dtype), actual), name
    verified = len(saved)
    del saved, loaded
    torch.cuda.synchronize()

    def predict(seed):
        policy.reset()
        torch.manual_seed(seed)
        torch.cuda.synchronize()
        started = time.perf_counter()
        with torch.inference_mode():
            actions = post(policy.predict_action_chunk(batch, num_steps=10)).detach().cpu()
        torch.cuda.synchronize()
        finished = time.perf_counter()
        assert tuple(actions.shape) == (1, 50, 7) and torch.isfinite(actions).all().item()
        arr = actions[0].numpy().copy()
        return dict(actions=arr, started=started, finished=finished, elapsed=finished-started,
                    seed=seed, action_sha256=hashlib.sha256(arr.tobytes()).hexdigest())

    metadata = dict(device=torch.cuda.get_device_name(0), torch=torch.__version__, hip=torch.version.hip,
        base=str(old.BASE), adapter=str(old.ADAPTER), tokenizer=str(old.TOKENIZER),
        adapter_sha256=old.sha256_file(old.ADAPTER / 'adapter_model.safetensors'),
        observation=str(old.OBSERVATION), observation_sha256=old.sha256_file(old.OBSERVATION),
        verified_adapter_tensors=verified, dtype='bfloat16', flow_steps=10, batch_size=1,
        predicted_chunk_steps=50, saved_n_action_steps=cfg.n_action_steps,
        compile_model=False, allocator_fraction=.85, cpu_threads=8)
    return predict, metadata


def self_test():
    active, max_active = 0, 0

    def fake(seed):
        nonlocal active, max_active
        active += 1
        max_active = max(active, max_active)
        started = time.perf_counter()
        time.sleep(.04)
        active -= 1
        return dict(actions=[[float(seed)] * 7 for _ in range(50)], seed=seed,
                    started=started, finished=time.perf_counter(),
                    elapsed=time.perf_counter()-started)

    with ThreadPoolExecutor(max_workers=1) as pool:
        a = experiment(fake, pool, 'sync', 20, 200, 4, 42)
        b = experiment(fake, pool, 'async', 20, 200, 4, 42)
    assert a['simulated_actions'] == b['simulated_actions'] == 80
    assert [p['seed'] for p in a['predictions']] == [p['seed'] for p in b['predictions']]
    assert max_active == 1
    assert a['boundary_wait_total_seconds'] > .09
    assert b['boundary_wait_total_seconds'] < a['boundary_wait_total_seconds'] * .5
    assert b['total_seconds'] < a['total_seconds']
    with ThreadPoolExecutor(max_workers=1) as pool:
        short = experiment(fake, pool, 'async', 2, 200, 4, 42)
    assert short['boundary_wait_total_seconds'] > .05, 'Short windows must still show starvation.'
    print('PASS：单推理线程、相同预测序列、异步重叠、短窗口仍缺动作。模拟延迟非模型实测。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--chunks', type=int, default=6)
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--warmup', type=int, default=3)
    parser.add_argument('--consume', type=int, nargs='+', default=[10, 20, 50])
    parser.add_argument('--fps', type=float, default=20.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.chunks < 2 or args.rounds < 1 or args.warmup < 1 or not (0 < args.fps <= 1000) or any(not 1 <= n <= 50 for n in args.consume):
        parser.error('chunks>=2, rounds>=1, warmup>=1, 0<fps<=1000, consume in 1..50')
    output = args.output or Path(__file__).resolve().parent.parent / '结果' / ('run_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir(parents=True, exist_ok=False)
    report = dict(success=False, robot_io=False, rtc_guidance=False,
        scope='Offline repeated recorded observation, one GPU inference worker, CPU dummy action consumer; no temporal realignment or robot execution.',
        timing_scope='Preprocessed input through full 50x7 postprocessed CPU output, GPU synchronized. Excludes input acquisition/preprocessing and model loading.',
        freshness_scope='Action age is measured from inference start on a reused historical observation, NOT real observation freshness.',
        warmup=args.warmup, rounds=args.rounds, chunks_each=args.chunks, fps=args.fps, runs=[])
    save(output / 'report.json', report)
    print('结果目录：', output, flush=True)
    predict, metadata = load_predictor()
    report['model'] = metadata
    with ThreadPoolExecutor(max_workers=1, thread_name_prefix='OfflineInference') as pool:
        for i in range(args.warmup):
            item = pool.submit(predict, 19000+i).result()
            print(f'预热 {i+1}/{args.warmup}：{item["elapsed"]:.3f} 秒', flush=True)
        for r in range(args.rounds):
            for consume in (args.consume if r % 2 == 0 else list(reversed(args.consume))):
                for mode in (['sync', 'async'] if r % 2 == 0 else ['async', 'sync']):
                    print(f'第 {r+1} 轮，{mode}，每块取 {consume} 步，共 {args.chunks} 块……', flush=True)
                    result = experiment(predict, pool, mode, consume, args.fps, args.chunks, 20000+r*100)
                    result['round'] = r+1
                    report['runs'].append(result)
                    save(output / 'report.json', report)
                    print(f'  单次中位 {result["prediction_seconds"]["median"]:.3f}s；'
                          f'块间等待合计 {result["boundary_wait_total_seconds"]:.3f}s；'
                          f'模拟动作有效频率 {result["post_startup_effective_action_hz"]:.2f}Hz', flush=True)
    report['success'] = True
    save(output / 'report.json', report)
    lines = ['π0.5 + LoRA 020000 同步/异步预取离线对比',
        '仅测录制输入上的推理及模拟动作队列吞吐；没有机械臂通信，没有 RTC 前缀指导或过期动作裁剪。',
        '每次模型输出50×7；取10/20/50步是不同消费窗口，同步与异步只在同一窗口内比较。',
        '单次时间包含GPU同步和动作后处理，不含采图、输入预处理、模型加载。',
        '有效频率排除每组初次预测等待，计入后续块间等待及每个动作的1/fps消费窗口。',
        '50步只是离线实验参数；不代表实机已验证能执行完整50步。', '',
        '轮次  窗口  模式    预测中位(s)  块间等待合计(s)  等待P95(s)  模拟有效Hz']
    for r in report['runs']:
        lines.append(f'{r["round"]:2d}    {r["consume_steps"]:2d}   {r["mode"]:5s}       '
                     f'{r["prediction_seconds"]["median"]:.4f}          {r["boundary_wait_total_seconds"]:.4f}         '
                     f'{r["boundary_wait_p95_seconds"]:.4f}       {r["post_startup_effective_action_hz"]:.2f}')
    lines += ['', '技术文档填写模板：',
        '在 Radeon 890M 上，固定 π0.5+LoRA 020000、BF16、batch=1、10步流匹配，',
        '使用相同录制观察，对同步执行与单线程异步预取进行离线对照。',
        '当每块消费__步、模拟消费频率20Hz时，异步预测中位__秒，块间等待P95为__秒，',
        '模拟有效动作频率为__Hz，对应同步为__Hz。结果反映队列调度性能，不代表实机任务成功率。']
    (output / '测速结果.txt').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines), flush=True)
    print('完成：', output / '测速结果.txt', flush=True)


if __name__ == '__main__':
    main()
