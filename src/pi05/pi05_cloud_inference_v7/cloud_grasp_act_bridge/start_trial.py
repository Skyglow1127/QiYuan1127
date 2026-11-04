#!/usr/bin/env python3
"""Display the concrete PI05 plan, or approve/run it via three explicit flags.

Default display reads local files only and never changes approval. The operator
may explicitly approve the displayed plan's exact SHA256 and onsite conditions
using --execute --approve-plan SHA256 --onsite-conditions-confirmed. Running that
command starts a physical trial; this script must not be invoked automatically.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from plan_workflow import verify_latest_preview, verify_review_content, validate_duration, duration_description
from nativequeuepolicy import QUEUE_CONTRACT, RUNTIME_ACTION_STEPS
from newconditioning import CONDITIONING_ID, profile_sha256

HERE = Path(__file__).resolve().parent
PLAN = HERE / 'grasp_plan.json'
PYTHON = '/home/aup/Lerobot/envs/piper-cpu/bin/python'


def read(path):
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'Expected a JSON object: {path}')
    return value


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def verify_preview(plan):
    return verify_latest_preview(plan, HERE)


def display(plan, plan_hash):
    evidence = plan['preview_evidence']
    duration = validate_duration(plan.get('duration_s'))
    duration_text = duration_description(duration)
    if (plan.get('schema_version') != 'pi05-expert-newcloud-act-bridge-plan/v1'
            or plan.get('trial_kind') != 'lerobot_native_queue_act_bridge_trial'
            or plan.get('n_action_steps') != RUNTIME_ACTION_STEPS
            or plan.get('action_indices') != list(range(RUNTIME_ACTION_STEPS))
            or plan.get('queue_contract') != QUEUE_CONTRACT
            or plan.get('command_conditioning_id') != CONDITIONING_ID
            or plan.get('command_conditioning_sha256') != profile_sha256(read(HERE/'pi05_execution_defaults.json')['command_conditioning'])
            or plan.get('startup', {}).get('joint_mode_speed_percent') != 5
            or plan.get('startup', {}).get('gripper_force_n') != 1.
            or plan.get('startup', {}).get('enable_reset_home_calibrate') is not False):
        raise ValueError('该入口仅用于已展示的0无限/1..300秒、J5%、1N、用户指定runtime20的官方队列且不复位/使能的具体计划。')
    scene = Path(evidence['last_scene_image'])
    if not scene.is_absolute():
        scene = Path(evidence['report']).parent / scene
    approved = bool(plan['readiness'].get('approved_goal_path_parameters', '').strip())
    print('π0.5 动作专家 B 官方LeRobot队列试跑计划（默认只展示，无网络/硬件操作）', flush=True)
    print('目标：' + plan['goal'], flush=True)
    print('路径意图：' + ' → '.join(plan['path_intent']), flush=True)
    print(f'参数：J模式5%；夹爪1N；{duration_text}；每次预测50步，按用户允许缩减为20步由官方队列依序执行前20步（checkpoint保存值10不变），再重新预测。执行节拍20Hz来自采集数据；官方通用rollout默认是30Hz。', flush=True)
    print('目标：保留官方原始FIFO，再复用ACT硬件适配：关节2°/夹爪1mm内边距；相对本周期输入反馈2°/3mm限幅；目标速率0.9；不加alpha平滑、抓放续步或原始预测差值门。实际目标双端保护不变。', flush=True)
    print('本次新增VLA闭爪否决：张开意图后，连续2个不同腕部图像需有红块中心x40%～60%、中心y≥70%、下沿≥95%、面积3%～20%且无大红色歧义；实际六轴须在示范抓取关节窗内，原始六轴目标距反馈≤3°。否则仅暂缓收爪，六轴照常经过原适配器。首次暂缓后8秒仍不能开始闭合即停止。单目条件不证明三维对齐/接触/抓住。', flush=True)
    print('本次新增闭合协调：允许闭合后保持实测六轴，锁存模型最小闭合意图（单独记录来源）；实际42.5～45.5mm稳定0.30秒/极差≤0.5mm且上次ACK向内差≥2mm才判为夹持候选。4秒未满足、空闭<35mm或实际锚点漂移>3°等即停。ACK后丢弃旧FIFO，用之后新观测重新预测。夹持后以确认时ACK固定上限保留≥2mm内压差，携带中放置窗外超过此上限即停；力只记录，不声称接触或成功。每次补读新反馈但不刷新原模型/相机时间。', flush=True)
    print('反馈语义：2°/3mm是相对本周期输入反馈的目标限制，不是整个运行时长内的累计位移限制；推理期间硬件反馈可能变化，接收端仍按真正当前反馈实施原8°/15mm跟踪保护。', flush=True)
    print('时序契约变化：队列第i步槽位=该块原始采集时间+i/20，允许槽位迟到≤400ms；原始块年龄另行记录，可能超过400ms。当前反馈仍须新鲜，不能以新观测或接收时间刷新旧块来源。', flush=True)
    print('物理保护：双端实际目标边界、6°/8mm单步、45°/s及50mm/s速率、8°/15mm实际跟踪误差、故障/租约/期限检查；超时、越界或故障立即结束。', flush=True)
    print(f"现场预览：{evidence['request_count']}个独立新动作块中{evidence['numeric_source_checks_pass_count']}次通过适配后首动作数值可行性/时效检查，{evidence['rejected_count']}次拒绝。后19步未来反馈、连续提交和碰撞路径未验证；原始全部50步边界另作诊断。", flush=True)
    print('参考场景：' + str(scene.resolve()), flush=True)
    print('现场条件：' + plan['onsite_conditions'], flush=True)
    print('若目标、路径、相机或姿态等变化，必须重新预览。', flush=True)
    print('当前已有明确批准：' + ('是；重用现有证据，不重写批准' if approved else '否；本次展示不写批准'), flush=True)
    print('计划：' + str(PLAN), flush=True)
    print('审阅内容 ID（SHA256；记录批准后保持不变）：' + plan_hash, flush=True)
    if plan.get('execution_ready') is not True:
        print('当前不可执行：最新预览未通过。', flush=True)
        for reason in plan.get('blocking_reasons', []):
            print('拦截原因：' + reason, flush=True)
        print('先排查预测与当前姿态；不要直接替换SHA、回选旧预览或改变保护门限。', flush=True)
    else:
        print('下面的显式命令会批准该具体计划并使机械臂运动；只有现场操作者确认后才亲自运行：', flush=True)
        print(str(HERE / 'start_trial.sh') + ' --execute --approve-plan ' + plan_hash + ' --onsite-conditions-confirmed', flush=True)


def check_approval_transition(before, after):
    """Approval must preserve every non-approval field and file fingerprint."""
    if before['review_content_sha256'] != verify_review_content(after):
        raise RuntimeError('批准准备期间计划内容、预览、代码或参数发生变化；未启动试跑，请重新审阅。')
    if after.get('physical_approval_pending') is not False:
        raise RuntimeError('批准记录没有完成；未启动。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='明确批准并开始该具体计划；机械臂将运动')
    parser.add_argument('--approve-plan', help='亲自审阅的当前审阅内容ID（SHA256）')
    parser.add_argument('--onsite-conditions-confirmed', action='store_true',
        help='确认保持现有供电、执行前无其他CAN控制流量且运行中监督、空载稳固承托、手退出、可立即断电，以及场景/目标/路径未变化')
    args = parser.parse_args()
    plan = read(PLAN)
    verify_preview(plan)
    if plan.get('schema_version') == 'pi05-expert-newcloud-act-bridge-diagnostic/v1':
        print('π0.5 最新预览诊断（不会启动机械臂）：', flush=True)
        for reason in plan.get('blocking_reasons', []):
            print(reason, flush=True)
        if args.execute or args.approve_plan or args.onsite_conditions_confirmed:
            raise ValueError('当前仅有诊断方案，禁止批准或执行。')
        return 0
    plan_hash = verify_review_content(plan)
    display(plan, plan_hash)
    if not args.execute and args.approve_plan is None and not args.onsite_conditions_confirmed:
        print('仅展示完成：未写批准、未连接云端/相机/机器人、未发动作。', flush=True)
        return 0
    missing = []
    if not args.execute:
        missing.append('--execute')
    if not args.approve_plan:
        missing.append('--approve-plan 当前SHA256')
    if not args.onsite_conditions_confirmed:
        missing.append('--onsite-conditions-confirmed')
    if missing:
        raise ValueError('未批准或启动：必须同时显式提供 ' + '、'.join(missing))
    if plan.get('execution_ready') is not True:
        raise ValueError('最新预览未通过；当前方案不可执行、不能记录执行批准。请先排查预测。')
    if not re.fullmatch(r'[0-9a-f]{64}', args.approve_plan) or args.approve_plan != plan_hash:
        raise ValueError('提供的SHA256与当前已展示计划不一致；未批准或启动。')
    # No approval/network writes occur above this point. Three explicit flags
    # constitute the terminal operator's approval of this exact displayed plan.
    duration = validate_duration(plan.get('duration_s'))
    duration_text = duration_description(duration)
    already_approved = bool(plan['readiness'].get('approved_goal_path_parameters', '').strip())
    if not already_approved:
        approval = (
            '现场操作者已明确以 --execute --approve-plan ' + plan_hash +
            ' --onsite-conditions-confirmed 批准刚展示的π0.5动作专家B新云端推理实物试跑计划：' + plan['goal'] +
            ' 路径意图：' + '→'.join(plan['path_intent']) +
            f'；J模式速度5%，夹爪1N，{duration_text}；官方PI05队列每块预测50步、按用户允许缩减为20步依序执行前20步（checkpoint保存值10不变），20Hz，原始绝对队列目标复用ACT硬件适配：2°关节/1mm夹爪内边距，本周期输入反馈相对2°/3mm限幅，目标速率0.9；不加alpha平滑/续步/原始8°预测门。'
            '已审阅时间契约变化：第i步槽位为原块采集时间+i/20，400ms限制槽位迟到，原块年龄单独记录并可超过400ms；新反馈不刷新旧块来源。'
            '已审阅本次新增VLA经验视觉闭爪否决的完整参数及8秒暂缓上限：只否决明显未接近的闭爪，不证明三维对齐/接触/成功抓住；闭合阶段允许本地保持六轴并锁存模型闭合意图，候选反馈条件42.5～45.5mm稳定0.30秒/极差≤0.5mm/ACK向内差≥2mm，4秒上限；候选ACK后重推，放置窗外再开即停；原FIFO、视觉否决和本地阶段目标分别留痕、来源不刷新。'
            '实际目标仍须通过双端全部边界、6°/8mm单步、速率、8°/15mm跟踪、反馈新鲜度、租约和停止保护；后续未来反馈与完整抓放尚未验证；'
            '按用户要求保持现有供电；执行前须无其他CAN控制流量，运行中监督发现其他控制源立即结束；控制静默不证明物理隔离或主臂断电。'
            '已确认从臂空载且稳固承托、手退出、现场可立即断电，且目标、黑框、相机、机械臂姿态和全臂路径与审阅场景一致。'
        )
        preview = verify_preview(plan)
        # Recheck the actual bytes immediately before recording approval.
        if verify_review_content(read(PLAN)) != plan_hash:
            raise RuntimeError('计划在批准前已变化；未写批准。')
        subprocess.run([PYTHON, '-B', str(HERE / 'prepare_trial.py'), '--preview', str(preview),
            '--duration', str(duration),
            '--approve-review-id', plan_hash, '--approval-note', approval,
            '--onsite-conditions-confirmed'], check=True)
        approved_plan = read(PLAN)
        check_approval_transition(plan, approved_plan)
        verify_preview(approved_plan)
        print('已记录操作者明确批准；审阅内容 ID 保持不变：' + plan_hash, flush=True)
    else:
        print('重用该SHA对应的现有批准与预览证据，未改写批准。', flush=True)
    # Lower layers still verify the exact current plan file bytes. The stable
    # user-facing identity is never substituted for their byte-integrity pin.
    return subprocess.run([str(HERE / 'trial.sh'), '--execute', '--approve-plan', digest(PLAN),
        '--onsite-conditions-confirmed'], check=False).returncode


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f'π0.5试跑未启动或已中止：{type(error).__name__}: {error}', file=sys.stderr)
        raise SystemExit(1)
