#!/usr/bin/env python3
"""Offline SDK response/accuracy analysis. Does not calibrate physical URDF axes."""
import argparse
import json
import statistics
from pathlib import Path


def analyze(folder, supplements=()):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    sources = [folder, *supplements]
    summaries = [json.loads((source/'summary.json').read_text()) for source in sources]
    items = [(source,item) for source,summary in zip(sources,summaries) for item in summary['results']]
    ids = [item['joint'] for _,item in items]
    if len(ids) != len(set(ids)):
        raise ValueError('Duplicate joint across runs; do not silently replace a failed measurement')
    result = {'sequence_completed': all(summary['completed'] for summary in summaries),
              'source_directories': [str(p.resolve()) for p in sources], 'joints': [],
              'physical_joint_mapping_validated': False, 'physical_axis_sign_validated': False,
              'mechanical_zero_offsets_calibrated': False,
              'note': 'All measurements use onboard SDK feedback. Two return-to-zero observations per joint are not a physical zero calibration.'}
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharey=True)
    for ax in axes.flat:
        ax.set_visible(False)
    for source, item in items:
        joint = item['joint']
        ax = axes.flat[joint]
        ax.set_visible(True)
        endpoint = {e['phase']: e for e in item['endpoints'] if e.get('usable')}
        positive, negative = endpoint.get('positive_settle'), endpoint.get('negative_settle')
        zero = [e['feedback_median_deg'] for key, e in endpoint.items() if key.startswith('zero_from_')]
        row = {'joint': joint, 'source_directory': str(source.resolve()),
               'sequence_completed': item['completed'], 'endpoints': item['endpoints'],
               'max_non_target_drift': item['non_target_max_drift'],
               'zero_return_readings_deg': zero,
               'zero_return_median_deg': statistics.median(zero) if len(zero)==2 else None,
               'zero_direction_difference_deg': zero[0]-zero[1] if len(zero)==2 else None}
        if item['completed'] and positive and negative:
            vector = [(a-b)/4 for a,b in zip(positive['all_joints_median'], negative['all_joints_median'])]
            response = positive['feedback_median_deg']-negative['feedback_median_deg']
            row.update(sdk_response_per_command_degree=vector, sdk_selected_channel_gain=vector[joint],
                       positive_command_increases_sdk_feedback=response>.6,
                       physical_rotation_direction='not independently observed')
        row['accuracy_within_0_3_count'] = sum(e.get('within_0_3_deg',False) for e in item['endpoints'])
        row['accuracy_evaluated_count'] = sum(e.get('usable',False) for e in item['endpoints'])
        result['joints'].append(row)
        command_t, command_q, feedback_t, feedback_q = [], [], [], []
        wire_count = 0
        for raw in (source/f'j{joint}'/'events.jsonl').read_text().splitlines():
            event = json.loads(raw)
            if event['kind'] == 'command':
                command_t.append(event['t']); command_q.append(event['target_deg'])
            if event['kind'] == 'output' and event['line'].startswith('[tx] '):
                wire = json.loads(event['line'][5:])
                if (wire.get('address') != 1 or wire.get('funcode') != 1
                        or wire.get('data',{}).get('id') != joint
                        or abs(wire['data']['angle']) > 2):
                    raise ValueError('Unexpected actual command in small-joint log')
                wire_count += 1
            if event['kind'] == 'output' and '[rx] ' in event['line']:
                msg=json.loads(event['line'].split('[rx] ',1)[1])
                if msg.get('address')==2 and msg.get('funcode')==1:
                    feedback_t.append(event['received_t']);feedback_q.append(msg['data'][f'angle{joint}'])
        ax.plot(command_t, command_q, label='Command', linewidth=1.2)
        ax.plot(feedback_t, feedback_q, label='SDK feedback', linewidth=1)
        ax.set(title=f'J{joint}: '+('complete' if item['completed'] else 'STOPPED'), xlabel='Host time (s)', ylabel='Angle (deg)')
        ax.grid(alpha=.25);ax.legend(fontsize=8)
        row['actual_wire_commands'] = wire_count
        row['wire_route_and_range_verified'] = wire_count == item['commands']
    fig.suptitle('Small joint response test — SDK feedback only; physical axis mapping NOT calibrated')
    fig.tight_layout()
    fig.savefig(folder/'motion_plot.png', dpi=150)
    plt.close(fig)
    (folder/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    parser.add_argument('--supplement',type=Path,action='append',default=[])
    args=parser.parse_args()
    analyze(args.folder,args.supplement)
