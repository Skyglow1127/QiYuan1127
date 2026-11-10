"""Audit saved ACT proposals as PREACTION sequence inputs, without inference.

Teacher examples provide only current q, camera provenance, and identity. Their
future actions, padding, and future images are NOT labels for unexecuted ACT.
"""
import argparse
import json
from pathlib import Path

from .act_sequence import REQUEST_SCHEMA, build_request_packet, read_json, sha


def request_from_record(record, example, role='global'):
    if role not in ('global', 'wrist'):
        raise ValueError('unsupported_camera_role')
    if record['observation_id'] != example['id']:
        raise ValueError('observation_join_mismatch')
    if record.get('sent_sdk_targets') != [] or record.get('actual_policy_robot_feedback') is not None:
        raise ValueError('requires_explicit_unsent_proposal')
    observation = record['historical_observation']
    if observation['q'] != example['q']:
        raise ValueError('q_join_mismatch')
    timing = record['review']['timing']
    started = timing['historical_trigger_time']
    if started != example['command_host_monotonic']:
        raise ValueError('trigger_join_mismatch')
    image = {**example['cameras'][role], 'role': role}
    return dict(schema=REQUEST_SCHEMA, chunk=record['raw_policy_action'],
                q_current=observation['q'], observation_id=example['id'],
                q_observed_at=observation['time'], prediction_started_at=started,
                prediction_completed_at=timing['virtual_completion_time'],
                # ACT emits ten full-arm proposals regardless of teacher padding.
                valid_steps=[True] * 10, commanded_joint_mask=[[True] * 7 for _ in range(10)],
                current_image=image,
                provenance=dict(kind='saved_unsent_ACT_proposal',
                    clock='historical trigger plus separately measured replay inference latency; virtual completion',
                    masks='ten full-joint model outputs; not teacher padding or proof of transmission',
                    historical_review_status=record['status'],
                    historical_rejection_reasons=record.get('rejection_reasons', []),
                    teacher_future_used_as_input=False, future_image_ground_truth_available=False,
                    teacher_trajectory_not_ACT_outcome=True))


def audit(dataset, review, output):
    dataset, review, out = map(lambda p: Path(p).resolve(), (dataset, review, output))
    manifest = read_json(dataset / 'manifest.json')
    examples_path = Path(manifest['examples_file'])
    if sha(examples_path) != manifest['examples_sha256']:
        raise ValueError('dataset_examples_hash_mismatch')
    protocol = read_json(review / 'protocol.json')
    result = read_json(review / 'summary.json')
    predictions_path = review / 'predictions.jsonl'
    if sha(predictions_path) != result['predictions_sha256']:
        raise ValueError('saved_predictions_hash_mismatch')
    if sha(dataset / 'manifest.json') != protocol['dataset_manifest_sha256']:
        raise ValueError('review_dataset_mismatch')
    if sha(review / 'protocol.json') != result['protocol_sha256']:
        raise ValueError('review_protocol_mismatch')
    rows = [json.loads(line) for line in examples_path.read_text().splitlines() if line.strip()]
    examples = {row['id']: row for row in rows}
    if len(examples) != len(rows):
        raise ValueError('duplicate_example_ids')
    out.mkdir(parents=True, exist_ok=False)
    accepted = rejected = expired = teacher_padded = source_count = 0
    split_counts = {}
    seen = set()
    with predictions_path.open() as source, (out / 'packets.jsonl').open('w') as packets, (out / 'rejections.jsonl').open('w') as errors:
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                continue
            source_count += 1
            record = json.loads(line)
            identity = record.get('observation_id')
            try:
                if identity in seen:
                    raise ValueError('duplicate_prediction_id')
                seen.add(identity)
                example = examples[identity]
                if record['raw_policy_action']['model_sha256'] != protocol['checkpoint_sha256']:
                    raise ValueError('model_provenance_mismatch')
                request = request_from_record(record, example)
                packet = build_request_packet(request)
                if packet['chunk']['commands'] != record['raw_policy_action']['commands']:
                    raise ValueError('raw_commands_not_preserved')
                if packet['chunk']['target_times'] != record['raw_policy_action']['target_times']:
                    raise ValueError('raw_times_not_preserved')
                packets.write(json.dumps(dict(sequence=packet, provenance=request['provenance']), allow_nan=False) + '\n')
                if accepted == 0:
                    (out / 'example_request.json').write_text(json.dumps(request, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
                accepted += 1
                expired += sum(not future for future in packet['future_at_completion_mask'])
                # Diagnostic count only; never used to build an ACT validity mask.
                teacher_padded += any(example['is_pad'])
                split_counts[example['split']] = split_counts.get(example['split'], 0) + 1
            except (ValueError, TypeError, KeyError) as exc:
                rejected += 1
                errors.write(json.dumps(dict(line=line_number, observation_id=identity, error=str(exc))) + '\n')
    summary = dict(schema='d1-preaction-act-sequence-audit-v1',
                   status='INTERFACE_AUDIT_COMPLETE' if rejected == 0 else 'INTERFACE_AUDIT_WITH_REJECTIONS',
                   audit_scope='action_sequence_contract_only', images_revalidated=False,
                   accepted=accepted, rejected=rejected, split_counts=split_counts,
                   source_prediction_count=source_count, expired_target_count=expired,
                   teacher_padded_rows_not_used_as_ACT_masks=teacher_padded,
                   raw_commands_and_times_preserved=True, time_shift_applied_s=0,
                   hypothetical_actions_have_physical_labels=False,
                   neural_forecast_tested=False, training_executed=False,
                   robot_commands=0, hardware_accessed=False, live_eligible=False,
                   source_sha256={str(p): sha(p) for p in (dataset / 'manifest.json', examples_path,
                       review / 'protocol.json', review / 'summary.json', predictions_path)},
                   output_sha256={f: sha(out / f) for f in ('packets.jsonl', 'rejections.jsonl')})
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--review', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(audit(args.dataset, args.review, args.output), ensure_ascii=False))


if __name__ == '__main__':
    main()
