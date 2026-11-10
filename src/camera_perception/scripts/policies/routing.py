"""Validate a Qwen/high-level TaskSpec before routing known skills.

This adapter does not generate Qwen text or execute geometry/ACT. The current
ACT lift pilot is shadow-only; it cannot replace a full grasp cycle.
"""
import json
import math

TASKSPEC_PROMPT = '''Return one JSON object with target="red_cube", skill_id="pick_lift_return", lift_height_m, return_to_origin=true. Use meters. Only select this skill for an explicitly requested red cube pick/lift/return task. If unsupported, return {"unsupported":true}. Never output joint values or claim execution.'''


def validate_task_spec(text):
    text=text.strip()
    lines=text.splitlines()
    if len(lines)>=3 and lines[0] in ('```json','```') and lines[-1]=='```':
        text='\n'.join(lines[1:-1])
    d=json.loads(text)
    if not isinstance(d,dict) or set(d)!={'target','skill_id','lift_height_m','return_to_origin'}:
        raise ValueError('unsupported_or_malformed_task_spec')
    if d['target']!='red_cube' or d['skill_id']!='pick_lift_return' or d['return_to_origin'] is not True:
        raise ValueError('unknown_target_or_skill')
    h=d['lift_height_m']
    if type(h) not in (int,float) or not math.isfinite(h) or not 0<h<=.10:
        raise ValueError('unsupported_height')
    return d


def route(task_spec,stage,mode='shadow'):
    validate_task_spec(json.dumps(task_spec))
    if stage not in ('approach','close','lift','lower','open','retreat'):raise ValueError('unknown_stage')
    if mode not in ('shadow','geometry'):raise ValueError('ACT is shadow-only')
    return dict(executing_provider='geometry',shadow_provider='act' if mode=='shadow' and stage=='lift' else None,
                stage=stage,learned_skill='red_cube_lift_fragment' if stage=='lift' else None,
                task_spec=task_spec,hardware_executed=False,requires_concrete_plan_approval=True)
