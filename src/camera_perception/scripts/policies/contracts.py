"""Public units, causal observation checks, and action provenance for D1 policies."""
from dataclasses import dataclass
import math

CAMERAS = ('global', 'wrist')
SERIALS = {'global': 'CC1365302K0', 'wrist': 'CC1365301EY'}
UNITS = ['degree'] * 6 + ['uncalibrated_sdk_raw']
LIMITS = [(-135,135),(-90,90),(-90,90),(-135,135),(-90,90),(-135,135),(0,65)]


def vector7(value, name):
    if len(value) != 7 or any(isinstance(x, bool) or not isinstance(x,(float,int)) or not math.isfinite(x) for x in value):
        raise ValueError(f'{name}: expected seven finite SDK positions')
    return value


def validate_sample(row, verify_image):
    """Recheck upstream masks; never use a later observation or fill missing targets."""
    if row.get('schema') != 'd1-causal-command-state-v1' or not row.get('trainable'):
        raise ValueError('upstream_sample_not_trainable')
    if not row.get('masks') or not all(row['masks'].values()):
        raise ValueError('invalid_upstream_mask')
    if row.get('labels',{}).get('demonstration_success') is not True:
        raise ValueError('missing_successful_stage_evidence')
    a, o = row['action'], row['observation']
    vector7(a['sdk_target'],'action'); vector7(o['q'],'observation')
    if a['units'] != UNITS or o['units'] != UNITS or a['commanded_joint_mask'] != [True]*7:
        raise ValueError('partial_action_or_units_mismatch')
    for values in (a['sdk_target'], o['q']):
        if any(not lo<=v<=hi for v,(lo,hi) in zip(values,LIMITS)):
            raise ValueError('outside_sdk_limits')
    t=row['command_host_monotonic']
    if not isinstance(t,(int,float)) or not math.isfinite(t):
        raise ValueError('missing_absolute_clock')
    if not 0 <= o['q_age_s'] <= .6 or not o['q_available_t'] < row['command_t']:
        raise ValueError('noncausal_or_stale_feedback')
    for name, expected in [('power_status',1),('enable_status',1),('error_status',0)]:
        s=o['status'][name]
        if s['value']!=expected or not 0<=s['age_s']<=.6:
            raise ValueError('invalid_status')
    for role in CAMERAS:
        c=o['cameras'][role]
        if c['serial']!=SERIALS[role] or not c['available_t']<t or not 0<=c['capture_age_estimate_s']<=.25:
            raise ValueError('noncausal_or_stale_camera')
        verify_image(c['path'],c['sha256'])
    return row


@dataclass(frozen=True)
class ActionChunk:
    commands: list
    target_times: list
    observation_id: str
    model_sha256: str
    action_type: str = 'sdk_joint_position'
    coordinate_frame: str = 'sdk_native'

    def validate(self):
        if self.action_type!='sdk_joint_position' or self.coordinate_frame!='sdk_native':
            raise ValueError('unsupported_action_contract')
        if not self.commands or len(self.commands)!=len(self.target_times):
            raise ValueError('invalid_chunk_length')
        for a in self.commands: vector7(a,'action')
        if any(not math.isfinite(t) for t in self.target_times) or any(b<=a for a,b in zip(self.target_times,self.target_times[1:])):
            raise ValueError('invalid_action_time')
        if not self.observation_id or len(self.model_sha256)!=64:
            raise ValueError('missing_action_provenance')
        return self
