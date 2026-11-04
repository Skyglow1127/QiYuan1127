from pathlib import Path
import math
import yaml


def default_config():
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('d1_pick_place')) / 'config/task.yaml')


def load_config(path):
    c = yaml.safe_load(Path(path).read_text())
    for key in ['base_frame', 'camera_frame', 'arm_group', 'tcp_link', 'gripper_joint']:
        if not isinstance(c.get(key), str) or not c[key]:
            raise ValueError(f'Missing {key}')
    ranges = {'cube_size_m': (.01, .1), 'marker_size_m': (.01, .5),
              'clearance_m': (.02, .3), 'place_release_gap_m': (.001, .01),
              'velocity_scaling': (.001, .3), 'acceleration_scaling': (.001, .3),
              'protected_tolerance_rad': (1e-7, .001),
              'planning_time_s': (1, 30), 'max_frame_age_s': (.05, 2),
              'max_sync_skew_s': (.001, .05), 'gripper_open': (0, .03),
              'gripper_closed': (0, .03)}
    for key, (low, high) in ranges.items():
        if not isinstance(c.get(key), (float, int)) or not math.isfinite(c[key]) or not low <= c[key] <= high:
            raise ValueError(f'{key} must be in {low}..{high}')
    if c['gripper_closed'] >= c['gripper_open']:
        raise ValueError('CAD gripper must close to smaller extension')
    if c.get('task_mode', 'transfer') not in ('transfer', 'lift_return'):
        raise ValueError('Unknown task_mode')
    if c.get('task_mode') == 'lift_return':
        lift = c.get('lift_height_m')
        if not isinstance(lift, (int, float)) or not math.isfinite(lift) or not .01 <= lift <= .05:
            raise ValueError('First lift height must be 0.01..0.05 m')
    if c.get('grasp_style', 'top') not in ('top', 'side'):
        raise ValueError('Unknown grasp_style')
    if c.get('grasp_style') == 'side':
        pitch = c.get('side_pitch_deg', 20.)
        if c.get('task_mode') != 'lift_return' or not isinstance(pitch, (int, float)) or not 0 <= pitch <= 30:
            raise ValueError('Side grasp requires lift_return with a 0..30 degree pitch')
    if len(c['arm_joints']) != 6 or len(set(c['arm_joints'])) != 6:
        raise ValueError('Expected six distinct arm joint names')
    # The original blanket J0..3 restriction was lifted by the user. Individual
    # task configurations can still protect any distinct subset of arm joints.
    protected = c.get('protected_joints')
    if (not isinstance(protected, list) or len(set(protected)) != len(protected)
            or any(n not in c['arm_joints'] for n in protected)):
        raise ValueError('protected_joints must be a distinct subset of arm_joints')
    for key, n in [('place_offset_m', 2), ('place_region_m', 2),
                   ('table_size_m', 3), ('table_center_xy_m', 2)]:
        values = c.get(key, [])
        if len(values) != n or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in values):
            raise ValueError(f'Invalid {key}')
    if any(x <= 0 for x in c['table_size_m']):
        raise ValueError('Invalid table size')
    if any(x < c['cube_size_m'] + .02 for x in c['place_region_m']):
        raise ValueError('Place region too small for cube plus margin')
    if not isinstance(c['marker_id'], int) or not 0 <= c['marker_id'] < 50:
        raise ValueError('marker_id must be 0..49')
    return c
