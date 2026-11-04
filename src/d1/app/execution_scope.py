"""Select an approved trajectory prefix without changing its geometry or speed."""

FULL_CYCLE = 'full_cycle'
PREGRASP = 'pregrasp'
FIXED_PREGRASP = 'fixed_pregrasp'


def select_execution_stages(stages, scope=FULL_CYCLE):
    if scope == FULL_CYCLE:
        return list(stages)
    if scope == FIXED_PREGRASP:
        if [stage.get('name') for stage in stages] != ['prepare_gripper', 'fixed_pregrasp']:
            raise ValueError('固定预抓取必须使用独立的固定姿态计划，请先准备固定预抓取')
        return list(stages)
    if scope != PREGRASP:
        raise ValueError('未知执行范围；请重新准备并选择完整抓放或预抓取')
    names = [stage.get('name') for stage in stages]
    if 'pregrasp' not in names:
        raise ValueError('当前路径缺少预抓取阶段，请重新准备')
    prefix = names[:names.index('pregrasp') + 1]
    if prefix not in (['pregrasp'], ['prepare_gripper', 'pregrasp']):
        raise ValueError('预抓取前包含其他运动阶段，不能作为一键预抓取路径')
    return list(stages[:len(prefix)])
