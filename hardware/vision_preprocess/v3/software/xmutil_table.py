"""Strict xmutil table parser reused from the existing pressure deployment flow."""
import re

def parse_listapps(output):
    """Accept only known AMD table layouts; unknown output fails closed.

    Primary format references: AMD Kria application running-on-board docs and
    KD240 app_deployment docs (the latter adds a Pid column). The installed
    dfx-mgr 2024.2 table uses slot->handle; retain both values explicitly.
    """
    rows, header = [], None
    for raw in output.splitlines():
        line = raw.strip()
        if not line or re.fullmatch(r'Socket [0-9]+ closed by client', line):
            continue
        fields = line.split()
        if fields[0] == 'Accelerator':
            if header is not None:
                raise ValueError('重复的 xmutil 表头')
            expected = ['Accelerator', 'Accel_type', 'Base', 'Base_type',
                        '#slots(PL+AIE)', 'Active_slot']
            normalized = [f for f in fields if f != 'Pid']
            normalized = ['#slots(PL+AIE)' if f == '#slots(RPU+PL+AIE)' else
                          'Active_slot' if f == 'slot->handle' else f for f in normalized]
            if normalized != expected or fields.count('Pid') > 1:
                raise ValueError('不支持的 xmutil 表头：' + line)
            if 'Pid' in fields and fields.index('Pid') != 3:
                raise ValueError('无法识别 Pid 列位置')
            header = fields
            continue
        if header is None or len(fields) != len(header):
            raise ValueError('无法严格解析 xmutil 行：' + line)
        row = dict(zip(header, fields))
        if not re.fullmatch(r'[A-Za-z0-9_.+-]+', row['Accelerator']):
            raise ValueError('无效应用名')
        slot_key = 'Active_slot' if 'Active_slot' in row else 'slot->handle'
        value = row[slot_key]
        if value == '-1':
            slots = []
            handles = []
        else:
            if not re.fullmatch(r'[0-9]+(?:->[0-9]+)?(?:,[0-9]+(?:->[0-9]+)?)*,?', value):
                raise ValueError('无法识别 slot：' + value)
            slots = []
            handles = []
            for item in value.rstrip(',').split(','):
                pair = item.split('->')
                if slot_key == 'Active_slot' and len(pair) != 1:
                    raise ValueError('Active_slot 列不应包含 handle')
                if slot_key == 'slot->handle' and len(pair) != 2:
                    raise ValueError('slot->handle 列缺少映射')
                slots.append(int(pair[0]))
                handles.append(int(pair[-1]))
            if len(set(slots)) != len(slots):
                raise ValueError('重复 active slot')
        slot_count_key = next(k for k in row if k.startswith('#slots'))
        if not re.fullmatch(r'\([0-9]+\+[0-9]+(?:\+[0-9]+)?\)', row[slot_count_key]):
            raise ValueError('无法识别 slots 数量')
        row['active_slots'] = slots
        row['active_handles'] = handles
        rows.append(row)
    if header is None or not rows:
        raise ValueError('xmutil 未返回非空可识别列表')
    if len({row['Accelerator'] for row in rows}) != len(rows):
        raise ValueError('存在重复应用名')
    return rows
