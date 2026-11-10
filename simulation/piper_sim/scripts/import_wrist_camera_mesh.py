#!/usr/bin/env python3
"""Rigidly align the supplied STL bracket; preserve its holes and facets."""
import hashlib
import json
import math
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parents[1]

def import_mesh(config):
    spec = config.get('mesh_import')
    if not spec:
        return
    source = ROOT/spec['source']
    raw = source.read_bytes()
    if hashlib.sha256(raw).hexdigest() != spec['source_sha256']:
        raise ValueError('Wrist camera source STL checksum changed')
    count = struct.unpack_from('<I',raw,80)[0]
    if len(raw) != 84 + count*50:
        raise ValueError('Expected a complete binary STL')
    rotation, offset, scale = spec['rotation'], spec['translation'], spec['scale']
    output = bytearray(b'User camera mount; rigid alignment; metres'.ljust(80,b' '))
    output.extend(struct.pack('<I',count))
    for i in range(count):
        record = struct.unpack_from('<12fH',raw,84+i*50)
        points = []
        for j in (3,6,9):
            vertex = record[j:j+3]
            points.append([scale*sum(rotation[a][b]*vertex[b] for b in range(3))+offset[a] for a in range(3)])
        u = [points[1][a]-points[0][a] for a in range(3)]
        v = [points[2][a]-points[0][a] for a in range(3)]
        normal = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
        length = math.sqrt(sum(x*x for x in normal))
        normal = [x/length for x in normal] if length else [0.,0.,0.]
        output.extend(struct.pack('<12fH',*(normal+[x for p in points for x in p]),record[-1]))
    target = ROOT/spec['output']
    target.parent.mkdir(exist_ok=True)
    if not target.exists() or target.read_bytes() != output:
        target.write_bytes(output)
    print(f'Imported supplied camera STL: {count} triangles, units inferred as mm -> m')

if __name__ == '__main__':
    import_mesh(json.loads((ROOT/'config/wrist_camera.json').read_text()))
