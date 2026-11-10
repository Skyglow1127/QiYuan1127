#!/usr/bin/env python3
"""Optional rebuild: pip install 'numpy<2' trimesh fast-simplification.
This affects the browser Canvas fallback only, never Gazebo geometry.
"""
import trimesh,json,pathlib
root=pathlib.Path(__file__).resolve().parents[1];data={}
for p in (root/'meshes').glob('*.STL'):
 m=trimesh.load(p)
 if len(m.faces)>2500:m=m.simplify_quadric_decimation(face_count=2500)
 data[p.name]={'vertices':m.vertices.round(7).tolist(),'faces':m.faces.tolist()}
(root/'generated/display_meshes.json').write_text(json.dumps(data,separators=(',',':')))
print('Display triangles',sum(len(m['faces']) for m in data.values()))
