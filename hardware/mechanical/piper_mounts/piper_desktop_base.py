"""Parametric PiPER desktop base, millimetres. Run with CadQuery 2.8.

Interface: 70 x 70 mm, four M5, AgileX PiPER manual section 2.4.1.
This file defines a fit prototype; it does not establish load capacity.
"""
from pathlib import Path
import json
import cadquery as cq

OUT = Path(__file__).resolve().parent
BASE_X, BASE_Y, BASE_T = 200.0, 180.0, 12.0
PAD_X, PAD_Y, TOTAL_H = 112.0, 112.0, 20.0
BASE_R, PAD_R = 14.0, 10.0
ARM_PITCH, ARM_D = 70.0, 5.6
HEAD_D, HEAD_DEPTH = 14.4, 7.0
SLOT_LENGTH, SLOT_WIDTH = 26.0, 8.6
SLOT_X, SLOT_Y = 77.0, 67.0
MESH_LINEAR_TOL, MESH_ANGULAR_TOL = 0.06, 0.10


def make_base():
    # Bottom at Z=0, robot mating plane at Z=20.
    base = (cq.Workplane("XY")
            .box(BASE_X, BASE_Y, BASE_T, centered=(True, True, False))
            .edges("|Z").fillet(BASE_R)
            .edges(">Z").chamfer(0.8)
            .edges("<Z").chamfer(0.4))
    pad = (cq.Workplane("XY").workplane(offset=BASE_T)
           .box(PAD_X, PAD_Y, TOTAL_H-BASE_T, centered=(True, True, False))
           .edges("|Z").fillet(PAD_R))
    part = base.union(pad)
    # Concave pad root fillet increases the section at the height transition.
    roots = [e for e in part.val().Edges()
             if abs(e.Center().z-BASE_T) < 1e-5
             and max(abs(e.Center().x), abs(e.Center().y)) < 60.0]
    part = part.newObject(roots).fillet(3.0)
    part = part.edges(">Z").fillet(1.0)

    arm_xy = [(x, y) for x in (-ARM_PITCH/2, ARM_PITCH/2)
              for y in (-ARM_PITCH/2, ARM_PITCH/2)]
    drill = (cq.Workplane("XY").workplane(offset=-1)
             .pushPoints(arm_xy).circle(ARM_D/2).extrude(TOTAL_H+2))
    heads = (cq.Workplane("XY").workplane(offset=-1)
             .pushPoints(arm_xy).circle(HEAD_D/2).extrude(HEAD_DEPTH+1))
    part = part.cut(drill).cut(heads)

    table_xy = [(x, y) for x in (-SLOT_X, SLOT_X) for y in (-SLOT_Y, SLOT_Y)]
    table_slots = (cq.Workplane("XY").workplane(offset=-1)
                   .pushPoints(table_xy).slot2D(SLOT_LENGTH, SLOT_WIDTH, 0)
                   .extrude(TOTAL_H+2))
    part = part.cut(table_slots)
    # Rear cable ties; neither slot intersects the robot's support pad.
    ties = (cq.Workplane("XY").workplane(offset=-1)
            .pushPoints([(-16.0, 77.0), (16.0, 77.0)])
            .slot2D(16.0, 4.0, 0).extrude(TOTAL_H+2))
    return part.cut(ties).clean()


if __name__ == "__main__":
    (OUT.parent / "work").mkdir(parents=True, exist_ok=True)
    model = make_base()
    solid = model.val()
    assert solid.isValid(), "Invalid CAD solid"
    assert len(model.solids().vals()) == 1, "Expected one connected solid"
    cq.exporters.export(model, str(OUT / "piper_desktop_base_v1.stl"),
                        tolerance=MESH_LINEAR_TOL, angularTolerance=MESH_ANGULAR_TOL)
    cq.exporters.export(model, str(OUT / "piper_desktop_base_v1.step"))
    vertices, faces = solid.tessellate(MESH_LINEAR_TOL, MESH_ANGULAR_TOL)
    mesh = {"vertices": [[round(v.x, 4), round(v.y, 4), round(v.z, 4)] for v in vertices],
            "faces": [list(f) for f in faces]}
    (OUT.parent / "work" / "mesh-preview.json").write_text(json.dumps(mesh, separators=(",", ":")))
    bb = solid.BoundingBox()
    report = {"units": "mm", "cad_valid": solid.isValid(), "cad_solids": len(model.solids().vals()),
              "cad_bounds_mm": [bb.xlen, bb.ylen, bb.zlen],
              "cad_volume_mm3": solid.Volume(),
              "arm_holes_mm": {"pitch": ARM_PITCH, "diameter": ARM_D,
                               "underside_counterbore_diameter": HEAD_D,
                               "underside_counterbore_depth": HEAD_DEPTH},
              "table_slots_mm": {"length": SLOT_LENGTH, "width": SLOT_WIDTH,
                                 "centers": [[x, y] for x in (-SLOT_X, SLOT_X) for y in (-SLOT_Y, SLOT_Y)]},
              "mesh_triangles": len(faces), "mesh_vertices": len(vertices),
              "mesh_linear_tolerance_mm": MESH_LINEAR_TOL,
              "load_validated": False}
    (OUT.parent / "work" / "cad-report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
