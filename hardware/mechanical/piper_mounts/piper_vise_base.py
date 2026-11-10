"""PiPER vise-held base V2. All dimensions are millimetres; CadQuery 2.8.

The four M5 screws attach the robot to this adapter. A bench vise grips the
integral lower block along Y. No bolt holes are provided for a worktop.
This is a fit prototype, without validated clamp force or load capacity.
"""
from pathlib import Path
import json
import cadquery as cq

OUT = Path(__file__).resolve().parent
WORK = OUT.parent / "work"
BASE_X, BASE_Y, BASE_T = 150.0, 140.0, 20.0
DECK_Z = BASE_T
BASE_R = 12.0
JAW_LENGTH, JAW_WIDTH, JAW_DROP = 100.0, 40.0, 35.0
JAW_ROOT_R, JAW_CORNER_R, JAW_BOTTOM_CHAMFER = 4.0, 2.0, 0.8
ARM_PITCH, ARM_D = 70.0, 5.6
HEAD_D, HEAD_DEPTH = 14.4, 7.0
MESH_TOL, MESH_ANGLE = 0.06, 0.10


def make_base():
    plate = (cq.Workplane("XY")
             .box(BASE_X, BASE_Y, BASE_T, centered=(True, True, False))
             .edges("|Z").fillet(BASE_R)
             .edges(">Z").chamfer(0.8)
             .edges("<Z").chamfer(0.4))
    # A flat, uniform deck can lie on the printer bed with the grip block up.
    part = plate

    block = (cq.Workplane("XY").workplane(offset=-JAW_DROP)
             .box(JAW_LENGTH, JAW_WIDTH, JAW_DROP, centered=(True, True, False))
             .edges("|Z").fillet(JAW_CORNER_R)
             .edges("<Z").chamfer(JAW_BOTTOM_CHAMFER))
    part = part.union(block)
    roots = [e for e in part.val().Edges()
             if abs(e.Center().z) < 1e-5
             and abs(e.Center().x) < JAW_LENGTH/2+0.1
             and abs(e.Center().y) < JAW_WIDTH/2+0.1]
    part = part.newObject(roots).fillet(JAW_ROOT_R)

    # Screw holes remain outside the lower block and its root fillets.
    arm_xy = [(x,y) for x in (-ARM_PITCH/2, ARM_PITCH/2)
              for y in (-ARM_PITCH/2, ARM_PITCH/2)]
    drill = (cq.Workplane("XY").workplane(offset=-1)
             .pushPoints(arm_xy).circle(ARM_D/2).extrude(DECK_Z+2))
    heads = (cq.Workplane("XY").workplane(offset=-1)
             .pushPoints(arm_xy).circle(HEAD_D/2).extrude(HEAD_DEPTH+1))
    return part.cut(drill).cut(heads).clean()


if __name__ == "__main__":
    WORK.mkdir(parents=True, exist_ok=True)
    part = make_base()
    solid = part.val()
    assert solid.isValid(), "Invalid CAD solid"
    assert len(part.solids().vals()) == 1, "Expected a single connected solid"
    cq.exporters.export(part,str(OUT / "piper_vise_base_v2.stl"),
                        tolerance=MESH_TOL,angularTolerance=MESH_ANGLE)
    cq.exporters.export(part,str(OUT / "piper_vise_base_v2.step"))
    vertices,faces=solid.tessellate(MESH_TOL,MESH_ANGLE)
    preview={"vertices":[[round(v.x,4),round(v.y,4),round(v.z,4)] for v in vertices],
             "faces":[list(f) for f in faces]}
    (WORK / "vise-mesh-preview.json").write_text(json.dumps(preview,separators=(",",":")))
    report={"units":"mm","cad_valid":True,"solids":1,
            "plate_xy_mm":[BASE_X,BASE_Y],"above_plate_bottom_mm":DECK_Z,
            "clamping_block_lwh_mm":[JAW_LENGTH,JAW_WIDTH,JAW_DROP],
            "clamping_direction":"Y","straight_jaw_contact_length_mm":JAW_LENGTH-2*JAW_CORNER_R,
            "straight_jaw_contact_height_mm":JAW_DROP-JAW_ROOT_R-JAW_BOTTOM_CHAMFER,
            "clamping_contact_z_mm":[-JAW_DROP+JAW_BOTTOM_CHAMFER,-JAW_ROOT_R],
            "root_fillet_mm":JAW_ROOT_R,"volume_mm3":solid.Volume(),
            "arm_holes_pitch_mm":ARM_PITCH,"arm_holes_diameter_mm":ARM_D,
            "underside_counterbore_diameter_mm":HEAD_D,"underside_counterbore_depth_mm":HEAD_DEPTH,
            "screw_recess_to_root_gap_y_mm":ARM_PITCH/2-HEAD_D/2-(JAW_WIDTH/2+JAW_ROOT_R),
            "mesh_triangles":len(faces),"load_validated":False}
    (WORK / "vise-cad-report.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
