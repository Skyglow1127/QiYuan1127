"""PiPER flat plate for a table-edge G/C/F clamp. Millimetres, CadQuery 2.8.

The clamp presses down on this flat plate and supports the underside of the
table. Clamp contact zones are clearance annotations, not modeled features.
The robot attaches using four M5 screws. No raised platform or lower block.
Fit prototype only: actual clamp, table and dynamic load are not validated.
"""
from pathlib import Path
import json
import cadquery as cq

OUT = Path(__file__).resolve().parent
WORK = OUT.parent / "work"
PLATE_X, PLATE_Y, PLATE_T = 200.0, 180.0, 14.0
CORNER_R = 10.0
ARM_PITCH, ARM_D = 70.0, 5.6
HEAD_D, HEAD_DEPTH = 14.4, 7.0
CLAMP_ZONES = [(-75.0, -68.0), (75.0, -68.0)]
ZONE_X, ZONE_Y = 36.0, 32.0
MESH_TOL, MESH_ANGLE = 0.06, 0.10


def make_base():
    # The two principal surfaces are parallel planes at Z=0 and Z=14.
    plate = (cq.Workplane("XY")
             .box(PLATE_X, PLATE_Y, PLATE_T, centered=(True, True, False))
             .edges("|Z").fillet(CORNER_R)
             .edges(">Z").chamfer(0.6)
             .edges("<Z").chamfer(0.3))
    xy = [(x,y) for x in (-ARM_PITCH/2,ARM_PITCH/2)
          for y in (-ARM_PITCH/2,ARM_PITCH/2)]
    holes = (cq.Workplane("XY").workplane(offset=-1)
             .pushPoints(xy).circle(ARM_D/2).extrude(PLATE_T+2))
    heads = (cq.Workplane("XY").workplane(offset=-1)
             .pushPoints(xy).circle(HEAD_D/2).extrude(HEAD_DEPTH+1))
    # Intentionally no clamp grooves, feet, lugs, blocks or table screw holes.
    return plate.cut(holes).cut(heads).clean()


if __name__ == "__main__":
    WORK.mkdir(parents=True,exist_ok=True)
    model = make_base()
    solid = model.val()
    assert solid.isValid(), "Invalid CAD solid"
    assert len(model.solids().vals()) == 1, "Expected one solid"
    cq.exporters.export(model,str(OUT / "piper_flat_clamp_base_v3.stl"),
                        tolerance=MESH_TOL,angularTolerance=MESH_ANGLE)
    cq.exporters.export(model,str(OUT / "piper_flat_clamp_base_v3.step"))
    report={"units":"mm","cad_valid":True,"solids":1,
            "plate_xyz_mm":[PLATE_X,PLATE_Y,PLATE_T],
            "front_table_edge_y_mm":-PLATE_Y/2,
            "clamp_contact_zone_centers_xy_mm":CLAMP_ZONES,
            "clamp_contact_zone_size_xy_mm":[ZONE_X,ZONE_Y],
            "contact_zones_are_annotations_only":True,
            "clamp_type":"table-edge clamp pressing plate and tabletop together vertically",
            "arm_hole_pitch_mm":ARM_PITCH,"arm_clearance_diameter_mm":ARM_D,
            "underside_counterbore_diameter_mm":HEAD_D,
            "underside_counterbore_depth_mm":HEAD_DEPTH,
            "screw_bearing_to_top_mm":PLATE_T-HEAD_DEPTH,
            "volume_mm3":solid.Volume(),"load_validated":False}
    (WORK / "flat-clamp-cad-report.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
