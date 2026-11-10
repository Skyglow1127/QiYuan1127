"""Open square box. Dimensions in millimetres; CadQuery 2.8.

External 150 x 150 x 80, 3 mm walls, 4 mm floor. No lid.
"""
from pathlib import Path
import json
import cadquery as cq

OUT = Path(__file__).resolve().parent
WORK = OUT.parent / "work"
OUTER_X, OUTER_Y, HEIGHT = 150.0, 150.0, 80.0
WALL, FLOOR = 3.0, 4.0
OUTER_R, INNER_R = 4.0, 1.0
RIM_CHAMFER = 0.4


def make_box():
    outer = (cq.Workplane("XY")
             .box(OUTER_X,OUTER_Y,HEIGHT,centered=(True,True,False))
             .edges("|Z").fillet(OUTER_R))
    cavity = (cq.Workplane("XY").workplane(offset=FLOOR)
              .box(OUTER_X-2*WALL,OUTER_Y-2*WALL,HEIGHT-FLOOR+1,
                   centered=(True,True,False))
              .edges("|Z").fillet(INNER_R))
    return outer.cut(cavity).edges(">Z").chamfer(RIM_CHAMFER).clean()


if __name__ == "__main__":
    WORK.mkdir(parents=True,exist_ok=True)
    part = make_box()
    assert part.val().isValid(), "Invalid CAD solid"
    assert len(part.solids().vals()) == 1, "Expected a single connected box"
    cq.exporters.export(part,str(OUT / "square_box_150x150x80.stl"),
                        tolerance=0.06,angularTolerance=0.10)
    cq.exporters.export(part,str(OUT / "square_box_150x150x80.step"))
    report={"units":"mm","type":"open box, no lid",
            "external_xyz_mm":[OUTER_X,OUTER_Y,HEIGHT],
            "wall_mm":WALL,"floor_mm":FLOOR,
            "internal_xy_mm":[OUTER_X-2*WALL,OUTER_Y-2*WALL],
            "internal_depth_mm":HEIGHT-FLOOR,
            "external_corner_radius_mm":OUTER_R,
            "internal_corner_radius_mm":INNER_R,
            "rim_chamfer_mm":RIM_CHAMFER,
            "cad_valid":part.val().isValid(),"solids":len(part.solids().vals()),
            "material_volume_mm3":part.val().Volume()}
    (WORK / "square-box-cad-report.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
