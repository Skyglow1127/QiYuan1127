"""120 mm M4 plate with four hexagonal nut recesses on the underside.

Millimetres; CadQuery 2.8. Ordinary M4 hex nuts: nominal AF 7 mm, height
3.2 mm. Pockets use AF 7.3 mm, depth 3.5 mm, with printing clearance.
"""
from pathlib import Path
from math import cos,pi
import json
import cadquery as cq

OUT=Path(__file__).resolve().parent
WORK=OUT.parent / "work"
WIDTH,DEPTH,THICKNESS=120.0,120.0,10.0
PITCH_X,PITCH_Y,HOLE_D=40.0,50.0,4.5
CORNER_R,EDGE_CHAMFER=3.0,0.4
NUT_AF,NUT_DEPTH=7.3,3.5
HOLES=[(x,y) for x in (-PITCH_X/2,PITCH_X/2)
       for y in (-PITCH_Y/2,PITCH_Y/2)]


def make_plate():
    plate=(cq.Workplane("XY")
           .box(WIDTH,DEPTH,THICKNESS,centered=(True,True,False))
           .edges("|Z").fillet(CORNER_R)
           .edges(">Z").chamfer(EDGE_CHAMFER)
           .edges("<Z").chamfer(EDGE_CHAMFER))
    holes=(cq.Workplane("XY").workplane(offset=-1)
           .pushPoints(HOLES).circle(HOLE_D/2).extrude(THICKNESS+2))
    # CadQuery's polygon diameter is the diameter through the vertices.
    # Convert the specified across-flats distance to that circumdiameter.
    circ_d=NUT_AF/cos(pi/6)
    pockets=(cq.Workplane("XY").workplane(offset=-1)
             .pushPoints(HOLES).polygon(6,circ_d).extrude(NUT_DEPTH+1))
    return plate.cut(holes).cut(pockets).clean()


if __name__=="__main__":
    WORK.mkdir(parents=True,exist_ok=True)
    plate=make_plate()
    assert plate.val().isValid()
    assert len(plate.solids().vals())==1
    cq.exporters.export(plate,str(OUT / "m4_plate_120x120x10_nut_recess.stl"),
                        tolerance=.04,angularTolerance=.10)
    cq.exporters.export(plate,str(OUT / "m4_plate_120x120x10_nut_recess.step"))
    report={"units":"mm","plate_xyz_mm":[WIDTH,DEPTH,THICKNESS],
            "hole_pitch_xy_mm":[PITCH_X,PITCH_Y],"hole_centres_xy_mm":HOLES,
            "through_hole_d_mm":HOLE_D,"nut_pocket_side":"underside, Z=0",
            "nut_pocket_across_flats_mm":NUT_AF,
            "nut_pocket_across_corners_mm":NUT_AF/cos(pi/6),
            "nut_pocket_depth_mm":NUT_DEPTH,
            "remaining_thickness_above_pocket_mm":THICKNESS-NUT_DEPTH,
            "nominal_nut_type":"ordinary M4 hex nut, AF 7 mm, height 3.2 mm",
            "nominal_nut_af_clearance_mm":NUT_AF-7.0,
            "nominal_nut_recess_below_bottom_mm":NUT_DEPTH-3.2,
            "cad_valid":plate.val().isValid(),"solids":1}
    (WORK / "m4-nut-plate-cad-report.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
