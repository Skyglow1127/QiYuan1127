"""120 x 120 x 10 mm plate, centred 40 x 50 mm M4 clearance-hole pattern.

CadQuery 2.8. X pitch is 40 mm; Y pitch is 50 mm.
M4 locations use 4.5 mm through holes, without threads or counterbores.
"""
from pathlib import Path
import json
import cadquery as cq

OUT = Path(__file__).resolve().parent
WORK = OUT.parent / "work"
WIDTH, DEPTH, THICKNESS = 120.0, 120.0, 10.0
PITCH_X, PITCH_Y, HOLE_D = 40.0, 50.0, 4.5
CORNER_R, EDGE_CHAMFER = 3.0, 0.4
HOLES = [(x,y) for x in (-PITCH_X/2,PITCH_X/2)
         for y in (-PITCH_Y/2,PITCH_Y/2)]


def make_plate():
    plate = (cq.Workplane("XY")
             .box(WIDTH,DEPTH,THICKNESS,centered=(True,True,False))
             .edges("|Z").fillet(CORNER_R)
             .edges(">Z").chamfer(EDGE_CHAMFER)
             .edges("<Z").chamfer(EDGE_CHAMFER))
    tool = (cq.Workplane("XY").workplane(offset=-1)
            .pushPoints(HOLES).circle(HOLE_D/2).extrude(THICKNESS+2))
    return plate.cut(tool).clean()


if __name__ == "__main__":
    WORK.mkdir(parents=True,exist_ok=True)
    plate = make_plate()
    assert plate.val().isValid()
    assert len(plate.solids().vals()) == 1
    cq.exporters.export(plate,str(OUT / "m4_plate_120x120x10.stl"),
                        tolerance=.04,angularTolerance=.10)
    cq.exporters.export(plate,str(OUT / "m4_plate_120x120x10.step"))
    report={"units":"mm","plate_xyz_mm":[WIDTH,DEPTH,THICKNESS],
            "hole_pitch_xy_mm":[PITCH_X,PITCH_Y],"hole_centres_xy_mm":HOLES,
            "hole_diameter_mm":HOLE_D,"hole_type":"M4 clearance, through, unthreaded",
            "edge_margins_to_hole_centres_xy_mm":[(WIDTH-PITCH_X)/2,(DEPTH-PITCH_Y)/2],
            "corner_radius_mm":CORNER_R,"edge_chamfer_mm":EDGE_CHAMFER,
            "cad_valid":plate.val().isValid(),"solids":1}
    (WORK / "m4-plate-cad-report.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
