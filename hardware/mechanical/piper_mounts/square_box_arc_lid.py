"""Arched tea-cup-style lid with a circular lifting eye for the 150 mm box.

All dimensions in mm; CadQuery 2.8. The seating plane is Z=0, with a 5 mm
locating lip below it. Place this plane at Z=80 on the matching box.
"""
from pathlib import Path
import json
import cadquery as cq

OUT = Path(__file__).resolve().parent
WORK = OUT.parent / "work"
LID_XY, RIM_T, LID_R = 154.0, 3.0, 6.0
CROWN_D, CROWN_RISE, SHELL_T = 130.0, 12.0, 3.0
LOCATOR_XY, LOCATOR_WALL, LOCATOR_DROP = 143.0, 2.0, 5.0
EYE_OUTER_D, EYE_HOLE_D, EYE_THICK, EYE_CENTER_Z = 26.0, 16.0, 8.0, 24.0
BOX_OPENING, BOX_HEIGHT = 144.0, 80.0


def make_lid():
    a=CROWN_D/2
    radius=(a*a+CROWN_RISE*CROWN_RISE)/(2*CROWN_RISE)
    apex=RIM_T+CROWN_RISE
    center_z=apex-radius
    brim=(cq.Workplane("XY").box(LID_XY,LID_XY,RIM_T,centered=(True,True,False))
          .edges("|Z").fillet(LID_R)
          .edges(">Z").fillet(0.6)
          .edges("<Z").chamfer(0.3))
    outer_sphere=cq.Workplane("XY").sphere(radius).translate((0,0,center_z))
    crop=(cq.Workplane("XY").workplane(offset=RIM_T)
          .box(CROWN_D+2,CROWN_D+2,CROWN_RISE+1,centered=(True,True,False)))
    cap=outer_sphere.intersect(crop)
    lid=brim.union(cap)
    transition=[e for e in lid.val().Edges()
                if e.geomType()=="CIRCLE" and abs(e.Center().z-RIM_T)<1e-5
                and abs(e.radius()-a)<1e-3]
    lid=lid.newObject(transition).fillet(1.5)
    # Cut through the centre of both cap and brim: the crown is a true shell.
    inner_sphere=cq.Workplane("XY").sphere(radius-SHELL_T).translate((0,0,center_z))
    lid=lid.cut(inner_sphere)

    outer_lip=(cq.Workplane("XY").workplane(offset=-LOCATOR_DROP)
               .box(LOCATOR_XY,LOCATOR_XY,LOCATOR_DROP+0.25,centered=(True,True,False))
               .edges("|Z").fillet(3.0))
    inner_lip=(cq.Workplane("XY").workplane(offset=-LOCATOR_DROP-1)
               .box(LOCATOR_XY-2*LOCATOR_WALL,LOCATOR_XY-2*LOCATOR_WALL,
                    LOCATOR_DROP+2,centered=(True,True,False))
               .edges("|Z").fillet(1.0))
    lip=outer_lip.cut(inner_lip).edges("<Z").chamfer(0.45)
    lid=lid.union(lip)

    # The circular eye lies in XZ; the 16 mm hole runs horizontally along Y.
    eye=(cq.Workplane("XZ").workplane(offset=-EYE_THICK/2)
         .center(0,EYE_CENTER_Z).circle(EYE_OUTER_D/2).circle(EYE_HOLE_D/2)
         .extrude(EYE_THICK).edges().fillet(0.8))
    return lid.union(eye).clean()


if __name__=="__main__":
    WORK.mkdir(parents=True,exist_ok=True)
    lid=make_lid()
    assert lid.val().isValid(),"Invalid CAD lid"
    assert len(lid.solids().vals())==1,"Lid must be one connected part"
    cq.exporters.export(lid,str(OUT / "square_box_arc_lid.stl"),tolerance=.06,angularTolerance=.10)
    cq.exporters.export(lid,str(OUT / "square_box_arc_lid.step"))
    report={"units":"mm","lid_outline_xy_mm":[LID_XY,LID_XY],
            "seating_plane_z_mm":0,"matching_box_height_mm":BOX_HEIGHT,
            "rim_thickness_mm":RIM_T,"crown_diameter_mm":CROWN_D,
            "crown_rise_mm":CROWN_RISE,"crown_shell_thickness_mm":SHELL_T,
            "locator_outer_xy_mm":LOCATOR_XY,"locator_wall_mm":LOCATOR_WALL,
            "locator_drop_mm":LOCATOR_DROP,"fit_clearance_per_side_mm":(BOX_OPENING-LOCATOR_XY)/2,
            "lifting_eye_outer_d_mm":EYE_OUTER_D,"lifting_hole_d_mm":EYE_HOLE_D,
            "lifting_eye_thickness_mm":EYE_THICK,"lifting_eye_axis":"Y",
            "cad_valid":lid.val().isValid(),"solids":len(lid.solids().vals()),
            "material_volume_mm3":lid.val().Volume()}
    (WORK / "arc-lid-cad-report.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
