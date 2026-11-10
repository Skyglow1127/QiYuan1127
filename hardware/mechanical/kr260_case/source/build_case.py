"""KR260 V2 protective enclosure. CadQuery 2.8.0; millimetres.
Original fan/heatsink and four feet retained, no expansion boards installed.
Real board and plug profiles are not available: port windows are generous envelopes.
"""
from pathlib import Path
import argparse,json,math
import cadquery as cq
from geometry import box,rr,cyl,foot_pads,guides,make_gauge,reference_board,P as P1
P=dict(P1)
P.update(wall=2.8,base_rim_z=5.8,hood_bottom=5.8,lid_under=51.2,lid_top=54.0,
         collar_w=157.6,collar_d=144.6,collar_t=2.4,collar_top=10.2,
         fan_cx=15.0,fan_cy=8.0,vent_slot_w=3.2,vent_pitch=4.8,
         hatch_x=-48.0,hatch_y=-3.0,hatch_w=40.0,hatch_d=90.0,
         hatch_gap=0.3,hatch_top=56.4,clip_y=45.0,clip_width=9.0,
         clip_t=1.6,clip_anchor_z=31.0)
# Open-bottom hood windows: x0,x1,upper-z. Board coordinate frame in source drawings.
FRONT=[(-55,-17,44),(-15,66,35)]
REAR=[(-65,65,33)]

def cut_ports(s,z0=-1):
    for side,spans in [(-1,FRONT),(1,REAR)]:
        for x0,x1,zt in spans:
            s=s.cut(box(x1-x0,15,zt-z0,(x0+x1)/2,side*74,z0))
    return s

def sd_cut(z0=-1,z1=18.0):
    return box(30,36,z1-z0,69,3,z0)

def base_shape(p):
    b=rr(p['outer_w'],p['outer_d'],p['floor_t'],7)
    rim=rr(164,151,p['base_rim_z']-2.8,7,z=2.8).cut(rr(153.0,140.0,5,3.4,z=2.7))
    collar=rr(p['collar_w'],p['collar_d'],p['collar_top']-5.8,3.8,z=5.8)
    collar=collar.cut(rr(p['collar_w']-4.8,p['collar_d']-4.8,6,1.4,z=5.7))
    # Front and rear openings extend through the collar: do not bury low connectors.
    b=b.union(cut_ports(rim)).union(cut_ports(collar)).union(foot_pads(p)).union(guides(p))
    # SD finger bay in floor, with open side access. Bottom otherwise remains solid.
    b=b.cut(sd_cut(-1,18))
    # Four shallow grooves receive hood hooks. Outside x=78.8; root x=77.5.
    for sx in [-1,1]:
        for sy in [-1,1]:
            b=b.cut(box(4,11,2.2,sx*79.5,sy*p['clip_y'],6.4))
            # Fingernail relief below each cantilever for outward release.
            b=b.cut(box(4.6,8,2.1,sx*81.7,sy*p['clip_y'],3.7))
    return b.clean()

def side_slot(y,p,sx):
    # Vertical capsule: only 3.2 mm maximum unsupported span in inverted print.
    r=1.6;zt=42.0;zb=23.0
    wp=cq.Workplane('YZ',origin=(sx*79.5,0,0))
    s=wp.center(y,(zt+zb)/2).slot2D(zt-zb,2*r,90).extrude(8,both=True)
    return s

def hook(p,sx,sy):
    # YZ is not used here: XZ cross-section, extruded along Y.
    # Flat retention surface at 8.0; lower insertion ramp rises from z=5.8.
    pts=[(80.4,5.8),(80.4,8.0),(78.15,8.0),(78.15,7.5)]
    h=cq.Workplane('XZ').polyline(pts).close().extrude(p['clip_width']/2,both=True).translate((0,sy*p['clip_y'],0))
    if sx<0:h=h.mirror('YZ')
    return h

def hood_shape(p):
    bottom=p['hood_bottom'];top=p['lid_top']
    h=rr(164,151,top-bottom,7,z=bottom)
    h=h.cut(rr(158.4,145.4,p['lid_under']-bottom+0.01,4.2,z=bottom-0.01))
    h=cut_ports(h,bottom-0.1)
    # SD opening reaches the lower edge: no closing sill to bridge in inverted print.
    h=h.cut(sd_cut(bottom-0.1,18.0))
    # Fan grille: 16 columns x 2 rows, with a 3 mm centre tie.
    for x in [15+(i-7.5)*p['vent_pitch'] for i in range(16)]:
        for y in [-12.25,28.25]:
            slot=rr(p['vent_slot_w'],37.5,5,1.5,z=50.8).translate((x,y,0))
            h=h.cut(slot)
    # Both sides exhaust / intake; no vent immediately over the clip anchors.
    for sx in [-1,1]:
        for y in [-33,-26,-19,-12,-5,2,9,16,23,30]:
            h=h.cut(side_slot(y,p,sx))
    hatch=rr(p['hatch_w'],p['hatch_d'],5,3,z=50.8).translate((p['hatch_x'],p['hatch_y'],0))
    h=h.cut(hatch)
    # Nail access: notch beneath front overhang of the small removable hatch.
    h=h.cut(box(10,4,1.2,p['hatch_x'],-49,53.0))
    for sx in [-1,1]:
        for sy in [-1,1]:
            y=sy*p['clip_y']
            # Slits separate long PETG cantilevers, anchored at z=31.
            for off in [-5.5,5.5]:
                h=h.cut(box(7,1.6,p['clip_anchor_z']-bottom,sx*80.5,y+off,bottom-0.01))
            # Thin the tongue from the inside; width between slits =9.4.
            h=h.cut(box(1.25,9.4,p['clip_anchor_z']-bottom,sx*79.775,y,bottom-0.01))
            h=h.union(hook(p,sx,sy))
    return h.clean()

def hatch_shape(p):
    # Print inverted: cover flange on bed, insertion plug above it.
    cap=rr(44,94,2.4,5,z=54).translate((p['hatch_x'],p['hatch_y'],0))
    plug=rr(p['hatch_w']-2*p['hatch_gap'],p['hatch_d']-2*p['hatch_gap'],2.5,2.7,z=51.6).translate((p['hatch_x'],p['hatch_y'],0))
    return cap.union(plug).clean()

def inverse_print(s,top):
    return s.rotate((0,0,0),(1,0,0),180).translate((0,0,top))

def coupon(p,b=None,h=None):
    # A local section of one actual collar/clip, exported as 2 separate print parts.
    clip=(h if h is not None else hood_shape(p)).intersect(box(12,17,33,78,45,3))
    recv=(b if b is not None else base_shape(p)).intersect(box(14,17,12,77,45,0))
    # Clip prints sideways, so the exact height relationship can be test-fitted.
    clip=clip.rotate((0,0,0),(0,1,0),90)
    bb=clip.val().BoundingBox();clip=clip.translate((-bb.xmin,-bb.ymin,-bb.zmin))
    bb=recv.val().BoundingBox();recv=recv.translate((-bb.xmin,-bb.ymin,-bb.zmin))
    return clip,recv

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=Path(__file__).resolve().parents[1]);a=ap.parse_args();o=a.out
    for d in ['stl','step','images']:(o/d).mkdir(parents=True,exist_ok=True)
    b=base_shape(P);h=hood_shape(P);cap=hatch_shape(P);g=make_gauge(P)
    parts=[('00_board_fit_gauge',g,g),('01_base',b,b),('02_hood_inverted',h,inverse_print(h,54)),('03_service_hatch_inverted',cap,inverse_print(cap,56.4).translate((48,-3,0)))]
    # Using only the local section avoids spending a full shell on snap calibration.
    clip,recv=coupon(P,b,h)
    parts.extend([('04_clip_test',clip,clip),('05_socket_test',recv,recv)])
    report={}
    import trimesh
    for name,s,pr in parts:
        assert s.val().isValid(),f'{name}: invalid CAD'
        assert len(s.solids().vals())==1,(name,len(s.solids().vals()))
        cq.exporters.export(s,str(o/'step'/f'{name}.step'))
        cq.exporters.export(pr,str(o/'stl'/f'{name}.stl'),tolerance=.04,angularTolerance=.09)
        m=trimesh.load(o/'stl'/f'{name}.stl',force='mesh')
        n=len(m.split());assert m.is_watertight and m.is_winding_consistent and n==1,(name,n)
        report[name]=dict(watertight=bool(m.is_watertight),consistent_winding=bool(m.is_winding_consistent),components=n,dimensions_mm=m.extents.round(3).tolist(),min_z_mm=round(float(m.bounds[0,2]),4),volume_cm3=round(float(m.volume/1000),2))
    ass=cq.Assembly(name='KR260_protective_case_V2')
    ass.add(b,name='base',color=cq.Color('#2e5661'));ass.add(h,name='hood',color=cq.Color('#375e68'));ass.add(cap,name='service_hatch',color=cq.Color('#dd9551'))
    ass.export(str(o/'step'/'case_assembly.step'))
    inter={}
    for na,sa,nb,sb in [('base',b,'hood',h),('hood',h,'hatch',cap),('base',b,'hatch',cap)]:
        v=sa.intersect(sb).val().Volume();inter[f'{na}:{nb}']=round(v,6)
        assert v<1e-5,(na,nb,v)
    report['case_intersections_mm3']=inter
    collisions={}
    for name,s in reference_board(P):
        for part,case in [('base',b),('hood',h),('hatch',cap)]:
            v=s.intersect(case).val().Volume()
            if v>1e-5:collisions[f'{part}:{name}']=round(v,4)
    report['approximate_board_envelope_intersections_mm3']=collisions
    assert not collisions,collisions
    cable_hits={}
    proxy_cables=[]
    for x in [-42,-26]:
        for z in [20,33]:proxy_cables.append(('RJ45',box(16,45,12,x,-75,z-6)))
    for x in [-5,14]:
        for z in [17,26]:proxy_cables.append(('USB',box(18,45,8,x,-75,z-4)))
    proxy_cables += [('DP',box(26,45,14,36,-75,11)),('DC',box(14,45,14,54,-75,12)),('SFP',box(20,50,14,-39,78,10)),('microUSB',box(12,40,9,-55,74,9.5))]
    for x in [-17,6,29,52]:proxy_cables.append(('Pmod',box(22,45,14,x,74,10)))
    for i,(name,s) in enumerate(proxy_cables):
        for part,case in [('base',b),('hood',h)]:
            v=s.intersect(case).val().Volume()
            if v>1e-5:cable_hits[f'{part}:{name}:{i}']=round(v,4)
    report['assumed_straight_plug_corridor_intersections_mm3']=cable_hits
    assert not cable_hits,cable_hits
    report['assembled_dimensions_mm']=[164,151,56.4]
    # Capsule area summed exactly (rounded rectangle slot: 3.2x37.5, R1.5).
    report['top_fan_grille_open_area_mm2']=round(32*(3.2*37.5-(4-math.pi)*1.5**2),1)
    report['fan_grille_slot_width_mm']=3.2
    report['lid_to_nominal_kit_clearance_mm']=51.2-(5.8+36)
    report['physical_print_fit_thermal_test']='NOT PERFORMED. All component and plug models are approximations.'
    report['snap_insertion_simulation']='NOT PERFORMED. Seated geometry checked only. Print both small test coupons first.'
    (o/'parameters.json').write_text(json.dumps(P,indent=2)+'\n');(o/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
