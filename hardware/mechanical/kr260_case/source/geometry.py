"""Shared board-envelope and drop-in support geometry; dimensions in mm.
Board/foot/connector models are approximate, as described in the guide.
"""
import cadquery as cq

P = dict(pcb_w=140.0, pcb_d=119.0, pcb_thickness=1.6,
         outer_w=164.0, outer_d=151.0, corner_radius=7.0,
         floor_t=2.8, foot_shelf_z=5.8, foot_height_assumed=5.5,
         foot_pad_w=20.0, foot_pad_d=18.0, foot_x=65.0, foot_y=54.5,
         edge_clearance=0.6, guide_t=2.4, guide_h=17.0,
         post_x=76.0, post_y=69.5, post_w=8.0,
         socket_top=9.8, socket_d=4.8, socket_depth=6.0,
         pin_d=4.3, pin_length=5.0,
         ring_bottom=50.0, ring_t=3.6, ring_open_w=144.0, ring_open_d=131.0,
         sd_notch_width=28.0, sd_notch_depth=60.0)


def box(w,d,h,x=0,y=0,z=0):
    return cq.Workplane('XY').box(w,d,h,centered=(True,True,False)).translate((x,y,z))


def rr(w,d,h,r,z=0):
    return box(w,d,h,z=z).edges('|Z').fillet(r)


def cyl(d,h,x=0,y=0,z=0):
    return cq.Workplane('XY').circle(d/2).extrude(h).translate((x,y,z))


def foot_pads(p):
    result=None
    for sx in [-1,1]:
        for sy in [-1,1]:
            s=rr(p['foot_pad_w'],p['foot_pad_d'],p['foot_shelf_z'],2).translate((sx*p['foot_x'],sy*p['foot_y'],0))
            result=s if result is None else result.union(s)
    return result


def guides(p):
    result=None
    for sx in [-1,1]:
        for sy in [-1,1]:
            # L-shaped outside guides: 0.6 mm nominal clearance to each PCB edge.
            xinner=p['pcb_w']/2+p['edge_clearance']
            yinner=p['pcb_d']/2+p['edge_clearance']
            z=p['foot_shelf_z']-0.3
            t=p['guide_t']; h=p['guide_h']-z
            a=box(t,11,h,sx*(xinner+t/2),sy*(yinner+t-5.5),z)
            b=box(9,t,h,sx*(xinner+t-4.5),sy*(yinner+t/2),z)
            s=a.union(b)
            result=s if result is None else result.union(s)
    return result


def make_gauge(p):
    g=rr(148,127,2,5).cut(rr(140,119,4,2,z=-1))
    g=g.union(foot_pads(p)).union(guides(p))
    # Side notch gives matching microSD finger access while preserving a U-frame.
    return g.cut(box(24,60,24,x=69,z=-1)).clean()


def reference_board(p):
    z=p['foot_shelf_z']; pcbz=z+p['foot_height_assumed']
    out=[]
    for sx in [-1,1]:
        for sy in [-1,1]:
            out.append(('foot',cyl(6.5,p['foot_height_assumed'],sx*65,sy*54.5,z)))
    out.append(('pcb',rr(140,119,p['pcb_thickness'],4,z=pcbz)))
    top=pcbz+p['pcb_thickness']
    # Approximate component envelopes for illustration only. No official STEP available.
    out += [('som',box(77,60,6.5,15,8,top)),
            ('sink',box(67,55,17,15,8,top+6.5)),
            ('fan',cyl(49,3,15,8,z+32.6)),
            ('fan_hub',cyl(17,3.4,15,8,z+32.6))]
    out += [('ethernet',box(32,27,26,-34,-49.5,top)),
            ('usb',box(14,20,16,-5,-51,top)),
            ('usb',box(14,20,16,14,-51,top)),
            ('dp',box(17,16,9,36,-52.5,top)),
            ('dc',box(9,15,11,54,-50,top)),
            ('sfp',box(15,49,10,-39,44,top)),
            ('micro_usb',box(8,6,3,-55,58,top)),
            ('hat',box(5,50,8,-65,-18,top)),
            ('slvs',box(3,14,5,-31,-3,top)),
            ('sd',box(16,16,2.2,62,3,pcbz-2.2))]
    for x in [-17,6,29,52]:out.append(('pmod',box(16,9,8,x,55,top)))
    return out


