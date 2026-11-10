# -*- coding: utf-8 -*-
"""AI370 + KR260：高细节、可编辑 Blender 生成脚本。

已在 Blender 4.5.15 LTS 实际运行并渲染；无需插件、外部模型或贴图。
在 Scripting → Text Editor → Open 中打开，点击 Run Script。
自动创建独立场景；桌面现有其它场景会保留。
Outliner 展开 AI370_Editable_Model / KR260_Editable_Model 可逐部件编辑。
接口为空腔和触点的真实几何；外壳通风与PCB安装孔均为真实开孔。
仅用于外观可视化：小器件、接口偏移与内部触点不是生产级CAD。

AI370 机型按用户既有工程中的 MINISFORUM EliteMini AI370 推定。
如实际主机型号不同，需要相应更改外壳及接口布局。
官方规格外形：AI370 130×127×46.9mm；KR260整机140×132×36mm。
所有网格最终转换为米，场景显示单位为毫米，各对象保持正常缩放。
"""

# ================= 可修改设置 =================
MODEL_TO_BUILD = 'BOTH'      # 'BOTH' / 'AI370' / 'KR260'
CREATE_STUDIO = True        # 创建灯光、地面、整体/前后/接口近景相机
RENDER_RESOLUTION = (2560, 1600)  # 高清输出；可改成 (3840, 2400)
RENDER_SAMPLES = 64
SAVE_BLEND_PATH = ''        # 留空不自动保存；例 '/your/folder/hardware.blend'
RENDER_PREVIEW_PATH = ''    # 留空不自动渲染；例 '/your/folder/preview.png'

HARDWARE_README = """AI370 / KR260 使用说明

1. 在 Blender 的 Scripting 工作区打开本 .py 文件，点击 Run Script。
2. 到 Layout 查看模型；Outliner 中每个接口、壳体、风扇等均可编辑。
3. AI370_ROOT / KR260_ROOT 可整体移动或旋转模型。
4. 相机：Overview、AI370_Front/Rear/PortDetail、KR260_Front/Rear/PortDetail。
   在 Scene Properties 中选择相机，按小键盘0进入相机视图，F12渲染。
5. File → Save As 保存为 .blend。脚本默认不自动写出文件。
6. 再次运行会替换此前脚本生成的部件，包括对这些部件做的编辑。
   原场景和另行添加且未标记的对象会保留。保留修改请先另存 .blend。

AI PC外壳机型：根据用户既有工程中的照片/DMI比对记录，先按
MINISFORUM EliteMini AI370（Ryzen AI 9 HX 370）制作。
用户本次确认的是迷你主机形态，尚未提供完整零售型号。

外观细节：AI370前后USB-A/USB-C/DP/HDMI/双LAN/音频/DC/按钮、侧斜槽。
KR260含4RJ45、4USB-A、DP、DC、MicroUSB、SFP+、4PMOD、40针、JTAG、
SLVS-EC、底部microSD、SOM、散热片、风扇叶片/格栅、主要芯片及丝印。
接口触点采用可辨识的几何近似；微小封装和PCB走线未按电气设计复刻。

精度范围：主要外形尺寸来自官方；孔位、器件位置、触点细节为图片推估。
该文件用于外观演示、布局与动画，不作为加工、接插件配合或布线依据。
KR260 PCB=140×119mm；132mm总深度含前后接口伸出，不是PCB板深。

主要来源：
AMD DS988 Rev1.1（Mechanical）
https://docs.amd.com/r/en-US/ds988-kr260-starter-kit/Mechanical
AMD UG1092（Interfaces）
https://docs.amd.com/r/en-US/ug1092-kr260-starter-kit/Interfaces
AMD KR260 官方产品图
https://www.amd.com/en/products/system-on-modules/kria/k26/kr260-robotics-starter-kit.html
MINISFORUM EliteMini AI370 官方图库
https://refurbished.minisforum.com/products/minisforum-elitemini-ai370-refurbished
MINISFORUM 官方规格图（由支持页链接）
https://pc-file.s3.us-west-1.amazonaws.com/ai370/spec/AI370.jpeg
"""


# ==================== hardware_geometry.py ====================

"""Editable hardware geometry for Blender 4.x.

All dimensions are millimetres. Connector local convention:
  front opening y=0, facing -Y; back extends +Y; footprint bottom z=0.
Methods return an Empty so complete components can be positioned/rotated.
No bpy.ops, external meshes, textures or add-ons are required.
"""
import bpy
import math
from mathutils import Vector


class Builder:
    def __init__(self, collection, parent=None, materials=None):
        self.collection = collection
        self.parent = parent
        self.materials = materials if materials is not None else {}
        self._palette()

    def _palette(self):
        definitions = {
            'metal': ((0.52, 0.57, 0.61, 1), 0.88, 0.28),
            'metal_dark': ((0.16, 0.20, 0.23, 1), 0.78, 0.33),
            'tin': ((0.68, 0.71, 0.73, 1), 0.92, 0.22),
            'gold': ((0.84, 0.49, 0.105, 1), 0.83, 0.24),
            'plastic': ((0.025, 0.030, 0.035, 1), 0.03, 0.40),
            'plastic_light': ((0.17, 0.19, 0.21, 1), 0.05, 0.42),
            'usb_blue': ((0.015, 0.17, 0.60, 1), 0.10, 0.34),
            'white': ((0.91, 0.94, 0.90, 1), 0.0, 0.48),
            'led_green': ((0.06, 0.68, 0.14, 1), 0.05, 0.23),
            'led_amber': ((1.0, 0.45, 0.025, 1), 0.05, 0.23),
        }
        for name, (color, metallic, roughness) in definitions.items():
            if name not in self.materials:
                self.material(name, color, metallic, roughness)

    def material(self, name, color=None, metallic=0.0, roughness=0.4, emission=0.0):
        if name in self.materials:
            return self.materials[name]
        mat = bpy.data.materials.new(name)
        mat.diffuse_color = color or (0.5, 0.5, 0.5, 1)
        mat.use_nodes = True
        shader = mat.node_tree.nodes.get('Principled BSDF')
        shader.inputs['Base Color'].default_value = mat.diffuse_color
        shader.inputs['Metallic'].default_value = metallic
        shader.inputs['Roughness'].default_value = roughness
        if emission:
            shader.inputs['Emission Color'].default_value = mat.diffuse_color
            shader.inputs['Emission Strength'].default_value = emission
        self.materials[name] = mat
        return mat

    def _mat(self, material):
        return self.materials[material] if isinstance(material, str) else material

    def _object(self, name, data):
        obj = bpy.data.objects.new(name, data)
        self.collection.objects.link(obj)
        if self.parent:
            obj.parent = self.parent
        return obj

    def empty(self, name, location=(0, 0, 0), rotation=(0, 0, 0)):
        obj = self._object(name, None)
        obj.empty_display_type = 'PLAIN_AXES'
        obj.empty_display_size = 3.0
        obj.location = location
        obj.rotation_euler = rotation
        return obj

    def child(self, parent):
        return Builder(self.collection, parent, self.materials)

    def mesh(self, name, vertices, faces, material=None, bevel=0.0):
        mesh = bpy.data.meshes.new(name + '_mesh')
        mesh.from_pydata(vertices, [], faces)
        mesh.update()
        obj = self._object(name, mesh)
        if material is not None:
            mesh.materials.append(self._mat(material))
        if bevel > 0:
            mod = obj.modifiers.new('Manufactured edge radius', 'BEVEL')
            mod.width = bevel
            mod.segments = 2
        return obj

    def box(self, name, center, size, material='plastic', bevel=0.0):
        sx, sy, sz = (v / 2 for v in size)
        vertices = [(-sx,-sy,-sz), (sx,-sy,-sz), (sx,sy,-sz), (-sx,sy,-sz),
                    (-sx,-sy,sz), (sx,-sy,sz), (sx,sy,sz), (-sx,sy,sz)]
        faces = [(0,3,2,1), (4,5,6,7), (0,1,5,4), (1,2,6,5),
                 (2,3,7,6), (3,0,4,7)]
        obj = self.mesh(name, vertices, faces, material, bevel)
        obj.location = center
        return obj

    def boxes(self, name, parts, material='metal', bevel=0.0):
        """A single editable mesh containing disjoint boxes (center, size)."""
        vertices, faces = [], []
        for center, size in parts:
            sx, sy, sz = [v / 2 for v in size]
            cx, cy, cz = center
            n = len(vertices)
            vertices += [(cx+x,cy+y,cz+z) for x,y,z in
                         [(-sx,-sy,-sz),(sx,-sy,-sz),(sx,sy,-sz),(-sx,sy,-sz),
                          (-sx,-sy,sz),(sx,-sy,sz),(sx,sy,sz),(-sx,sy,sz)]]
            faces += [tuple(n+i for i in f) for f in
                      [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),
                       (2,3,7,6),(3,0,4,7)]]
        return self.mesh(name, vertices, faces, material, bevel)

    def cylinder(self, name, center, radius, depth, material='metal', vertices=32,
                 rotation=(0, 0, 0), bevel=0.0):
        verts = [(radius*math.cos(2*math.pi*i/vertices),
                  radius*math.sin(2*math.pi*i/vertices), z)
                 for z in (-depth/2, depth/2) for i in range(vertices)]
        faces = [tuple(reversed(range(vertices))), tuple(range(vertices,2*vertices))]
        faces += [(i,(i+1)%vertices,(i+1)%vertices+vertices,i+vertices)
                  for i in range(vertices)]
        obj = self.mesh(name, verts, faces, material, bevel)
        obj.location = center
        obj.rotation_euler = rotation
        for polygon in obj.data.polygons[2:]:
            polygon.use_smooth = True
        return obj

    def tube(self, name, center, outer_radius, inner_radius, depth,
             material='metal', vertices=40, rotation=(0,0,0)):
        verts = [(r*math.cos(2*math.pi*i/vertices),r*math.sin(2*math.pi*i/vertices),z)
                 for z in (-depth/2, depth/2) for r in (outer_radius,inner_radius)
                 for i in range(vertices)]
        faces=[]
        n=vertices
        for i in range(n):
            j=(i+1)%n
            faces += [(i,j,2*n+j,2*n+i),
                      (n+j,n+i,3*n+i,3*n+j),
                      (j,i,n+i,n+j),
                      (2*n+i,2*n+j,3*n+j,3*n+i)]
        obj=self.mesh(name,verts,faces,material)
        obj.location=center
        obj.rotation_euler=rotation
        for polygon in obj.data.polygons:
            if polygon.index%4 < 2:
                polygon.use_smooth=True
        return obj

    def text(self, name, body, center, size=1.0, material='white',
             rotation=(0,0,0), align='LEFT', extrude=0.004):
        data=bpy.data.curves.new(name+'_text','FONT')
        data.body=body
        data.size=size
        data.align_x=align
        data.extrude=extrude
        data.resolution_u=4
        obj=self._object(name,data)
        obj.location=center
        obj.rotation_euler=rotation
        data.materials.append(self._mat(material))
        return obj

    def beam(self, name, start, end, width=0.3, depth=0.13, material='gold'):
        vector=Vector(end)-Vector(start)
        obj=self.box(name,(Vector(start)+Vector(end))/2,(width,depth,vector.length),material)
        obj.rotation_euler=vector.to_track_quat('Z','Y').to_euler()
        return obj

    def frame_xz(self, name, outer, inner, y0, y1, material='metal'):
        """Hollow extrusion of matching polygons in the X/Z plane."""
        n=len(outer)
        verts=[(x,y,z) for y in (y0,y1) for contour in (outer,inner) for x,z in contour]
        faces=[]
        for i in range(n):
            j=(i+1)%n
            faces += [(i,j,2*n+j,2*n+i),
                      (n+j,n+i,3*n+i,3*n+j),
                      (j,i,n+i,n+j),
                      (2*n+i,2*n+j,3*n+j,3*n+i)]
        return self.mesh(name,verts,[tuple(reversed(face)) for face in faces],material)

    def _port(self, name, x, y, z, rotation=None):
        root=self.empty(name,(x,y,z),rotation or (0,0,0))
        return root,self.child(root)

    def rj45(self, name, x, y, z, rotation=None):
        """16.0 W x 21.5 D x 13.6 H, shielded 8P8C socket."""
        root,b=self._port(name,x,y,z,rotation)
        b.box('RJ45_shield_left',(-7.8,10.75,6.8),(.4,21.5,13.6),'metal',.05)
        b.box('RJ45_shield_right',(7.8,10.75,6.8),(.4,21.5,13.6),'metal',.05)
        b.box('RJ45_shield_roof',(0,10.75,13.4),(15.6,21.5,.4),'metal',.04)
        b.box('RJ45_base',(0,10.75,.55),(15.6,21.5,1.1),'plastic',.12)
        b.box('RJ45_back',(0,20.8,6.65),(15.2,1.4,12.4),'plastic',.1)
        # Narrow bezel surrounds a real 13 mm deep cavity; latch notch at bottom.
        b.box('RJ45_bezel_top',(0,.45,12.2),(15.2,.9,2),'metal',.06)
        for side in (-1,1):
            b.box('RJ45_bezel_side',(side*7.05,.45,6.55),(1.1,.9,9.3),'metal',.06)
            b.box('RJ45_inner_guide',(side*6.1,8.1,6.2),(.8,14.8,9.0),'plastic',.07)
            b.box('RJ45_latch_shoulder',(side*4.65,2.3,1.45),(4.7,4.6,1.5),'plastic',.06)
            b.box('RJ45_LED_frame',(side*5.65,-.06,2.8),(2.5,.55,1.6),'plastic',.05)
            b.box('RJ45_LED_lens',(side*5.65,-.39,2.8),(1.85,.17,1.0),
                  'led_green' if side<0 else 'led_amber',.08)
            b.box('RJ45_shield_solder_tab',(side*8.1,15,-.8),(.6,2.3,2.5),'tin',.04)
        b.box('RJ45_inner_ceiling',(0,10.6,10.9),(11.5,15.8,.65),'plastic',.05)
        for i in range(8):
            px=(i-3.5)*1.02
            b.beam('RJ45_gold_contact_%02d_A'%(i+1),(px,15.8,10.2),(px,7.0,7.0),.36,.14)
            b.beam('RJ45_gold_contact_%02d_B'%(i+1),(px,7.0,7.0),(px,3.1,6.8),.36,.14)
            b.box('RJ45_tail_%02d'%(i+1),(px,19.8,-.7),(.36,.5,2.2),'tin',.03)
        for yy in (5,12,18):
            b.box('RJ45_roof_spring',(0,yy,13.66),(6.4,1.6,.14),'tin',.03)
        return root

    def usb_a_stack(self, name, x, y, z, levels=2, rotation=None):
        """14.6 W x 17 D; H = 7.0*levels + 0.5 mm. USB 3 Type A."""
        root,b=self._port(name,x,y,z,rotation)
        b.box('USB_stack_rear',(0,15.7,3.5*levels),(14.0,2.6,7.0*levels),'plastic',.1)
        for level in range(levels):
            zz=level*7.0
            b.box('USB_shell_left',(-7.05,7.9,zz+3.3),(.5,15.8,6.6),'metal',.05)
            b.box('USB_shell_right',(7.05,7.9,zz+3.3),(.5,15.8,6.6),'metal',.05)
            b.box('USB_shell_top',(0,7.9,zz+6.35),(13.7,15.8,.5),'metal',.05)
            b.box('USB_shell_bottom',(0,7.9,zz+.25),(13.7,15.8,.5),'metal',.05)
            b.box('USB_blue_tongue',(0,8.8,zz+2.65),(11.4,12.8,1.1),'usb_blue',.12)
            # Nine separately modelled SuperSpeed contact strips, two staggered rows.
            for i in range(4):
                b.box('USB_contact_4_%d'%(i+1),((i-1.5)*2.40,5.8,zz+3.235),(.65,5.0,.10),'gold',.025)
            for i in range(5):
                b.box('USB_contact_5_%d'%(i+1),((i-2)*1.82,10.9,zz+3.235),(.43,3.0,.10),'gold',.02)
            for side in (-1,1):
                b.box('USB_retention_spring',(side*4.2,3.3,zz+5.97),(1.7,3.2,.22),'tin',.04)
                b.box('USB_front_fold',(side*6.56,-.04,zz+3.2),(.25,.22,5.2),'tin',.025)
            b.box('USB_top_seam',(0,9.6,zz+6.64),(.25,12.1,.09),'metal_dark')
        for side in (-1,1):
            b.box('USB_solder_tab',(side*7.4,12,-1),(.7,2.4,2.9),'tin',.03)
        return root

    def displayport(self, name, x, y, z, rotation=None):
        """16.2 W x 15 D x 6.2 H; asymmetric, hollow full-size DP socket."""
        root,b=self._port(name,x,y,z,rotation)
        outer=[(-8.1,0),(8.1,0),(8.1,4.65),(6.55,6.2),(-8.1,6.2)]
        inner=[(-7.6,.48),(7.6,.48),(7.6,4.45),(6.35,5.7),(-7.6,5.7)]
        b.frame_xz('DP_asymmetric_hollow_shell',outer,inner,0,14.5,'metal')
        b.box('DP_rear',(0,14.3,3.0),(15.2,1.4,5.3),'plastic',.1)
        b.box('DP_contact_tongue',(-.25,6.3,2.75),(13.3,10.6,.8),'plastic',.08)
        for i in range(20):
            xx=-6.35+i*.64
            b.box('DP_gold_contact_%02d'%(i+1),(xx,3.7,3.19),(.29,4.8,.09),'gold',.015)
        for xx in (-4.8,4.8):
            b.box('DP_retention_tab',(xx,3.7,5.55),(1.6,4.5,.23),'tin',.03)
        return root

    def micro_usb(self, name, x, y, z, rotation=None):
        """8.0 W x 6.2 D x 3.1 H, USB Micro-B."""
        root,b=self._port(name,x,y,z,rotation)
        outer=[(-3.1,0),(3.1,0),(4,.9),(4,3.1),(-4,3.1),(-4,.9)]
        inner=[(-2.95,.28),(2.95,.28),(3.65,1.0),(3.65,2.78),(-3.65,2.78),(-3.65,1.0)]
        b.frame_xz('MicroUSB_hollow_shell',outer,inner,0,6.0,'metal')
        b.box('MicroUSB_back',(0,5.9,1.5),(7.1,.6,2.5),'plastic',.04)
        b.box('MicroUSB_tongue',(0,3.6,1.35),(5.35,3.3,.48),'plastic',.035)
        for i in range(5):
            b.box('MicroUSB_contact_%d'%(i+1),((i-2)*.65,3.0,1.62),(.29,2.3,.07),'gold',.01)
        for side in (-1,1):
            b.box('MicroUSB_mount',(side*4.12,3,-.2),(.6,1.7,.5),'tin',.02)
        return root

    def barrel_jack(self, name, x, y, z, rotation=None):
        """10 W x 14 D x 10 H, hollow 5.5/2.1 mm DC barrel receptacle."""
        root,b=self._port(name,x,y,z,rotation)
        b.boxes('DC_jack_housing_walls', [
            ((-4.15,8.8,5),(1.7,10.4,10)), ((4.15,8.8,5),(1.7,10.4,10)),
            ((0,8.8,.85),(6.6,10.4,1.7)), ((0,8.8,9.15),(6.6,10.4,1.7)),
            ((0,13.6,5),(6.6,.8,6.6))], 'plastic', .18)
        b.tube('DC_front_round_mouth',(0,2.0,5),4.65,2.8,4.0,'plastic',48,(math.pi/2,0,0))
        b.tube('DC_metal_entry_ring',(0,.12,5),3.25,2.73,.32,'tin',48,(math.pi/2,0,0))
        b.tube('DC_inner_barrel',(0,4.2,5),2.77,2.50,7.7,'metal_dark',40,(math.pi/2,0,0))
        b.cylinder('DC_center_pin',(0,5.1,5),1.05,6.4,'tin',32,(math.pi/2,0,0),.07)
        for xx,yy in ((0,11),(-3,8),(3,8)):
            b.box('DC_solder_tab',(xx,yy,-1),(.75,1.5,3),'tin',.05)
        return root

    def sfp(self, name, x, y, z, rotation=None):
        """14.5 W x 58 D x 12.0 H, empty SFP cage with geometric perforations."""
        root,b=self._port(name,x,y,z,rotation)
        # The socket is intentionally empty: it is a cage, not an optical module.
        b.box('SFP_left_wall',(-7.05,29,6),(.4,58,12),'metal',.04)
        b.box('SFP_right_wall',(7.05,29,6),(.4,58,12),'metal',.04)
        b.box('SFP_floor',(0,29,.2),(13.7,58,.4),'metal_dark',.04)
        b.box('SFP_back',(0,57.8,6),(13.7,.4,11.6),'metal',.03)
        # Roof is built from rails and cross strips; gaps are actual open holes.
        parts=[]
        for xx in (-6.65,-2.22,2.22,6.65):
            parts.append(((xx,29,11.8),(.55,58,.4)))
        for yy in (0.8,6.5,12.2,17.9,23.6,29.3,35.0,40.7,46.4,52.1,57.2):
            parts.append(((0,yy,11.8),(13.7,1.5,.4)))
        b.boxes('SFP_perforated_roof',parts,'metal',.025)
        b.box('SFP_front_top_lip',(0,-.25,11.8),(14.5,.5,.4),'tin',.03)
        b.box('SFP_front_floor_lip',(0,-.25,.25),(14.5,.5,.5),'tin',.03)
        for side in (-1,1):
            b.box('SFP_front_side_lip',(side*7.05,-.25,6),(.4,.5,11.2),'tin',.03)
            for yy in (2.6,5.5,8.4):
                tab=b.box('SFP_EMI_spring',(side*7.21,yy,6),(.18,1.7,9.4),'tin',.025)
                tab.rotation_euler[2]=side*math.radians(5)
            for yy in (17,38,51):
                b.box('SFP_mount_leg',(side*6.3,yy,-1),(.7,1.8,2.5),'tin',.02)
        b.box('SFP_rear_connector',(0,51.7,4.1),(11.9,6.3,5.1),'plastic',.15)
        b.box('SFP_card_edge_slot',(0,47.95,4.4),(10.6,1.0,.9),'plastic_light',.03)
        for i in range(20):
            b.box('SFP_rear_contact_%02d'%(i+1),((i-9.5)*.49,47.7,4.9),(.22,1.8,.14),'gold',.012)
        return root

    def pin_header(self, name, center, cols, rows=2, pitch=2.54, rotation=None):
        """Male header. center is the center of footprint at PCB top (base z=0)."""
        root,b=self._port(name,*center,rotation=rotation)
        parts=[]
        for row in range(rows):
            for col in range(cols):
                xx=(col-(cols-1)/2)*pitch
                yy=(row-(rows-1)/2)*pitch
                parts.append(((xx,yy,1.25),(pitch-.08,pitch-.08,2.5)))
                b.box('Header_pin_%02d_%d'%(col+1,row+1),(xx,yy,2.6),(.64,.64,10.0),'gold',.085)
        b.boxes('Header_insulators',parts,'plastic',.12)
        return root

    def screw(self, name, x, y, z, radius=2.0, shaft=3.0, rotation=None):
        """M2.5-like socket head. z is the mounting surface, shaft below it."""
        root,b=self._port(name,x,y,z,rotation)
        b.cylinder('Screw_thread_core',(0,0,-shaft/2),radius*.53,shaft,'metal_dark',24)
        for i in range(max(2,int(shaft/.55))):
            b.tube('Screw_thread_ridge',(0,0,-.24-i*.55),radius*.59,radius*.51,.12,'metal',24)
        # Circular outside and true six-sided recessed socket, with a deep floor.
        n=48
        out=[(radius*math.cos(2*math.pi*i/n),radius*math.sin(2*math.pi*i/n)) for i in range(n)]
        inner=[]
        for i in range(n):
            angle=2*math.pi*i/n
            local=(angle+math.pi/6)%(math.pi/3)-math.pi/6
            rr=radius*.42/math.cos(local)
            inner.append((rr*math.cos(angle),rr*math.sin(angle)))
        verts=[(xx,yy,zz) for zz in (.25,1.3) for loop in (out,inner) for xx,yy in loop]
        faces=[]
        for i in range(n):
            j=(i+1)%n
            faces += [(i,j,2*n+j,2*n+i),(n+j,n+i,3*n+i,3*n+j),
                      (2*n+i,2*n+j,3*n+j,3*n+i),(j,i,n+i,n+j)]
        b.mesh('Screw_socket_head',verts,faces,'metal',.035)
        b.cylinder('Screw_socket_floor',(0,0,.26),radius*.49,.10,'metal_dark',6)
        return root

    def usb_c(self, name, x, y, z, rotation=None):
        """9.2 W x 8.0 D x 3.4 H, reversible USB Type C socket."""
        root,b=self._port(name,x,y,z,rotation)
        def capsule(width,height,steps=12):
            radius=height/2
            shift=width/2-radius
            result=[]
            for side,start in ((1,-math.pi/2),(-1,math.pi/2)):
                for i in range(steps+1):
                    angle=start+math.pi*i/steps
                    result.append((side*shift+radius*math.cos(angle),1.7+radius*math.sin(angle)))
            return result
        b.frame_xz('USBC_capsule_hollow_shell',capsule(9.2,3.4),capsule(8.55,2.75),0,7.8,'metal')
        b.box('USBC_back',(0,7.55,1.7),(8.0,.9,2.65),'plastic',.10)
        b.box('USBC_contact_tongue',(0,4.1,1.7),(6.6,6.0,.61),'plastic',.12)
        for face in (-1,1):
            for i in range(12):
                b.box('USBC_%s_contact_%02d'%('upper' if face>0 else 'lower',i+1),
                      ((i-5.5)*.5,2.7,1.7+face*.34),(.24,2.5,.055),'gold',.008)
        for side in (-1,1):
            b.box('USBC_mounting_tab',(side*4.55,5.7,-.24),(.65,1.7,.55),'tin',.03)
        return root

    def hdmi(self, name, x, y, z, rotation=None):
        """15.0 W x 14.5 D x 5.2 H, hollow HDMI Type A socket."""
        root,b=self._port(name,x,y,z,rotation)
        outer=[(-5.95,0),(5.95,0),(7.5,1.75),(7.5,5.2),(-7.5,5.2),(-7.5,1.75)]
        inner=[(-5.75,.4),(5.75,.4),(7.07,1.9),(7.07,4.78),(-7.07,4.78),(-7.07,1.9)]
        b.frame_xz('HDMI_trapezoid_hollow_shell',outer,inner,0,14.0,'metal')
        b.box('HDMI_back',(0,13.75,2.6),(13.2,1.2,4.1),'plastic',.08)
        b.box('HDMI_contact_tongue',(0,6.3,2.7),(11.4,10.9,.63),'plastic',.10)
        for row,count in ((1,10),(-1,9)):
            for i in range(count):
                b.box('HDMI_%s_contact_%02d'%('upper' if row>0 else 'lower',i+1),
                      ((i-(count-1)/2)*1.05,3.45,2.7+row*.355),(.46,4.5,.08),'gold',.015)
        for side in (-1,1):
            b.box('HDMI_retention_finger',(side*6.85,4.7,3.3),(.18,3.3,1.5),'tin',.025)
            b.box('HDMI_mounting_foot',(side*7.65,10,-.25),(.7,2.0,.5),'tin',.025)
        return root

    def audio_jack(self, name, x, y, z, rotation=None):
        """7.0 W x 13.0 D x 7.0 H, hollow 3.5 mm TRRS audio socket."""
        root,b=self._port(name,x,y,z,rotation)
        b.boxes('Audio_jack_housing_walls', [
            ((-2.9,8.5,3.5),(1.2,9,7)), ((2.9,8.5,3.5),(1.2,9,7)),
            ((0,8.5,.6),(4.6,9,1.2)), ((0,8.5,6.4),(4.6,9,1.2)),
            ((0,12.6,3.5),(4.6,.8,4.6))], 'plastic', .15)
        b.tube('Audio_jack_front_sleeve',(0,1.8,3.5),3.15,1.83,3.6,'plastic',48,(math.pi/2,0,0))
        b.tube('Audio_jack_metal_rim',(0,.19,3.5),2.59,1.78,.42,'tin',48,(math.pi/2,0,0))
        b.tube('Audio_jack_inner_sleeve',(0,6.0,3.5),2.1,1.82,8.8,'plastic_light',48,(math.pi/2,0,0))
        for i in range(3):
            b.tube('Audio_contact_ring_%d'%(i+1),(0,2.7+i*2.4,3.5),1.93,1.77,.48,'tin',40,(math.pi/2,0,0))
        for side in (-1,1):
            for yy in (5.3,9.8):
                b.box('Audio_solder_pin',(side*2.4,yy,-.7),(.5,.8,2.1),'tin',.02)
        return root


# ==================== ai370_body.py ====================

"""MINISFORUM EliteMini AI370 exterior, dimensions in mm.
Overall 130 x 127 x 46.9 mm per official specification image.
Connector positions and small cosmetic details are photo-derived approximations.
"""
import math
import bpy


def rounded_profile(width, depth, radius, segments=12):
    points = []
    for cx, cy, angle in (
        (width / 2 - radius, depth / 2 - radius, 0),
        (-width / 2 + radius, depth / 2 - radius, 90),
        (-width / 2 + radius, -depth / 2 + radius, 180),
        (width / 2 - radius, -depth / 2 + radius, 270),
    ):
        for i in range(segments + 1):
            theta = math.radians(angle + 90 * i / segments)
            points.append((cx + radius * math.cos(theta), cy + radius * math.sin(theta)))
    return points


def rounded_plate(b, name, width, depth, radius, bottom, top, material):
    points = rounded_profile(width, depth, radius)
    n = len(points)
    vertices = [(x, y, z) for z in (bottom, top) for x, y in points]
    faces = [tuple(reversed(range(n))), tuple(range(n, 2 * n))]
    faces += [(i, (i + 1) % n, (i + 1) % n + n, i + n) for i in range(n)]
    return b.mesh(name, vertices, faces, material)


def rounded_shell(b, name, width, depth, radius, thickness, bottom, top, material):
    outer = rounded_profile(width, depth, radius)
    inner = rounded_profile(width - 2 * thickness, depth - 2 * thickness, radius - thickness)
    n = len(outer)
    vertices = [(x, y, z) for z in (bottom, top) for profile in (outer, inner) for x, y in profile]
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, 2*n+j, 2*n+i),
                  (n+j, n+i, 3*n+i, 3*n+j),
                  (j, i, n+i, n+j),
                  (2*n+i, 2*n+j, 3*n+j, 3*n+i)]
    return b.mesh(name, vertices, faces, material)


def bake_collection_difference(obj, cutters):
    """Cut every real port/vent opening in one evaluated Boolean operation."""
    modifier = obj.modifiers.new('Machined ports and ventilation slots', 'BOOLEAN')
    modifier.operation = 'DIFFERENCE'
    modifier.operand_type = 'COLLECTION'
    modifier.collection = cutters
    modifier.solver = 'EXACT'
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    evaluated = obj.evaluated_get(depsgraph)
    mesh = bpy.data.meshes.new_from_object(evaluated, depsgraph=depsgraph)
    old = obj.data
    obj.modifiers.clear()
    obj.data = mesh
    if old.users == 0:
        bpy.data.meshes.remove(old)
    for cutter in list(cutters.objects):
        data = cutter.data
        bpy.data.objects.remove(cutter, do_unlink=True)
        if data.users == 0:
            bpy.data.meshes.remove(data)
    bpy.data.collections.remove(cutters)


def build_ai370(b):
    silver = b.material('AI370 anodized silver', (0.57, 0.61, 0.65, 1), metallic=.78, roughness=.30)
    lidmat = b.material('AI370 satin lid', (.67, .70, .73, 1), metallic=.74, roughness=.34)
    logo = b.material('AI370 graphite print', (.065, .075, .082, 1), metallic=.25, roughness=.45)
    red = b.material('MINISFORUM red accent', (.66, .018, .025, 1), roughness=.4)
    seam = b.material('AI370 joint', (.026, .032, .039, 1), metallic=.15, roughness=.45)
    root = b.parent
    root['device'] = 'MINISFORUM EliteMini AI370 / AMD Ryzen AI 9 HX 370'
    root['reference_size_mm'] = [130.0, 127.0, 46.9]
    root['accuracy'] = 'Overall dimensions official; small parts and port offsets estimated from official photos.'
    chassis = rounded_shell(b, 'AI370_Chassis_REAL_OPENINGS', 130, 127, 8.5, 1.4, 2.4, 44.6, silver)
    rounded_plate(b, 'AI370_LowerBase', 129.4, 126.4, 8.3, .9, 2.4, seam)
    rounded_plate(b, 'AI370_LidSeam', 129.6, 126.6, 8.35, 44.6, 45.0, seam)
    lid = rounded_plate(b, 'AI370_TopLid', 130, 127, 8.5, 45.0, 46.9, lidmat)
    edge = lid.modifiers.new('Fine lid edge', 'BEVEL')
    edge.width = .28
    edge.segments = 3

    # Cutters are temporary. The delivered chassis has actual holes in its mesh.
    cutters = bpy.data.collections.new('AI370_temporary_cutters')
    b.collection.children.link(cutters)
    cb = Builder(cutters, root, b.materials)
    front_y = -63.5
    for name, x, z, width, height in (
        ('USB4', -12.5, 25.4, 9.1, 3.7),
        ('USBA1', 11.7, 25.4, 14.8, 7.2),
        ('USBA2', 35.0, 25.4, 14.8, 7.2),
    ):
        cb.box('cut_front_' + name, (x, front_y, z), (width, 5, height), bevel=.3)
    for name, x, z, radius in (
        ('Power', -44.7, 25.5, 4.55), ('Audio', -31.4, 25.4, 2.7),
        ('CMOS', -54.2, 25.5, .65),
    ):
        cb.cylinder('cut_' + name, (x, front_y, z), radius, 5, vertices=48,
                    rotation=(math.pi/2, 0, 0))

    rear = b.empty('AI370_RearPanel_coordinates', (0, 63.5, 0), (0, 0, math.pi))
    rb = b.child(rear)
    rcb = Builder(cutters, rear, b.materials)
    for name, x, z, width, height in (
        ('USB_stack', -49, 30, 14.8, 14.3),
        ('DP', -27, 35.4, 16.6, 6.5),
        ('HDMI', -27, 24.8, 14.8, 5.6),
        ('LAN1', -3, 29.6, 16.4, 14.0),
        ('LAN2', 21, 29.6, 16.4, 14.0),
        ('Security', -56.5, 12.4, 2.7, 7.0),
    ):
        rcb.box('cut_rear_' + name, (x, 0, z), (width, 5, height), bevel=.28)
    rcb.cylinder('cut_rear_DC', (47.6, 0, 27.4), 3.45, 5,
                 vertices=48, rotation=(math.pi/2, 0, 0))
    for index in range(6):
        rcb.box('cut_rear_exhaust_%02d' % index, (-43.7 + index*17.5, 0, 11.8),
                 (15.2, 5, 8.2), bevel=.6)
    # Both sidewalls: real inclined ventilation cuts rather than dark decals.
    for side in (-1, 1):
        for index in range(30):
            cut = cb.box('cut_side_%s_%02d' % (side, index),
                         (side*64.6, -43.5 + index*3.0, 25), (5, .82, 20.5), bevel=.34)
            cut.rotation_euler.x = math.radians(-28 if side == 1 else 28)
    bake_collection_difference(chassis, cutters)
    edge = chassis.modifiers.new('Soft machined rims', 'BEVEL')
    edge.width = .10
    edge.segments = 2
    edge.limit_method = 'ANGLE'

    # Recessed connector geometry, with internal tongues and metal contacts.
    b.usb_c('AI370_Front_USB4', -12.5, -63.65, 23.55)
    b.usb_a_stack('AI370_Front_USB_A_1', 11.7, -63.65, 21.9, levels=1)
    b.usb_a_stack('AI370_Front_USB_A_2', 35.0, -63.65, 21.9, levels=1)
    b.audio_jack('AI370_Front_Audio', -31.4, -63.65, 21.9)
    b.tube('Power_button_annulus', (-44.7, -63.72, 25.5), 4.45, 4.12, .22,
           'metal_dark', vertices=64, rotation=(math.pi/2, 0, 0))
    b.cylinder('Power_button', (-44.7, -63.65, 25.5), 4.0, .5, silver,
               vertices=64, rotation=(math.pi/2, 0, 0), bevel=.12)
    b.cylinder('Power_LED', (-44.7, -63.94, 25.5), .65, .06, 'white',
               vertices=32, rotation=(math.pi/2, 0, 0))
    b.cylinder('Clear_CMOS_recess', (-54.2, -62.9, 25.5), .58, .8,
               'plastic', vertices=24, rotation=(math.pi/2, 0, 0))
    for body, x, z in [('CLR CMOS', -54.2, 20.5), ('AUDIO', -31.4, 20.5),
                       ('USB4', -12.5, 20.5), ('SS 10', 11.7, 19.2), ('SS 10', 35, 19.2)]:
        b.text('Front_label_' + body, body, (x, -63.56, z), 1.18, logo,
               rotation=(math.pi/2, 0, 0), align='CENTER', extrude=.006)
    rb.usb_a_stack('AI370_Rear_USB_A_pair', -49, -.15, 23, levels=2)
    rb.displayport('AI370_Rear_DisplayPort', -27, -.15, 32.3)
    rb.hdmi('AI370_Rear_HDMI', -27, -.15, 22.1)
    rb.rj45('AI370_Rear_2p5G_LAN_1', -3, -.15, 22.8)
    rb.rj45('AI370_Rear_2p5G_LAN_2', 21, -.15, 22.8)
    # The AI370 rear DC hole is round; keep the hidden square barrel body inside.
    rb.barrel_jack('AI370_Rear_DC19V', 47.6, .25, 22.4)
    rb.box('Security_slot_dark_interior', (-56.5, .9, 12.4), (2.8, .5, 7.1), 'plastic')
    for index in range(6):
        cx = -43.7 + index*17.5
        for j in range(16):
            rb.box('Rear_exhaust_fin_%s_%s' % (index, j),
                   (cx-6.7+j*.9, 2.8, 11.8), (.23, 3.2, 7.6), 'metal_dark')
    for body, x, z in [('DP', -27, 30.5), ('HDMI', -27, 19.4),
                       ('2.5G LAN', -3, 20.3), ('2.5G LAN', 21, 20.3), ('DC IN', 47.6, 20.3)]:
        rb.text('Rear_label_' + body, body, (x, -.06, z), 1.15, logo,
                rotation=(math.pi/2, 0, 0), align='CENTER', extrude=.005)
    # Shallow internal dark plenum, visible only through the side ventilation.
    b.box('AI370_Internal_dark_plenum', (0, 0, 20), (119, 108, 29), 'plastic', bevel=3)
    for x in (-51, 51):
        for y in (-50, 50):
            b.cylinder('Rubber_foot', (x, y, .5), 5.4, 1.0, 'plastic', vertices=48, bevel=.25)
    b.text('MINISFORUM_lid_logo', 'MINISFORUM', (0, -1.4, 46.93), 5.1, logo,
           align='CENTER', extrude=.009)
    b.box('Logo_red_accent', (1.2, 3.2, 46.95), (2.7, .65, .025), red)
    root['front_ports'] = 'Clear CMOS, power, 3.5mm audio, USB4 Type-C, 2x USB-A'
    root['rear_ports'] = '2x USB-A, DisplayPort, HDMI, 2x 2.5GbE RJ45, DC19V'
    return root


# ==================== kr260_body.py ====================

"""KR260 visual reconstruction for Blender, all dimensions in mm.

Documented: PCB 140 x 119 mm, assembled envelope 140 x 132 x 36 mm,
SOM/thermal footprint 77 x 60 mm. Interface inventory from AMD UG1092.
Individual package geometry and placements are photo-derived visual estimates,
not a PCB netlist, mechanical certification or production connector CAD.
"""
import bpy
import math
import random


def _cut_holes(b, board, holes, name):
    """Keep board as an editable mesh with actual through holes."""
    # Apply any manufacturing edge modifier before the hole Booleans.
    for existing in list(board.modifiers):
        bpy.context.view_layer.objects.active = board
        with bpy.context.temp_override(object=board, active_object=board):
            bpy.ops.object.modifier_apply(modifier=existing.name)
    for i, (x, y, r) in enumerate(holes):
        cutter = b.cylinder(name + '_cutter_%02d' % i, (x, y, board.location.z),
                            r, 20, 'metal', vertices=40)
        mod = board.modifiers.new('Mounting hole %02d' % i, 'BOOLEAN')
        mod.operation = 'DIFFERENCE'
        mod.solver = 'EXACT'
        mod.object = cutter
        bpy.context.view_layer.objects.active = board
        with bpy.context.temp_override(object=board, active_object=board):
            bpy.ops.object.modifier_apply(modifier=mod.name)
        cutter_mesh = cutter.data
        bpy.data.objects.remove(cutter, do_unlink=True)
        if cutter_mesh.users == 0:
            bpy.data.meshes.remove(cutter_mesh)


def _smd_batches(b):
    """Sparse, plausible component placement; no inferred electrical routes."""
    rng = random.Random(260)
    ceramic, resistor, solder, copper = [], [], [], []
    regions = [(-57,-37,-30,17,36), (-12,62,-40,-29,46),
               (-19,58,43,49,22), (55,66,-28,43,22),
               (-56,-25,-34,-18,22)]
    z=1.6
    for xa,xb,ya,yb,count in regions:
        for i in range(count):
            x,y=rng.uniform(xa,xb),rng.uniform(ya,yb)
            # Keep designated connectors, controls and mounting holes clear.
            if x < -46 and y > 15: continue
            if x > 59 and (-27<y<-6 or 12<y<21): continue
            if x < -57: continue
            w,d=(1.0,.48) if i%3 else (1.8,.9)
            matlist=resistor if i%3 else ceramic
            matlist.append(((x,y,z+.25),(w,d,.45)))
            for sx in (-1,1):
                solder.append(((x+sx*(w/2+.16),y,z+.13),(.36,d+.12,.22)))
    b.boxes('KR260_Carrier_Ceramic_Capacitors',ceramic,'ceramic',.05)
    b.boxes('KR260_Carrier_SMD_Resistors',resistor,'chip_black',.04)
    b.boxes('KR260_Carrier_Solder_Lands',solder,'tin',.025)
    # A few short broad decorative mask traces only, not a PCB routing replica.
    for x,y,length in [(-52,-26,9),(-48,-23,13),(-47,8,11),(-17,-35,13),
                        (10,-34,16),(31,-32,13),(57,45,7),(-9,46,10)]:
        copper.append(((x,y,1.615),(length,.17,.018)))
    b.boxes('Illustrative_Mask_Trace_Accents_Not_Electrical',copper,'trace_green')


def _ic(b,name,x,y,w,d,txt,rotation=0):
    root=b.empty(name,(x,y,1.6),(0,0,rotation)); c=b.child(root)
    c.box(name+'_Package',(0,0,.75),(w,d,1.5),'chip_black',.18)
    c.cylinder(name+'_Pin1',(-w*.34,-d*.34,1.515),.25,.025,'plastic_light',16)
    c.text(name+'_Marking',txt,(-w*.35,-d*.10,1.53),min(w/5,1.2),'silk_dim',extrude=.002)
    pins=[]
    for sign in (-1,1):
        for i in range(max(4,int(d/.7))):
            pins.append(((sign*(w/2+.25),(i-(int(d/.7)-1)/2)*.7,.35),(.5,.23,.28)))
    c.boxes(name+'_Leadframe',pins,'tin',.02)
    return root


def _female_pmod(b,name,cx):
    root=b.empty(name,(cx,0,0));c=b.child(root)
    # Both rows of six sockets face rear +Y and are genuinely hollow.
    for row in range(2):
        zz=3.15+row*2.54
        for col in range(6):
            xx=(col-2.5)*2.54
            outer=[(xx-1.27,zz-1.27),(xx+1.27,zz-1.27),
                   (xx+1.27,zz+1.27),(xx-1.27,zz+1.27)]
            inner=[(xx-.55,zz-.55),(xx+.55,zz-.55),
                   (xx+.55,zz+.55),(xx-.55,zz+.55)]
            c.frame_xz(name+'_Socket_%d_%d'%(row+1,col+1),outer,inner,52,59.8,'plastic')
            c.box(name+'_Socket_Back',(xx,52.2,zz),(1.0,.3,1.0),'plastic_light')
            c.box(name+'_Gold_Spring',(xx,55.0,zz-.41),(.52,4.8,.16),'gold',.02)
            # Right-angle tails disappear through the PCB into solder pads.
            tail_y=49.7-row*1.0
            c.beam(name+'_Gold_Elbow',(xx,52.1,zz),(xx,tail_y,zz),.54,.54)
            c.beam(name+'_Gold_Tail',(xx,tail_y,zz),(xx,tail_y,-.3),.54,.54)
            c.cylinder(name+'_Tail_Solder',(xx,tail_y,1.65),.6,.12,'tin',16)
    return root


def _cooler(b,cx,cy):
    root=b.empty('K26_Active_Fansink',(cx,cy,0)); c=b.child(root)
    c.box('K26_Heatspreader',(0,0,7.25),(66,56,2.1),'metal_dark',1.0)
    # Footprint-correct cooler frame with open fan aperture.
    c.box('Cooler_Frame_Left',(-32,0,19.5),(3,53,20.2),'heatsink',1.1)
    c.box('Cooler_Frame_Right',(32,0,19.5),(3,53,20.2),'heatsink',1.1)
    for sy in (-1,1):
        c.box('Cooler_Frame_End',(0,sy*27,19.5),(64,3.6,20.2),'heatsink',1.0)
    # Parallel fins with a circular opening, rather than a disk hiding the fan.
    finparts=[]
    for j in range(17):
        yy=(j-8)*3.0
        gap=math.sqrt(max(0,23.6**2-yy**2)) if abs(yy)<23.6 else 0.0
        if gap<.2:
            finparts.append(((0,yy,20.0),(61,1.35,20.0)))
        else:
            for sign in (-1,1):
                width=30.5-gap
                finparts.append(((sign*(gap+width/2),yy,20.0),(width,1.35,20.0)))
    c.boxes('Heatsink_Parallel_Fins',finparts,'heatsink',.35)
    c.tube('Fan_Shroud_Open_Ring',(0,0,28.9),24.4,22.7,3.5,'heatsink',96)
    c.tube('Fan_Grille_Outer_Rim',(0,0,30.75),23.5,22.2,.5,'fan_plastic',96)
    c.cylinder('Fan_Motor_Housing',(0,0,24),8.7,8.3,'fan_plastic',64,bevel=.4)
    # Ten swept blades, with camber and thickness, individually editable.
    for i in range(10):
        a=i*2*math.pi/10
        points=[]
        # Curved annular quadrilateral approximates moulded fan blades.
        for rr,aa in [(8.3,a-.19),(13,a-.05),(19,a+.12),(22.1,a+.25),
                      (21.2,a+.47),(16,a+.38),(11,a+.21),(8.3,a+.13)]:
            points.append((rr*math.cos(aa),rr*math.sin(aa)))
        verts=[(x,y,z+.9*(math.hypot(x,y)-8.3)/14) for z in (25.5,26.0) for x,y in points]
        n=len(points);faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]
        faces += [(k,(k+1)%n,(k+1)%n+n,k+n) for k in range(n)]
        blade=c.mesh('Fan_Swept_Blade_%02d'%(i+1),verts,faces,'fan_plastic',.1)
        for p in blade.data.polygons:p.use_smooth=True
    # Fine grille rings and four supports keep the opening visibly open.
    for r in (10.7,15.3,19.5):
        c.tube('Fan_Protective_Grille_Ring',(0,0,30.9),r+.35,r-.35,.45,'fan_plastic',96)
    for i in range(4):
        a=math.pi/4+i*math.pi/2
        c.beam('Fan_Grille_Support',(7.7*math.cos(a),7.7*math.sin(a),30.9),
               (22.6*math.cos(a+.25),22.6*math.sin(a+.25),30.9),1.0,.55,'fan_plastic')
    c.cylinder('Fan_Center_Badge',(0,0,31.0),8.7,.5,'fan_plastic',64,bevel=.1)
    # AMD-style open arrow symbol, geometric and editable.
    arrow=[(-3.6,3.5),(3.6,3.5),(3.6,-3.7),(1.1,-1.2),(1.1,1.0),(-1.2,1.0)]
    c.mesh('AMD_Fan_Arrow',[(x,y,31.29) for x,y in arrow],[tuple(range(len(arrow)))],'white')
    arrow2=[(-3.6,-3.5),(-3.6,1.0),(-1.1,-1.4),(1.1,-1.4),(-1.2,-3.5)]
    c.mesh('AMD_Fan_Arrow_Lower',[(x,y,31.29) for x,y in arrow2],[tuple(range(len(arrow2)))],'white')
    for x in (-31.7,31.7):
        for y in (-14.5,14.5):
            c.screw('Cooler_Fan_Mount_Screw',x,y,29.1,radius=1.75,shaft=5)
    # Four-wire fan lead routed to the genuine carrier fan-power area.
    for i,mat in enumerate(('wire_black','wire_red','wire_yellow','wire_blue')):
        c.beam('Fan_Wire',(30,-19-i*.5,11),(40,-26-i*.5,6),.34,.34,mat)
        c.beam('Fan_Wire',(40,-26-i*.5,6),(47.8,-34-i*.3,4),.34,.34,mat)
    return root


def build_kr260(b):
    """Build inside b.parent; return that complete-model root."""
    colors={
      'pcb_green':((.035,.23,.145,1),.05,.47),
      'som_green':((.036,.25,.115,1),.03,.44),
      'trace_green':((.05,.31,.205,1),.12,.45),
      'heatsink':((.032,.038,.042,1),.65,.32),
      'fan_plastic':((.018,.022,.025,1),.07,.39),
      'chip_black':((.025,.032,.036,1),.05,.43),
      'ceramic':((.36,.25,.13,1),.08,.5),
      'silk_dim':((.60,.76,.66,1),0,.6),
      'wire_black':((.012,.014,.018,1),0,.4),
      'wire_red':((.35,.016,.01,1),0,.4),
      'wire_yellow':((.8,.48,.01,1),0,.4),
      'wire_blue':((.01,.07,.38,1),0,.4)}
    for k,(col,metal,rough) in colors.items():b.material(k,col,metal,rough)
    if b.parent is None:
        b=b.child(b.empty('AMD_Kria_KR260_Robotics_Starter_Kit'))
    root=b.parent
    root['source']='AMD DS988 v1.1 and UG1092 v1.1, official product photographs'
    root['geometry_note']='Visual reconstruction; small placements/holes/connectors photo-estimated. Decorative traces do not represent electrical nets.'
    root['official_pcb_mm']='140 x 119'
    root['official_assembly_mm']='140 x 132 x 36'
    perimeter=[]
    for cx,cy,start in [(65.2,54.7,0),(-65.2,54.7,90),(-65.2,-54.7,180),(65.2,-54.7,270)]:
        for i in range(9):
            a=math.radians(start+i*90/8);perimeter.append((cx+4.8*math.cos(a),cy+4.8*math.sin(a)))
    n=len(perimeter);verts=[(x,y,z) for z in (0,1.6) for x,y in perimeter]
    faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
    pcb=b.mesh('KR260_Carrier_PCB_140x119mm',verts,faces,'pcb_green')
    mount=[(-65,54.5),(65,54.5),(-65,-54.5),(65,-54.5)]
    inner=[(-20.5,35.5),(17.5,35.5),(50.5,35.5),(-20.5,-18.5),(17.5,-18.5),(50.5,-18.5)]
    _cut_holes(b,pcb,[(x,y,2.5) for x,y in mount]+[(x,y,1.6) for x,y in inner],'Carrier')
    for x,y in mount:
        b.tube('Mounting_Hole_Plated_Annulus',(x,y,1.63),4.15,2.5,.06,'gold',48)
        b.tube('Underside_Mount_Annulus',(x,y,-.03),4.15,2.5,.06,'tin',48)
        # Feet support the starter kit without filling its visible mounting hole.
        b.tube('Nylon_Support_Foot',(x,y,-2.35),3.1,2.5,4.7,'plastic_light',48)
    for x,y in inner:
        b.tube('SOM_Mount_Plated_Annulus',(x,y,1.64),2.8,1.6,.08,'gold',32)
    # True 2x2 front RJ45 bank. Each individual socket remains editable.
    for col,x in enumerate((-42,-26)):
        for row in range(2):
            designator=(('J10A','J10B'),('J10C','J10D'))[col][row]
            desc=(('ETH3_PL','ETH2_PL'),('ETH1_PS','ETH0_PS'))[col][row]
            b.rj45(designator+'_'+desc,x,-63,1.6+13.6*row)
    b.usb_a_stack('U46_USB1_Dual_USB3_TypeA',-5,-61,1.6,levels=2)
    b.usb_a_stack('U44_USB0_Dual_USB3_TypeA',14.5,-61,1.6,levels=2)
    b.displayport('J6_DisplayPort_1_2a',36,-60,1.6)
    b.barrel_jack('J12_12V_DC_Input',54,-59.5,1.6)
    b.micro_usb('J4_MicroUSB_UART_JTAG',-55.5,60.5,1.6,rotation=(0,0,math.pi))
    sfp=b.sfp('J23_J24_SFPplus_Cage',-39,68.05,1.6,rotation=(0,0,math.pi))
    sfp.scale.y=.84;sfp.scale.z=.85
    for x,name in zip((-16.8,6,28.8,51.6),('J20','J19','J18','J2')):
        _female_pmod(b,name+'_Pmod_2x6',x)
        b.text(name+'_Label',name,(x+9.2,48.1,1.66),1.1,'white')
        b.text(name+'_Pin1','1',(x+7.7,50.7,1.66),.8,'white')
    # Raspberry Pi 40-way header; long dimension along Y.
    b.pin_header('J21_RaspberryPi_HAT_40pin',(-64,-16,1.6),20,2,2.54,rotation=(0,0,math.pi/2))
    b.text('J21_Header_Label','RPI GPIO',(-58.9,-7.0,1.66),1.2,'white',rotation=(0,0,math.pi/2))
    # Shrouded 14-pin PC4 JTAG with visible gold pins and open mouth upward.
    jtag=b.empty('J3_PC4_JTAG',(-59,25.5,1.6)); j=b.child(jtag)
    j.box('JTAG_Base',(0,0,.7),(17,7,1.4),'plastic',.3)
    for x in (-8.2,8.2):j.box('JTAG_End_Wall',(x,0,3.3),(.7,7,5.2),'plastic',.15)
    for y in (-3.2,3.2):j.box('JTAG_Side_Wall',(0,y,3.3),(16,.6,5.2),'plastic',.12)
    jtagpins=j.pin_header('JTAG_2x7_Pins',(0,0,.3),7,2,2.0)
    jtagpins.scale.z=.60
    b.text('J3_Label','JTAG',(-66,31,1.66),1.3,'white')
    # Narrow SLVS-EC receptacle, real top slot and fine contacts.
    sl=b.empty('J22_SLVS_EC_FinePitch',(-30,0,1.6));s=b.child(sl)
    s.box('SLVS_Connector_Base',(0,0,.5),(3.7,16,1.0),'plastic',.1)
    for x in (-1.6,1.6):s.box('SLVS_Long_Rail',(x,0,1.7),(.5,16,2.3),'plastic',.06)
    for y in (-7.8,7.8):s.box('SLVS_End',(0,y,1.7),(3.0,.5,2.3),'plastic',.06)
    for side in (-1,1):
        for i in range(28):s.box('SLVS_Gold_Contact',(side*1.1,(i-13.5)*.5,1.4),(.25,.2,1.4),'gold',.01)
    b.text('J22_Label','SLVS-EC',(-36,-11,1.66),1.15,'white')
    # Two fine-pitch SOM board-to-board connectors, visible beneath the raised module.
    for x in (-18.5,48.5):
        b.box('SOM_240pin_BoardToBoard_Base',(x,8.5,3.05),(3.8,42,2.9),'plastic',.2)
        pins=[]
        for side in (-1,1):
            for i in range(60):pins.append(((x+side*1.6,8.5+(i-29.5)*.635,3.2),(.3,.23,2.2)))
        b.boxes('SOM_240pin_GoldContacts',pins,'gold',.015)
    for x,y in ((-20.5,35.5),(50.5,35.5),(-20.5,-18.5),(50.5,-18.5)):
        b.cylinder('SOM_Brass_Standoff',(x,y,3.0),2.3,2.8,'gold',6,bevel=.08)
    som=b.box('K26_SOM_PCB_77x60',(14.5,8.5,4.8),(77,60,1.6),'som_green',.6)
    _cut_holes(b,som,[(x,y,1.4) for x,y in ((-20.5,35.5),(50.5,35.5),(-20.5,-18.5),(50.5,-18.5))],'SOM')
    for x,y in ((-20.5,35.5),(50.5,35.5),(-20.5,-18.5),(50.5,-18.5)):
        b.tube('SOM_Corner_Gold_Pad',(x,y,5.63),3,1.4,.06,'gold',40)
        b.screw('SOM_Mount_Screw',x,y,5.66,radius=1.85,shaft=4)
    _cooler(b,14.5,8.5)
    # SOM status LEDs sit along the exposed front lip.
    for i,label in enumerate(('DS34 PL DONE','DS35 HEARTBEAT','DS36 PS STATUS')):
        x=-11+i*3.1
        b.box(label,(x,-20.2,5.9),(1.3,.8,.45),'led_green',.09)
    # White two-way fan-power header with pins and real open top.
    f=b.empty('J13_Fan_Power',(63,-27.5,1.6));q=b.child(f)
    q.box('Fan_Header_Base',(0,0,.5),(5.5,6,1),'white',.1)
    for x in (-2.5,2.5):q.box('Fan_Header_Wall',(x,0,2.6),(.5,6,4.2),'white',.08)
    q.box('Fan_Header_Back',(0,2.7,2.6),(5,.6,4.2),'white',.08)
    for x in (-1,1):q.box('Fan_Header_GoldPin',(x,0,2),(.6,.6,3),'gold',.04)
    b.text('J13_Label','FAN HDR',(56,-35,1.66),1.05,'white')
    b.pin_header('J1_Watchdog_Selector',(64,-18.5,1.6),3,1,2.54)
    b.text('J1_Label','WATCHDOG',(56,-22.0,1.66),.85,'white')
    for name,y,label in [('SW1',-9.7,'FW UPDATE'),('SW2',15.8,'RESET')]:
        b.box(name+'_Tactile_Metal',(62,y,2.05),(3.2,4.2,.9),'tin',.2)
        b.cylinder(name+'_Black_Push',(62,y,2.8),1.05,.9,'plastic',32,bevel=.1)
        b.text(name+'_Label',label,(54.8,y+3,1.66),.9,'white')
    for i in range(8):
        y=26.6+i*2.45
        b.box('DS%d_Status_LED'%(i+1),(64.4,y,1.95),(1.45,.8,.5),'led_green' if i<6 else 'led_amber',.1)
        b.text('DS%d_Label'%(i+1),'DS%d'%(i+1),(60.3,y-.25,1.66),.65,'white')
    # Under-side microSD slot on the RPi-side edge (orientation from UG1092 bottom view).
    sd=b.empty('J11_Underside_microSD',(-60,4,-.1));t=b.child(sd)
    t.box('microSD_Under_Shield',(0,0,-1.2),(12,14,.35),'metal',.25)
    for x in (-5.8,5.8):t.box('microSD_Edge',(x,0,-.7),(.4,14,1.5),'metal',.08)
    t.box('microSD_Insulator',(0,2,-.3),(10.5,10,.5),'plastic',.1)
    for i in range(8):t.box('microSD_Contact',((i-3.5)*1.1,-2,-.65),(.5,5,.12),'gold',.01)
    # Main visible PHYs/USB hubs, clock chips and regulator packages.
    for args in [('U35_EthernetPHY',-52,-19,7,7,'GBE PHY'),
                 ('U80_EthernetPHY',-35,-19,7,7,'GBE PHY'),
                 ('U61_USB_Hub',34,-39,6.2,6.2,'USB3'),
                 ('U4_PowerController',57,-38,5.4,4.5,'PWR'),
                 ('U49_FTDI',-54,43,5.5,5.5,'FTDI'),
                 ('U42_IO',62,48,3.5,3.5,'IO')]:_ic(b,*args)
    for x in (-38,-22):
        b.box('Ethernet_Clock_Crystal',(x,-31,2.05),(5.2,3.2,.9),'metal',.3)
    for x in (35,45):
        b.box('Power_Inductor',(x,-31,3),(4.8,4.8,2.8),'plastic_light',.45)
        b.text('Inductor_Value','2R2',(x-1.5,-31.8,4.43),.9,'silk_dim',extrude=.002)
    for x,y in [(29,-35),(46,-38),(-50,-30),(60,-40)]:
        b.cylinder('Electrolytic_Capacitor',(x,y,3.1),1.45,3,'metal_dark',24,bevel=.12)
        b.cylinder('Electrolytic_Top',(x,y,4.63),1.32,.08,'tin',24)
    _smd_batches(b)
    # Readable genuine board identifiers, deliberately sparse routing decoration.
    for name,txt,loc,size in [
       ('Board_Product','KRIA KR260',(18,44,1.67),2.15),
       ('Board_Desc','ROBOTICS STARTER KIT',(14,41.3,1.67),1.1),
       ('Board_AMD','AMD',(-59,40.3,1.67),1.6),
       ('ETH_PL_Label','PL ETH',(-49,-36,1.67),1.25),
       ('ETH_PS_Label','PS ETH',(-32,-36,1.67),1.25),
       ('USB1_Label','USB1',(-9,-40,1.67),1.0),
       ('USB0_Label','USB0',(10,-40,1.67),1.0),
       ('DP_Label','DP',(32,-42,1.67),1.15),
       ('Power_Label','12V PWR',(48,-41,1.67),1.05),
       ('MicroUSB_Label','UART / JTAG',(-64,49,1.67),.92),
       ('SFP_Label','SFP+',(-46,15,1.67),1.2)]:
        b.text(name,txt,loc,size,'white')
    for i,(x,y) in enumerate(mount):
        b.text('Mount_MH%d'%(i+1),'MH%d'%(i+1),(x-2.5,y-6,1.67),.9,'white')
    return root


# ==================== hardware_studio.py ====================

"""Metre-scale studio for the editable AI370 / KR260 hardware scene.
Only cameras, lights, a ground plane and render settings are created here.
This module never renders or saves and adds no labels to the products.
"""
import bpy
from mathutils import Vector


def setup_studio(scene, collection, centers):
    """Create a photographic studio; centers are device origins in metres.

    Example: {'AI370':(-.105,0,0), 'KR260':(.105,0,.004)}.
    All cameras track actual objects using explicit location/target vectors, and
    can be selected normally in Blender or via the NUMPAD 0 camera view.
    Returns a dictionary mapping camera names to camera objects.
    """
    scene.unit_settings.system='METRIC'
    scene.unit_settings.length_unit='MILLIMETERS'
    scene.unit_settings.scale_length=1.0
    scene.render.engine='CYCLES'
    scene.cycles.device='CPU'
    scene.cycles.samples=64
    scene.cycles.preview_samples=16
    scene.cycles.use_denoising=True
    scene.render.resolution_x=2560
    scene.render.resolution_y=1600
    scene.render.resolution_percentage=100
    scene.render.film_transparent=False
    scene.render.image_settings.file_format='PNG'
    scene.render.image_settings.color_mode='RGBA'
    scene.render.image_settings.color_depth='8'
    scene.view_settings.view_transform='AgX'
    try:
        scene.view_settings.look='AgX - Medium High Contrast'
    except (TypeError, ValueError):
        pass
    scene.render.fps=24
    world=bpy.data.worlds.new('Hardware studio soft grey world')
    world.use_nodes=True
    world.node_tree.nodes['Background'].inputs['Color'].default_value=(.80,.84,.90,1)
    world.node_tree.nodes['Background'].inputs['Strength'].default_value=.36
    scene.world=world

    # One physical ground plane, just below the device mounting plane.  Its
    # generous size avoids a horizon line in the close-up and three-quarter views.
    ground_mesh=bpy.data.meshes.new('Studio ground mesh')
    ground_mesh.from_pydata([(-2,-2,-.0008),(2,-2,-.0008),(2,2,-.0008),(-2,2,-.0008)],[],[(0,1,2,3)])
    ground_mesh.update()
    ground=bpy.data.objects.new('Studio_Ground',ground_mesh)
    collection.objects.link(ground)
    ground_material=bpy.data.materials.new('Studio warm light grey')
    ground_material.use_nodes=True
    shader=ground_material.node_tree.nodes['Principled BSDF']
    shader.inputs['Base Color'].default_value=(.58,.62,.67,1)
    shader.inputs['Roughness'].default_value=.67
    ground_mesh.materials.append(ground_material)

    def aim(obj,target):
        obj.rotation_euler=(Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()

    def light(name,location,power,size,color=(1,1,1),target=(0,0,.025)):
        data=bpy.data.lights.new(name,'AREA')
        data.energy=power
        data.shape='DISK'
        data.size=size
        data.color=color
        obj=bpy.data.objects.new(name,data)
        collection.objects.link(obj)
        obj.location=location
        aim(obj,target)
        return obj

    light('Studio_Key',(-.26,-.32,.43),38,.34,(1.0,.95,.88))
    light('Studio_Fill',(.30,-.19,.26),24,.28,(.82,.90,1.0))
    light('Studio_Rim',(.03,.31,.39),43,.30,(1.0,1.0,1.0))
    light('Studio_Front_Softbox',(0,-.40,.10),8,.24,(1.0,.98,.94))

    cameras={}
    def camera(name,location,target,scale,lens=65):
        data=bpy.data.cameras.new(name)
        data.type='ORTHO'
        data.ortho_scale=scale
        data.lens=lens
        data.clip_start=.0003
        data.clip_end=20.0
        data.dof.use_dof=False
        data.display_size=.018
        obj=bpy.data.objects.new(name,data)
        collection.objects.link(obj)
        obj.location=location
        aim(obj,target)
        cameras[name]=obj
        return obj

    # A broad overview balances the closed mini PC against the exposed FPGA board.
    if len(centers)>1:
        camera('Overview',(.32,-.45,.34),(0,0,.022),.45)
    elif centers:
        origin=Vector(next(iter(centers.values())))
        camera('Overview',origin+Vector((.23,-.30,.22)),origin+Vector((0,0,.02)),.24)
    else:
        camera('Overview',(.32,-.45,.34),(0,0,.022),.45)

    if 'AI370' in centers:
        origin=Vector(centers['AI370'])
        camera('AI370_Front',origin+Vector((.17,-.23,.14)),
               origin+Vector((0,0,.022)),.205)
        camera('AI370_Rear',origin+Vector((-.17,.23,.13)),
               origin+Vector((0,0,.022)),.205)
        # A nearly level view makes all front socket cavities and contact strips
        # readable, with enough elevation to retain visible housing thickness.
        camera('AI370_PortDetail',origin+Vector((.025,-.20,.058)),
               origin+Vector((0,-.063,.024)),.145)

    if 'KR260' in centers:
        origin=Vector(centers['KR260'])
        camera('KR260_Front',origin+Vector((.17,-.23,.20)),
               origin+Vector((0,0,.016)),.215)
        camera('KR260_Rear',origin+Vector((-.17,.23,.20)),
               origin+Vector((0,0,.016)),.215)
        # Front I/O occupies the -Y edge of the carrier.  This shallow angle shows
        # the connector mouths while keeping the PCB, shields and cooling visible.
        camera('KR260_PortDetail',origin+Vector((.026,-.19,.065)),
               origin+Vector((0,-.047,.011)),.145)

    scene.camera=cameras['Overview']
    # Stored descriptions help users identify the cameras without placing text
    # or floating annotations into the scene or onto the product surfaces.
    for name,obj in cameras.items():
        obj['view_purpose']='Interface close-up' if 'Detail' in name else 'Product view'
    return cameras


# ==================== hardware_main.py ====================

"""Scene assembly, metric conversion and optional file output."""
from mathutils import Matrix, Vector


GENERATOR_TAG = 'AI370_KR260_visual_models_v1'


def prepare_hardware_scene():
    if MODEL_TO_BUILD not in {'BOTH', 'AI370', 'KR260'}:
        raise ValueError('MODEL_TO_BUILD must be BOTH, AI370 or KR260.')
    if bpy.context.object is not None and bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    scene = next((s for s in bpy.data.scenes if s.get('hardware_generator') == GENERATOR_TAG), None)
    if scene is None:
        scene = bpy.data.scenes.new('AI370_and_KR260')
        scene['hardware_generator'] = GENERATOR_TAG
    if bpy.context.window:
        bpy.context.window.scene = scene
    if scene.world and scene.world.get('hardware_generator') == GENERATOR_TAG:
        scene.world = None
    # Only replace tagged objects from earlier runs. Other scenes are preserved.
    for obj in list(scene.objects):
        if obj.get('hardware_generator') == GENERATOR_TAG:
            data = obj.data
            kind = obj.type
            bpy.data.objects.remove(obj, do_unlink=True)
            if data is not None and data.users == 0:
                table = {'MESH': bpy.data.meshes, 'FONT': bpy.data.curves,
                         'CURVE': bpy.data.curves, 'CAMERA': bpy.data.cameras,
                         'LIGHT': bpy.data.lights}.get(kind)
                if table is not None:
                    table.remove(data)
    for col in list(bpy.data.collections):
        if col.get('hardware_generator') == GENERATOR_TAG and not col.objects and not col.children:
            bpy.data.collections.remove(col)
    for mat in list(bpy.data.materials):
        if mat.get('hardware_generator') == GENERATOR_TAG and mat.users == 0:
            bpy.data.materials.remove(mat)
    for world in list(bpy.data.worlds):
        if world.get('hardware_generator') == GENERATOR_TAG and world.users == 0:
            bpy.data.worlds.remove(world)
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = 1.0
    scene.unit_settings.length_unit = 'MILLIMETERS'
    return scene


def model_millimeters_to_meters(collection):
    """Convert geometry, local positions and radii; object scales stay editable.

    Actual mesh coordinates are in metres. There is no hidden 0.001 parent scale.
    Nested component locations and text dimensions are converted as well.
    """
    seen = set()
    scale = Matrix.Scale(.001, 4)
    for obj in collection.all_objects:
        obj.location *= .001
        if obj.type == 'EMPTY':
            obj.empty_display_size *= .001
        if obj.data is not None and obj.data.as_pointer() not in seen:
            seen.add(obj.data.as_pointer())
            if obj.type == 'MESH':
                obj.data.transform(scale)
                obj.data.update()
            elif obj.type == 'FONT':
                obj.data.size *= .001
                obj.data.extrude *= .001
                obj.data.bevel_depth *= .001
                obj.data.offset *= .001
            elif obj.type == 'CURVE':
                obj.data.transform(scale)
                obj.data.bevel_depth *= .001
                obj.data.extrude *= .001
        for modifier in obj.modifiers:
            if modifier.type == 'BEVEL':
                modifier.width *= .001
            elif modifier.type == 'SOLIDIFY':
                modifier.thickness *= .001
    bpy.context.view_layer.update()


def new_hardware_collection(scene, name):
    collection = bpy.data.collections.new(name)
    collection['hardware_generator'] = GENERATOR_TAG
    scene.collection.children.link(collection)
    return collection


def generate_hardware():
    scene = prepare_hardware_scene()
    original_objects = set(bpy.data.objects)
    original_materials = set(bpy.data.materials)
    centers = {}
    roots = []
    palette = {}
    requests = [('AI370', build_ai370), ('KR260', build_kr260)]
    for label, build in requests:
        if MODEL_TO_BUILD not in {'BOTH', label}:
            continue
        collection = new_hardware_collection(scene, label + '_Editable_Model')
        base = Builder(collection, materials=palette)
        root = base.empty(label + '_ROOT')
        build(base.child(root))
        model_millimeters_to_meters(collection)
        # Place feet/lowest connector tails on the studio ground plane.
        minimum = min((obj.matrix_world @ Vector(corner)).z
                      for obj in collection.all_objects if obj.type == 'MESH'
                      for corner in obj.bound_box)
        x = (-.105 if label == 'AI370' else .105) if MODEL_TO_BUILD == 'BOTH' else 0
        root.location = (x, 0, -minimum)
        root.empty_display_size = .018
        centers[label] = tuple(root.location)
        roots.append(root)
        print('Generated:', label, 'objects:', len(collection.all_objects))
    bpy.context.view_layer.update()

    cameras = {}
    if CREATE_STUDIO:
        studio = new_hardware_collection(scene, 'Studio_Cameras_and_Lights')
        cameras = setup_studio(scene, studio, centers)
        scene.render.resolution_x, scene.render.resolution_y = RENDER_RESOLUTION
        scene.cycles.samples = RENDER_SAMPLES
        # A small studio at true physical scale needs modest exposure.
        scene.view_settings.exposure = -2.5
    for obj in set(bpy.data.objects) - original_objects:
        obj['hardware_generator'] = GENERATOR_TAG
    for mat in set(bpy.data.materials) - original_materials:
        mat['hardware_generator'] = GENERATOR_TAG
    if scene.world:
        scene.world['hardware_generator'] = GENERATOR_TAG

    readme = bpy.data.texts.get('Hardware_README') or bpy.data.texts.new('Hardware_README')
    readme.clear()
    readme.write(HARDWARE_README)
    if bpy.context.window:
        for obj in bpy.context.view_layer.objects:
            obj.select_set(False)
        for root in roots:
            root.select_set(True)
        bpy.context.view_layer.objects.active = roots[0]
        for area in bpy.context.screen.areas:
            if area.type == 'VIEW_3D':
                space = area.spaces.active
                space.clip_start = .0001
                space.clip_end = 100
                space.shading.type = 'SOLID'
                space.shading.color_type = 'MATERIAL'
                space.overlay.show_extras = False
                region = space.region_3d
                region.view_location = Vector((0, 0, .025))
                region.view_distance = .50 if len(roots) == 2 else .27
                region.view_perspective = 'PERSP'
                if scene.camera:
                    region.view_rotation = scene.camera.rotation_euler.to_quaternion()

    if SAVE_BLEND_PATH:
        bpy.ops.wm.save_as_mainfile(filepath=bpy.path.abspath(SAVE_BLEND_PATH))
    if RENDER_PREVIEW_PATH:
        if not scene.camera:
            raise ValueError('Rendering requires CREATE_STUDIO = True.')
        scene.render.filepath = bpy.path.abspath(RENDER_PREVIEW_PATH)
        bpy.ops.render.render(write_still=True, scene=scene.name)
    print('完成：AI370 / KR260 模型可在 Outliner 展开并单独编辑。')
    return scene, roots, cameras


if __name__ == '__main__':
    generate_hardware()
