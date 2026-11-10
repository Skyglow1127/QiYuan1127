#!/usr/bin/env python3
"""Build the photo-derived, static Gazebo workspace from its shared manifest.

All dimensions in the manifest are estimates. The robot, mounting plate,
dynamic red cube and named ``floor`` support collider belong to lab.sdf.
This generator intentionally does not alter them or connect to hardware.
Only the Python standard library is required.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
COLOR = re.compile(r"^#[0-9a-fA-F]{6}$")


def vector(value, count, context, *, positive=False):
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{context}: expected {count} numbers")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(f"{context}: expected finite numbers")
    if positive and any(v <= 0 for v in value):
        raise ValueError(f"{context}: dimensions must be positive")


def validate(manifest):
    if manifest.get("version") != 1:
        raise ValueError("Unsupported environment manifest version")
    names = set()
    for visual in manifest["shapes"]:
        name = visual["name"]
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_.-]*", name) or name in names:
            raise ValueError(f"Invalid or duplicate visual name: {name}")
        names.add(name)
        vector(visual["position"], 3, f"{name}.position")
        vector(visual.get("rpy", [0, 0, 0]), 3, f"{name}.rpy")
        if not COLOR.fullmatch(visual["color"]):
            raise ValueError(f"{name}: expected #rrggbb color")
        geometry = visual["geometry"]
        if geometry == "box":
            vector(visual["size"], 3, f"{name}.size", positive=True)
        elif geometry in ("cylinder", "sphere"):
            vector([visual["radius"]], 1, f"{name}.radius", positive=True)
            if geometry == "cylinder":
                vector([visual["length"]], 1, f"{name}.length", positive=True)
        else:
            raise ValueError(f"{name}: unsupported geometry {geometry}")
        for key in ("roughness", "metalness"):
            if key in visual and not 0 <= visual[key] <= 1:
                raise ValueError(f"{name}.{key}: expected value from 0 to 1")
    names = set()
    for collider in manifest["colliders"]:
        name = collider["name"]
        if name in names or name in ("floor", "table", "worktop"):
            raise ValueError(f"Invalid/duplicate reserved collision name: {name}")
        names.add(name)
        vector(collider["position"], 3, f"{name}.position")
        vector(collider["size"], 3, f"{name}.size", positive=True)
        vector(collider.get("rpy", [0, 0, 0]), 3, f"{name}.rpy")
    support = manifest["support_surface"]
    if support["name"] != "floor":
        raise ValueError("The existing red-cube floor support name must be retained")
    vector(support["position"], 3, "support_surface.position")
    vector(support["size"], 3, "support_surface.size", positive=True)
    vector(support.get("rpy", [0, 0, 0]), 3, "support_surface.rpy")
    for key in ("position", "target"):
        vector(manifest["camera"][key], 3, f"camera.{key}")
    for label in manifest.get("labels", []):
        vector(label["position"], 3, "label.position")


def sub(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def numbers(values):
    return " ".join(f"{v:.9g}" for v in values)


def pose(element, item):
    sub(element, "pose", numbers(item["position"] + item.get("rpy", [0, 0, 0])))


def geometry(element, item):
    geo = sub(element, "geometry")
    primitive = sub(geo, item.get("geometry", "box"))
    if item.get("geometry", "box") == "box":
        sub(primitive, "size", numbers(item["size"]))
    else:
        sub(primitive, "radius", item["radius"])
        if item["geometry"] == "cylinder":
            sub(primitive, "length", item["length"])


def material(visual, color, roughness=0.65, metalness=0.0, emissive=False):
    rgb = [int(color[i : i + 2], 16) / 255 for i in (1, 3, 5)]
    rgba = numbers(rgb + [1])
    mat = sub(visual, "material")
    sub(mat, "ambient", rgba)
    sub(mat, "diffuse", rgba)
    sub(mat, "specular", numbers([0.15 + metalness * 0.35] * 3 + [1]))
    if emissive:
        sub(mat, "emissive", numbers([c * 0.28 for c in rgb] + [1]))
    metal = sub(sub(mat, "pbr"), "metal")
    sub(metal, "roughness", roughness)
    sub(metal, "metalness", metalness)


# Small, self-contained geometric lettering. Each filled font pixel becomes
# two triangles, so native Gazebo labels need neither textures nor system fonts.
FONT = {
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "I": ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "N": ["10001", "11001", "11001", "10101", "10011", "10011", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "X": ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    "Z": ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "6": ["01110", "10000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00001", "01110"],
    " ": ["00000"] * 7,
}


def write_text_mesh(output, text, pixel):
    vertices = []
    triangles = []
    total_width = (len(text) * 6 - 1) * pixel
    for char_index, char in enumerate(text):
        for row, bits in enumerate(FONT[char]):
            for column, filled in enumerate(bits):
                if filled != "1":
                    continue
                # Front view: +X points up the writing, -Y points right.
                x0 = (3 - row) * pixel
                y0 = total_width / 2 - (char_index * 6 + column) * pixel
                width = pixel * 0.87
                offset = len(vertices)
                vertices.extend([(x0, y0, 0), (x0 - width, y0, 0), (x0 - width, y0 - width, 0), (x0, y0 - width, 0)])
                triangles.extend([offset, offset + 1, offset + 2, offset, offset + 2, offset + 3])
    root = ET.Element("COLLADA", {"xmlns": "http://www.collada.org/2005/11/COLLADASchema", "version": "1.4.1"})
    asset = sub(root, "asset")
    sub(asset, "created", "2026-10-26T00:00:00Z")
    sub(asset, "modified", "2026-10-26T00:00:00Z")
    sub(asset, "unit", name="meter", meter="1")
    sub(asset, "up_axis", "Z_UP")
    mesh = sub(sub(sub(root, "library_geometries"), "geometry", id="lettering", name="lettering"), "mesh")
    source = sub(mesh, "source", id="positions")
    sub(source, "float_array", numbers([n for v in vertices for n in v]), id="positions-array", count=str(len(vertices) * 3))
    accessor = sub(sub(source, "technique_common"), "accessor", source="#positions-array", count=str(len(vertices)), stride="3")
    for axis in ("X", "Y", "Z"):
        sub(accessor, "param", name=axis, type="float")
    source = sub(mesh, "source", id="normals")
    sub(source, "float_array", "0 0 1", id="normals-array", count="3")
    accessor = sub(sub(source, "technique_common"), "accessor", source="#normals-array", count="1", stride="3")
    for axis in ("X", "Y", "Z"):
        sub(accessor, "param", name=axis, type="float")
    sub(sub(mesh, "vertices", id="vertices"), "input", semantic="POSITION", source="#positions")
    tri = sub(mesh, "triangles", count=str(len(triangles) // 3))
    sub(tri, "input", semantic="VERTEX", source="#vertices", offset="0")
    sub(tri, "input", semantic="NORMAL", source="#normals", offset="1")
    sub(tri, "p", " ".join(f"{index} 0" for index in triangles))
    scene = sub(sub(root, "library_visual_scenes"), "visual_scene", id="Scene", name="Scene")
    sub(sub(scene, "node", id="Text", name="Text"), "instance_geometry", url="#lettering")
    sub(sub(root, "scene"), "instance_visual_scene", url="#Scene")
    ET.indent(root)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    return {"vertices": vertices, "faces": [triangles[i:i+3] for i in range(0,len(triangles),3)]}


def build(config, output):
    manifest = json.loads(config.read_text())
    validate(manifest)
    output.mkdir(parents=True, exist_ok=True)
    root = ET.Element("sdf", version="1.9")
    model = sub(root, "model", name="photo_workspace")
    sub(model, "static", "true")
    link = sub(model, "link", name="workspace")
    sub(link, "gravity", "false")
    for item in manifest["shapes"]:
        # SDF requires visual/collision names to be unique within the same
        # link as well, so preserve collider IDs and prefix display IDs.
        visual = sub(link, "visual", name="visual_" + item["name"])
        pose(visual, item)
        geometry(visual, item)
        material(visual, item["color"], item.get("roughness", 0.65), item.get("metalness", 0.0), "_led" in item["name"] or "status_light" in item["name"])
        if item["group"] == "cables":
            sub(visual, "cast_shadows", "false")
    for item in manifest["colliders"]:
        collision = sub(link, "collision", name=item["name"])
        pose(collision, item)
        geometry(collision, item)
        surface = sub(collision, "surface")
        sub(sub(surface, "contact"), "collide_bitmask", "1")
        ode = sub(sub(surface, "friction"), "ode")
        sub(ode, "mu", "0.8")
        sub(ode, "mu2", "0.8")
    meshes = output / "meshes"
    meshes.mkdir(exist_ok=True)
    markings = manifest.get("markings", [
        ("aipc_brand", "MINISFORUM", 0.00115, [-0.08, -0.31, 0.0548], "#69777e"),
        ("aipc_processor", "RYZEN AI 9 HX 370", 0.00072, [-0.107, -0.31, 0.0548], "#69777e"),
        ("kr260_name", "AMD KR260", 0.00072, [-0.165, -0.603, 0.0146], "#e5dfbf"),
    ])
    display_markings = []
    for marking in markings:
        name, text, pixel, position, color = marking[:5]
        rpy = marking[5] if len(marking)>5 else [0,0,0]
        display_geometry = write_text_mesh(meshes / f"{name}.dae", text, pixel)
        display_markings.append({"name":name,"position":position,"rpy":rpy,"color":color,**display_geometry})
        visual = sub(link, "visual", name=name + "_lettering")
        pose(visual, {"position": position,"rpy":rpy})
        geo = sub(visual, "geometry")
        mesh = sub(geo, "mesh")
        sub(mesh, "uri", f"model://photo_workspace/meshes/{name}.dae")
        material(visual, color, 0.8)
        sub(visual, "cast_shadows", "false")
    (output / "markings.json").write_text(json.dumps(display_markings,separators=(',',':')))
    ET.indent(root)
    model_path = output / "model.sdf"
    ET.ElementTree(root).write(model_path, encoding="utf-8", xml_declaration=True)
    config_root = ET.Element("model")
    sub(config_root, "name", "D1-T photo laboratory workspace")
    sub(config_root, "version", "1.0")
    sub(config_root, "sdf", "model.sdf", version="1.9")
    sub(config_root, "description", "Workspace reconstruction using user photos and cited official device images/drawings. Device package dimensions follow those sources; placement and local surface details remain approximate. Static geometry and conservative collision envelopes only.")
    ET.indent(config_root)
    ET.ElementTree(config_root).write(output / "model.config", encoding="utf-8", xml_declaration=True)
    # Read back generated XML to catch malformed output before a launcher uses it.
    parsed = ET.parse(model_path)
    if len(parsed.findall(".//collision")) != len(manifest["colliders"]):
        raise RuntimeError("Generated collision count differs from manifest")
    for mesh_path in meshes.glob("*.dae"):
        ET.parse(mesh_path)
    print(f"Built {model_path}: {len(manifest['shapes'])} primitives, {len(markings)} geometric text labels, {len(manifest['colliders'])} collision envelopes")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/environment.json")
    parser.add_argument("--output", type=Path, default=ROOT / "models/photo_workspace")
    args = parser.parse_args()
    build(args.config, args.output)
