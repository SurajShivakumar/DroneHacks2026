"""Package the supplied OBJ/MTL as a static Gazebo world; never starts flight.

Run from the repository root: python tools/build_course.py
Requires Gazebo Harmonic on Linux to open the generated scene. The world has
no aircraft, autopilot, sensors, localization, checkpoint grader or recorder.
"""

import argparse
from hashlib import sha256
import json
from pathlib import Path
import shutil
import struct
from math import sqrt
import xml.etree.ElementTree as ET


def split_visuals(source, destination):
    """Preserve face/vertex geometry while separating net material treatment."""
    vertices, groups, name, material = [], {}, "ungrouped", None
    colors = {}
    for line in source.with_suffix('.mtl').read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if fields and fields[0] == "newmtl":
            material = fields[1]
        elif fields and fields[0] == "Kd":
            colors[material] = fields[1:4]
    material = None
    for line in source.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "v":
            vertices.append(" ".join(fields[1:4]))
        elif fields[0] == "g":
            name = " ".join(fields[1:])
        elif fields[0] == "usemtl":
            material = fields[1]
        elif fields[0] == "f":
            face = [int(item.split('/')[0]) for item in fields[1:]]
            face = [i - 1 if i > 0 else len(vertices) + i for i in face]
            groups.setdefault(name, {"material": material, "faces": []})["faces"].append(face)
    records = []
    for i, (name, entry) in enumerate(groups.items()):
        triangles = [(face[0], face[j], face[j + 1]) for face in entry["faces"]
                     for j in range(1, len(face) - 1)]
        filename = f"visual_{i:02d}.stl"
        with (destination / filename).open('wb') as stream:
            stream.write(b'DroneHacks visual mesh with explicit facet normals'.ljust(80, b' '))
            stream.write(struct.pack('<I', len(triangles)))
            for triangle in triangles:
                points = [list(map(float, vertices[index].split())) for index in triangle]
                a = [points[1][k] - points[0][k] for k in range(3)]
                b = [points[2][k] - points[0][k] for k in range(3)]
                normal = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
                length = sqrt(sum(x*x for x in normal))
                normal = [x / length for x in normal] if length else [0, 0, 0]
                stream.write(struct.pack('<12fH', *normal, *points[0], *points[1], *points[2], 0))
        records.append({"name": name, "file": filename, "net": name.startswith("Net "),
                        "color": colors.get(entry["material"], ["0.5"] * 3),
                        "triangles": len(triangles)})
    return records


def inspect_obj(source):
    vertices, groups = [], {}
    group = "ungrouped"
    for line in source.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "g":
            group = " ".join(fields[1:])
        elif fields[0] == "v":
            vertices.append(tuple(map(float, fields[1:4])))
        elif fields[0] == "f":
            indices = groups.setdefault(group, set())
            for item in fields[1:]:
                index = int(item.split("/")[0])
                indices.add(index - 1 if index > 0 else len(vertices) + index)
    bounds = {}
    for name, indices in groups.items():
        points = [vertices[i] for i in indices]
        bounds[name] = {
            "min": [min(p[k] for p in points) for k in range(3)],
            "max": [max(p[k] for p in points) for k in range(3)],
        }
    return {"units": "metres", "up": "Z", "groups": bounds,
            "warning": "Bounding boxes are not opening clearance or onboard observations."}


def build(source, output):
    source, output = Path(source), Path(output)
    obj = source / "dronehacks-course-v3.obj"
    mtl = source / "dronehacks-course-v3.mtl"
    # Refuse replacement, so an inspected scene or its evidence is preserved.
    if output.exists():
        raise FileExistsError(f"choose a new output directory: {output}")
    obj_bytes, mtl_bytes = obj.read_bytes(), mtl.read_bytes()
    bounds = inspect_obj(obj)
    mesh_dir = output / "models" / "dronehacks_course" / "meshes"
    mesh_dir.mkdir(parents=True)
    shutil.copyfile(obj, mesh_dir / obj.name)
    shutil.copyfile(mtl, mesh_dir / mtl.name)
    model_dir = mesh_dir.parent
    model_config = '''<?xml version="1.0"?>
<model><name>DroneHacks Course v3</name><version>1.0</version>
<sdf version="1.6">model.sdf</sdf>
<description>Published preliminary course geometry. Units metres, Z up.</description></model>
'''
    model_sdf = '''<?xml version="1.0"?>
<sdf version="1.6"><model name="dronehacks_course"><static>true</static>
<link name="course">
<collision name="course_collision"><geometry><mesh>
<uri>model://dronehacks_course/meshes/dronehacks-course-v3.obj</uri>
</mesh></geometry></collision>
<visual name="course_visual"><geometry><mesh>
<uri>model://dronehacks_course/meshes/dronehacks-course-v3.obj</uri>
</mesh></geometry></visual>
</link></model></sdf>
'''
    pieces = split_visuals(obj, mesh_dir)
    model_xml = ET.fromstring(model_sdf)
    link = model_xml.find("model/link")
    link.remove(link.find("visual"))
    for i, piece in enumerate(pieces):
        visual = ET.SubElement(link, "visual", name=f"piece_{i:02d}")
        mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = "model://dronehacks_course/meshes/" + piece["file"]
        # Set SDF colors explicitly; OBJ material handling varies by renderer.
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "ambient").text = "0 0 0 1"
        ET.SubElement(material, "diffuse").text = " ".join(piece["color"]) + " 1"
        ET.SubElement(material, "specular").text = "0 0 0 1"
        if piece["net"]:
            ET.SubElement(visual, "transparency").text = "0.94"
            ET.SubElement(visual, "cast_shadows").text = "false"
    model_sdf = ET.tostring(model_xml, encoding="unicode")
    world_sdf = '''<?xml version="1.0"?>
<sdf version="1.9"><world name="dronehacks">
<physics name="default" type="ignored"><max_step_size>0.004166666666667</max_step_size>
<real_time_factor>1.0</real_time_factor></physics>
<gravity>0 0 -9.80665</gravity>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
<render_engine>ogre2</render_engine></plugin>
<scene><ambient>0.35 0.35 0.35 1</ambient><background>0.7 0.75 0.8 1</background></scene>
<light name="sun" type="directional"><pose>0 0 8 0 0 0</pose>
<diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular>
<direction>-0.5 0.2 -1</direction><cast_shadows>true</cast_shadows></light>
<include><uri>model://dronehacks_course</uri></include>
</world></sdf>
'''
    for relative, content in (("models/dronehacks_course/model.config", model_config),
                              ("models/dronehacks_course/model.sdf", model_sdf),
                              ("course.sdf", world_sdf)):
        ET.fromstring(content)
        (output / relative).write_text(content, encoding="utf-8")
    # Also allow scene inspection on an existing Gazebo Classic 11 install.
    # This compatibility scene is not the target ROS 2/PX4 flight environment.
    classic = ET.fromstring(world_sdf)
    classic.set("version", "1.6")
    world = classic.find("world")
    world.find("physics").set("type", "ode")
    for plugin in list(world.findall("plugin")):
        world.remove(plugin)
    ET.ElementTree(classic).write(output / "course-classic.world", encoding="utf-8",
                                  xml_declaration=True)
    provenance = {
        "kind": "static_course_scene_not_flight_simulation",
        "obj_sha256": sha256(obj_bytes).hexdigest(),
        "mtl_sha256": sha256(mtl_bytes).hexdigest(),
        "runtime_validated": False,
        "geometry": bounds,
        "visual_pieces": pieces,
        "net_visual": "transparent non-shadowing sheets; original collision mesh retained",
    }
    (output / "scene_manifest.json").write_text(
        json.dumps(provenance, indent=2) + "\n", encoding="utf-8")
    return provenance


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path("dronehacks-course-v3-obj"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/course-v3"))
    args = parser.parse_args()
    build(args.source, args.output)
    print(f"Built static course scene at {args.output}; Gazebo runtime validation pending.")
