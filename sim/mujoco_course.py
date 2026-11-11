"""Build a MuJoCo scene from the published course, without flight commands.

OBJ groups retain independent visuals. Concave collision meshes are decomposed,
never replaced by a single convex hull that would close the tunnel or window.
This scene and its generic quadrotor are not a Sky Beta/PX4 dynamics model.
"""

import argparse
from hashlib import sha256
import json
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


def load_groups(source):
    vertices, groups = [], {}
    name, material = "ungrouped", None
    colors = {}
    mtl = source.with_suffix(".mtl")
    for line in mtl.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "newmtl":
            material = fields[1]
        elif fields[0] == "Kd":
            colors[material] = list(map(float, fields[1:4]))
    material = None
    for line in source.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "g":
            name = " ".join(fields[1:])
        elif fields[0] == "v":
            vertices.append(list(map(float, fields[1:4])))
        elif fields[0] == "usemtl":
            material = fields[1]
        elif fields[0] == "f":
            entry = groups.setdefault(name, {"faces": [], "color": colors.get(material, [.5] * 3)})
            face = [int(x.split("/")[0]) for x in fields[1:]]
            face = [i - 1 if i > 0 else len(vertices) + i for i in face]
            for i in range(1, len(face) - 1):
                entry["faces"].append([face[0], face[i], face[i + 1]])
    vertices = np.array(vertices)
    result = {}
    for name, entry in groups.items():
        faces = np.array(entry["faces"])
        ids, inverse = np.unique(faces, return_inverse=True)
        mesh = trimesh.Trimesh(vertices=vertices[ids], faces=inverse.reshape(-1, 3), process=True)
        result[name] = (mesh, entry["color"])
    return result


def numbers(values):
    return " ".join(f"{float(x):.9g}" for x in values)


def add_quad(world, root):
    """Illustrative 0.8 kg X quad; rotor force, inertia and mounts need calibration."""
    body = ET.SubElement(world, "body", name="proxy_quad", pos="-2.5146 .9144 .12")
    ET.SubElement(body, "freejoint", name="quad_free")
    ET.SubElement(body, "inertial", pos="0 0 0", mass="0.8", diaginertia=".009 .009 .016")
    ET.SubElement(body, "geom", name="quad_center", type="box", size=".09 .06 .035",
                  rgba=".1 .4 .8 1", mass="0")
    actuator = ET.SubElement(root, "actuator")
    for i, (x, y, spin) in enumerate(((.14, .14, 1), (-.14, .14, -1),
                                    (-.14, -.14, 1), (.14, -.14, -1))):
        ET.SubElement(body, "geom", name=f"arm_{i}", type="capsule",
                      fromto=numbers([0, 0, 0, x, y, 0]), size=".012", mass="0",
                      rgba=".2 .2 .2 1")
        ET.SubElement(body, "geom", name=f"rotor_envelope_{i}", type="cylinder",
                      pos=numbers([x, y, .025]), size=".075 .004", mass="0",
                      rgba=".15 .15 .15 .6")
        ET.SubElement(body, "site", name=f"rotor_{i}", pos=numbers([x, y, 0]), size=".006")
        ET.SubElement(actuator, "motor", name=f"thrust_{i}", site=f"rotor_{i}",
                      gear=numbers([0, 0, 1, 0, 0, spin * .02]),
                      ctrllimited="true", ctrlrange="0 4")
    ET.SubElement(body, "site", name="imu", pos="0 0 0", size=".003")
    # MuJoCo rangefinder shoots along local +Z, hence the downward orientation.
    ET.SubElement(body, "site", name="down_range", pos="0 0 -.04", quat="0 1 0 0", size=".003")
    for name, y in (("left", .0375), ("right", -.0375), ("rgb", 0)):
        ET.SubElement(body, "camera", name=name, pos=numbers([.09, y, .025]),
                      xyaxes="0 -1 0 0 0 1", fovy="55")
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="gyro", site="imu")
    ET.SubElement(sensors, "accelerometer", name="accelerometer", site="imu")
    ET.SubElement(sensors, "rangefinder", name="down_range", site="down_range", cutoff="4")
    # Deliberately no ground-truth pose/velocity sensor or fake VIO position fix.


def build(source, output):
    import coacd
    source, output = Path(source), Path(output)
    if output.exists():
        raise FileExistsError(f"choose a new output directory: {output}")
    groups = load_groups(source)
    mesh_dir = output / "meshes"
    mesh_dir.mkdir(parents=True)
    started = time.perf_counter()
    root = ET.Element("mujoco", model="dronehacks_proxy")
    ET.SubElement(root, "compiler", angle="radian", meshdir="meshes", autolimits="true")
    ET.SubElement(root, "option", timestep="0.004166666666667", gravity="0 0 -9.80665",
                  integrator="implicitfast")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    ET.SubElement(visual, "headlight", ambient=".25 .25 .25", diffuse=".6 .6 .6")
    asset = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 0 5", dir="0 0 -1", directional="true",
                  diffuse=".6 .6 .6", castshadow="true")
    manifest = {"kind": "mujoco_proxy_scene_not_qualified_flight", "groups": [],
                "obj_sha256": sha256(source.read_bytes()).hexdigest(),
                "mtl_sha256": sha256(source.with_suffix('.mtl').read_bytes()).hexdigest(),
                "collision_threshold_metres": .01, "decomposition_seed": 0,
                "collision_method": "convex pieces; CoACD for concave groups",
                "quad": "illustrative 0.8 kg, uncalibrated; not Sky Beta or PX4",
                "position_estimator": None, "flight_run": False}
    coacd.set_log_level("warn")
    for i, (name, (mesh, color)) in enumerate(groups.items()):
        key = f"g{i:02d}_" + re.sub(r"[^a-zA-Z0-9]+", "_", name)
        mesh.export(mesh_dir / f"{key}.obj")
        ET.SubElement(asset, "mesh", name=key, file=f"{key}.obj")
        alpha = .06 if name.startswith("Net ") else 1.0
        ET.SubElement(world, "geom", name=f"visual_{key}", type="mesh", mesh=key,
                      rgba=numbers([*color, alpha]), contype="0", conaffinity="0",
                      group="1")
        decorative = name.startswith(("Guideline", "Launch mark", "Landing pad", "Approach ring"))
        parts = []
        method = "decorative_no_collision"
        if not decorative:
            hull = mesh.convex_hull
            ratio = abs(mesh.volume) / hull.volume
            if mesh.is_watertight and ratio > .999:
                parts = [(hull.vertices, hull.faces)]
                method = "convex_hull"
            else:
                print(f"Decomposing {name} (volume ratio {ratio:.4f})", flush=True)
                parts = coacd.run_coacd(coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.faces)),
                                        threshold=.01, real_metric=True, seed=0,
                                        mcts_iterations=60, mcts_nodes=15,
                                        preprocess_mode="auto", merge=True)
                method = "coacd"
            for j, (vertices, faces) in enumerate(parts):
                part_name = f"{key}_c{j}"
                trimesh.Trimesh(vertices, faces, process=False).export(mesh_dir / f"{part_name}.obj")
                ET.SubElement(asset, "mesh", name=part_name, file=f"{part_name}.obj")
                ET.SubElement(world, "geom", name=f"collision_{part_name}", type="mesh",
                              mesh=part_name, rgba="1 .2 .2 .2", group="3",
                              friction=".6 .005 .0001")
        manifest["groups"].append({"name": name, "key": key, "method": method,
                                    "collision_parts": len(parts), "bounds": mesh.bounds.tolist()})
    add_quad(world, root)
    ET.indent(root)
    ET.ElementTree(root).write(output / "course.xml", encoding="utf-8", xml_declaration=True)
    manifest["build_seconds"] = time.perf_counter() - started
    manifest["mjcf_sha256"] = sha256((output / "course.xml").read_bytes()).hexdigest()
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path,
                        default=Path("dronehacks-course-v3-obj/dronehacks-course-v3.obj"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/mujoco-course-v1"))
    args = parser.parse_args()
    build(args.source, args.output)
    print(args.output / "course.xml")


if __name__ == "__main__":
    main()
