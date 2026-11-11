"""Import supplied STEP assembly with its explicit SI material densities.

Preserves original assets. Exports metre/FLU meshes, mass/COM/inertia and source
provenance. Missing propellers/wiring/motor calibration remain explicit gaps.
"""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import re

import numpy as np
import trimesh

CAD_TO_FLU = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])


def step_densities(text):
    """Read the supplied material-property graph; densities are kg/m^3.

    The archive explicitly uses kg and metre^-3 for density, while geometric
    length is millimetres. Reject unexpected density dimensions/units.
    """
    if 'SI_UNIT(.KILO.,.GRAM.)' not in text or 'SI_UNIT($,.METRE.)' not in text:
        raise ValueError('review density unit graph before importing another STEP')
    definitions = dict(re.findall(r"#(\d+)=PRODUCT_DEFINITION\('([^']*)'", text))
    properties = dict(re.findall(r"#(\d+)=PROPERTY_DEFINITION\('material property','density of part',#(\d+)\)", text))
    links = dict(re.findall(r"PROPERTY_DEFINITION_REPRESENTATION\(#(\d+),#(\d+)\)", text))
    representations = dict(re.findall(r"#(\d+)=REPRESENTATION\('density',\(#(\d+)\)", text))
    values = {key: (float(value), unit) for key, value, unit in re.findall(
        r"#(\d+)=MEASURE_REPRESENTATION_ITEM\('density measure',\s*POSITIVE_RATIO_MEASURE\(([^)]+)\),#(\d+)\)", text)}
    derived = {key: (first, second) for key, first, second in re.findall(
        r'#(\d+)=DERIVED_UNIT\(\(#(\d+),#(\d+)\)\)', text)}
    elements = {key: (unit, float(power)) for key, unit, power in re.findall(
        r'#(\d+)=DERIVED_UNIT_ELEMENT\(#(\d+),([^)]+)\)', text)}
    base_units = {key: body for key, body in re.findall(
        r'#(\d+)=\(\s*((?:MASS_UNIT|LENGTH_UNIT)\(\)[^;]+)\);', text)}
    result = {}
    for prop, definition in properties.items():
        value, unit = values[representations[links[prop]]]
        dimensions = [(base_units[base], power) for base, power in
                      (elements[element] for element in derived[unit])]
        if (len(dimensions) != 2 or not any('SI_UNIT(.KILO.,.GRAM.)' in body and power == 1 for body, power in dimensions)
                or not any('SI_UNIT($,.METRE.)' in body and power == -3 for body, power in dimensions)):
            raise ValueError('density must explicitly use kg per metre cubed')
        if not 100 <= value <= 25000:
            raise ValueError('implausible SI material density')
        result[definitions[definition]] = value
    if len(result) != 19:
        raise ValueError('supplied assembly density coverage changed; review required')
    return result


def aggregate_mass_properties(parts):
    solids = [p for p in parts if p['mass_kg'] > 0]
    total = sum(p['mass_kg'] for p in solids)
    if not np.isfinite(total) or total <= 0:
        raise ValueError('positive finite aggregate mass required')
    com = sum(p['mass_kg']*np.asarray(p['com_flu_m']) for p in solids)/total
    inertia = np.zeros((3, 3))
    for part in solids:
        delta = np.asarray(part['com_flu_m'])-com
        tensor = np.asarray(part['inertia_about_part_com_flu_kg_m2'])
        if tensor.shape != (3, 3) or not np.isfinite(tensor).all() or not np.allclose(tensor, tensor.T):
            raise ValueError('finite symmetric component inertia required')
        inertia += tensor+part['mass_kg']*(delta@delta*np.eye(3)-np.outer(delta, delta))
    if not np.isfinite(com).all() or np.linalg.eigvalsh(inertia).min() <= 0:
        raise ValueError('nonphysical aggregate inertia')
    return total, com, inertia


def import_cad(source, mount, output):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRep import BRep_Tool
    from OCP.GProp import GProp_GProps
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label, TDF_LabelSequence
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    output.mkdir(parents=True, exist_ok=False)
    (output/'meshes').mkdir()
    text = source.read_text(encoding='utf-8')
    densities = step_densities(text)
    document = TDocStd_Document(TCollection_ExtendedString('Sky Beta'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetMatMode(True)
    if reader.ReadFile(str(source.resolve())) != IFSelect_RetDone or not reader.Transfer(document):
        raise RuntimeError('STEP transfer failed')
    tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())

    def label_name(label):
        attribute = TDataStd_Name()
        return attribute.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attribute) else ''

    leaves = []
    def walk(label, location):
        if tool.IsReference_s(label):
            referred = TDF_Label()
            tool.GetReferredShape_s(label, referred)
            walk(referred, location.Multiplied(tool.GetLocation_s(label)))
        elif label_name(label) in densities:
            # Material may belong to a complete component assembly (e.g. the
            # simplified Jetson), not its unnamed internal COMPOUND leaves.
            # Integrate that component once, including all placed child solids.
            leaves.append((label_name(label), tool.GetShape_s(label).Moved(location)))
        elif tool.IsAssembly_s(label):
            components = TDF_LabelSequence()
            tool.GetComponents_s(label, components)
            for i in range(1, components.Length()+1):
                walk(components.Value(i), location)
        else:
            leaves.append((label_name(label), tool.GetShape_s(label).Moved(location)))

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length()+1):
        walk(roots.Value(i), TopLoc_Location())
    parts = []
    all_meshes = []
    for index, (name, shape) in enumerate(leaves):
        properties = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape, properties, True, False, False)
        volume_mm3 = properties.Mass()
        density = densities.get(name)
        # The supplied camera mount is a separate STL, not a STEP solid.
        if volume_mm3 <= 1e-9:
            parts.append({'name': name, 'volume_m3': 0., 'mass_kg': 0., 'note': 'empty/non-solid STEP placeholder'})
            continue
        if density is None:
            raise ValueError(f'No supplied density for solid part {name!r}')
        center = properties.CentreOfMass()
        com = CAD_TO_FLU @ np.array([center.X(), center.Y(), center.Z()])*.001
        inertia_mm5 = properties.MatrixOfInertia()
        inertia = np.array([[inertia_mm5.Value(i, j) for j in range(1, 4)] for i in range(1, 4)])
        inertia = CAD_TO_FLU @ inertia @ CAD_TO_FLU.T * density*1e-15
        mass = volume_mm3*density*1e-9
        BRepMesh_IncrementalMesh(shape, .25, False, .35, True).Perform()
        vertices, faces = [], []
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        while explorer.More():
            face = TopoDS.Face_s(explorer.Current())
            location = TopLoc_Location()
            triangulation = BRep_Tool.Triangulation_s(face, location)
            if triangulation is not None:
                offset = len(vertices)
                for i in range(1, triangulation.NbNodes()+1):
                    point = triangulation.Node(i).Transformed(location.Transformation())
                    vertices.append(CAD_TO_FLU @ np.array([point.X(), point.Y(), point.Z()])*.001)
                for i in range(1, triangulation.NbTriangles()+1):
                    triangle = list(triangulation.Triangle(i).Get())
                    if face.Orientation() == TopAbs_REVERSED:
                        triangle.reverse()
                    faces.append([offset+j-1 for j in triangle])
            explorer.Next()
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
        filename = f'{index:02d}_'+re.sub(r'[^a-zA-Z0-9]+', '_', name).strip('_')+'.stl'
        mesh.export(output/'meshes'/filename)
        mesh.convex_hull.export(output/'meshes'/filename.replace('.stl', '_collision.stl'))
        all_meshes.append(mesh)
        parts.append({'name': name, 'mesh': 'meshes/'+filename, 'density_kg_m3': density,
                      'mass_kg': mass, 'volume_m3': volume_mm3*1e-9, 'com_flu_m': com.tolist(),
                      'inertia_about_part_com_flu_kg_m2': inertia.tolist(),
                      'bounds_flu_m': mesh.bounds.tolist(), 'triangles': len(mesh.faces)})
        print(name, 'kg', round(mass, 6), flush=True)
    mount_mesh = trimesh.load_mesh(mount, process=True)
    mount_mesh.apply_transform(np.block([[CAD_TO_FLU*.001, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]))
    mount_mesh.export(output/'meshes/camera_mount.stl')
    mount_mesh.convex_hull.export(output/'meshes/camera_mount_collision.stl')
    if not mount_mesh.is_watertight or mount_mesh.volume <= 0:
        raise ValueError('STL mount must enclose a positive volume before assigning supplied PLA density')
    mount_mass = mount_mesh.volume*1240.
    parts.append({'name': 'Supplied STL camera mount', 'mesh': 'meshes/camera_mount.stl',
                  'mass_kg': mount_mass, 'volume_m3': mount_mesh.volume,
                  'density_kg_m3': 1240., 'com_flu_m': mount_mesh.center_mass.tolist(),
                  'inertia_about_part_com_flu_kg_m2': (mount_mesh.moment_inertia*1240.).tolist(),
                  'bounds_flu_m': mount_mesh.bounds.tolist(), 'triangles': len(mount_mesh.faces)})
    all_meshes.append(mount_mesh)
    total, com, inertia = aggregate_mass_properties(parts)
    trimesh.util.concatenate(all_meshes).export(output/'sky_beta_assembled.stl')
    report = {'kind': 'sky_beta_cad_geometry_and_documented_mass_estimate',
              'source_sha256': {p.name: sha256(p.read_bytes()).hexdigest() for p in (source, mount)},
              'cad_units': 'millimetres', 'output_units': 'metres',
              'cad_to_flu_rotation': CAD_TO_FLU.tolist(), 'output_origin': 'original CAD origin',
              'cad_mass_kg_excluding_omissions': total, 'com_flu_m': com.tolist(),
              'inertia_about_com_flu_kg_m2': inertia.tolist(), 'parts': parts,
              'omissions': ['propellers', 'wiring', 'downward VL53L1X rangefinder'],
              'dynamics_missing': ['propeller diameter/pitch/mass', 'motor/prop thrust and torque curves',
                                   'ESC/motor response', 'measured all-up mass and COM verification'],
              'flight_dynamics_qualified': False, 'mesh_deflection_mm': .25}
    (output/'cad-properties.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('CAD assembly mass kg', total, 'COM FLU m', com.tolist(), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--mount', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import_cad(args.source, args.mount, args.output)
