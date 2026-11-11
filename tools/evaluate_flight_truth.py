"""Independent offline evaluation of a flown run from simulator truth poses.

Truth poses are read ONLY here, after the flight, for grading; they never
reach the estimator or controller. Checks: every envelope contact with the
course mesh between arm and disarm (grouped by course part, floor separated
from structure), and landing position versus the published pad. Ordered gate
crossings are checked separately with tools/audit_scene_gates.py on the same
truth CSV. This is not an official competition score.
"""
import argparse
from collections import defaultdict
from hashlib import sha256
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('scene', 'truth', 'mission', 'contract', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--pad-center', type=float, nargs=2, default=(-2.042, -0.792))
    p.add_argument('--pad-outer-radius', type=float, default=0.762)
    a = p.parse_args()
    mission = json.loads(a.mission.read_text())
    events = {e['kind']: e for e in mission['events']}
    arm = events['armed_observed']['sim_s']
    disarm = events['landed_disarmed']['disarmed_source_sim_s']
    airborne = (events['navigation_started']['sim_s'], events['request_land']['sim_s'])
    contract = json.loads(a.contract.read_text())
    s = np.genfromtxt(a.truth, delimiter=',', names=True)
    states = np.stack([s[k] for k in ('t', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz')], axis=-1)

    tree = ET.parse(a.scene)
    body = tree.find("worldbody/body[@name='sky_beta']")
    for geom in body.findall('geom'):
        geom.set('contype', '0'); geom.set('conaffinity', '0')
    ET.SubElement(body, 'geom', name='envelope', type='cylinder',
                  size=f"{contract['collision_radius_m']} {contract['collision_half_height_m']}",
                  contype='1', conaffinity='1', mass='0')
    model = mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(), encoding='unicode'))
    data = mujoco.MjData(model)
    address = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'sky_beta_free')]
    envelope = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'envelope')
    groups = defaultdict(lambda: {'samples': 0, 'first_s': None, 'last_s': None, 'max_penetration_m': 0.,
                                  'airborne_samples': 0})
    dt = .005
    for t in np.arange(arm, disarm+1e-9, dt):
        pose = np.array([np.interp(t, states[:, 0], states[:, i]) for i in range(1, 8)])
        pose[3:] /= np.linalg.norm(pose[3:])
        data.qpos[address:address+7] = pose
        mujoco.mj_forward(model, data)
        hit = {}
        for c in data.contact[:data.ncon]:
            if envelope in (c.geom1, c.geom2) and c.dist < -1e-5:
                other = c.geom2 if c.geom1 == envelope else c.geom1
                name = re.sub(r'_c\d+$', '', mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other) or str(other))
                hit[name] = max(hit.get(name, 0.), float(-c.dist))
        for name, depth in hit.items():
            g = groups[name]
            g['samples'] += 1
            g['first_s'] = float(t) if g['first_s'] is None else g['first_s']
            g['last_s'] = float(t)
            g['max_penetration_m'] = max(g['max_penetration_m'], depth)
            g['airborne_samples'] += int(airborne[0] <= t <= airborne[1])
    structure = {k: v for k, v in groups.items() if 'Floor' not in k}
    floor = {k: v for k, v in groups.items() if 'Floor' in k}

    final = states[np.searchsorted(states[:, 0], disarm)-1]
    offset = float(np.hypot(final[1]-a.pad_center[0], final[2]-a.pad_center[1]))
    radius = contract['collision_radius_m']
    report = {
        'kind': 'independent_truth_evaluation_not_official_score',
        'input_sha256': {str(x): sha256(x.read_bytes()).hexdigest() for x in (a.scene, a.truth, a.mission, a.contract)},
        'arm_sim_s': arm, 'disarm_sim_s': disarm, 'arm_to_disarm_s': disarm-arm,
        'airborne_navigation_interval_sim_s': list(airborne),
        'envelope': {'radius_m': radius, 'half_height_m': contract['collision_half_height_m']},
        'structure_contacts': structure, 'structure_contact_free': not structure,
        'floor_contacts': floor,
        'floor_contact_during_navigation': any(v['airborne_samples'] for v in floor.values()),
        'landing': {'final_xy_m': final[1:3].tolist(), 'pad_center_xy_m': list(a.pad_center),
                    'center_offset_m': offset, 'pad_outer_radius_m': a.pad_outer_radius,
                    'center_inside_outer_ring': offset <= a.pad_outer_radius,
                    'whole_envelope_inside_outer_ring': offset+radius <= a.pad_outer_radius},
        'gate_events_onboard_estimator': mission['gate_events'],
    }
    a.output.write_text(json.dumps(report, indent=2))
    print(json.dumps({k: report[k] for k in ('arm_to_disarm_s', 'structure_contact_free', 'structure_contacts',
                                              'floor_contact_during_navigation', 'landing')}, indent=2))



def tilt_aware_gate_audit(poses, gates, *, radius, half_height):
    """Same ordered-plane replay as dronebench.gate_audit, but with a tilt-aware
    plane-section bound instead of the full-radius fallback used for any
    non-vertical attitude. For body axis a, plane normal n, lateral l and signed
    centre distance d, the section's lateral half-extent is bounded by
      sqrt(R^2 - max(0, |d| - H|n.a|)^2) + H|l.a| + R*(1 - a_z)
    (exact for a vertical body; the last term over-covers the small
    non-orthogonality of the tilted disk). Height uses the unchanged bound.
    Reported beside, never instead of, the strict auditor.
    """
    from dronebench.gate_audit import cylinder_support
    w, x, y, z = poses[:, 4:8].T
    axes = np.stack([2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)], axis=1)
    events, earliest, worst = [], poses[0, 0], {}
    for gate in gates:
        c = np.asarray(gate['center'], float); n = np.asarray(gate['normal'], float)
        l = np.cross([0., 0., 1.], n)
        dist = (poses[:, 1:4]-c)@n
        sup = np.array([cylinder_support(n, b, radius, half_height) for b in axes])
        up = np.flatnonzero((poses[:, 0] <= earliest) & (dist < -sup))
        if not len(up):
            return {'ordered_planes_cleared': False, 'events': events, 'next_gate': gate['name']}
        traversing = invalid = done = False
        margin_w = margin_h = np.inf
        for i in range(int(up[-1]), len(poses)):
            a, d = axes[i], dist[i]
            if d >= -sup[i]:
                traversing = True
            if traversing and abs(d) <= sup[i]:
                eff = max(0., abs(d)-half_height*abs(n@a))
                ext = np.sqrt(max(0., radius**2-eff**2))+half_height*abs(l@a)+radius*(1-a[2])
                width = abs((poses[i, 1:4]-c)@l)+ext
                height = abs(poses[i, 3]-c[2])+cylinder_support(np.array([0, 0, 1.]), a, radius, half_height)
                margin_w = min(margin_w, gate['half_width']-width)
                margin_h = min(margin_h, gate['half_height']-height)
                invalid |= width > gate['half_width'] or height > gate['half_height']
            if traversing and d >= sup[i]:
                events.append({'gate': gate['name'], 'source_s': float(poses[i, 0]),
                               'full_envelope_clearance': not invalid,
                               'min_lateral_margin_m': float(margin_w), 'min_vertical_margin_m': float(margin_h)})
                if invalid:
                    return {'ordered_planes_cleared': False, 'events': events, 'failed_gate': gate['name']}
                earliest, done = poses[i, 0], True
                break
        if not done:
            return {'ordered_planes_cleared': False, 'events': events, 'next_gate': gate['name']}
    return {'ordered_planes_cleared': True, 'events': events}


def gates_main():
    p = argparse.ArgumentParser(description='tilt-aware ordered gate audit on truth poses')
    for name in ('repo', 'truth', 'contract', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1])); sys.path.insert(0, str(a.repo.resolve()))
    from drl.config import load_config
    _, _, r = load_config(str(a.repo.resolve()/'config/course_v3.yaml'))
    gates = [dict(name=r.names[i], center=r.centre[i].tolist(), normal=r.normal[i].tolist(),
                  half_width=float(r.half_w[i]), half_height=float(r.half_h[i]))
             for i in range(len(r)) if not r.is_hover[i]]
    c = json.loads(a.contract.read_text())
    s = np.genfromtxt(a.truth, delimiter=',', names=True)
    poses = np.stack([s[k] for k in ('t', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz')], axis=1)
    poses[:, 4:8] /= np.linalg.norm(poses[:, 4:8], axis=1)[:, None]
    w, x, y, z = poses[:, 4:8].T
    tilt = np.degrees(np.arccos(np.clip(1-2*(x*x+y*y), -1, 1)))
    result = tilt_aware_gate_audit(poses, gates, radius=c['collision_radius_m'], half_height=c['collision_half_height_m'])
    result.update(kind='tilt_aware_truth_gate_audit_not_official_score', max_tilt_deg=float(tilt.max()),
                  input_sha256={str(f): sha256(f.read_bytes()).hexdigest() for f in (a.truth, a.contract, Path(__file__))})
    a.output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=1))


if __name__ == '__main__':
    import sys as _s
    if len(_s.argv) > 1 and _s.argv[1] == 'gates':
        _s.argv.pop(1); gates_main()
    else:
        main()
