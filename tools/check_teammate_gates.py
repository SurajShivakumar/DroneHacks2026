"""Independent ordered gate-plane checks on a saved trajectory; no flight I/O."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', required=True, type=Path)
    parser.add_argument('--trajectory', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.repo.resolve()))
    from drl.config import load_config
    cfg, course, gates = load_config()
    states = np.loadtxt(args.trajectory, delimiter=',', skiprows=1)
    p, start, result = states[:, 1:4], 0, []
    for k, name in enumerate(gates.names):
        if gates.is_hover[k]:
            continue
        d = (p-gates.centre[k]) @ gates.normal[k]
        crossings = np.where((d[:-1] < 0) & (d[1:] >= 0))[0]
        crossings = crossings[crossings >= start]
        if not len(crossings):
            result.append({'gate': name, 'ordered_crossing_found': False})
            break
        index = int(crossings[0])
        f = -d[index]/(d[index+1]-d[index])
        point = p[index]+f*(p[index+1]-p[index])
        lateral = abs((point-gates.centre[k]) @ gates.lateral[k])
        vertical = abs(point[2]-gates.centre[k, 2])
        record = {'gate': name, 'ordered_crossing_found': True,
                  'crossing_s': float(states[index, 0]+f*(states[index+1, 0]-states[index, 0])),
                  'center_inside_rectangle': bool(lateral <= gates.half_w[k] and vertical <= gates.half_h[k]),
                  'center_lateral_margin_m': float(gates.half_w[k]-lateral),
                  'center_vertical_margin_m': float(gates.half_h[k]-vertical)}
        # Rectangle support is exact for their cylindrical envelope at crossing
        # orientation. Tube openings are elliptical: use the separate mesh check.
        if name == 'window':
            q = states[index, 7:11]+f*(states[index+1, 7:11]-states[index, 7:11])
            q /= np.linalg.norm(q)
            w, x, y, z = q
            body_z = np.array([2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)])
            r, h = cfg['drone']['collision_radius'], cfg['drone']['collision_half_height']
            def support(axis):
                dot = float(axis @ body_z)
                return r*np.sqrt(max(0., 1-dot*dot))+h*abs(dot)
            record['tilted_envelope_lateral_margin_m'] = float(gates.half_w[k]-lateral-support(gates.lateral[k]))
            record['tilted_envelope_vertical_margin_m'] = float(gates.half_h[k]-vertical-support(np.array([0, 0, 1.])))
        result.append(record)
        start = index+1
    report = {'kind': 'ordered_teammate_gate_check_not_official_grade', 'gates': result,
              'all_nine_plane_gates_crossed_in_order': len(result) == 9 and all(r.get('ordered_crossing_found') and r.get('center_inside_rectangle') for r in result),
              'landing_and_disarm_checked': False, 'hardware_footprint_qualified': False}
    with args.output.open('x', encoding='utf-8') as output:
        json.dump(report, output, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
