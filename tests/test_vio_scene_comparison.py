"""Independent scene comparison must expose orientation drift after alignment."""
import json
from math import radians, sin, cos
import numpy as np
from tools.compare_vio_scene import compare


def test_orientation_drift_is_not_removed_by_initial_alignment(tmp_path):
    poses = tmp_path / 'poses.csv'
    poses.write_text('t,x,y,z,qw,qx,qy,qz\n0,0,0,0,1,0,0,0\n1,0,0,0,1,0,0,0\n')
    contract = tmp_path / 'contract.json'
    contract.write_text(json.dumps({'body_from_imu': np.eye(4).tolist()}))
    rows = []
    # Initial yaw offset is aligned once; the added ten-degree drift must remain.
    for stamp, yaw in [(0, 30), (1, 40)]:
        rows.append({'topic': '/vio_shadow/odomimu', 'source_s': stamp,
                     'frame_id': 'global', 'child_frame_id': 'imu',
                     'position': dict(x=3, y=4, z=0),
                     'orientation': dict(w=cos(radians(yaw)/2), x=0, y=0,
                                         z=sin(radians(yaw)/2))})
    odometry = tmp_path / 'odometry.jsonl'
    odometry.write_text('\n'.join(json.dumps(row) for row in rows))
    result = compare(poses, odometry, contract, tmp_path / 'result.json')
    assert result['position_error_rmse_m'] == 0
    assert abs(result['maximum_attitude_error_rad'] - radians(10)) < 1e-10
    assert result['maximum_attitude_error_source_s'] == 1
