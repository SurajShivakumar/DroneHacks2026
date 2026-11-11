import numpy as np
import pytest
from tools.prepare_efficient_reference import MotionBudget,sample_braked_route


def test_direction_changes_brake_without_forcing_camera_yaw_and_bound_derivatives():
    points=[(0,0),(2,0),(2,1),(1,1)]
    budget=MotionBudget();rows,legs=sample_braked_route(points,budget=budget)
    dt=np.diff(rows[:,0]);velocity=rows[:,4:7]
    assert np.all(dt>0) and dt.max()<=budget.dt_s+1e-9
    assert np.linalg.norm(velocity,axis=1).max()<=budget.cruise_mps+1e-9
    acceleration=np.diff(velocity,axis=0)/dt[:,None]
    assert np.linalg.norm(acceleration,axis=1).max()<=budget.acceleration_mps2+1e-4
    midpoint=(rows[1:,0]+rows[:-1,0])/2
    jerk=np.diff(acceleration,axis=0)/np.diff(midpoint)[:,None]
    assert np.linalg.norm(jerk,axis=1).max()<=budget.jerk_mps3+1e-3
    assert np.allclose(rows[:,7:],rows[0,7:])
    for point in points:
        at=np.flatnonzero(np.linalg.norm(rows[:,1:3]-point,axis=1)<1e-9)
        assert len(at)>0 and np.linalg.norm(velocity[at[-1]])<1e-9
    assert all(leg['peak_acceleration_mps2']<=budget.acceleration_mps2+1e-12 for leg in legs)


def test_invalid_inputs_do_not_create_a_candidate():
    for kwargs in ({'cruise_mps':0},{'dt_s':.1},{'fixed_yaw_rad':np.nan},{'dwell_s':-1}):
        with pytest.raises(ValueError):MotionBudget(**kwargs)
    for points in ([(0,0)],[(0,0),(0,0)],[(0,0),(1,np.nan)]):
        with pytest.raises(ValueError):sample_braked_route(points)
