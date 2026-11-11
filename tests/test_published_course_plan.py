import numpy as np
from tools.plan_published_course import sampled_reference
from dronebench.teammate_policy import ReferenceTrajectory


def test_stop_turn_go_keeps_turns_stationary_and_bounds_motion_derivatives(tmp_path):
    rows=sampled_reference([(0,0),(1,0),(1,1)])
    dt=np.diff(rows[:,0]);velocity=rows[:,4:7]
    assert np.all(dt>0) and np.max(np.linalg.norm(velocity,axis=1))<=.250001
    acceleration=np.diff(velocity,axis=0)/dt[:,None]
    assert np.max(np.linalg.norm(acceleration,axis=1))<.151
    yaws=np.unwrap(2*np.arctan2(rows[:,10],rows[:,7]))
    turns=np.abs(np.diff(yaws))>1e-6
    assert np.all(np.linalg.norm(np.diff(rows[:,1:4],axis=0)[turns],axis=1)<1e-12)
    assert np.max(np.abs(np.diff(yaws)/dt))<.150001
    path=tmp_path/'trajectory.csv'
    np.savetxt(path,rows,delimiter=',',header='t,x,y,z,vx,vy,vz,qw,qx,qy,qz',comments='')
    reference=ReferenceTrajectory(path,time_scale=1.)
    assert np.allclose(reference.sample(reference.duration_s)[0],[1,1,.9])
