from copy import deepcopy
import numpy as np
import pytest
from tools.prepare_return_reference import return_waypoints,sample_return


def fixture():
    groups={'Launch mark':dict(min=[-2.6,.8,0],max=[-2.4,1.,.01]),
            'Landing pad 5 ft':dict(min=[-2.8,-1.6,0],max=[-1.2,0.,.01]),
            'Floor 20 x 20 ft':dict(min=[-3,-3,-.01],max=[3,3,0]),
            'wall':dict(min=[-3,.4,0],max=[3,.5,1.73]),
            'Net roof':dict(min=[-3,-3,3],max=[3,3,3.01]),
            'Net side west':dict(min=[-3.01,-3,0],max=[-3,3,3]),
            'Net side east':dict(min=[3,-3,0],max=[3.01,3,3]),
            'Net side south':dict(min=[-3,-3.01,0],max=[3,-3,3]),
            'Net side north':dict(min=[-3,3,0],max=[3,3.01,3])}
    return dict(launch_xy_m=[-2.5,.9],pad_center_xy_m=[-2,-.8],takeoff_agl_m=.9),dict(geometry=dict(groups=groups)),dict(collision_radius_m=.28,collision_half_height_m=.13)


def test_whole_envelope_separates_barrier_and_roof_without_assuming_columns_clear():
    s,m,c=fixture();points,meta=return_waypoints(s,m,c)
    assert meta['transit_bottom_with_margin_m']>1.73
    assert meta['transit_top_with_margin_m']<3
    assert not meta['vertical_columns_independently_screened']
    np.testing.assert_allclose(points[0],[-2,-.8,.9]);np.testing.assert_allclose(points[-1],[-2.5,.9,.9])
    bad=deepcopy(m);bad['geometry']['groups']['Net roof']['min'][2]=1.95
    with pytest.raises(ValueError):return_waypoints(s,bad,c)
    bad=deepcopy(s);bad['launch_xy_m']=[0,0]
    with pytest.raises(ValueError):return_waypoints(bad,m,c)


def test_airborne_return_brakes_all_three_segments_and_bounds_vertical_motion():
    s,m,c=fixture();points,_=return_waypoints(s,m,c);rows,legs=sample_return(points)
    assert len(legs)==3
    assert np.diff(rows[:,0]).max()<=.02+1e-9
    assert np.abs(rows[:,6]).max()<=.25+1e-9
    assert np.linalg.norm(rows[:,4:6],axis=1).max()<=.4+1e-9
    dt=np.diff(rows[:,0]);accel=np.diff(rows[:,4:7],axis=0)/dt[:,None]
    assert np.linalg.norm(accel,axis=1).max()<=.25+1e-4
    mid=(rows[1:,0]+rows[:-1,0])/2
    jerk=np.diff(accel,axis=0)/np.diff(mid)[:,None]
    assert np.linalg.norm(jerk,axis=1).max()<=.5+1e-3
    for point in points:
        indices=np.flatnonzero(np.linalg.norm(rows[:,1:4]-point,axis=1)<1e-9)
        assert len(indices)>0 and np.linalg.norm(rows[indices[-1],4:7])<1e-9
    np.testing.assert_allclose(rows[0,1:4],points[0]);np.testing.assert_allclose(rows[-1,1:4],points[-1])
    bad=points.copy();bad[1,0]+=.1
    with pytest.raises(ValueError):sample_return(bad)
