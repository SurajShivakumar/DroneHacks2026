import numpy as np
from tools.prepare_robust_corner_reference import corner_segments,sample


def test_local_curves_pass_centre_with_normal_tangent_and_stop_at_every_splice():
    normal=np.array([1,1,0])/np.sqrt(2)
    gates=[dict(name='corner',center=[0,0,.9],normal=normal.tolist())]
    pieces,corners=corner_segments([[0,-1],[0,0],[1,0]],gates,splice=.45,handle=.25)
    pre=next(P for P,k,n in pieces if k=='gate_approach');post=next(P for P,k,n in pieces if k=='gate_departure')
    np.testing.assert_allclose(pre[-1]-pre[-2],normal[:2]*.25)
    np.testing.assert_allclose(post[1]-post[0],normal[:2]*.25)
    rows,reports=sample(pieces)
    assert np.isfinite(rows).all() and np.max(np.diff(rows[:,0]))<=.015+1e-10
    for P,_,_ in pieces:
        at=np.flatnonzero(np.linalg.norm(rows[:,1:3]-P[-1],axis=1)<1e-10)
        # The next curve can start within positional roundoff of the stopped
        # endpoint while already having a tiny nonzero velocity; require the
        # actual explicit rest samples rather than the last near-position row.
        assert len(at) and np.any(np.linalg.norm(rows[at,4:7],axis=1)<1e-12)
    assert max(r['peak_speed_mps'] for r in reports)<=.267
    assert max(r['peak_acceleration_mps2'] for r in reports)<=.111
    assert max(r['peak_jerk_mps3'] for r in reports)<=.148
    dt=np.diff(rows[:,0]);acceleration=np.diff(rows[:,4:7],axis=0)/dt[:,None]
    middle=(rows[1:,0]+rows[:-1,0])/2
    jerk=np.diff(acceleration,axis=0)/np.diff(middle)[:,None]
    assert np.linalg.norm(acceleration,axis=1).max()<=.111+1e-4
    assert np.linalg.norm(jerk,axis=1).max()<=.148+1e-3
