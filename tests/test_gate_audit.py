import numpy as np
import pytest
from dronebench.gate_audit import audit, cylinder_support


def trajectory(end=1., lateral=0., height=0.):
    t=np.arange(101)*.01
    rows=np.zeros((101,8)); rows[:,0]=t; rows[:,1]=np.linspace(-1,end,101)
    rows[:,2]=lateral; rows[:,3]=height; rows[:,4]=1
    return rows


GATE=dict(name='opening',center=[0,0,0],normal=[1,0,0],half_width=.5,half_height=.3)


def test_center_crossing_does_not_mean_trailing_envelope_clear():
    result=audit(trajectory(end=.1),[GATE],radius=.28,half_height=.13)
    assert not result['ordered_planes_cleared'] and not result['events']
    result=audit(trajectory(),[GATE],radius=.28,half_height=.13)
    assert result['ordered_planes_cleared']
    assert result['events'][0]['source_s']>.5


@pytest.mark.parametrize('lateral,height',[(.3,0),(0,.2)])
def test_center_fits_but_aircraft_does_not(lateral,height):
    result=audit(trajectory(lateral=lateral,height=height),[GATE],radius=.28,half_height=.13)
    assert not result['ordered_planes_cleared']
    assert result['failed_gate']=='opening'


def test_reversed_pass_and_sparse_sampling_do_not_pass():
    assert not audit(trajectory()[::-1]*[ -1,1,1,1,1,1,1,1],[GATE],radius=.28,half_height=.13)['ordered_planes_cleared']
    with pytest.raises(ValueError,match='densely sampled'):
        audit(trajectory()[::10],[GATE],radius=.28,half_height=.13)


def test_tilted_envelope_support_accounts_for_height():
    assert cylinder_support(np.array([1,0,0]),np.array([1,0,0]),.28,.13)==.13
    assert cylinder_support(np.array([0,0,1]),np.array([1,0,0]),.28,.13)==.28


def test_missing_aperture_or_teleported_motion_cannot_pass():
    with pytest.raises(ValueError,match='positive aperture'):
        audit(trajectory(),[dict(GATE,half_width=float('nan'))],radius=.28,half_height=.13)
    rows=trajectory(); rows[50:,1]+=10
    with pytest.raises(ValueError,match='spatially dense'):
        audit(rows,[GATE],radius=.28,half_height=.13)


def test_diagonal_plane_uses_cylinder_section_not_full_projection():
    # Follow an L around a corner centered on a diagonal route marker.
    rows=trajectory(); half=len(rows)//2
    rows[:half,1]=0; rows[:half,2]=np.linspace(-1,-.02,half)
    rows[half:,1]=np.linspace(0,1,len(rows)-half); rows[half:,2]=0
    gate=dict(GATE,normal=[2**-.5,2**-.5,0],half_width=.4)
    assert audit(rows,[gate],radius=.28,half_height=.13)['ordered_planes_cleared']


def test_overlapping_plane_slabs_still_require_ordered_trailing_clearance():
    gates=[GATE,dict(GATE,name='second',center=[.4,0,0])]
    result=audit(trajectory(),gates,radius=.28,half_height=.13)
    assert result['ordered_planes_cleared']
    assert result['events'][1]['source_s']>result['events'][0]['source_s']
    assert not audit(trajectory(),gates[::-1],radius=.28,half_height=.13)['ordered_planes_cleared']
