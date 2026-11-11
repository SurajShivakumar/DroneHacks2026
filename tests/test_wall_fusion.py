import numpy as np
import pytest
from dronebench.wall_fusion import WallPositionFilter


def test_single_wall_does_not_create_tangential_information():
    f=WallPositionFilter([1.,2.],np.eye(2)*.01)
    assert f.update([1,0],1.1,.0025)
    assert f.position[0]>1 and f.position[1]==2
    assert f.covariance[0,0]<.01 and f.covariance[1,1]==.01
    f.predict([.1,.2],1.)
    assert f.covariance[1,1]>.01


def test_inconsistent_observation_rejects_without_changing_state():
    f=WallPositionFilter([0,0],np.eye(2)*.01)
    assert not f.update([1,0],10,.001)
    np.testing.assert_equal(f.position,[0,0])
    np.testing.assert_equal(f.covariance,np.eye(2)*.01)
    with pytest.raises(ValueError):f.predict([0,np.nan],1.)
    with pytest.raises(ValueError):f.predict([0,0],-1.)


def test_correlated_repetition_cannot_erase_systematic_floor():
    f=WallPositionFilter([0,0],np.eye(2)*.01)
    for _ in range(100):f.update([1,0],0,.0001)
    assert f.covariance[0,0]>=.03**2
