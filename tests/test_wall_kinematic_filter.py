import numpy as np
import pytest
from dronebench.wall_kinematic_filter import WallKinematicFilter


def test_repeated_velocity_does_not_invent_tangential_position_observability():
    f=WallKinematicFilter([0,0],[0,0])
    for _ in range(100):
        f.predict(.1); f.observe_velocity([0,0],np.eye(2)*.001)
        f.observe_wall([1,0],0,.001)
    assert np.sqrt(f.covariance[0,0])<.1
    assert np.sqrt(f.covariance[1,1])>.3
    assert f.position[1]==0


def test_wall_updates_can_distinguish_velocity_bias_from_real_motion():
    f=WallKinematicFilter([0,0],[.1,0])
    for _ in range(200):
        f.predict(.1); f.observe_velocity([.1,0],np.eye(2)*.001)
        f.observe_wall([1,0],0,.001)
    assert abs(f.position[0])<.03
    assert abs(f.velocity[0])<.02
    assert abs(f.mean[4]-.1)<.02
    assert np.linalg.eigvalsh(f.covariance).min()>0


def test_source_gaps_and_inconsistent_factors_reject():
    f=WallKinematicFilter([0,0],[0,0])
    with pytest.raises(ValueError):f.predict(1.)
    assert not f.observe_wall([1,0],10,.001)
    np.testing.assert_equal(f.position,[0,0])
    assert f.last_observations['wall']['accepted'] is False
    assert f.last_observations['wall']['normalized_squared_innovation']>f.last_observations['wall']['gate']


def test_vio_position_bias_cannot_be_mistaken_for_a_precise_absolute_fix():
    f=WallKinematicFilter([0,0],[0,0])
    for _ in range(1000):f.observe_vio_position([0,0],np.eye(2)*.001)
    assert np.sqrt(f.covariance[0,0])>.034
    for _ in range(100):
        f.predict(.1); f.observe_vio_position([.1,0],np.eye(2)*.001)
        f.observe_velocity([0,0],np.eye(2)*.001); f.observe_wall([1,0],0,.001)
    assert abs(f.position[0])<.03
    assert abs(f.mean[6]-.1)<.03
