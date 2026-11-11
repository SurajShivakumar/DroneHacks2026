from math import pi, sqrt
import numpy as np
import pytest

from dronebench.teammate_policy import CourseFrame, ReferenceTrajectory, TrackingClock, floor_height_from_range


def test_course_alignment_uses_actual_pose_and_range_not_ned_zero_or_nose_north():
    frame = CourseFrame.from_launch(position_ned=[4., -2., .08], heading_ned=.3,
        launch_xy=[-2.515, .914], body_agl=.23, physical_heading_in_course=pi/2, placement_reviewed=True)
    assert np.allclose(frame.to_course([4., -2., .08]), [-2.515, .914, .23])
    assert np.allclose(frame.to_ned(frame.to_course([5., 3., -1.])), [5., 3., -1.])
    assert frame.north_heading_in_course == pi/2+.3
    forward_ned = np.array([np.cos(.3), np.sin(.3), 0.])
    assert np.allclose(frame.ned_to_course @ forward_ned, [0., 1., 0.])
    with pytest.raises(ValueError, match='reviewed'):
        CourseFrame.from_launch(position_ned=[0]*3, heading_ned=0, launch_xy=[0, 0],
                                body_agl=.23, physical_heading_in_course=0)


def test_floor_calibration_respects_sensor_mount_and_attitude():
    assert floor_height_from_range(.18, [1, 0, 0, 0], [0, 0, .05], [0, 0, 1]) == pytest.approx(.23)
    with pytest.raises(ValueError, match='unit'):
        floor_height_from_range(.18, [0, 0, 0, 0], [0, 0, .05], [0, 0, 1])
    with pytest.raises(ValueError, match='downward'):
        floor_height_from_range(.18, [sqrt(.5), sqrt(.5), 0, 0], [0, 0, .05], [0, 0, 1])


def test_return_reverses_reference_velocity_without_reversing_the_ppo(tmp_path):
    path = tmp_path/'trajectory.csv'
    values = np.zeros((2, 15)); values[:, 0] = [0, 2]; values[:, 1] = [0, 4]; values[:, 4] = 2; values[:,7]=1
    np.savetxt(path, values, delimiter=',', header='fixture', comments='')
    out = ReferenceTrajectory(path, time_scale=4)
    back = ReferenceTrajectory(path, time_scale=4, reverse=True)
    assert np.allclose(out.sample(2)[0], [1, 0, 0])
    assert np.allclose(back.sample(2)[0], [3, 0, 0])
    assert np.allclose(back.sample(2)[1], [-.5, 0, 0])
    assert back.duration_s == 8


def test_reference_pauses_for_braking_tracking_and_camera_heading():
    clock=TrackingClock()
    clock.step(now_sim=1.,error_m=0.,desired_yaw=pi/2,actual_yaw=0.)
    elapsed,yaw=clock.step(now_sim=1.1,error_m=0.,desired_yaw=pi/2,actual_yaw=0.)
    assert elapsed==0 and yaw==pytest.approx(.05)
    clock.step(now_sim=1.2,error_m=.2,desired_yaw=0.,actual_yaw=0.)
    assert clock.elapsed_s==0
    elapsed,_=clock.step(now_sim=1.3,error_m=.05,desired_yaw=0.,actual_yaw=0.)
    assert elapsed==pytest.approx(.1)
    with pytest.raises(ValueError,match='deviation'):
        clock.step(now_sim=1.4,error_m=.3,desired_yaw=0.,actual_yaw=0.)


def test_narrow_course_tracking_limits_cannot_silently_use_proxy_tolerance():
    clock=TrackingClock(max_error_m=.04,pause_error_m=.025)
    clock.step(now_sim=1.,error_m=.03,desired_yaw=0.,actual_yaw=0.)
    clock.step(now_sim=1.1,error_m=.03,desired_yaw=0.,actual_yaw=0.)
    assert clock.elapsed_s==0
    with pytest.raises(ValueError,match='deviation'):
        clock.step(now_sim=1.2,error_m=.041,desired_yaw=0.,actual_yaw=0.)
    with pytest.raises(ValueError,match='limits'):
        TrackingClock(max_error_m=.04,pause_error_m=.08)


def test_smoothed_reference_brakes_without_a_velocity_discontinuity():
    clock=TrackingClock(max_error_m=.04,pause_error_m=.025,rate_slew_per_s=.5)
    for t in np.arange(0,2.11,.1):
        clock.step(now_sim=float(t),error_m=0.,desired_yaw=0.,actual_yaw=0.)
    assert clock.speed_fraction==pytest.approx(1.)
    previous=clock.elapsed_s
    elapsed,_=clock.step(now_sim=2.2,error_m=.03,desired_yaw=1.,actual_yaw=0.)
    assert clock.speed_fraction==pytest.approx(.95)
    assert elapsed-previous==pytest.approx(.095)
    with pytest.raises(ValueError,match='deviation'):
        clock.step(now_sim=2.3,error_m=.041,desired_yaw=0.,actual_yaw=0.)


def test_local_projection_distinguishes_path_clearance_from_time_target_lag(tmp_path):
    states=np.zeros((5,15));states[:,0]=np.arange(5);states[:,1]=np.arange(5);states[:,7]=1
    path=tmp_path/'reference.csv';np.savetxt(path,states,delimiter=',',header='t,x,y,z,vx,vy,vz,qw,qx,qy,qz,a,b,c,d',comments='')
    reference=ReferenceTrajectory(path,time_scale=1.)
    phase,point,error=reference.nearest([1.03,.02,0],around_s=1.)
    assert phase==pytest.approx(1.03) and point==pytest.approx([1.03,0,0]) and error==pytest.approx(.02)
    phase,_,error=reference.nearest([3.5,0,0],around_s=0.,ahead_s=1.)
    assert phase==1 and error==2.5  # Never jump ahead to a later nearby segment.
    clock=TrackingClock(max_error_m=.04,pause_error_m=.025,rate_slew_per_s=.5)
    clock.step(now_sim=1.,error_m=.02,desired_yaw=0.,actual_yaw=0.,progress_floor_s=1.03)
    assert clock.elapsed_s==pytest.approx(1.03)
    with pytest.raises(ValueError,match='deviation'):
        clock.step(now_sim=1.1,error_m=.041,desired_yaw=0.,actual_yaw=0.,progress_floor_s=1.05)


def test_conservative_camera_yaw_rate_is_an_actual_command_bound():
    clock=TrackingClock(max_yaw_rate=.2)
    clock.step(now_sim=1.,error_m=0.,desired_yaw=1.,actual_yaw=0.)
    _,yaw=clock.step(now_sim=1.1,error_m=0.,desired_yaw=1.,actual_yaw=0.)
    assert yaw==pytest.approx(.02)


def test_lag_pause_default_matches_original_and_separates_lag_from_cross_track():
    def run(**kw):
        clock=TrackingClock(max_error_m=.04,pause_error_m=.025,rate_slew_per_s=.5,**kw)
        for t in np.arange(0,3.01,.1):
            clock.step(now_sim=float(t),error_m=.005,desired_yaw=0.,actual_yaw=0.,lag_error_m=.02)
        return clock.speed_fraction
    original=run()
    assert original==pytest.approx(run(lag_pause_m=.025))
    assert original<.4                      # 2 cm lag throttles the original clock
    assert run(lag_pause_m=.10)==pytest.approx(1.)
    clock=TrackingClock(max_error_m=.04,pause_error_m=.025,rate_slew_per_s=.5,lag_pause_m=.10)
    with pytest.raises(ValueError,match='deviation'):   # cross-track abort unchanged
        clock.step(now_sim=0.,error_m=.041,desired_yaw=0.,actual_yaw=0.,lag_error_m=0.)
    with pytest.raises(ValueError,match='lag pause'):
        TrackingClock(max_error_m=.04,pause_error_m=.025,lag_pause_m=.01)
