"""Safety boundary regressions for the live feature watchdog."""
from dronebench.vio_quality import VioFeatureMonitor


def observed_monitor():
    monitor = VioFeatureMonitor()
    monitor.observe(source_s=14.637497658000001, received_wall_s=100., feature_count=12)
    return monitor


def test_equal_stamp_float_roundoff_does_not_latch_fault():
    monitor = observed_monitor()
    result = monitor.status(now_sim_s=14.637497658, now_wall_s=100.)
    assert result['state'] == 'WAITING'
    assert monitor.fault is None


def test_actual_future_stamp_latches_fault():
    monitor = observed_monitor()
    result = monitor.status(now_sim_s=14.63, now_wall_s=100.)
    assert result['state'] == 'FAULT'
    assert 'ahead' in result['reason']


def test_stale_source_still_faults_despite_fresh_arrival():
    monitor = observed_monitor()
    result = monitor.status(now_sim_s=15.2, now_wall_s=100.)
    assert result['state'] == 'FAULT'
    assert 'source age' in result['reason']


def test_stale_arrival_still_faults_despite_current_source():
    monitor = observed_monitor()
    result = monitor.status(now_sim_s=14.637497658, now_wall_s=100.6)
    assert result['state'] == 'FAULT'
    assert 'arrival' in result['reason']


def test_ready_requires_continuous_tracks_and_persistent_loss_faults():
    monitor = VioFeatureMonitor()
    for step in range(32):
        stamp = step / 10
        monitor.observe(source_s=stamp, received_wall_s=stamp, feature_count=12)
        result = monitor.status(now_sim_s=stamp, now_wall_s=stamp)
    assert result['state'] == 'READY'
    for step in range(32, 59):
        stamp = step / 10
        monitor.observe(source_s=stamp, received_wall_s=stamp, feature_count=0)
        result = monitor.status(now_sim_s=stamp, now_wall_s=stamp)
    assert result['state'] == 'FAULT'
    assert 'below threshold' in result['reason']


def test_one_physics_step_clock_lag_is_not_a_future_stamp():
    # v35: stamp 124.45831342 vs cached ROS clock 124.454146754 (one 240 Hz step).
    monitor = VioFeatureMonitor()
    monitor.observe(source_s=124.45831342, received_wall_s=100., feature_count=47)
    result = monitor.status(now_sim_s=124.454146754, now_wall_s=100.)
    assert result['state'] != 'FAULT'
    assert monitor.fault is None


def test_two_physics_step_lead_still_faults():
    monitor = VioFeatureMonitor()
    monitor.observe(source_s=124.45831342, received_wall_s=100., feature_count=47)
    result = monitor.status(now_sim_s=124.45831342 - 2/240, now_wall_s=100.)
    assert result['state'] == 'FAULT'
    assert 'ahead' in result['reason']
