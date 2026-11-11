from copy import deepcopy
import pytest
from tools.summarize_stereo_shadow import summarize


def row(t):
    return dict(source_s=t,right_source_s=t,support_source_s={'vio':t-.02,'range':t-.03},
        processing_wall_s=.01,accepted_wall_factors=0,sensor={'valid_points':20})


def test_processed_frame_does_not_claim_position_observability():
    report=summarize([row(1),row(1.1)])
    assert report['frames_without_wall_factor']==2
    assert report['published_flight_odometry'] is False
    assert report['processing_wall_ms']['median']==10


def test_bad_source_timing_is_not_an_audit_pass():
    rows=[row(1),row(1.1)]
    for key,value in [('vio',1.1001),('range',.9)]:
        bad=deepcopy(rows);bad[1]['support_source_s'][key]=value
        with pytest.raises(ValueError):summarize(bad)
    with pytest.raises(ValueError):summarize([row(1),row(.9)])
    bad=row(1);bad['right_source_s']=float('nan')
    with pytest.raises(ValueError):summarize([bad])


def test_repeated_single_normal_does_not_count_as_two_observed_directions():
    r=row(1);r['covariance_xy_m2']=[[.001,0],[0,.04]]
    r['factors']=[dict(accepted=True,normal=[1,0])]*20
    result=summarize([r])['reported_uncertainty_not_calibrated']
    assert result['frames_with_two_accepted_normal_directions']==0
    assert result['maximum_largest_xy_std_m']==.2
    r['covariance_xy_m2']=[[1,2],[2,1]]
    with pytest.raises(ValueError):summarize([r])
