from types import SimpleNamespace as NS
import numpy as np
import pytest
from tools.observe_published_stereo import checked_stereo_camera


def test_standard_right_projection_baseline_is_checked_without_mutating_message():
    head=NS(stamp=NS(sec=1,nanosec=0),frame_id='right_optical')
    info=NS(header=head,width=640,height=400,distortion_model='plumb_bob',d=[0.]*5,
            k=[380.,0,320,0,380.,200,0,0,1],r=np.eye(3).ravel().tolist(),
            p=[380.,0,320,-28.5,0,380.,200,0,0,0,1,0],binning_x=0,binning_y=0,
            roi=NS(x_offset=0,y_offset=0,width=0,height=0))
    image=NS(header=head,width=640,height=400)
    assert checked_stereo_camera(info,image,'right_optical',baseline_m=.075).fx==380
    assert info.p[3]==-28.5
    with pytest.raises(ValueError):checked_stereo_camera(info,image,'right_optical',baseline_m=0)
    with pytest.raises(ValueError):checked_stereo_camera(info,image,'left_optical',baseline_m=.075)
    info.p[3]=-20
    with pytest.raises(ValueError):checked_stereo_camera(info,image,'right_optical',baseline_m=.075)
