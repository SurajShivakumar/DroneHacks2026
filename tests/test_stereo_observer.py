from types import SimpleNamespace as NS
import numpy as np
import pytest
from dronebench.stereo_observer import CausalSamples,decode_mono8,body_height_from_range


def test_causal_history_never_uses_nearest_future_sample_or_stale_arrival():
    s=CausalSamples(); s.append(1,'old',10); s.append(1.04,'future',10.01)
    assert s.at(1.03,10.1)==(1,'old')
    with pytest.raises(ValueError):s.at(1.03,11)
    with pytest.raises(ValueError):s.at(1.2,10.1)
    with pytest.raises(ValueError):s.append(.9,'reset',10.2)
    with pytest.raises(ValueError):s.at(1.04,10.2)


def test_range_projects_sensor_offset_and_tilt_without_inventing_height():
    angle=.3; c,s=np.cos(angle),np.sin(angle)
    R=np.array([[c,0,s],[0,1,0],[-s,0,c]])
    assert body_height_from_range(1,R,[.1,0,-.02],[0,0,-1])==pytest.approx(c+.1*s+.02*c)
    with pytest.raises(ValueError):body_height_from_range(1,np.eye(3),[0,0,0],[0,0,1])
    with pytest.raises(ValueError):body_height_from_range(np.nan,R,[0,0,0],[0,0,-1])


def test_mono_decoder_preserves_padding_and_rejects_rgb():
    image=NS(encoding='mono8',width=2,height=2,step=3,data=b'\x01\x02\xff\x03\x04\xff')
    np.testing.assert_equal(decode_mono8(image),[[1,2],[3,4]])
    image.encoding='rgb8'
    with pytest.raises(ValueError):decode_mono8(image)
