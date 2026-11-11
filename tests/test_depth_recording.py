import json
import numpy as np
import pytest
from dronebench.depth_recording import DepthSequence


def test_depth_evidence_retains_pixel_indices_unknowns_and_source_stamps(tmp_path):
    sequence=DepthSequence(tmp_path/'depth',topic='/depth',provenance='synthetic test',stride=2,period_ns=10)
    depth=np.arange(12,dtype=float).reshape(3,4); depth[2,2]=np.nan
    assert sequence.append(depth,100,'optical')
    assert not sequence.append(depth,105,'optical')
    assert sequence.append(depth,110,'optical')
    with pytest.raises(ValueError,match='regressed'):sequence.append(depth,109,'optical')
    saved=np.load(tmp_path/'depth/frame_000000.npy',allow_pickle=False)
    np.testing.assert_equal(saved,depth[::2,::2])
    result=sequence.close()
    assert [r['stamp_ns'] for r in result['frames']]==[100,110]
    assert result['frames'][0]['source_shape']==[3,4]
    assert json.loads((tmp_path/'depth/sequence.json').read_text())['requires_separate_camera_calibration']
