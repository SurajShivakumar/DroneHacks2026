import numpy as np
from tools.review_stereo_pairs import reproject_center_depth


def test_ideal_diagnostic_reference_uses_left_viewpoint_and_exact_pixel_stride():
    # Center point u=0 at Z=2 shifts by 4 pixels in a left camera offset.
    transform=np.eye(4); transform[0,3]=.04
    depth=np.full((1,2),2.)
    reference=reproject_center_depth(depth,8,[200,200,0,0],transform,(1,20))
    assert np.isfinite(reference[0,4]) and np.isfinite(reference[0,12])
    assert not np.isfinite(reference[0,0])
    assert reference[0,4]==2.
