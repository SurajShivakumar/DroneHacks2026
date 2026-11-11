import numpy as np
import pytest
from dronebench.map_guided_stereo import MapGuidedStereoConfig,map_guided_stereo_depth


def images(periodic=False):
    rng=np.random.default_rng(42)
    if periodic:
        y,x=np.indices((64,192)); left=np.where(((x%12)<6)^((y%12)<6),230,20).astype(np.uint8)
    else:left=rng.integers(0,256,(64,192),dtype=np.uint8)
    right=rng.integers(0,256,left.shape,dtype=np.uint8); right[:,:-4]=left[:,4:]
    return left,right


def run_pair(pair,lo=4.8,hi=5.2,max_width=1.):
    pytest.importorskip('cv2')
    intervals=[np.full(pair[0].shape,v) for v in (lo,hi,lo,hi)]
    cfg=MapGuidedStereoConfig(fx=200,baseline_m=.1,max_disparity=31,max_prior_width_m=max_width)
    return map_guided_stereo_depth(*pair,*intervals,config=cfg)


def test_known_plane_uses_image_evidence_not_just_prior():
    result=run_pair(images())
    assert result.valid[:,40:140].mean()>.7
    assert abs(np.nanmedian(result.depth_m)-5)<.1
    a,b=images(); b=np.random.default_rng(4).integers(0,256,b.shape,dtype=np.uint8)
    assert run_pair((a,b)).valid.mean()<.001


def test_periodic_phases_rejected_in_broad_prior_and_selected_only_in_declared_narrow_prior():
    pair=images(True)
    broad=run_pair(pair,lo=.7,hi=6,max_width=10)
    assert broad.valid[:,40:140].mean()==0
    narrow=run_pair(pair)
    # Flat checker interiors have no unique subpixel support. Require recovery
    # at the actual two-directional corners instead of arbitrary whole-image
    # coverage (the first draft's >50% coverage assumption failed at 46%).
    y,x=np.indices(narrow.valid.shape)
    corners=(x>=40)&(x<140)&(y>=8)&(y<56)&np.isin(x%6,[0,5])&np.isin(y%6,[0,5])
    assert narrow.valid[corners].mean()>.9
    assert abs(np.nanmedian(narrow.depth_m)-5)<.1


def test_unknown_prior_does_not_create_depth():
    result=run_pair(images(),lo=np.nan,hi=np.nan)
    assert not result.valid.any() and np.isnan(result.depth_m).all()


def test_mismatched_prior_rejected_before_matching():
    a,b=images()
    with pytest.raises(ValueError,match='prior dimensions'):
        map_guided_stereo_depth(a,b,np.zeros((1,1)),a,a,a,config=MapGuidedStereoConfig(fx=200,baseline_m=.1))
