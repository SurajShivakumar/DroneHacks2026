import numpy as np
import pytest
from dronebench.map_guided_stereo import MapGuidedStereoConfig
from dronebench.sparse_map_stereo import match_sparse_patches


def test_sparse_known_plane_and_signed_reverse_matching():
    pytest.importorskip('cv2')
    rng=np.random.default_rng(20); left=rng.integers(0,256,(64,192),dtype=np.uint8)
    right=rng.integers(0,256,left.shape,dtype=np.uint8); right[:,:-4]=left[:,4:]
    uv=np.array([(x,y) for y in range(10,55,10) for x in range(40,150,10)])
    lo=np.full(len(uv),4.8); hi=np.full(len(uv),5.2)
    cfg=MapGuidedStereoConfig(fx=200,baseline_m=.1,max_disparity=31)
    a=match_sparse_patches(left,right,uv,lo,hi,sign=1,config=cfg)
    assert a.valid.mean()>.9 and abs(np.nanmedian(a.depth_m)-5)<.1
    reverse=uv.astype(float); reverse[:,0]-=a.disparity_px
    b=match_sparse_patches(right,left,reverse,lo,hi,sign=-1,config=cfg)
    good=a.valid&b.valid
    assert good.mean()>.9 and np.max(abs(a.disparity_px[good]+b.disparity_px[good]))<.75


def test_sparse_periodic_phases_need_a_declared_prior_and_unknowns_stay_unknown():
    pytest.importorskip('cv2'); y,x=np.indices((64,192))
    left=np.where(((x%12)<6)^((y%12)<6),230,20).astype(np.uint8)
    right=np.roll(left,-4,axis=1); uv=np.array([(x,y) for y in (12,24,36) for x in (48,60,72,84)])
    cfg=MapGuidedStereoConfig(fx=200,baseline_m=.1,max_disparity=31,max_prior_width_m=10)
    a=match_sparse_patches(left,right,uv,np.ones(len(uv)),np.full(len(uv),6.),sign=1,config=cfg)
    assert not a.valid.any()
    b=match_sparse_patches(left,right,uv,np.full(len(uv),4.8),np.full(len(uv),5.2),sign=1,config=cfg)
    assert b.valid.all() and abs(np.median(b.depth_m)-5)<.1
    c=match_sparse_patches(left,right,uv,np.full(len(uv),np.nan),np.full(len(uv),np.nan),sign=1,config=cfg)
    assert not c.valid.any()
