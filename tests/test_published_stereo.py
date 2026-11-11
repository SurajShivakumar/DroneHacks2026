from types import SimpleNamespace
import numpy as np
import pytest
from dronebench.published_raycast import build_published_raycast
from dronebench.published_stereo import PublishedSparseStereo,depth_prior
from dronebench.depth_mapping import Pinhole


def scene_and_cameras():
    course=SimpleNamespace(box_c=np.array([[2.05,0,1]]),box_u=np.array([[1.,0]]),box_h=np.array([[.05,10,10]]),box_names=['front'])
    scene=build_published_raycast(course,published_provenance='synthetic analytic wall')
    left=np.eye(4); left[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]; left[1,3]=.05
    right=left.copy(); right[1,3]=-.05
    return scene,left,right


def test_declared_depth_interval_is_optical_z_and_invalid_rotation_rejects():
    scene,left,_=scene_and_cameras()
    lo,hi=depth_prior(scene,np.array([[0,0,1.]]),left,[0,0,.9],np.eye(3),[.02,.02,.02],0)
    assert abs(lo[0]-1.97)<1e-8 and abs(hi[0]-2.03)<1e-8
    with pytest.raises(ValueError):depth_prior(scene,np.array([[0,0,1.]]),left,[0,0,.9],np.zeros((3,3)),[.02]*3,0)


def test_recorded_pixels_are_reconstructed_with_both_camera_lever_arms():
    pytest.importorskip('cv2'); scene,left,right=scene_and_cameras()
    rng=np.random.default_rng(22); a=rng.integers(0,256,(64,192),dtype=np.uint8)
    b=rng.integers(0,256,a.shape,dtype=np.uint8); b[:,:-10]=a[:,10:]
    model=PublishedSparseStereo(scene,Pinhole(192,64,200.,200.,96.,32.),left,right,max_features=200,feature_band_fraction=.5)
    observation=model.observe(a,b,position=[0,0,.9],rotation=np.eye(3),error_xyz=[.02]*3,angular_error_rad=.001)
    assert len(observation.points_base_xyz)>20
    assert np.max(abs(observation.points_base_xyz[:,0]-2.))<.03
