"""Sparse measured stereo against an explicitly supplied published-map prior.

No world files, simulator state, position fixes or flight messages. The caller
owns source synchronization, calibrated rectification, pose/error validity,
height, resets and freshness. A wrong local prior or omitted occluder can still
cause consistent aliases. Outputs require independent validation before fusion.
"""
from dataclasses import dataclass
import time
import numpy as np
from dronebench.depth_mapping import Pinhole
from dronebench.map_guided_stereo import MapGuidedStereoConfig
from dronebench.sparse_map_stereo import match_sparse_patches


def depth_prior(ray_map,rays,body_from_camera,position,rotation,error_xyz,angular_error_rad):
    rays=np.asarray(rays,float); T=np.asarray(body_from_camera,float)
    p=np.asarray(position,float); R=np.asarray(rotation,float); error_xyz=np.asarray(error_xyz,float)
    if (rays.ndim!=2 or rays.shape[1]!=3 or T.shape!=(4,4) or p.shape!=(3,) or R.shape!=(3,3)
            or error_xyz.shape!=(3,) or not np.isfinite([*rays.ravel(),*T.ravel(),*p,*R.ravel(),*error_xyz,angular_error_rad]).all()
            or np.any(error_xyz<0) or angular_error_rad<0
            or not np.allclose(R.T@R,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(R),1,atol=1e-6)
            or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6)
            or not np.isclose(np.linalg.det(T[:3,:3]),1,atol=1e-6)):
        raise ValueError('finite rays, proper calibration/rotation and nonnegative pose-error bounds required')
    offset=R@T[:3,3]; directions=rays@(R@T[:3,:3]).T
    hits=ray_map.cast(p+offset,directions)
    if hits.origin_inside_solid:raise ValueError('declared camera prior lies inside a published solid')
    denominator=np.abs(np.einsum('ni,ni->n',hits.normals,directions))
    translation=np.abs(hits.normals)@error_xyz
    rotation_error=angular_error_rad*(hits.t*np.linalg.norm(directions,axis=1)+np.linalg.norm(offset))
    with np.errstate(divide='ignore',invalid='ignore'):uncertainty=(translation+rotation_error+.01)/denominator
    lo=np.maximum(.25,hits.t-uncertainty); hi=np.minimum(8.,hits.t+uncertainty)
    valid=np.isfinite(hits.t)&(denominator>.1)&(lo<hi)
    lo[~valid]=np.nan; hi[~valid]=np.nan
    return lo,hi


@dataclass(frozen=True)
class PublishedStereoObservation:
    points_base_xyz: np.ndarray
    pixels_left_uv: np.ndarray
    diagnostics: dict


class PublishedSparseStereo:
    def __init__(self,ray_map,camera,body_from_left,body_from_right,*,max_features=500,feature_band_fraction=.16):
        if not isinstance(camera,Pinhole) or type(max_features) is not int or not 20<=max_features<=1000 or not .1<=feature_band_fraction<=1:
            raise ValueError('calibrated pinhole and bounded feature selection required')
        self.ray_map=ray_map; self.camera=camera
        self.left=np.asarray(body_from_left,float); self.right=np.asarray(body_from_right,float)
        for transform in (self.left,self.right):
            if (transform.shape!=(4,4) or not np.isfinite(transform).all() or not np.allclose(transform[3],[0,0,0,1])
                    or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-6)
                    or not np.isclose(np.linalg.det(transform[:3,:3]),1,atol=1e-6)):
                raise ValueError('proper calibrated body-from-optical transforms required')
        baseline=self.left[:3,:3].T@(self.right[:3,3]-self.left[:3,3])
        if not np.allclose(self.left[:3,:3],self.right[:3,:3]) or baseline[0]<=0 or not np.allclose(baseline[1:],0,atol=1e-7):
            raise ValueError('parallel rectified stereo with positive optical-X baseline required')
        self.config=MapGuidedStereoConfig(fx=camera.fx,baseline_m=float(baseline[0]))
        self.max_features=max_features; self.band=feature_band_fraction

    def _rays(self,uv):
        k=self.camera
        return np.c_[(uv[:,0]-k.cx)/k.fx,(uv[:,1]-k.cy)/k.fy,np.ones(len(uv))]

    def observe(self,left,right,*,position,rotation,error_xyz,angular_error_rad):
        left=np.asarray(left); right=np.asarray(right); k=self.camera
        if left.shape!=(k.height,k.width) or right.shape!=left.shape or left.dtype!=np.uint8 or right.dtype!=np.uint8:
            raise ValueError('rectified mono8 images must match supplied camera calibration')
        import cv2
        started=time.perf_counter(); mask=np.zeros(left.shape,np.uint8)
        top=int(k.height*(1-self.band)/2); mask[top:k.height-top,:]=255
        detected=cv2.goodFeaturesToTrack(left,maxCorners=self.max_features,qualityLevel=.01,minDistance=8,blockSize=5,mask=mask)
        uv=detected.reshape(-1,2) if detected is not None else np.empty((0,2))
        lo,hi=depth_prior(self.ray_map,self._rays(uv),self.left,position,rotation,error_xyz,angular_error_rad)
        forward=match_sparse_patches(left,right,uv,lo,hi,sign=1,config=self.config)
        selected=np.flatnonzero(forward.valid); right_uv=uv[selected].copy(); right_uv[:,0]-=forward.disparity_px[selected]
        lo,hi=depth_prior(self.ray_map,self._rays(right_uv),self.right,position,rotation,error_xyz,angular_error_rad)
        backward=match_sparse_patches(right,left,right_uv,lo,hi,sign=-1,config=self.config)
        good=backward.valid&(abs(forward.disparity_px[selected]+backward.disparity_px)<=self.config.lr_max_error_px)
        selected=selected[good]; pixels=uv[selected]; z=forward.depth_m[selected]
        optical=self._rays(pixels)*z[:,None]
        body=optical@self.left[:3,:3].T+self.left[:3,3]
        return PublishedStereoObservation(body,pixels,dict(keypoints=len(uv),valid_points=len(body),wall_s=time.perf_counter()-started,
            forward=forward.diagnostics,backward=backward.diagnostics,
            scope='actual_stereo_and_declared_local_published_prior_not_position_fix_or_flight_qualification'))
