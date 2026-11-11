"""Sparse patch correspondence with explicit published-map depth intervals.

No world files, ideal depth, pose fixes or flight interfaces. Prior error,
occlusion and periodic phases remain limitations; image checks alone cannot
certify global matching. Call twice with independent eye priors for LR checks.
"""
from dataclasses import dataclass,asdict
import numpy as np
from dronebench.map_guided_stereo import MapGuidedStereoConfig


@dataclass(frozen=True)
class SparseStereoMatches:
    disparity_px: np.ndarray
    depth_m: np.ndarray
    valid: np.ndarray
    diagnostics: dict


def match_sparse_patches(image,other,pixels,depth_lo,depth_hi,*,sign,config):
    image=np.asarray(image); other=np.asarray(other); uv=np.asarray(pixels,float)
    lo=np.asarray(depth_lo,float); hi=np.asarray(depth_hi,float)
    if (image.ndim!=2 or image.shape!=other.shape or image.dtype!=np.uint8 or other.dtype!=np.uint8
            or uv.ndim!=2 or uv.shape[1]!=2 or len(uv)>2000 or not np.isfinite(uv).all()
            or lo.shape!=(len(uv),) or hi.shape!=(len(uv),) or sign not in (-1,1)
            or not isinstance(config,MapGuidedStereoConfig)):
        raise ValueError('mono8 pair, finite Nx2 pixels, per-point intervals and signed matching config required')
    cfg=config
    if cfg.block_size>9 or cfg.max_disparity>255:
        raise ValueError('sparse matcher bounds are block<=9, disparity<=255')
    import cv2
    n=len(uv); h,w=image.shape; radius=cfg.block_size//2
    prior=np.isfinite(lo)&np.isfinite(hi)&(lo>0)&(lo<hi)&(hi-lo<=cfg.max_prior_width_m)
    if n==0:return SparseStereoMatches(np.array([],np.float32),np.array([],np.float32),np.array([],bool),dict(points=0,valid_points=0))
    yy,xx=np.mgrid[-radius:radius+1,-radius:radius+1]
    px=xx.ravel(); py=yy.ravel(); count=len(px)
    disparities=sign*np.arange(1,cfg.max_disparity+1,dtype=float)
    sx=(uv[:,0,None]+px).astype(np.float32); sy=(uv[:,1,None]+py).astype(np.float32)
    reference=cv2.remap(image,sx,sy,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT).astype(np.float32)
    tx=(uv[:,0,None,None]-disparities[None,:,None]+px).astype(np.float32)
    ty=np.broadcast_to(uv[:,1,None,None]+py,tx.shape).astype(np.float32)
    patches=cv2.remap(other,tx.reshape(n,-1),ty.reshape(n,-1),cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_CONSTANT).reshape(n,len(disparities),count).astype(np.float32)
    cost=np.mean(np.abs(patches-reference[:,None,:]),axis=2)
    geometric=((uv[:,1,None]>=radius)&(uv[:,1,None]<h-radius)&(uv[:,0,None]>=radius)&
               (uv[:,0,None]<w-radius)&(uv[:,0,None]-disparities>=radius)&(uv[:,0,None]-disparities<w-radius))
    z=cfg.fx*cfg.baseline_m/np.abs(disparities)
    allowed=prior[:,None]&geometric&(lo[:,None]<=z)&(z<=hi[:,None])
    masked=np.where(allowed,cost,np.inf)
    index=np.argmin(masked,axis=1); rows=np.arange(n); best=masked[rows,index]
    chosen=disparities[index].copy()
    separated=np.abs(disparities[None,:]-chosen[:,None])>cfg.ambiguity_exclusion_px
    alternative=np.min(np.where(allowed&separated,cost,np.inf),axis=1)
    unique=alternative>best*(1+cfg.ambiguity_relative_margin)+cfg.ambiguity_absolute_margin
    # Cost array order is reversed for signed right matching. Refinement is
    # in array-index space, then converted back to disparity using sign.
    before=cost[rows,np.maximum(index-1,0)]; after=cost[rows,np.minimum(index+1,len(disparities)-1)]
    curvature=before-2*cost[rows,index]+after
    refine=(index>0)&(index<len(disparities)-1)&(curvature>1e-5)
    chosen[refine]+=sign*np.clip(.5*(before[refine]-after[refine])/curvature[refine],-.5,.5)
    depth=cfg.fx*cfg.baseline_m/np.abs(chosen)
    textured=(np.std(reference,axis=1)>=cfg.min_texture_std)&(np.std(patches[rows,index],axis=1)>=cfg.min_texture_std)
    valid=np.isfinite(best)&unique&textured&(best<=cfg.max_photometric_error)&(lo<=depth)&(depth<=hi)
    chosen[~valid]=np.nan; depth[~valid]=np.nan
    return SparseStereoMatches(chosen.astype(np.float32),depth.astype(np.float32),valid,
        dict(points=n,prior_points=int(prior.sum()),valid_points=int(valid.sum()),
             ambiguous_points=int((np.isfinite(best)&~unique).sum()),configuration=asdict(cfg),
             scope='sparse_image_and_declared_prior_correspondence_not_pose_or_depth_certificate'))
