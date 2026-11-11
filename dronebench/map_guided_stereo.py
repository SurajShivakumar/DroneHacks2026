"""Offline image correspondence conditioned on explicit per-pixel depth priors.

Priors must come from a declared published map and a valid pose neighborhood.
They are not measured depth. Wrong priors, omitted occluders and repetitive
texture can still select a wrong phase; outputs never establish a position fix
or flight readiness. No image holes are filled and no map/world files are read.
"""
from dataclasses import asdict,dataclass
from math import isfinite
import numpy as np


@dataclass(frozen=True)
class MapGuidedStereoConfig:
    fx: float
    baseline_m: float
    max_disparity: int = 127
    block_size: int = 5
    texture_window: int = 7
    min_texture_std: float = 5.
    max_photometric_error: float = 20.
    ambiguity_exclusion_px: int = 2
    ambiguity_relative_margin: float = .15
    ambiguity_absolute_margin: float = 2.
    lr_max_error_px: float = .75
    max_prior_width_m: float = 1.

    def __post_init__(self):
        for key in ('fx','baseline_m','min_texture_std','max_photometric_error',
                    'ambiguity_relative_margin','ambiguity_absolute_margin','lr_max_error_px','max_prior_width_m'):
            if not isfinite(getattr(self,key)) or getattr(self,key)<=0:raise ValueError('positive finite matching bounds required')
        for key in ('max_disparity','block_size','texture_window','ambiguity_exclusion_px'):
            if type(getattr(self,key)) is not int or getattr(self,key)<1:raise ValueError('positive integer matching sizes required')
        if self.block_size%2!=1 or self.texture_window%2!=1:raise ValueError('odd support windows required')
        if not isfinite(self.fx*self.baseline_m):raise ValueError('finite focal-baseline product required')


@dataclass(frozen=True)
class MapGuidedStereoResult:
    depth_m: np.ndarray
    valid: np.ndarray
    disparity_px: np.ndarray
    diagnostics: dict


def map_guided_stereo_depth(left,right,left_lo,left_hi,right_lo,right_hi,*,config):
    """Stream two block-cost passes per eye with independent signed matching.

    Depth intervals are optical Z in metres, in each eye's OWN viewpoint.
    Nonfinite/unordered/nonpositive/overwide intervals reject those pixels.
    Comparable separated alternatives inside the prior reject a match. The
    absence of alternatives outside a narrow prior is not visual uniqueness.
    """
    if not isinstance(config,MapGuidedStereoConfig):raise ValueError('matching configuration required')
    cfg=config; left=np.asarray(left); right=np.asarray(right)
    if left.ndim!=2 or left.shape!=right.shape or left.dtype!=np.uint8 or right.dtype!=np.uint8:
        raise ValueError('equal rectified mono8 images required')
    intervals=[np.asarray(a,float) for a in (left_lo,left_hi,right_lo,right_hi)]
    if any(a.shape!=left.shape for a in intervals):raise ValueError('per-pixel prior dimensions mismatch')
    h,w=left.shape; margin=max(cfg.block_size,cfg.texture_window)//2
    if h<=2*margin or w<=cfg.max_disparity+2*margin:raise ValueError('images too small for configured disparity search')
    import cv2
    yy,xx=np.indices(left.shape); fb=cfg.fx*cfg.baseline_m

    def prior_mask(lo,hi):
        return np.isfinite(lo)&np.isfinite(hi)&(lo>0)&(hi>lo)&(hi-lo<=cfg.max_prior_width_m)

    def texture(image):
        a=image.astype(np.float32); window=(cfg.texture_window,)*2
        return np.maximum(cv2.boxFilter(a*a,-1,window)-cv2.boxFilter(a,-1,window)**2,0)>=cfg.min_texture_std**2

    def match(image,other,lo,hi,sign):
        prior=prior_mask(lo,hi); disparities=range(sign,sign*(cfg.max_disparity+1),sign)
        best=np.full(image.shape,np.inf,np.float32); chosen=np.zeros(image.shape,np.int16)
        image=image.astype(np.float32); other=other.astype(np.float32)
        def costs(d):
            shifted=np.zeros_like(other)
            if d>0:shifted[:,d:]=other[:,:-d]
            else:shifted[:,:d]=other[:,-d:]
            cost=cv2.boxFilter(np.abs(image-shifted),-1,(cfg.block_size,)*2)
            geometric=(yy>=margin)&(yy<h-margin)&(xx>=margin)&(xx<w-margin)&(xx-d>=margin)&(xx-d<w-margin)
            return cost,geometric
        for d in disparities:
            cost,geometric=costs(d); z=fb/abs(d)
            allowed=prior&geometric&(lo<=z)&(z<=hi)
            better=allowed&(cost<best); best[better]=cost[better]; chosen[better]=d
        alt=np.full_like(best,np.inf); before=np.full_like(best,np.inf); after=np.full_like(best,np.inf)
        for d in disparities:
            cost,geometric=costs(d); z=fb/abs(d)
            allowed=prior&geometric&(lo<=z)&(z<=hi)
            competing=allowed&(np.abs(d-chosen)>cfg.ambiguity_exclusion_px)
            alt[competing]=np.minimum(alt[competing],cost[competing])
            take=geometric&(d==chosen-1); before[take]=cost[take]
            take=geometric&(d==chosen+1); after[take]=cost[take]
        supported=np.isfinite(best)
        unique=alt>best*(1+cfg.ambiguity_relative_margin)+cfg.ambiguity_absolute_margin
        d=chosen.astype(np.float32)
        refine=supported&np.isfinite(before)&np.isfinite(after)
        curvature=np.zeros_like(best); curvature[refine]=before[refine]-2*best[refine]+after[refine]
        refine &=curvature>1e-5
        d[refine]+=np.clip(.5*(before[refine]-after[refine])/curvature[refine],-.5,.5)
        with np.errstate(divide='ignore',invalid='ignore'):z=fb/np.abs(d)
        good=supported&unique&(best<=cfg.max_photometric_error)&texture(image)&(lo<=z)&(z<=hi)
        d[~good]=np.nan
        return d,good,int(supported.sum()),int((supported&~unique).sum())

    dl,vl,support_l,ambiguous_l=match(left,right,*intervals[:2],1)
    dr,vr,support_r,ambiguous_r=match(right,left,*intervals[2:],-1)
    xr=xx-np.nan_to_num(dl,nan=0.); x0=np.floor(xr).astype(int); x1=x0+1
    inside=(x0>=0)&(x1<w); i0=np.clip(x0,0,w-1); i1=np.clip(x1,0,w-1)
    fraction=xr-x0; counterpart=(1-fraction)*dr[yy,i0]+fraction*dr[yy,i1]
    valid=vl&inside&vr[yy,i0]&vr[yy,i1]&(np.abs(dl+counterpart)<=cfg.lr_max_error_px)
    depth=np.full(left.shape,np.nan,np.float32); depth[valid]=fb/dl[valid]
    dl[~valid]=np.nan
    diagnostics=dict(scope='offline_published_prior_conditioned_matching_not_standalone_depth_or_fix',
        configuration=asdict(cfg),opencv_version=cv2.__version__,valid_pixels=int(valid.sum()),valid_fraction=float(valid.mean()),
        left_supported_pixels=support_l,right_supported_pixels=support_r,
        left_ambiguity_rejected=ambiguous_l,right_ambiguity_rejected=ambiguous_r,
        left_prior_pixels=int(prior_mask(*intervals[:2]).sum()),right_prior_pixels=int(prior_mask(*intervals[2:]).sum()))
    return MapGuidedStereoResult(depth,valid,dl,diagnostics)
