"""Published-geometry camera-yaw proposal; not measured observability or control.

Normal diversity is a planning proxy only. Transparent netting, darkness,
texture, correspondence failure and omitted geometry still need actual sensor
checks. This module publishes nothing and never grants flight authority.
"""
from dataclasses import dataclass
from math import pi
import numpy as np
from dronebench.depth_mapping import Pinhole


@dataclass(frozen=True)
class ViewChoice:
    yaw: float
    score: float
    minimum_normal_information: float
    informative_rays: int
    candidates: tuple


def choose_view_yaw(ray_map,camera,body_from_camera,position,current_yaw,*,
                    candidate_offsets=None,rotation_cost=.1,samples=41):
    T=np.asarray(body_from_camera,float); p=np.asarray(position,float)
    if (not isinstance(camera,Pinhole) or T.shape!=(4,4) or p.shape!=(3,)
            or not np.isfinite([*T.ravel(),*p,current_yaw,rotation_cost]).all() or rotation_cost<0
            or type(samples) is not int or samples<5
            or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6)
            or not np.isclose(np.linalg.det(T[:3,:3]),1,atol=1e-6)):
        raise ValueError('calibrated camera, rigid transform, finite pose and bounded scoring inputs required')
    offsets=np.linspace(-pi,pi,25) if candidate_offsets is None else np.asarray(candidate_offsets,float)
    if offsets.ndim!=1 or not len(offsets) or not np.isfinite(offsets).all():raise ValueError('finite yaw candidates required')
    pixels=np.linspace(0,camera.width-1,samples)
    optical=np.c_[(pixels-camera.cx)/camera.fx,np.zeros(samples),np.ones(samples)]
    records=[]
    for offset in offsets:
        yaw=current_yaw+offset; c,s=np.cos(yaw),np.sin(yaw)
        R=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        directions=optical@(R@T[:3,:3]).T
        hits=ray_map.cast(p+R@T[:3,3],directions)
        if hits.origin_inside_solid:raise ValueError('camera proposal starts inside a published solid')
        incidence=np.abs(np.einsum('ni,ni->n',hits.normals,directions))/np.linalg.norm(directions,axis=1)
        useful=np.isfinite(hits.t)&(hits.t>.25)&(hits.t<5)&(np.abs(hits.normals[:,2])<.1)&(incidence>.2)
        n=hits.normals[useful,:2]; weights=incidence[useful]**2
        information=(n.T@(n*weights[:,None]))/samples if len(n) else np.zeros((2,2))
        minimum=float(np.linalg.eigvalsh(information).min())
        delta=float((offset+pi)%(2*pi)-pi)
        score=minimum-rotation_cost*abs(delta)
        records.append(dict(yaw=float((yaw+pi)%(2*pi)-pi),score=score,
                            minimum_normal_information=minimum,informative_rays=int(useful.sum())))
    best=max(records,key=lambda row:row['score'])
    return ViewChoice(best['yaw'],best['score'],best['minimum_normal_information'],best['informative_rays'],tuple(records))
