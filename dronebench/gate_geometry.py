"""Offline conservative whole-cylinder plane crossing measurements, no I/O."""
import numpy as np


def cylinder_support(quaternion_wxyz, axis, radius, half_height):
    q=np.asarray(quaternion_wxyz,dtype=float)
    axis=np.asarray(axis,dtype=float)
    if q.shape[-1]!=4 or not np.isfinite(q).all() or not np.allclose(np.linalg.norm(q,axis=-1),1,atol=1e-3):
        raise ValueError('unit finite pose quaternions required')
    if axis.shape!=(3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1)>1e-6 or radius<=0 or half_height<=0:
        raise ValueError('physical cylinder and unit plane axis required')
    w,x,y,z=np.moveaxis(q,-1,0)
    up=np.stack([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)],axis=-1)
    dot=np.clip(up@axis,-1,1)
    return radius*np.sqrt(1-dot*dot)+half_height*np.abs(dot)


def whole_crossing(times, positions, quaternions, *, centre, normal, lateral,
                   half_width, half_height, radius, body_half_height, start_index=0):
    times=np.asarray(times); positions=np.asarray(positions)
    if (times.ndim!=1 or len(times)<2 or positions.shape!=(len(times),3)
            or not np.isfinite(times).all() or not np.isfinite(positions).all()
            or np.any(np.diff(times)<=0)):
        raise ValueError('finite chronological pose samples required')
    offset=positions-np.asarray(centre)
    d=offset@normal
    normal_support=cylinder_support(quaternions,normal,radius,body_half_height)
    front=d+normal_support; rear=d-normal_support
    # Checkpoint completion order does not require disjoint entry intervals:
    # an aircraft may enter the next plane's slab before clearing the previous.
    before=np.flatnonzero(front<0)
    if not len(before):
        return {'whole_crossed':False,'reason':'no fully approaching aircraft observed'}
    after=np.flatnonzero((np.arange(len(times))>=start_index)&(np.arange(len(times))>before[0])&(rear>=0))
    if not len(after):
        return {'whole_crossed':False,'reason':'centre/nose crossing without entire aircraft beyond plane'}
    last=int(after[0]); first=int(before[before<last][-1])
    slab=(np.arange(len(times))>=first)&(np.arange(len(times))<=last)&(front>=0)&(rear<=0)
    # Enclosing cylinder projection is conservative: this does not substitute
    # for exact aperture contact geometry or an organizer's checkpoint lines.
    side_margin=half_width-np.abs(offset@lateral)-cylinder_support(quaternions,lateral,radius,body_half_height)
    vertical_margin=half_height-np.abs(offset[:,2])-cylinder_support(quaternions,[0,0,1],radius,body_half_height)
    clearance=bool(np.any(slab) and np.all(side_margin[slab]>=0) and np.all(vertical_margin[slab]>=0))
    return {'whole_crossed':True,'entire_aircraft_beyond_plane_source_s':float(times[last]),
        'sample_index':last,'conservative_rectangle_clearance':clearance,
        'minimum_lateral_margin_m':float(np.min(side_margin[slab])) if np.any(slab) else None,
        'minimum_vertical_margin_m':float(np.min(vertical_margin[slab])) if np.any(slab) else None}
