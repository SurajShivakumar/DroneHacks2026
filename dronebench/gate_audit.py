"""Offline full-envelope route-gate audit; never a flight controller or score."""
import numpy as np


def cylinder_support(axis, body_z, radius, half_height):
    axial=float(np.dot(axis, body_z))
    return radius*np.sqrt(max(0.,float(np.dot(axis,axis))-axial*axial))+half_height*abs(axial)


def audit(poses, gates, *, radius, half_height, maximum_gap_s=.025):
    """Require the trailing envelope to clear each ordered plane aperture.

    Gate definitions may include teammate route markers, not official scoring
    checkpoints. This does not check mesh contacts, the pad, disarm or scoring.
    A sampled enclosing cylinder is conservative; it is not swept geometry.
    """
    poses=np.asarray(poses,float)
    if (poses.ndim!=2 or poses.shape[1]!=8 or len(poses)<2 or not np.isfinite(poses).all()
            or np.any(np.diff(poses[:,0])<=0) or np.any(np.diff(poses[:,0])>maximum_gap_s)
            or not np.isfinite([radius,half_height,maximum_gap_s]).all()
            or min(radius,half_height,maximum_gap_s)<=0):
        raise ValueError('finite chronological densely sampled poses and positive envelope required')
    if np.any(abs(np.linalg.norm(poses[:,4:8],axis=1)-1)>.001):
        raise ValueError('unit pose quaternions required')
    if not gates or np.any(np.linalg.norm(np.diff(poses[:,1:4],axis=0),axis=1)>min(radius,half_height)):
        raise ValueError('nonempty gates and spatially dense pose sampling required')
    for gate in gates:
        center=np.asarray(gate['center'],float); normal=np.asarray(gate['normal'],float)
        aperture=[gate['half_width'],gate['half_height']]
        if (center.shape!=(3,) or normal.shape!=(3,) or not np.isfinite([*center,*normal,*aperture]).all()
                or abs(np.linalg.norm(normal)-1)>1e-6 or abs(normal[2])>1e-6
                or min(aperture)<=0):
            raise ValueError('finite horizontal unit gate normal and positive aperture required')
    events=[]; earliest=poses[0,0]
    w,x,y,z=poses[:,4:8].T
    body_axes=np.stack([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)],axis=1)
    for gate in gates:
        center=np.asarray(gate['center'],float); normal=np.asarray(gate['normal'],float)
        lateral=np.cross([0.,0.,1.],normal)
        distances=(poses[:,1:4]-center)@normal
        supports=np.array([cylinder_support(normal,b,radius,half_height) for b in body_axes])
        upstream=np.flatnonzero((poses[:,0]<=earliest)&(distances < -supports))
        if not len(upstream):
            return {'ordered_planes_cleared':False,'events':events,'next_gate':gate['name']}
        # Adjacent plane slabs can overlap. Replay from the latest upstream
        # sample before the preceding completion rather than requiring the
        # next aircraft approach to begin only after that completion.
        traversing=False; invalid=False; completed=False
        for i in range(int(upstream[-1]),len(poses)):
            t=poses[i,0]; point=poses[i,1:4]; body_z=body_axes[i]
            support=supports[i]; distance=distances[i]
            if distance>=-support: traversing=True
            if traversing and abs(distance)<=support:
                extent=cylinder_support(lateral,body_z,radius,half_height)
                if np.linalg.norm(body_z-np.array([0.,0.,1.]))<1e-8:
                    extent=np.sqrt(max(0.,radius*radius-distance*distance))
                width=abs(np.dot(point-center,lateral))+extent
                height=abs(point[2]-center[2])+cylinder_support(np.array([0.,0.,1.]),body_z,radius,half_height)
                invalid |= width>gate['half_width'] or height>gate['half_height']
            if traversing and distance>=support:
                invalid |= bool(events) and t<=earliest
                events.append({'gate':gate['name'],'source_s':float(t),
                               'full_envelope_clearance':not bool(invalid)})
                if invalid:
                    return {'ordered_planes_cleared':False,'events':events,'failed_gate':gate['name']}
                earliest=t; completed=True; break
        if not completed:
            return {'ordered_planes_cleared':False,'events':events,'next_gate':gate['name']}
    return {'ordered_planes_cleared':True,'events':events,'next_gate':None}
