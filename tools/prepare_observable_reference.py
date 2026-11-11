"""Offline gentle lateral-excitation candidate; no flight or simulator interface.

Adds <=1.5 cm of actual-time course-X motion to a supplied public-map reference
to test camera parallax. It is not a localization fix or proven VIO improvement.
The existing tracking/authority/quality checks must remain in force in flight.
"""
import argparse
from dataclasses import dataclass,asdict
from hashlib import sha256
import json
from pathlib import Path
import shutil
import numpy as np


@dataclass(frozen=True)
class Excitation:
    amplitude_m: float=.015
    period_s: float=2.5
    fade_s: float=6.

    def __post_init__(self):
        if (not np.isfinite(list(asdict(self).values())).all() or
                not 0<self.amplitude_m<=.015 or self.period_s<2.5 or self.fade_s<6):
            raise ValueError('positive amplitude<=1.5cm, period>=2.5s and fade>=6s required')


def smooth_step(u):
    """Quintic fade and derivatives with respect to its dimensionless input."""
    u=np.asarray(u,float);v=np.clip(u,0,1);active=(u>=0)&(u<=1)
    return (10*v**3-15*v**4+6*v**5,
        np.where(active,30*v**2-60*v**3+30*v**4,0),
        np.where(active,60*v-180*v**2+120*v**3,0),
        np.where(active,60-360*v+360*v**2,0))


def excitation_motion(t,duration,settings=None):
    settings=settings or Excitation();t=np.asarray(t,float)
    if (not isinstance(settings,Excitation) or not np.isfinite(t).all() or
            not np.isfinite(duration) or duration<2*settings.fade_s or np.any(t<0) or np.any(t>duration)):
        raise ValueError('finite covered source times and nonoverlapping fade windows required')
    F=settings.fade_s
    a,a1,a2,a3=smooth_step(t/F);b,b1,b2,b3=smooth_step((duration-t)/F)
    e=a*b;e1=(a1*b-a*b1)/F;e2=(a2*b-2*a1*b1+a*b2)/F**2
    e3=(a3*b-3*a2*b1+3*a1*b2-a*b3)/F**3
    w=2*np.pi/settings.period_s;s=np.sin(w*t);c=np.cos(w*t);A=settings.amplitude_m
    return np.stack([A*e*s,A*(e1*s+e*w*c),
        A*(e2*s+2*e1*w*c-e*w*w*s),
        A*(e3*s+3*e2*w*c-3*e1*w*w*s-e*w**3*c)],axis=-1)


def dense_scaled_reference(rows,time_scale):
    """Densify supplied positions/velocities with local cubic Hermite segments.

    Retains every source sample; inserted samples follow the local endpoint
    tangents. Constant yaw is retained. No interpolation across large gaps.
    """
    base=rows.copy();base[:,0]*=time_scale;base[:,4:7]/=time_scale
    if np.max(np.diff(base[:,0]))>.05+1e-9:raise ValueError('source sampling too sparse to resample safely')
    dense=[base[0].copy()]
    for a,b in zip(base,base[1:]):
        h=b[0]-a[0];count=int(np.ceil(h/.02))
        for i in range(1,count+1):
            if i==count:dense.append(b.copy());continue
            u=i/count;row=a.copy();row[0]=a[0]+u*h
            row[1:4]=(2*u**3-3*u**2+1)*a[1:4]+(u**3-2*u**2+u)*h*a[4:7]+(-2*u**3+3*u**2)*b[1:4]+(u**3-u**2)*h*b[4:7]
            row[4:7]=(6*u**2-6*u)/h*a[1:4]+(3*u**2-4*u+1)*a[4:7]+(-6*u**2+6*u)/h*b[1:4]+(3*u**2-2*u)*b[4:7]
            dense.append(row)
    return np.asarray(dense)


def gate_slab_envelope(rows,gates,radius,amplitude,fade_distance=.5):
    """Zero the offset whenever any nominal cylinder can intersect a gate.

    Includes the complete possible lateral displacement in the zero envelope.
    Conservative infinite plane slabs suppress motion even far from an opening.
    No opening width or checkpoint threshold is changed.
    """
    if not gates or not np.isfinite([radius,amplitude,fade_distance]).all() or min(radius,amplitude,fade_distance)<=0:
        raise ValueError('published gates and positive finite slab bounds required')
    value=np.ones(len(rows));derivative=np.zeros(len(rows));core=radius+amplitude+.005
    for gate in gates:
        centre=np.asarray(gate['center'],float);normal=np.asarray(gate['normal'],float)
        if centre.shape!=(3,) or normal.shape!=(3,) or not np.isfinite([*centre,*normal]).all() or abs(np.linalg.norm(normal)-1)>1e-8 or abs(normal[2])>1e-8:
            raise ValueError('finite published centre and horizontal unit normal required')
        d=(rows[:,1:4]-centre)@normal;velocity=rows[:,4:7]@normal
        f,df,_,_=smooth_step((abs(d)-core)/fade_distance)
        rate=df*np.sign(d)*velocity/fade_distance
        derivative=derivative*f+value*rate;value=value*f
    return value,derivative


def add_excitation(rows,*,time_scale,settings=None,gates=None,radius=None):
    rows=np.asarray(rows,float);settings=settings or Excitation()
    if (rows.ndim!=2 or rows.shape[1]!=11 or len(rows)<3 or not np.isfinite(rows).all() or
            rows[0,0]!=0 or np.any(np.diff(rows[:,0])<=0) or not np.isfinite(time_scale) or time_scale<1):
        raise ValueError('finite chronological t/XYZ/velocity/wxyz reference and time scale>=1 required')
    if not np.allclose(rows[:,7:],rows[0,7:],atol=1e-10) or abs(np.linalg.norm(rows[0,7:])-1)>1e-8:
        raise ValueError('constant calibrated unit body yaw required')
    if not np.allclose(rows[[0,-1],4:7],0,atol=1e-10):raise ValueError('source endpoints must be at rest')
    result=dense_scaled_reference(rows,time_scale)
    motion=excitation_motion(result[:,0],result[-1,0],settings)
    envelope,rate=(gate_slab_envelope(result,gates,radius,settings.amplitude_m) if gates is not None
                   else (np.ones(len(result)),np.zeros(len(result))))
    offset=motion[:,0]*envelope;velocity=motion[:,1]*envelope+motion[:,0]*rate
    result[:,1]+=offset;result[:,4]+=velocity
    # Floating point sin at the endpoints is multiplied by an exact zero fade.
    np.testing.assert_allclose(result[[0,-1],1:4],rows[[0,-1],1:4],atol=1e-12)
    np.testing.assert_allclose(result[[0,-1],4:7],0,atol=1e-12)
    dt=np.diff(result[:,0]);acc=np.diff(result[:,4:7],axis=0)/dt[:,None]
    mid=(result[1:,0]+result[:-1,0])/2;jerk=np.diff(acc,axis=0)/np.diff(mid)[:,None]
    report=dict(maximum_offset_m=float(np.max(abs(offset))),
        maximum_speed_mps=float(np.linalg.norm(result[:,4:7],axis=1).max()),
        maximum_sampled_acceleration_mps2=float(np.linalg.norm(acc,axis=1).max()),
        maximum_sampled_jerk_mps3=float(np.linalg.norm(jerk,axis=1).max()),
        max_sample_gap_s=float(dt.max()),excitation_peak_velocity_mps=float(abs(motion[:,1]).max()),
        excitation_peak_acceleration_mps2=float(abs(motion[:,2]).max()),
        excitation_peak_jerk_mps3=float(abs(motion[:,3]).max()),
        gate_slab_fade_applied=gates is not None,zero_slab_radius_m=radius+settings.amplitude_m+.005 if gates is not None else None,
        fraction_samples_with_excitation_above_1mm=float(np.mean(abs(offset)>.001)))
    if report['maximum_speed_mps']>.32 or report['maximum_sampled_acceleration_mps2']>.25 or report['maximum_sampled_jerk_mps3']>.5:
        raise ValueError('candidate exceeds reviewed slow speed/acceleration/jerk bounds')
    return result,report


def build(source,settings_path,contract,output,*,excitation=None,gate_audit_path=None):
    source,settings_path,contract,output=map(Path,(source,settings_path,contract,output))
    options=json.loads(settings_path.read_text());scale=options['trajectory_time_scale']
    names=['t','x','y','z','vx','vy','vz','qw','qx','qy','qz']
    original=np.genfromtxt(source,delimiter=',',names=True)
    if original.dtype.names!=tuple(names):raise ValueError('expected complete public reference CSV layout')
    rows=np.stack([original[name] for name in names],axis=1)
    gates=None;radius=None
    if gate_audit_path is not None:
        gate_audit_path=Path(gate_audit_path);gates=json.loads(gate_audit_path.read_text())['gates']
        radius=json.loads(contract.read_text())['collision_radius_m']
    excitation=excitation or Excitation();result,metrics=add_excitation(rows,time_scale=scale,settings=excitation,gates=gates,radius=radius)
    output.mkdir(parents=True,exist_ok=False)
    np.savetxt(output/'trajectory.csv',result,delimiter=',',header=','.join(names),comments='')
    report=dict(kind='offline_lateral_excitation_reference_not_flight_qualification',
        input_hashes={str(path):sha256(path.read_bytes()).hexdigest() for path in (source,settings_path,contract,*([gate_audit_path] if gate_audit_path else []))},
        generator_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),
        source_time_scale_baked_in=scale,required_caller_time_scale=1.,reference_duration_s=float(result[-1,0]),
        densification='local cubic Hermite position/velocity, all original samples retained, <=20ms actual-time spacing',
        excitation=asdict(excitation),metrics=metrics,course_axis='fixed X',altitude_changed=False,yaw_changed=False,
        endpoint_positions_and_rest_preserved=True,flight_completed=False,finish_time_s=None,
        strict_mesh_clearance_pending=True,ordered_full_envelope_gate_audit_pending=True,
        limitations='Unflown local parallax hypothesis, not demonstrated VIO improvement. Nominal forward motion stops at vertices while small lateral excitation continues. No simulator truth, extra sensors, feature maps or relaxed guards. Must pass unchanged strict clearance/gate checks before any review for flight; no takeoff/landing included.')
    (output/'plan.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    shutil.copy2(source,output/'source-trajectory.csv');shutil.copy2(settings_path,output/'source-settings.json')
    shutil.copy2(contract,output/'airframe-contract.json');shutil.copy2(Path(__file__),output/'generator.py')
    if gate_audit_path:shutil.copy2(gate_audit_path,output/'source-gate-audit.json')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','settings','contract','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--gate-audit',type=Path)
    args=parser.parse_args();print(json.dumps(build(args.source,args.settings,args.contract,args.output,gate_audit_path=args.gate_audit),indent=2))
