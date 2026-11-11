"""Offline XY position/velocity and VIO position/velocity-bias experiment.

Scalar wall factors observe only their normal. Explicit velocity-bias states
keep unobserved tangential position uncertainty from disappearing merely
because correlated VIO velocities arrive repeatedly. Noise settings remain
engineering assumptions; this supplies no ROS odometry or flight authority.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class WallKinematicConfig:
    acceleration_density: float = .3
    bias_walk_std: float = .01
    initial_position_std: float = .05
    initial_velocity_std: float = .10
    initial_bias_std: float = .05
    position_bias_walk_std: float = .02
    initial_position_bias_std: float = .05
    position_floor_std: float = .03
    velocity_observation_floor_std: float = .03
    max_prediction_gap_s: float = .5

    def __post_init__(self):
        if any(not np.isfinite(x) or x<=0 for x in vars(self).values()):
            raise ValueError('positive finite kinematic uncertainty/timing bounds required')


class WallKinematicFilter:
    def __init__(self,position,velocity,*,config=None):
        self.config=config or WallKinematicConfig()
        self.last_observations={}
        if not isinstance(self.config,WallKinematicConfig):raise ValueError('kinematic configuration required')
        p=np.asarray(position,float); v=np.asarray(velocity,float)
        if p.shape!=(2,) or v.shape!=(2,) or not np.isfinite([*p,*v]).all():raise ValueError('finite XY position/velocity required')
        self.mean=np.r_[p,v,0.,0.,0.,0.]
        c=self.config; self.covariance=np.diag([c.initial_position_std**2]*2+[c.initial_velocity_std**2]*2+
            [c.initial_bias_std**2]*2+[c.initial_position_bias_std**2]*2)

    @property
    def position(self):return self.mean[:2]

    @property
    def velocity(self):return self.mean[2:4]

    def predict(self,dt_s):
        c=self.config
        if not np.isfinite(dt_s) or not 0<dt_s<=c.max_prediction_gap_s:
            raise ValueError('prediction needs advancing bounded source-time gap')
        F=np.eye(8); F[:2,2:4]=np.eye(2)*dt_s
        Q=np.zeros((8,8)); a=c.acceleration_density**2
        Q[:2,:2]=np.eye(2)*a*dt_s**3/3
        Q[:2,2:4]=Q[2:4,:2]=np.eye(2)*a*dt_s**2/2
        Q[2:4,2:4]=np.eye(2)*a*dt_s
        Q[4:6,4:6]=np.eye(2)*c.bias_walk_std**2*dt_s
        Q[6:8,6:8]=np.eye(2)*c.position_bias_walk_std**2*dt_s
        self.mean=F@self.mean; self.covariance=F@self.covariance@F.T+Q

    def _observe(self,H,z,R,gate,kind):
        residual=z-H@self.mean; S=H@self.covariance@H.T+R
        statistic=float(residual@np.linalg.solve(S,residual))
        accepted=statistic<=gate
        self.last_observations[kind]=dict(accepted=accepted,innovation=residual.tolist(),normalized_squared_innovation=statistic,gate=gate)
        if not accepted:return False
        K=np.linalg.solve(S,(self.covariance@H.T).T).T
        self.mean+=K@residual; J=np.eye(8)-K@H
        self.covariance=J@self.covariance@J.T+K@R@K.T
        self.covariance=(self.covariance+self.covariance.T)/2
        self.covariance[:2,:2]+=np.diag(np.maximum(self.config.position_floor_std**2-np.diag(self.covariance)[:2],0))
        return True

    def observe_velocity(self,velocity,covariance):
        v=np.asarray(velocity,float); R=np.asarray(covariance,float)
        if (v.shape!=(2,) or R.shape!=(2,2) or not np.isfinite([*v,*R.ravel()]).all()
                or not np.allclose(R,R.T) or np.linalg.eigvalsh(R).min()<0):
            raise ValueError('finite velocity and positive-semidefinite velocity covariance required')
        R=R.copy(); R+=np.diag(np.maximum(self.config.velocity_observation_floor_std**2-np.diag(R),0))
        H=np.zeros((2,8)); H[:,2:4]=np.eye(2); H[:,4:6]=np.eye(2)
        return self._observe(H,v,R,9.21,'vio_velocity')

    def observe_vio_position(self,position,covariance):
        p=np.asarray(position,float); R=np.asarray(covariance,float)
        if (p.shape!=(2,) or R.shape!=(2,2) or not np.isfinite([*p,*R.ravel()]).all()
                or not np.allclose(R,R.T) or np.linalg.eigvalsh(R).min()<0):
            raise ValueError('finite VIO position and positive-semidefinite covariance required')
        R=R.copy(); R+=np.diag(np.maximum(self.config.position_floor_std**2-np.diag(R),0))
        H=np.zeros((2,8)); H[:,:2]=np.eye(2); H[:,6:8]=np.eye(2)
        return self._observe(H,p,R,9.21,'vio_position')

    def observe_wall(self,normal,projection,variance):
        n=np.asarray(normal,float)
        if (n.shape!=(2,) or not np.isfinite([*n,projection,variance]).all()
                or abs(np.linalg.norm(n)-1)>1e-6 or variance<=0):
            raise ValueError('unit observed normal, finite projection and positive variance required')
        H=np.zeros((1,8)); H[0,:2]=n
        return self._observe(H,np.array([projection]),np.array([[variance]]),9.,'wall')
