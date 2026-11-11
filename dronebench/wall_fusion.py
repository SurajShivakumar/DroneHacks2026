"""Offline XY filtering experiment; not a PX4 odometry message or pose fix.

Caller supplies declared initialization, odometry increments and independent
scalar wall observations. This omits sensor timing, cross-correlations, map
bias, height/attitude/velocity and estimator resets: no flight qualification.
"""
import numpy as np


class WallPositionFilter:
    def __init__(self, position, covariance, *, process_std_m_sqrt_s=.03, systematic_std_m=.03):
        self.position=np.asarray(position,float).copy(); self.covariance=np.asarray(covariance,float).copy()
        if (self.position.shape!=(2,) or self.covariance.shape!=(2,2) or
                not np.isfinite([*self.position,*self.covariance.ravel(),process_std_m_sqrt_s,systematic_std_m]).all()
                or not np.allclose(self.covariance,self.covariance.T) or np.linalg.eigvalsh(self.covariance).min()<=0
                or process_std_m_sqrt_s<=0 or systematic_std_m<=0):
            raise ValueError('finite XY, positive definite covariance and positive uncertainty bounds required')
        self.process_variance=process_std_m_sqrt_s**2; self.floor_variance=systematic_std_m**2

    def predict(self,delta_xy,dt_s):
        delta=np.asarray(delta_xy,float)
        if delta.shape!=(2,) or not np.isfinite([*delta,dt_s]).all() or dt_s<=0:
            raise ValueError('finite odometry increment and positive elapsed source time required')
        self.position+=delta; self.covariance+=np.eye(2)*self.process_variance*dt_s

    def update(self,normal,projection,variance):
        n=np.asarray(normal,float)
        if (n.shape!=(2,) or not np.isfinite([*n,projection,variance]).all()
                or abs(np.linalg.norm(n)-1)>1e-6 or variance<=0):
            raise ValueError('unit wall normal, finite projection and positive variance required')
        innovation=projection-float(n@self.position)
        residual_variance=float(n@self.covariance@n+variance)
        if innovation**2>9*residual_variance:return False
        gain=self.covariance@n/residual_variance
        self.position+=gain*innovation
        J=np.eye(2)-np.outer(gain,n)
        self.covariance=J@self.covariance@J.T+np.outer(gain,gain)*variance
        self.covariance=(self.covariance+self.covariance.T)/2
        # A systematic floor prevents repeated correlated pixels/frames from
        # pretending to eliminate all calibration/association uncertainty.
        self.covariance+=np.diag(np.maximum(self.floor_variance-np.diag(self.covariance),0))
        return True
