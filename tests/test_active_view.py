from types import SimpleNamespace
from math import pi
import numpy as np
from dronebench.active_view import choose_view_yaw
from dronebench.depth_mapping import Pinhole
from dronebench.published_raycast import build_published_raycast


def test_viewing_corner_gives_two_normal_directions_instead_of_single_wall():
    course=SimpleNamespace(box_c=np.empty((0,3)),box_u=np.empty((0,2)),box_h=np.empty((0,3)),box_names=[])
    scene=build_published_raycast(course,published_provenance='synthetic corner',net_bounds=(-4,2,-4,2,0,3))
    transform=np.eye(4); transform[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
    camera=Pinhole(640,400,381.,381.,320.,200.)
    choice=choose_view_yaw(scene,camera,transform,[0,0,1.],0,candidate_offsets=[0,pi/4],rotation_cost=0)
    assert abs(choice.yaw-pi/4)<1e-8
    assert choice.candidates[0]['minimum_normal_information']==0
    assert choice.minimum_normal_information>.1
