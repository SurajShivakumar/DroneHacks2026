import numpy as np
from dronebench.gate_geometry import whole_crossing


def crossing(x):
    return whole_crossing(np.arange(len(x)),np.c_[x,np.zeros((len(x),2))],
        np.tile([1,0,0,0],(len(x),1)),centre=[0,0,0],normal=np.array([1,0,0]),
        lateral=np.array([0,1,0]),half_width=1,half_height=1,radius=.28,body_half_height=.13)


def test_centre_crossing_does_not_count_as_whole_aircraft():
    result=crossing([-.5,-.1,.1])
    assert not result['whole_crossed']
    result=crossing([-.5,-.1,.1,.3])
    assert result['whole_crossed'] and result['entire_aircraft_beyond_plane_source_s']==3
    assert result['conservative_rectangle_clearance']


def test_adjacent_gate_entry_can_overlap_previous_whole_completion():
    result=whole_crossing(np.arange(5),np.c_[[-.5,-.2,.1,.2,.3],np.zeros((5,2))],
        np.tile([1,0,0,0],(5,1)),centre=[0,0,0],normal=np.array([1,0,0]),
        lateral=np.array([0,1,0]),half_width=1,half_height=1,radius=.28,
        body_half_height=.13,start_index=3)
    assert result['whole_crossed'] and result['sample_index']==4
