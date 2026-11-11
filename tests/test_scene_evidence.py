import io
import json
import pytest
from sim.scene_evidence import json_messages, model_samples


def message(ns,x=0):
    return {'header':{'stamp':{'sec':'1','nsec':ns}},'pose':[
        {'name':'sky','position':{'x':x},'orientation':{'w':1}},
        {'name':'base_link','position':{'z':99},'orientation':{'w':1}}]}


def test_transport_chunk_boundaries_and_actual_model_selection():
    raw='\n'.join(json.dumps(message(ns,x)) for ns,x in [(0,2),(0,2),(5,3)])
    rows=list(model_samples(json_messages(io.StringIO(raw),chunk_size=3),'sky'))
    assert rows==[[1,2,0,0,1,0,0,0],[1.000000005,3,0,0,1,0,0,0]]


def test_partial_capture_and_regression_are_not_silent_success():
    with pytest.raises(ValueError,match='invalid raw'):
        list(json_messages(io.StringIO('{"header":')))
    with pytest.raises(ValueError,match='regressed'):
        list(model_samples([message(2),message(1)],'sky'))
    with pytest.raises(ValueError,match='ambiguous'):
        list(model_samples([message(2)],'not_the_drone'))
