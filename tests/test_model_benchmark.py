import json
from pathlib import Path

import pytest

from tools.model_benchmark import checkpoint_hashes, decode_aerovla, digest, run, validate_observations


def observations(tmp_path):
    frames=[]
    for i in range(3):
        path=tmp_path/f'{i}.png';path.write_bytes(bytes([i]))
        frames.append({'path':str(path),'sha256':digest(path),'stamp_ns':i*100000000})
    return {'instruction':'Fly straight ahead and find the exit.', 'front_history':frames}


def test_history_and_hashes(tmp_path):
    data=observations(tmp_path)
    assert len(validate_observations(data,'openfly'))==3
    Path(data['front_history'][0]['path']).write_bytes(b'changed')
    with pytest.raises(ValueError,match='hash'): validate_observations(data,'openfly')


def test_duplicate_history_rejected(tmp_path):
    data=observations(tmp_path)
    data['front_history'][1]['stamp_ns']=0
    with pytest.raises(ValueError,match='increasing'): validate_observations(data,'openfly')


def test_aerovla_requires_real_extra_view(tmp_path):
    data=observations(tmp_path)
    with pytest.raises(ValueError,match='downward'): validate_observations(data,'aerovla')
    data['extra_down_camera_diagnostic']=True
    data['down']=data['front_history'][-1]
    with pytest.raises(ValueError,match='duplicating'): validate_observations(data,'aerovla')


def test_invalid_output_is_not_silent_landing():
    assert not decode_aerovla('Action: malformed')['valid_numeric_action']
    assert decode_aerovla('Action: 0 99 99')['forward']==0
    assert decode_aerovla('Action: 0 99 99')['yaw']==pytest.approx(1.1)
    assert decode_aerovla('Action: <LAND>')['explicit_land_token']
    assert not decode_aerovla('Action: -1 49 49')['valid_numeric_action']


def test_incomplete_weights_save_failure_without_inference(tmp_path):
    from types import SimpleNamespace
    data=observations(tmp_path)
    manifest=tmp_path/'input.json';manifest.write_text(json.dumps(data))
    staged=tmp_path/'weights';staged.mkdir()
    (staged/'model-00001-of-00003.safetensors.upload-v1').write_bytes(b'partial')
    args=SimpleNamespace(output=tmp_path/'result',strategy='openfly',checkpoint='IPEC-COMMUNITY/openfly-agent-7b',
                         observations=manifest,local_checkpoint=staged,local_base=None)
    with pytest.raises(ValueError,match='incomplete'):run(args)
    result=json.loads((args.output/'result.json').read_text())
    assert result['status']=='failed_not_flown'
    assert result['finish_time_sim_s'] is None
    assert not result['commands_published']


def test_hash_cache_invalidates_changed_file_and_excludes_uploads(tmp_path):
    import os
    root=tmp_path/'weights';root.mkdir()
    weight=root/'file.safetensors';weight.write_bytes(b'original')
    (root/'file.safetensors.upload-v1').write_bytes(b'partial')
    cache=tmp_path/'hashes.json'
    first,stats=checkpoint_hashes(root,cache)
    assert stats['fresh_full_hashes']==1
    second,stats=checkpoint_hashes(root,cache)
    assert first==second and stats['reused_full_hashes']==1
    old=weight.stat();weight.write_bytes(b'modified')
    os.utime(weight,ns=(old.st_atime_ns,old.st_mtime_ns+1000000000))
    third,stats=checkpoint_hashes(root,cache)
    assert third!=first and stats['fresh_full_hashes']==1


def test_transformers_loading_sets_preserve_samples_and_diagnostic_keys_on_export():
    from tools.model_benchmark import metadata_json_default
    record={'checkpoint_loading_info':{'missing_keys':set(),'unexpected_keys':{'b','a'}},
            'samples':[{'wall_s':.973793,'prediction':{'raw_unnormalized_action':[.1,.2,.3]}}]}
    actual=json.loads(json.dumps(record,default=metadata_json_default,allow_nan=False))
    assert actual['checkpoint_loading_info']=={'missing_keys':[],'unexpected_keys':['a','b']}
    assert actual['samples']==record['samples']
    with pytest.raises(TypeError):
        json.dumps({'unknown':object()},default=metadata_json_default)
