import json
from pathlib import Path
from hashlib import sha256
import numpy as np
import pytest
from tools.prepare_observable_reference import Excitation,add_excitation,excitation_motion,build,dense_scaled_reference


SOURCE=Path('artifacts/published-course-efficient-v1/trajectory.csv')
SETTINGS=Path('configs/sky_beta_first_completion.json')
CONTRACT=Path('artifacts/sky-beta-estimated-v4/airframe-contract.json')


@pytest.mark.local_artifacts(str(SOURCE))
def test_parallax_motion_is_bounded_and_endpoint_rest_is_preserved():
    original=np.genfromtxt(SOURCE,delimiter=',',skip_header=1)
    rows,metrics=add_excitation(original,time_scale=1.5)
    nominal=dense_scaled_reference(original,1.5)
    np.testing.assert_allclose(rows[[0,-1],1:4],original[[0,-1],1:4],atol=1e-12)
    np.testing.assert_allclose(rows[[0,-1],4:7],0,atol=1e-12)
    assert np.max(abs(rows[:,1]-nominal[:,1]))<=.015+1e-12
    np.testing.assert_equal(rows[:,[2,3,7,8,9,10]],nominal[:,[2,3,7,8,9,10]])
    at=np.searchsorted(nominal[:,0],original[:,0]*1.5)
    np.testing.assert_allclose(nominal[at,1:4],original[:,1:4],atol=1e-12)
    np.testing.assert_allclose(nominal[at,4:7],original[:,4:7]/1.5,atol=1e-12)
    assert np.max(np.diff(rows[:,0]))<=.02+1e-10
    assert metrics['maximum_speed_mps']<=.32
    assert metrics['maximum_sampled_acceleration_mps2']<=.25
    assert metrics['maximum_sampled_jerk_mps3']<=.5
    assert rows[-1,0]==pytest.approx(original[-1,0]*1.5)


def test_analytic_excitation_derivatives_match_numerical_motion():
    t=np.linspace(0,30,30001);v=excitation_motion(t,30)
    for order in range(3):
        observed=np.gradient(v[:,order],t)
        mask=(t>.02)&(t<29.98)&(abs(t-6)>.02)&(abs(t-24)>.02)
        np.testing.assert_allclose(observed[mask],v[mask,order+1],atol=2e-6)
    for kwargs in ({'amplitude_m':.02},{'period_s':1},{'fade_s':0},{'amplitude_m':np.nan}):
        with pytest.raises(ValueError):Excitation(**kwargs)


@pytest.mark.local_artifacts(str(SOURCE), str(CONTRACT))
def test_generation_pins_source_settings_and_preserves_existing_evidence(tmp_path):
    out=tmp_path/'candidate';report=build(SOURCE,SETTINGS,CONTRACT,out)
    assert report['required_caller_time_scale']==1 and report['flight_completed'] is False
    assert report['input_hashes'][str(SOURCE)]==sha256(SOURCE.read_bytes()).hexdigest()
    assert json.loads((out/'source-settings.json').read_text())==json.loads(SETTINGS.read_text())
    assert (out/'source-trajectory.csv').read_bytes()==SOURCE.read_bytes()
    with pytest.raises(FileExistsError):build(SOURCE,SETTINGS,CONTRACT,out)


@pytest.mark.local_artifacts(
    str(SOURCE), str(CONTRACT),
    'artifacts/published-course-efficient-v1/ordered-gates.json',
)
def test_gate_slab_suppresses_excitation_without_changing_nominal_gate_intersection():
    original=np.genfromtxt(SOURCE,delimiter=',',skip_header=1);nominal=dense_scaled_reference(original,1.5)
    gates=json.loads(Path('artifacts/published-course-efficient-v1/ordered-gates.json').read_text())['gates']
    radius=json.loads(CONTRACT.read_text())['collision_radius_m']
    changed,metrics=add_excitation(original,time_scale=1.5,gates=gates,radius=radius)
    for gate in gates:
        d=(nominal[:,1:4]-np.asarray(gate['center']))@np.asarray(gate['normal'])
        near=abs(d)<=radius+.015+.005
        np.testing.assert_allclose(changed[near,1:4],nominal[near,1:4],atol=1e-12)
    assert metrics['gate_slab_fade_applied'] and 0<metrics['fraction_samples_with_excitation_above_1mm']<1
