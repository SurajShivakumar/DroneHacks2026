import numpy as np
import pytest

from tools.import_sky_beta_cad import CAD_TO_FLU, aggregate_mass_properties
from sim.sky_beta_model import check_properties


def test_cad_camera_forward_is_flu_forward_without_reflection():
    assert np.allclose(CAD_TO_FLU @ [0, -1, 0], [1, 0, 0])
    assert np.allclose(CAD_TO_FLU @ [0, 0, 1], [0, 0, 1])
    assert np.linalg.det(CAD_TO_FLU) == 1


def test_component_parallel_axis_inertia_is_about_actual_aggregate_com():
    parts = [{'mass_kg': 1., 'com_flu_m': [x, 0, 0],
              'inertia_about_part_com_flu_kg_m2': np.diag([.1, .2, .3]).tolist()}
             for x in (-1., 1.)]
    mass, com, inertia = aggregate_mass_properties(parts)
    assert mass == 2 and np.allclose(com, [0, 0, 0])
    assert np.allclose(inertia, np.diag([.2, 2.4, 2.6]))


def test_positive_diagonal_alone_does_not_make_a_physical_inertia():
    properties = {'cad_mass_kg_excluding_omissions': 1., 'com_flu_m': [0, 0, 0],
                  'inertia_about_com_flu_kg_m2': np.diag([.1, .1, 1.]).tolist()}
    with pytest.raises(ValueError, match='triangle inequality'):
        check_properties(properties)


def test_hover_command_inverts_native_quadratic_output_and_spinning_idle():
    from sim.sky_beta_flight_model import normalized_thrust_for_force
    force=np.array([3.603466,2.561488,3.609796,2.555158])
    coefficient=10/1500**2
    command=normalized_thrust_for_force(force,coefficient,150,1500)
    actual_speed=150+(1500-150)*np.sqrt(command)
    assert np.allclose(coefficient*actual_speed**2,force)
    with pytest.raises(ValueError,match='envelope'):
        normalized_thrust_for_force(0,coefficient,150,1500)
