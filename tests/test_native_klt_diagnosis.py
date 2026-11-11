import numpy as np
import pytest
from tools.review_native_klt import preprocess,hypothesis,ambiguity_count
from tools.evaluate_klt_net_phase import grid_junction_mask


def test_distinct_similar_cost_disparities_are_ambiguity_not_multiple_depth_fixes():
    make=lambda d,e:dict(valid=np.array([True]),disparity=np.array([d]),error=np.array([e]))
    assert ambiguity_count([make(20,5),make(38,6)])==1
    assert ambiguity_count([make(20,5),make(21,5)])==0
    assert ambiguity_count([make(20,5),make(38,20)])==0


def test_known_textured_translation_recovers_positive_disparity():
    pytest.importorskip('cv2');rng=np.random.default_rng(3)
    left=rng.integers(0,256,(128,192),dtype=np.uint8);right=rng.integers(0,256,left.shape,dtype=np.uint8);right[:,:-8]=left[:,8:]
    uv=np.array([[x,y] for x in range(40,150,15) for y in range(30,100,15)],np.float32)
    result=hypothesis(left,right,uv,8)
    assert sum(result['valid'])>len(uv)*.9
    np.testing.assert_allclose(result['disparity'][result['valid']],8,atol=.1)
    with pytest.raises(ValueError):preprocess(left,'unknown')


def test_grid_junction_classifier_requires_both_line_families():
    pytest.importorskip('cv2')
    image=np.full((120,160),200,np.uint8);image[:,20::20]=0;image[20::20,:]=0
    points=np.array([[20,20],[40,60],[25,25],[70,70]],float)
    selected,counts=grid_junction_mask(image,points)
    np.testing.assert_equal(selected,[True,True,False,False])
    assert counts['vertical_lines']>0 and counts['horizontal_lines']>0
