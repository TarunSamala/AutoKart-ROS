"""SYNTHETIC geometry fixtures only; no physical calibration or camera claims."""
from dataclasses import replace
import pytest
from autokart_perception_adapter.depth_obstacles import DepthObstacleConfig,extract_depth_obstacles


def configuration(**changes):
    # Invented test camera 1m over an invented flat floor; optical Z -> base X,
    # optical X -> -base Y, optical Y -> -base Z. No physical values are implied.
    c=DepthObstacleConfig(image_width=2,image_height=2,fx=2.,fy=2.,cx=.5,cy=0.,
        depth_scale_m=1.,min_depth_m=.1,max_depth_m=5.,
        T_base_camera=((0.,0.,1.,0.),(-1.,0.,0.,0.),(0.,-1.,0.,1.),(0.,0.,0.,1.)),
        calibration_hash='SYNTHETIC_ONLY_fixture',roi=(0,1,2,2),
        ground_z_m=0.,ground_tolerance_m=.05,obstacle_min_height_m=.2,
        obstacle_max_height_m=2.,corridor_near_m=1.5,corridor_far_m=2.5,
        corridor_half_width_m=1.,coverage_cell_m=1.,minimum_samples_per_cell=1,
        maximum_invalid_fraction=0.)
    return replace(c,**changes)


def extract(depth=None,config=None,**flags):
    return extract_depth_obstacles(depth if depth is not None else [[2.,2.],[2.,2.]],
        config or configuration(),calibration_verified=flags.get('calibration',True),
        depth_alignment_verified=flags.get('alignment',True))


def test_synthetic_observed_ground_has_no_raised_hit():
    result=extract()
    assert result['valid'] and result['coverage_complete']
    assert result['obstacles']==[]
    assert result['measured_point_count']==2


def test_synthetic_raised_surfaces_yield_actual_cluster_geometry():
    result=extract(config=configuration(roi=(0,0,2,2)))
    assert result['valid'] and result['coverage_complete']
    obstacle=result['obstacles'][0]
    assert obstacle['point_count']==2
    assert obstacle['x_m']==pytest.approx(2.)
    assert obstacle['y_m']==pytest.approx(0.)
    assert obstacle['min_z_m']==pytest.approx(1.)
    assert obstacle['radius_m']>0
    assert obstacle['observed_surface_only'] is True


def test_synthetic_holes_cannot_claim_empty_corridor():
    result=extract([[2.,2.],[0.,2.]])
    assert result['valid'] and not result['coverage_complete']
    assert 'UNKNOWN_CORRIDOR_CELLS' in result['reason_codes']
    assert 'INVALID_DEPTH_COVERAGE' in result['reason_codes']
    assert result['invalid_depth_fraction']==.5


def test_observed_near_floor_does_not_cover_unseen_far_region():
    result=extract(config=configuration(corridor_far_m=4.5))
    assert not result['coverage_complete']
    assert len(result['unknown_cells'])==4


@pytest.mark.parametrize('flag,value', [('calibration',False),('calibration','true'),('alignment',False),('alignment',1)])
def test_qualification_requires_explicit_verified_boolean(flag,value):
    result=extract(**{flag:value})
    assert not result['valid'] and not result['coverage_complete']
    assert result['obstacles']==[]


@pytest.mark.parametrize('value',[float('nan'),float('inf'),-1.,0.])
def test_invalid_depth_never_proves_free_space(value):
    assert not extract([[2.,2.],[value,value]])['coverage_complete']


@pytest.mark.parametrize('matrix',[
    ((2.,0.,0.,0.),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.)),
    ((-1.,0.,0.,0.),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.)),
    ((1.,0.,0.,0.),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,0.)),
    ((float('nan'),0.,0.,0.),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.)),
    None])
def test_wrong_transform_rejected(matrix):
    result=extract(config=configuration(T_base_camera=matrix))
    assert not result['valid'] and not result['coverage_complete']


def test_missing_calibration_identity_rejected():
    assert not extract(config=configuration(calibration_hash=''))['valid']


def test_nonfinite_intrinsics_rejected():
    assert not extract(config=configuration(fx=float('nan')))['valid']


def test_wrong_dimensions_rejected():
    assert not extract([[2.]])['valid']


def test_unclassified_heights_remain_unknown():
    result=extract(config=configuration(ground_z_m=.1))
    assert not result['coverage_complete']
    assert 'UNCLASSIFIED_HEIGHT_RETURNS' in result['reason_codes']


def test_input_depth_remains_unchanged():
    data=[[2.,2.],[2.,2.]]
    extract(data)
    assert data==[[2.,2.],[2.,2.]]
