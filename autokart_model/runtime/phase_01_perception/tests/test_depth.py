import numpy as np
import pytest
from autokart_perception.depth import sample_depth, localize, validate_frame, DepthError
from autokart_perception.synthetic import Synthetic
from autokart_perception.types import Detection

CFG={'depth_min_m':.3,'depth_max_m':20,'min_depth_pixels':12,'max_depth_mad_m':.25,'cone_radius_uncertainty_m':.15,'max_ground_contact_offset_m':.5}

def test_sampling_and_invalid_depth():
    d=np.full((480,640),2000,np.uint16)
    assert sample_depth(d,(100,100,160,300),.001)[0]==2
    d[:]=0
    assert sample_depth(d,(100,100,160,300),.001) is None
    d[230:280,110:150]=30000
    assert sample_depth(d,(100,100,160,300),.001) is None

def test_backprojection_coordinate_transform():
    f=Synthetic().read();f.depth[:]=2000
    # Optical z -> kart x, optical x -> kart -y, optical y -> kart -z.
    t=np.array([[0,0,1,0],[ -1,0,0,0],[0,-1,0,.2],[0,0,0,1]],float)
    d=Detection((300,200,340,300),'orange',.9,7)
    c=localize(f,d,t,CFG)
    assert c is not None and np.allclose(c.xy,(1.6949,0.),atol=.01) and c.sigma_m>=.15

def test_alignment_and_timestamp_rejection():
    f=Synthetic().read();f.aligned=False
    with pytest.raises(DepthError,match='DEPTH_ALIGNMENT_UNAVAILABLE'):validate_frame(f)
    f.aligned=True;f.rgb_timestamp=100;f.depth_timestamp=200
    with pytest.raises(DepthError,match='TIMESTAMP_MISMATCH'):validate_frame(f)
    f.rgb_timestamp=0;f.depth_timestamp=0;f.source='LIVE';f.timestamp_ns=1
    with pytest.raises(DepthError,match='STALE_FRAME'):validate_frame(f,now_ns=1_000_000_000)

def test_live_rgb_depth_sequence_mismatch_rejected():
    f=Synthetic().read()
    f.source='LIVE';f.rgb_sequence=41;f.depth_sequence=42
    with pytest.raises(DepthError,match='RGB_DEPTH_FRAME_MISMATCH'):
        validate_frame(f,now_ns=f.timestamp_ns)

def test_replay_rgb_depth_sequence_mismatch_rejected():
    f=Synthetic().read()
    f.source='REPLAY';f.rgb_sequence=41;f.depth_sequence=42
    with pytest.raises(DepthError,match='RGB_DEPTH_FRAME_MISMATCH'):
        validate_frame(f,now_ns=f.timestamp_ns)

def test_foreground_mode_selected_over_background():
    d=np.full((480,640),4000,np.uint16)
    d[240:275,118:142]=2000
    s=sample_depth(d,(100,100,160,300),.001)
    assert s is not None and s[0]==2.0 and s[2]>=.2

def test_nonrigid_mount_rejected():
    from autokart_perception.depth import valid_mount
    m=np.eye(4);m[2,3]=.5
    assert valid_mount(m)
    m[0,0]=2
    assert not valid_mount(m)


def test_clipped_cone_retains_surface_position_without_ground_contact():
    from autokart_perception.mount import mount_from_config
    from autokart_perception import default_config
    from autokart_perception.depth import localize_diagnostic
    f=Synthetic().read(); f.depth[:]=800
    cone,detail=localize_diagnostic(f,Detection((80,210,220,480),'orange',.9,3),
                                    mount_from_config(default_config())[0],default_config())
    assert cone is None
    assert detail['localization_status']=='PROVISIONAL_3D_POSITION'
    assert detail['position_kind']=='PROVISIONAL_3D_POSITION'
    assert detail['ground_contact_status']=='UNAVAILABLE_CONE_FOOT_CLIPPED'
    assert detail['depth_status']=='DEPTH_OK'
    assert len(detail['optical_xyz_m'])==3
    assert len(detail['nominal_body_xyz_m'])==3
    assert detail['surface_uncertainty_m']>0
    assert 'ground_xy_m' not in detail
    assert detail['sample_roi'][1] < 350


def test_clipped_cone_invalid_depth_stays_invalid():
    from autokart_perception.mount import mount_from_config
    from autokart_perception import default_config
    from autokart_perception.depth import localize_diagnostic
    f=Synthetic().read(); f.depth[:]=0
    cone,detail=localize_diagnostic(f,Detection((80,210,220,480),'orange',.9,3),
                                    mount_from_config(default_config())[0],default_config())
    assert cone is None
    assert detail['position_kind']=='INVALID_DEPTH'
    assert detail['depth_status']=='INSUFFICIENT_VALID_DEPTH'
    assert 'optical_xyz_m' not in detail
