import numpy as np
from autokart_perception.mount import mount_from_config
from autokart_perception.depth import localize_diagnostic
from autokart_perception.synthetic import Synthetic
from autokart_perception.types import Detection


def test_measured_level_mount_and_rejections():
    config={'camera_mount':{'height_reference':'optical_center','height_m':.18,
        'x_forward_m':0,'y_left_m':0,'pitch_down_deg':0,'roll_deg':0,'yaw_left_deg':0}}
    matrix,status=mount_from_config(config)
    assert status=='MOUNT_CONFIGURED_UNVERIFIED'
    assert np.allclose(matrix[:3,:3],[[0,0,1],[-1,0,0],[0,-1,0]])
    assert np.allclose(matrix[:3,3],[0,0,.18])
    config['camera_mount']['height_reference']='housing'
    assert mount_from_config(config)[1]=='MOUNT_CALIBRATION_INVALID'
    assert mount_from_config({'camera_mount':None})[1]=='MOUNT_CALIBRATION_UNAVAILABLE'


def test_reported_pitch_sign_and_optical_axes_from_canonical_configuration():
    from autokart_perception import default_config
    config=default_config()
    assert config['camera_mount']['height_m']==.8382
    assert config['camera_mount']['pitch_down_deg']==17.8
    assert config['operator_measurements']['camera_to_front_bumper_horizontal_m']==1.0668
    assert config['operator_measurements']['bumper_width_m']==1.016
    matrix,status=mount_from_config(config)
    optical_forward=matrix[:3,:3]@np.array([0.,0.,1.])
    assert status=='MOUNT_CONFIGURED_UNVERIFIED'
    assert optical_forward[0]>0 and optical_forward[2]<0
    assert np.allclose(matrix[:3,:3]@np.array([1.,0.,0.]),[0.,-1.,0.],atol=1e-12)


def test_cone_specific_depth_reason_even_without_mount():
    frame=Synthetic().read()
    frame.depth[:]=0
    detection=Detection((300,200,340,300),'orange',.9,1)
    cfg={'depth_min_m':.3,'depth_max_m':4,'min_depth_pixels':12,'max_depth_mad_m':.25,
         'cone_radius_uncertainty_m':.15,'max_ground_contact_offset_m':.5}
    cone,detail=localize_diagnostic(frame,detection,np.empty((0,0)),cfg)
    assert cone is None and detail['depth_status']=='INSUFFICIENT_VALID_DEPTH'
    assert detail['valid_depth_pixels']==0 and detail['required_depth_pixels']==12
    frame.depth[:]=2000
    cone,detail=localize_diagnostic(frame,detection,np.empty((0,0)),cfg)
    assert cone is None and detail['depth_status']=='DEPTH_OK'
    assert detail['valid_depth_pixels']>12 and detail['coherent_depth_pixels']>12
    assert detail['depth_mad_m']==0
    assert detail['localization_status']=='MOUNT_CALIBRATION_UNAVAILABLE'
    matrix,status=mount_from_config({'camera_mount':{'height_reference':'optical_center',
        'height_m':.18,'x_forward_m':0,'y_left_m':0,'pitch_down_deg':0,
        'roll_deg':0,'yaw_left_deg':0}})
    cone,detail=localize_diagnostic(frame,detection,matrix,cfg)
    assert detail['sample_roi']==[310,265,330,290]
    assert detail['valid_depth_fraction']==1
    assert 'surface_xyz_m' in detail and 'ground_contact_offset_m' in detail
    assert detail['ground_candidate_xy_m'][0]>0
