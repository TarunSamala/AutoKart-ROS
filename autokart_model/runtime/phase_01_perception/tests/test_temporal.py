import numpy as np

from autokart_perception.temporal import (StationaryDisplayFilter,
                                          estimate_motion_from_correspondences,
                                          estimate_rgbd_motion)
from autokart_perception.types import Cone, Detection, Frame, Intrinsics


def test_moving_or_unverified_camera_preserves_current_measurements():
    tracker=StationaryDisplayFilter(hold_ms=150,tau_ms=120)
    first=Detection((10,10,30,40),'orange',.9,0)
    cone=Cone(0,'orange',.9,(2.,1.),.05,2.24,first.box)
    boxes,metric,predicted=tracker.update([first],[cone],1_000_000_000)
    assert boxes[0].box==first.box and metric[0].xy==cone.xy and predicted==[]
    second=Detection((12,10,32,40),'orange',.8,3)
    moved=Cone(3,'orange',.8,(2.2,1.),.05,2.42,second.box)
    boxes,metric,predicted=tracker.update([second],[moved],1_050_000_000)
    assert boxes[0].box==second.box
    assert metric[0].xy==moved.xy
    assert metric[0].id==3 and predicted==[]
    boxes,metric,predicted=tracker.update([],[],1_100_000_000)
    assert boxes==[] and metric==[]
    assert predicted==[]
    assert tracker.update([],[],1_300_000_000)[2]==[]


def test_stationary_smoothing_requires_explicit_verification():
    tracker=StationaryDisplayFilter()
    first=Detection((10,10,30,40),'orange',.9,0)
    cone=Cone(0,'orange',.9,(2.,1.),.05,2.24,first.box)
    tracker.update([first],[cone],1_000_000_000,stationary_verified=True)
    second=Detection((12,10,32,40),'orange',.8,3)
    moved=Cone(3,'orange',.8,(2.2,1.),.05,2.42,second.box)
    boxes,metric,_=tracker.update([second],[moved],1_050_000_000,stationary_verified=True)
    assert 10<boxes[0].box[0]<12
    assert 2<metric[0].xy[0]<2.2
    _,_,predicted=tracker.update([],[],1_100_000_000,stationary_verified=True)
    assert predicted[0]['status']=='PREDICTED_DISPLAY_ONLY'
    assert predicted[0]['age_ms']==50


def test_unknown_motion_after_stationary_frame_stops_smoothing():
    tracker=StationaryDisplayFilter()
    first=Detection((10,10,30,40),'blue',.9,0)
    cone=Cone(0,'blue',.9,(2.,1.),.05,2.24,first.box)
    tracker.update([first],[cone],1_000_000_000,stationary_verified=True)
    moved=Detection((12,10,32,40),'blue',.8,3)
    current=Cone(3,'blue',.8,(1.5,1.),.05,1.8,moved.box)
    boxes,metric,_=tracker.update([moved],[current],1_050_000_000)
    assert boxes[0].box==moved.box
    assert metric[0].xy==current.xy
    assert tracker.update([],[],1_100_000_000)[2]==[]


def test_stationarity_cannot_be_asserted_retroactively():
    tracker=StationaryDisplayFilter()
    first=Detection((10,10,30,40),'blue',.9,0)
    cone=Cone(0,'blue',.9,(2.,1.),.05,2.24,first.box)
    tracker.update([first],[cone],1_000_000_000)
    second=Detection((12,10,32,40),'blue',.8,3)
    current=Cone(3,'blue',.8,(1.8,1.),.05,2.0,second.box)
    boxes,metric,_=tracker.update([second],[current],1_050_000_000,stationary_verified=True)
    assert boxes[0].box==second.box
    assert metric[0].xy==current.xy


def test_out_of_order_timestamp_resets_filter():
    tracker=StationaryDisplayFilter()
    box=Detection((1,1,10,10),'orange',.9,0)
    cone=Cone(0,'orange',.9,(1.,1.),.1,1.4,box.box)
    tracker.update([box],[cone],1_000_000_000)
    assert tracker.update([],[],900_000_000)[2]==[]


def test_relative_camera_motion_is_robust_but_provisional():
    rng=np.random.default_rng(101)
    intr=Intrinsics(500.,500.,320.,240.)
    previous=np.column_stack((rng.uniform(-1.,1.,80),rng.uniform(-.7,.7,80),
                              rng.uniform(2.,5.,80)))
    # A stationary scene observed after the camera moves right by 0.1 m.
    current_xyz=previous-np.array((.1,0.,0.))
    pixels=np.column_stack((intr.fx*current_xyz[:,0]/current_xyz[:,2]+intr.cx,
                            intr.fy*current_xyz[:,1]/current_xyz[:,2]+intr.cy))
    pixels+=rng.normal(0.,.25,pixels.shape)
    pixels[:10]=rng.uniform((0,0),(640,480),(10,2))
    motion=estimate_motion_from_correspondences(previous,pixels,intr,
                                                1_000_000_000,1_050_000_000)
    assert motion.status=='PROVISIONAL_CAMERA_MOTION'
    assert motion.qualified is False
    assert motion.inlier_count>=60
    assert motion.reprojection_rms_px<1.
    assert abs(motion.translation_m[0]+.1)<.02
    assert motion.image_only_translation_std_m>0


def test_relative_camera_motion_rejects_stale_and_weak_correspondences():
    intr=Intrinsics(500.,500.,320.,240.)
    points=np.column_stack((np.linspace(-1.,1.,20),np.zeros(20),np.ones(20)*3))
    pixels=np.ones((20,2))
    assert estimate_motion_from_correspondences(points,pixels,intr,
        1_000_000_000,1_000_000_000).status=='MOTION_TIMING_INVALID'
    assert estimate_motion_from_correspondences(points[:4],pixels[:4],intr,
        1_000_000_000,1_050_000_000).status=='MOTION_MATCHES_INVALID'


def test_rgbd_motion_frontend_rejects_unaligned_and_featureless_frames():
    rgb=np.zeros((48,64,3),dtype=np.uint8)
    depth=np.full((48,64),1000,dtype=np.uint16)
    intr=Intrinsics(100.,100.,32.,24.)
    def frame(time,aligned):
        sequence=1 if time==1_000_000_000 else 2
        return Frame(rgb,depth,time,0,0,sequence,sequence,intr,.001,aligned,'SYNTHETIC')
    previous=frame(1_000_000_000,True)
    assert estimate_rgbd_motion(previous,frame(1_050_000_000,False)).status=='MOTION_ALIGNMENT_INVALID'
    assert estimate_rgbd_motion(previous,frame(1_050_000_000,True)).status=='MOTION_FEATURES_INSUFFICIENT'


def test_rgbd_motion_frontend_on_synthetic_shift_remains_unqualified():
    import cv2

    rng=np.random.default_rng(123)
    image=rng.integers(0,256,(240,320,3),dtype=np.uint8)
    shifted=cv2.warpAffine(image,np.float32(((1,0,3),(0,1,0))),(320,240))
    depth=np.full((240,320),3000,dtype=np.uint16)
    intr=Intrinsics(300.,300.,160.,120.)
    first=Frame(image,depth,1_000_000_000,0,0,1,1,intr,.001,True,'SYNTHETIC')
    second=Frame(shifted,depth,1_050_000_000,0,0,2,2,intr,.001,True,'SYNTHETIC')
    motion=estimate_rgbd_motion(first,second)
    assert motion.status=='PROVISIONAL_CAMERA_MOTION'
    assert motion.qualified is False
    assert motion.inlier_count>=100
    assert abs(motion.translation_m[0]-.03)<.02
