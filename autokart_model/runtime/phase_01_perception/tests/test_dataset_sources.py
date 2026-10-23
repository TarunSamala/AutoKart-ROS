import cv2
import numpy as np

from autokart_perception.dataset_sources import audit_overlap


def test_brt_derived_images_excluded_and_exact_overlap_found(tmp_path):
    fsoco=tmp_path/'fsoco';brt=tmp_path/'brt';fsoco.mkdir();brt.mkdir()
    image=np.full((12,12,3),47,np.uint8)
    assert cv2.imwrite(str(fsoco/'one.png'),image)
    (brt/'copy.png').write_bytes((fsoco/'one.png').read_bytes())
    assert cv2.imwrite(str(brt/'other.png'),np.full((12,12,3),91,np.uint8))
    report=audit_overlap(fsoco,brt)
    assert report['confirmed_identical_images']==1
    assert report['brt_images_excluded_from_independent_count']==2
