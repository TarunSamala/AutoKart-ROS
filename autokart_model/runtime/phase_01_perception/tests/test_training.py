import hashlib
import csv
import json

import cv2
import numpy as np
import pytest
import yaml

from autokart_perception.detector import DetectorError, Yolo11OnnxDetector
from autokart_perception.training import MIN_TRAIN_CONE_IMAGES, audit_dataset, train


def dataset(tmp_path, train_count=3):
    session_rows=[]
    for split, count in [('train', train_count), ('val', 1), ('test', 1)]:
        images=tmp_path/'images'/split;labels=tmp_path/'labels'/split
        images.mkdir(parents=True);labels.mkdir(parents=True)
        for i in range(count):
            image=np.full((16,16,3), 20+i+({'train':0,'val':60,'test':120}[split]), np.uint8)
            assert cv2.imwrite(str(images/f'{i}.png'), image)
            (labels/f'{i}.txt').write_text('0 0.5 0.5 0.5 0.5\n')
            session_rows.append((f'images/{split}/{i}.png',f'{split}-session'))
    with (tmp_path/'sessions.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(('image','session_id'));writer.writerows(session_rows)
    config={'path':str(tmp_path),'train':'images/train','val':'images/val',
            'test':'images/test','names':['orange'],'source':'unit fixture','license':'test-only',
            'session_manifest':'sessions.csv'}
    path=tmp_path/'data.yaml';path.write_text(yaml.safe_dump(config))
    return path


def without_manifest(data):
    config=yaml.safe_load(data.read_text())
    config.pop('session_manifest')
    data.write_text(yaml.safe_dump(config))


def test_experimental_audit_without_manifest_keeps_image_and_label_checks(tmp_path):
    data=dataset(tmp_path)
    without_manifest(data)
    report=audit_dataset(data,minimum=1,stage='experimental')
    assert report['status']=='EXPERIMENTAL_READY'
    assert report['errors']==[]
    assert report['session_independence']=='NOT_VERIFIED'
    assert report['sessions_by_split']=={'train':[],'val':[],'test':[]}
    assert report['session_counts']=={'train':0,'val':0,'test':0}
    assert report['splits']['train']['cone_positive_images']==3
    assert report['provenance']=={'source':'unit fixture','license':'test-only'}


def test_qualified_audit_requires_manifest(tmp_path):
    data=dataset(tmp_path)
    without_manifest(data)
    with pytest.raises(ValueError,match='SESSION_MANIFEST_REQUIRED'):
        audit_dataset(data)


def test_experimental_audit_without_manifest_rejects_duplicate_across_splits(tmp_path):
    data=dataset(tmp_path)
    without_manifest(data)
    (tmp_path/'images/val/0.png').write_bytes((tmp_path/'images/train/0.png').read_bytes())
    report=audit_dataset(data,minimum=1,stage='experimental')
    assert report['status']=='DATASET_REJECTED'
    assert any('val:0.png:DUPLICATE_IMAGE_OF:train/0.png' in error for error in report['errors'])


def test_experimental_audit_without_manifest_rejects_missing_label(tmp_path):
    data=dataset(tmp_path)
    without_manifest(data)
    (tmp_path/'labels/train/0.txt').unlink()
    report=audit_dataset(data,minimum=1,stage='experimental')
    assert report['status']=='DATASET_REJECTED'
    assert 'train:0.png:MISSING_LABEL' in report['errors']


@pytest.mark.parametrize('label,error', [
    ('0 0.9 0.5 0.5 0.5\n','BOX_OUT_OF_BOUNDS'),
    ('1 0.5 0.5 0.5 0.5\n','INVALID_CLASS_OR_BOX'),
])
def test_experimental_audit_without_manifest_rejects_bad_boxes_and_classes(tmp_path,label,error):
    data=dataset(tmp_path)
    without_manifest(data)
    (tmp_path/'labels/train/0.txt').write_text(label)
    report=audit_dataset(data,minimum=1,stage='experimental')
    assert report['status']=='DATASET_REJECTED'
    assert any(error in item for item in report['errors'])


def test_experimental_audit_without_manifest_rejects_undecodable_image(tmp_path):
    data=dataset(tmp_path)
    without_manifest(data)
    (tmp_path/'images/train/0.png').write_bytes(b'not an image')
    report=audit_dataset(data,minimum=1,stage='experimental')
    assert report['status']=='DATASET_REJECTED'
    assert any('UNDECODABLE_IMAGE' in error for error in report['errors'])


@pytest.mark.parametrize('missing', ['source','license'])
def test_experimental_audit_without_manifest_requires_provenance(tmp_path,missing):
    data=dataset(tmp_path)
    without_manifest(data)
    config=yaml.safe_load(data.read_text())
    config.pop(missing)
    data.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError,match='DATASET_SOURCE_AND_LICENSE_REQUIRED'):
        audit_dataset(data,minimum=1,stage='experimental')


def test_experimental_audit_validates_supplied_manifest(tmp_path):
    data=dataset(tmp_path)
    report=audit_dataset(data,minimum=1,stage='experimental')
    assert report['status']=='EXPERIMENTAL_READY'
    assert report['session_independence']=='VERIFIED_FROM_MANIFEST'
    assert report['sessions_by_split']['test']==['test-session']
    (tmp_path/'sessions.csv').write_text('image,session_id\nimages/train/0.png,\n')
    with pytest.raises(ValueError,match='INVALID_SESSION_MANIFEST_ENTRY'):
        audit_dataset(data,minimum=1,stage='experimental')


def test_experimental_audit_rejects_incomplete_supplied_manifest(tmp_path):
    data=dataset(tmp_path)
    lines=(tmp_path/'sessions.csv').read_text().splitlines()
    (tmp_path/'sessions.csv').write_text('\n'.join(lines[1:])+'\n')
    with pytest.raises(ValueError,match='INVALID_SESSION_MANIFEST'):
        audit_dataset(data,minimum=1,stage='experimental')


def test_audit_counts_distinct_cone_positive_images_and_blocks_real_minimum(tmp_path):
    data=dataset(tmp_path)
    report=audit_dataset(data,minimum=1,stage='experimental')
    assert report['status']=='EXPERIMENTAL_READY'
    assert report['splits']['train']['cone_positive_images']==3
    assert report['splits']['train']['boxes_by_class']['orange']==3
    assert len(report['dataset_sha256'])==64
    assert audit_dataset(data)['status']=='DATASET_REJECTED'
    assert MIN_TRAIN_CONE_IMAGES==19385
    with pytest.raises(ValueError,match='DATASET_REJECTED'):
        train(data,tmp_path/'new-run')
    assert not (tmp_path/'new-run').exists()


def test_audit_rejects_duplicates_and_bad_boxes(tmp_path):
    data=dataset(tmp_path)
    (tmp_path/'images/val/0.png').write_bytes((tmp_path/'images/train/0.png').read_bytes())
    report=audit_dataset(data,minimum=1,stage='experimental')
    assert any('DUPLICATE_IMAGE' in error for error in report['errors'])
    (tmp_path/'images/val/0.png').write_bytes(cv2.imencode('.png',np.full((16,16,3),199,np.uint8))[1].tobytes())
    (tmp_path/'labels/train/0.txt').write_text('0 0.9 0.5 0.5 0.5\n')
    report=audit_dataset(data,minimum=1,stage='experimental')
    assert any('BOX_OUT_OF_BOUNDS' in error for error in report['errors'])
    assert report['splits']['train']['cone_positive_images']==2


def test_audit_requires_source_and_license(tmp_path):
    data=dataset(tmp_path)
    config=yaml.safe_load(data.read_text());config.pop('license')
    data.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError,match='DATASET_SOURCE_AND_LICENSE_REQUIRED'):
        audit_dataset(data,minimum=1,stage='experimental')


def test_session_leakage_and_experimental_stage(tmp_path):
    data=dataset(tmp_path)
    report=audit_dataset(data,minimum=1,stage='experimental')
    assert report['sessions_by_split']['test']==['test-session']
    lines=(tmp_path/'sessions.csv').read_text().replace('test-session','train-session')
    (tmp_path/'sessions.csv').write_text(lines)
    report=audit_dataset(data,minimum=1,stage='experimental')
    assert any('SESSION_SPLIT_LEAKAGE' in error for error in report['errors'])
    assert report['status']=='DATASET_REJECTED'
    assert report['session_independence']=='NOT_VERIFIED'
    qualified=audit_dataset(data)
    assert qualified['status']=='DATASET_REJECTED'
    assert any('SESSION_SPLIT_LEAKAGE' in error for error in qualified['errors'])
    assert 'INSUFFICIENT_TRAIN_CONE_IMAGES' in qualified['errors']
    with pytest.raises(ValueError,match='QUALIFIED_MINIMUM_CANNOT_BE_REDUCED'):
        audit_dataset(data,minimum=1)


def test_experimental_train_runs_audit_then_reports_missing_dependencies(tmp_path):
    data=dataset(tmp_path)
    with pytest.raises(RuntimeError,match='TRAINING_DEPENDENCIES_MISSING'):
        train(data,tmp_path/'run',stage='experimental')
    assert not (tmp_path/'run').exists()


def test_yolo11_manifest_class_mapping_and_raw_nms(tmp_path,monkeypatch):
    model=tmp_path/'model.onnx';model.write_bytes(b'mock model identity')
    manifest={'format':'ultralytics_yolo11_raw_onnx','model_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),
              'input_size':640,'classes':['blue','orange'],'train_cone_images':19385}
    model.with_suffix('.json').write_text(json.dumps(manifest))
    class FakeNet:
        def setInput(self,blob):self.blob=blob
        def forward(self):
            rows=np.array([[320,320,100,120,.9,.05],[320,320,100,120,.05,.9]],dtype=np.float32)
            return rows.T[None,:,:]
    network=FakeNet()
    monkeypatch.setattr(cv2.dnn,'readNetFromONNX',lambda path:network)
    detector=Yolo11OnnxDetector(model)
    image=np.full((480,640,3),[10,20,30],np.uint8)
    detections=detector.detect(image)
    assert network.blob.shape==(1,3,640,640)
    assert network.blob[0, :, 320, 320] == pytest.approx([30/255, 20/255, 10/255])
    assert [d.label for d in detections]==['blue','orange']
    assert all(d.box==(270,180,370,300) for d in detections)
    assert detector.last_counts == {'raw_output_rows':2,'above_confidence':2,
                                   'valid_boxes':2,'after_nms':2}
    duplicates=np.array([[320,320,100,120,.9,.05],
                         [320,320,100,120,.8,.05]],dtype=np.float32)
    assert len(detector.postprocess(duplicates,(640,480,1.0,0,80)))==1
    assert detector.last_counts['after_nms']==1
    assert detector.verified_checkpoint is False
    assert detector.postprocess(*detector.infer(image),confidence=.95)==[]
    model.write_bytes(b'changed')
    with pytest.raises(DetectorError,match='MODEL_CHECKSUM_MISMATCH'):
        Yolo11OnnxDetector(model)


def test_experimental_manifest_explicitly_selectable(tmp_path,monkeypatch):
    model=tmp_path/'model.onnx';model.write_bytes(b'experimental model')
    model.with_suffix('.json').write_text(json.dumps({'format':'ultralytics_yolo11_raw_onnx',
        'model_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),'input_size':640,
        'classes':['orange'],'train_cone_images':3,'stage':'experimental',
        'status':'EXPERIMENTAL_HP60C_VALIDATION_PENDING'}))
    monkeypatch.setattr(cv2.dnn,'readNetFromONNX',lambda path:object())
    detector=Yolo11OnnxDetector(model)
    assert detector.stage=='experimental' and not detector.verified_checkpoint
