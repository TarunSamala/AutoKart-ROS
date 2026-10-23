"""Audit cone-positive YOLO images before optional YOLO11n training.

This module does not download data or infer that augmented copies are new photos.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml

MIN_TRAIN_CONE_IMAGES = 19_385
IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.webp', '.bmp'}
ALIASES = {
    'blue': 'blue', 'blue cone': 'blue',
    'yellow': 'yellow', 'yellow cone': 'yellow',
    'orange': 'orange', 'orange cone': 'orange', 'small orange cone': 'orange',
    'large orange': 'large_orange', 'large orange cone': 'large_orange',
    'big orange cone': 'large_orange', 'orange big': 'large_orange',
    'orange big cone': 'large_orange',
    'unknown': 'unknown', 'unknown cone': 'unknown', 'other cone': 'unknown',
    'undefined': 'unknown',
    'cone': 'unknown',
}


def _classes(value):
    if isinstance(value, dict):
        if sorted(value) != list(range(len(value))):
            raise ValueError('NONCONTIGUOUS_CLASS_IDS')
        names = [value[i] for i in range(len(value))]
    elif isinstance(value, list):
        names = value
    else:
        raise ValueError('INVALID_CLASS_NAMES')
    labels = []
    for name in names:
        if not isinstance(name, str):
            raise ValueError('INVALID_CLASS_NAMES')
        canonical = ALIASES.get(name.lower().replace('_', ' ').replace('-', ' ').strip())
        if canonical is None:
            raise ValueError(f'UNSUPPORTED_CONE_CLASS:{name}')
        labels.append(canonical)
    if not labels or len(set(labels)) != len(labels):
        raise ValueError('DUPLICATE_OR_EMPTY_CONE_CLASSES')
    return names, labels


def _image_and_label_dirs(data, split, base):
    value = data.get(split)
    if not isinstance(value, str):
        raise ValueError(f'MISSING_{split.upper()}_SPLIT')
    rel = Path(value)
    if rel.is_absolute() or '..' in rel.parts or len(rel.parts) < 2 or 'images' not in rel.parts:
        raise ValueError(f'INVALID_{split.upper()}_PATH')
    # Mirror Ultralytics: swap the last 'images' component for 'labels'
    # (supports both images/<split> and <split>/images layouts).
    i = len(rel.parts) - 1 - rel.parts[::-1].index('images')
    image_dir = (base / rel).resolve()
    label_dir = (base / Path(*rel.parts[:i], 'labels', *rel.parts[i + 1:])).resolve()
    if not image_dir.is_dir() or not label_dir.is_dir():
        raise ValueError(f'MISSING_{split.upper()}_DIRECTORY')
    return image_dir, label_dir


def audit_dataset(data_yaml, minimum=MIN_TRAIN_CONE_IMAGES, stage='qualified'):
    """Require distinct, decodable cone-positive train frames and clean split boundaries."""
    if type(minimum) is not int or minimum < 1:
        raise ValueError('INVALID_MINIMUM')
    if stage not in ('qualified', 'experimental'):
        raise ValueError('INVALID_TRAINING_STAGE')
    if stage == 'qualified' and minimum != MIN_TRAIN_CONE_IMAGES:
        raise ValueError('QUALIFIED_MINIMUM_CANNOT_BE_REDUCED')
    data_yaml = Path(data_yaml).resolve()
    data = yaml.safe_load(data_yaml.read_text())
    if not isinstance(data, dict):
        raise ValueError('INVALID_DATA_YAML')
    root = Path(data.get('path', data_yaml.parent))
    root = (data_yaml.parent / root).resolve() if not root.is_absolute() else root.resolve()
    names, labels = _classes(data.get('names'))
    provenance = {key: data.get(key) for key in ('source', 'license')}
    if any(not isinstance(value, str) or not value.strip() for value in provenance.values()):
        raise ValueError('DATASET_SOURCE_AND_LICENSE_REQUIRED')
    report = {'dataset': str(data_yaml), 'root': str(root), 'stage': stage,
              'minimum_train_cone_images': minimum,
              'names': names, 'canonical_classes': labels, 'provenance': provenance,
              'splits': {}, 'errors': [],
              'status': 'DATASET_REJECTED'}
    seen = {};aggregate = hashlib.sha256();image_sessions={};session_splits={}
    has_manifest = 'session_manifest' in data
    if stage == 'qualified' or has_manifest:
        manifest_name=data.get('session_manifest')
        if not isinstance(manifest_name,str) or not manifest_name.strip():
            raise ValueError('SESSION_MANIFEST_REQUIRED')
        manifest_path=(root/manifest_name).resolve()
        if not manifest_path.is_relative_to(root) or not manifest_path.is_file():
            raise ValueError('SESSION_MANIFEST_NOT_FOUND')
        with manifest_path.open(newline='') as stream:
            reader=csv.DictReader(stream)
            if not {'image','session_id'}.issubset(reader.fieldnames or []):
                raise ValueError('INVALID_SESSION_MANIFEST')
            for row in reader:
                image=row['image'];session=row['session_id']
                if not image or not session or image in image_sessions or Path(image).is_absolute() or '..' in Path(image).parts:
                    raise ValueError('INVALID_SESSION_MANIFEST_ENTRY')
                image_sessions[image]=session
        aggregate.update(manifest_path.read_bytes())
    for split in ('train', 'val', 'test'):
        image_dir, label_dir = _image_and_label_dirs(data, split, root)
        images = sorted(p for p in image_dir.rglob('*') if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES)
        counts = {'images': len(images), 'cone_positive_images': 0, 'boxes': 0,
                  'boxes_by_class': {label: 0 for label in labels}}
        report['splits'][split] = counts
        if not images:
            report['errors'].append(f'{split}:NO_IMAGES')
        for image in images:
            relative = image.relative_to(image_dir)
            image_key=image.relative_to(root).as_posix()
            if has_manifest:
                session=image_sessions.get(image_key)
                if session is None:
                    report['errors'].append(f'{split}:{relative}:SESSION_NOT_DOCUMENTED')
                    continue
                if session in session_splits and session_splits[session]!=split:
                    report['errors'].append(f'{split}:{relative}:SESSION_SPLIT_LEAKAGE:{session}')
                    continue
                session_splits[session]=split
            label_file = (label_dir / relative).with_suffix('.txt')
            if not label_file.is_file():
                report['errors'].append(f'{split}:{relative}:MISSING_LABEL')
                continue
            try:
                digest = hashlib.sha256(image.read_bytes()).hexdigest()
                if digest in seen:
                    report['errors'].append(f'{split}:{relative}:DUPLICATE_IMAGE_OF:{seen[digest]}')
                    continue
                seen[digest] = f'{split}/{relative}'
                if cv2.imread(str(image), cv2.IMREAD_UNCHANGED) is None:
                    raise ValueError('UNDECODABLE_IMAGE')
                label_text = label_file.read_text()
                lines = label_text.splitlines()
                for line in lines:
                    parts = line.split()
                    if len(parts) != 5:
                        raise ValueError('INVALID_YOLO_BOX')
                    index = int(parts[0])
                    values = np.asarray([float(x) for x in parts[1:]], dtype=float)
                    if not 0 <= index < len(labels) or not np.isfinite(values).all():
                        raise ValueError('INVALID_CLASS_OR_BOX')
                    x, y, w, h = values
                    if not (0 < w <= 1 and 0 < h <= 1 and 0 <= x-w/2 and x+w/2 <= 1
                            and 0 <= y-h/2 and y+h/2 <= 1):
                        raise ValueError('BOX_OUT_OF_BOUNDS')
                    counts['boxes'] += 1
                    counts['boxes_by_class'][labels[index]] += 1
                if lines:
                    counts['cone_positive_images'] += 1
                aggregate.update(f'{split}/{relative}:{digest}\n'.encode())
                aggregate.update(label_text.encode())
            except (OSError, UnicodeError, ValueError, OverflowError) as exc:
                report['errors'].append(f'{split}:{relative}:{exc}')
            if len(report['errors']) > 100:
                report['errors'] = report['errors'][:100] + ['TOO_MANY_ERRORS']
                break
    if report['splits']['train']['cone_positive_images'] < minimum:
        report['errors'].append('INSUFFICIENT_TRAIN_CONE_IMAGES')
    for image_key in image_sessions:
        if not (root/image_key).is_file():
            report['errors'].append(f'SESSION_IMAGE_NOT_FOUND:{image_key}')
    for split in ('val', 'test'):
        if report['splits'][split]['cone_positive_images'] == 0:
            report['errors'].append(f'NO_{split.upper()}_CONE_IMAGES')
    report['status'] = ('READY_FOR_TRAINING' if stage=='qualified' else 'EXPERIMENTAL_READY') if not report['errors'] else 'DATASET_REJECTED'
    report['sessions_by_split']={split:sorted(s for s,v in session_splits.items() if v==split) for split in ('train','val','test')}
    report['session_counts']={split:len(report['sessions_by_split'][split]) for split in ('train','val','test')}
    report['session_independence'] = ('VERIFIED_FROM_MANIFEST' if has_manifest and
        not any('SESSION_' in error for error in report['errors']) else 'NOT_VERIFIED')
    report['dataset_sha256'] = aggregate.hexdigest()
    return report


def train(data_yaml, run_dir, *, epochs=100, image_size=640, batch=16,
          device='0', weights='yolo11n.pt', stage='qualified'):
    """Fine-tune pretrained YOLO11n after an explicit stage-specific audit."""
    report = audit_dataset(data_yaml, minimum=MIN_TRAIN_CONE_IMAGES if stage=='qualified' else 1,stage=stage)
    if report['status'] not in ('READY_FOR_TRAINING','EXPERIMENTAL_READY'):
        raise ValueError('DATASET_REJECTED:' + ','.join(report['errors'][:5]))
    if not (1 <= epochs <= 1000 and image_size in (416, 512, 640, 768)
            and 1 <= batch <= 128 and device in ('cpu', '0')):
        raise ValueError('INVALID_TRAINING_OPTIONS')
    try:
        from ultralytics import YOLO
        import ultralytics
        import torch
    except ImportError as exc:
        raise RuntimeError('TRAINING_DEPENDENCIES_MISSING:install ultralytics and torch') from exc
    if device == '0' and not torch.cuda.is_available():
        raise RuntimeError('CUDA_NOT_AVAILABLE:use a CUDA host or explicitly select --device cpu')
    if Path(weights).name!='yolo11n.pt' or (weights!='yolo11n.pt' and not Path(weights).is_file()):
        raise ValueError('PRETRAINED_YOLO11N_WEIGHTS_REQUIRED')
    run_dir = Path(run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / 'dataset_audit.json').write_text(json.dumps(report, indent=2) + '\n')
    data = yaml.safe_load(Path(data_yaml).read_text())
    prepared = {'path': report['root'], 'train': data['train'], 'val': data['val'],
                'test': data['test'], 'names': report['names']}
    prepared_path = run_dir / 'data.yaml'
    prepared_path.write_text(yaml.safe_dump(prepared, sort_keys=False))
    os.environ['YOLO_AUTOINSTALL'] = 'False'
    model = YOLO(weights)
    if getattr(model,'task',None)!='detect':
        raise RuntimeError('PRETRAINED_WEIGHTS_NOT_DETECTION_MODEL')
    original = getattr(model, 'ckpt_path', None) or weights
    original = Path(original)
    base_sha256 = hashlib.sha256(original.read_bytes()).hexdigest() if original.is_file() else None
    model.train(data=str(prepared_path), epochs=epochs, imgsz=image_size, batch=batch,
                device=device, workers=2, project=str(run_dir), name='fit', exist_ok=False,
                seed=42, cache=False)
    best = run_dir / 'fit' / 'weights' / 'best.pt'
    if not best.is_file():
        raise RuntimeError('TRAINING_CHECKPOINT_MISSING')
    trained = YOLO(str(best))
    metrics = trained.val(data=str(prepared_path), split='test', imgsz=image_size, device=device)
    scores = {'precision': float(metrics.box.mp), 'recall': float(metrics.box.mr),
              'map50': float(metrics.box.map50), 'map50_95': float(metrics.box.map)}
    if not all(np.isfinite(list(scores.values()))):
        raise RuntimeError('VALIDATION_METRICS_NONFINITE:ONNX_NOT_EXPORTED')
    exported = Path(trained.export(format='onnx', imgsz=image_size, opset=12,
                                   simplify=False, dynamic=False, nms=False, device=device))
    output = run_dir / 'model.onnx'
    shutil.copyfile(exported, output)
    net = cv2.dnn.readNetFromONNX(str(output))
    from .detector import _blob_and_geometry
    test_images=sorted(p for p in (Path(report['root'])/data['test']).rglob('*') if p.suffix.lower() in IMAGE_SUFFIXES)
    sample=None;sample_path=None
    for candidate in test_images:
        sample=cv2.imread(str(candidate))
        if sample is not None:sample_path=candidate;break
    if sample is None:raise RuntimeError('NO_DECODABLE_TEST_IMAGE')
    blob,_=_blob_and_geometry(sample,image_size)
    net.setInput(blob)
    raw=net.forward()
    if raw.ndim != 3 or (4 + len(report['canonical_classes'])) not in raw.shape[1:]:
        raise RuntimeError(f'UNSUPPORTED_EXPORTED_ONNX_SHAPE:{raw.shape}')
    # Compare raw predictions before any postprocessing so empty scenes cannot pass vacuously.
    trained.model.eval()
    model_device=next(trained.model.parameters()).device
    with torch.no_grad():
        prediction=trained.model(torch.from_numpy(blob).to(model_device))
    if isinstance(prediction,(tuple,list)):prediction=prediction[0]
    reference=prediction.detach().cpu().numpy()
    if reference.shape!=raw.shape or not np.allclose(reference,raw,rtol=.01,atol=.01):
        raise RuntimeError('PYTORCH_ONNX_OUTPUT_MISMATCH')
    manifest = {'format': 'ultralytics_yolo11_raw_onnx', 'model_sha256':
                hashlib.sha256(output.read_bytes()).hexdigest(), 'input_size': image_size,
                'classes': report['canonical_classes'], 'dataset_audit': 'dataset_audit.json',
                'dataset_sha256': report['dataset_sha256'], 'dataset_provenance': report['provenance'],
                'base_checkpoint': str(original), 'base_checkpoint_sha256': base_sha256,
                'ultralytics_version': ultralytics.__version__, 'torch_version': torch.__version__,
                'training_parameters': {'epochs': epochs, 'image_size': image_size,
                                        'batch': batch, 'device': device, 'seed': 42},
                'train_cone_images': report['splits']['train']['cone_positive_images'],
                'held_out_test_metrics': scores, 'stage':stage,
                'onnx_consistency':{'sample':str(sample_path),'rtol':.01,'atol':.01},
                'status': 'EXPERIMENTAL_HP60C_VALIDATION_PENDING' if stage=='experimental' else 'HP60C_VALIDATION_PENDING'}
    (run_dir / 'model.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description='Audit and optionally train a cone YOLO11n model')
    parser.add_argument('action', choices=('audit', 'train'))
    parser.add_argument('--data', required=True, help='YOLO data.yaml with train/val/test')
    parser.add_argument('--run-dir', help='New output directory, required for train')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--device', choices=('0', 'cpu'), default='0')
    parser.add_argument('--weights', default='yolo11n.pt')
    parser.add_argument('--stage',choices=('qualified','experimental'),default='qualified')
    args = parser.parse_args(argv)
    if args.action == 'audit':
        try:result = audit_dataset(args.data,minimum=MIN_TRAIN_CONE_IMAGES if args.stage=='qualified' else 1,stage=args.stage)
        except (ValueError, OSError, yaml.YAMLError) as exc:
            print(f'Dataset status: {exc}')
            return 2
        print(json.dumps(result, indent=2))
        return 0 if result['status'] in ('READY_FOR_TRAINING','EXPERIMENTAL_READY') else 2
    if not args.run_dir:
        parser.error('--run-dir is required for train')
    try:
        result = train(args.data, args.run_dir, epochs=args.epochs, image_size=args.imgsz,
                       batch=args.batch, device=args.device, weights=args.weights,stage=args.stage)
    except (ValueError, RuntimeError, OSError, cv2.error, yaml.YAMLError) as exc:
        print(f'Training status: {exc}')
        return 2
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
