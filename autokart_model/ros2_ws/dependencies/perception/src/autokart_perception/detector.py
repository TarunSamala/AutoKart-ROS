import hashlib
import json
import time
from importlib import resources
from pathlib import Path
import cv2
import numpy as np
from .types import Detection

MODEL_SHA256='67c8dd8922fa793a0577297c8936de1fc5decaebfb7c79c1f9a7aa1542f93faf'


def builtin_model_manifest() -> dict:
    """Return the pinned one-class model identity, not an accuracy claim."""
    manifest = json.loads(resources.files("autokart_perception").joinpath(
        "data/coneslayer_manifest.json").read_text())
    if (manifest.get("schema_version") != 1 or
            manifest.get("model_sha256") != MODEL_SHA256 or
            manifest.get("output_classes") != ["orange"] or
            manifest.get("input_width") != 416 or manifest.get("input_height") != 416):
        raise DetectorError("BUILTIN_MODEL_MANIFEST_INVALID")
    return manifest

class DetectorError(RuntimeError):
    pass


def _blob_and_geometry(image, size):
    if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:
        raise DetectorError('INVALID_RGB_FRAME')
    h,w=image.shape[:2]
    ratio=min(size/w,size/h); rw,rh=round(w*ratio),round(h*ratio)
    canvas=np.full((size,size,3),114,dtype=np.uint8)
    left=(size-rw)//2; top=(size-rh)//2
    canvas[top:top+rh,left:left+rw]=cv2.resize(image,(rw,rh))
    return cv2.dnn.blobFromImage(canvas,1/255,(size,size),swapRB=True,crop=False),(w,h,ratio,left,top)

class ConeDetector:
    """Coneslayer YOLOv7-tiny ONNX, one orange traffic-cone class only."""
    def __init__(self, path, confidence=0.54, iou=0.5, expected_sha256=MODEL_SHA256, label='orange'):
        path=Path(path)
        if not path.exists(): raise DetectorError('MODEL_NOT_AVAILABLE')
        if expected_sha256 == MODEL_SHA256 and label == 'orange':
            manifest=builtin_model_manifest()
            if path.stat().st_size != manifest['model_size_bytes']:
                raise DetectorError('MODEL_SIZE_MISMATCH')
        model_sha256=hashlib.sha256(path.read_bytes()).hexdigest()
        if expected_sha256 and model_sha256!=expected_sha256:
            raise DetectorError('MODEL_CHECKSUM_MISMATCH')
        self.net=cv2.dnn.readNetFromONNX(str(path))
        self.confidence=confidence; self.iou=iou; self.version=path.name
        self.latency_ms=0.0; self.label=label; self.input_size=416
        self.backend='OpenCV DNN default CPU'
        self.verified_checkpoint=expected_sha256==MODEL_SHA256 and label=='orange'
        self.supported_classes=('orange',) if self.verified_checkpoint else ('unknown',)
        self.model_sha256=model_sha256

    def infer(self, image):
        # HP60C SDK supplies BGR8. YOLOv7 expects RGB in [0,1].
        blob,geometry=_blob_and_geometry(image,self.input_size)
        t=time.perf_counter()
        self.net.setInput(blob)
        outputs=self.net.forward(self.net.getUnconnectedOutLayersNames())
        self.latency_ms=(time.perf_counter()-t)*1000
        rows=next((o.reshape(-1,6) for o in outputs if o.size%6==0 and o.ndim==3 and o.shape[-1]==6),None)
        if rows is None: raise DetectorError('UNSUPPORTED_MODEL_OUTPUT')
        return rows,geometry

    def postprocess(self, rows, geometry, confidence=None, iou=None):
        confidence=self.confidence if confidence is None else confidence
        iou=self.iou if iou is None else iou
        if not (0<=confidence<=1 and 0<iou<=1):raise DetectorError('INVALID_THRESHOLDS')
        w,h,ratio,left,top=geometry
        boxes=[]; scores=[]
        for row in rows:
            cx,cy,bw,bh,obj,cls=map(float,row)
            score=obj*cls
            if not np.isfinite(score) or score<confidence: continue
            x1=max(0,int((cx-bw/2-left)/ratio)); y1=max(0,int((cy-bh/2-top)/ratio))
            x2=min(w,int((cx+bw/2-left)/ratio)); y2=min(h,int((cy+bh/2-top)/ratio))
            if x2<=x1 or y2<=y1: continue
            boxes.append([x1,y1,x2-x1,y2-y1]); scores.append(score)
        indices=cv2.dnn.NMSBoxes(boxes,scores,confidence,iou)
        return [Detection((boxes[i][0],boxes[i][1],boxes[i][0]+boxes[i][2],boxes[i][1]+boxes[i][3]),
                          self.label,float(scores[i]),n) for n,i in enumerate(np.asarray(indices).reshape(-1))]

    def detect(self, image):
        return self.postprocess(*self.infer(image))


class Yolo11OnnxDetector:
    """Raw YOLO11 detect export with a checksum-bound class manifest."""
    def __init__(self,path,confidence=.25,iou=.5,manifest_path=None):
        path=Path(path)
        manifest_path=Path(manifest_path) if manifest_path is not None else path.with_suffix('.json')
        if not path.is_file() or not manifest_path.is_file():
            raise DetectorError('MODEL_OR_MANIFEST_NOT_AVAILABLE')
        try:manifest=json.loads(manifest_path.read_text())
        except (ValueError,OSError) as exc:raise DetectorError('INVALID_MODEL_MANIFEST') from exc
        classes=manifest.get('classes')
        if manifest.get('format')!='ultralytics_yolo11_raw_onnx' or not isinstance(classes,list) or not classes:
            raise DetectorError('UNSUPPORTED_MODEL_MANIFEST')
        if any(label not in ('blue','yellow','orange','large_orange','unknown') for label in classes):
            raise DetectorError('UNSUPPORTED_MODEL_CLASSES')
        if len(set(classes))!=len(classes):raise DetectorError('DUPLICATE_MODEL_CLASSES')
        trained_images=manifest.get('train_cone_images')
        stage=manifest.get('stage','qualified')
        if stage not in ('qualified','experimental') or type(trained_images) is not int or trained_images<1:
            raise DetectorError('TRAINING_IMAGE_MINIMUM_NOT_DOCUMENTED')
        if stage=='qualified' and trained_images<19_385:
            raise DetectorError('TRAINING_IMAGE_MINIMUM_NOT_DOCUMENTED')
        if stage=='experimental' and manifest.get('status')!='EXPERIMENTAL_HP60C_VALIDATION_PENDING':
            raise DetectorError('EXPERIMENTAL_MODEL_STATUS_REQUIRED')
        size=manifest.get('input_size')
        if type(size) is not int or size not in (416,512,640,768):
            raise DetectorError('UNSUPPORTED_MODEL_INPUT')
        model_sha256=hashlib.sha256(path.read_bytes()).hexdigest()
        if model_sha256!=manifest.get('model_sha256'):
            raise DetectorError('MODEL_CHECKSUM_MISMATCH')
        if not 0<=confidence<=1 or not 0<iou<=1:raise DetectorError('INVALID_THRESHOLDS')
        self.net=cv2.dnn.readNetFromONNX(str(path))
        self.classes=classes;self.input_size=size;self.confidence=confidence;self.iou=iou
        self.version=path.name;self.latency_ms=0.0;self.stage=stage
        self.backend='OpenCV DNN default CPU'
        self.supported_classes=tuple(classes)
        self.model_sha256=model_sha256
        # A valid file checksum proves identity, not HP60C accuracy.
        self.verified_checkpoint=False
        self.last_counts={'raw_output_rows':0,'above_confidence':0,
                          'valid_boxes':0,'after_nms':0}

    def infer(self,image):
        blob,geometry=_blob_and_geometry(image,self.input_size)
        start=time.perf_counter();self.net.setInput(blob);raw=self.net.forward()
        self.latency_ms=(time.perf_counter()-start)*1000
        channels=4+len(self.classes)
        if raw.ndim!=3 or raw.shape[0]!=1:raise DetectorError('UNSUPPORTED_MODEL_OUTPUT')
        if raw.shape[1]==channels:rows=raw[0].T
        elif raw.shape[2]==channels:rows=raw[0]
        else:raise DetectorError('UNSUPPORTED_MODEL_OUTPUT')
        return rows,geometry

    def postprocess(self,rows,geometry,confidence=None,iou=None):
        confidence=self.confidence if confidence is None else confidence
        iou=self.iou if iou is None else iou
        if not 0<=confidence<=1 or not 0<iou<=1:raise DetectorError('INVALID_THRESHOLDS')
        w,h,ratio,left,top=geometry
        candidates={i:([],[]) for i in range(len(self.classes))}
        raw_rows=len(rows)
        if isinstance(rows,np.ndarray) and rows.ndim==2 and rows.shape[1]==4+len(self.classes):
            # Skip low-score anchors before Python decoding; keep the same NMS and scaling.
            valid=np.isfinite(rows).all(axis=1)
            if len(rows):valid &= np.max(rows[:,4:],axis=1)>=confidence
            rows=rows[valid]
        above_confidence=0
        for row in rows:
            if len(row)!=4+len(self.classes) or not np.isfinite(row).all():continue
            klass=int(np.argmax(row[4:]));score=float(row[4+klass])
            if score<confidence:continue
            above_confidence+=1
            cx,cy,bw,bh=map(float,row[:4])
            x1=max(0,int((cx-bw/2-left)/ratio));y1=max(0,int((cy-bh/2-top)/ratio))
            x2=min(w,int((cx+bw/2-left)/ratio));y2=min(h,int((cy+bh/2-top)/ratio))
            if x2<=x1 or y2<=y1:continue
            boxes,scores=candidates[klass];boxes.append([x1,y1,x2-x1,y2-y1]);scores.append(score)
        selected=[]
        for klass,(boxes,scores) in candidates.items():
            if not boxes:continue
            indices=np.asarray(cv2.dnn.NMSBoxes(boxes,scores,confidence,iou)).reshape(-1)
            for index in indices:
                x,y,bw,bh=boxes[index]
                selected.append(((x,y,x+bw,y+bh),self.classes[klass],float(scores[index])))
        selected.sort(key=lambda item:(-item[2],item[1],item[0]))
        self.last_counts={'raw_output_rows':raw_rows,'above_confidence':above_confidence,
                          'valid_boxes':sum(len(boxes) for boxes,_ in candidates.values()),
                          'after_nms':len(selected)}
        return [Detection(box,label,score,index) for index,(box,label,score) in enumerate(selected)]

    def detect(self,image):
        return self.postprocess(*self.infer(image))


class Yolo26OnnxDetector:
    """YOLO26 end-to-end export: rows are xyxy, score, class ID."""

    def __init__(self, path, confidence=.25, manifest_path=None):
        path = Path(path)
        manifest_path = Path(manifest_path) if manifest_path is not None else path.with_suffix('.json')
        if not path.is_file() or not manifest_path.is_file():
            raise DetectorError('MODEL_OR_MANIFEST_NOT_AVAILABLE')
        try:
            manifest = json.loads(manifest_path.read_text())
        except (ValueError, OSError) as exc:
            raise DetectorError('INVALID_MODEL_MANIFEST') from exc
        classes = manifest.get('classes')
        if (manifest.get('format') != 'ultralytics_yolo26_end2end_onnx' or
                classes != ['blue', 'large_orange', 'orange', 'unknown', 'yellow'] or
                manifest.get('input_size') != 640 or
                manifest.get('output_shape') != [1, 300, 6] or
                manifest.get('stage') != 'experimental' or
                manifest.get('status') != 'EXPERIMENTAL_HP60C_VALIDATION_PENDING'):
            raise DetectorError('UNSUPPORTED_MODEL_MANIFEST')
        model_sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        if model_sha256 != manifest.get('model_sha256'):
            raise DetectorError('MODEL_CHECKSUM_MISMATCH')
        if not 0 <= confidence <= 1:
            raise DetectorError('INVALID_THRESHOLDS')
        try:
            import onnxruntime as ort
        except ImportError:
            self.session = None
            self.net = cv2.dnn.readNetFromONNX(str(path))
            self.backend = 'OpenCV DNN default CPU'
        else:
            options = ort.SessionOptions()
            options.intra_op_num_threads = max(1, cv2.getNumThreads())
            options.inter_op_num_threads = 1
            options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.session = ort.InferenceSession(str(path), sess_options=options,
                                                providers=['CPUExecutionProvider'])
            if ([item.shape for item in self.session.get_inputs()] != [[1, 3, 640, 640]] or
                    [item.shape for item in self.session.get_outputs()] != [[1, 300, 6]]):
                raise DetectorError('UNSUPPORTED_MODEL_OUTPUT')
            self.net = None
            self.backend = 'ONNX Runtime CPU'
        self.classes = classes
        self.input_size = 640
        self.confidence = confidence
        self.version = path.name
        self.latency_ms = 0.0
        self.stage = 'experimental'
        self.supported_classes = tuple(classes)
        self.model_sha256 = model_sha256
        self.verified_checkpoint = False
        self.last_counts = {'raw_output_rows': 0, 'above_confidence': 0,
                            'valid_boxes': 0, 'after_nms': 0}

    def infer(self, image):
        blob, geometry = _blob_and_geometry(image, self.input_size)
        start = time.perf_counter()
        if self.session is None:
            self.net.setInput(blob)
            raw = self.net.forward()
        else:
            raw = self.session.run(None, {self.session.get_inputs()[0].name: blob})[0]
        self.latency_ms = (time.perf_counter() - start) * 1000
        if raw.shape != (1, 300, 6):
            raise DetectorError('UNSUPPORTED_MODEL_OUTPUT')
        return raw[0], geometry

    def postprocess(self, rows, geometry, confidence=None):
        confidence = self.confidence if confidence is None else confidence
        if not 0 <= confidence <= 1:
            raise DetectorError('INVALID_THRESHOLDS')
        rows = np.asarray(rows)
        if rows.ndim != 2 or rows.shape[1] != 6:
            raise DetectorError('UNSUPPORTED_MODEL_OUTPUT')
        w, h, ratio, left, top = geometry
        selected = []
        above_confidence = 0
        for row in rows:
            if not np.isfinite(row).all() or not confidence <= row[4] <= 1:
                continue
            above_confidence += 1
            class_id = int(row[5])
            if row[5] != class_id or not 0 <= class_id < len(self.classes):
                continue
            x1 = max(0, int((row[0] - left) / ratio))
            y1 = max(0, int((row[1] - top) / ratio))
            x2 = min(w, int((row[2] - left) / ratio))
            y2 = min(h, int((row[3] - top) / ratio))
            if x2 <= x1 or y2 <= y1:
                continue
            selected.append(((x1, y1, x2, y2), self.classes[class_id], float(row[4])))
        selected.sort(key=lambda item: (-item[2], item[1], item[0]))
        # The model's one-to-one TopK head supplies the final rows; there is no external NMS.
        self.last_counts = {'raw_output_rows': len(rows), 'above_confidence': above_confidence,
                            'valid_boxes': len(selected), 'after_nms': len(selected)}
        return [Detection(box, label, score, index)
                for index, (box, label, score) in enumerate(selected)]

    def detect(self, image):
        return self.postprocess(*self.infer(image))
