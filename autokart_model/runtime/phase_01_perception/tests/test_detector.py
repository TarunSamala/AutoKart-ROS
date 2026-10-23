import numpy as np
from autokart_perception.detector import ConeDetector, builtin_model_manifest


class FakeNet:
    def setInput(self,blob):self.blob=blob
    def getUnconnectedOutLayersNames(self):return ['decoded']
    def forward(self,names):
        return [np.array([[[208,208,65,130,.9,.8]]],dtype=np.float32)]


def test_bgr_to_rgb_letterbox_and_box_scaling():
    detector=ConeDetector.__new__(ConeDetector)
    detector.net=FakeNet();detector.input_size=416;detector.confidence=.54;detector.iou=.5;detector.label='orange'
    image=np.empty((480,640,3),dtype=np.uint8);image[:]=[10,20,30]
    rows,geometry=detector.infer(image)
    assert detector.net.blob.shape==(1,3,416,416)
    assert np.allclose(detector.net.blob[0,:,208,208],[30/255,20/255,10/255])
    assert np.allclose(detector.net.blob[0,:,0,0],[114/255]*3)
    result=detector.postprocess(rows,geometry)
    assert len(result)==1 and result[0].label=='orange'
    assert result[0].box==(270,140,370,340)
    assert detector.postprocess(rows,geometry,confidence=.75)==[]


def test_pinned_detector_manifest_identifies_only_orange_class():
    from hashlib import sha256
    from pathlib import Path
    manifest = builtin_model_manifest()
    model = Path(__file__).parents[1] / "models/coneslayer-simplified.onnx"
    assert manifest["output_classes"] == ["orange"]
    assert manifest["model_size_bytes"] == model.stat().st_size
    assert manifest["model_sha256"] == sha256(model.read_bytes()).hexdigest()
    assert manifest["qualification"] == "IDENTITY_VERIFIED_HP60C_ACCURACY_UNVERIFIED"
