import time
import numpy as np
import cv2
from .types import Frame, Intrinsics, Detection, Cone

class Synthetic:
    def __init__(self): self.index=0
    def read(self):
        self.index+=1
        rgb=np.full((480,640,3),32,dtype=np.uint8)
        depth=np.full((480,640),4000,dtype=np.uint16)
        # This is an interface exercise. Metric observations are deterministic
        # scenario input, never inferred from the painted pixels.
        for u,color in ((210,(255,0,0)),(430,(0,255,255))):
            cv2.drawContours(rgb,[np.array([[u,260],[u-20,380],[u+20,380]])],0,color,-1)
        now=time.monotonic_ns()
        return Frame(rgb,depth,now,0,0,self.index,self.index,Intrinsics(500,500,320,240),
            0.001,True,'SYNTHETIC',0,{})

def synthetic_cones():
    coords=[(2,1.5,'blue'),(2,-1.5,'yellow'),(5,1.5,'blue'),(5,-1.5,'yellow'),(8,1.5,'blue'),(8,-1.5,'yellow')]
    return [Cone(i,label,0.95,(x,y),0.05,float(np.hypot(x,y)),(0,0,0,0))
            for i,(x,y,label) in enumerate(coords)]
