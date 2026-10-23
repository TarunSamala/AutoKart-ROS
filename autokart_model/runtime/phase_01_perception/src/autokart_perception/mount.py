"""Measured optical-camera pose in the stationary kart/bench frame."""
import math
import numpy as np

from .depth import valid_mount


def mount_from_config(config):
    """Return (camera_to_kart, status). Angles use kart-frame right-hand axes.

    Positive yaw turns optical forward toward kart left. Positive pitch points
    optical forward down. Positive roll follows the right-hand rule about kart X.
    The zero-angle optical axes are right/down/forward; kart axes are
    forward/left/up. All translations describe the optical center.
    """
    explicit=config.get('camera_to_kart')
    if explicit is not None:
        try: matrix=np.asarray(explicit,dtype=float)
        except (ValueError,TypeError):return np.empty((0,0)), 'MOUNT_CALIBRATION_INVALID'
        return matrix, 'MOUNT_CONFIGURED_UNVERIFIED' if valid_mount(matrix) else 'MOUNT_CALIBRATION_INVALID'
    p=config.get('camera_mount')
    if p is None:return np.empty((0,0)), 'MOUNT_CALIBRATION_UNAVAILABLE'
    required=('height_m','x_forward_m','y_left_m','pitch_down_deg','roll_deg','yaw_left_deg')
    if not isinstance(p,dict) or p.get('height_reference')!='optical_center' or any(k not in p for k in required):
        return np.empty((0,0)), 'MOUNT_CALIBRATION_INVALID'
    try: height,x,y,pitch,roll,yaw=(float(p[k]) for k in required)
    except (ValueError,TypeError):return np.empty((0,0)), 'MOUNT_CALIBRATION_INVALID'
    if not all(math.isfinite(v) for v in (height,x,y,pitch,roll,yaw)) or height<=0:
        return np.empty((0,0)), 'MOUNT_CALIBRATION_INVALID'
    pth,rol,ya=(math.radians(v) for v in (pitch,roll,yaw))
    cp,sp=math.cos(pth),math.sin(pth)
    cr,sr=math.cos(rol),math.sin(rol)
    cy,sy=math.cos(ya),math.sin(ya)
    yaw_r=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    pitch_r=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    roll_r=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    optical_to_kart=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    matrix=np.eye(4)
    matrix[:3,:3]=yaw_r@pitch_r@roll_r@optical_to_kart
    matrix[:3,3]=[x,y,height]
    return matrix, 'MOUNT_CONFIGURED_UNVERIFIED' if valid_mount(matrix) else 'MOUNT_CALIBRATION_INVALID'
