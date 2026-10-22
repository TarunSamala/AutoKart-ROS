"""Headless arc-length polyline track geometry and deterministic surface lookup."""
from __future__ import annotations
from dataclasses import dataclass, asdict
from math import atan2, hypot, isfinite
from pathlib import Path
from typing import Any
import yaml

EPS = 1e-9

@dataclass(frozen=True)
class SurfaceMaterial:
    id: str; friction_longitudinal: float; friction_lateral: float; rolling_resistance: float; roughness_m: float = 0.0

@dataclass(frozen=True)
class TrackPoint:
    s_m: float; x_m: float; y_m: float; z_m: float; heading_rad: float; curvature_1pm: float; grade: float; bank_rad: float; left_width_m: float; right_width_m: float

@dataclass(frozen=True)
class Projection:
    valid: bool; s_m: float; lateral_m: float; vertical_m: float; heading_rad: float; projection_distance_m: float; left_margin_m: float; right_margin_m: float; segment_id: int

@dataclass(frozen=True)
class SurfaceLookup:
    material: SurfaceMaterial; valid: bool; surface_height_m: float; bank_rad: float

class Track:
    def __init__(self, name: str, points: list[tuple[float,float,float]], *, closed: bool, left_width: float, right_width: float, banking: float = 0.0, materials: dict[str,SurfaceMaterial] | None = None, regions: list[dict[str,Any]] | None = None) -> None:
        if len(points) < (3 if closed else 2): raise ValueError("track has too few centreline points")
        if left_width <= 0 or right_width <= 0: raise ValueError("track widths must be positive")
        if not all(all(isfinite(v) for v in p) for p in points): raise ValueError("track point is non-finite")
        self.name, self.points, self.closed, self.left_width, self.right_width, self.banking = name, points, closed, left_width, right_width, banking
        self.materials = materials or {"dry_asphalt": SurfaceMaterial("dry_asphalt", 1.0, .95, .018)}
        self.regions = sorted(regions or [], key=lambda r: (-int(r.get("priority", 0)), str(r.get("id", ""))))
        self._segments = [(i, (i+1)%len(points)) for i in range(len(points)-(0 if closed else 1))]
        self._lengths=[]; total=0.0
        for i,j in self._segments:
            a,b=points[i],points[j]; length=hypot(b[0]-a[0],b[1]-a[1])
            if length <= EPS: raise ValueError("duplicate consecutive track point")
            self._lengths.append(length); total += length
        self.length_m=total; self._s=[0.0]
        for length in self._lengths: self._s.append(self._s[-1]+length)

    @classmethod
    def from_yaml(cls, path: str|Path) -> "Track":
        doc=yaml.safe_load(Path(path).read_text()); root=doc["track"]
        pts=[tuple(map(float,p)) for p in root["centreline"]["control_points"]]
        m={k:SurfaceMaterial(k,float(v["friction_longitudinal"]),float(v["friction_lateral"]),float(v["rolling_resistance"]),float(v.get("roughness_m",0))) for k,v in doc.get("materials",{}).items()}
        return cls(root["name"],pts,closed=root.get("type","open")=="closed",left_width=float(root["width"]["default_left_m"]),right_width=float(root["width"]["default_right_m"]),banking=float(root.get("banking",{}).get("default_rad",0)),materials=m or None,regions=doc.get("surface_regions",[]))

    def _wrap(self,s:float)->float:
        return s%self.length_m if self.closed else min(self.length_m,max(0.0,s))
    def sample(self,s:float)->TrackPoint:
        s=self._wrap(s); idx=min(len(self._lengths)-1,next(i for i,end in enumerate(self._s[1:]) if s<=end+EPS)); a,b=self.points[self._segments[idx][0]],self.points[self._segments[idx][1]]; t=(s-self._s[idx])/self._lengths[idx]
        x,y,z=[a[k]+t*(b[k]-a[k]) for k in range(3)]; heading=atan2(b[1]-a[1],b[0]-a[0]); prev=(idx-1)%len(self._lengths) if self.closed else max(0,idx-1); nxt=(idx+1)%len(self._lengths) if self.closed else min(len(self._lengths)-1,idx+1)
        h0=atan2(self.points[self._segments[prev][1]][1]-self.points[self._segments[prev][0]][1],self.points[self._segments[prev][1]][0]-self.points[self._segments[prev][0]][0]); dh=(heading-h0+3.141592653589793)%(2*3.141592653589793)-3.141592653589793
        curvature=dh/max(EPS,(self._lengths[prev]+self._lengths[idx])/2); grade=(b[2]-a[2])/self._lengths[idx]
        return TrackPoint(s,x,y,z,heading,curvature,grade,self.banking,self.left_width,self.right_width)
    def world_from_track(self,s:float,lateral_m:float=0.0,vertical_m:float=0.0)->tuple[float,float,float]:
        p=self.sample(s); return p.x_m-lateral_m*__import__('math').sin(p.heading_rad),p.y_m+lateral_m*__import__('math').cos(p.heading_rad),p.z_m+vertical_m
    def project(self,x:float,y:float,z:float=0.0,previous_s_m:float|None=None)->Projection:
        candidates=[]
        for idx,(i,j) in enumerate(self._segments):
            a,b=self.points[i],self.points[j]; dx,dy=b[0]-a[0],b[1]-a[1]; q=max(0,min(1,((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy))); px,py=a[0]+q*dx,a[1]+q*dy; dist=hypot(x-px,y-py); s=self._s[idx]+q*self._lengths[idx]; candidates.append((dist,abs(((s-(previous_s_m or s)+self.length_m/2)%self.length_m)-self.length_m/2) if previous_s_m is not None and self.closed else 0,idx,q,s,px,py))
        candidates.sort(key=lambda c:(c[0],c[1])); dist,_,idx,q,s,px,py=candidates[0]; p=self.sample(s); lateral=-(x-px)*__import__('math').sin(p.heading_rad)+(y-py)*__import__('math').cos(p.heading_rad); return Projection(True,s,lateral,z-p.z_m,p.heading_rad,dist,p.left_width_m-lateral,p.right_width_m+lateral,idx)
    def surface_at(self,s:float,lateral_m:float)->SurfaceLookup:
        for r in self.regions:
            if r.get("coordinates","track")!="track": continue
            inside=r["start_s_m"]<=self._wrap(s)<=r["end_s_m"] and r["min_lateral_m"]<=lateral_m<=r["max_lateral_m"]
            if inside: return SurfaceLookup(self.materials[r["material"]],True,self.sample(s).z_m,self.banking)
        return SurfaceLookup(self.materials["dry_asphalt"],True,self.sample(s).z_m,self.banking)
    def boundaries(self,spacing_m:float=1.0)->list[dict[str,float]]:
        return [{"s_m":s,"left":self.world_from_track(s,self.left_width),"right":self.world_from_track(s,-self.right_width)} for s in [i*spacing_m for i in range(int(self.length_m/spacing_m)+1)]]
    def validate(self)->list[str]:
        warnings=[]
        if abs(self.banking)>0.6:warnings.append("ERROR: banking exceeds 0.6 rad")
        if self.closed and hypot(self.points[0][0]-self.points[-1][0],self.points[0][1]-self.points[-1][1])<EPS:warnings.append("ERROR: closed track repeats seam point")
        return warnings

def segment_corners(track:Track, threshold:float=.02)->list[dict[str,Any]]:
    samples=[track.sample(i*.5) for i in range(int(track.length_m/.5))]; groups=[]; active=[]
    for p in samples:
        if abs(p.curvature_1pm)>=threshold: active.append(p)
        elif active:
            if len(active)>=3: groups.append(active)
            active=[]
    if active and len(active)>=3:groups.append(active)
    return [{"id":f"C{i+1:02d}","source":"automatic","direction":"left" if sum(p.curvature_1pm for p in g)>0 else "right","start_s_m":g[0].s_m,"geometric_apex_s_m":max(g,key=lambda p:abs(p.curvature_1pm)).s_m,"end_s_m":g[-1].s_m,"classification":{"type":"constant_or_variable_radius"}} for i,g in enumerate(groups)]
