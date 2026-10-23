import cv2
import numpy as np
import time
import math

COLORS={'blue':(255,80,30),'yellow':(0,230,255),'orange':(0,120,255),'large_orange':(0,70,255),'unknown':(180,180,180)}

class Viewer:
    def __init__(self,web=False):
        self.web=web
        self.flags={k:True for k in ('boxes','labels','delaunay','gabriel','boundaries','gates','centerline','curvature')}
        self.last_ns=None; self.fps=0.0
        self.backend='tk' if any('GUI:' in line and 'NONE' in line for line in cv2.getBuildInformation().splitlines()) else 'opencv'
        self.root=None; self.closed=False

    def _key(self,key):
        if key=='q':self.closed=True
        mapping={'b':'boxes','l':'labels','d':'delaunay','g':'gabriel','s':'boundaries','t':'gates','c':'centerline','k':'curvature'}
        if key in mapping:self.flags[mapping[key]]=not self.flags[mapping[key]]

    def _init_tk(self):
        import tkinter as tk
        self.root=tk.Tk()
        self.root.title('AutoKart Phase 1')
        self.label=tk.Label(self.root)
        self.label.pack()
        self.root.bind('<Key>',lambda event:self._key(event.char.lower()))
        self.root.protocol('WM_DELETE_WINDOW',lambda:self._key('q'))

    def draw(self,result):
        frame=result.frame; left=frame.rgb.copy(); right=np.full((480,640,3),25,dtype=np.uint8)
        if self.flags['boxes']:
            for d in (result.filtered_detections if frame.source=='LIVE' else result.detections):
                cv2.rectangle(left,d.box[:2],d.box[2:],COLORS.get(d.label,COLORS['unknown']),2)
                if self.flags['labels']:
                    cone=next((c for c in result.cones if c.id==d.id),None)
                    label=f'{d.label} {d.confidence:.2f}' + (f' {cone.distance_m:.1f}m' if cone else '')
                    cv2.putText(left,label,(d.box[0],max(15,d.box[1]-5)),cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,255),1)
        origin=(320,450)
        forward_extent=max(4,math.ceil(max((c.xy[0] for c in result.cones),default=0)+.5))
        lateral_extent=max(2.5,max((abs(c.xy[1]) for c in result.cones),default=0)+.5)
        scale=min(400/forward_extent,290/lateral_extent)
        point=lambda xy: (int(origin[0]-xy[1]*scale),int(origin[1]-xy[0]*scale))
        for x in range(forward_extent+1):
            cv2.line(right,point((x,-lateral_extent)),point((x,lateral_extent)),(55,55,55),1)
            cv2.putText(right,f'{x}m',point((x,lateral_extent-.12)),cv2.FONT_HERSHEY_SIMPLEX,.4,(140,140,140),1)
        for y in range(-math.ceil(lateral_extent),math.ceil(lateral_extent)+1):
            cv2.line(right,point((0,y)),point((forward_extent,y)),(55,55,55),1)
        cv2.circle(right,origin,8,(255,255,255),-1);cv2.arrowedLine(right,origin,point((1,0)),(255,255,255),2)
        byid={c.id:c for c in result.cones}
        if self.flags['delaunay']:
            for a,b in result.delaunay.edges:
                cv2.line(right,point(byid[a].xy),point(byid[b].xy),(80,80,80),1)
        if self.flags['gabriel']:
            for a,b in result.gabriel_edges:
                cv2.line(right,point(byid[a].xy),point(byid[b].xy),(180,100,180),1)
        if self.flags['boundaries'] and result.boundaries and result.boundaries.status=='CANDIDATE':
            for ids,color in ((result.boundaries.left_ids,(255,120,50)),
                              (result.boundaries.right_ids,(40,220,255))):
                if len(ids)>=2:
                    cv2.polylines(right,[np.asarray([point(byid[i].xy) for i in ids if i in byid],
                                                    dtype=np.int32)],False,color,2)
        if self.flags['gates']:
            for g in result.gates:
                cv2.line(right,point(byid[g.left_id].xy),point(byid[g.right_id].xy),(0,200,0),2)
                cv2.circle(right,point(g.midpoint),3,(255,255,255),-1)
        if self.flags['centerline'] and len(result.centerline)>1:
            cv2.polylines(right,[np.array([point(p) for p in result.centerline])],False,(255,255,255),2)
        if self.flags['curvature']:
            for sample in result.path_profile:
                if sample.curvature_m_inv is not None:
                    cv2.putText(right,f'k={sample.curvature_m_inv:+.2f}/m',point(sample.xy),
                                cv2.FONT_HERSHEY_SIMPLEX,.4,(220,220,220),1)
        display_cones=result.filtered_cones if frame.source=='LIVE' else result.cones
        for c in display_cones:
            cv2.circle(right,point(c.xy),6,COLORS.get(c.label,COLORS['unknown']),-1)
        for predicted in result.predicted_cones:
            cv2.circle(right,point(predicted['xy']),7,(125,125,125),1)
        now=frame.timestamp_ns
        if self.last_ns and now>self.last_ns:self.fps=.8*self.fps+.2*1e9/(now-self.last_ns)
        self.last_ns=now
        cv2.putText(left,f'{frame.source}  RGB {result.diagnostics["rgb_fps"]:.1f} fps  depth {result.diagnostics["depth_fps"]:.1f} fps',
                    (10,22),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        cv2.putText(right,f'{frame.source} | {result.status}',(10,22),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
        if frame.source=='LIVE':
            cv2.putText(right,'Filled: current filtered   Hollow: predicted display only',
                        (10,43),cv2.FONT_HERSHEY_SIMPLEX,.38,(160,160,160),1)
        lines=[f'Camera: {result.diagnostics["camera"]}  Model: {result.diagnostics["model"]}',
            f'Cones: {len(result.detections)} detected / {len(result.cones)} {"synthetic points" if frame.source=="SYNTHETIC" else "current metric"} / {len(result.predicted_cones)} predicted display  Delaunay/Gabriel: {len(result.delaunay.edges)}/{len(result.gabriel_edges)}',
            f'Age: {result.diagnostics["frame_age_ms"]:.0f}ms  Inference: {result.diagnostics.get("inference_ms",0):.0f}ms  Proc: {result.diagnostics["processing_ms"]:.0f}ms',
            f'Depth valid: {result.diagnostics["valid_depth_pct"]:.1f}%  output: {result.diagnostics["end_to_end_fps"]:.1f} fps  CPU: {result.diagnostics["cpu_pct"]:.0f}%  peak RSS: {result.diagnostics["peak_memory_mb"]:.0f}MB',
            ('Use browser switches to control overlays' if self.web else
             'keys: b boxes | l labels | d Delaunay | g Gabriel | s sides | t gates | c centerline | k curvature | q quit')]
        canvas=np.vstack([np.hstack([left,right]),np.full((110,1280,3),25,dtype=np.uint8)])
        for i,line in enumerate(lines): cv2.putText(canvas,line,(12,500+i*21),cv2.FONT_HERSHEY_SIMPLEX,.48,(230,230,230),1)
        return canvas

    def show(self,result):
        canvas=self.draw(result)
        if result.frame.source=='LIVE':
            latency=(time.monotonic_ns()-result.frame.timestamp_ns)/1e6
            cv2.putText(canvas,f'Capture to display: {latency:.0f}ms',(810,22),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        if self.backend=='tk':
            if self.root is None:self._init_tk()
            import tkinter as tk
            rgb=cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB)
            ppm=f'P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n'.encode()+rgb.tobytes()
            self.photo=tk.PhotoImage(data=ppm,format='PPM')
            self.label.configure(image=self.photo)
            self.root.update_idletasks()
            self.root.update()
        else:
            cv2.imshow('AutoKart Phase 1',canvas)
            key=cv2.waitKey(1)&0xff
            if key!=255:self._key(chr(key))
        return not self.closed

    def close(self):
        if self.root is not None:self.root.destroy();self.root=None
        if self.backend=='opencv':cv2.destroyAllWindows()
