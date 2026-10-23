import time
import resource
import hashlib
from dataclasses import replace
import numpy as np
from .depth import validate_frame, localize_diagnostic, DepthError
from .mount import mount_from_config
from .calibration_store import artifact_summary, verify_artifact
from .geometry import triangulate
from .gabriel import filter_gabriel
from .gates import select_track_gates
from .boundaries import estimate_boundaries
from .centerline import generate_centerline
from .curvature import profile_centerline
from .temporal import StationaryDisplayFilter
from .types import Result, Cone, Detection
from .interfaces import DetectorProtocol
from .synthetic import synthetic_cones
from .safety import assess, topology_reasons

# Per-frame data-quality outcomes (no localizable cone foot, sparse depth, one
# unpaired timestamp, SDK intrinsics during warmup) invalidate that frame only.
# Only these sensor/integrity failures persist until an operator clears them.
LATCHING_FAULTS={'INVALID_SOURCE','REPLAY_INTEGRITY_FAILURE','INFERENCE_FAILURE','INVALID_FRAME_FORMAT',
    'DEPTH_ALIGNMENT_UNAVAILABLE','MOUNT_CALIBRATION_UNAVAILABLE','MOUNT_CALIBRATION_INVALID',
    'MODEL_NOT_AVAILABLE','CALIBRATION_MISMATCH','FRAME_FROZEN','FRAME_OUT_OF_ORDER'}

class Pipeline:
    def __init__(self,detector: DetectorProtocol | None,config: dict,synthetic: bool=False):
        self.detector=detector; self.config=config; self.synthetic=synthetic
        self.mount,self.mount_status=mount_from_config(config)
        self.depth_scale_factor=1.0; self.calibration_reasons=[]; self.artifact_intrinsics=None
        artifact=config.get('calibration_artifact')
        if artifact and self.mount_status=='MOUNT_CONFIGURED_UNVERIFIED':
            status,self.calibration_reasons,factor=verify_artifact(artifact,self.mount)
            if status=='MOUNT_VERIFIED':
                self.mount_status=status;self.depth_scale_factor=factor
                identity=(artifact_summary(artifact) or {}).get('device_identity') or {}
                self.artifact_intrinsics=identity.get('intrinsics')
        self.prev_end_ns=None; self.prev_cpu=None; self.prev_rgb_ts=None; self.prev_depth_ts=None
        self.prev_infer_ns=None
        self.display_filter=StationaryDisplayFilter()
        self.processed_frames=0
        self.last_live_stamp=None
        self.last_replay_stamp=None
        self.latched_fault_reason=None

    def note_sensor_fault(self,reason):
        """Latch safety-critical input loss until a new process is started."""
        if self.latched_fault_reason is None:self.latched_fault_reason=str(reason)
        self.display_filter.reset()

    def clear_sensor_fault(self):
        """Explicit operator acknowledgement; returns the cleared reason. Never called automatically."""
        reason,self.latched_fault_reason=self.latched_fault_reason,None
        self.display_filter.reset();self.last_live_stamp=None
        return reason

    def set_mount(self,changes):
        """Apply a measured bench pose to subsequent frames; never mark it surveyed."""
        limits={'height_m':(.05,2.0),'pitch_down_deg':(-60.0,75.0),
                'roll_deg':(-45.0,45.0),'yaw_left_deg':(-180.0,180.0)}
        if not isinstance(changes,dict) or set(changes)!=set(limits):
            raise ValueError('INVALID_MOUNT_FIELDS')
        values={}
        for key,(low,high) in limits.items():
            value=changes[key]
            if type(value) not in (int,float):
                raise ValueError(f'INVALID_{key.upper()}')
            try:value=float(value)
            except OverflowError:raise ValueError(f'INVALID_{key.upper()}') from None
            if not np.isfinite(value) or not low<=value<=high:
                raise ValueError(f'INVALID_{key.upper()}')
            values[key]=value
        old=self.config.get('camera_mount')
        if not isinstance(old,dict) or old.get('height_reference')!='optical_center':
            raise ValueError('MOUNT_CONFIG_NOT_EDITABLE')
        candidate={**self.config,'camera_to_kart':None,'camera_mount':{**old,**values}}
        matrix,status=mount_from_config(candidate)
        if status!='MOUNT_CONFIGURED_UNVERIFIED':raise ValueError(status)
        self.config=candidate;self.mount=matrix;self.mount_status=status
        return self.mount_parameters()

    def mount_parameters(self):
        params=self.config.get('camera_mount')
        return {key:params[key] for key in ('height_m','x_forward_m','y_left_m',
            'pitch_down_deg','roll_deg','yaw_left_deg')} if isinstance(params,dict) else None

    def process(self,frame):
        calibration_status,calibration_reasons=self.mount_status,list(self.calibration_reasons)
        if self.mount_status=='MOUNT_VERIFIED' and frame.source=='LIVE' and self.artifact_intrinsics:
            current=vars(frame.intrinsics)
            if any(abs(float(current[k])-float(self.artifact_intrinsics[k]))>0.5 for k in ('fx','fy','cx','cy')):
                calibration_status='MOUNT_CONFIGURED_UNVERIFIED';calibration_reasons.append('CALIBRATION_CAMERA_MISMATCH')
        if frame.source=='LIVE' and self.depth_scale_factor!=1.0:
            frame=replace(frame,depth_scale=frame.depth_scale*self.depth_scale_factor)
        start=time.perf_counter(); start_ns=time.monotonic_ns()
        mount_hash=hashlib.sha256(np.asarray(self.mount,dtype='<f8').tobytes()).hexdigest()
        diagnostics={'source':frame.source,'camera':'CONNECTED' if frame.source=='LIVE' else frame.source,
            'model':('READY' if self.detector.verified_checkpoint else 'USER_MODEL_UNVERIFIED') if self.detector else
                    'NOT_USED_SYNTHETIC' if frame.source=='SYNTHETIC' and self.synthetic else 'MODEL_NOT_AVAILABLE',
            'model_version':self.detector.version if self.detector else 'none',
            'inference_device':self.detector.backend if self.detector else 'none',
            'confidence_threshold':getattr(self.detector,'confidence',self.config['confidence']),
            'nms_iou_threshold':getattr(self.detector,'iou',self.config['nms_iou']),
            'calibration_status':calibration_status,'calibration_reasons':calibration_reasons,
            'depth_scale_factor':self.depth_scale_factor,
            'rgb_depth_registration':('SDK_REPORTS_REGISTERED' if frame.aligned else 'DEPTH_ALIGNMENT_UNAVAILABLE')
                if frame.source=='LIVE' else 'SYNTHETIC_ASSUMED' if frame.source=='SYNTHETIC' else 'REPLAY_RECORDED',
            'cone_details':[],
            'dropped_frames':frame.drops,
            'frame_age_ms':(start_ns-frame.timestamp_ns)/1e6 if frame.source=='LIVE' else 0.0,
            'calibration_sha256':mount_hash,
            'stage_timestamps_ns':{'frame':frame.timestamp_ns,'processing_start':start_ns}}
        if frame.source=='LIVE':
            stamp=(frame.timestamp_ns,frame.rgb_sequence,frame.depth_sequence)
            if self.last_live_stamp is not None:
                diagnostics['frame_frozen']=stamp==self.last_live_stamp
                diagnostics['frame_out_of_order']=any(new<old for new,old in zip(stamp,self.last_live_stamp))
            self.last_live_stamp=stamp
        elif frame.source=='REPLAY':
            stamp=(frame.timestamp_ns,frame.rgb_sequence,frame.depth_sequence)
            if self.last_replay_stamp is not None:
                diagnostics['frame_frozen']=stamp==self.last_replay_stamp
                diagnostics['frame_out_of_order']=any(new<old for new,old in zip(stamp,self.last_replay_stamp))
            self.last_replay_stamp=stamp
            recorded_hash=frame.calibration.get('recorded_mount_sha256')
            diagnostics['replay_provenance_missing']=recorded_hash is None
            diagnostics['calibration_mismatch']=recorded_hash is not None and recorded_hash!=mount_hash
        invalid_sequence=diagnostics.get('frame_frozen') or diagnostics.get('frame_out_of_order')
        detections=[]; cones=[]; status='OK';inference_fps=0.0
        if invalid_sequence:
            status='FRAME_FROZEN' if diagnostics.get('frame_frozen') else 'FRAME_OUT_OF_ORDER'
        elif frame.source=='REPLAY' and 'recorded_cones' in frame.calibration:
            try:
                validate_frame(frame,self.config['max_frame_age_ms'])
                cones=[Cone(**item) for item in frame.calibration['recorded_cones']]
                detections=[Detection(**item) for item in frame.calibration.get('recorded_detections',[])]
                if any(not np.isfinite(c.xy).all() or c.sigma_m<0 for c in cones):
                    raise ValueError('INVALID_RECORDED_CONE')
                status='RECORDED_OBSERVATIONS'
                diagnostics['model']=f"RECORDED:{frame.calibration.get('recorded_model_version','unknown')}"
                diagnostics['cone_details']=frame.calibration.get('recorded_diagnostics',{}).get('cone_details',[])
            except (DepthError,ValueError,TypeError) as e:
                cones=[];detections=[];status=f'INVALID_REPLAY:{e}'
        elif self.synthetic and frame.source=='SYNTHETIC':
            cones=synthetic_cones(); status='SYNTHETIC_GEOMETRY'
        else:
            try: validate_frame(frame,self.config['max_frame_age_ms'])
            except DepthError as e: status=str(e)
            if self.detector and status=='OK':
                try:
                    detections=self.detector.detect(frame.rgb)
                    diagnostics['inference_ms']=self.detector.latency_ms
                    diagnostics['detector_counts']=dict(getattr(self.detector,'last_counts',{}))
                    infer_ns=time.monotonic_ns()
                    if self.prev_infer_ns and infer_ns>self.prev_infer_ns:
                        inference_fps=1e9/(infer_ns-self.prev_infer_ns)
                    self.prev_infer_ns=infer_ns
                    diagnostics['stage_timestamps_ns']['detections']=infer_ns
                except Exception as e:
                    status=f'INFERENCE_FAILURE:{e}'
            elif status=='OK':
                status='MODEL_NOT_AVAILABLE'
            if status=='OK' and detections:
                depth_start=time.perf_counter()
                for detection in detections:
                    cone,detail=localize_diagnostic(frame,detection,self.mount,self.config)
                    if frame.source=='LIVE':
                        detail['validation_reasons']=(([] if calibration_status=='MOUNT_VERIFIED' else
                            ['DEPTH_SCALE_UNVERIFIED','MOUNT_TO_BASE_LINK_UNVERIFIED']) if cone is not None else
                            [detail['localization_status']])
                    diagnostics['cone_details'].append(detail)
                    if cone is not None:cones.append(cone)
                diagnostics['depth_localization_ms']=(time.perf_counter()-depth_start)*1000
                if self.mount_status not in ('MOUNT_CONFIGURED_UNVERIFIED','MOUNT_VERIFIED'):
                    status=self.mount_status
                elif not cones:status='METRIC_LOCALIZATION_FAILED'
            elif detections and status!='OK':
                diagnostics['cone_details']=[{'id':d.id,'label':d.label,'confidence':d.confidence,
                    'box':d.box,'valid_depth_pixels':0,'required_depth_pixels':self.config['min_depth_pixels'],
                    'depth_status':status,'localization_status':status} for d in detections]
        diagnostics['stage_timestamps_ns'].setdefault('detections',time.monotonic_ns())
        diagnostics['stage_timestamps_ns']['depth']=time.monotonic_ns()
        detections=[replace(d,observed_ns=d.observed_ns or frame.timestamp_ns) for d in detections]
        cones=[replace(c,observed_ns=c.observed_ns or frame.timestamp_ns) for c in cones]
        if 'manual_annotations' in frame.calibration:
            from .evaluation import match, box_iou
            truth=frame.calibration['manual_annotations']
            _,matched=match({0:detections},{0:truth},.5)
            diagnostics['annotated_visible_cones']=len(truth)
            diagnostics['visible_cone_recall']=len(matched)/len(truth) if truth else None
            diagnostics['annotated_false_positives']=len(detections)-len(matched)
            used=set()
            for detection in sorted(detections,key=lambda item:item.confidence,reverse=True):
                available=[(box_iou(detection.box,item['box']),i) for i,item in enumerate(truth) if i not in used]
                if not available:continue
                overlap,index=max(available)
                if overlap<.5:continue
                used.add(index);reference=truth[index]
                detail=next((item for item in diagnostics['cone_details'] if item['id']==detection.id),None)
                if detail is None:continue
                surveyed=reference.get('xy_m')
                measured=reference.get('distance_m')
                if measured is None and surveyed is not None:measured=float(np.linalg.norm(surveyed))
                if measured is not None:
                    detail['measured_distance_m']=measured
                    if 'distance_m' in detail:detail['distance_error_m']=detail['distance_m']-measured
                if surveyed is not None:
                    detail['survey_xy_m']=surveyed
                    if 'ground_xy_m' in detail:
                        detail['position_error_m']=float(np.linalg.norm(np.asarray(detail['ground_xy_m'])-surveyed))
        filtered_detections=[];filtered_cones=[];predicted=[]
        if frame.source=='LIVE' and status not in ('STALE_FRAME','INVALID_FRAME_FORMAT',
                'CALIBRATION_UNAVAILABLE','TIMESTAMP_MISMATCH','DEPTH_ALIGNMENT_UNAVAILABLE') and \
                not status.startswith('INFERENCE_FAILURE') and not invalid_sequence:
            filtered_detections,filtered_cones,predicted=self.display_filter.update(
                detections,cones,frame.timestamp_ns)
        else:self.display_filter.reset()
        geometry_start=time.perf_counter()
        unqualified_recorded_live=(frame.source=='REPLAY' and
            frame.calibration.get('recorded_source') not in ('SYNTHETIC','SIMULATED'))
        graph_cones=[c for c in cones if frame.source!='LIVE' and not unqualified_recorded_live and
                     np.isfinite(c.sigma_m) and
                     0<=c.sigma_m<=self.config.get('max_cone_sigma_m',.6) and
                     np.isfinite(c.confidence) and c.confidence>0]
        graph=triangulate(graph_cones)
        by_id={}
        for cone in graph_cones:by_id.setdefault(cone.id,cone)
        graph_cones=[by_id[cone_id] for cone_id in graph.ids]
        graph_end=time.perf_counter()
        gabriel=filter_gabriel(graph)
        gabriel_end=time.perf_counter()
        boundaries=estimate_boundaries(graph_cones,self.config)
        boundary_end=time.perf_counter()
        gates=select_track_gates(graph_cones,graph.edges,boundaries,self.config)
        gates=[replace(g,observed_ns=frame.timestamp_ns) for g in gates]
        gate_end=time.perf_counter()
        diagnostics['stage_timestamps_ns']['gates']=time.monotonic_ns()
        centerline,center_status=generate_centerline(gates,self.config['min_centerline_gates'],self.config['max_lateral_jump_m'],
            self.config['max_heading_change_deg'],self.config['max_width_change_m'],graph_cones)
        path_profile,curvature_status=profile_centerline(centerline,
            max_abs_curvature_m_inv=self.config.get('max_abs_curvature_m_inv',2.0))
        path_profile=[replace(p,observed_ns=frame.timestamp_ns) for p in path_profile]
        diagnostics['stage_timestamps_ns']['path']=time.monotonic_ns()
        topology_errors=topology_reasons(gates,centerline,graph_cones,
            self.config['min_track_width_m'],self.config['max_track_width_m'])
        diagnostics['rejected_gates']=[{'id':g.id,'left_id':g.left_id,'right_id':g.right_id,
            'reason':g.status} for g in gates if g.status!='CANDIDATE']
        if topology_errors:diagnostics['topology_rejection_reasons']=topology_errors
        diagnostics['geometry_ms']=(time.perf_counter()-geometry_start)*1000
        diagnostics.update({'delaunay_ms':(graph_end-geometry_start)*1000,
            'gabriel_ms':(gabriel_end-graph_end)*1000,
            'boundary_ms':(boundary_end-gabriel_end)*1000,
            'gate_ms':(gate_end-boundary_end)*1000,
            'centerline_curvature_ms':(time.perf_counter()-gate_end)*1000,
            'graph_eligible_cones':len(graph_cones),
            'boundary_mode':boundaries.mode,'boundary_status':boundaries.status,
            'boundary_reason':boundaries.reason,'left_boundary_cones':len(boundaries.left_ids),
            'right_boundary_cones':len(boundaries.right_ids),
            'centerline_status':center_status,'curvature_status':curvature_status,
            'max_abs_curvature_m_inv':max((abs(p.curvature_m_inv) for p in path_profile
                                        if p.curvature_m_inv is not None),default=None)})
        if status in ('OK','RECORDED_OBSERVATIONS') and center_status!='PROVISIONAL': status=center_status
        end_ns=time.monotonic_ns();cpu=time.process_time()
        self.processed_frames+=1
        elapsed_s=(end_ns-self.prev_end_ns)/1e9 if self.prev_end_ns else 0
        cpu_pct=100*(cpu-self.prev_cpu)/elapsed_s if self.prev_cpu is not None and elapsed_s>0 else 0
        rgb_fps=1000/(frame.rgb_timestamp-self.prev_rgb_ts) if self.prev_rgb_ts and frame.rgb_timestamp>self.prev_rgb_ts else 0
        depth_fps=1000/(frame.depth_timestamp-self.prev_depth_ts) if self.prev_depth_ts and frame.depth_timestamp>self.prev_depth_ts else 0
        self.prev_end_ns=end_ns;self.prev_cpu=cpu;self.prev_rgb_ts=frame.rgb_timestamp;self.prev_depth_ts=frame.depth_timestamp
        valid_depth_pct=0.0
        if isinstance(frame.depth,np.ndarray) and frame.depth.shape==(480,640) and frame.depth.dtype==np.uint16:
            depths=frame.depth.astype(np.float32)*frame.depth_scale
            valid_depth_pct=100*float(np.count_nonzero((depths>=self.config['depth_min_m']) &
                (depths<=self.config['depth_max_m'])))/frame.depth.size
        diagnostics.update({'detected_cones':len(detections),'localized_cones':len(cones),
            'application_detections':len(detections),
            'rejected_metric_cones':max(0,len(detections)-len(cones)),'inference_fps':inference_fps,
            'filtered_display_cones':len(filtered_cones),'predicted_display_cones':len(predicted),
            'received_frames':frame.received_frames,'processed_frames':self.processed_frames,
            'capture_elapsed_s':frame.capture_elapsed_ns/1e9,
            'mailbox_overwrites':frame.drops,
            'mailbox_overwrite_pct':100*frame.drops/frame.received_frames if frame.received_frames else 0,
            'valid_depth_pct':valid_depth_pct,
            'processing_ms':(time.perf_counter()-start)*1000,'graph':graph.status,
            'delaunay_edges':len(graph.edges),'gabriel_edges':len(gabriel),'candidate_gates':len(gates),
            'accepted_gate_candidates':sum(g.status=='CANDIDATE' for g in gates),
            'rgb_fps':rgb_fps,'depth_fps':depth_fps,'end_to_end_fps':1/elapsed_s if elapsed_s>0 else 0,
            'cpu_pct':cpu_pct,'peak_memory_mb':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024})
        produced_ns=time.monotonic_ns()
        diagnostics['stage_timestamps_ns']['assessment']=produced_ns
        age_ms=(produced_ns-frame.timestamp_ns)/1e6 if frame.source=='LIVE' else None
        diagnostics['frame_age_ms']=age_ms if age_ms is not None else 0.0
        safety=assess(frame.source,frame.timestamp_ns,produced_ns,age_ms,status,diagnostics,
            boundaries,gates,centerline,path_profile,topology_errors,self.config)
        latching=[r for r in safety.reasons if r in LATCHING_FAULTS]
        if safety.state=='SENSOR_FAULT' and latching:self.note_sensor_fault(latching[0])
        if self.latched_fault_reason is not None:
            safety=replace(safety,state='SENSOR_FAULT',
                reasons=tuple(dict.fromkeys((*safety.reasons,'MANUAL_RESTART_REQUIRED',
                                             self.latched_fault_reason))))
        if safety.state in ('STALE','SENSOR_FAULT','AMBIGUOUS'):
            centerline=[];path_profile=[]
        diagnostics.update({'safety_state':safety.state,'safety_reasons':safety.reasons,
            'speed_limit_advisory_mps':safety.speed_limit_advisory_mps,
            'vehicle_authorized':safety.vehicle_authorized,'path_confidence':safety.confidence})
        return Result(frame,detections,cones,graph,gabriel,gates,centerline,status,diagnostics,
                      boundaries,path_profile,filtered_detections,filtered_cones,predicted,safety)
