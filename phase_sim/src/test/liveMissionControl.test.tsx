import { act } from 'react';
import { createRoot } from 'react-dom/client';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { LiveMissionControl } from '../live/LiveMissionControl';
import { registerCleanup } from './testUtils';

afterEach(() => vi.unstubAllGlobals());

describe('AUT-05 current-frame display', () => {
  it('draws SDK synthetic geometry while keeping hardware qualification false', async () => {
    // LiveSession(mode='sim') publishes a base_link graph with unverified physical calibration.
    const synthetic = {
      schema_version: 1, frame_id: 'simulated-1', source: 'SIMULATED', status: 'DEGRADED',
      reasons: ['CONTROL_DEPENDENCY_UNAVAILABLE'], base_frame: 'base_link',
      calibration_status: 'MOUNT_CONFIGURED_UNVERIFIED', depth_scale_qualified: false,
      detections: [], observations: [
        { id: 1, label: 'blue', confidence: 1, xy: [2, 1], distance_m: 2.2, sigma_m: 0 },
        { id: 2, label: 'yellow', confidence: 1, xy: [2, -1], distance_m: 2.2, sigma_m: 0 },
        { id: 3, label: 'blue', confidence: 1, xy: [4, 1], distance_m: 4.1, sigma_m: 0 },
      ],
      geometry: { delaunay_edges: [[1, 2], [1, 3]], gabriel_edges: [[1, 2]],
        gates: [{ left_id: 1, right_id: 2, midpoint: [2, 0], status: 'VALID' }],
        centerline: [[1, 0], [2, 0], [3, 0]], boundaries: { left_ids: [1, 3], right_ids: [2] } },
      camera_motion: { status: 'PROVISIONAL', previous_timestamp_ns: 100,
        current_timestamp_ns: 200, matched_points: 24, inlier_count: 18,
        reprojection_rms_px: 1.25, qualified: false, frame: 'CAMERA_OPTICAL' },
      planning: { path_valid: false, local_path: [] },
      control_candidate: { valid: false, target_speed_mps: 0, steering_angle_rad: 0 },
      vehicle_authorized: false as const, ready: false,
    };
    vi.stubGlobal('fetch', vi.fn(async (url: string) => url === '/api/snapshot' ?
      { ok: true, json: async () => synthetic } : { ok: true, blob: async () => new Blob(['jpeg']) }));
    vi.stubGlobal('URL', { ...URL, createObjectURL: vi.fn(() => 'blob:synthetic'), revokeObjectURL: vi.fn() });
    const container = document.createElement('div');
    document.body.appendChild(container);
    const root = createRoot(container);
    registerCleanup(() => root.unmount());
    await act(async () => { root.render(<LiveMissionControl />); await Promise.resolve(); });
    expect(container.querySelector('.live-bev svg')?.getAttribute('aria-label'))
      .toContain('simulated SDK geometry; camera calibration unverified');
    expect(container.querySelectorAll('.live-delaunay')).toHaveLength(2);
    expect(container.querySelectorAll('.live-gabriel')).toHaveLength(1);
    expect(container.querySelectorAll('.live-gate')).toHaveLength(1);
    expect(container.querySelectorAll('.live-centerline')).toHaveLength(1);
    expect(container.textContent).toContain('3 simulated cones');
    expect(container.textContent).toContain('SIMULATED GEOMETRY · HP60C CALIBRATION UNVERIFIED');
    expect(container.textContent).toContain('CAMERA_OPTICAL provisional · 100 → 200 ns · 18/24 inliers · 1.25 px RMS');
    expect(Array.from(container.querySelectorAll('.live-details dt')).find(item => item.textContent === 'Pose')
      ?.nextElementSibling?.textContent).toBe('UNAVAILABLE');
    expect(container.textContent).toContain('VEHICLE AUTHORITY DISABLED');
    expect(container.textContent).not.toContain('VALIDATED METRIC VIEW UNAVAILABLE');
  });

  it('keeps a fixed metric viewport and labels YOLO26 end-to-end selection correctly', async () => {
    vi.stubGlobal('fetch', vi.fn(async (url: string) => url === '/api/snapshot' ?
      { ok: true, json: async () => ({
        schema_version: 1, frame_id: 'live-yolo26', source: 'LIVE', status: 'SENSOR_FAULT',
        reasons: ['INVALID_POSE'], model: { name: 'YOLO26s', supported_classes: [],
          qualification: 'EXPERIMENTAL_HP60C_VALIDATION_PENDING' },
        detections: [], observations: [{ id: 1, label: 'blue', confidence: .9,
          xy: [2, 1], distance_m: 2.2, sigma_m: .1 }],
        depth_scale_qualified: true, calibration_status: 'MOUNT_VERIFIED',
        detector_counts: { raw_output_rows: 300, after_nms: 1 },
        geometry: { delaunay_edges: [], gabriel_edges: [], gates: [], centerline: [],
          boundaries: { left_ids: [], right_ids: [] } },
        planning: { path_valid: false, local_path: [] },
        control_candidate: { valid: false, target_speed_mps: 0, steering_angle_rad: 0 },
        vehicle_authorized: false, ready: false,
      }) } : { ok: true, blob: async () => new Blob(['jpeg']) }));
    vi.stubGlobal('URL', { ...URL, createObjectURL: vi.fn(() => 'blob:fixed'), revokeObjectURL: vi.fn() });
    const container = document.createElement('div');
    document.body.appendChild(container);
    const root = createRoot(container);
    registerCleanup(() => root.unmount());
    await act(async () => { root.render(<LiveMissionControl />); await Promise.resolve(); });
    expect(container.textContent).toContain('fixed 50 px/m');
    expect(container.textContent).toContain('End-to-end selected');
    expect(container.textContent).not.toContain('After NMS');
    expect(container.querySelector('.live-bev svg circle')?.getAttribute('cx')).toBe('270');
    expect(container.querySelector('.live-bev svg circle')?.getAttribute('cy')).toBe('340');
  });

  it('watermarks replay faults and clears geometry instead of showing an old path', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => ({ ok: true, json: async () => ({
      schema_version: 1, frame_id: null, source: 'REPLAY', status: 'SENSOR_FAULT',
      reasons: ['REPLAY_ARRAY_INTEGRITY_FAILURE'], detections: [], observations: [],
      geometry: { delaunay_edges: [], gabriel_edges: [], gates: [], centerline: [],
        boundaries: { left_ids: [], right_ids: [] } },
      planning: { path_valid: false, local_path: [] },
      control_candidate: { valid: false, target_speed_mps: 0, steering_angle_rad: 0 },
      vehicle_authorized: false, ready: false,
    }) })));
    const container = document.createElement('div');
    document.body.appendChild(container);
    const root = createRoot(container);
    registerCleanup(() => root.unmount());
    await act(async () => { root.render(<LiveMissionControl />); await Promise.resolve(); });
    expect(container.textContent).toContain('REPLAY');
    expect(container.textContent).toContain('REPLAY_ARRAY_INTEGRITY_FAILURE');
    expect(container.textContent).toContain('Geometry and motion candidates are cleared');
    expect(container.textContent).toContain('No current camera image');
    expect(container.textContent).toContain('VEHICLE AUTHORITY DISABLED');
    expect(container.querySelectorAll('.live-delaunay')).toHaveLength(0);
  });

  it('keeps current RGB and depth visible when perception rejects the path', async () => {
    vi.stubGlobal('fetch', vi.fn(async (url: string) => url === '/api/snapshot' ?
      { ok: true, json: async () => ({
        schema_version: 1, frame_id: 'live-1', source: 'LIVE', status: 'SENSOR_FAULT',
        reasons: ['CENTERLINE_AMBIGUOUS'],
        model: { name: 'ConeNet v1', supported_classes: ['blue', 'large_orange', 'orange', 'unknown', 'yellow'],
          qualification: 'EXPERIMENTAL_HP60C_VALIDATION_PENDING' },
        detections: ['blue', 'large_orange', 'orange', 'unknown', 'yellow'].map((label, id) =>
          ({ id, label, confidence: 0.8, box: [id, 1, id + 2, 3] })), observations: [],
        depth_scale_qualified: false,
        perception: { diagnostics: { cone_details: [{ id: 0, localization_status: 'METRIC_OK',
          depth_status: 'DEPTH_OK', distance_m: .5 }] } },
        geometry: { delaunay_edges: [], gabriel_edges: [], gates: [], centerline: [],
          boundaries: { left_ids: [], right_ids: [] } },
        planning: { path_valid: false, local_path: [] },
        control_candidate: { valid: false, target_speed_mps: 0, steering_angle_rad: 0 },
        vehicle_authorized: false, ready: false, camera_streaming: true,
      }) } : { ok: true, blob: async () => new Blob(['jpeg']) }));
    vi.stubGlobal('URL', { ...URL, createObjectURL: vi.fn(() => 'blob:current'), revokeObjectURL: vi.fn() });
    const container = document.createElement('div');
    document.body.appendChild(container);
    const root = createRoot(container);
    registerCleanup(() => root.unmount());
    await act(async () => { root.render(<LiveMissionControl />); await Promise.resolve(); });
    expect(container.textContent).toContain('Current images remain visible for diagnosis');
    expect(container.textContent).toContain('STREAMING · UNQUALIFIED');
    expect(container.textContent).toContain('ConeNet v1 · EXPERIMENTAL_HP60C_VALIDATION_PENDING');
    for (const label of ['blue', 'large_orange', 'orange', 'unknown', 'yellow']) {
      expect(container.textContent).toContain(label);
    }
    expect(container.textContent).toContain('0.50 m raw-scale candidate · unverified · RAW_SCALE_CANDIDATE');
    expect(container.textContent).not.toContain('METRIC_OK');
    expect(container.querySelectorAll('.live-image img')).toHaveLength(2);
  });

  it('retains only a marked historical live view through a stale camera gap', async () => {
    const frame = {
      schema_version: 1, frame_id: 'live-42', source: 'LIVE', status: 'SENSOR_FAULT',
      reasons: ['INVALID_POSE'], detections: [],
      observations: [{ id: 1, label: 'yellow', confidence: .8, xy: [1, .5], distance_m: 1.1, sigma_m: .1 }],
      geometry: { delaunay_edges: [], gabriel_edges: [], gates: [], centerline: [],
        boundaries: { left_ids: [], right_ids: [] } },
      planning: { path_valid: false, local_path: [] },
      control_candidate: { valid: false, target_speed_mps: 0, steering_angle_rad: 0 },
      vehicle_authorized: false, ready: false, camera_streaming: true,
    };
    const stale = { ...frame, frame_id: null, status: 'STALE', reasons: ['STALE_FRAME'],
      observations: [], camera_streaming: false };
    let snapshots = 0;
    vi.stubGlobal('fetch', vi.fn(async (url: string) => url === '/api/snapshot' ?
      { ok: true, json: async () => ++snapshots === 1 ? frame : stale } :
      { ok: true, blob: async () => new Blob(['jpeg']) }));
    vi.stubGlobal('URL', { ...URL, createObjectURL: vi.fn(() => 'blob:frame'), revokeObjectURL: vi.fn() });
    const container = document.createElement('div');
    document.body.appendChild(container);
    const root = createRoot(container);
    registerCleanup(() => root.unmount());
    await act(async () => { root.render(<LiveMissionControl />); await Promise.resolve(); });
    await act(async () => { await new Promise(resolve => setTimeout(resolve, 250)); });
    expect(snapshots).toBeGreaterThan(1);
    expect(container.textContent).toContain('STALE_FRAME');
    expect(container.textContent).toContain('Geometry and motion candidates are cleared');
    expect(container.querySelectorAll('.live-image img')).toHaveLength(2);
    expect(container.querySelectorAll('.live-historical-label')).toHaveLength(3);
    expect(container.querySelector('.live-bev svg')?.getAttribute('viewBox')).toBe('0 0 640 480');
    expect(container.textContent).toContain('VALIDATED METRIC VIEW UNAVAILABLE');
    expect(container.textContent).toContain('VEHICLE AUTHORITY DISABLED');
  });

  it('shows nominal candidates separately from validated geometry and lists rejections', async () => {
    vi.stubGlobal('fetch', vi.fn(async (url: string) => url === '/api/snapshot' ?
      { ok: true, json: async () => ({
        schema_version: 1, frame_id: 'live-candidate', source: 'LIVE', status: 'SENSOR_FAULT',
        reasons: ['CALIBRATION_UNVERIFIED'], base_frame: 'camera_ground_nominal',
        depth_scale_qualified: false, calibration_status: 'MOUNT_CONFIGURED_UNVERIFIED',
        detections: [], observations: [], candidate_observations: [{ id: 4, label: 'orange',
          confidence: .8, xy: [2, 1], distance_m: 2.2, sigma_m: .2 }],
        provisional_3d_observations: [{ id: 6, label: 'yellow', confidence: .7,
          nominal_xy_m: [1, -.5], nominal_body_xyz_m: [1, -.5, .1],
          optical_xyz_m: [.5, .1, 1], uncertainty_m: .2,
          ground_contact_status: 'UNAVAILABLE_CONE_FOOT_CLIPPED',
          position_kind: 'PROVISIONAL_3D_POSITION' }],
        rejected_candidates: [{ id: 5, localization_status: 'INSUFFICIENT_VALID_DEPTH',
          box: [10, 20, 30, 480] }],
        geometry: { delaunay_edges: [], gabriel_edges: [], gates: [], centerline: [],
          boundaries: { left_ids: [], right_ids: [] } },
        planning: { path_valid: false, local_path: [] },
        control_candidate: { valid: false, target_speed_mps: 0, steering_angle_rad: 0 },
        vehicle_authorized: false, ready: false,
      }) } : { ok: true, blob: async () => new Blob(['jpeg']) }));
    const container = document.createElement('div');
    document.body.appendChild(container);
    const root = createRoot(container);
    registerCleanup(() => root.unmount());
    await act(async () => { root.render(<LiveMissionControl />); await Promise.resolve(); });
    expect(container.textContent).toContain('camera_ground_nominal');
    expect(container.textContent).toContain('1 ground candidates · 1 body samples · 0 validated');
    expect(container.textContent).toContain('#5 INSUFFICIENT_VALID_DEPTH');
    expect(container.textContent).toContain('#6 provisional body');
    expect(container.textContent).toContain('#6 (1.00, -0.50) m ±0.20 m UNAVAILABLE_CONE_FOOT_CLIPPED');
    expect(container.querySelector('.live-bev svg circle[stroke="#ffd284"]')?.getAttribute('cx')).toBe('270');
    expect(container.querySelectorAll('.live-delaunay')).toHaveLength(0);
  });

  it('marks a published frame stale when processing exceeds its freshness limit', async () => {
    vi.stubGlobal('fetch', vi.fn(async (url: string) => url === '/api/snapshot' ?
      { ok: true, json: async () => ({
        schema_version: 1, frame_id: 'live-late', source: 'LIVE', status: 'STALE',
        reasons: ['STALE_FRAME'], last_good_age_ms: 15,
        latencies_ms: { capture_to_publish: 175 }, detections: [], observations: [],
        geometry: { delaunay_edges: [], gabriel_edges: [], gates: [], centerline: [],
          boundaries: { left_ids: [], right_ids: [] } },
        planning: { path_valid: false, local_path: [] },
        control_candidate: { valid: false, target_speed_mps: 0, steering_angle_rad: 0 },
        vehicle_authorized: false, ready: false,
      }) } : { ok: true, blob: async () => new Blob(['jpeg']) }));
    vi.stubGlobal('URL', { ...URL, createObjectURL: vi.fn(() => 'blob:late'), revokeObjectURL: vi.fn() });
    const container = document.createElement('div');
    document.body.appendChild(container);
    const root = createRoot(container);
    registerCleanup(() => root.unmount());
    await act(async () => { root.render(<LiveMissionControl />); await Promise.resolve(); });
    expect(container.querySelectorAll('.live-historical-label')).toHaveLength(3);
    expect(container.textContent).toContain('Stale images are diagnostic only');
    expect(container.textContent).toContain('0.2 s old');
    expect(container.textContent).toContain('STALE · NOT READY');
  });

  it('keeps boxes, counters, and both images on the same accepted frame when an older snapshot arrives', async () => {
    const frame = (id: number) => ({
      schema_version: 1, frame_id: `live-${id}`, source: 'LIVE', status: 'SENSOR_FAULT',
      capture_timestamp_ns: id, reasons: ['INVALID_POSE'], camera_sequence: id,
      depth_sequence: id, detections: [{ id, label: 'orange', confidence: .8,
        box: [10, 20, 30, 40] }], observations: [],
      detector_counts: { raw_output_rows: 8400, after_nms: 1, application_detections: 1 },
      geometry: { delaunay_edges: [], gabriel_edges: [], gates: [], centerline: [],
        boundaries: { left_ids: [], right_ids: [] } },
      planning: { path_valid: false, local_path: [] },
      control_candidate: { valid: false, target_speed_mps: 0, steering_angle_rad: 0 },
      vehicle_authorized: false as const, ready: false, camera_streaming: true,
    });
    let polls = 0;
    const requests: string[] = [];
    vi.stubGlobal('fetch', vi.fn(async (url: string) => {
      requests.push(url);
      if (url === '/api/snapshot') {
        polls++;
        return { ok: true, json: async () => polls === 1 ? frame(20) : frame(19) };
      }
      return { ok: true, blob: async () => new Blob([url]) };
    }));
    vi.stubGlobal('URL', { ...URL, createObjectURL: vi.fn((blob: Blob) =>
      `blob:${blob.size}:${requests.at(-1)}`), revokeObjectURL: vi.fn() });
    const container = document.createElement('div');
    document.body.appendChild(container);
    const root = createRoot(container);
    registerCleanup(() => root.unmount());
    await act(async () => { root.render(<LiveMissionControl />); await Promise.resolve(); });
    await act(async () => { await new Promise(resolve => setTimeout(resolve, 230)); });
    expect(polls).toBeGreaterThan(1);
    expect(container.textContent).toContain('Out-of-order frame live-19 ignored');
    expect(container.textContent).toContain('live-20');
    expect(container.textContent).toContain('20 / 20');
    expect(container.textContent).toContain('1 historical detections');
    expect(container.textContent).toContain('#20 orange');
    expect(requests).toContain('/api/image/live-20/rgb');
    expect(requests).toContain('/api/image/live-20/depth');
    expect(requests).not.toContain('/api/image/live-19/rgb');
    expect(container.querySelectorAll('.live-image img:not([hidden])')).toHaveLength(2);
  });

  it('holds the last paired view through a missing depth image and resumes on reconnect', async () => {
    const frame = (id: number) => ({
      schema_version: 1, frame_id: `live-${id}`, source: 'LIVE', status: 'SENSOR_FAULT',
      capture_timestamp_ns: id, reasons: ['INVALID_POSE'], detections: id === 2 ? [] :
        [{ id, label: 'orange', confidence: .8, box: [1, 2, 3, 4] }], observations: [],
      geometry: { delaunay_edges: [], gabriel_edges: [], gates: [], centerline: [],
        boundaries: { left_ids: [], right_ids: [] } },
      planning: { path_valid: false, local_path: [] },
      control_candidate: { valid: false, target_speed_mps: 0, steering_angle_rad: 0 },
      vehicle_authorized: false as const, ready: false, camera_streaming: true,
    });
    let polls = 0;
    let allowRecovery = false;
    vi.stubGlobal('fetch', vi.fn(async (url: string) => {
      if (url === '/api/snapshot') return { ok: true, json: async () => {
        polls++;
        return frame(polls === 1 ? 1 : allowRecovery ? 3 : 2);
      } };
      return { ok: !url.includes('/live-2/depth'), blob: async () => new Blob([url]) };
    }));
    vi.stubGlobal('URL', { ...URL, createObjectURL: vi.fn(() => `blob:${Math.random()}`),
      revokeObjectURL: vi.fn() });
    const container = document.createElement('div');
    document.body.appendChild(container);
    const root = createRoot(container);
    registerCleanup(() => root.unmount());
    await act(async () => { root.render(<LiveMissionControl />); await Promise.resolve(); });
    await act(async () => { await new Promise(resolve => setTimeout(resolve, 125)); });
    expect(container.textContent).toContain('Matching frame images expired');
    expect(container.textContent).toContain('live-1');
    expect(container.textContent).toContain('#1 orange');
    allowRecovery = true;
    await act(async () => { await new Promise(resolve => setTimeout(resolve, 160)); });
    expect(container.textContent).toContain('live-3');
    expect(container.textContent).toContain('#3 orange');
    expect(container.textContent).not.toContain('Matching frame images expired');
  });
});
