import { act } from 'react';
import { createRoot } from 'react-dom/client';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { App, ROUTES } from '../apps/dashboard/App';
import { registerCleanup } from './testUtils';

type Json = Record<string, unknown>;
const snap = (over: Json = {}): Json => ({
  schema_version: 1, frame_id: 'live-1', source: 'LIVE', status: 'SENSOR_FAULT', reasons: ['CALIBRATION_UNVERIFIED'],
  capture_timestamp_ns: 100, camera_sequence: 1, calibration_status: 'MOUNT_CONFIGURED_UNVERIFIED', depth_scale_qualified: false,
  frame_age_ms: 40, detections: [{ id: 0, label: 'orange', confidence: 0.67, box: [10, 10, 60, 50] }], observations: [], candidate_observations: [],
  geometry: { delaunay_edges: [], gabriel_edges: [], gates: [], centerline: [], boundaries: { left_ids: [], right_ids: [] } },
  planning: { path_valid: false, local_path: [] }, control_candidate: { valid: false, target_speed_mps: 0, steering_angle_rad: 0 },
  vehicle_authorized: false, ready: false, ...over,
});
const chan = (state: string, payload: Json | null = null, age: number | null = 10) => ({ current: state === 'CURRENT', state, age_ms: age, payload });
const gateway = (over: Json = {}): Json => ({ source: 'REAL_KART', mode: 'LIVE', safety_requests_enabled: true, ros_connected: true, run_id: 'r', scenario: null,
  odometry: chan('CURRENT'), vcu: chan('CURRENT', { faults: [] }), safety: chan('MISSING', null, null),
  safety_envelope: chan('MISSING', null, null), stages: { perception: chan('MISSING', null, null), mapping: chan('MISSING', null, null), planning: chan('MISSING', null, null) },
  read_only: true, vehicle_authorized: false, ...over });
const pipeline = { schema_version: 1, service: { name: 'svc', mode: 'live', source: 'LIVE', camera_owner: true }, overall: 'CALIBRATION_UNVERIFIED',
  hops: [{ name: 'sdk_capture', state: 'OK' }, { name: 'detector', state: 'OK' }, { name: 'geometry', state: 'CALIBRATION_UNVERIFIED', reason: 'MOUNT_CONFIGURED_UNVERIFIED' }, { name: 'http', state: 'OK' }] };

type Cfg = { snapshot?: Json | 'down'; gateway?: Json | 'down'; pipeline?: Json | '404' | 'down' };
function mock(cfg: Cfg) {
  vi.stubGlobal('fetch', vi.fn(async (url: string) => {
    const ok = (b: unknown) => ({ ok: true, status: 200, json: async () => b });
    if (url === '/api/snapshot') { if (cfg.snapshot === 'down') throw new Error('ECONNREFUSED'); return ok(cfg.snapshot ?? snap()); }
    if (url === '/mission/api/status') { if (cfg.gateway === 'down') throw new Error('ECONNREFUSED'); return ok(cfg.gateway ?? gateway()); }
    if (url === '/api/v1/diagnostics/pipeline') {
      if (cfg.pipeline === 'down') throw new Error('ECONNREFUSED');
      if (cfg.pipeline === '404') return { ok: false, status: 404, json: async () => ({}) };
      return ok(cfg.pipeline ?? pipeline);
    }
    if (url.startsWith('/api/image/')) return { ok: true, status: 200, blob: async () => new Blob(['x']) };
    return { ok: false, status: 404, json: async () => ({}) };
  }));
}
class FakeWS {
  static instances: FakeWS[] = [];
  onopen: (() => void) | null = null; onmessage: unknown; onclose: unknown; onerror: unknown;
  sent: string[] = [];
  close() {}
  send(value: string) { this.sent.push(value); }
  constructor(public url: string) { FakeWS.instances.push(this); }
}

beforeEach(() => {
  FakeWS.instances = [];
  vi.stubGlobal('WebSocket', FakeWS);
  vi.stubGlobal('URL', Object.assign(URL, { createObjectURL: vi.fn(() => 'blob:x'), revokeObjectURL: vi.fn() }));
  HTMLCanvasElement.prototype.getContext = (() => null) as never;
});
afterEach(() => { vi.unstubAllGlobals(); window.history.replaceState(null, '', '/'); });

async function render(path: string, cfg: Cfg) {
  mock(cfg);
  window.history.replaceState(null, '', path);
  const container = document.createElement('div'); document.body.appendChild(container);
  const root = createRoot(container); registerCleanup(() => root.unmount());
  await act(async () => { root.render(<App />); for (let i = 0; i < 6; i++) await new Promise(r => setTimeout(r, 20)); });
  return container;
}

describe('AutoKart Vehicle Dashboard', () => {
  it('shows the not-fitted warning and bounded safety request controls', async () => {
    const c = await render('/overview', {});
    expect(c.textContent).toContain('AUTOKART VEHICLE DASHBOARD');
    expect(c.querySelector('[data-testid=real-monitor-banner]')?.textContent).toBe('REAL MONITOR');
    expect(c.querySelector('select')).toBeNull();
    expect(c.textContent).toContain('E-STOP INPUT: NOT FITTED - MANUAL POWER CUT ONLY');
    expect(c.textContent).toContain('SOFTWARE STOP - NOT AN EMERGENCY STOP');
    expect(c.textContent).toContain('Confirm physical power cut within reach for next ARM');
    expect(c.textContent).not.toMatch(/REPLAY|\bLAB\b|SCRUB|INJECT/);
    expect(c.textContent).toContain('VEHICLE AUTHORITY');
    expect(c.querySelector('[data-testid=source-banner]')).toBeNull();
  });

  it('shows a loud red banner when the backend source is SIMULATED', async () => {
    const c = await render('/overview', { snapshot: snap({ source: 'SIMULATED', base_frame: 'base_link' }) });
    const b = c.querySelector('[data-testid=source-banner]');
    expect(b?.textContent).toBe('BACKEND SOURCE: SIMULATED — NOT A PHYSICAL CAMERA');
    expect(b?.className).toContain('banner-red');
  });

  it('sends only bounded requests for manual confirmation, ARM, RESET FAULT and software STOP', async () => {
    const c = await render('/overview', {});
    expect(c.textContent).toContain('Request ARM (refused unless every gate and the manual power cut hold)');
    expect(c.textContent).toContain('RESET FAULT');
    const buttons = [...c.querySelectorAll('button')];
    await act(async () => { buttons[0].click(); buttons[1].click(); buttons[2].click(); buttons[3].click(); });
    const requests = FakeWS.instances.slice(-4);
    await act(async () => { requests.forEach(socket => socket.onopen?.()); });
    expect(requests[0].sent).toEqual([JSON.stringify({ type: 'request', action: 'confirm_manual_power_cut' })]);
    expect(requests[1].sent).toEqual([JSON.stringify({ type: 'request', action: 'arm' })]);
    expect(requests[2].sent).toEqual([JSON.stringify({ type: 'request', action: 'reset_fault' })]);
    expect(requests[3].sent).toEqual([JSON.stringify({ type: 'request', action: 'stop' })]);
  });

  it('surfaces a backend refusal of an ARM request rather than assuming it succeeded', async () => {
    const c = await render('/overview', {});
    const armButton = [...c.querySelectorAll('button')].find(b => b.textContent?.startsWith('Request ARM'))!;
    await act(async () => { armButton.click(); });
    const socket = FakeWS.instances.at(-1)!;
    await act(async () => { socket.onopen?.(); });
    await act(async () => { (socket.onmessage as (e: { data: string }) => void)?.({ data: JSON.stringify({ type: 'request_rejected' }) }); });
    expect(c.textContent).toContain('Request rejected: arm');
  });

  it('names the perception service as failed while the ROS panel still renders', async () => {
    const c = await render('/ros', { snapshot: 'down', pipeline: 'down', gateway: gateway({ ros_connected: true }) });
    const strip = c.querySelector('.strip')!.textContent!;
    expect(strip).toContain('PERCEPTION SERVICEPERCEPTION_OFFLINE');
    expect(strip).toContain('CAMERAPERCEPTION_OFFLINE');
    expect(strip).toContain('ROS GATEWAYREACHABLE');
    expect(c.textContent).toContain('stage: perception');
    expect(c.textContent).toContain('odometry');
  });

  it('names the gateway as failed while perception still renders', async () => {
    const c = await render('/camera', { gateway: 'down' });
    expect(c.querySelector('.strip')!.textContent).toContain('ROS GATEWAYGATEWAY_OFFLINE');
    expect(c.textContent).toContain('live-1');
  });

  it('reports ROS_OFFLINE when the gateway is up but no topic is current', async () => {
    const c = await render('/ros', { gateway: gateway({ ros_connected: false }) });
    expect(c.querySelector('.strip')!.textContent).toContain('ROS DATAROS_OFFLINE');
  });

  it('shows STALE with age and freezes animation for a stale snapshot', async () => {
    const c = await render('/perception', { snapshot: snap({ status: 'STALE', frame_age_ms: 5000 }) });
    expect(c.querySelector('.strip')!.textContent).toContain('CAMERASTALE');
    const bev = c.querySelector('[data-testid=birdseye]')!;
    expect(bev.getAttribute('data-animating')).toBe('false');
    expect(bev.textContent).toContain('FROZEN');
    expect(c.querySelector('.strip')!.textContent).not.toMatch(/CAMERALIVE/);
  });

  it('shows NO QUALIFIED LIVE GEOMETRY and animates for fresh unqualified frames', async () => {
    const c = await render('/perception', {});
    expect(c.textContent).toContain('NO QUALIFIED LIVE GEOMETRY');
    expect(c.querySelector('[data-testid=birdseye]')?.getAttribute('data-animating')).toBe('true');
    expect(c.querySelector('.strip')!.textContent).toContain('CAMERACALIBRATION_UNVERIFIED');
    expect(c.querySelector('[data-testid=frame-counter]')?.textContent).toBe('#1');
  });

  it('draws SIMULATED geometry as labelled simulated, with per-layer toggles', async () => {
    const c = await render('/perception', { snapshot: snap({ source: 'SIMULATED', base_frame: 'base_link',
      observations: [{ id: 1, label: 'blue', confidence: 1, xy: [2, 1] }, { id: 2, label: 'yellow', confidence: 1, xy: [2, -1] }],
      geometry: { delaunay_edges: [[1, 2]], gabriel_edges: [[1, 2]], gates: [], centerline: [], boundaries: { left_ids: [], right_ids: [] } } }) });
    const bev = c.querySelector('[data-testid=birdseye]')!;
    expect(bev.getAttribute('data-mode')).toBe('SIMULATED');
    expect(bev.getAttribute('data-cones')).toBe('2');
    expect(c.querySelectorAll('.layers input[type=checkbox]').length).toBe(10);
  });

  it.each(ROUTES.map(r => r.path))('renders %s on direct load', async path => {
    const c = await render(path, {});
    expect(c.querySelector('.page-title')?.textContent?.length).toBeGreaterThan(2);
    expect(c.querySelector('main .card')).not.toBeNull();
    expect(c.querySelector('nav a[aria-current=page]')?.getAttribute('href')).toBe(path);
  });

  it('redirects / to /overview', async () => {
    await render('/', {});
    expect(window.location.pathname).toBe('/overview');
  });

  it('never shows CLEAR when the safety envelope is stale, missing, or gateway offline', async () => {
    for (const cfg of [{ gateway: gateway({ safety_envelope: chan('STALE', null, 5000) }) }, { gateway: gateway() }, { gateway: 'down' as const }]) {
      const c = await render('/safety', cfg);
      expect(c.textContent).toContain('UNKNOWN');
      expect(c.querySelector('.big .chip')?.textContent).toBe('UNKNOWN');
      expect(c.querySelector('.strip')!.textContent).not.toContain('SAFETY ENVELOPECLEAR');
      document.body.innerHTML = '';
    }
  });

  it('never shows green CLEAR for an unverified or non-LIVE envelope', async () => {
    const env = { state: 'CLEAR', reason: 'x', valid: true, calibration_verified: false, data_source: 'SIMULATED', clear_distance_m: null, stopping_distance_m: null };
    const c = await render('/safety', { gateway: gateway({ safety_envelope: chan('CURRENT', env) }) });
    const chip = c.querySelector('.big .chip')!;
    expect(chip.textContent).toBe('CLEAR · UNVERIFIED');
    expect(chip.className).toContain('tone-warn');
    expect(c.textContent).toContain('NOT A PHYSICAL SENSOR READING');
  });

  it('handles a 404 pipeline route gracefully', async () => {
    const c = await render('/ros', { pipeline: '404' });
    expect(c.textContent).toContain('pipeline diagnostics unavailable from this service version');
  });

  it('shows missing values as NO DATA, never 0', async () => {
    const c = await render('/vehicle', {});
    expect(c.textContent).toContain('— (NO DATA)');
    expect(c.textContent).not.toMatch(/Speed \(measured\)0/);
  });

  it('reports SDK_ERROR when the camera hop faults with no frame', async () => {
    const c = await render('/camera', { snapshot: { schema_version: 1, frame_id: null, source: 'LIVE', status: 'SENSOR_FAULT', reasons: ['CameraError:CAMERA_DISCONNECTED'] },
      pipeline: { ...pipeline, hops: [{ name: 'sdk_capture', state: 'SDK_ERROR', last_error: 'CAMERA_DISCONNECTED' }] } });
    expect(c.querySelector('.strip')!.textContent).toContain('CAMERASDK_ERROR');
  });
});
