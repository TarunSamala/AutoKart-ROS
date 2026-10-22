import { act } from 'react';
import { createRoot } from 'react-dom/client';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { RosLivePanel } from '../mission/RosLivePanel';
import { registerCleanup } from './testUtils';

afterEach(() => { vi.unstubAllGlobals(); vi.useRealTimers(); });

class FakeWS {
  static instances: FakeWS[] = [];
  onopen: (() => void) | null = null; onmessage: ((event: { data: string }) => void) | null = null;
  onclose: unknown; onerror: (() => void) | null = null;
  sent: string[] = [];
  close() {}
  send(value: string) { this.sent.push(value); }
  constructor(public url: string) { FakeWS.instances.push(this); }
}
beforeEach(() => { FakeWS.instances = []; vi.stubGlobal('WebSocket', FakeWS); });

const baseStatus = {
  source: 'SIMULATION', mode: 'LIVE', ros_connected: true,
  odometry: { current: true, state: 'CURRENT', age_ms: 10, payload: null },
  vcu: { current: true, state: 'CURRENT', age_ms: 10, payload: { faults: [] } },
  safety: { current: true, state: 'CURRENT', age_ms: 10, payload: null },
  stages: {
    perception: { current: false, state: 'MISSING', age_ms: null, payload: null },
    mapping: { current: false, state: 'MISSING', age_ms: null, payload: null },
    planning: { current: false, state: 'MISSING', age_ms: null, payload: null },
  },
  vehicle_authorized: false as const,
};

const baseCamera = {
  available: true, frame_id: 'f1', source: 'SIMULATED', width: 640, height: 480,
  sequence: 1, valid: true, depth_qualified: true, detections: [],
};

function mockGateway(status: Record<string, unknown>, camera: Record<string, unknown>) {
  vi.stubGlobal('fetch', vi.fn(async (url: string) => {
    if (url === '/mission/api/status') return { ok: true, json: async () => status };
    if (url === '/mission/api/camera') return { ok: true, json: async () => camera };
    return { ok: true, json: async () => ({}) };
  }));
}

async function render(status: Record<string, unknown>, camera: Record<string, unknown>,
  source: 'SIMULATION' | 'REAL_KART' = 'SIMULATION',
  demoDriveProps: { sendDemoDrive?: (m: Record<string, unknown>) => boolean;
    demoDriveRejected?: string | null; wsConnected?: boolean } = {}) {
  mockGateway(status, camera);
  const container = document.createElement('div');
  document.body.appendChild(container);
  const root = createRoot(container);
  registerCleanup(() => root.unmount());
  await act(async () => {
    root.render(<RosLivePanel source={source} frame={null} {...demoDriveProps} />);
    await new Promise((resolve) => setTimeout(resolve, 0));
    await new Promise((resolve) => setTimeout(resolve, 0));
  });
  return container;
}

describe('RC-06 front safety envelope display', () => {
  it('shows the active commissioning gap and fixed red zone from planner status', async () => {
    const status = { ...baseStatus, stages: { ...baseStatus.stages,
      planning: { current: true, state: 'CURRENT', age_ms: 10,
        payload: { source: 'SIMULATED', valid: false, decision: 'STOP_CANDIDATE',
          stop_gap_from_bumper_m: 1.0, red_zone_m: 0.15 } } } };
    const container = await render(status, baseCamera);
    expect(container.textContent).toContain('ACTIVE BUMPER STOP GAP');
    expect(container.textContent).toContain('1.00 m · COMMISSIONING');
    expect(container.textContent).toContain('0.15 m · FRONT BUMPER');
  });

  it('shows BRAKING with clearance, trigger distance and a schematic overlay', async () => {
    const status = { ...baseStatus, safety_envelope: {
      current: true, state: 'CURRENT', age_ms: 42,
      payload: { state: 'BRAKING', reason: 'OBSTACLE_WITHIN_BRAKING_ENVELOPE:CALIBRATION_UNVERIFIED', valid: true,
        calibration_verified: false, clear_distance_m: 2.0, stopping_distance_m: 2.8, guaranteed_deceleration_mps2: 2.0,
        data_source: 'SIMULATED' } } };
    const container = await render(status, baseCamera);
    expect(container.textContent).toContain('FRONT SAFETY ENVELOPE · BRAKING');
    expect(container.textContent).toContain('2.00 m from bumper');
    expect(container.textContent).toContain('2.80 m · provisional');
    expect(container.querySelector('.mc-safety-overlay')?.getAttribute('aria-label')).toContain('BRAKING');
    expect(container.querySelector('.mc-safety-overlay')?.getAttribute('aria-label')).toContain('geometry unverified');
    expect(container.textContent).toContain('GEOMETRY UNVERIFIED');
    expect(container.textContent).toContain('UNVERIFIED · schematic overlay only');
  });

  it('never shows CLEAR when the /safety/envelope topic has stopped publishing', async () => {
    const status = { ...baseStatus, safety_envelope: { current: false, state: 'MISSING', age_ms: null, payload: null } };
    const container = await render(status, baseCamera);
    expect(container.textContent).toContain('FRONT SAFETY ENVELOPE · UNKNOWN');
    expect(container.textContent).toContain('MISSING ·');
    expect(container.textContent).not.toContain('FRONT SAFETY ENVELOPE · CLEAR');
  });

  it('tracks /safety/envelope freshness independently of the odometry channel', async () => {
    const status = {
      ...baseStatus,
      odometry: { current: false, state: 'STALE', age_ms: 900, payload: null },
      safety_envelope: { current: true, state: 'CURRENT', age_ms: 10,
        payload: { state: 'CLEAR', reason: 'NO_OBSTACLE_IN_MONITORED_CORRIDOR', valid: true,
          calibration_verified: false, clear_distance_m: null, stopping_distance_m: 1.3, guaranteed_deceleration_mps2: 2.0,
          data_source: 'SIMULATED' } },
    };
    const container = await render(status, baseCamera);
    // Odometry staleness does not gate the envelope channel; it is sampled and reported independently.
    expect(container.textContent).toContain('FRONT SAFETY ENVELOPE · CLEAR');
    expect(container.textContent).toContain('NO OBSTACLE IN CORRIDOR');
    // No odometry frame reached this render, so speed must read UNKNOWN, never fabricated.
    expect(container.textContent).toContain('VEHICLE AUTHORITY');
    expect(container.textContent).toContain('OFF · DIAGNOSTIC_ONLY');
  });

  it('shows CRITICAL state and highlights the minimum-clearance band', async () => {
    const status = { ...baseStatus, safety_envelope: { current: true, state: 'CURRENT', age_ms: 5,
      payload: { state: 'CRITICAL', reason: 'OBSTACLE_WITHIN_MINIMUM_CLEARANCE:CALIBRATION_UNVERIFIED', valid: true,
        calibration_verified: false, clear_distance_m: 0.4, stopping_distance_m: 2.1, guaranteed_deceleration_mps2: 2.0,
        data_source: 'SIMULATED' } } };
    const container = await render(status, baseCamera);
    expect(container.textContent).toContain('FRONT SAFETY ENVELOPE · CRITICAL');
    expect(container.textContent).toContain('0.40 m from bumper');
  });

  it('suppresses camera-derived overlay and detections when the camera is unavailable', async () => {
    const status = { ...baseStatus, safety_envelope: { current: true, state: 'CURRENT', age_ms: 5,
      payload: { state: 'CLEAR', reason: 'NO_OBSTACLE_IN_MONITORED_CORRIDOR', valid: true,
        calibration_verified: false, clear_distance_m: null, stopping_distance_m: 1.3, guaranteed_deceleration_mps2: 2.0,
        data_source: 'SIMULATED' } } };
    const container = await render(status, { available: false, reason: 'CAMERA_PAIR_MISSING' });
    expect(container.querySelector('.mc-safety-overlay')).toBeNull();
    expect(container.textContent).toContain('CAMERA DISCONNECTED');
    // Status/state readout stays independent of camera availability.
    expect(container.textContent).toContain('FRONT SAFETY ENVELOPE · CLEAR');
  });
});

describe('RC-06 audit finding B: synthetic data must remain identifiable as synthetic', () => {
  it('flags data missing a provenance tag as UNVERIFIED, never silently as hardware-validated', async () => {
    const status = { ...baseStatus, safety_envelope: { current: true, state: 'CURRENT', age_ms: 5,
      payload: { state: 'CLEAR', reason: 'NO_OBSTACLE_IN_MONITORED_CORRIDOR', valid: true,
        calibration_verified: false, clear_distance_m: null, stopping_distance_m: 1.3, guaranteed_deceleration_mps2: 2.0 } } };
    const container = await render(status, baseCamera);
    expect(container.textContent).toContain('DATA SOURCE UNVERIFIED');
    expect(container.textContent).toContain('NOT A PHYSICAL SENSOR READING');
    expect(container.querySelector('.mc-safety-overlay')?.getAttribute('aria-label')).toContain('data source UNVERIFIED');
  });

  it('warns when a real-kart dashboard receives a SIMULATED envelope instead of LIVE', async () => {
    const status = { ...baseStatus, source: 'REAL_KART', safety_envelope: { current: true, state: 'CURRENT', age_ms: 5,
      payload: { state: 'BRAKING', reason: 'OBSTACLE_WITHIN_BRAKING_ENVELOPE', valid: true,
        calibration_verified: false, clear_distance_m: 2.0, stopping_distance_m: 2.8, guaranteed_deceleration_mps2: 2.0,
        data_source: 'SIMULATED' } } };
    const container = await render(status, { ...baseCamera, source: 'LIVE' }, 'REAL_KART');
    expect(container.textContent).toContain('SIMULATED · MISMATCH (dashboard expects LIVE)');
    expect(container.textContent).toContain('DATA SOURCE: SIMULATED');
    expect(container.textContent).toContain('NOT A PHYSICAL SENSOR READING');
  });

  it('shows no synthetic-data warning when data_source is genuinely LIVE on a real-kart dashboard', async () => {
    const status = { ...baseStatus, source: 'REAL_KART', safety_envelope: { current: true, state: 'CURRENT', age_ms: 5,
      payload: { state: 'CLEAR', reason: 'NO_OBSTACLE_IN_MONITORED_CORRIDOR', valid: true,
        calibration_verified: false, clear_distance_m: null, stopping_distance_m: 1.3, guaranteed_deceleration_mps2: 2.0,
        data_source: 'LIVE' } } };
    const container = await render(status, { ...baseCamera, source: 'LIVE' }, 'REAL_KART');
    expect(container.textContent).toContain('DATA SOURCE');
    expect(container.textContent).not.toContain('NOT A PHYSICAL SENSOR READING');
    expect(container.querySelector('.mc-safety-overlay')?.getAttribute('aria-label')).toContain('data source LIVE');
    expect(container.querySelector('.mc-safety-overlay')?.textContent).not.toContain('NOT A PHYSICAL SENSOR');
  });
});

describe('/ops bounded safety requests: ARM and RESET FAULT', () => {
  const liveStatus = { ...baseStatus, source: 'REAL_KART', safety_requests_enabled: true };

  it('renders ARM and RESET FAULT buttons alongside the existing confirm/stop controls', async () => {
    const container = await render(liveStatus, baseCamera, 'REAL_KART');
    expect(container.textContent).toContain('Request ARM (refused unless every gate and the manual power cut hold)');
    expect(container.textContent).toContain('RESET FAULT');
    expect(container.textContent).toContain('Confirm physical power cut within reach for next ARM');
    expect(container.textContent).toContain('SOFTWARE STOP - NOT AN EMERGENCY STOP');
  });

  it('sends only the whitelisted arm/reset_fault/stop/confirm_manual_power_cut action strings', async () => {
    const container = await render(liveStatus, baseCamera, 'REAL_KART');
    const findButton = (text: string) => [...container.querySelectorAll('button')].find(b => b.textContent?.startsWith(text))!;
    await act(async () => { findButton('Confirm physical power cut').click(); });
    await act(async () => { findButton('Request ARM').click(); });
    await act(async () => { findButton('RESET FAULT').click(); });
    await act(async () => { findButton('SOFTWARE STOP').click(); });
    const requests = FakeWS.instances;
    await act(async () => { requests.forEach(socket => socket.onopen?.()); });
    expect(requests.map(socket => JSON.parse(socket.sent[0]).action)).toEqual(
      ['confirm_manual_power_cut', 'arm', 'reset_fault', 'stop']);
  });

  it('surfaces a backend refusal of ARM rather than assuming it was accepted', async () => {
    const container = await render(liveStatus, baseCamera, 'REAL_KART');
    const armButton = [...container.querySelectorAll('button')].find(b => b.textContent?.startsWith('Request ARM'))!;
    await act(async () => { armButton.click(); });
    const socket = FakeWS.instances.at(-1)!;
    await act(async () => { socket.onopen?.(); });
    await act(async () => { socket.onmessage?.({ data: JSON.stringify({ type: 'request_rejected' }) }); });
    expect(container.textContent).toContain('Request rejected: arm');
  });

  it('does not send ARM when live safety request forwarding is disabled', async () => {
    const container = await render({ ...liveStatus, safety_requests_enabled: false }, baseCamera, 'REAL_KART');
    const armButton = [...container.querySelectorAll('button')].find(b => b.textContent?.startsWith('Request ARM'))!;
    await act(async () => { armButton.click(); });
    expect(FakeWS.instances.length).toBe(0);
    expect(container.textContent).toContain('Live safety request forwarding unavailable');
  });
});

describe('DEMO DRIVE panel: real motion, not autonomy', () => {
  const liveStatus = { ...baseStatus, source: 'REAL_KART', safety_requests_enabled: true };

  it('gates HOLD TO DRIVE behind the area-clear checkbox', async () => {
    const container = await render(liveStatus, baseCamera, 'REAL_KART', { sendDemoDrive: () => true, wsConnected: true });
    const hold = [...container.querySelectorAll('button')].find(b => b.textContent?.startsWith('HOLD TO DRIVE'))! as HTMLButtonElement;
    expect(hold.disabled).toBe(true);
    const checkbox = container.querySelector('input[type="checkbox"]') as HTMLInputElement;
    await act(async () => { checkbox.click(); });
    expect(hold.disabled).toBe(false);
  });

  it('holding sends one drive message then a ping cadence; releasing sends stop and clears the interval', async () => {
    const sent: Record<string, unknown>[] = [];
    const container = await render(liveStatus, baseCamera, 'REAL_KART',
      { sendDemoDrive: (m) => { sent.push(m); return true; }, wsConnected: true });
    const checkbox = container.querySelector('input[type="checkbox"]') as HTMLInputElement;
    await act(async () => { checkbox.click(); });
    const hold = [...container.querySelectorAll('button')].find(b => b.textContent?.startsWith('HOLD TO DRIVE'))!;
    await act(async () => { hold.dispatchEvent(new MouseEvent('mousedown', { bubbles: true })); });
    expect(sent[0]).toEqual({ action: 'drive', direction: 'forward', duration_s: 2.0 });
    await act(async () => { await new Promise((resolve) => setTimeout(resolve, 450)); });
    const pings = sent.filter(m => m.action === 'ping').length;
    expect(pings).toBeGreaterThanOrEqual(2);
    await act(async () => { hold.dispatchEvent(new MouseEvent('mouseup', { bubbles: true })); });
    expect(sent.at(-1)).toEqual({ action: 'stop' });
    const afterStop = sent.length;
    await act(async () => { await new Promise((resolve) => setTimeout(resolve, 400)); });
    expect(sent.length).toBe(afterStop); // interval was really cleared, not still pinging
  }, 10000);

  it('STOP DEMO DRIVE always sends stop, even if never armed or holding', async () => {
    const sent: Record<string, unknown>[] = [];
    const container = await render(liveStatus, baseCamera, 'REAL_KART',
      { sendDemoDrive: (m) => { sent.push(m); return true; }, wsConnected: true });
    const stop = [...container.querySelectorAll('button')].find(b => b.textContent?.startsWith('STOP DEMO DRIVE'))!;
    await act(async () => { stop.dispatchEvent(new MouseEvent('click', { bubbles: true })); });
    expect(sent).toEqual([{ action: 'stop' }]);
  });

  it('surfaces a backend demo-drive rejection rather than assuming the drive request succeeded', async () => {
    const container = await render(liveStatus, baseCamera, 'REAL_KART',
      { sendDemoDrive: () => true, wsConnected: true, demoDriveRejected: 'drive' });
    expect(container.textContent).toContain('rejected: drive');
  });

  it('never renders the DEMO DRIVE panel for a SIMULATION-sourced view', async () => {
    const container = await render({ ...baseStatus }, baseCamera, 'SIMULATION');
    expect(container.textContent).not.toContain('DEMO DRIVE');
  });
});
