import { act } from 'react';
import { createRoot } from 'react-dom/client';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { registerCleanup } from './testUtils';

// jsdom has no WebGL: MissionScene builds a real THREE.WebGLRenderer on mount. Same stub
// as missionControlLive.test.tsx; this suite tests the Lab's data/UI contract, not WebGL.
vi.mock('three', async () => {
  const actual = await vi.importActual<typeof import('three')>('three');
  class FakeWebGLRenderer {
    domElement = document.createElement('canvas');
    setPixelRatio() {}
    setSize() {}
    setClearColor() {}
    render() {}
    dispose() {}
  }
  return { ...actual, WebGLRenderer: FakeWebGLRenderer };
});

const { LabApp } = await import('../apps/lab/LabApp');
const { LAB_ROUTES } = await import('../apps/lab/routes');
const { assertLabUrl } = await import('../apps/lab/services');

class FakeResizeObserver { observe() {} unobserve() {} disconnect() {} }

type Listener = ((event: any) => void) | null;
class MockWebSocket {
  static instances: MockWebSocket[] = [];
  onopen: Listener = null; onmessage: Listener = null; onclose: Listener = null; onerror: Listener = null;
  closed = false;
  constructor(public url: string) { MockWebSocket.instances.push(this); }
  send() {}
  close() { if (this.closed) return; this.closed = true; this.onclose?.({}); }
}

const DIAGNOSTIC = {
  algorithm_version: 'shared-python-test', configuration_sha256: 'abc123def4567890', source: 'SIMULATED', mission: 'STRAIGHT_TRACK',
  reasons: [], mission_state: { state: 'DIAGNOSTIC' },
  observations: [
    { id: 0, label: 'blue', confidence: 0.95, xy: [2, 1.5], distance_m: 2.5 },
    { id: 1, label: 'yellow', confidence: 0.95, xy: [2, -1.5], distance_m: 2.5 },
    { id: 2, label: 'blue', confidence: 0.95, xy: [5, 1.5], distance_m: 5.2 },
    { id: 3, label: 'yellow', confidence: 0.95, xy: [5, -1.5], distance_m: 5.2 },
  ],
  geometry: {
    status: 'OK', delaunay_edges: [[0, 1], [0, 2], [1, 2], [1, 3], [2, 3]], delaunay_triangles: [[0, 1, 2], [1, 2, 3]],
    gabriel_edges: [[0, 1], [0, 2], [1, 2], [1, 3], [2, 3]],
    boundaries: { left_ids: [0, 2], right_ids: [1, 3], status: 'CANDIDATE' },
    gates: [{ left_id: 0, right_id: 1, midpoint: [2, 0], width: 3, quality: 0.9, status: 'CANDIDATE' },
      { left_id: 2, right_id: 3, midpoint: [5, 0], width: 3, quality: 0.9, status: 'CANDIDATE' }],
    rejected_candidates: [{ left_id: 0, right_id: 3, reason: 'NOT_SELECTED' }],
    centerline: [[2, 0], [5, 0]], centerline_status: 'PROVISIONAL', skidpad_topology: null,
  },
  planning: { path_valid: true },
  control_candidate: { valid: true, target_speed_mps: 1.2, steering_angle_rad: 0.01, target_point_base_m: [4, 0], published_to_vcu: false },
  vehicle_authorized: false, processing_ms: 1.5,
};
const SNAPSHOT = { source: 'SIMULATED', tick: 7, cones: [], kart: { x_m: 0, y_m: 0, yaw_rad: 0, speed_mps: 0.5 }, odometry: { x_m: 0, y_m: 0, yaw_rad: 0 }, diagnostic: DIAGNOSTIC };
const STATUS = { status: 'SIMULATED', vehicle_authorized: false, algorithm_version: 'shared-python-test', configuration_sha256: 'abc123def4567890' };

const json = (body: unknown, status = 200) => ({ ok: status < 400, status, headers: { get: () => 'application/json' }, json: async () => body });
let requested: string[] = [];
let gatewayStatus: unknown = null;

function stubNetwork() {
  requested = [];
  MockWebSocket.instances = [];
  vi.stubGlobal('fetch', vi.fn(async (input: RequestInfo | URL) => {
    const url = String(input);
    requested.push(url);
    if (url === '/api/core/status') return json(STATUS);
    if (/^\/api\/core\/(snapshot|step|reset|move-cone)$/.test(url)) return json(SNAPSHOT);
    if (url === '/mission/api/status' && gatewayStatus) return json(gatewayStatus);
    return { ok: false, status: 502, headers: { get: () => 'text/plain' }, json: async () => ({}) };
  }));
  vi.stubGlobal('WebSocket', MockWebSocket as unknown as typeof WebSocket);
  vi.stubGlobal('ResizeObserver', FakeResizeObserver);
  vi.stubGlobal('requestAnimationFrame', () => 1);
  vi.stubGlobal('cancelAnimationFrame', () => undefined);
}

async function settle(ms = 30) { await act(async () => { await new Promise((resolve) => setTimeout(resolve, ms)); }); }

async function waitFor(check: () => boolean, label: string, timeout = 4000) {
  const start = Date.now();
  while (!check()) {
    if (Date.now() - start > timeout) throw new Error(`Timed out waiting for ${label}`);
    await settle(25);
  }
}

async function renderAt(path: string) {
  window.history.replaceState(null, '', path);
  const container = document.createElement('div');
  document.body.appendChild(container);
  const root = createRoot(container);
  registerCleanup(() => root.unmount());
  await act(async () => { root.render(<LabApp />); });
  await waitFor(() => !!container.querySelector('[data-page]'), `page at ${path}`);
  await settle();
  return container;
}

const text = (container: HTMLElement) => container.textContent ?? '';
const activeProvenance = (container: HTMLElement) => container.querySelector('[data-testid="provenance-active"]')?.textContent ?? '';

async function chooseFile(input: HTMLInputElement, file: File) {
  Object.defineProperty(input, 'files', { value: [file], configurable: true });
  await act(async () => { input.dispatchEvent(new Event('change', { bubbles: true })); });
  await settle(50);
}

function frame(source: string, sequence: number) {
  return { schema_version: 1, source, run_id: `rec-${source.toLowerCase()}`, sequence, timestamp_s: sequence * 0.1,
    vehicle: { x_m: sequence, y_m: 0, yaw_rad: 0, speed_mps: 2 }, perception: { cones: [] },
    planning: { delaunay_edges: [], gabriel_edges: [], gates: [], candidate_paths: [], selected_path: [] }, safety: {}, metrics: {} };
}

beforeEach(() => {
  gatewayStatus = null;
  try { window.sessionStorage.clear(); } catch { /* ignore */ }
  stubNetwork();
});
afterEach(() => vi.unstubAllGlobals());

describe('AutoKart Algorithm Lab: routing and shell', () => {
  it.each(LAB_ROUTES.map((route) => route.path))('renders %s on direct load with the Lab header, nav and provenance banner', async (path) => {
    const container = await renderAt(path);
    expect(window.location.pathname).toBe(path);
    expect(container.querySelector('.lab-app')?.getAttribute('data-route')).toBe(path);
    expect(text(container)).toContain('AUTOKART ALGORITHM LAB');
    const banner = container.querySelector('[data-testid="provenance-banner"]');
    expect(banner).not.toBeNull();
    expect(Array.from(banner!.querySelectorAll('[data-provenance]')).map((chip) => chip.getAttribute('data-provenance'))).toEqual(['SYNTHETIC', 'REPLAY', 'LIVE READ-ONLY']);
    expect(banner!.querySelector('[data-provenance="LIVE READ-ONLY"]')?.getAttribute('aria-disabled')).toBe('true');
    expect(activeProvenance(container)).toMatch(/^ACTIVE: (SYNTHETIC|REPLAY|NONE)/);
    expect(container.querySelector(`nav[aria-label="Lab pages"] a[aria-current="page"]`)?.getAttribute('href')).toBe(path);
    expect(container.querySelector('h1')).not.toBeNull();
    expect(container.querySelector(`[data-page]`)?.getAttribute('data-page')).toBe(path.slice(1));
  });

  it('redirects / to /home and the legacy /shared to /core', async () => {
    await renderAt('/');
    expect(window.location.pathname).toBe('/home');
    const core = await renderAt('/shared');
    expect(window.location.pathname).toBe('/core');
    expect(core.querySelector('[data-page="core"]')).not.toBeNull();
  });

  it('does not serve Vehicle Dashboard paths and shows a not-found page for unknown paths', async () => {
    const live = await renderAt('/live');
    expect(text(live)).toContain('Not part of the Algorithm Lab');
    expect(activeProvenance(live)).toContain('ACTIVE: NONE');
    const unknown = await renderAt('/no-such-page');
    expect(text(unknown)).toContain('No such Lab page');
  });

  it('navigates client-side without a reload and keeps the provenance banner', async () => {
    const container = await renderAt('/home');
    const link = container.querySelector<HTMLAnchorElement>('nav[aria-label="Lab pages"] a[href="/gates"]')!;
    await act(async () => { link.dispatchEvent(new MouseEvent('click', { bubbles: true, cancelable: true, button: 0 })); });
    await waitFor(() => !!container.querySelector('[data-page="gates"]'), 'gates page');
    expect(window.location.pathname).toBe('/gates');
    expect(container.querySelector('[data-testid="provenance-banner"]')).not.toBeNull();
  });
});

describe('AutoKart Algorithm Lab: provenance and honesty', () => {
  it('labels Python CORE pages SYNTHETIC with real backend values', async () => {
    const core = await renderAt('/core');
    await waitFor(() => text(core).includes('shared-python-test'), 'CORE status');
    expect(activeProvenance(core)).toContain('ACTIVE: SYNTHETIC');
    expect(activeProvenance(core)).toContain('lab_api :8767');
  });

  it.each(['/perception', '/depth-calibration', '/benchmark', '/faults'])('%s shows an explicit not-implemented state and no fake output', async (path) => {
    const container = await renderAt(path);
    const state = container.querySelector('[data-testid="not-implemented"]');
    expect(state?.textContent).toContain('NOT IMPLEMENTED IN THIS BUILD');
    expect(text(container)).toContain('What is missing');
    expect(activeProvenance(container)).toContain('ACTIVE: NONE');
    const page = container.querySelector('[data-page]')!;
    expect(page.querySelector('svg, canvas, img, table')).toBeNull();
    expect(page.textContent).not.toMatch(/[✓✔✅]|\bPASS\b|\bHEALTHY\b/);
  });

  it('dataset page says the real camera is unavailable and claims no provenance without a recording', async () => {
    const container = await renderAt('/dataset');
    expect(container.querySelector('[data-testid="camera-unavailable"]')?.textContent).toContain('REAL CAMERA: NOT AVAILABLE IN THIS BUILD');
    expect(activeProvenance(container)).toContain('ACTIVE: NONE');
    expect(container.querySelector('[data-page] img')).toBeNull();
  });

  it('stage pages render only backend geometry (gates with provenance ids, Gabriel cross-check agrees)', async () => {
    const gates = await renderAt('/gates');
    await waitFor(() => text(gates).includes('Selected gates (2)'), 'gates table');
    expect(text(gates)).toContain('Rejected candidate pairs (1)');
    expect(text(gates)).toContain('NOT_SELECTED');
    const gabriel = await renderAt('/gabriel');
    await waitFor(() => text(gabriel).includes('Delaunay edges tested (5)'), 'gabriel table');
    expect(text(gabriel)).not.toContain('DISAGREES');
    const paths = await renderAt('/paths');
    await waitFor(() => text(paths).includes('SIM-ONLY / DIAGNOSTIC'), 'controller candidate');
    expect(text(paths)).toContain('PUBLISHED TO VCU');
  });

  it('home offers SYNTHETIC and REPLAY; LIVE READ-ONLY is a disabled option with its reason', async () => {
    const container = await renderAt('/home');
    const radios = Array.from(container.querySelectorAll<HTMLInputElement>('input[name="run-mode"]'));
    expect(radios.map((radio) => [radio.value, radio.disabled])).toEqual([['SYNTHETIC', false], ['REPLAY', false], ['LIVE READ-ONLY', true]]);
    expect(text(container)).toContain('no network route to the live vehicle gateway');
    expect(text(container)).toContain('SIM ROS GATEWAY :8775 · OFFLINE');
  });
});

describe('AutoKart Algorithm Lab: REAL_KART and live-service isolation', () => {
  it('mission view offers only sandbox, ROS simulation and replay sources', async () => {
    const container = await renderAt('/mission');
    const source = container.querySelector<HTMLSelectElement>('select[aria-label="Data source"]')!;
    expect(Array.from(source.options).map((option) => option.value)).toEqual(['LAB', 'SIMULATION', 'REPLAY']);
    expect(text(container)).toContain('SEE');
    expect(text(container)).toContain('CONTROL');
    expect(activeProvenance(container)).toContain('ACTIVE: SYNTHETIC');
  });

  it('no route exposes a selectable REAL_KART / live control', async () => {
    for (const { path } of LAB_ROUTES) {
      const container = await renderAt(path);
      await settle(60);
      const options = Array.from(container.querySelectorAll('option'));
      expect(options.filter((option) => /REAL|LIVE/i.test(`${option.value} ${option.textContent}`))).toEqual([]);
      const enabledLive = Array.from(container.querySelectorAll<HTMLInputElement | HTMLButtonElement>('input, button'))
        .filter((element) => !element.disabled && /REAL[_ ]KART|LIVE READ-ONLY/i.test(`${(element as HTMLInputElement).value ?? ''} ${element.textContent ?? ''}`));
      expect(enabledLive).toEqual([]);
    }
  });

  it('simulation source connects only to the Lab /mission socket and shows the offline gateway honestly', async () => {
    const container = await renderAt('/mission');
    const source = container.querySelector<HTMLSelectElement>('select[aria-label="Data source"]')!;
    await act(async () => { source.value = 'SIMULATION'; source.dispatchEvent(new Event('change', { bubbles: true })); });
    await settle(80);
    expect(MockWebSocket.instances.length).toBeGreaterThan(0);
    for (const socket of MockWebSocket.instances) expect(new URL(socket.url).pathname).toBe('/mission/ws');
    expect(MockWebSocket.instances.every((socket) => !/:876[58]/.test(socket.url))).toBe(true);
    expect(text(container)).toContain('NO CURRENT SIMULATION FRAME');
    expect(activeProvenance(container)).toContain('ACTIVE: NONE');
  });

  it('a REAL_KART recording is replayed from file only (REPLAY provenance, no socket, no live option)', async () => {
    const container = await renderAt('/replay');
    expect(text(container)).toContain('No recording loaded');
    const file = new File([[frame('REAL_KART', 1), frame('REAL_KART', 2), frame('REAL_KART', 3)].map((item) => JSON.stringify(item)).join('\n')], 'kart.jsonl');
    await chooseFile(container.querySelector<HTMLInputElement>('input[type="file"]')!, file);
    await waitFor(() => !!container.querySelector('select[aria-label="Data source"]'), 'replay mission view');
    await waitFor(() => activeProvenance(container).includes('ACTIVE: REPLAY'), 'REPLAY provenance');
    expect(text(container)).toContain('captured from the REAL KART');
    expect(MockWebSocket.instances).toEqual([]);
    const options = Array.from(container.querySelector<HTMLSelectElement>('select[aria-label="Data source"]')!.options).map((option) => option.value);
    expect(options).toEqual(['LAB', 'SIMULATION', 'REPLAY']);
    // Stepping a replay stays in REPLAY mode, so no live socket is opened for the recording's source.
    const step = Array.from(container.querySelectorAll('button')).find((button) => button.textContent === 'STEP')!;
    await act(async () => { step.click(); });
    await settle(60);
    expect(MockWebSocket.instances).toEqual([]);
    expect(activeProvenance(container)).toContain('ACTIVE: REPLAY');
  });

  it('refuses a simulation gateway that reports REAL_KART instead of displaying it', async () => {
    gatewayStatus = { source: 'REAL_KART', mode: 'LIVE', vehicle_authorized: false };
    const container = await renderAt('/home');
    await waitFor(() => text(container).includes('SIM ROS GATEWAY :8775 · REFUSED'), 'refused gateway');
  });

  it('never requests live perception endpoints or live ports from any route', async () => {
    for (const { path } of LAB_ROUTES) {
      await renderAt(path);
      await settle(60);
    }
    expect(requested.length).toBeGreaterThan(0);
    for (const url of requested) {
      expect(url.startsWith('/api/core/') || url.startsWith('/mission/')).toBe(true);
      expect(url).not.toMatch(/\/api\/(snapshot|image)|:8765|:8768/);
    }
    for (const socket of MockWebSocket.instances) expect(socket.url).not.toMatch(/:8765|:8768/);
  });

  it('the Lab network guard rejects live endpoints outright', () => {
    expect(() => assertLabUrl('/api/snapshot')).toThrow(/refusing/);
    expect(() => assertLabUrl('/api/image/latest')).toThrow(/refusing/);
    expect(() => assertLabUrl('http://127.0.0.1:8765/mission/ws')).toThrow(/refusing/);
    expect(() => assertLabUrl('/api/core/status')).not.toThrow();
  });
});
