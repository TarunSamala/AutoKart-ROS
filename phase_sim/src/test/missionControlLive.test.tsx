import { act } from 'react';
import { createRoot } from 'react-dom/client';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { registerCleanup } from './testUtils';

// jsdom has no WebGL context; MissionScene.tsx constructs a real THREE.WebGLRenderer
// on mount, which throws in this environment. Stub it with a no-op renderer so the
// component tree mounts -- this test is about the live-only data/UI contract, not
// about exercising the WebGL rendering pipeline itself (unrelated to this change).
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

const { MissionControlLive } = await import('../mission/MissionControlLive');

class FakeResizeObserver {
  observe() {}
  unobserve() {}
  disconnect() {}
}

afterEach(() => vi.unstubAllGlobals());

type Listener = ((event: any) => void) | null;

/** Minimal controllable WebSocket mock so tests can drive /mission/ws by hand. */
class MockWebSocket {
  static instances: MockWebSocket[] = [];
  onopen: Listener = null;
  onmessage: Listener = null;
  onclose: Listener = null;
  onerror: Listener = null;
  closed = false;
  constructor(public url: string) {
    MockWebSocket.instances.push(this);
  }
  send() {}
  close() {
    if (this.closed) return;
    this.closed = true;
    this.onclose?.({});
  }
  open() { this.onopen?.({}); }
  message(data: unknown) { this.onmessage?.({ data: JSON.stringify(data) }); }
}

function mockGatewayUnreachable() {
  vi.stubGlobal('fetch', vi.fn(async () => ({ ok: false, status: 503 })));
}

function realKartFrame(overrides: Record<string, unknown> = {}) {
  return {
    schema_version: 1, source: 'REAL_KART', run_id: 'run-1', sequence: 1, timestamp_s: 1,
    vehicle: { x_m: 0, y_m: 0, yaw_rad: 0, speed_mps: 1 },
    perception: { cones: [] },
    planning: { delaunay_edges: [], gabriel_edges: [], gates: [], candidate_paths: [], selected_path: [] },
    safety: {},
    metrics: {},
    ...overrides,
  };
}

async function render() {
  MockWebSocket.instances = [];
  vi.stubGlobal('WebSocket', MockWebSocket as unknown as typeof WebSocket);
  vi.stubGlobal('ResizeObserver', FakeResizeObserver);
  const container = document.createElement('div');
  document.body.appendChild(container);
  const root = createRoot(container);
  registerCleanup(() => root.unmount());
  await act(async () => {
    root.render(<MissionControlLive />);
    await Promise.resolve();
  });
  return container;
}

describe('Agent 6: live-only Mission Control entry point (/ops)', () => {
  it('renders locked to REAL_KART with no reachable control to change the data source', async () => {
    mockGatewayUnreachable();
    const container = await render();

    // The data source is fixed text, never an interactive control.
    expect(container.querySelector('select[aria-label="Operating mode"]')).toBeNull();
    expect(container.querySelectorAll('select').length).toBe(0);
    expect(container.textContent).toContain('REAL_KART');
    expect(container.textContent).toContain('MISSION CONTROL · LIVE');
    expect(container.textContent).toContain('READ ONLY');

    // No lab/replay/simulation-source affordances exist anywhere in the rendered tree. DEMO DRIVE
    // is an intentional exception: it is a real, explicit, non-autonomy drive mode, not a data
    // source switcher -- so it must say so plainly, never silently, and never let a source
    // switcher hide behind the same word.
    expect(container.textContent).not.toMatch(/\bLAB\b/);
    expect(container.textContent).not.toContain('SIMULATION');
    expect(container.textContent).not.toContain('REPLAY');
    expect(container.textContent).toContain('DEMO DRIVE — REAL MOTION, NOT AUTONOMY');
    expect(container.querySelector('select')).toBeNull();
    expect(container.querySelector('input[type="file"]')).toBeNull();
    expect(Array.from(container.querySelectorAll('button')).some((b) => /GATEWAY RUN|OPEN FILE/.test(b.textContent ?? ''))).toBe(false);
  });

  it('shows a disconnected/stale state instead of fabricated data when the gateway is unavailable', async () => {
    mockGatewayUnreachable();
    const container = await render();
    // Gateway WS never opens (test never calls .open()), so no live frame ever arrives.
    expect(container.textContent).toContain('NO LIVE CONNECTION');
    expect(container.textContent).toContain('DISCONNECTED FROM GATEWAY · NO LIVE FRAME DISPLAYED');
    // Nothing fabricated is shown as vehicle telemetry.
    expect(container.textContent).toContain('SPEED — m/s');
    expect(container.querySelector('.mc-world canvas')).toBeNull();
  });

  it('drops a live connection and shows disconnected rather than keeping stale data as current', async () => {
    mockGatewayUnreachable();
    const container = await render();
    const ws = MockWebSocket.instances[0];
    expect(ws).toBeDefined();
    await act(async () => {
      ws.open();
      ws.message({ type: 'manifest', manifest: { source: 'REAL_KART' } });
      ws.message({ type: 'frame', frame: realKartFrame() });
      await Promise.resolve();
    });
    expect(container.textContent).toContain('CURRENT ROS FRAME');
    expect(container.textContent).not.toContain('NO LIVE CONNECTION');

    await act(async () => {
      ws.close();
      await Promise.resolve();
    });
    // After disconnect, liveFrameFresh must drop and the view must say so --
    // never keep presenting the last frame as if it were still live.
    expect(container.textContent).toContain('DISCONNECTED');
  });

  it('never renders a REAL_KART-tagged frame payload that was actually tagged SIMULATED or LAB', async () => {
    mockGatewayUnreachable();
    const container = await render();
    const ws = MockWebSocket.instances[0];
    await act(async () => {
      ws.open();
      ws.message({ type: 'manifest', manifest: { source: 'SIMULATION' } });
      ws.message({ type: 'frame', frame: realKartFrame({ source: 'SIMULATION', vehicle: { x_m: 0, y_m: 0, yaw_rad: 0, speed_mps: 99 } }) });
      await Promise.resolve();
    });
    // A SIMULATION-tagged frame must be rejected by the live-only hook: no 99 m/s shows up.
    expect(container.textContent).not.toContain('99.00 m/s');
    expect(container.textContent).toContain('SOURCE MISMATCH');
  });
});
