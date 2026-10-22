import { createContext, useContext, useEffect, useRef, useState, type ReactNode } from 'react';
import type { DashEvent, GatewayStatus, Pipeline, RosFrame, Snapshot } from './types';
import { wallClock } from './format';
import { deriveHealth } from './derive';

// All data access for the dashboard. Everything here is a GET (or a receive-only WebSocket):
// the dashboard has no code path that sends a command to the vehicle.

export type PercState = {
  link: 'connecting' | 'ok' | 'offline'; error: string;
  snapshot: Snapshot | null; displayed: Snapshot | null;
  receivedAt: number | null; lastFrameAt: number | null;
  frames: number; fps: number | null; rgb: string | null; depth: string | null;
  displayLatencyMs: number | null; clientSkipped: number; notice: string; okAt: number | null;
};
export type PipeState = { link: 'connecting' | 'ok' | 'unavailable' | 'offline'; data: Pipeline | null; error: string; at: number | null };
export type GwState = { link: 'connecting' | 'ok' | 'offline'; status: GatewayStatus | null; error: string; at: number | null };
export type RosState = { transport: 'CONNECTING' | 'CONNECTED' | 'DISCONNECTED' | 'UNSUPPORTED'; frame: RosFrame | null; at: number | null; frames: number };
export type Data = { tick: number; perc: PercState; pipe: PipeState; gw: GwState; ros: RosState; events: DashEvent[] };

const initial: Data = {
  tick: 0,
  perc: { link: 'connecting', error: '', snapshot: null, displayed: null, receivedAt: null, lastFrameAt: null, frames: 0, fps: null,
    rgb: null, depth: null, displayLatencyMs: null, clientSkipped: 0, notice: '', okAt: null },
  pipe: { link: 'connecting', data: null, error: '', at: null },
  gw: { link: 'connecting', status: null, error: '', at: null },
  ros: { transport: 'CONNECTING', frame: null, at: null, frames: 0 },
  events: [],
};

const Ctx = createContext<Data>(initial);
export const useDash = () => useContext(Ctx);

async function getJson<T>(url: string, timeoutMs: number): Promise<{ status: number; body: T | null }> {
  const ctl = new AbortController();
  const timer = window.setTimeout(() => ctl.abort(), timeoutMs);
  try {
    const r = await fetch(url, { cache: 'no-store', signal: ctl.signal });
    if (!r.ok) return { status: r.status, body: null };
    return { status: r.status, body: await r.json() as T };
  } finally { window.clearTimeout(timer); }
}

export function DashboardProvider({ children, wantImages }: { children: ReactNode; wantImages: boolean }) {
  const [data, setData] = useState<Data>(() => ({ ...initial, tick: performance.now() }));
  const wantImagesRef = useRef(wantImages);
  wantImagesRef.current = wantImages;
  const dataRef = useRef(data);
  dataRef.current = data;

  // 4 Hz clock so ages advance and STALE is detected even when no data arrives.
  useEffect(() => {
    const timer = window.setInterval(() => setData(d => ({ ...d, tick: performance.now() })), 250);
    return () => window.clearInterval(timer);
  }, []);

  // Perception snapshot + frame-matched images (single in-flight request, out-of-order rejection).
  useEffect(() => {
    let stopped = false; let timer: number;
    let acceptedTs = 0; let lastSeq: number | null = null; let lastSetAt = 0;
    const stamps: number[] = [];
    let displayedId: string | null = null;
    const retired: string[] = [];
    const revokeLater = (urls: (string | null)[]) => {
      const live = urls.filter((u): u is string => !!u);
      if (!live.length) return;
      retired.push(...live);
      window.setTimeout(() => { live.forEach(u => { URL.revokeObjectURL(u); const i = retired.indexOf(u); if (i >= 0) retired.splice(i, 1); }); }, 300);
    };
    const patch = (p: Partial<PercState>) => setData(d => ({ ...d, perc: { ...d.perc, ...p } }));
    async function poll() {
      if (stopped) return;
      const ctl = new AbortController();
      const timeout = window.setTimeout(() => ctl.abort(), 2000);
      try {
        const r = await fetch('/api/snapshot', { cache: 'no-store', signal: ctl.signal });
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        const snap = await r.json() as Snapshot;
        if (stopped) return;
        const now = performance.now();
        const cur = dataRef.current.perc;
        if (!snap.frame_id) {
          // Fault / no-frame snapshot: keep link ok, drop any previously displayed frame so nothing old looks current.
          revokeLater([cur.rgb, cur.depth]); displayedId = null;
          patch({ link: 'ok', error: '', snapshot: snap, displayed: null, rgb: null, depth: null, receivedAt: now, okAt: now, notice: '' });
        } else if (snap.frame_id === displayedId) {
          if (now - lastSetAt > 400) { lastSetAt = now; patch({ link: 'ok', error: '', snapshot: snap, receivedAt: now, okAt: now }); }
          else patch({ okAt: now });
        } else if (snap.capture_timestamp_ns != null && snap.capture_timestamp_ns <= acceptedTs) {
          patch({ notice: `Out-of-order frame ${snap.frame_id} ignored`, okAt: now, link: 'ok' });
        } else {
          let rgb: string | null = null, depth: string | null = null;
          if (wantImagesRef.current) {
            const [a, b] = await Promise.all([
              fetch(`/api/image/${snap.frame_id}/rgb`, { cache: 'no-store', signal: ctl.signal }),
              fetch(`/api/image/${snap.frame_id}/depth`, { cache: 'no-store', signal: ctl.signal })]);
            if (!a.ok || !b.ok) throw new Error('Matching frame images expired (frame-matched JPEG not available)');
            const [ab, bb] = await Promise.all([a.blob(), b.blob()]);
            if (stopped) return;
            rgb = URL.createObjectURL(ab); depth = URL.createObjectURL(bb);
            try {
              await Promise.all([rgb, depth].map(async u => { const im = new Image(); im.src = u; if (typeof im.decode === 'function') await im.decode(); }));
            } catch (e) { URL.revokeObjectURL(rgb); URL.revokeObjectURL(depth); throw e; }
            if (stopped) { URL.revokeObjectURL(rgb); URL.revokeObjectURL(depth); return; }
          }
          const done = performance.now();
          acceptedTs = snap.capture_timestamp_ns ?? acceptedTs;
          displayedId = snap.frame_id; lastSetAt = done;
          stamps.push(done); while (stamps.length && done - stamps[0] > 2000) stamps.shift();
          const fps = stamps.length > 1 ? (stamps.length - 1) / ((done - stamps[0]) / 1000) : null;
          const seq = snap.camera_sequence ?? null;
          const skipped = seq != null && lastSeq != null && seq > lastSeq + 1 ? seq - lastSeq - 1 : 0;
          lastSeq = seq ?? lastSeq;
          const latency = typeof snap.frame_age_ms === 'number' ? snap.frame_age_ms + (done - now) : null;
          revokeLater([cur.rgb, cur.depth]);
          setData(d => ({ ...d, perc: { ...d.perc, link: 'ok', error: '', snapshot: snap, displayed: snap, rgb, depth, receivedAt: now, okAt: done,
            lastFrameAt: done, frames: d.perc.frames + 1, fps, displayLatencyMs: latency, clientSkipped: d.perc.clientSkipped + skipped, notice: '' } }));
        }
      } catch (e) {
        if (!stopped) {
          const msg = e instanceof Error ? e.message : String(e);
          // Keep last snapshot for "last known" display, but the link is marked offline; derive.ts never shows it as current.
          patch({ link: 'offline', error: msg });
        }
      } finally { window.clearTimeout(timeout); if (!stopped) timer = window.setTimeout(poll, 40); }
    }
    void poll();
    return () => { stopped = true; window.clearTimeout(timer); retired.forEach(u => URL.revokeObjectURL(u)); const c = dataRef.current.perc; [c.rgb, c.depth].forEach(u => u && URL.revokeObjectURL(u)); };
  }, []);

  // Pipeline per-hop diagnostics (1.5 Hz). 404 = older service version without the route.
  useEffect(() => {
    let stopped = false; let timer: number;
    async function poll() {
      try {
        const { status, body } = await getJson<Pipeline>('/api/v1/diagnostics/pipeline', 1000);
        if (stopped) return;
        const now = performance.now();
        if (status === 404) setData(d => ({ ...d, pipe: { link: 'unavailable', data: null, error: 'HTTP 404', at: now } }));
        else if (!body) setData(d => ({ ...d, pipe: { link: 'offline', data: null, error: `HTTP ${status}`, at: now } }));
        else setData(d => ({ ...d, pipe: { link: 'ok', data: body, error: '', at: now } }));
      } catch (e) {
        if (!stopped) setData(d => ({ ...d, pipe: { link: 'offline', data: null, error: e instanceof Error ? e.message : String(e), at: performance.now() } }));
      } finally { if (!stopped) timer = window.setTimeout(poll, 1500); }
    }
    void poll();
    return () => { stopped = true; window.clearTimeout(timer); };
  }, []);

  // ROS gateway status (2 Hz).
  useEffect(() => {
    let stopped = false; let timer: number;
    async function poll() {
      try {
        const { status, body } = await getJson<GatewayStatus>('/mission/api/status', 1500);
        if (stopped) return;
        if (!body) throw new Error(`HTTP ${status}`);
        setData(d => ({ ...d, gw: { link: 'ok', status: body, error: '', at: performance.now() } }));
      } catch (e) {
        if (!stopped) setData(d => ({ ...d, gw: { link: 'offline', status: null, error: e instanceof Error ? e.message : String(e), at: performance.now() } }));
      } finally { if (!stopped) timer = window.setTimeout(poll, 500); }
    }
    void poll();
    return () => { stopped = true; window.clearTimeout(timer); };
  }, []);

  // Receive-only ROS WebSocket: only REAL_KART-tagged frames are accepted.
  useEffect(() => {
    if (typeof WebSocket === 'undefined') { setData(d => ({ ...d, ros: { ...d.ros, transport: 'UNSUPPORTED' } })); return; }
    let cancelled = false; let retry: number | undefined; let ws: WebSocket | null = null;
    const connect = () => {
      try {
        const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
        ws = new WebSocket(`${proto}//${location.host}/mission/ws`);
      } catch { setData(d => ({ ...d, ros: { ...d.ros, transport: 'DISCONNECTED' } })); retry = window.setTimeout(connect, 2000); return; }
      ws.onopen = () => setData(d => ({ ...d, ros: { ...d.ros, transport: 'CONNECTED' } }));
      ws.onmessage = ev => {
        try {
          const m = JSON.parse(String(ev.data));
          if (m?.type !== 'frame' || m.frame?.source !== 'REAL_KART') return;
          setData(d => ({ ...d, ros: { ...d.ros, frame: m.frame as RosFrame, at: performance.now(), frames: d.ros.frames + 1 } }));
        } catch { /* malformed frame ignored */ }
      };
      ws.onclose = () => { if (cancelled) return; setData(d => ({ ...d, ros: { ...d.ros, transport: 'DISCONNECTED' } })); retry = window.setTimeout(connect, 1500); };
      ws.onerror = () => ws?.close();
    };
    connect();
    return () => { cancelled = true; window.clearTimeout(retry); ws?.close(); };
  }, []);

  // Event log: state transitions observed by THIS browser session (not a vehicle log).
  const lastStates = useRef<Record<string, string>>({});
  useEffect(() => {
    const health = deriveHealth(data);
    const next: DashEvent[] = [];
    const wall = wallClock();
    for (const chip of health.chips) {
      const prev = lastStates.current[chip.key];
      if (prev !== chip.state) {
        if (prev !== undefined) next.push({ t: data.tick, wall, hop: chip.label, from: prev, to: chip.state, detail: chip.detail });
        lastStates.current[chip.key] = chip.state;
      }
    }
    if (next.length) setData(d => ({ ...d, events: [...next.reverse(), ...d.events].slice(0, 300) }));
  }, [data.tick]); // eslint-disable-line react-hooks/exhaustive-deps

  return <Ctx.Provider value={data}>{children}</Ctx.Provider>;
}
