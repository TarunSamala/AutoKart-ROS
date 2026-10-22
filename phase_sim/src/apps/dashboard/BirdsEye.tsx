import { useEffect, useMemo, useRef, useState } from 'react';
import type { Pt, Snapshot } from './types';

// Animated bird's-eye view (base_link: x forward, y left). A requestAnimationFrame loop eases cone, kart-frame
// and path positions toward each new snapshot and fades edges in/out, so motion is evidence of new data.
// When `frozen` (stale / offline) the loop stops advancing time: nothing moves and STALE is shown.

export type LayerKey = 'cones' | 'candidates' | 'delaunay' | 'gabriel' | 'gabrielRejected' | 'boundaries' | 'gates' | 'centerline' | 'path' | 'target';
export const LAYER_META: Record<LayerKey, { label: string; color: string; dash?: boolean }> = {
  cones: { label: 'Detections → metric cones', color: '#4f9dff' },
  candidates: { label: 'Unverified candidates', color: '#ffd284', dash: true },
  delaunay: { label: 'Delaunay edges', color: '#6f7f93' },
  gabriel: { label: 'Gabriel accepted', color: '#41d3a2' },
  gabrielRejected: { label: 'Gabriel rejected', color: '#e0566a', dash: true },
  boundaries: { label: 'Boundaries L / R', color: '#b88cff' },
  gates: { label: 'Gates', color: '#ffb454' },
  centerline: { label: 'Centerline', color: '#f4f7fa' },
  path: { label: 'Planned path', color: '#3ddc84' },
  target: { label: 'Control target', color: '#ff5fa2' },
};
export const DEFAULT_LAYERS = Object.fromEntries(Object.keys(LAYER_META).map(k => [k, true])) as Record<LayerKey, boolean>;
const CONE_COLOR: Record<string, string> = { blue: '#4f9dff', yellow: '#f5d04c', orange: '#ff9442', large_orange: '#ff6a3d', unknown: '#a8b0ba' };

export type SceneCone = { key: string; x: number; y: number; label: string; text: string; kind: 'metric' | 'candidate' | 'provisional'; r: number };
export type SceneEdge = { key: string; a: string; b: string; layer: 'delaunay' | 'gabriel' | 'gabrielRejected' | 'gate'; text?: string };
export type SceneLine = { key: string; layer: 'boundaries' | 'centerline' | 'path'; side?: 'L' | 'R'; pts: Pt[] };
export type Scene = {
  mode: 'QUALIFIED' | 'SIMULATED' | 'NONE'; frameId: string | null;
  cones: SceneCone[]; edges: SceneEdge[]; lines: SceneLine[]; target: Pt | null; note: string;
};
const fin = (p: unknown): p is Pt => Array.isArray(p) && Number.isFinite(p[0]) && Number.isFinite(p[1]);

export function buildScene(s: Snapshot | null, mode: Scene['mode'], why: string): Scene {
  const scene: Scene = { mode, frameId: s?.frame_id ?? null, cones: [], edges: [], lines: [], target: null, note: why };
  if (!s) return scene;
  if (mode === 'NONE') {
    (s.candidate_observations ?? []).forEach(c => fin(c.xy) && scene.cones.push({ key: `c${c.id}`, x: c.xy[0], y: c.xy[1], label: c.label, kind: 'candidate', r: 9,
      text: `#${c.id} ${c.label} unverified ground candidate ${c.xy[0].toFixed(2)}, ${c.xy[1].toFixed(2)} m` }));
    (s.provisional_3d_observations ?? []).forEach(c => fin(c.nominal_xy_m) && scene.cones.push({ key: `p${c.id}`, x: c.nominal_xy_m[0], y: c.nominal_xy_m[1], label: c.label, kind: 'provisional',
      r: Math.min(60, Math.max(8, (c.uncertainty_m ?? 0.2) * 48)), text: `#${c.id} provisional body ${c.nominal_xy_m[0].toFixed(2)}, ${c.nominal_xy_m[1].toFixed(2)} m` }));
    return scene;
  }
  const g = s.geometry ?? {};
  const obs = (s.observations ?? []).filter(c => fin(c.xy));
  obs.forEach(c => scene.cones.push({ key: `o${c.id}`, x: c.xy[0], y: c.xy[1], label: c.label, kind: 'metric', r: c.label === 'large_orange' ? 11 : 9,
    text: `#${c.id} ${c.label} ${(100 * c.confidence).toFixed(0)}%` }));
  const ids = new Set(obs.map(c => c.id));
  const gabriel = new Set((g.gabriel_edges ?? []).map(([a, b]) => `${Math.min(a, b)}-${Math.max(a, b)}`));
  (g.delaunay_edges ?? []).forEach(([a, b]) => {
    if (!ids.has(a) || !ids.has(b)) return;
    const k = `${Math.min(a, b)}-${Math.max(a, b)}`;
    scene.edges.push({ key: `d${k}`, a: `o${a}`, b: `o${b}`, layer: 'delaunay' });
    if (!gabriel.has(k)) scene.edges.push({ key: `r${k}`, a: `o${a}`, b: `o${b}`, layer: 'gabrielRejected' });
  });
  (g.gabriel_edges ?? []).forEach(([a, b]) => ids.has(a) && ids.has(b) && scene.edges.push({ key: `g${Math.min(a, b)}-${Math.max(a, b)}`, a: `o${a}`, b: `o${b}`, layer: 'gabriel' }));
  (g.gates ?? []).forEach((gt, i) => ids.has(gt.left_id) && ids.has(gt.right_id) && scene.edges.push({ key: `gate${gt.id ?? i}`, a: `o${gt.left_id}`, b: `o${gt.right_id}`, layer: 'gate',
    text: `G${gt.id ?? i}${typeof gt.width === 'number' ? ` ${gt.width.toFixed(2)} m` : ''}` }));
  const byId = new Map(obs.map(c => [c.id, c.xy] as const));
  const side = (list: number[] | undefined) => (list ?? []).map(id => byId.get(id)).filter((p): p is Pt => !!p);
  const L = side(g.boundaries?.left_ids), R = side(g.boundaries?.right_ids);
  if (L.length > 1) scene.lines.push({ key: 'bl', layer: 'boundaries', side: 'L', pts: L });
  if (R.length > 1) scene.lines.push({ key: 'br', layer: 'boundaries', side: 'R', pts: R });
  const cl = (g.centerline ?? []).filter(fin);
  if (cl.length > 1) scene.lines.push({ key: 'cl', layer: 'centerline', pts: cl });
  const path = (s.planning?.local_path ?? []).filter(fin);
  if (s.planning?.path_valid && path.length > 1) scene.lines.push({ key: 'path', layer: 'path', pts: path });
  const t = s.control_candidate?.target_point_base_m;
  if (s.control_candidate?.valid && fin(t)) scene.target = t;
  return scene;
}

type AEnt = { x: number; y: number; tx: number; ty: number; a: number; ta: number };
type ACone = AEnt & SceneCone;
type AEdge = { e: SceneEdge; a: number; ta: number };
type ALine = { l: SceneLine; pts: Pt[]; tpts: Pt[]; a: number; ta: number };
const W = 640, H = 480, SCALE = 48, OX = 320, OY = 440;
const px = (x: number, y: number): Pt => [OX - y * SCALE, OY - x * SCALE];

export function BirdsEye({ scene, layers, frozen, frameCounter, statusText, dataFps, compact }: {
  scene: Scene; layers: Record<LayerKey, boolean>; frozen: boolean; frameCounter: number; statusText: string; dataFps: number | null; compact?: boolean;
}) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const anim = useRef({ cones: new Map<string, ACone>(), edges: new Map<string, AEdge>(), lines: new Map<string, ALine>(),
    target: null as (AEnt | null), mode: scene.mode });
  const live = useRef({ layers, frozen, scene, statusText });
  live.current = { layers, frozen, scene, statusText };
  const [renderFps, setRenderFps] = useState<number | null>(null);
  const [advancing, setAdvancing] = useState(false);

  // Push each new snapshot into animation targets (positions ease; appearance/disappearance fades).
  useEffect(() => {
    const A = anim.current;
    if (A.mode !== scene.mode) { A.cones.clear(); A.edges.clear(); A.lines.clear(); A.target = null; A.mode = scene.mode; }
    const seen = new Set<string>();
    scene.cones.forEach(c => {
      seen.add(c.key); const cur = A.cones.get(c.key);
      if (cur) Object.assign(cur, c, { tx: c.x, ty: c.y, ta: 1, x: cur.x, y: cur.y });
      else A.cones.set(c.key, { ...c, tx: c.x, ty: c.y, a: 0, ta: 1 });
    });
    A.cones.forEach((c, k) => { if (!seen.has(k)) c.ta = 0; });
    const eseen = new Set<string>();
    scene.edges.forEach(e => { eseen.add(e.key); const cur = A.edges.get(e.key); if (cur) { cur.e = e; cur.ta = 1; } else A.edges.set(e.key, { e, a: 0, ta: 1 }); });
    A.edges.forEach((e, k) => { if (!eseen.has(k)) e.ta = 0; });
    const lseen = new Set<string>();
    scene.lines.forEach(l => {
      lseen.add(l.key); const cur = A.lines.get(l.key);
      if (cur && cur.pts.length === l.pts.length) { cur.l = l; cur.tpts = l.pts; cur.ta = 1; }
      else A.lines.set(l.key, { l, pts: l.pts, tpts: l.pts, a: 0, ta: 1 });
    });
    A.lines.forEach((l, k) => { if (!lseen.has(k)) l.ta = 0; });
    if (scene.target) { if (A.target) { A.target.tx = scene.target[0]; A.target.ty = scene.target[1]; A.target.ta = 1; } else A.target = { x: scene.target[0], y: scene.target[1], tx: scene.target[0], ty: scene.target[1], a: 0, ta: 1 }; }
    else if (A.target) A.target.ta = 0;
  }, [scene]);

  useEffect(() => {
    const el = canvas.current; if (!el) return;
    let ctx: CanvasRenderingContext2D | null = null;
    try { ctx = el.getContext('2d'); } catch { ctx = null; }
    if (!ctx || typeof requestAnimationFrame !== 'function') return;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    el.width = W * dpr; el.height = H * dpr;
    const c2 = ctx; c2.setTransform(dpr, 0, 0, dpr, 0, 0);
    let raf = 0, last = performance.now(), frames = 0, fpsStart = last, moving = 0;
    const step = (t: number) => {
      const L = live.current, A = anim.current;
      const dt = L.frozen ? 0 : Math.min(0.1, (t - last) / 1000); last = t;
      const kp = 1 - Math.exp(-dt / 0.09), kf = 1 - Math.exp(-dt / 0.18);
      let moved = false;
      A.cones.forEach((c, k) => {
        const dx = c.tx - c.x, dy = c.ty - c.y; if (Math.abs(dx) + Math.abs(dy) > 1e-4) moved = true;
        c.x += dx * kp; c.y += dy * kp; c.a += (c.ta - c.a) * kf; if (c.ta === 0 && c.a < 0.02) A.cones.delete(k);
      });
      A.edges.forEach((e, k) => { e.a += (e.ta - e.a) * kf; if (e.ta === 0 && e.a < 0.02) A.edges.delete(k); });
      A.lines.forEach((l, k) => {
        l.pts = l.pts.length === l.tpts.length ? l.pts.map((p, i) => { const q = l.tpts[i]; if (Math.abs(q[0] - p[0]) + Math.abs(q[1] - p[1]) > 1e-4) moved = true; return [p[0] + (q[0] - p[0]) * kp, p[1] + (q[1] - p[1]) * kp] as Pt; }) : l.tpts;
        l.a += (l.ta - l.a) * kf; if (l.ta === 0 && l.a < 0.02) A.lines.delete(k);
      });
      if (A.target) { A.target.x += (A.target.tx - A.target.x) * kp; A.target.y += (A.target.ty - A.target.y) * kp; A.target.a += (A.target.ta - A.target.a) * kf; if (A.target.ta === 0 && A.target.a < 0.02) A.target = null; }
      draw(c2, A, L.layers, L.scene, L.frozen, L.statusText);
      frames++; if (moved) moving++;
      if (t - fpsStart >= 1000) { setRenderFps(Math.round(frames * 1000 / (t - fpsStart))); setAdvancing(moving > 0 && !L.frozen); frames = 0; moving = 0; fpsStart = t; }
      raf = requestAnimationFrame(step);
    };
    raf = requestAnimationFrame(step);
    return () => cancelAnimationFrame(raf);
  }, []);

  const counts = useMemo(() => ({ cones: scene.cones.length, edges: scene.edges.length }), [scene]);
  const label = scene.mode === 'NONE' ? "Bird's-eye diagnostic grid; no qualified live geometry"
    : scene.mode === 'SIMULATED' ? "Bird's-eye SIMULATED geometry, not a physical measurement" : "Bird's-eye qualified metric geometry";
  return <figure className={`bev${compact ? ' bev-compact' : ''}`} data-testid="birdseye" data-mode={scene.mode} data-frame={scene.frameId ?? ''}
    data-animating={frozen ? 'false' : 'true'} data-cones={counts.cones} data-edges={counts.edges}>
    <div className="bev-bar" aria-live="off">
      <span className="mono">FRAME #{frameCounter}</span>
      <span className="mono">DATA {dataFps != null ? `${dataFps.toFixed(1)} Hz` : '— (NO DATA)'}</span>
      <span className="mono">RENDER {renderFps != null ? `${renderFps} fps` : '—'}</span>
      <span className={`chip tone-${frozen ? 'alert' : 'ok'}`}>{frozen ? `FROZEN · ${statusText}` : advancing ? 'ANIMATING' : 'LIVE · settled'}</span>
    </div>
    <canvas ref={canvas} width={W} height={H} role="img" aria-label={label} />
    <figcaption className="sr-only">{label}. {counts.cones} markers, {counts.edges} edges. {scene.note}</figcaption>
  </figure>;
}

function draw(c: CanvasRenderingContext2D, A: { cones: Map<string, ACone>; edges: Map<string, AEdge>; lines: Map<string, ALine>; target: AEnt | null },
  layers: Record<LayerKey, boolean>, scene: Scene, frozen: boolean, status: string) {
  c.clearRect(0, 0, W, H); c.fillStyle = '#071320'; c.fillRect(0, 0, W, H);
  c.font = '11px ui-monospace, monospace'; c.lineWidth = 1;
  for (let y = -6; y <= 6; y++) { const [gx] = px(0, y); c.strokeStyle = y === 0 ? '#2a4a66' : '#12283a'; c.beginPath(); c.moveTo(gx, 0); c.lineTo(gx, H); c.stroke(); }
  for (let x = 0; x <= 9; x++) { const [, gy] = px(x, 0); c.strokeStyle = x === 0 ? '#2a4a66' : '#12283a'; c.beginPath(); c.moveTo(0, gy); c.lineTo(W, gy); c.stroke(); c.fillStyle = '#58718a'; c.fillText(`${x} m`, 6, gy - 3); }
  c.strokeStyle = '#1b3a52'; [2, 4, 6, 8].forEach(r => { c.beginPath(); c.arc(OX, OY, r * SCALE, Math.PI, 2 * Math.PI); c.stroke(); });
  const dashed = (on: boolean) => c.setLineDash(on ? [5, 4] : []);
  const edgeLayer = (k: SceneEdge['layer']): LayerKey => (k === 'gate' ? 'gates' : k);
  A.edges.forEach(({ e, a }) => {
    if (!layers[edgeLayer(e.layer)]) return;
    const ca = A.cones.get(e.a), cb = A.cones.get(e.b); if (!ca || !cb) return;
    const [x1, y1] = px(ca.x, ca.y), [x2, y2] = px(cb.x, cb.y);
    const meta = LAYER_META[edgeLayer(e.layer)];
    c.globalAlpha = Math.min(a, ca.a, cb.a) * (e.layer === 'delaunay' ? 0.55 : 0.95); c.strokeStyle = meta.color; c.lineWidth = e.layer === 'gate' ? 2.5 : 1.6; dashed(!!meta.dash);
    c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke();
    if (e.layer === 'gate' && e.text) { c.setLineDash([]); c.fillStyle = meta.color; c.fillText(e.text, (x1 + x2) / 2 + 6, (y1 + y2) / 2 - 6); }
  });
  c.setLineDash([]); c.lineWidth = 2.4;
  A.lines.forEach(({ l, pts, a }) => {
    if (!layers[l.layer] || pts.length < 2) return;
    c.globalAlpha = a; c.strokeStyle = l.layer === 'boundaries' ? (l.side === 'L' ? '#4f9dff' : '#f5d04c') : LAYER_META[l.layer].color;
    c.lineWidth = l.layer === 'path' ? 3.5 : 2.2; dashed(l.layer === 'centerline');
    c.beginPath(); pts.forEach((p, i) => { const [x, y] = px(p[0], p[1]); i ? c.lineTo(x, y) : c.moveTo(x, y); }); c.stroke();
  });
  c.setLineDash([]); c.lineWidth = 2;
  A.cones.forEach(cn => {
    const [x, y] = px(cn.x, cn.y);
    if (cn.kind === 'metric') {
      if (!layers.cones) return; c.globalAlpha = cn.a; c.fillStyle = CONE_COLOR[cn.label] ?? CONE_COLOR.unknown; c.strokeStyle = '#04101a';
      c.beginPath(); c.arc(x, y, cn.r, 0, 6.283); c.fill(); c.stroke(); c.fillStyle = '#cfe0ee'; c.fillText(cn.text.split(' ')[0], x + cn.r + 3, y - cn.r);
    } else {
      if (!layers.candidates) return; c.globalAlpha = cn.a; c.strokeStyle = '#ffd284'; dashed(true); c.beginPath();
      if (cn.kind === 'candidate') c.arc(x, y, cn.r, 0, 6.283); else c.arc(x, y, cn.r, 0, 6.283); c.stroke(); dashed(false);
      c.fillStyle = '#e9a548'; c.fillRect(x - 3, y - 3, 6, 6);
    }
  });
  if (A.target && layers.target) {
    const [x, y] = px(A.target.x, A.target.y), [ox, oy] = px(0, 0); c.globalAlpha = A.target.a; c.strokeStyle = LAYER_META.target.color;
    dashed(true); c.beginPath(); c.moveTo(ox, oy); c.lineTo(x, y); c.stroke(); dashed(false);
    c.beginPath(); c.arc(x, y, 10, 0, 6.283); c.stroke(); c.beginPath(); c.arc(x, y, 3, 0, 6.283); c.fillStyle = LAYER_META.target.color; c.fill();
  }
  c.globalAlpha = 1; c.fillStyle = '#e8eef5'; c.beginPath(); c.moveTo(OX, OY - 22); c.lineTo(OX - 11, OY + 4); c.lineTo(OX + 11, OY + 4); c.closePath(); c.fill();
  c.fillStyle = '#8fa4bb'; c.fillText(scene.mode === 'QUALIFIED' ? 'base_link origin' : scene.mode === 'SIMULATED' ? 'simulated base_link origin' : 'camera-ground nominal origin', OX + 16, OY + 4);
  c.strokeStyle = '#8fa4bb'; c.beginPath(); c.moveTo(560, 462); c.lineTo(560 + SCALE, 462); c.stroke(); c.fillText('1 m', 560 + SCALE / 2 - 8, 456);
  if (scene.mode === 'NONE') {
    c.fillStyle = 'rgba(22,43,59,0.93)'; c.strokeStyle = '#a67c35'; c.lineWidth = 1.5; c.fillRect(70, 24, 500, 74); c.strokeRect(70, 24, 500, 74);
    c.textAlign = 'center'; c.fillStyle = '#ffd284'; c.font = 'bold 18px system-ui, sans-serif'; c.fillText('NO QUALIFIED LIVE GEOMETRY', OX, 52);
    c.font = '12px system-ui, sans-serif'; c.fillStyle = '#c9d8e5'; c.fillText(scene.note.slice(0, 78) || 'depth scale / camera mount not verified', OX, 74);
    c.fillStyle = '#9fb4c6'; c.fillText(`dashed markers = ${scene.cones.length} unverified candidates (diagnostic only, not metric)`, OX, 90); c.textAlign = 'left';
  } else if (scene.mode === 'SIMULATED') {
    c.fillStyle = 'rgba(120,20,30,0.9)'; c.fillRect(0, 0, W, 22); c.fillStyle = '#fff'; c.font = 'bold 12px system-ui, sans-serif'; c.textAlign = 'center';
    c.fillText('SIMULATED GEOMETRY — NOT A PHYSICAL MEASUREMENT', OX, 15); c.textAlign = 'left';
  }
  if (frozen) {
    c.fillStyle = 'rgba(7,19,32,0.55)'; c.fillRect(0, 0, W, H); c.fillStyle = '#ffb454'; c.font = 'bold 30px system-ui, sans-serif'; c.textAlign = 'center';
    c.fillText(status.slice(0, 40), OX, 240); c.font = '13px system-ui, sans-serif'; c.fillText('animation frozen · last data shown is NOT current', OX, 264); c.textAlign = 'left';
  }
  c.font = '11px ui-monospace, monospace';
}
