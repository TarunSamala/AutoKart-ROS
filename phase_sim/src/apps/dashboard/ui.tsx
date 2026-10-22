import { useState, type ReactNode } from 'react';
import type { Detection, Tone } from './types';
import { NO_DATA } from './format';

export function Chip({ tone, children, title }: { tone: Tone; children: ReactNode; title?: string }) {
  return <span className={`chip tone-${tone}`} title={title}>{children}</span>;
}
export function Card({ title, children, wide, right, id }: { title: string; children: ReactNode; wide?: boolean; right?: ReactNode; id?: string }) {
  return <section className={`card${wide ? ' wide' : ''}`} aria-label={title} id={id}>
    <header><h2>{title}</h2>{right}</header>{children}</section>;
}
export function KV({ rows }: { rows: [string, ReactNode, Tone?][] }) {
  return <dl className="kv">{rows.map(([k, v, tone]) => <div key={k}><dt>{k}</dt><dd className={tone ? `tone-text-${tone}` : ''}>{v === '' || v == null ? NO_DATA : v}</dd></div>)}</dl>;
}
export function Unavailable({ what, reason }: { what: string; reason: string }) {
  return <p className="unavailable" role="status"><strong>{what}: UNAVAILABLE.</strong> {reason}</p>;
}

type Placed = { d: Detection; x: number; y: number; w: number; h: number; bx: number; by: number };
/** RGB/depth frame with detection boxes. Only a short "#id" badge is drawn on the image; classes and confidences are in a table, so labels never overlap. */
export function FrameView({ src, alt, detections, width = 640, height = 480, overlay }: { src: string | null; alt: string; detections?: Detection[]; width?: number; height?: number; overlay?: boolean }) {
  const [dims, setDims] = useState<[number, number]>([width, height]);
  const placed: Placed[] = [];
  if (overlay && detections) {
    const taken: [number, number, number, number][] = [];
    for (const d of detections) {
      if (!Array.isArray(d.box) || d.box.length !== 4 || !d.box.every(Number.isFinite)) continue;
      const [x1, y1, x2, y2] = d.box; let by = Math.max(0, y1), bx = Math.max(0, x1);
      for (let i = 0; i < 12; i++) { if (!taken.some(([a, b, c, e]) => bx < c && bx + 30 > a && by < e && by + 16 > b)) break; by += 17; }
      taken.push([bx, by, bx + 30, by + 16]); placed.push({ d, x: x1, y: y1, w: x2 - x1, h: y2 - y1, bx, by });
    }
  }
  return <div className="frame" style={{ aspectRatio: `${dims[0]} / ${dims[1]}` }}>
    {src ? <img src={src} alt={alt} onLoad={e => { const i = e.currentTarget; if (i.naturalWidth) setDims([i.naturalWidth, i.naturalHeight]); }} />
      : <div className="frame-empty"><strong>NO CURRENT FRAME</strong><span>{NO_DATA}</span></div>}
    {src && overlay && <svg viewBox={`0 0 ${dims[0]} ${dims[1]}`} preserveAspectRatio="none" aria-label="Detection boxes">
      {placed.map(p => <g key={p.d.id}>
        <rect x={p.x} y={p.y} width={p.w} height={p.h} className="box" />
        <rect x={p.bx} y={p.by} width="30" height="16" className="badge" /><text x={p.bx + 3} y={p.by + 12} className="badge-t">#{p.d.id}</text></g>)}
    </svg>}
  </div>;
}
