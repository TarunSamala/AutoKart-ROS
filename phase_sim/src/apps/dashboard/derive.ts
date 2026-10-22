import type { Data } from './data';
import type { Envelope, Tone } from './types';
import { isNum } from './format';

// Pure derivation of truthful health states. Nothing here turns "HTTP 200" into green: LIVE needs
// a fresh frame, and CLEAR needs a CURRENT, valid, verified, LIVE-sourced safety envelope.

export const STALE_AFTER_MS = 1500;
export type Chip = { key: string; label: string; state: string; tone: Tone; detail: string };

export const toneOf = (state: string): Tone => ({
  LIVE: 'ok', CLEAR: 'ok', CONNECTED: 'ok', CURRENT: 'ok', OK: 'ok',
  REACHABLE: 'info', CALIBRATION_UNVERIFIED: 'warn', PARTIAL: 'warn', DEGRADED: 'warn', BRAKING: 'warn',
  STARTING: 'idle', NOT_APPLICABLE: 'idle', CONNECTING: 'idle', UNSUPPORTED: 'idle',
  STALE: 'alert', UNKNOWN: 'alert',
  CRITICAL: 'fault', NO_FRAME: 'fault', SDK_ERROR: 'fault', ROS_OFFLINE: 'fault', GATEWAY_OFFLINE: 'fault',
  PERCEPTION_OFFLINE: 'fault', DISCONNECTED: 'fault', MISSING: 'fault',
} as Record<string, Tone>)[state] ?? 'warn';

export type CameraView = { state: string; detail: string; fresh: boolean; ageMs: number | null };

export function cameraView(d: Data): CameraView {
  const p = d.perc, s = p.snapshot;
  if (p.link === 'offline') return { state: 'PERCEPTION_OFFLINE', detail: `perception service not reachable (${p.error || 'no response'}); camera state unknown`, fresh: false, ageMs: null };
  if (!s) return { state: 'CONNECTING', detail: 'waiting for first /api/snapshot', fresh: false, ageMs: null };
  const reasons = s.reasons ?? [];
  const sdk = d.pipe.data?.hops?.find(h => h.name === 'sdk_capture');
  if (!s.frame_id) {
    if (sdk?.state === 'SDK_ERROR' || (s.status === 'SENSOR_FAULT' && reasons.some(r => r.includes('Camera'))))
      return { state: 'SDK_ERROR', detail: `camera SDK fault: ${sdk?.last_error ?? reasons.join(', ')}`, fresh: false, ageMs: null };
    if (s.status === 'STALE' || sdk?.state === 'STALE') return { state: 'STALE', detail: 'no current frame (stale)', fresh: false, ageMs: s.last_good_age_ms ?? null };
    return { state: 'NO_FRAME', detail: `no camera frame (${s.status ?? 'status unknown'}${reasons.length ? ': ' + reasons.join(', ') : ''})`, fresh: false, ageMs: null };
  }
  const sinceFrame = p.lastFrameAt != null ? d.tick - p.lastFrameAt : null;
  const serverAge = isNum(s.frame_age_ms) && p.receivedAt != null ? s.frame_age_ms + Math.max(0, d.tick - p.receivedAt) : null;
  const ageMs = serverAge ?? sinceFrame;
  const stale = s.status === 'STALE' || (isNum(s.frame_age_ms) && s.frame_age_ms > 1000) || (sinceFrame != null && sinceFrame > STALE_AFTER_MS);
  if (stale) return { state: 'STALE', detail: `last frame ${s.frame_id} is ${((ageMs ?? 0) / 1000).toFixed(1)} s old`, fresh: false, ageMs };
  if (reasons.includes('CALIBRATION_UNVERIFIED') || (s.calibration_status && s.calibration_status !== 'MOUNT_VERIFIED') || s.depth_scale_qualified === false)
    return { state: 'CALIBRATION_UNVERIFIED', detail: `frames current; ${s.calibration_status ?? 'calibration unknown'}, depth scale ${s.depth_scale_qualified ? 'qualified' : 'unqualified'}`, fresh: true, ageMs };
  if (s.status && s.status !== 'VALID') return { state: 'DEGRADED', detail: `perception status ${s.status}`, fresh: true, ageMs };
  return { state: 'LIVE', detail: 'current frames, calibration verified', fresh: true, ageMs };
}

export function geometryQualified(d: Data): { mode: 'QUALIFIED' | 'SIMULATED' | 'NONE'; why: string } {
  const s = d.perc.snapshot;
  if (!s) return { mode: 'NONE', why: 'no snapshot' };
  if (s.source === 'SIMULATED' && s.base_frame === 'base_link') return { mode: 'SIMULATED', why: 'SIMULATED backend: geometry is synthetic, not a physical measurement' };
  if (s.source !== 'SIMULATED' && s.depth_scale_qualified === true && s.calibration_status === 'MOUNT_VERIFIED') return { mode: 'QUALIFIED', why: 'depth scale and mount verified' };
  const why = [s.calibration_status !== 'MOUNT_VERIFIED' ? `mount ${s.calibration_status ?? 'unknown'}` : '',
    s.depth_scale_qualified !== true ? 'depth scale unqualified' : ''].filter(Boolean).join(', ');
  return { mode: 'NONE', why };
}

export type SafetyView = { state: string; label: string; tone: Tone; reason: string; channel: string; ageMs: number | null; notes: string[]; envelope: Envelope | null };

export function safetyView(d: Data): SafetyView {
  const notes: string[] = [];
  if (d.gw.link === 'connecting') return { state: 'UNKNOWN', label: 'UNKNOWN', tone: 'idle', reason: 'CONNECTING to ROS gateway', channel: 'CONNECTING', ageMs: null, notes, envelope: null };
  if (d.gw.link === 'offline' || !d.gw.status) return { state: 'UNKNOWN', label: 'UNKNOWN', tone: 'fault', reason: 'GATEWAY_OFFLINE: safety envelope cannot be read', channel: 'GATEWAY_OFFLINE', ageMs: null, notes, envelope: null };
  const ch = d.gw.status.safety_envelope;
  if (!ch || ch.state !== 'CURRENT' || !ch.payload) {
    return { state: 'UNKNOWN', label: 'UNKNOWN', tone: 'alert', reason: `/safety/envelope ${ch?.state ?? 'MISSING'}: not publishing; never shown as CLEAR`, channel: ch?.state ?? 'MISSING', ageMs: ch?.age_ms ?? null, notes, envelope: null };
  }
  const e = ch.payload;
  let state = ['CLEAR', 'BRAKING', 'CRITICAL', 'UNKNOWN'].includes(e.state) ? e.state : 'UNKNOWN';
  if (e.valid === false && state === 'CLEAR') { state = 'UNKNOWN'; notes.push('Envelope marked invalid: CLEAR suppressed.'); }
  if (e.data_source !== 'LIVE') notes.push(`DATA SOURCE ${e.data_source ?? 'UNVERIFIED'}: NOT A PHYSICAL SENSOR READING`);
  if (e.calibration_verified !== true) notes.push('Calibration unverified: distances are schematic, not hardware-validated.');
  let label = state; let tone = toneOf(state);
  if (state === 'CLEAR' && (e.data_source !== 'LIVE' || e.calibration_verified !== true)) { label = 'CLEAR · UNVERIFIED'; tone = 'warn'; }
  return { state, label, tone, reason: e.reason ?? '', channel: 'CURRENT', ageMs: ch.age_ms, notes, envelope: e };
}

export function rosView(d: Data): { state: string; detail: string } {
  if (d.gw.link === 'connecting') return { state: 'CONNECTING', detail: 'waiting for /mission/api/status' };
  if (d.gw.link === 'offline' || !d.gw.status) return { state: 'GATEWAY_OFFLINE', detail: `ROS gateway not reachable (${d.gw.error || 'no response'}); ROS state unknown` };
  const g = d.gw.status;
  if (!g.ros_connected) return { state: 'ROS_OFFLINE', detail: 'gateway reachable but no ROS topic is current (odometry, vcu, perception, mapping, planning)' };
  if (g.odometry?.current) return { state: 'LIVE', detail: 'odometry current' };
  return { state: 'PARTIAL', detail: 'some ROS topics current; odometry not current' };
}

export function sourceBanners(d: Data): string[] {
  const out: string[] = [];
  const src = d.perc.snapshot?.source ?? d.pipe.data?.service?.source;
  if (src && src !== 'LIVE') out.push(`BACKEND SOURCE: ${src} — NOT A PHYSICAL CAMERA`);
  const g = d.gw.status;
  if (g && ((g.source && g.source !== 'REAL_KART') || (g.mode && g.mode !== 'LIVE')))
    out.push(`ROS GATEWAY SOURCE: ${g.source ?? 'UNKNOWN'} / ${g.mode ?? 'UNKNOWN'} — NOT THE REAL KART`);
  return out;
}

export function deriveHealth(d: Data) {
  const cam = cameraView(d), geo = geometryQualified(d), ros = rosView(d), safety = safetyView(d);
  const perc = d.perc.link === 'connecting' ? 'CONNECTING' : d.perc.link === 'offline' ? 'PERCEPTION_OFFLINE' : 'REACHABLE';
  const gw = d.gw.link === 'connecting' ? 'CONNECTING' : d.gw.link === 'offline' ? 'GATEWAY_OFFLINE' : 'REACHABLE';
  const geoState = !cam.fresh ? (['CONNECTING', 'STALE', 'PERCEPTION_OFFLINE'].includes(cam.state) ? cam.state : 'NO_FRAME')
    : geo.mode === 'QUALIFIED' ? 'LIVE' : 'CALIBRATION_UNVERIFIED';
  const safetyChip = safety.state === 'CLEAR' && safety.tone === 'warn' ? 'CLEAR (UNVERIFIED)' : safety.state;
  const chips: Chip[] = [
    { key: 'perception', label: 'PERCEPTION SERVICE', state: perc, tone: toneOf(perc), detail: d.perc.link === 'offline' ? d.perc.error : 'HTTP reachable (not a health claim)' },
    { key: 'camera', label: 'CAMERA', state: cam.state, tone: toneOf(cam.state), detail: cam.detail },
    { key: 'geometry', label: 'METRIC GEOMETRY', state: geoState, tone: toneOf(geoState), detail: geo.mode === 'NONE' ? `NO QUALIFIED LIVE GEOMETRY: ${geo.why}` : geo.why },
    { key: 'gateway', label: 'ROS GATEWAY', state: gw, tone: toneOf(gw), detail: d.gw.link === 'offline' ? d.gw.error : 'HTTP reachable (not a health claim)' },
    { key: 'ros', label: 'ROS DATA', state: ros.state, tone: toneOf(ros.state), detail: ros.detail },
    { key: 'safety', label: 'SAFETY ENVELOPE', state: safetyChip, tone: safety.tone, detail: safety.reason },
  ];
  return { chips, cam, geo, ros, safety, banners: sourceBanners(d), animating: cam.fresh };
}
