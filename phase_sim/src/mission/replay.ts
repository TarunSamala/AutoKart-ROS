import type { MissionFrame } from './schema';

const MAX_FRAMES = 6000;

export function validFrame(value: unknown): value is MissionFrame {
  if (!value || typeof value !== 'object') return false;
  const frame = value as Partial<MissionFrame>;
  return frame.schema_version === 1 && Number.isFinite(frame.timestamp_s) && !!frame.vehicle &&
    Number.isFinite(frame.vehicle.x_m) && Number.isFinite(frame.vehicle.y_m) &&
    Array.isArray(frame.perception?.cones) && Array.isArray(frame.planning?.delaunay_edges);
}

export function orderedFrames(frames: unknown[]): MissionFrame[] {
  return frames.filter(validFrame).sort((a, b) => a.timestamp_s - b.timestamp_s || a.sequence - b.sequence).slice(-MAX_FRAMES);
}

export function encodeReplay(frames: MissionFrame[]): string {
  return frames.map((frame) => JSON.stringify(frame)).join('\n') + '\n';
}

export function decodeReplay(text: string): MissionFrame[] {
  const trimmed = text.trim();
  if (!trimmed) return [];
  let parsed: unknown;
  try { parsed = JSON.parse(trimmed); }
  catch { parsed = trimmed.split(/\r?\n/).map((line) => JSON.parse(line)); }
  if (Array.isArray(parsed)) return orderedFrames(parsed);
  if (validFrame(parsed)) return [parsed];
  if (parsed && typeof parsed === 'object' && Array.isArray((parsed as { frames?: unknown }).frames)) return orderedFrames((parsed as { frames: unknown[] }).frames);
  return [];
}
