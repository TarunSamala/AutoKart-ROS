import { exportTelemetryCsv, exportTelemetryJson } from '../mission/labAlgorithms';
import type { MissionFrame } from '../mission/schema';

/** Triggers a browser download of `text` as `filename`. Same pattern useMissionData.exportReplay
 * uses for the existing "SAVE RUN (.jsonl)" button -- reused here instead of duplicated. */
export function downloadText(text: string, filename: string, mime: string) {
  const blob = new Blob([text], { type: mime });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = filename;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

/** Downsamples a frame history to roughly `hz` frames/second by timestamp bucket, for the
 * "Export 1Hz CSV" / "Export 10Hz CSV" buttons. Keeps the first frame seen per bucket. Frames
 * are assumed sorted by `timestamp_s` ascending (true for both the Lab sandbox and replay). */
export function downsampleByRate(frames: MissionFrame[], hz: number): MissionFrame[] {
  if (!frames.length || hz <= 0) return frames;
  const period = 1 / hz;
  const out: MissionFrame[] = [];
  let nextBucket = -Infinity;
  for (const frame of frames) {
    if (frame.timestamp_s >= nextBucket) {
      out.push(frame);
      nextBucket = frame.timestamp_s + period - 1e-9;
    }
  }
  return out;
}

export function exportCsvAtRate(frames: MissionFrame[], hz: number, runId: string) {
  downloadText(exportTelemetryCsv(downsampleByRate(frames, hz)), `${runId}-${hz}hz.csv`, 'text/csv');
}

export function exportJsonSummary(frames: MissionFrame[], runId: string) {
  downloadText(exportTelemetryJson(frames), `${runId}-summary.json`, 'application/json');
}
