import { useId, useState } from 'react';
import { useRun } from '../runContext';

/** Opens a MissionFrame recording through the existing replay decoder (src/mission/replay.ts). */
export function RecordingPicker({ label = 'Open recording (.jsonl / .json)' }: { label?: string }) {
  const { openRecording } = useRun();
  const [message, setMessage] = useState('');
  const [error, setError] = useState(false);
  const id = useId();
  return <div className="lab-recording-picker">
    <label htmlFor={id} className="lab-file-label">{label}</label>
    <input id={id} type="file" accept=".jsonl,.json,application/json,application/x-ndjson" onChange={async (event) => {
      const file = event.target.files?.[0];
      if (!file) return;
      try {
        const recording = await openRecording(file);
        setError(false); setMessage(`Opened ${recording.name}: ${recording.frames.length} frames.`);
      } catch (cause) {
        setError(true); setMessage(cause instanceof Error ? cause.message : String(cause));
      }
      event.target.value = '';
    }} />
    {message && <p className={error ? 'lab-error' : 'lab-note'} role={error ? 'alert' : 'status'}>{message}</p>}
  </div>;
}
