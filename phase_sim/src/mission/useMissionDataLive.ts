import { useEffect, useRef, useState } from 'react';
import type { MissionFrame, Mode } from './schema';
import { frameAt } from './schema';

/**
 * Data layer for the live-only operational Mission Control entry point
 * (MissionControlLive.tsx / the `/ops` route).
 *
 * This hook is intentionally a SEPARATE, minimal module from `useMissionData.ts`.
 * It is hard-locked to the real ROS2 gateway (`source: 'REAL_KART'`) and has
 * zero code path that can select or render LAB, SIMULATION, REPLAY, or DEMO
 * data: there is no `setSource`, no lab engine, no demo engine, and no file
 * or gateway-replay loader. It imports nothing from `./LabSkidpad`,
 * `../engine/engine`, or `./demo` -- only `./schema` types/helpers, so the
 * live-only bundle cannot pull in any synthetic-data generator at build time.
 *
 * On gateway disconnect it clears `liveFrameFresh` and `current` becomes
 * `null` (while `frames.at(-1)` -- the last stale value -- is still exposed
 * for display as a "STALE"/disconnected read, see MissionControlLive.tsx).
 * Nothing here ever substitutes a fabricated frame for a missing live one.
 */

const MAX_FRAMES = 6000;
const STALE_AFTER_MS = 1000;

export function useMissionDataLive() {
  const [frames, setFrames] = useState<MissionFrame[]>([]);
  const [transportStatus, setTransportStatus] = useState('CONNECTING');
  const [mode, setMode] = useState<Mode>('LIVE');
  const [time, setTime] = useState(0);
  const [manifest, setManifest] = useState<Record<string, unknown>>({});
  const [liveFrameFresh, setLiveFrameFresh] = useState(false);
  const [demoDriveRejected, setDemoDriveRejected] = useState<string | null>(null);
  const latest = useRef(0);
  const lastFrameArrival = useRef(0);
  const modeRef = useRef(mode);
  modeRef.current = mode;
  // DEMO DRIVE (not autonomy): reuses this same persistent /mission/ws connection rather than
  // opening a second socket, so a hold-to-drive ping cadence doesn't need its own reconnect logic.
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    let cancelled = false;
    let reconnect: ReturnType<typeof setTimeout> | undefined;
    let ws: WebSocket | null = null;
    const connect = () => {
      const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
      ws = new WebSocket(`${protocol}//${location.host}/mission/ws`);
      wsRef.current = ws;
      setTransportStatus('CONNECTING');
      ws.onopen = () => setTransportStatus('CONNECTED');
      ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          if (message.type === 'manifest') {
            setManifest(message.manifest ?? {});
            if (message.manifest?.source !== 'REAL_KART') setTransportStatus('SOURCE MISMATCH');
            return;
          }
          if (message.type === 'demo_drive_rejected') { setDemoDriveRejected(String(message.action)); return; }
          if (message.type !== 'frame') return;
          const frame = message.frame as MissionFrame;
          // Only ever accept frames that are self-tagged REAL_KART -- never
          // fall through to rendering a simulated/lab/demo-tagged frame here.
          if (frame.source !== 'REAL_KART') return;
          if (frame.timestamp_s < latest.current) setFrames([]);
          latest.current = frame.timestamp_s;
          lastFrameArrival.current = performance.now();
          setLiveFrameFresh(true);
          setFrames((previous) => [...previous.filter((item) => item.timestamp_s <= frame.timestamp_s), frame].slice(-MAX_FRAMES));
          if (modeRef.current === 'LIVE') setTime(frame.timestamp_s);
        } catch {
          setTransportStatus('INVALID FRAME');
        }
      };
      ws.onclose = () => {
        if (cancelled) return;
        wsRef.current = null;
        setLiveFrameFresh(false);
        setTransportStatus('DISCONNECTED');
        reconnect = setTimeout(connect, 1500);
      };
      ws.onerror = () => ws?.close();
    };
    connect();
    return () => { cancelled = true; clearTimeout(reconnect); ws?.close(); wsRef.current = null; };
  }, []);

  // DEMO DRIVE only: sends one JSON message on the live connection if (and only if) it is open.
  // Returns false without sending anything on any disconnect -- the caller (DemoDrivePanel) must
  // treat that as "stop", never retry silently, matching "STOP always works from any state".
  const sendDemoDrive = (message: Record<string, unknown>): boolean => {
    const socket = wsRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) return false;
    socket.send(JSON.stringify({ type: 'demo_drive', ...message }));
    return true;
  };

  useEffect(() => {
    const timer = window.setInterval(() => {
      if (performance.now() - lastFrameArrival.current > STALE_AFTER_MS) setLiveFrameFresh(false);
    }, 250);
    return () => window.clearInterval(timer);
  }, []);

  // LIVE shows the newest frame only while it is still fresh -- never a held
  // or synthesized frame. PAUSE freezes the view at the timestamp captured
  // when pause was pressed, read back from the already-received buffer.
  const current = mode === 'LIVE'
    ? (liveFrameFresh ? frames[frames.length - 1] ?? null : null)
    : frameAt(frames, time);

  const status = transportStatus === 'CONNECTED'
    ? (liveFrameFresh ? 'CURRENT ROS FRAME' : 'GATEWAY CONNECTED · NO CURRENT ROS FRAME')
    : transportStatus;

  const pause = () => { setTime(frames[frames.length - 1]?.timestamp_s ?? time); setMode('PAUSE'); };
  const resumeLive = () => setMode('LIVE');

  return {
    source: 'REAL_KART' as const,
    frames,
    current,
    status,
    mode,
    pause,
    resumeLive,
    time,
    manifest,
    liveFrameFresh,
    sendDemoDrive,
    demoDriveRejected,
    wsConnected: transportStatus !== 'DISCONNECTED' && transportStatus !== 'CONNECTING',
  };
}
