import { useEffect, useRef, useState } from 'react';
import type { MissionFrame, Mode, Source } from './schema';
import { frameAt } from './schema';
import { AutonomyEngine } from '../engine/engine';
import { demoFrame } from './demo';
import { LabSkidpad } from './LabSkidpad';
import { DEFAULT_LAB_SETTINGS, type LabSettings, type ScenarioName } from './labScenarios';
import { decodeReplay, encodeReplay, orderedFrames, validFrame } from './replay';

const MAX_FRAMES = 6000;

/**
 * User-visible names for each Source. The internal value 'LAB' is kept for replay-file
 * compatibility, but it is the client-side JS skidpad sandbox (LabSkidpad.ts) -- NOT the
 * Python shared_core ALGORITHM LAB at /shared -- so it is never displayed as "LAB".
 */
export const SOURCE_LABEL: Record<Source, string> = {
  LAB: 'SKIDPAD SANDBOX · JS',
  SIMULATION: 'ROS SIMULATION',
  REAL_KART: 'REAL KART · READ-ONLY',
  DEMO: 'LEGACY DEMO · SYNTHETIC',
};

/**
 * The default route is the dev/simulation tool, so it opens in the client-side sandbox
 * (no backend, clearly labelled) rather than in a real-kart view; the locked real-only
 * view lives at /ops and the production live dashboard at /live.
 */
export const DEFAULT_SOURCE: Source = 'LAB';

export function useMissionData() {
  const lab = useRef<LabSkidpad | null>(null);
  if (!lab.current) lab.current = new LabSkidpad();
  const [frames, setFrames] = useState<MissionFrame[]>(() => DEFAULT_SOURCE === 'LAB' ? [lab.current!.getFrame()] : []);
  const [transportStatus, setStatus] = useState('CONNECTING');
  const [source, setSource] = useState<Source>(DEFAULT_SOURCE);
  const [mode, setMode] = useState<Mode>(DEFAULT_SOURCE === 'LAB' ? 'PAUSE' : 'LIVE');
  const [labHasRun, setLabHasRun] = useState(false);
  const [labScenario, setLabScenario] = useState<ScenarioName>('SKIDPAD');
  const [labSettings, setLabSettings] = useState<LabSettings>(DEFAULT_LAB_SETTINGS);
  const [labGeneration, setLabGeneration] = useState(0);
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [manifest, setManifest] = useState<Record<string, unknown>>(() => DEFAULT_SOURCE === 'LAB'
    ? { run_id: lab.current!.getFrame().run_id, scenario: 'SKIDPAD', source: 'LAB', settings: DEFAULT_LAB_SETTINGS } : {});
  const socket = useRef<WebSocket | null>(null);
  const latest = useRef(0);
  const lastFrameArrival = useRef(0);
  const [liveFrameFresh, setLiveFrameFresh] = useState(false);
  const demo = useRef<AutonomyEngine | null>(null);
  const [demoPreset, setDemoPreset] = useState('mission-skidpad');
  const modeRef = useRef(mode);
  modeRef.current = mode;

  useEffect(() => {
    if (source !== 'LAB' || modeRef.current === 'REPLAY') return;
    const engine = lab.current!;
    let animation = 0;
    let lastPublished = 0;
    const tick = (now: number) => {
      const running = modeRef.current === 'LIVE';
      if (engine.isRunning() !== running) engine.setRunning(running);
      if (running) engine.advanceFrame(now);
      if (running && now - lastPublished >= 100) {
        lastPublished = now;
        const frame = engine.getFrame();
        setFrames((previous) => [...previous, frame].slice(-MAX_FRAMES));
        setTime(frame.timestamp_s);
        if (frame.planning.skidpad?.state === 'STOP') setMode('PAUSE');
      }
      animation = requestAnimationFrame(tick);
    };
    animation = requestAnimationFrame(tick);
    return () => { cancelAnimationFrame(animation); engine.setRunning(false); };
  }, [source, mode === 'REPLAY', labGeneration]);

  useEffect(() => {
    if (source !== 'DEMO' || modeRef.current === 'REPLAY') return;
    const engine = new AutonomyEngine(demoPreset);
    engine.setRunning(true);
    demo.current = engine;
    setFrames([]); setTime(0); setStatus('DEMO · SYNTHETIC'); setManifest({ run_id: `demo-${demoPreset}`, scenario: demoPreset, source: 'DEMO' });
    setFrames([demoFrame(engine.getSnapshot(), demoPreset)]);
    let animation = 0;
    let last = 0;
    const tick = (now: number) => {
      if (engine.isRunning() !== (modeRef.current === 'LIVE')) engine.setRunning(modeRef.current === 'LIVE');
      if (modeRef.current === 'LIVE') engine.advanceFrame(now);
      if (modeRef.current === 'LIVE' && now - last >= 100) {
        last = now;
        const frame = demoFrame(engine.getSnapshot(), demoPreset);
        setFrames((previous) => [...previous, frame].slice(-MAX_FRAMES));
        setTime(frame.timestamp_s);
      }
      animation = requestAnimationFrame(tick);
    };
    animation = requestAnimationFrame(tick);
    return () => { cancelAnimationFrame(animation); demo.current = null; };
  }, [source, demoPreset, mode === 'REPLAY']);

  useEffect(() => {
    if (mode === 'REPLAY' || source === 'DEMO' || source === 'LAB') { socket.current?.close(); return; }
    let cancelled = false;
    let replayStream = false;
    let reconnect: ReturnType<typeof setTimeout> | undefined;
    let ws: WebSocket | null = null;
    const connect = () => {
      const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
      ws = new WebSocket(`${protocol}//${location.host}/mission/ws`);
      socket.current = ws;
      setStatus('CONNECTING');
      ws.onopen = () => setStatus('CONNECTED');
      ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          if (message.type === 'manifest') { setManifest(message.manifest ?? {}); replayStream = message.manifest?.mode === 'REPLAY'; if (message.manifest?.source !== source) setStatus('SOURCE MISMATCH'); return; }
          if (message.type !== 'frame' || !validFrame(message.frame)) return;
          const frame = message.frame as MissionFrame;
          if (frame.source !== source) return;
          if (frame.timestamp_s < latest.current) { setFrames([]); }
          latest.current = frame.timestamp_s;
          lastFrameArrival.current = performance.now();
          setLiveFrameFresh(true);
          setFrames((previous) => [...previous.filter((item) => item.timestamp_s <= frame.timestamp_s), frame].slice(-MAX_FRAMES));
          if (modeRef.current === 'LIVE') setTime(frame.timestamp_s);
        } catch { setStatus('INVALID FRAME'); }
      };
      ws.onclose = () => { if (!cancelled) { setLiveFrameFresh(false); if (replayStream) { setStatus('REPLAY LOADED'); setMode('REPLAY'); } else { setStatus('DISCONNECTED'); reconnect = setTimeout(connect, 1500); } } };
      ws.onerror = () => ws?.close();
    };
    connect();
    return () => { cancelled = true; clearTimeout(reconnect); ws?.close(); };
  }, [mode === 'REPLAY', source]);

  useEffect(() => {
    if (source !== 'REAL_KART' && source !== 'SIMULATION') return;
    const timer = window.setInterval(() => {
      if (performance.now() - lastFrameArrival.current > 1000) setLiveFrameFresh(false);
    }, 250);
    return () => window.clearInterval(timer);
  }, [source]);

  useEffect(() => {
    if (!playing || mode !== 'REPLAY' || frames.length < 2) return;
    let animation = 0;
    let wall = performance.now();
    const advance = (now: number) => {
      const dt = Math.min(0.1, (now - wall) / 1000);
      wall = now;
      setTime((current) => {
        const next = current + dt;
        if (next >= frames[frames.length - 1].timestamp_s) { setPlaying(false); return frames[frames.length - 1].timestamp_s; }
        return next;
      });
      animation = requestAnimationFrame(advance);
    };
    animation = requestAnimationFrame(advance);
    return () => cancelAnimationFrame(animation);
  }, [playing, mode, frames]);

  const loadReplay = async (file: File) => {
    const next = decodeReplay(await file.text());
    if (!next.length) throw new Error('No Mission Control frames in file');
    setMode('REPLAY'); setPlaying(false); setFrames(next); setSource(next[0].source); setTime(next[0].timestamp_s);
    setManifest({ run_id: next[0].run_id, samples: next.length, source: next[0].source });
    setStatus('REPLAY LOADED');
  };
  const openGatewayReplay = async () => {
    const response = await fetch('/mission/api/replay');
    if (!response.ok) throw new Error(`Gateway replay: HTTP ${response.status}`);
    const payload = await response.json();
    const next = orderedFrames(payload.frames ?? []);
    if (!next.length) throw new Error('Gateway has no replay frames');
    setMode('REPLAY'); setPlaying(false); setFrames(next); setSource(next[0].source); setTime(next[0].timestamp_s);
    setManifest(payload.manifest ?? {}); setStatus('REPLAY LOADED');
  };
  const current = mode === 'LIVE' && (source === 'REAL_KART' || source === 'SIMULATION')
    ? liveFrameFresh ? frames[frames.length - 1] ?? null : null
    : mode === 'LIVE' ? frames[frames.length - 1] ?? null : frameAt(frames, time);
  const resetLab = () => {
    const engine = lab.current!;
    engine.reset();
    setSource('LAB'); setMode('PAUSE'); setPlaying(false); setLabHasRun(false);
    setFrames([engine.getFrame()]); setTime(0);
    setManifest({ run_id: engine.getFrame().run_id, scenario: labScenario, source: 'LAB', settings: labSettings });
  };
  const configureLab = (scenario: ScenarioName, settings: LabSettings = labSettings) => {
    const engine = new LabSkidpad(scenario, settings);
    lab.current = engine;
    setLabScenario(scenario); setLabSettings({ ...settings }); setLabGeneration((current) => current + 1);
    setSource('LAB'); setMode('PAUSE'); setPlaying(false); setLabHasRun(false);
    setFrames([engine.getFrame()]); setTime(0);
    setManifest({ run_id: engine.getFrame().run_id, scenario, source: 'LAB', settings });
  };
  const runLab = () => {
    if (mode === 'REPLAY') resetLab();
    setLabHasRun(true); setMode('LIVE'); setPlaying(false);
  };
  const chooseSource = (next: Source) => {
    if (next === 'LAB') { resetLab(); return; }
    setSource(next); setMode('LIVE'); setPlaying(false); latest.current = 0; lastFrameArrival.current = 0; setLiveFrameFresh(false); setFrames([]); setTime(0); setManifest({});
  };
  const status = source === 'LAB' && mode !== 'REPLAY' ? mode === 'LIVE' ? 'RUNNING' : labHasRun ? 'PAUSED' : 'READY'
    : mode === 'LIVE' && (source === 'REAL_KART' || source === 'SIMULATION') && transportStatus === 'CONNECTED'
      ? liveFrameFresh ? 'CURRENT ROS FRAME' : 'GATEWAY CONNECTED · NO CURRENT ROS FRAME'
      : transportStatus;
  const exportReplay = () => {
    const blob = new Blob([encodeReplay(frames)], { type: 'application/x-ndjson' });
    const url = URL.createObjectURL(blob), link = document.createElement('a');
    link.href = url; link.download = `${String(manifest.run_id ?? 'autokart-run')}.jsonl`; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  return { frames, current, status, source, setSource: chooseSource, mode, setMode, time, setTime, playing, setPlaying, manifest, loadReplay, openGatewayReplay, demoPreset, setDemoPreset, runLab, resetLab, configureLab, labScenario, labSettings, exportReplay };
}
