import { useEffect, useRef, useState } from 'react';
import { COURSE_PRESETS } from './engine/course';
import { AutonomyEngine } from './engine/engine';
import type { EditTool, InspectionSelection, PlanningSnapshot } from './engine/types';
import { HistoryGraphs, MethodViews } from './ui/MethodViews';
import { LivePanel } from './ui/LivePanel';
import { SimulationCanvas } from './ui/SimulationCanvas';

declare global {
  interface Window {
    __AUTONOMY_ENGINE__?: AutonomyEngine;
    __PLANNING_SNAPSHOT__?: PlanningSnapshot;
  }
}

const TOOLS: { id: EditTool; label: string }[] = [
  { id: 'inspect', label: 'INSPECT' },
  { id: 'blue', label: 'BLUE CONE' },
  { id: 'yellow', label: 'YELLOW CONE' },
  { id: 'obstacle', label: 'OBSTACLE' },
  { id: 'erase', label: 'ERASE' },
];

export function App() {
  const engineRef = useRef<AutonomyEngine | null>(null);
  if (!engineRef.current) {
    engineRef.current = new AutonomyEngine();
    engineRef.current.setRunning(true);
  }
  const engine = engineRef.current;
  const [snapshot, setSnapshot] = useState(() => engine.getSnapshot());
  const [selection, setSelection] = useState<InspectionSelection>({ kind: 'kart', id: 'vehicle' });
  const [tool, setTool] = useState<EditTool>('inspect');
  const [presetId, setPresetId] = useState('oval');

  const refresh = () => {
    const next = engine.getSnapshot();
    window.__PLANNING_SNAPSHOT__ = next;
    setSnapshot(next);
  };

  useEffect(() => {
    window.__AUTONOMY_ENGINE__ = engine;
    window.__PLANNING_SNAPSHOT__ = snapshot;
  }, [engine, snapshot]);

  useEffect(() => {
    let animationFrame = 0;
    let revision = engine.getUiRevision();
    const loop = (time: number) => {
      engine.advanceFrame(time);
      const nextRevision = engine.getUiRevision();
      if (nextRevision !== revision) {
        revision = nextRevision;
        const next = engine.getSnapshot();
        window.__PLANNING_SNAPSHOT__ = next;
        setSnapshot(next);
      }
      animationFrame = requestAnimationFrame(loop);
    };
    animationFrame = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(animationFrame);
  }, [engine]);

  const toggleRun = (running: boolean) => {
    engine.setRunning(running);
    refresh();
  };

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand"><span className="brand-mark">Δ</span><div><b>DELAUNAY KART LAB</b><small>REAL-TIME AUTONOMY WORKBENCH</small></div></div>
        <div className="control-group primary-controls">
          <button className={snapshot.running ? '' : 'active'} onClick={() => toggleRun(true)}>▶ RUN</button>
          <button className={!snapshot.running ? 'active' : ''} onClick={() => toggleRun(false)}>Ⅱ PAUSE</button>
          <button disabled={snapshot.running} onClick={() => { engine.stepPlanningCycle(); refresh(); }}>STEP CYCLE</button>
          <button disabled={snapshot.running} onClick={() => { engine.stepTime(0.02); refresh(); }}>STEP 0.02 s</button>
          <button onClick={() => { engine.reset(); engine.setRunning(true); setSelection({ kind: 'kart', id: 'vehicle' }); refresh(); }}>↺ RESET</button>
        </div>
        <div className="control-group tool-controls">
          {TOOLS.map((item) => <button key={item.id} className={tool === item.id ? `active tool-${item.id}` : `tool-${item.id}`} onClick={() => setTool(item.id)}>{item.label}</button>)}
        </div>
        <div className="select-controls">
          <label>PRESET<select value={presetId} onChange={(event) => { setPresetId(event.target.value); engine.loadPreset(event.target.value); engine.setRunning(true); setSelection({ kind: 'kart', id: 'vehicle' }); refresh(); }}>
            {COURSE_PRESETS.map((preset) => <option key={preset.id} value={preset.id}>{preset.name}</option>)}
          </select></label>
          <label>SIM SPEED<select value={snapshot.simulationSpeed} onChange={(event) => { engine.setSimulationSpeed(Number(event.target.value)); refresh(); }}>
            {[0.25, 0.5, 1, 2].map((speed) => <option value={speed} key={speed}>{speed}×</option>)}
          </select></label>
        </div>
      </header>

      <main className="workbench">
        <section className="world-pane">
          <div className="world-title"><div><span className="eyebrow">MAIN SIMULATION WORLD</span><h1>Runtime geometry</h1></div><div className="legend"><span><i className="blue-dot" /> BLUE</span><span><i className="yellow-dot" /> YELLOW</span><span><i className="midpoint-dot" /> MIDPOINT</span><span><i className="path-line" /> CENTERLINE</span></div></div>
          <div className={`canvas-wrap cursor-${tool}`}>
            <SimulationCanvas engine={engine} selection={selection} tool={tool} onSelect={(next) => { setSelection(next); setTool('inspect'); }} onWorldEdit={refresh} />
            <div className="canvas-corner"><b>{tool.toUpperCase()}</b><span>{tool === 'inspect' ? 'Click geometry to inspect its source calculations' : 'Click in the world to edit the live course'}</span></div>
          </div>
        </section>
        <LivePanel snapshot={snapshot} selection={selection} onSelect={setSelection} />
      </main>

      <footer className="analysis-deck">
        <div className="bottom-heading"><div><span className="eyebrow">METHODOLOGY VIEWS</span><h2>One planning cycle, four transformations</h2></div><span>snapshot #{snapshot.cycle} · t {snapshot.simulationTime.toFixed(2)} s</span></div>
        <MethodViews snapshot={snapshot} onSelect={setSelection} />
        <div className="bottom-heading history-heading"><div><span className="eyebrow">ROLLING ENGINEERING HISTORY</span><h2>Last ~10 seconds</h2></div><span>all series sampled from PlanningSnapshot</span></div>
        <HistoryGraphs snapshot={snapshot} />
      </footer>
    </div>
  );
}
