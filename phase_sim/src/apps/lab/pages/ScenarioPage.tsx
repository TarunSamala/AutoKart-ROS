import { useMemo, useState } from 'react';
import { DEFAULT_LAB_SETTINGS, generateScenario, type LabScenario, type LabSettings, type ScenarioName } from '../../../mission/labScenarios';
import { Link } from '../router';
import { useReportProvenance, useRun } from '../runContext';

/**
 * /scenario: seeded course generator (existing labScenarios.generateScenario) with fixture
 * import/export. A fixture is (scenario, settings); generation is deterministic, so the
 * fingerprint below lets you confirm a re-opened fixture reproduces the same cone map.
 */
export const FIXTURE_KIND = 'autokart-lab-scenario';

type Field = { key: keyof LabSettings; label: string; min: number; max: number; step: number };
const COMMON: Field[] = [
  { key: 'width', label: 'Track width (m)', min: 2.5, max: 6, step: 0.1 },
  { key: 'noise', label: 'Detection noise (m)', min: 0, max: 0.5, step: 0.01 },
  { key: 'dropout', label: 'Detection dropout (0–1)', min: 0, max: 0.5, step: 0.01 },
  { key: 'seed', label: 'Seed', min: 0, max: 999999, step: 1 },
];
const SKIDPAD: Field[] = [
  { key: 'skidpadRadius', label: 'Loop radius (m)', min: 5, max: 15, step: 0.5 },
  { key: 'coneSpacing', label: 'Cone spacing (m)', min: 0.7, max: 3, step: 0.1 },
  { key: 'entryLength', label: 'Entry length (m)', min: 8, max: 30, step: 0.5 },
  { key: 'exitLength', label: 'Exit length (m)', min: 8, max: 30, step: 0.5 },
  { key: 'junctionLength', label: 'Clear junction (m)', min: 1.5, max: 5, step: 0.1 },
];
const AUTOCROSS: Field[] = [
  { key: 'complexity', label: 'Complexity (0–1)', min: 0, max: 1, step: 0.05 },
  { key: 'length', label: 'Length (m)', min: 170, max: 500, step: 10 },
  { key: 'hairpins', label: 'Hairpins', min: 1, max: 5, step: 1 },
  { key: 'sBends', label: 'S bends', min: 2, max: 8, step: 1 },
  { key: 'slaloms', label: 'Slalom sets', min: 2, max: 8, step: 1 },
  { key: 'coneSpacing', label: 'Cone spacing (m)', min: 1, max: 4, step: 0.1 },
];

export function scenarioFingerprint(scenario: LabScenario): string {
  let hash = 0x811c9dc5;
  const feed = (text: string) => { for (let i = 0; i < text.length; i++) { hash ^= text.charCodeAt(i); hash = Math.imul(hash, 0x01000193) >>> 0; } };
  scenario.cones.forEach((cone) => feed(`${cone.id}:${cone.class}:${cone.x_m.toFixed(3)}:${cone.y_m.toFixed(3)};`));
  if (scenario.obstacle) feed(`obs:${scenario.obstacle.x_m.toFixed(3)}:${scenario.obstacle.y_m.toFixed(3)}`);
  return hash.toString(16).padStart(8, '0');
}

export function parseFixture(text: string): { scenario: ScenarioName; settings: LabSettings } {
  const value = JSON.parse(text) as Record<string, unknown>;
  if (value.kind !== FIXTURE_KIND || value.version !== 1) throw new Error(`Not an ${FIXTURE_KIND} v1 fixture`);
  if (!['SKIDPAD', 'AUTOCROSS', 'BRAKE'].includes(value.scenario as string)) throw new Error('Unknown scenario in fixture');
  const raw = (value.settings ?? {}) as Record<string, unknown>;
  const settings = { ...DEFAULT_LAB_SETTINGS };
  for (const key of Object.keys(settings) as Array<keyof LabSettings>) if (typeof raw[key] === 'number' && Number.isFinite(raw[key])) settings[key] = raw[key] as number;
  return { scenario: value.scenario as ScenarioName, settings };
}

function Preview({ scenario }: { scenario: LabScenario }) {
  const xs = scenario.cones.map((cone) => cone.x_m), ys = scenario.cones.map((cone) => cone.y_m);
  const minX = Math.min(...xs, scenario.start.x_m) - 3, maxX = Math.max(...xs, scenario.start.x_m) + 3;
  const minY = Math.min(...ys, scenario.start.y_m) - 3, maxY = Math.max(...ys, scenario.start.y_m) + 3;
  const w = maxX - minX, h = maxY - minY;
  const color = (side: string) => side === 'BLUE' ? '#4a8fff' : side === 'YELLOW' ? '#ffd451' : '#ff923e';
  const r = Math.max(w, h) / 220;
  return <figure className="lab-figure">
    <svg viewBox={`${minX} ${-maxY} ${w} ${h}`} role="img" aria-label={`${scenario.name} cone map preview, ${scenario.cones.length} cones`}>
      <rect x={minX} y={-maxY} width={w} height={h} className="fig-bg" />
      {scenario.guide.length > 1 && <polyline points={scenario.guide.map((p) => `${p.x_m},${-p.y_m}`).join(' ')} fill="none" stroke="#2f5d6b" strokeWidth={r * 0.8} strokeDasharray={`${r * 3} ${r * 2}`} />}
      {scenario.cones.map((cone) => <circle key={cone.id} cx={cone.x_m} cy={-cone.y_m} r={r * 1.6} fill={color(cone.class)} />)}
      {scenario.obstacle && <circle cx={scenario.obstacle.x_m} cy={-scenario.obstacle.y_m} r={scenario.obstacle.radius_m} fill="none" stroke="#ff6674" strokeWidth={r} />}
      <g transform={`translate(${scenario.start.x_m} ${-scenario.start.y_m}) rotate(${-scenario.start.heading * 180 / Math.PI})`}><path d={`M ${r * 7} 0 L ${-r * 4} ${-r * 4} L ${-r * 4} ${r * 4} Z`} fill="#36edc1" /></g>
    </svg>
    <figcaption>Ground-truth cone map from <code>generateScenario</code> (top-down, X right, Y up). Blue = left, yellow = right, orange = special, red ring = obstacle, teal = start pose.</figcaption>
  </figure>;
}

export function ScenarioPage() {
  const { run, setRun } = useRun();
  const [name, setName] = useState<ScenarioName>(run.scenario);
  const [settings, setSettings] = useState<LabSettings>(run.settings);
  const [message, setMessage] = useState<{ text: string; error: boolean } | null>(null);
  const scenario = useMemo(() => generateScenario(name, settings), [name, settings]);
  const fingerprint = scenarioFingerprint(scenario);
  useReportProvenance('SYNTHETIC', `generated ${name} course · seed ${settings.seed} · fingerprint ${fingerprint}`);
  const fields = [...(name === 'SKIDPAD' ? SKIDPAD : name === 'AUTOCROSS' ? AUTOCROSS : []), ...COMMON];
  const applied = run.scenario === name && JSON.stringify(run.settings) === JSON.stringify(settings);
  const exportFixture = () => {
    const body = JSON.stringify({ kind: FIXTURE_KIND, version: 1, scenario: name, settings, run_id: run.runId, cone_count: scenario.cones.length, fingerprint, exported_utc: new Date().toISOString() }, null, 2);
    const url = URL.createObjectURL(new Blob([body], { type: 'application/json' }));
    const link = document.createElement('a');
    link.href = url; link.download = `scenario-${name.toLowerCase()}-s${settings.seed}.json`; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  return <article className="lab-page lab-doc" data-page="scenario" aria-labelledby="scenario-title">
    <p className="lab-eyebrow">MISSION & SIM · SCENARIO BUILDER</p>
    <h1 id="scenario-title">Scenario builder</h1>
    <p>Generates the client-side sandbox course used by <Link to="/mission">Mission</Link>. The Python CORE pages use lab_api's own fixed world and are not affected.</p>
    <div className="lab-stage-layout">
      <Preview scenario={scenario} />
      <section className="lab-card" aria-labelledby="scenario-settings">
        <h2 id="scenario-settings">Settings</h2>
        <div className="lab-form">
          <label>Scenario<select value={name} onChange={(event) => setName(event.target.value as ScenarioName)}>
            <option value="SKIDPAD">SKIDPAD</option><option value="AUTOCROSS">AUTOCROSS</option><option value="BRAKE">BRAKE</option></select></label>
          {fields.map((field) => <label key={field.key}>{field.label}<input type="number" min={field.min} max={field.max} step={field.step} value={settings[field.key]}
            onChange={(event) => { const value = Number(event.target.value); if (Number.isFinite(value)) setSettings({ ...settings, [field.key]: Math.min(field.max, Math.max(field.min, value)) }); }} /></label>)}
        </div>
        <dl className="lab-facts">
          <div><dt>CONES</dt><dd>{scenario.cones.length}</dd></div>
          <div><dt>OBSTACLE</dt><dd>{scenario.obstacle ? `${scenario.obstacle.id} @ ${scenario.obstacle.x_m.toFixed(1)}, ${scenario.obstacle.y_m.toFixed(1)} m` : 'none'}</dd></div>
          <div><dt>FINGERPRINT</dt><dd><code data-testid="scenario-fingerprint">{fingerprint}</code></dd></div>
        </dl>
        <div className="lab-toolbar">
          <button onClick={() => { setRun({ scenario: name, settings }); setMessage({ text: 'Applied to the run. Mission will generate this course on entry.', error: false }); }} disabled={applied}>{applied ? 'In use for this run' : 'Use for this run'}</button>
          <button onClick={exportFixture}>Export fixture (.json)</button>
          <label className="lab-file-label">Import fixture<input type="file" accept=".json,application/json" onChange={async (event) => {
            const file = event.target.files?.[0];
            if (!file) return;
            try { const fixture = parseFixture(await file.text()); setName(fixture.scenario); setSettings(fixture.settings); setMessage({ text: `Imported ${file.name}. Compare the fingerprint with the exported one.`, error: false }); }
            catch (cause) { setMessage({ text: cause instanceof Error ? cause.message : String(cause), error: true }); }
            event.target.value = '';
          }} /></label>
        </div>
        {message && <p className={message.error ? 'lab-error' : 'lab-note'} role={message.error ? 'alert' : 'status'}>{message.text}</p>}
        <p className="lab-note"><b>Not implemented in this build:</b> free-hand cone placement, vehicle start-pose editing and custom obstacles. Courses come only from (scenario, settings, seed). To move individual cones against the real Python core, use <Link to="/core">Core Lab</Link>.</p>
      </section>
    </div>
  </article>;
}
