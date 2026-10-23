import { useState } from 'react';
import type { ScenarioName } from '../../../mission/labScenarios';
import type { Services } from '../services';
import { Link } from '../router';
import { LAB_ROUTES } from '../routes';
import { LIVE_READ_ONLY_REASON, useReportProvenance, useRun } from '../runContext';
import { RecordingPicker } from './RecordingPicker';

function serviceRow(name: string, where: string, state: Services['core'], extra?: string) {
  const text = state.state === 'CHECKING' ? 'checking…' : state.state === 'REACHABLE' ? `reachable${extra ? ` · ${extra}` : ''}`
    : state.state === 'REFUSED' ? `REFUSED · ${state.detail}` : `offline · ${state.detail}`;
  return <div key={name}><dt>{name}</dt><dd><code>{where}</code> — {text}</dd></div>;
}

export function HomePage({ services }: { services: Services }) {
  const { run, setRun, newRunId, recording, clearRecording } = useRun();
  const [seedText, setSeedText] = useState(String(run.settings.seed));
  const replayReady = run.mode === 'REPLAY' && !!recording;
  useReportProvenance(run.mode === 'SYNTHETIC' ? 'SYNTHETIC' : replayReady ? 'REPLAY' : null,
    run.mode === 'SYNTHETIC' ? `run ${run.runId} · ${run.scenario} · seed ${run.settings.seed} (selected for sandbox pages)` : replayReady ? `recording ${recording!.name} · ${recording!.frames.length} frames` : '');
  const core = services.core.state === 'REACHABLE' ? services.core.payload : null;
  const gateway = services.simGateway.state === 'REACHABLE' ? services.simGateway.payload : null;
  const applySeed = (text: string) => {
    setSeedText(text);
    const seed = Number(text);
    if (Number.isInteger(seed) && seed >= 0 && seed <= 999999) setRun({ settings: { ...run.settings, seed } });
  };
  const seedInvalid = !(Number.isInteger(Number(seedText)) && Number(seedText) >= 0 && Number(seedText) <= 999999);
  return <article className="lab-page lab-doc" data-page="home" aria-labelledby="home-title">
    <p className="lab-eyebrow">RUN MANAGER</p>
    <h1 id="home-title">Lab Home</h1>
    <p>The Algorithm Lab runs only on synthetic or recorded data. It has no route to the vehicle, the live gateway or the live camera, and no page can command a kart.</p>

    <div className="lab-grid-2">
      <section className="lab-card" aria-labelledby="run-source">
        <h2 id="run-source">1 · Data source</h2>
        <fieldset className="lab-radio-group">
          <legend>Run provenance</legend>
          <label><input type="radio" name="run-mode" value="SYNTHETIC" checked={run.mode === 'SYNTHETIC'} onChange={() => setRun({ mode: 'SYNTHETIC' })} />
            <span><b>SYNTHETIC</b> — generated scenarios (client sandbox) and the Python CORE simulation.</span></label>
          <label><input type="radio" name="run-mode" value="REPLAY" checked={run.mode === 'REPLAY'} onChange={() => setRun({ mode: 'REPLAY' })} />
            <span><b>REPLAY</b> — a recorded MissionFrame <code>.jsonl</code> / <code>.json</code> file opened in this tab.</span></label>
          <label className="disabled"><input type="radio" name="run-mode" value="LIVE READ-ONLY" disabled aria-describedby="home-live-reason" />
            <span><b>LIVE READ-ONLY</b> — <span id="home-live-reason">{LIVE_READ_ONLY_REASON}</span></span></label>
        </fieldset>
        {run.mode === 'REPLAY' && <div className="lab-subcard">
          <h3>Recording</h3>
          {recording ? <p><b>{recording.name}</b> · {recording.frames.length} frames · run <code>{String(recording.frames[0].run_id)}</code> · recorded source <code>{recording.frames[0].source}</code>
            {' '}<button className="lab-link-button" onClick={clearRecording}>Close recording</button></p>
            : <p className="lab-warn-text">No recording loaded. Recordings live only in this tab's memory and must be re-opened after a reload.</p>}
          <RecordingPicker />
        </div>}
      </section>

      <section className="lab-card" aria-labelledby="run-config">
        <h2 id="run-config">2 · Run identity and config</h2>
        <div className="lab-form">
          <label>Run id<input value={run.runId} maxLength={120} onChange={(event) => setRun({ runId: event.target.value })} /></label>
          <button onClick={newRunId}>New run id</button>
          <label>Scenario<select value={run.scenario} onChange={(event) => setRun({ scenario: event.target.value as ScenarioName })}>
            <option value="SKIDPAD">SKIDPAD</option><option value="AUTOCROSS">AUTOCROSS</option><option value="BRAKE">BRAKE</option><option value="MY_TRACK">MY_TRACK</option></select></label>
          <label>Seed<input inputMode="numeric" value={seedText} aria-invalid={seedInvalid} aria-describedby="seed-help" onChange={(event) => applySeed(event.target.value)} /></label>
          <p id="seed-help" className={seedInvalid ? 'lab-warn-text' : 'lab-note'}>{seedInvalid ? 'Seed must be an integer 0–999999; the last valid seed is kept.' : 'Integer 0–999999.'}</p>
        </div>
        <p className="lab-note">Scenario and seed configure the client-side sandbox (<Link to="/mission">Mission</Link>, <Link to="/scenario">Scenario</Link>). The Python CORE pages use lab_api's own fixed STRAIGHT_TRACK world, which has no seed parameter. Full scenario settings are on <Link to="/scenario">Scenario</Link>.</p>
        <dl className="lab-facts">
          <div><dt>CORE algorithm</dt><dd><code>{String(core?.algorithm_version ?? '—')}</code></dd></div>
          <div><dt>CORE config sha256</dt><dd><code title={String(core?.configuration_sha256 ?? '')}>{String(core?.configuration_sha256 ?? '—').slice(0, 16)}</code></dd></div>
          <div><dt>Model</dt><dd>— (no detector runs in the Lab build)</dd></div>
        </dl>
      </section>
    </div>

    <section className="lab-card" aria-labelledby="backends">
      <h2 id="backends">3 · Backends</h2>
      <dl className="lab-facts lab-facts-wide">
        {serviceRow('Lab CORE API', '/api/core → 127.0.0.1:8767', services.core, core ? `status ${String(core.status)}` : undefined)}
        {serviceRow('Simulation ROS gateway', '/mission → 127.0.0.1:8775', services.simGateway, gateway ? `source ${String(gateway.source ?? '—')} · mode ${String(gateway.mode ?? '—')}` : undefined)}
      </dl>
      <p className="lab-note">"Reachable" means the endpoint answered with JSON. It is not a health verdict. When the simulation gateway is offline, the ROS SIMULATION source on Mission shows as disconnected and the other pages keep working.</p>
    </section>

    <section className="lab-card" aria-labelledby="pages">
      <h2 id="pages">4 · Pages in this build</h2>
      <div className="lab-table-wrap" role="region" aria-label="Lab pages" tabIndex={0}><table className="lab-table">
        <thead><tr><th scope="col">PAGE</th><th scope="col">STATUS</th><th scope="col">BACKED BY</th><th scope="col">CONTENT</th></tr></thead>
        <tbody>{LAB_ROUTES.map((route) => <tr key={route.path}>
          <td><Link to={route.path}>{route.label}</Link> <code>{route.path}</code></td>
          <td className={`route-status status-${route.status.replace(' ', '-').toLowerCase()}`}>{route.status}</td>
          <td>{route.backing.join(' + ')}</td><td>{route.summary}</td></tr>)}</tbody>
      </table></div>
    </section>
  </article>;
}
