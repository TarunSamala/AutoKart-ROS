import type { Services } from '../services';
import { LAB_ALLOWED_PREFIXES } from '../services';
import { useReportProvenance, useRun } from '../runContext';

function stateText(state: Services['core']) {
  if (state.state === 'CHECKING') return 'checking…';
  const when = new Date(state.checkedAt).toISOString();
  return `${state.state.toLowerCase()} · ${state.detail} · checked ${when}`;
}

export function LogsPage({ services }: { services: Services }) {
  const { run, recording } = useRun();
  useReportProvenance(null, '');
  const core = services.core.state === 'REACHABLE' ? services.core.payload : null;
  const gateway = services.simGateway.state === 'REACHABLE' ? services.simGateway.payload : null;
  const config = { run_id: run.runId, mode: run.mode, scenario: run.scenario, settings: run.settings, recording: recording ? { name: recording.name, frames: recording.frames.length } : null,
    core: core ? { algorithm_version: core.algorithm_version, configuration_sha256: core.configuration_sha256 } : null, build_mode: import.meta.env.MODE, time_convention: 'ISO-8601 UTC' };
  return <article className="lab-page lab-doc" data-page="logs" aria-labelledby="logs-title">
    <p className="lab-eyebrow">QA & OPS</p>
    <h1 id="logs-title">Logs and configuration</h1>
    <section className="lab-card" aria-labelledby="logs-software">
      <h2 id="logs-software">Software and configuration</h2>
      <dl className="lab-facts lab-facts-wide">
        <div><dt>LAB BUILD MODE</dt><dd><code>{import.meta.env.MODE}</code></dd></div>
        <div><dt>CORE ALGORITHM VERSION</dt><dd><code>{String(core?.algorithm_version ?? '— (lab_api not reachable)')}</code></dd></div>
        <div><dt>CORE CONFIG SHA-256</dt><dd><code className="lab-wrap">{String(core?.configuration_sha256 ?? '—')}</code></dd></div>
        <div><dt>CORE STATUS</dt><dd>{stateText(services.core)}</dd></div>
        <div><dt>SIM ROS GATEWAY</dt><dd>{stateText(services.simGateway)}{gateway ? ` · source ${String(gateway.source ?? '—')} · ros_connected ${String(gateway.ros_connected ?? '—')}` : ''}</dd></div>
        <div><dt>DETECTOR MODEL</dt><dd>— (no detector runs in the Lab build)</dd></div>
        <div><dt>CALIBRATION ARTIFACT</dt><dd>— (no calibration backend in the Lab build)</dd></div>
      </dl>
    </section>
    <section className="lab-card" aria-labelledby="logs-network">
      <h2 id="logs-network">Network boundary</h2>
      <p>Lab code only calls same-origin <code>{LAB_ALLOWED_PREFIXES.join('</code> and <code>')}</code>. The Lab server proxies <code>/api/core</code> to 127.0.0.1:8767 (lab_api) and <code>/mission</code> to 127.0.0.1:8775 (simulation gateway, separate ROS domain). There is no route to the live gateway (:8765) or live perception (:8768).</p>
    </section>
    <section className="lab-card" aria-labelledby="logs-run">
      <h2 id="logs-run">Current run configuration</h2>
      <pre className="lab-pre" tabIndex={0} aria-label="Run configuration JSON">{JSON.stringify(config, null, 2)}</pre>
    </section>
    <section className="lab-card lab-unavailable-inline" aria-labelledby="logs-missing">
      <h2 id="logs-missing">Not implemented in this build</h2>
      <ul>
        <li>Trace IDs and a time-aligned event log: no log/trace store exists for the Lab.</li>
        <li>ROS topic list, QoS and node health for the Lab domain: the simulation gateway exposes only its status summary.</li>
        <li>Comparison reports between runs: there is no results store (see Benchmark).</li>
      </ul>
    </section>
  </article>;
}
