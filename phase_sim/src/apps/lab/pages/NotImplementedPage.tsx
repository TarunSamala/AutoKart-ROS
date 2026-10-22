import { Link } from '../router';
import type { LabRoute } from '../routes';
import { useReportProvenance } from '../runContext';

export type NotImplementedSpec = { missing: string[]; why: string; instead?: Array<{ to: string; label: string }> };

/** Pages with no backend or component in this build. They say so; they never simulate output. */
export const NOT_IMPLEMENTED: Record<string, NotImplementedSpec> = {
  '/perception': {
    missing: [
      'An offline inference service: the Lab has no endpoint that runs the cone detector (YOLO26s / ConeNet) on a recorded RGB frame.',
      'Recorded RGB frames: MissionFrame recordings carry cone lists, not camera images.',
      'Model checksum and selected inference path for the Lab domain.',
    ],
    why: 'Detector inference runs only inside the live perception service, which the Lab must never contact.',
    instead: [{ to: '/dataset', label: 'Inspect recorded cone lists in Dataset' }, { to: '/delaunay', label: 'Observations fed to shared_core in Delaunay' }],
  },
  '/depth-calibration': {
    missing: [
      'Depth frames (RAW16) and a depth probe: no depth data reaches the Lab.',
      'A calibration artifact API (intrinsics, extrinsics, survey residuals, verification state).',
      'An offline survey / fit workflow.',
    ],
    why: 'No calibration backend exists for the Lab. Showing nominal or synthetic values here could be mistaken for a verified calibration, so nothing is shown.',
  },
  '/benchmark': {
    missing: [
      'A benchmark harness that replays a fixed input set through shared_core and records FPS / p50 / p95 / p99 timing.',
      'A results store for pass/fail regression history and downloadable reports.',
    ],
    why: 'lab_api only evaluates one frame at a time and keeps no history. Per-frame processing time is visible on the Core Lab stage panel (OBSERVE → CORE PROCESSING), but no benchmark is computed.',
    instead: [{ to: '/core', label: 'Per-frame processing time in Core Lab' }],
  },
  '/faults': {
    missing: [
      'A lab-domain fault-injection API (delayed / dropped frames, missing depth, stale odometry, invalid TF, crashed node).',
      'A running Lab simulation ROS graph to inject into (the sim gateway on :8775 is read-only from the browser).',
    ],
    why: 'Injecting faults requires a backend that owns the Lab ROS domain. None exists in this build, and the browser must not fake fault effects.',
  },
};

export function NotImplementedPage({ route, spec }: { route: LabRoute; spec: NotImplementedSpec }) {
  useReportProvenance(null, '');
  return <article className="lab-page lab-doc lab-unavailable" data-page={route.path.slice(1)} aria-labelledby="unavailable-title">
    <p className="lab-eyebrow">{route.group} · {route.label}</p>
    <h1 id="unavailable-title">{route.label}: not implemented in this build</h1>
    <p className="lab-unavailable-state" data-testid="not-implemented"><b>NOT IMPLEMENTED IN THIS BUILD</b> — {route.summary}</p>
    <h2>What is missing</h2>
    <ul>{spec.missing.map((item) => <li key={item}>{item}</li>)}</ul>
    <h2>Why</h2>
    <p>{spec.why}</p>
    {spec.instead && <><h2>What you can use instead</h2><ul>{spec.instead.map((item) => <li key={item.to}><Link to={item.to}>{item.label}</Link></li>)}</ul></>}
  </article>;
}
