/**
 * AutoKart Algorithm Lab route table.
 *
 * Every page declares what backs it. `backing` is shown in the nav and on the Home page so
 * nobody has to guess whether a page is real, partial, or not implemented in this build.
 *   CORE     -> Python autokart_perception.lab_api (:8767, proxied as /api/core/*)
 *   SANDBOX  -> client-side TypeScript (no backend)
 *   SIM_ROS  -> Lab-only simulation ROS gateway (:8775, proxied as /mission/*)
 *   REPLAY   -> a recording file opened in this browser tab
 *   NONE     -> not implemented in this build (page says exactly what is missing)
 */
export type Backing = 'CORE' | 'SANDBOX' | 'SIM_ROS' | 'REPLAY' | 'NONE';
export type RouteStatus = 'AVAILABLE' | 'PARTIAL' | 'NOT IMPLEMENTED';

export type LabRoute = {
  path: string;
  label: string;
  group: 'RUN' | 'ALGORITHM CORE' | 'MISSION & SIM' | 'DATA' | 'QA & OPS';
  status: RouteStatus;
  backing: Backing[];
  summary: string;
};

export const LAB_ROUTES: readonly LabRoute[] = [
  { path: '/home', label: 'Home', group: 'RUN', status: 'AVAILABLE', backing: ['SANDBOX', 'REPLAY'],
    summary: 'Run manager: choose SYNTHETIC or REPLAY, run id, seed and scenario config.' },
  { path: '/core', label: 'Core Lab', group: 'ALGORITHM CORE', status: 'AVAILABLE', backing: ['CORE'],
    summary: 'Python shared_core lab: start/pause/step/reset, drag cones, full stage ribbon.' },
  { path: '/delaunay', label: 'Delaunay', group: 'ALGORITHM CORE', status: 'AVAILABLE', backing: ['CORE'],
    summary: 'Observed vertices, triangles and edges from the current CORE evaluation.' },
  { path: '/gabriel', label: 'Gabriel', group: 'ALGORITHM CORE', status: 'AVAILABLE', backing: ['CORE'],
    summary: 'Edges kept / removed by the Python Gabriel filter, with a display-only geometric cross-check.' },
  { path: '/gates', label: 'Gates', group: 'ALGORITHM CORE', status: 'AVAILABLE', backing: ['CORE'],
    summary: 'Boundary assignment, selected gates and rejected candidate pairs with reasons.' },
  { path: '/paths', label: 'Paths', group: 'ALGORITHM CORE', status: 'PARTIAL', backing: ['CORE'],
    summary: 'Centerline, curvature profile and the sim-only controller candidate. No path variants.' },
  { path: '/mission', label: 'Mission', group: 'MISSION & SIM', status: 'AVAILABLE', backing: ['SANDBOX', 'SIM_ROS', 'REPLAY'],
    summary: '3D mission view with the SEE → … → CONTROL stage ribbon. Sandbox, ROS simulation or replay only.' },
  { path: '/mytrack', label: 'MY_TRACK', group: 'MISSION & SIM', status: 'AVAILABLE', backing: ['SANDBOX'],
    summary: 'MY_TRACK instrument panel: top-down overview, follow view, synthetic HP60C frame, vehicle/controller telemetry, deviation.' },
  { path: '/scenario', label: 'Scenario', group: 'MISSION & SIM', status: 'PARTIAL', backing: ['SANDBOX'],
    summary: 'Seeded course generator with fixture import/export. No free-hand cone/obstacle editing.' },
  { path: '/demo', label: 'Demo', group: 'MISSION & SIM', status: 'AVAILABLE', backing: ['SANDBOX'],
    summary: 'Client-side Delaunay kart workbench (TypeScript engine, no backend).' },
  { path: '/dataset', label: 'Dataset', group: 'DATA', status: 'PARTIAL', backing: ['REPLAY'],
    summary: 'Recording inspector for MissionFrame .jsonl files. No camera images in this build.' },
  { path: '/replay', label: 'Replay', group: 'DATA', status: 'AVAILABLE', backing: ['REPLAY'],
    summary: 'Play / pause / step / seek a recording through the mission view.' },
  { path: '/perception', label: 'Perception', group: 'DATA', status: 'NOT IMPLEMENTED', backing: ['NONE'],
    summary: 'Raw frame + detector inspection.' },
  { path: '/depth-calibration', label: 'Depth & Calib.', group: 'DATA', status: 'NOT IMPLEMENTED', backing: ['NONE'],
    summary: 'Depth probe, intrinsics/extrinsics and calibration residuals.' },
  { path: '/benchmark', label: 'Benchmark', group: 'QA & OPS', status: 'NOT IMPLEMENTED', backing: ['NONE'],
    summary: 'Pipeline timing and regression history.' },
  { path: '/faults', label: 'Faults', group: 'QA & OPS', status: 'NOT IMPLEMENTED', backing: ['NONE'],
    summary: 'Lab-domain fault injection.' },
  { path: '/logs', label: 'Logs & Config', group: 'QA & OPS', status: 'PARTIAL', backing: ['CORE', 'SIM_ROS'],
    summary: 'Run configuration, software/config hashes and service reachability. No trace log store.' },
];

export const ROUTE_GROUPS = ['RUN', 'ALGORITHM CORE', 'MISSION & SIM', 'DATA', 'QA & OPS'] as const;

/** Paths that exist in other AutoKart apps. The Lab never serves or proxies them. */
export const FOREIGN_PATHS: Record<string, string> = {
  '/live': 'the Vehicle Dashboard (live camera/perception)',
  '/ops': 'the Vehicle Dashboard (live mission view)',
  '/dashboard': 'the Vehicle Dashboard',
};

/** Legacy / convenience aliases that redirect (History replaceState) to a canonical Lab path. */
export const ALIASES: Record<string, string> = {
  '/': '/home',
  '/lab': '/home',
  '/shared': '/core',
  '/algorithm': '/core',
};

export function normalizePath(pathname: string): string {
  const trimmed = pathname.length > 1 ? pathname.replace(/\/+$/, '') : pathname;
  const lower = trimmed.toLowerCase();
  return ALIASES[lower] ?? lower;
}

export function findRoute(path: string): LabRoute | undefined {
  return LAB_ROUTES.find((route) => route.path === path);
}
