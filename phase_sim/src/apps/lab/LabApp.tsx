import { Suspense, lazy, useEffect, useRef, useState, type ReactNode } from 'react';
import { Link, usePath } from './router';
import { FOREIGN_PATHS, LAB_ROUTES, ROUTE_GROUPS, findRoute, type LabRoute } from './routes';
import { LIVE_READ_ONLY_REASON, RunProvider, useRun, type Provenance } from './runContext';
import { expectCore, expectSimulationGateway, useServiceStatus, type ServiceState, type Services } from './services';
import { HomePage } from './pages/HomePage';
import { CorePage } from './pages/CorePage';
import { CoreStagePage } from './pages/CoreStagePage';
import { DatasetPage } from './pages/DatasetPage';
import { ScenarioPage } from './pages/ScenarioPage';
import { LogsPage } from './pages/LogsPage';
import { NotImplementedPage, NOT_IMPLEMENTED } from './pages/NotImplementedPage';
import { ThemeToggle } from '../../ui/ThemeToggle';
import '../../ui/tokens.css';
import './lab.css';

// three.js and the client-side demo engine are only fetched when their page is opened.
const MissionPage = lazy(() => import('./pages/MissionPage').then((module) => ({ default: module.MissionPage })));
const DemoPage = lazy(() => import('./pages/DemoPage').then((module) => ({ default: module.DemoPage })));
const MyTrackPage = lazy(() => import('./pages/MyTrackPage').then((module) => ({ default: module.MyTrackPage })));



const PROVENANCE_LABELS: Provenance[] = ['SYNTHETIC', 'REPLAY', 'LIVE READ-ONLY'];

function serviceText(name: string, state: ServiceState) {
  if (state.state === 'CHECKING') return `${name} · CHECKING`;
  if (state.state === 'REACHABLE') return `${name} · REACHABLE`;
  if (state.state === 'REFUSED') return `${name} · REFUSED`;
  return `${name} · OFFLINE`;
}

function ProvenanceBanner() {
  const { pageProvenance } = useRun();
  const active = pageProvenance?.label ?? null;
  return <section className="lab-provenance" data-testid="provenance-banner" aria-label="Data provenance">
    <span className="lab-provenance-title">PROVENANCE</span>
    <ul className="lab-provenance-chips">
      {PROVENANCE_LABELS.map((label) => {
        const disabled = label === 'LIVE READ-ONLY';
        return <li key={label} className={`lab-chip ${active === label ? 'active' : ''} ${disabled ? 'disabled' : ''}`}
          data-provenance={label} aria-current={active === label ? 'true' : undefined} aria-disabled={disabled ? 'true' : undefined}
          title={disabled ? LIVE_READ_ONLY_REASON : undefined}>
          {label}{disabled && <small> · UNAVAILABLE</small>}
        </li>;
      })}
    </ul>
    <p className="lab-provenance-detail" data-testid="provenance-active">
      {pageProvenance ? <><b>ACTIVE: {pageProvenance.label}</b> — {pageProvenance.detail}</> : <><b>ACTIVE: NONE</b> — this page is not displaying algorithm or vehicle data</>}
    </p>
    <p className="lab-provenance-live" id="live-readonly-reason"><b>LIVE READ-ONLY:</b> {LIVE_READ_ONLY_REASON}</p>
  </section>;
}

/**
 * Grouped nav: one cluster per `route.group` (RUN / ALGORITHM CORE / MISSION & SIM / DATA /
 * QA & OPS, per routes.ts metadata). Routes marked `NOT IMPLEMENTED` (Perception,
 * Depth & Calib., Benchmark, Faults) are pulled out of their normal groups and collapsed into
 * one "Unavailable in this build" dropdown instead of each showing up as its own disabled
 * inline tab.
 */
function UnavailableMenu({ path, routes }: { path: string; routes: LabRoute[] }) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    const onDoc = (e: MouseEvent) => { if (ref.current && !ref.current.contains(e.target as Node)) setOpen(false); };
    document.addEventListener('mousedown', onDoc);
    return () => document.removeEventListener('mousedown', onDoc);
  }, [open]);
  const activeInside = routes.some((r) => r.path === path);
  return <div className="lab-nav-group lab-unavailable-menu" ref={ref}>
    <button type="button" className={`lab-group-label lab-unavailable-toggle ${activeInside ? 'active' : ''}`} aria-expanded={open} onClick={() => setOpen((o) => !o)}>
      Unavailable in this build <small className="lab-tag">{routes.length}</small>
    </button>
    {open && <ul className="lab-pages-strip lab-unavailable-dropdown" role="menu">
      {routes.map((route) => <li key={route.path}>
        <Link to={route.path} aria-current={route.path === path ? 'page' : undefined} className="lab-page-link status-not-implemented"
          title={`${route.summary} (not implemented)`} onClick={() => setOpen(false)}>
          {route.label}<small className="lab-tag">N/A</small>
        </Link>
      </li>)}
    </ul>}
  </div>;
}

function LabNavBar({ path }: { path: string }) {
  const unavailable = LAB_ROUTES.filter((r) => r.status === 'NOT IMPLEMENTED');
  return <nav className="lab-pages" aria-label="Lab pages">
    <div className="lab-pages-groups">
      {ROUTE_GROUPS.map((group) => {
        const routes = LAB_ROUTES.filter((r) => r.group === group && r.status !== 'NOT IMPLEMENTED');
        if (!routes.length) return null;
        return <div key={group} className="lab-nav-group">
          <span className="lab-group-label" aria-hidden="true">{group}</span>
          <ul className="lab-pages-strip">
            {routes.map((route) => <li key={route.path}>
              <Link to={route.path} aria-current={route.path === path ? 'page' : undefined} className={`lab-page-link status-${route.status.replace(' ', '-').toLowerCase()}`}
                title={`${route.summary} (${route.status.toLowerCase()})`}>
                {route.label}{route.status === 'PARTIAL' && <small className="lab-tag partial">PARTIAL</small>}
              </Link>
            </li>)}
          </ul>
        </div>;
      })}
      {unavailable.length > 0 && <UnavailableMenu path={path} routes={unavailable} />}
    </div>
  </nav>;
}

/**
 * Small colored dot + title tooltip, used for the CORE/GATEWAY reachability and PROVENANCE-mode
 * indicators. The full label is also in a visually-hidden span (not just `title`) so it's still
 * in the DOM's text content for assistive tech and the tests that pin exact service-status text.
 */
function Dot({ tone, label }: { tone: 'ok' | 'warn' | 'bad' | 'idle'; label: string }) {
  return <span className={`lab-dot lab-dot-${tone}`} title={label}>
    <span aria-hidden="true">●</span><span className="lab-visually-hidden">{label}</span>
  </span>;
}

function serviceTone(state: ServiceState): 'ok' | 'warn' | 'bad' | 'idle' {
  if (state.state === 'REACHABLE') return 'ok';
  if (state.state === 'CHECKING') return 'idle';
  if (state.state === 'REFUSED') return 'bad';
  return 'warn';
}

function LabHeader({ path, route, services }: { path: string; route: LabRoute | undefined; services: Services }) {
  const { pageProvenance } = useRun();
  const provenanceLabel = pageProvenance?.label ?? 'NONE';
  return <header className="lab-header">
    <div className="lab-bar">
      <Link to="/home" className="lab-brand-mini" aria-label="AutoKart Algorithm Lab home"><span aria-hidden="true">AK</span><b className="lab-visually-hidden">AUTOKART ALGORITHM LAB</b></Link>
      <span className="lab-bar-title">{route ? route.label : 'AutoKart Algorithm Lab'}</span>
      <span className="lab-bar-dots" aria-label="Status">
        <Dot tone={serviceTone(services.core)} label={serviceText('CORE :8767', services.core)} />
        <Dot tone={serviceTone(services.simGateway)} label={serviceText('SIM ROS GATEWAY :8775', services.simGateway)} />
        <Dot tone={provenanceLabel === 'NONE' ? 'idle' : 'ok'} label={`PROVENANCE · ACTIVE: ${provenanceLabel}${pageProvenance ? ` — ${pageProvenance.detail}` : ''}`} />
      </span>
      <LabNavBar path={path} />
      <ThemeToggle />
      <span className="lab-authority-badge" title="VEHICLE AUTHORITY OFF · SIM-ONLY OUTPUT" aria-label="VEHICLE AUTHORITY OFF · SIM-ONLY OUTPUT">AUTH OFF</span>
    </div>
    {/* Full provenance detail kept in the DOM for assistive tech / tests; visually collapsed by default
        now that the dot above carries the at-a-glance state. Expand via the native <details> toggle. */}
    <details className="lab-provenance-details">
      <summary className="lab-provenance-summary">Provenance detail</summary>
      <ProvenanceBanner />
    </details>
  </header>;
}

function NotFoundPage({ path }: { path: string }) {
  const foreign = FOREIGN_PATHS[path];
  return <article className="lab-page lab-doc" data-page="not-found">
    <h1>{foreign ? 'Not part of the Algorithm Lab' : 'No such Lab page'}</h1>
    {foreign
      ? <p><code>{path}</code> belongs to {foreign}, a separate application on its own port. The Lab has no route to live services and does not link into them.</p>
      : <p>There is no Lab page at <code>{path}</code>.</p>}
    <p><Link to="/home">Go to Lab Home</Link></p>
  </article>;
}

function PageFor({ route, path, services }: { route: LabRoute | undefined; path: string; services: Services }): ReactNode {
  if (!route) return <NotFoundPage path={path} />;
  switch (route.path) {
    case '/home': return <HomePage services={services} />;
    case '/core': return <CorePage />;
    case '/delaunay': return <CoreStagePage stage="delaunay" />;
    case '/gabriel': return <CoreStagePage stage="gabriel" />;
    case '/gates': return <CoreStagePage stage="gates" />;
    case '/paths': return <CoreStagePage stage="paths" />;
    case '/mission': return <MissionPage variant="mission" />;
    case '/replay': return <MissionPage variant="replay" />;
    case '/mytrack': return <MyTrackPage />;
    case '/demo': return <DemoPage />;
    case '/dataset': return <DatasetPage />;
    case '/scenario': return <ScenarioPage />;
    case '/logs': return <LogsPage services={services} />;
    default: {
      const spec = NOT_IMPLEMENTED[route.path];
      return spec ? <NotImplementedPage route={route} spec={spec} /> : <NotFoundPage path={path} />;
    }
  }
}

function LabShell() {
  const path = usePath();
  const route = findRoute(path);
  const core = useServiceStatus('/api/core/status', 5000, expectCore);
  const simGateway = useServiceStatus('/mission/api/status', 5000, expectSimulationGateway);
  const services = { core, simGateway };
  useEffect(() => { document.title = route ? `${route.label} · AutoKart Algorithm Lab` : 'AutoKart Algorithm Lab'; }, [route]);
  const fill = path === '/mission' || path === '/replay';
  return <div className={`lab-app ${fill ? 'lab-fill' : ''}`} data-route={path}>
    <a className="lab-skip" href="#lab-main">Skip to page content</a>
    <LabHeader path={path} route={route} services={services} />
    <main id="lab-main" className="lab-main" tabIndex={-1}>
      <Suspense fallback={<p className="lab-loading" role="status">Loading page…</p>}>
        <PageFor key={path} route={route} path={path} services={services} />
      </Suspense>
    </main>
  </div>;
}

export function LabApp() {
  return <RunProvider><LabShell /></RunProvider>;
}
