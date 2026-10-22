import { Suspense, lazy, useEffect, useState, type ReactNode } from 'react';
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
import './lab.css';

// three.js and the client-side demo engine are only fetched when their page is opened.
const MissionPage = lazy(() => import('./pages/MissionPage').then((module) => ({ default: module.MissionPage })));
const DemoPage = lazy(() => import('./pages/DemoPage').then((module) => ({ default: module.DemoPage })));



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

function LabNavBar({ path }: { path: string }) {
  const [open, setOpen] = useState(false);
  useEffect(() => setOpen(false), [path]);
  return <nav className={`lab-pages ${open ? 'open' : ''}`} aria-label="Lab pages">
    <button className="lab-pages-toggle" aria-expanded={open} aria-controls="lab-pages-list" onClick={() => setOpen(!open)}>
      {open ? 'Close pages' : 'Pages'} <span aria-hidden="true">{open ? '▴' : '▾'}</span>
    </button>
    <div id="lab-pages-list" className="lab-pages-groups">
      {ROUTE_GROUPS.map((group) => <div key={group} className="lab-pages-group" role="group" aria-label={group}>
        <span className="lab-pages-group-label">{group}</span>
        <ul>{LAB_ROUTES.filter((route) => route.group === group).map((route) => <li key={route.path}>
          <Link to={route.path} aria-current={route.path === path ? 'page' : undefined} className={`lab-page-link status-${route.status.replace(' ', '-').toLowerCase()}`}
            title={`${route.summary} (${route.status.toLowerCase()})`}>
            {route.label}{route.status === 'NOT IMPLEMENTED' && <small className="lab-tag">N/A</small>}{route.status === 'PARTIAL' && <small className="lab-tag partial">PARTIAL</small>}
          </Link>
        </li>)}</ul>
      </div>)}
    </div>
  </nav>;
}

function LabHeader({ path, services }: { path: string; services: Services }) {
  return <header className="lab-header">
    <div className="lab-header-row">
      <Link to="/home" className="lab-brand" aria-label="AutoKart Algorithm Lab home"><span aria-hidden="true">AK</span><b>AUTOKART ALGORITHM LAB</b></Link>
      <ul className="lab-services" aria-label="Lab backend reachability">
        <li className={`svc-${services.core.state.toLowerCase()}`} title={'detail' in services.core ? services.core.detail : undefined}>{serviceText('CORE :8767', services.core)}</li>
        <li className={`svc-${services.simGateway.state.toLowerCase()}`} title={'detail' in services.simGateway ? services.simGateway.detail : undefined}>{serviceText('SIM ROS GATEWAY :8775', services.simGateway)}</li>
      </ul>
      <span className="lab-authority">VEHICLE AUTHORITY OFF · SIM-ONLY OUTPUT</span>
    </div>
    <ProvenanceBanner />
    <LabNavBar path={path} />
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
    <LabHeader path={path} services={services} />
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
