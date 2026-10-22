import { useEffect, useState, type MouseEvent, type ReactNode } from 'react';
import { DashboardProvider, useDash } from './data';
import { deriveHealth } from './derive';
import { Chip } from './ui';
import { AboutPage, CameraPage, EventsPage, LocalizationPage, MissionPage, Overview, PerceptionPage, RosPage, SafetyPage, VehiclePage } from './pages';
import './dashboard.css';

export const ROUTES: { path: string; label: string; page: () => ReactNode }[] = [
  { path: '/overview', label: 'Overview', page: Overview }, { path: '/camera', label: 'Camera', page: CameraPage },
  { path: '/perception', label: 'Perception', page: PerceptionPage }, { path: '/localization', label: 'Localization', page: LocalizationPage },
  { path: '/mission', label: 'Mission', page: MissionPage }, { path: '/vehicle', label: 'Vehicle', page: VehiclePage },
  { path: '/safety', label: 'Safety', page: SafetyPage }, { path: '/ros', label: 'ROS', page: RosPage },
  { path: '/events', label: 'Events', page: EventsPage }, { path: '/about', label: 'About', page: AboutPage },
];

const normalise = (p: string) => (p.length > 1 ? p.replace(/\/+$/, '') : p);

function usePath() {
  const [path, setPath] = useState(() => {
    const p = normalise(window.location.pathname);
    if (p === '/' || p === '') { window.history.replaceState(null, '', '/overview'); return '/overview'; }
    return p;
  });
  useEffect(() => {
    const on = () => setPath(normalise(window.location.pathname) || '/overview');
    window.addEventListener('popstate', on);
    return () => window.removeEventListener('popstate', on);
  }, []);
  const go = (to: string) => { if (to !== path) { window.history.pushState(null, '', to); setPath(to); } };
  return [path, go] as const;
}

function Shell({ path, go }: { path: string; go: (p: string) => void }) {
  const d = useDash(); const h = deriveHealth(d);
  const [requestResult, setRequestResult] = useState('');
  const sendSafetyRequest = (action: 'arm' | 'stop' | 'reset_fault' | 'confirm_manual_power_cut') => {
    const status = d.gw.status;
    if (status?.source !== 'REAL_KART' || status.mode !== 'LIVE' || status.safety_requests_enabled !== true) {
      setRequestResult('Request unavailable: live safety request forwarding disabled'); return;
    }
    const socket = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/mission/ws`);
    socket.onopen = () => { socket.send(JSON.stringify({ type: 'request', action })); setRequestResult(`${action} requested; awaiting supervisor decision`); window.setTimeout(() => socket.close(), 1000); };
    socket.onerror = () => { setRequestResult('Request failed: gateway connection unavailable'); socket.close(); };
    socket.onmessage = (event) => { try { if (JSON.parse(event.data).type === 'request_rejected') setRequestResult(`Request rejected: ${action}`); } catch { /* ignore telemetry */ } };
  };
  const route = ROUTES.find(r => r.path === path);
  const Page = route?.page;
  const onNav = (to: string) => (e: MouseEvent) => { if (e.metaKey || e.ctrlKey || e.shiftKey || e.button !== 0) return; e.preventDefault(); go(to); };
  return <div className="shell">
    <header className="top">
      <div className="brand"><h1>AUTOKART VEHICLE DASHBOARD</h1>
        <span className="chip tone-info" data-testid="real-monitor-banner">REAL MONITOR</span>
        <Chip tone="info">DISPLAY + BOUNDED SAFETY REQUESTS · VEHICLE AUTHORITY OFF</Chip></div>
      <nav aria-label="Pages">{ROUTES.map(r => <a key={r.path} href={r.path} onClick={onNav(r.path)} aria-current={r.path === path ? 'page' : undefined}>{r.label}</a>)}</nav>
    </header>
    {h.banners.map(b => <div key={b} className="banner-red" role="alert" data-testid="source-banner">{b}</div>)}
    <div className="estop-amber" role="status">E-STOP INPUT: NOT FITTED - MANUAL POWER CUT ONLY</div>
    <div className="safety-actions">
      <button type="button" onClick={() => sendSafetyRequest('confirm_manual_power_cut')}>Confirm physical power cut within reach for next ARM</button>
      {/* Only sends the whitelisted 'arm' request; autokart_vcu_safety alone decides whether every
          precondition holds, and may refuse it (surfaced below via request_rejected). This app
          computes no arm-allowed gate of its own. */}
      <button type="button" onClick={() => sendSafetyRequest('arm')}>Request ARM (refused unless every gate and the manual power cut hold)</button>
      <button type="button" onClick={() => sendSafetyRequest('reset_fault')}>RESET FAULT</button>
      <button type="button" className="software-stop" onClick={() => sendSafetyRequest('stop')}>SOFTWARE STOP - NOT AN EMERGENCY STOP</button>
      <span role="status">{requestResult}</span>
    </div>
    <div className="strip" aria-label="Connection status">
      {h.chips.map(c => <span key={c.key} className="strip-item" title={c.detail}><small>{c.label}</small><Chip tone={c.tone}>{c.state}</Chip></span>)}
      <span className="strip-item"><small>FRAME</small><b className="mono" data-testid="frame-counter">#{d.perc.frames}</b></span>
      <span className="strip-item"><small>FPS</small><b className="mono">{d.perc.fps != null && h.cam.fresh ? d.perc.fps.toFixed(1) : '— (NO DATA)'}</b></span>
    </div>
    <main id="main" tabIndex={-1}>
      <h2 className="page-title">{route?.label ?? 'Not found'}</h2>
      {Page ? <Page /> : <div className="card"><p>No page at <code>{path}</code>.</p><a href="/overview" onClick={onNav('/overview')}>Go to Overview</a></div>}
    </main>
  </div>;
}

export function App() {
  const [path, go] = usePath();
  const wantImages = path === '/overview' || path === '/camera';
  return <DashboardProvider wantImages={wantImages}><Shell path={path} go={go} /></DashboardProvider>;
}
