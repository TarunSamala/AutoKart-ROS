import './labNav.css';

/**
 * One navigation bar for the whole "Simulation / Algorithm Lab" family.
 *
 * The Lab is built on two independent backends plus one client-only sandbox;
 * this bar exists so a user can always tell which one they are looking at:
 *   /shared -> ALGORITHM LAB: the real Python shared_core (autokart_perception.lab_api, :8767)
 *   /       -> SANDBOX & ROS VIEWER: client-side JS skidpad sandbox (LabSkidpad.ts, no backend)
 *              plus read-only ROS gateway views (autokart_dashboard_gateway, :8765)
 * The production live dashboard (/live) is a separate application and is only
 * linked here, never imported. Nothing in this component can command a vehicle.
 */
export type LabSurface = 'algorithm' | 'sandbox';

export const LAB_SURFACES: ReadonlyArray<{ id: LabSurface; href: string; title: string; detail: string }> = [
  { id: 'algorithm', href: '/shared', title: 'ALGORITHM LAB', detail: 'Python shared_core · lab_api :8767' },
  { id: 'sandbox', href: '/', title: 'SANDBOX & ROS VIEWER', detail: 'JS skidpad sandbox · ROS gateway :8765 (read-only)' },
];

export function LabNav({ current }: { current: LabSurface }) {
  return <nav className="lab-nav" aria-label="Simulation and Algorithm Lab navigation">
    <span className="lab-nav-brand">AUTOKART <b>SIMULATION &amp; ALGORITHM LAB</b></span>
    {LAB_SURFACES.map((surface) => surface.id === current
      ? <span key={surface.id} className="lab-nav-item current" aria-current="page"><b>{surface.title}</b><small>{surface.detail}</small></span>
      : <a key={surface.id} className="lab-nav-item" href={surface.href}><b>{surface.title}</b><small>{surface.detail}</small></a>)}
    <span className="lab-nav-note">DEV / SIMULATION TOOL · not the live dashboard (<a href="/live">/live</a> is a separate app)</span>
    <span className="lab-nav-authority">VEHICLE AUTHORITY OFF</span>
  </nav>;
}
