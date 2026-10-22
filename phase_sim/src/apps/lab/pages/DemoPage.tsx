import { App } from '../../../App';
import { useReportProvenance } from '../runContext';

/** /demo: the existing client-side Delaunay kart workbench (App), reused unchanged. */
export function DemoPage() {
  useReportProvenance('SYNTHETIC', 'client-side TypeScript engine (src/engine) · no backend · sim-only controller');
  return <div className="lab-page lab-demo" data-page="demo">
    <h1 className="lab-visually-hidden">Client-side simulation demo</h1>
    <p className="lab-note">Client-side workbench: its own TypeScript engine, not the Python shared_core. Desktop layout (scroll sideways on narrow screens).</p>
    <div className="lab-demo-frame"><App /></div>
  </div>;
}
