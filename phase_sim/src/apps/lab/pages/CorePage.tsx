import { SharedLab } from '../../../shared/SharedLab';
import { useReportProvenance } from '../runContext';

/**
 * /core: the existing, browser-verified Python-CORE lab (SharedLab), reused unchanged.
 * Its own legacy LabNav (links to the root app's /shared, / and /live) is hidden by lab.css
 * because the Lab shell's navigation replaces it; nothing else is altered.
 */
export function CorePage() {
  useReportProvenance('SYNTHETIC', 'Python shared_core via lab_api :8767 · kart_sim.PlanarBicycleKart · controller output sim-only');
  return <div className="lab-page lab-core" data-page="core"><h1 className="lab-visually-hidden">Core Lab</h1><SharedLab /></div>;
}
