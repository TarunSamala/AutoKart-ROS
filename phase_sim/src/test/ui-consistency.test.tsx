import { act } from 'react';
import { createRoot } from 'react-dom/client';
import { describe, expect, it } from 'vitest';
import { AutonomyEngine } from '../engine/engine';
import { radToDeg } from '../engine/math';
import type { PlanningSnapshot } from '../engine/types';
import { App } from '../App';
import { LivePanel } from '../ui/LivePanel';
import { registerCleanup } from './testUtils';

function value(container: HTMLElement, testId: string): string {
  const element = container.querySelector(`[data-testid="${testId}"]`);
  if (!element) throw new Error(`Missing displayed value ${testId}`);
  return element.textContent ?? '';
}

async function renderSnapshot(snapshot: PlanningSnapshot) {
  const container = document.createElement('div');
  document.body.appendChild(container);
  const root = createRoot(container);
  registerCleanup(() => root.unmount());
  await act(async () => {
    root.render(<LivePanel snapshot={snapshot} selection={{ kind: 'kart', id: 'vehicle' }} />);
  });
  return container;
}

describe('PlanningSnapshot → displayed UI acceptance contract', () => {
  it('matches all required live values within their displayed formatting precision', async () => {
    const engine = new AutonomyEngine('oval');
    engine.setRunning(true);
    const start = performance.now();
    for (let frame = 0; frame < 100; frame += 1) engine.advanceFrame(start + frame * (1000 / 60));
    engine.setRunning(false);
    const snapshot = engine.getSnapshot();
    const container = await renderSnapshot(snapshot);

    expect(Number.parseFloat(value(container, 'steering-angle'))).toBeCloseTo(radToDeg(snapshot.controller.steeringAngle), 2);
    expect(Number.parseInt(value(container, 'visible-cone-count'), 10)).toBe(snapshot.sensor.visibleCones.length);
    expect(Number.parseInt(value(container, 'triangle-count'), 10)).toBe(snapshot.delaunay.triangles.length);
    expect(Number.parseInt(value(container, 'interior-count'), 10)).toBe(snapshot.filtering.interiorTriangles.length);
    expect(Number.parseInt(value(container, 'edge-count'), 10)).toBe(snapshot.edges.crossTrackEdges.length);
    expect(Number.parseInt(value(container, 'midpoint-count'), 10)).toBe(snapshot.midpoints.length);
    expect(Number.parseFloat(value(container, 'lookahead-distance'))).toBeCloseTo(snapshot.controller.lookaheadDistance, 3);
    expect(Number.parseFloat(value(container, 'curvature'))).toBeCloseTo(snapshot.controller.curvature, 3);
    expect(Number.parseFloat(value(container, 'vehicle-x'))).toBeCloseTo(snapshot.vehicle.x, 3);
    expect(Number.parseFloat(value(container, 'vehicle-y'))).toBeCloseTo(snapshot.vehicle.y, 3);
  });

  it('mounts the complete workbench with the engineering controls and views', async () => {
    window.requestAnimationFrame = () => 1;
    window.cancelAnimationFrame = () => undefined;
    const container = document.createElement('div');
    document.body.appendChild(container);
    const root = createRoot(container);
    registerCleanup(() => root.unmount());
    await act(async () => root.render(<App />));

    expect(container.textContent).toContain('LIVE AUTONOMY COMPUTATION');
    expect(container.textContent).toContain('STEP CYCLE');
    expect(container.textContent).toContain('WHY DID THE KART DO THIS?');
    expect(container.textContent).toContain('REMOVE EXTERIOR');
    expect(container.textContent).toContain('ROLLING ENGINEERING HISTORY');
    expect(window.__PLANNING_SNAPSHOT__).toBeDefined();
  });
});
