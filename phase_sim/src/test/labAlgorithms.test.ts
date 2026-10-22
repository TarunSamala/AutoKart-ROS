import { describe, expect, it } from 'vitest';
import { delaunayFromCones, fitCircle, gabrielFilter, observeCones } from '../mission/labAlgorithms';
import { DEFAULT_LAB_SETTINGS, generateScenario } from '../mission/labScenarios';
import { LabSkidpad } from '../mission/LabSkidpad';
import type { Cone } from '../mission/schema';

const cone = (id: string, x_m: number, y_m: number): Cone => ({ id, class: 'BLUE', confidence: 1, x_m, y_m });

describe('observed geometry and scenario gates', () => {
  it('only triangulates cones inside the moving FOV', () => {
    const truth = [cone('ahead', 4, 0), cone('side', 0, 4), cone('behind', -4, 0), cone('ahead2', 5, 1), cone('ahead3', 5, -1)];
    const visible = observeCones(truth, { x_m: 0, y_m: 0, yaw_rad: 0 }, 6, Math.PI / 2);
    expect(visible.map((item) => item.id)).toEqual(['ahead', 'ahead2', 'ahead3']);
    const mesh = delaunayFromCones(visible);
    expect(mesh.triangles).toHaveLength(1);
    expect(mesh.edges).toHaveLength(3);
    expect(mesh.edges.every((edge) => visible.some((item) => item.id === edge.a) && visible.some((item) => item.id === edge.b))).toBe(true);
  });

  it('accepts an empty diameter disk and identifies a blocking cone', () => {
    const a = cone('A', -2, 0), b = cone('B', 2, 0), outside = cone('C', 0, 2.01), inside = cone('D', 0, 1.99);
    const edge = [{ id: 'E1', a: 'A', b: 'B' }];
    expect(gabrielFilter(edge, [a, b, outside])[0]).toMatchObject({ accepted: true, radius_m: 2 });
    expect(gabrielFilter(edge, [a, b, inside])[0]).toMatchObject({ accepted: false, blocking_cone_id: 'D', radius_m: 2 });
    expect(gabrielFilter(edge, [a, b, cone('boundary', 0, 2)])[0].accepted).toBe(true);
  });

  it('fits a loop center and radius from observed positions', () => {
    const points = Array.from({ length: 12 }, (_, i) => ({ x_m: 8 + 6.5 * Math.cos(i * Math.PI / 6), y_m: 6.5 * Math.sin(i * Math.PI / 6) }));
    expect(fitCircle(points)?.center.x_m).toBeCloseTo(8, 5);
    expect(fitCircle(points)?.center.y_m).toBeCloseTo(0, 5);
    expect(fitCircle(points)?.radius_m).toBeCloseTo(6.5, 5);
    expect(fitCircle(points.slice(0, 2))).toBeNull();
  });

  it('generates seeded autocross maps and a brake obstacle', () => {
    const a = generateScenario('AUTOCROSS', DEFAULT_LAB_SETTINGS);
    const b = generateScenario('AUTOCROSS', DEFAULT_LAB_SETTINGS);
    const c = generateScenario('AUTOCROSS', { ...DEFAULT_LAB_SETTINGS, seed: 99 });
    expect(a).toEqual(b);
    expect(a.guide.length).toBeGreaterThan(350);
    expect(a.cones.length).toBeGreaterThan(250);
    expect(c.guide).not.toEqual(a.guide);
    expect(generateScenario('BRAKE').obstacle?.x_m).toBeGreaterThan(30);
  });

  it('brake override stops before the detected obstacle', () => {
    const lab = new LabSkidpad('BRAKE');
    lab.setRunning(true);
    let sawObstacle = false, sawOverride = false;
    for (let i = 0; i <= 2400; i++) {
      lab.advanceFrame(i * 1000 / 60);
      if (i % 30 === 0) { const frame = lab.getFrame(); sawObstacle ||= Boolean(frame.perception.obstacles?.length); sawOverride ||= frame.safety.state === 'BRAKE OVERRIDE'; }
    }
    const frame = lab.getFrame();
    expect(sawObstacle).toBe(true);
    expect(sawOverride).toBe(true);
    expect(frame.vehicle.speed_mps).toBe(0);
    expect(frame.vehicle.x_m).toBeLessThan(35.6);
  });

  it('drives beyond the autocross straight into its later features', () => {
    const lab = new LabSkidpad('AUTOCROSS');
    lab.setRunning(true);
    let maxX = -Infinity;
    for (let i = 0; i <= 6000; i++) {
      lab.advanceFrame(i * 1000 / 60);
      if (i % 60 === 0) maxX = Math.max(maxX, lab.getFrame().vehicle.x_m);
    }
    expect(maxX).toBeGreaterThan(130);
    expect(lab.getFrame().vehicle.y_m).toBeGreaterThan(15);
  }, 15000);
});
