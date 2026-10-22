import { describe, expect, it } from 'vitest';
import { DEFAULT_LAB_SETTINGS, generateScenario } from '../mission/labScenarios';
import { fitCircle } from '../mission/labAlgorithms';

describe('skidpad physical world', () => {
  it('has a symmetric swept corridor, straight entry/exit, and a clear junction', () => {
    const settings = DEFAULT_LAB_SETTINGS;
    const world = generateScenario('SKIDPAD', settings);
    const physical = world.cones.filter((cone) => cone.class !== 'ORANGE');
    const bins = ['BLUE', 'YELLOW'].map((color) => [6.5, 9.5].map((radius) => physical.filter((cone) => cone.x_m < -4 && cone.class === color && Math.abs(Math.hypot(cone.x_m + 8, cone.y_m) - radius) < 0.45).length));
    const fit = ['BLUE', 'YELLOW'].map((color) => fitCircle(physical.filter((cone) => cone.x_m < -4 && cone.class === color && Math.abs(Math.hypot(cone.x_m + 8, cone.y_m) - (color === 'BLUE' ? 6.5 : 9.5)) < 0.45)));
    expect(bins[0][0]).toBeGreaterThan(20);
    expect(bins[1][1]).toBeGreaterThan(20);
    expect(fit[0]?.center.x_m).toBeCloseTo(-settings.skidpadRadius, 1);
    expect(fit[1]?.center.x_m).toBeCloseTo(-settings.skidpadRadius, 1);
    expect(world.start.x_m).toBe(0);
    expect(world.start.y_m).toBeLessThan(-settings.entryLength);
    expect(world.cones.filter((cone) => cone.class === 'ORANGE')).toHaveLength(4);
    expect(physical.length).toBeGreaterThan(150);
    expect(physical.every((cone) => Math.hypot(cone.x_m, cone.y_m) >= settings.junctionLength)).toBe(true);
    const extents = [Math.min(...physical.map((cone) => cone.x_m)), Math.max(...physical.map((cone) => cone.x_m))];
    expect(Math.abs(extents[0] + extents[1])).toBeLessThan(0.5);
    for (const cone of physical) {
      const mirror = physical.some((other) => Math.hypot(other.x_m + cone.x_m, other.y_m - cone.y_m) < 0.4);
      expect(mirror).toBe(true);
    }
    let minDistance = Infinity;
    for (let i = 0; i < world.cones.length; i++) for (let j = i + 1; j < world.cones.length; j++) minDistance = Math.min(minDistance, Math.hypot(world.cones[i].x_m - world.cones[j].x_m, world.cones[i].y_m - world.cones[j].y_m));
    expect(minDistance).toBeGreaterThan(0.5);
  });
});
