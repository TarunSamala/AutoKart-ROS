import { describe, expect, it } from 'vitest';
import { MY_TRACK_LAYOUT, myTrackScenario, validateMyTrackLayout } from '../mission/myTrackLayout';
import { generateScenario } from '../mission/labScenarios';

describe('myTrackLayout', () => {
  it('loads 120 cones in the 80/20/20 orange/blue/yellow split, mirroring test_my_track_layout.py', () => {
    expect(MY_TRACK_LAYOUT.rev).toBe(7);
    expect(MY_TRACK_LAYOUT.corridor_m).toBe(3.0);
    expect(MY_TRACK_LAYOUT.length_m).toBe(50.0);
    expect(MY_TRACK_LAYOUT.spacing_m).toBe(1.0);
    expect(MY_TRACK_LAYOUT.cones).toHaveLength(120);
    const counts: Record<string, number> = { BLUE: 0, YELLOW: 0, ORANGE: 0 };
    for (const cone of MY_TRACK_LAYOUT.cones) counts[cone.color]++;
    expect(counts).toEqual({ BLUE: 20, YELLOW: 20, ORANGE: 80 });
  });

  it('does not throw on the real layout', () => {
    expect(() => validateMyTrackLayout()).not.toThrow();
  });

  it('rejects a layout with the wrong cone count', () => {
    const bad = { ...MY_TRACK_LAYOUT, cones: MY_TRACK_LAYOUT.cones.slice(1) };
    expect(() => validateMyTrackLayout(bad)).toThrow(/120 cones/);
  });

  it('builds a LabScenario with all 120 cones and a start pose at the origin', () => {
    const scenario = myTrackScenario();
    expect(scenario.name).toBe('MY_TRACK');
    expect(scenario.cones).toHaveLength(120);
    expect(scenario.start.x_m).toBe(0);
    expect(scenario.start.y_m).toBe(0);
    expect(scenario.start.heading).toBe(0);
  });

  it('is registered and reachable via generateScenario', () => {
    const scenario = generateScenario('MY_TRACK');
    expect(scenario.name).toBe('MY_TRACK');
    expect(scenario.cones.length).toBe(120);
  });

  it('marks the four finish-line cones as markers, not plain boundary cones', () => {
    const scenario = myTrackScenario();
    expect(scenario.markers).toHaveLength(4);
    expect(scenario.markers.every((m) => m.label === 'FINISH')).toBe(true);
  });
});
