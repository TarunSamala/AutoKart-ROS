import { describe, expect, it } from 'vitest';
import { demoFrame, gabrielForDemo } from '../mission/demo';
import { frameAt, stageFrame, STAGES, type MissionFrame } from '../mission/schema';
import { MISSION_DEMO_PRESETS } from '../engine/course';
import { AutonomyEngine } from '../engine/engine';

describe('Mission Control geometry and timebase', () => {
  it('accepts an empty Gabriel diameter disk and identifies its blocker', () => {
    const edges = [{ id: 'e', a: 'a', b: 'b' }];
    const cones = [
      { id: 'a', class: 'BLUE', confidence: 1, x_m: -1, y_m: 0 },
      { id: 'b', class: 'YELLOW', confidence: 1, x_m: 1, y_m: 0 },
    ];
    expect(gabrielForDemo(edges, cones)[0].accepted).toBe(true);
    expect(gabrielForDemo(edges, [...cones, { id: 'c', class: 'BLUE', confidence: 1, x_m: 0, y_m: 0 }])[0]).toMatchObject({ accepted: false, blocking_cone_id: 'c' });
  });
  it('selects the same frame at the same timestamp and reveals step layers in order', () => {
    const frame = { schema_version: 1, timestamp_s: 2, sequence: 1, planning: { delaunay_edges: [{ id: 'e' }], gabriel_edges: [{ id: 'g' }], gates: [{ id: 'gate' }], candidate_paths: [{ id: 'path' }], selected_path: [{ x_m: 1, y_m: 2 }], lookahead: { x_m: 1, y_m: 2 } } } as unknown as MissionFrame;
    expect(frameAt([frame], 2)).toBe(frame);
    expect(frameAt([frame], 1.9)).toBeNull();
    expect(stageFrame(frame, 2).planning.delaunay_edges).toHaveLength(1);
    expect(stageFrame(frame, 2).planning.gabriel_edges).toHaveLength(0);
    expect(STAGES).toHaveLength(8);
    expect(stageFrame(frame, 4).planning.candidate_paths).toHaveLength(0);
    expect(stageFrame(frame, 5).planning.candidate_paths).toHaveLength(1);
    expect(stageFrame(frame, 6).planning.selected_path).toHaveLength(0);
    expect(stageFrame(frame, 7).planning.selected_path).toHaveLength(1);
    expect(stageFrame(frame, 7).planning.lookahead).toBeNull();
    expect(stageFrame(frame, 8).planning.lookahead).toEqual({ x_m: 1, y_m: 2 });
  });
  it('keeps the three larger DEMO courses connected to the existing engine', () => {
    expect(MISSION_DEMO_PRESETS.map((preset) => preset.id)).toEqual(['mission-skidpad', 'mission-autocross', 'mission-brake']);
    for (const preset of MISSION_DEMO_PRESETS) {
      const engine = new AutonomyEngine(preset.id);
      const initial = engine.getSnapshot();
      expect(initial.world.cones.length, preset.id).toBeGreaterThan(70);
      expect(initial.sensor.visibleCones.length, preset.id).toBeGreaterThanOrEqual(4);
      expect(initial.interpolation.path.length, preset.id).toBeGreaterThan(2);
      engine.setRunning(true);
      const start = performance.now();
      let brakeRequired = false;
      for (let frame = 0; frame <= 3600; frame += 1) {
        engine.advanceFrame(start + frame * (1000 / 60));
        if (engine.getSnapshot().avoidance.phase === 'BLOCKED') brakeRequired = true;
      }
      const after = engine.getSnapshot();
      expect(Math.hypot(after.vehicle.x - initial.vehicle.x, after.vehicle.y - initial.vehicle.y), preset.id).toBeGreaterThan(0.5);
      if (preset.id === 'mission-autocross') expect(after.controller.stopReason).toBeNull();
      if (preset.id === 'mission-brake') {
        expect(brakeRequired).toBe(true);
        expect(after.vehicle.velocity).toBeLessThan(0.05);
        expect(after.vehicle.x).toBeLessThan(preset.obstacles[0].x - preset.obstacles[0].radius);
        const display = demoFrame(after, preset.id);
        expect(display.safety.obstacle?.brake_trigger).toBe(true);
        expect(display.safety.obstacle?.ttc_s).toBeNull();
        expect(display.vehicle.brake_command).toBe(1);
      }
      const display = demoFrame(after, preset.id);
      expect(display.visualization?.course_cones?.some((cone) => cone.class === 'ORANGE')).toBe(true);
      expect(display.planning.skidpad).toBeUndefined();
    }
  });
});
