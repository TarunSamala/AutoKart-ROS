import { describe, expect, it } from 'vitest';
import { LabSkidpad } from '../mission/LabSkidpad';

describe('LAB common frame', () => {
  it('starts with a complete stationary course in the common frame', () => {
    const lab = new LabSkidpad();
    const frame = lab.getFrame();
    expect(frame.source).toBe('LAB');
    expect(frame.timestamp_s).toBe(0);
    expect(frame.vehicle.speed_mps).toBe(0);
    expect(frame.vehicle.requested_steering_rad).toBeNull();
    expect(frame.vehicle.target_speed_mps).toBeNull();
    expect(frame.visualization?.course_cones?.length).toBeGreaterThan(190);
    expect(frame.visualization?.course_cones?.filter((cone) => cone.class === 'ORANGE')).toHaveLength(4);
    expect(frame.visualization?.course_guide).toBeUndefined();
    expect(frame.perception.cones).toHaveLength(0);
    expect(frame.planning.delaunay_edges).toHaveLength(0);
    expect(frame.planning.gabriel_edges).toHaveLength(0);
    expect(frame.planning.selected_path).toHaveLength(0);
  });

  it('uses kinematic motion to traverse both lobes, pauses, and resets', () => {
    const lab = new LabSkidpad();
    const start = lab.getFrame();
    const positions = [];
    let verifiedBranch = false;
    lab.setRunning(true);
    for (let i = 0; i <= 9600; i += 1) {
      lab.advanceFrame(i * 1000 / 60);
      if (i % 60 === 0) {
        const frame = lab.getFrame(); const vehicle = frame.vehicle;
        positions.push(vehicle.x_m);
        for (const branch of frame.planning.skidpad?.branches ?? []) {
          const target = branch.points.at(-1)!;
          expect(frame.planning.gates.some((gate) => Math.hypot(gate.midpoint.x_m - target.x_m, gate.midpoint.y_m - target.y_m) < 1e-6)).toBe(true);
          verifiedBranch = true;
        }
      }
    }
    const moving = lab.getFrame();
    expect(moving.planning.skidpad?.state).toBe('STOP');
    expect(moving.planning.skidpad?.w_right).toBeLessThan(-1.9);
    expect(moving.planning.skidpad?.w_left).toBeGreaterThan(1.9);
    expect(moving.planning.skidpad?.left_center?.x_m).toBeCloseTo(-8, 0);
    expect(moving.planning.skidpad?.right_center?.x_m).toBeCloseTo(8, 0);
    expect(moving.timestamp_s).toBeGreaterThan(80);
    expect(Math.min(...positions)).toBeLessThan(-3);
    expect(Math.max(...positions)).toBeGreaterThan(5);
    expect(moving.visualization?.trail?.length).toBeGreaterThan(100);
    expect(verifiedBranch).toBe(true);
    lab.setRunning(false);
    lab.advanceFrame(200000);
    expect(lab.getFrame().timestamp_s).toBe(moving.timestamp_s);
    lab.reset();
    expect(lab.getFrame().vehicle).toMatchObject({ x_m: start.vehicle.x_m, y_m: start.vehicle.y_m, speed_mps: 0 });
    expect(lab.getFrame().timestamp_s).toBe(0);
  }, 15000);
});
