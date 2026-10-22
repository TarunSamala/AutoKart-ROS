import { describe, expect, it } from 'vitest';
import { LabSkidpad } from '../mission/LabSkidpad';

describe('LAB skidpad route targets', () => {
  it('keeps displayed targets on the selected branch and clears the terminal target', () => {
    const lab = new LabSkidpad();
    const states = new Set<string>();
    let priorBranch = '';
    let priorProgress = -Infinity;
    lab.setRunning(true);
    for (let i = 0; i <= 9600; i += 1) {
      lab.advanceFrame(i * 1000 / 60);
      if (i % 15 !== 0) continue;
      const frame = lab.getFrame();
      const state = frame.planning.skidpad?.state;
      if (state) states.add(state);
      const target = frame.planning.lookahead;
      const selected = frame.planning.selected_path.at(-1);
      const branch = frame.planning.skidpad?.branch;
      if (branch !== priorBranch) { priorBranch = branch ?? ''; priorProgress = -Infinity; }
      if (target) {
        expect(selected, `${state}: target must come from the active path`).toEqual(target);
        const right = branch === 'RIGHT';
        const left = branch === 'LEFT';
        const centerX = right ? 8 : -8;
        const direction = right ? -1 : 1;
        const angle = Math.atan2(target.y_m, target.x_m - centerX);
        const current = Math.atan2(frame.vehicle.y_m, frame.vehicle.x_m - centerX);
        const ahead = (direction * (angle - current) + Math.PI * 4) % (Math.PI * 2);
        const progress = right || left ? Math.max(0, right ? -(frame.planning.skidpad?.w_right ?? 0) : frame.planning.skidpad?.w_left ?? 0) + ahead / (Math.PI * 2) : target.y_m;
        expect(progress, `${state}: target progress went backwards`).toBeGreaterThanOrEqual(priorProgress - 0.03);
        priorProgress = Math.max(priorProgress, progress);
      }
      if (state === 'RIGHT_LOOPS' && frame.vehicle.y_m > -2 && target) expect(target.x_m).toBeGreaterThan(0);
      if (state === 'LEFT_LOOPS' && target) expect(target.x_m).toBeLessThan(0);
      if (state === 'STOP') {
        expect(target).toBeNull();
        expect(frame.vehicle.speed_mps).toBe(0);
        expect(frame.vehicle.target_speed_mps).toBe(0);
        break;
      }
    }
    expect(states).toEqual(new Set(['RIGHT_LOOPS', 'LEFT_LOOPS', 'EXIT', 'STOP']));
  }, 15000);

  it.each([0, 0.12])('uses an observed central EXIT segment and stops with noise %.2f m', (noise) => {
    const defaults = new LabSkidpad().getSettings();
    const lab = new LabSkidpad('SKIDPAD', { ...defaults, noise });
    lab.setRunning(true);
    let lastLeftTarget: { x_m: number; y_m: number } | null = null;
    let lastLeftProgress = -Infinity;
    let sawFinalLeft = false;
    let sawExit = false;
    let minConeDistance = Infinity;
    let finalState = '';
    for (let index = 0; index < 9600; index += 1) {
      lab.advanceFrame(index * 1000 / 60);
      if (index % 15) continue;
      const frame = lab.getFrame();
      const state = frame.planning.skidpad?.state ?? '';
      finalState = state;
      const target = frame.planning.lookahead;
      if (state === 'LEFT_LOOPS' && (frame.planning.skidpad?.w_left ?? 0) >= 1.6) {
        sawFinalLeft = true;
        if (target) {
          expect(target.x_m).toBeLessThan(0);
          expect(frame.planning.selected_path.at(-1)).toEqual(target);
          const progress = frame.planning.skidpad?.progress_index ?? -Infinity;
          expect(progress).toBeGreaterThanOrEqual(lastLeftProgress - 0.03);
          lastLeftProgress = progress;
          if (lastLeftTarget) expect(Math.hypot(target.x_m - lastLeftTarget.x_m, target.y_m - lastLeftTarget.y_m)).toBeLessThan(2);
          lastLeftTarget = target;
        }
      }
      if (state === 'EXIT') {
        sawExit = true;
        if (target) {
          expect(Math.abs(target.x_m)).toBeLessThan(defaults.width * 0.25);
          expect(target.y_m).toBeGreaterThan(defaults.junctionLength);
          expect(frame.planning.selected_path.at(-1)).toEqual(target);
          const start = frame.planning.selected_path[0];
          const dx = target.x_m - start.x_m, dy = target.y_m - start.y_m;
          const lengthSq = dx * dx + dy * dy;
          for (const cone of frame.visualization?.course_cones ?? []) {
            const fraction = Math.max(0, Math.min(1, ((cone.x_m - start.x_m) * dx + (cone.y_m - start.y_m) * dy) / lengthSq));
            const distance = Math.hypot(cone.x_m - start.x_m - fraction * dx, cone.y_m - start.y_m - fraction * dy);
            expect(distance, `EXIT connector approaches cone ${cone.id}`).toBeGreaterThan(0.6);
          }
        }
      }
      for (const cone of frame.visualization?.course_cones ?? []) {
        minConeDistance = Math.min(minConeDistance, Math.hypot(cone.x_m - frame.vehicle.x_m, cone.y_m - frame.vehicle.y_m));
      }
      if (state === 'STOP') {
        expect(frame.planning.selected_path).toEqual([]);
        expect(target).toBeNull();
        expect(frame.vehicle.speed_mps).toBe(0);
        expect(frame.planning.skidpad?.w_right).toBeLessThan(-1.9);
        expect(frame.planning.skidpad?.w_left).toBeGreaterThan(1.9);
        break;
      }
    }
    expect(sawFinalLeft).toBe(true);
    expect(sawExit).toBe(true);
    expect(finalState).toBe('STOP');
    // This is a conservative center-to-cone screen for a 1.2 m-wide proxy,
    // not a surveyed vehicle footprint or dynamic collision certificate.
    expect(minConeDistance).toBeGreaterThan(0.6);
  }, 30000);
});
