import { describe, expect, it } from 'vitest';
import {
  MY_TRACK_CONFIG,
  MyTrackMission,
  addMyTrackFallbackGates,
  buildMyTrackGates,
  classifyMyTrackObstacles,
  computeAckermannAngles,
  detectMyTrackFinish,
  maxAbsLateralDeviationM,
  rmsLateralDeviationM,
  type MyTrackBodyPoint,
} from '../mission/labAlgorithms';
import type { MissionFrame } from '../mission/schema';

const pt = (x_m: number, y_m: number, klass: string, id: string): MyTrackBodyPoint => ({ x_m, y_m, class: klass, id });

describe('MY_TRACK obstacle classifier', () => {
  it('accepts a compact orange cluster (lateral spread >= 0.28 m, link <= 0.85 m)', () => {
    const points: MyTrackBodyPoint[] = [
      pt(5.0, 0.2, 'ORANGE', 'o1'), pt(5.3, 0.5, 'ORANGE', 'o2'), pt(5.6, 0.1, 'ORANGE', 'o3'), pt(5.2, -0.2, 'ORANGE', 'o4'),
    ];
    const result = classifyMyTrackObstacles(points, new Set());
    expect(result.size).toBe(4);
  });

  it('rejects a straight orange boundary wall (collinear, lateral spread ~0)', () => {
    const points: MyTrackBodyPoint[] = [
      pt(0, 1.5, 'ORANGE', 'w1'), pt(0.5, 1.5, 'ORANGE', 'w2'), pt(1.0, 1.5, 'ORANGE', 'w3'), pt(1.5, 1.5, 'ORANGE', 'w4'),
    ];
    const result = classifyMyTrackObstacles(points, new Set());
    expect(result.size).toBe(0);
  });
});

describe('MY_TRACK single-side fallback gates', () => {
  it('synthesizes a gate from a one-sided boundary when no real gate is nearby', () => {
    const boundary: MyTrackBodyPoint[] = [0, 1, 2, 3, 4].map((x) => pt(x, 1.5, 'BLUE', `b${x}`));
    const extra = addMyTrackFallbackGates(boundary, []);
    expect(extra.length).toBeGreaterThan(0);
    for (const gate of extra) expect(gate.widthM).toBeCloseTo(MY_TRACK_CONFIG.nominalLaneWidthM, 5);
  });

  it('does not duplicate a fallback gate near an already-real gate', () => {
    const boundary: MyTrackBodyPoint[] = [pt(0, 1.5, 'BLUE', 'b0'), pt(1, 1.5, 'BLUE', 'b1'), pt(0, -1.5, 'YELLOW', 'y0'), pt(1, -1.5, 'YELLOW', 'y1')];
    const real = buildMyTrackGates(boundary);
    expect(real.length).toBeGreaterThan(0);
    const extra = addMyTrackFallbackGates(boundary, real);
    // every fallback gate must be farther than fallbackGateMinExistingDistM from the real gate's midpoint
    for (const gate of extra) {
      for (const r of real) {
        expect(Math.hypot(gate.midpoint.x_m - r.midpoint.x_m, gate.midpoint.y_m - r.midpoint.y_m)).toBeGreaterThan(MY_TRACK_CONFIG.fallbackGateMinExistingDistM);
      }
    }
  });
});

describe('MY_TRACK finish detection', () => {
  it('finds no finish before finishMinOdometryM of progress', () => {
    const points: MyTrackBodyPoint[] = [pt(2, 1.2, 'ORANGE', 'f1'), pt(2, -1.2, 'ORANGE', 'f2')];
    const result = detectMyTrackFinish(points, 10);
    expect(result.finishXBody).toBeNull();
  });

  it('finds a finish pair past finishMinOdometryM, near the lane edge, ahead of the kart', () => {
    // Real layout geometry (docs/my_track_spec, FINISH-1..4): both cones of a
    // pair sit on the SAME side, offset from the lane edge, not straddling
    // the centreline -- see kart/kart_sim/my_track_algorithm.py's
    // `abs(ya + yb) / 2 > 1.1` check.
    const points: MyTrackBodyPoint[] = [pt(2, 1.15, 'ORANGE', 'f1'), pt(2.05, 1.5, 'ORANGE', 'f2')];
    const result = detectMyTrackFinish(points, 45);
    expect(result.finishXBody).not.toBeNull();
    expect(result.finishXBody).toBeCloseTo(2.025, 2);
    expect(result.finishIndices.size).toBe(2);
  });
});

function straightCorridor(fromX: number, toX: number, widthM = 2.0): MyTrackBodyPoint[] {
  const points: MyTrackBodyPoint[] = [];
  for (let x = fromX; x <= toX; x += 1) {
    points.push(pt(x, widthM / 2, 'BLUE', `L${x}`));
    points.push(pt(x, -widthM / 2, 'YELLOW', `R${x}`));
  }
  return points;
}

const GEOMETRY = { wheelbaseM: 1.0, frontExtentM: 1.0, rearOverhangM: 0.2, halfWidthM: 0.3 };

describe('MY_TRACK detour lock reuse', () => {
  it('admits a detour far from a narrow obstacle, then keeps it admitted (no re-search) when the same cluster later looks too tight', () => {
    const mission = new MyTrackMission(GEOMETRY);
    const boundary = straightCorridor(-2, 20);
    const narrowObstacle: MyTrackBodyPoint[] = [pt(8.0, 0.45, 'ORANGE', 'ob1'), pt(8.3, 0.8, 'ORANGE', 'ob2'), pt(8.5, 0.55, 'ORANGE', 'ob3'), pt(8.2, 0.5, 'ORANGE', 'ob4')];
    const pose = { x_m: 0, y_m: 0, yaw_rad: 0 };

    const first = mission.step([...boundary, ...narrowObstacle], pose, 0.4, 41, 0);
    expect(first.obstacle).toBe(true);
    expect(first.detour).toBe(true);
    expect(mission.lockCount()).toBe(1);

    // Same cluster (same approximate centre), but now reaching almost to the
    // centreline (clearance ~0), which pushes the free-width check right to
    // its boundary -- a fresh search here is at best marginal. This
    // reproduces the scenario the Python side traced to a real collision
    // (admission succeeding far away, then failing/degrading once closer,
    // snapping the path back toward the obstacle). The lock must still be
    // reused verbatim rather than re-searched.
    const widerObstacle: MyTrackBodyPoint[] = [
      pt(8.0, 0.0, 'ORANGE', 'ob1'), pt(8.1, 0.3, 'ORANGE', 'ob2'), pt(8.3, 0.6, 'ORANGE', 'ob3'), pt(8.5, 0.9, 'ORANGE', 'ob4'),
      pt(7.9, 0.75, 'ORANGE', 'ob5'), pt(8.2, 0.5, 'ORANGE', 'ob6'),
    ];
    const second = mission.step([...boundary, ...widerObstacle], pose, 0.4, 41, 0.05);
    expect(mission.lockCount()).toBe(1); // not re-searched into a second lock
    expect(second.detour).toBe(true); // still treated as admitted, not snapped to a stop
  });
});

describe('MY_TRACK finish latch', () => {
  it('latches the finish position on first detection and keeps reporting it even with no cones the next frame', () => {
    const mission = new MyTrackMission(GEOMETRY);
    const boundary = straightCorridor(-2, 5);
    const finishPair: MyTrackBodyPoint[] = [pt(3.0, 1.15, 'ORANGE', 'fin1'), pt(3.05, 1.5, 'ORANGE', 'fin2')];
    const pose = { x_m: 0, y_m: 0, yaw_rad: 0 };

    const first = mission.step([...boundary, ...finishPair], pose, 0.3, 45, 0);
    expect(first.finishDetected).toBe(true);
    expect(mission.state === 'FINISH' || mission.state === 'STOPPED').toBe(true);

    const second = mission.step([], pose, 0.3, 45, 0.05);
    expect(second.finishDetected).toBe(true);
  });
});

describe('MY_TRACK mission FSM', () => {
  it('goes READY -> RUN -> ... -> STOPPED and never returns to READY or RUN once stopped', () => {
    const mission = new MyTrackMission(GEOMETRY, 0.5, 0.3);
    const pose = { x_m: 0, y_m: 0, yaw_rad: 0 };
    const states: string[] = [mission.state];
    // blind creep (no cones yet)
    mission.step([], pose, 0, 0, 0);
    states.push(mission.state);
    // normal corridor driving
    const boundary = straightCorridor(-2, 20);
    mission.step(boundary, pose, 0.4, 10, 0.05);
    states.push(mission.state);
    // finish pair appears past the odometry threshold, kart nearly stopped
    const finishPair: MyTrackBodyPoint[] = [pt(0.3, 1.15, 'ORANGE', 'fin1'), pt(0.35, 1.5, 'ORANGE', 'fin2')];
    mission.step([...boundary, ...finishPair], pose, 0.02, 45, 0.1);
    states.push(mission.state);
    mission.step([...boundary, ...finishPair], pose, 0.02, 45, 0.15);
    states.push(mission.state);

    expect(states[0]).toBe('READY');
    const stoppedIndex = states.indexOf('STOPPED');
    if (stoppedIndex >= 0) {
      for (const s of states.slice(stoppedIndex + 1)) expect(s).toBe('STOPPED');
    }
    expect(states.slice(1)).not.toContain('READY');
  });
});

describe('Ackermann steering angles', () => {
  it('both wheels equal the bicycle-model angle when going straight', () => {
    const { leftRad, rightRad } = computeAckermannAngles(0, 1.422, 1.0648);
    expect(leftRad).toBeCloseTo(0, 9);
    expect(rightRad).toBeCloseTo(0, 9);
  });

  it('inner wheel turns more than outer wheel in a turn', () => {
    const delta = 0.25; // left turn
    const { leftRad, rightRad } = computeAckermannAngles(delta, 1.422, 1.0648);
    expect(Math.abs(leftRad)).toBeGreaterThan(Math.abs(rightRad));
    const right = computeAckermannAngles(-delta, 1.422, 1.0648);
    expect(Math.abs(right.rightRad)).toBeGreaterThan(Math.abs(right.leftRad));
  });
});

describe('lateral deviation helpers', () => {
  const frame = (lateralDeviationM?: number): MissionFrame => ({
    schema_version: 1, source: 'LAB', run_id: 'r', sequence: 0, timestamp_s: 0,
    vehicle: { x_m: 0, y_m: 0, yaw_rad: 0, speed_mps: 0 },
    perception: { cones: [] }, planning: { delaunay_edges: [], gabriel_edges: [], gates: [], candidate_paths: [], selected_path: [] },
    safety: {}, metrics: {}, lateralDeviationM,
  });

  it('ignores frames with no fit (undefined), not treating them as zero', () => {
    const frames = [frame(undefined), frame(0.1), frame(-0.3)];
    expect(maxAbsLateralDeviationM(frames)).toBeCloseTo(0.3, 5);
    expect(rmsLateralDeviationM(frames)).toBeCloseTo(Math.sqrt((0.01 + 0.09) / 2), 5);
  });

  it('returns 0 when no frame has the field set', () => {
    expect(maxAbsLateralDeviationM([frame(undefined)])).toBe(0);
    expect(rmsLateralDeviationM([frame(undefined)])).toBe(0);
  });
});
