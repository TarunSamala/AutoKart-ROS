import { describe, expect, it } from 'vitest';
import { LabSkidpad } from '../mission/LabSkidpad';
import { decodeReplay, encodeReplay } from '../mission/replay';
import { frameAt, stageFrame } from '../mission/schema';

describe('common frame replay', () => {
  it('round trips recorded frames and freezes stages on the same frame', () => {
    const lab = new LabSkidpad();
    const first = lab.getFrame();
    lab.setRunning(true);
    for (let i = 0; i <= 180; i++) lab.advanceFrame(i * 1000 / 60);
    const second = lab.getFrame();
    const decoded = decodeReplay(encodeReplay([first, second]));
    expect(decoded).toEqual([first, second]);
    expect(frameAt(decoded, second.timestamp_s - 0.001)).toEqual(first);
    expect(frameAt(decoded, second.timestamp_s)).toEqual(second);
    const see = stageFrame(second, 1), control = stageFrame(second, 8);
    expect(see.timestamp_s).toBe(control.timestamp_s);
    expect(see.perception.cones).toEqual(control.perception.cones);
    expect(see.planning.delaunay_edges).toHaveLength(0);
    expect(control.planning.delaunay_edges.length).toBeGreaterThan(0);
    expect(decodeReplay(JSON.stringify({ frames: [first, second] }))).toEqual(decoded);
  });
});
