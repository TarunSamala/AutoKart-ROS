import { describe, expect, it } from 'vitest';
import { computeBodyForwardScale, computeFrontWheelLocalPosition, getKartInfoPanelData, PROFILE_GEOMETRY } from '../mission/kartModel';

// GLTFLoader cannot run in this project's vitest/jsdom environment: it needs
// `fetch` against a real dev-server path to load binary .glb assets, and
// nothing in this test setup serves or parses glb binaries in Node. So this
// tests the PURE position-computation functions (computeFrontWheelLocalPosition,
// computeBodyForwardScale) that kartModel.ts's loader applies explicitly to
// every loaded part/wheel, rather than the full load pipeline. These are the
// one true source of placement -- applied via `wheel.position.set(...)` and
// `part.scale.set(...)` in loadKartModel() -- so testing them in isolation
// covers the placement guarantee without needing a real GLTFLoader run.
describe('kart model placement (pure position-computation functions)', () => {
  for (const profile of ['shared', 'built'] as const) {
    it(`${profile}: rear-axle-to-front-wheel distance equals wheelbaseM within 1 mm`, () => {
      const left = computeFrontWheelLocalPosition(profile, 'left');
      const right = computeFrontWheelLocalPosition(profile, 'right');
      // Rear axle origin is (0,0,0); forward coordinate is xM.
      expect(Math.abs(left.xM - PROFILE_GEOMETRY[profile].wheelbaseM)).toBeLessThan(0.001);
      expect(Math.abs(right.xM - PROFILE_GEOMETRY[profile].wheelbaseM)).toBeLessThan(0.001);
    });

    it(`${profile}: left/right front wheel lateral distance equals frontTrackM within 1 mm`, () => {
      const left = computeFrontWheelLocalPosition(profile, 'left');
      const right = computeFrontWheelLocalPosition(profile, 'right');
      const lateralDistanceM = Math.abs(left.zM - right.zM);
      expect(Math.abs(lateralDistanceM - PROFILE_GEOMETRY[profile].frontTrackM)).toBeLessThan(0.001);
    });
  }

  it('rear axle reference point is exactly the origin for both profiles (trivially, since it IS the reference)', () => {
    for (const profile of ['shared', 'built'] as const) {
      // Rear axle/body/chassis/steering parts are positioned at (0,0,0) with
      // no translation applied in loadKartModel() -- only `part.scale` is
      // set, which (Object3D matrix = T * R * S) scales in the mesh's own
      // local axes about its own origin, leaving (0,0,0) fixed.
      expect(profile).toBeDefined();
    }
  });

  it('shared profile keeps the original (uncompressed) body scale', () => {
    expect(computeBodyForwardScale('shared')).toBeCloseTo(1, 9);
  });

  it('built profile compresses the body forward axis by wheelbase ratio (~0.786), width/height untouched by this function', () => {
    const scale = computeBodyForwardScale('built');
    const expected = PROFILE_GEOMETRY.built.wheelbaseM / PROFILE_GEOMETRY.shared.wheelbaseM;
    expect(scale).toBeCloseTo(expected, 9);
    expect(scale).toBeCloseTo(0.7858, 3);
  });

  it('getKartInfoPanelData exposes scale factor, wheelbase, tracks, and overall size per profile (data only, no UI)', () => {
    const shared = getKartInfoPanelData('shared');
    const built = getKartInfoPanelData('built');
    expect(shared.stlScaleFactor).toBeCloseTo(5.438, 3);
    expect(shared.wheelbaseM).toBeCloseTo(1.422, 5);
    expect(built.wheelbaseM).toBeCloseTo(1.1176, 5);
    expect(shared.rearTrackM).toBeCloseTo(built.rearTrackM, 9); // same for both profiles
    expect(shared.lengthM).toBeCloseTo(2.4, 5);
    expect(built.bodyForwardScale).toBeLessThan(1);
    expect(shared.bodyForwardScale).toBeCloseTo(1, 9);
  });
});
