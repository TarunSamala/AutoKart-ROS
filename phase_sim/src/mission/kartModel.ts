// Loads the real kart glTF assets (prepared by kart/tools/prepare_kart_models.py
// from the OpenRC GoKart STL -- a 1:5-scale RC hobby model of a DIFFERENT
// vehicle, visual reference only; see docs/my_track_spec/STL_FIT.md) in place
// of the box+cone placeholder kart. Falls back to the exact dimension-sheet
// footprint rectangle if any part fails to load, per the master prompt's own
// stated fallback.
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';

export type KartProfile = 'shared' | 'built';

// Dimension sheet (kart/configs/karts/{shared,built}.yaml), rear-axle-referenced.
export const PROFILE_GEOMETRY: Record<KartProfile, { frontExtentM: number; rearOverhangM: number; halfWidthM: number; wheelbaseM: number; frontTrackM: number }> = {
  shared: { frontExtentM: 2.0, rearOverhangM: 0.4, halfWidthM: 1.181 / 2, wheelbaseM: 1.422, frontTrackM: 1.0648 },
  built: { frontExtentM: 2.0, rearOverhangM: 0.4, halfWidthM: 1.181 / 2, wheelbaseM: 1.1176, frontTrackM: 1.0668 },
};

const BODY_PARTS = ['body', 'chassis', 'rear_axle', 'steering'] as const;

/** The body/chassis/rear_axle/steering glb meshes are a SINGLE set, uniformly
 * scaled once on the "shared" profile's 2.400 m length (see
 * docs/my_track_spec/STL_FIT.md). For "built" (wheelbase 1.1176 m vs
 * "shared"'s 1.422 m) this compresses them along the LOCAL FORWARD axis
 * only (width/height untouched) so the body visually tracks the shorter
 * wheelbase instead of reusing "shared"'s full-length body with the
 * (correctly, shorter) built-profile wheels sitting disconnected from it. */
export function computeBodyForwardScale(profile: KartProfile): number {
  return PROFILE_GEOMETRY[profile].wheelbaseM / PROFILE_GEOMETRY.shared.wheelbaseM;
}

/** Where a front wheel's hub SHOULD sit, in the car group's local frame
 * (x forward, y up, z left/right), referenced to the rear axle at the
 * origin -- the one true source for front-wheel placement, applied
 * explicitly below rather than trusting the glb's own baked transform.
 * Pure function so it can be unit-tested without GLTFLoader/WebGL (which
 * do not run in this project's vitest/jsdom environment: GLTFLoader needs
 * `fetch` against a real dev-server path, and nothing here parses raw glb
 * binaries in Node). */
export function computeFrontWheelLocalPosition(profile: KartProfile, side: 'left' | 'right'): { xM: number; yM: number; zM: number } {
  const geo = PROFILE_GEOMETRY[profile];
  const lateralSign = side === 'left' ? 1 : -1;
  return { xM: geo.wheelbaseM, yM: 0, zM: lateralSign * (geo.frontTrackM / 2) };
}

/** Data-only "visual model; physics uses the dimension sheet" info-panel
 * content -- the designer's info panel (existing or future) calls this
 * rather than this file building any UI itself. */
export function getKartInfoPanelData(profile: KartProfile) {
  const geo = PROFILE_GEOMETRY[profile];
  return {
    profile,
    note: 'Visual model only -- physics, footprint, and clearance checks use the dimension sheet (kart/configs/karts/*.yaml), never this glTF asset.',
    stlScaleFactor: 5.438,
    wheelbaseM: geo.wheelbaseM,
    frontTrackM: geo.frontTrackM,
    rearTrackM: 1.1811,
    lengthM: 2.4,
    widthM: 1.181,
    heightM: 1.118,
    bodyForwardScale: computeBodyForwardScale(profile),
  };
}

// The STL prep script exported vertices as (forward, left, up) = (x, y, z) --
// i.e. Z-up, while this scene (and glTF's own convention) is Y-up. A -90 deg
// rotation about X converts Z-up data into this scene's Y-up frame.
const Z_UP_TO_Y_UP = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);

function footprintFallback(profile: KartProfile): THREE.Mesh {
  const geo = PROFILE_GEOMETRY[profile];
  const length = geo.frontExtentM + geo.rearOverhangM;
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(length, 0.3, geo.halfWidthM * 2),
    new THREE.MeshStandardMaterial({ color: 0xf0f4f0, metalness: 0.1, transparent: true, opacity: 0.85 }),
  );
  // Box is centred at its own origin; shift so x=0 is the rear axle, matching
  // the real glTF parts' origin convention.
  mesh.position.set((geo.frontExtentM - geo.rearOverhangM) / 2, 0.15, 0);
  mesh.userData = { kind: 'kart-fallback' };
  return mesh;
}

/** Loads the real kart model into `car`, replacing nothing itself -- caller
 * owns clearing any previous placeholder. Returns the two front-wheel
 * Object3Ds (once loaded) for live steering-angle rotation, via a callback
 * since loading is async. On any failure, leaves (or adds) the footprint
 * fallback and resolves with an empty wheel array. */
export function loadKartModel(car: THREE.Group, profile: KartProfile, onFrontWheels: (wheels: THREE.Object3D[]) => void): void {
  const fallback = footprintFallback(profile);
  car.add(fallback);
  const loader = new GLTFLoader();
  const base = '/kart';

  const loadOne = (path: string) =>
    new Promise<THREE.Object3D>((resolve, reject) => {
      loader.load(`${base}/${path}`, (gltf) => resolve(gltf.scene), undefined, reject);
    });

  Promise.all(BODY_PARTS.map((part) => loadOne(`${part}.glb`)))
    .then((parts) => {
      car.remove(fallback);
      const forwardScale = computeBodyForwardScale(profile);
      for (const part of parts) {
        // Scale FIRST, in the mesh's own original (pre-rotation) local
        // axes where native output X is forward (see STL_FIT.md's axis
        // convention) -- Object3D applies scale before its own rotation,
        // so this compresses length only, never width/height, regardless
        // of the Y-up/Z-up quaternion applied next.
        part.scale.set(forwardScale, 1, 1);
        part.quaternion.premultiply(Z_UP_TO_Y_UP);
        part.traverse((node) => {
          if (node instanceof THREE.Mesh) node.material = new THREE.MeshStandardMaterial({ color: 0xd8dee2, metalness: 0.15, roughness: 0.7 });
        });
        car.add(part);
      }
      return Promise.all([loadOne(`front_wheel_${profile}_left.glb`), loadOne(`front_wheel_${profile}_right.glb`)]);
    })
    .then((wheels) => {
      const sides: Array<'left' | 'right'> = ['left', 'right'];
      wheels.forEach((wheel, index) => {
        wheel.quaternion.premultiply(Z_UP_TO_Y_UP);
        // Explicit, pure-function-derived placement (computeFrontWheelLocalPosition)
        // rather than trusting the glb's own baked transform -- guarantees the
        // wheelbase/front-track numbers below hold exactly, and is what
        // the placement test in kartModel.test.ts exercises.
        const pos = computeFrontWheelLocalPosition(profile, sides[index]);
        wheel.position.set(pos.xM, pos.yM, pos.zM);
        wheel.traverse((node) => {
          if (node instanceof THREE.Mesh) node.material = new THREE.MeshStandardMaterial({ color: 0x15181c, roughness: 0.9 });
        });
        car.add(wheel);
      });
      onFrontWheels(wheels);
    })
    .catch((error) => {
      // eslint-disable-next-line no-console
      console.warn('kart model load failed, keeping footprint fallback', error);
      if (!car.children.includes(fallback)) car.add(fallback);
      onFrontWheels([]);
    });
}
