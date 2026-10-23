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
const PROFILE_GEOMETRY: Record<KartProfile, { frontExtentM: number; rearOverhangM: number; halfWidthM: number; wheelbaseM: number }> = {
  shared: { frontExtentM: 2.0, rearOverhangM: 0.4, halfWidthM: 1.181 / 2, wheelbaseM: 1.422 },
  built: { frontExtentM: 2.0, rearOverhangM: 0.4, halfWidthM: 1.181 / 2, wheelbaseM: 1.1176 },
};

const BODY_PARTS = ['body', 'chassis', 'rear_axle', 'steering'] as const;

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
      for (const part of parts) {
        part.quaternion.premultiply(Z_UP_TO_Y_UP);
        part.traverse((node) => {
          if (node instanceof THREE.Mesh) node.material = new THREE.MeshStandardMaterial({ color: 0xd8dee2, metalness: 0.15, roughness: 0.7 });
        });
        car.add(part);
      }
      return Promise.all([loadOne(`front_wheel_${profile}_left.glb`), loadOne(`front_wheel_${profile}_right.glb`)]);
    })
    .then((wheels) => {
      for (const wheel of wheels) {
        wheel.quaternion.premultiply(Z_UP_TO_Y_UP);
        wheel.traverse((node) => {
          if (node instanceof THREE.Mesh) node.material = new THREE.MeshStandardMaterial({ color: 0x15181c, roughness: 0.9 });
        });
        car.add(wheel);
      }
      onFrontWheels(wheels);
    })
    .catch((error) => {
      // eslint-disable-next-line no-console
      console.warn('kart model load failed, keeping footprint fallback', error);
      if (!car.children.includes(fallback)) car.add(fallback);
      onFrontWheels([]);
    });
}
