import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import type { Cone, Edge, MissionFrame, Point } from './schema';

export type Layer = 'groundTruth' | 'tracked' | 'detections' | 'delaunay' | 'gabriel' | 'gates' | 'boundaries' | 'candidates' | 'selected' | 'safety' | 'sensor';
export type Selection = { kind: string; id: string };
type Props = { frame: MissionFrame | null; layers: Record<Layer, boolean>; cameraMode: string; selection: Selection; onSelect: (selection: Selection) => void; animationStage?: number };
export const VISUAL_COLORS = { BLUE: 0x428dff, YELLOW: 0xffd451, ORANGE: 0xff923e, UNKNOWN: 0xa8bac3, DELAUNAY: 0x56d8ed, ACCEPTED: 0x54e2aa, REJECTED: 0xff6674, CANDIDATE: 0xf2f6f5, SELECTED: 0xff61ca } as const;
export const CONE_CLASS_COLORS: Record<string, number> = { BLUE: VISUAL_COLORS.BLUE, LEFT: VISUAL_COLORS.BLUE, YELLOW: VISUAL_COLORS.YELLOW, RIGHT: VISUAL_COLORS.YELLOW, ORANGE: VISUAL_COLORS.ORANGE, SPECIAL: VISUAL_COLORS.ORANGE, START: VISUAL_COLORS.ORANGE, FINISH: VISUAL_COLORS.ORANGE, ENTRY: VISUAL_COLORS.ORANGE, EXIT: VISUAL_COLORS.ORANGE };
const point3 = (p: Point, y = 0.1) => new THREE.Vector3(p.x_m, y + (p.z_m ?? 0), -p.y_m);
const coneColor = (value: string) => CONE_CLASS_COLORS[value.toUpperCase()] ?? VISUAL_COLORS.UNKNOWN;
const LINE_CAPACITY = 8192;

type Lines = { object: THREE.LineSegments; positions: Float32Array; count: number };
function lines(hex: number, opacity = 1): Lines {
  const positions = new Float32Array(LINE_CAPACITY * 3);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3).setUsage(THREE.DynamicDrawUsage));
  geometry.setDrawRange(0, 0);
  const object = new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: hex, transparent: opacity < 1, opacity, depthWrite: false }));
  return { object, positions, count: 0 };
}
function updateLines(pool: Lines, segments: Array<[Point, Point]>, ids?: string[], kind?: string) {
  const count = Math.min(segments.length, LINE_CAPACITY / 2);
  for (let i = 0; i < count; i++) {
    const [a, b] = segments[i];
    pool.positions.set([a.x_m, 0.16, -a.y_m, b.x_m, 0.16, -b.y_m], i * 6);
  }
  pool.count = count;
  pool.object.geometry.setDrawRange(0, count * 2);
  (pool.object.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
  pool.object.visible = count > 0;
  pool.object.userData = { kind, ids };
}
function edgeSegments(edges: Edge[], cones: Map<string, Cone>): { segments: Array<[Point, Point]>; ids: string[] } {
  const segments: Array<[Point, Point]> = [], ids: string[] = [];
  edges.forEach((edge) => { const a = cones.get(edge.a), b = cones.get(edge.b); if (a && b) { segments.push([a, b]); ids.push(edge.id); } });
  return { segments, ids };
}
function polySegments(points: Point[]): Array<[Point, Point]> { return points.slice(1).map((point, index) => [points[index], point]); }
function dispose(root: THREE.Object3D) { root.traverse((object) => { const mesh = object as THREE.Mesh; mesh.geometry?.dispose(); const mats = mesh.material ? Array.isArray(mesh.material) ? mesh.material : [mesh.material] : []; mats.forEach((material) => material.dispose()); }); }

type SceneState = {
  scene: THREE.Scene; renderer: THREE.WebGLRenderer; perspective: THREE.PerspectiveCamera; top: THREE.OrthographicCamera;
  content: THREE.Group; raycaster: THREE.Raycaster; pointer: THREE.Vector2; orbit: { az: number; el: number; distance: number; pan: THREE.Vector3 };
  drag: { x: number; y: number; startX: number; startY: number; button: number } | null; focus: THREE.Vector3; mode: string;
  course: THREE.InstancedMesh; courseBase: THREE.InstancedMesh; detections: THREE.InstancedMesh; detectionBase: THREE.InstancedMesh;
  graph: Record<'delaunay' | 'accepted' | 'rejected' | 'gates' | 'boundaryLeft' | 'boundaryRight' | 'candidates' | 'selected' | 'trail' | 'guide' | 'distance' | 'branchValid' | 'branchInvalid' | 'leftArc' | 'rightArc' | 'angle', Lines>;
  car: THREE.Group; frontWheels: THREE.Mesh[]; lookahead: THREE.Mesh; obstacle: THREE.Mesh; stop: THREE.Mesh; envelope: THREE.Mesh;
  fov: THREE.Mesh; fovRange: number; fovAngle: number; disk: THREE.Mesh; rings: THREE.Mesh[]; labels: Array<{ element: HTMLDivElement; point: THREE.Vector3 }>;
  frame: MissionFrame | null; selection: Selection; animationStage: number; stageStarted: number; courseCenter: THREE.Vector3;
};

export function MissionScene({ frame, layers, cameraMode, selection, onSelect, animationStage = 0 }: Props) {
  const host = useRef<HTMLDivElement>(null);
  const state = useRef<SceneState | null>(null);
  const callback = useRef(onSelect);
  callback.current = onSelect;

  useEffect(() => {
    const element = host.current;
    if (!element) return;
    const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.6));
    renderer.setSize(element.clientWidth, element.clientHeight);
    renderer.setClearColor(0x081923);
    element.appendChild(renderer.domElement);
    const labelsHost = document.createElement('div'); labelsHost.className = 'mc-scene-labels'; element.appendChild(labelsHost);
    const scene = new THREE.Scene(); scene.fog = new THREE.Fog(0x081923, 95, 250);
    const perspective = new THREE.PerspectiveCamera(49, 1, 0.1, 500);
    const top = new THREE.OrthographicCamera(-30, 30, 30, -30, 0.1, 500);
    top.up.set(0, 0, -1);
    const content = new THREE.Group(); scene.add(content);
    scene.add(new THREE.AmbientLight(0xffffff, 1.65));
    const sun = new THREE.DirectionalLight(0xffffff, 2.3); sun.position.set(15, 30, 12); scene.add(sun);
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(360, 360), new THREE.MeshStandardMaterial({ color: 0x102833, roughness: 1 }));
    ground.rotation.x = -Math.PI / 2; ground.position.y = -0.07; scene.add(ground);
    const grid = new THREE.GridHelper(360, 90, 0x284a56, 0x1a3642); grid.position.y = -0.055; scene.add(grid);
    const coneGeometry = new THREE.ConeGeometry(0.31, 0.83, 12);
    const baseGeometry = new THREE.CylinderGeometry(0.39, 0.39, 0.07, 12);
    const coneMaterial = new THREE.MeshStandardMaterial({ roughness: 0.68, metalness: 0.04 });
    const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x11242d, roughness: 0.95 });
    const course = new THREE.InstancedMesh(coneGeometry, coneMaterial, 1024);
    const courseBase = new THREE.InstancedMesh(baseGeometry, baseMaterial, 1024);
    const detections = new THREE.InstancedMesh(coneGeometry.clone(), coneMaterial.clone(), 512);
    const detectionBase = new THREE.InstancedMesh(baseGeometry.clone(), baseMaterial.clone(), 512);
    [course, courseBase, detections, detectionBase].forEach((mesh) => { mesh.count = 0; mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage); content.add(mesh); });
    const graph = {
      delaunay: lines(VISUAL_COLORS.DELAUNAY, 0.64), accepted: lines(VISUAL_COLORS.ACCEPTED, 0.95),
      rejected: lines(VISUAL_COLORS.REJECTED, 0.56), gates: lines(0xffdc84, 0.88),
      boundaryLeft: lines(VISUAL_COLORS.BLUE, 0.8), boundaryRight: lines(VISUAL_COLORS.YELLOW, 0.8),
      candidates: lines(VISUAL_COLORS.CANDIDATE, 0.44), selected: lines(VISUAL_COLORS.SELECTED, 1),
      trail: lines(0xdbe8ee, 0.75), guide: lines(0x85b2bc, 0.48), distance: lines(0xffa879, 0.9),
      branchValid: lines(0x8be6d7, 0.82), branchInvalid: lines(0xff747c, 0.35),
      leftArc: lines(0xd4a4ff, 1), rightArc: lines(0xd4a4ff, 1), angle: lines(0xf4dd9b, 0.7),
    };
    Object.values(graph).forEach((pool) => content.add(pool.object));
    const car = new THREE.Group(); car.userData = { kind: 'kart', id: 'vehicle' }; content.add(car);
    const body = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.28, 0.98), new THREE.MeshStandardMaterial({ color: 0xf0f4f0, metalness: 0.22 })); body.position.y = 0.16; car.add(body);
    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.33, 0.76, 3), new THREE.MeshStandardMaterial({ color: 0x58e6b2 })); nose.rotation.z = -Math.PI / 2; nose.position.set(1.04, 0.18, 0); car.add(nose);
    const frontWheels: THREE.Mesh[] = [];
    [-1, 1].forEach((x) => [-1, 1].forEach((z) => { const wheel = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.27, 0.15), new THREE.MeshStandardMaterial({ color: 0x071016 })); wheel.position.set(x * 0.6, 0.01, z * 0.53); car.add(wheel); if (x > 0) frontWheels.push(wheel); }));
    const lookahead = new THREE.Mesh(new THREE.SphereGeometry(0.33, 20, 12), new THREE.MeshBasicMaterial({ color: 0xffe47c })); content.add(lookahead);
    const obstacle = new THREE.Mesh(new THREE.BoxGeometry(1, 0.95, 1), new THREE.MeshStandardMaterial({ color: 0xff735f, transparent: true, opacity: 0.78 })); obstacle.userData = { kind: 'obstacle', id: '' }; content.add(obstacle);
    const stop = new THREE.Mesh(new THREE.RingGeometry(0.38, 0.53, 36), new THREE.MeshBasicMaterial({ color: 0xff8e69, side: THREE.DoubleSide })); stop.rotation.x = -Math.PI / 2; content.add(stop);
    const envelope = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ color: 0x5be7b0, transparent: true, opacity: 0.14, side: THREE.DoubleSide, depthWrite: false })); envelope.rotation.x = -Math.PI / 2; content.add(envelope);
    const fov = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ color: 0x66cfe9, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false })); content.add(fov);
    const disk = new THREE.Mesh(new THREE.CircleGeometry(1, 48), new THREE.MeshBasicMaterial({ color: VISUAL_COLORS.ACCEPTED, transparent: true, opacity: 0.15, side: THREE.DoubleSide, depthWrite: false })); disk.rotation.x = -Math.PI / 2; content.add(disk);
    const rings = [0, 1].map(() => { const mesh = new THREE.Mesh(new THREE.RingGeometry(1, 1.035, 80), new THREE.MeshBasicMaterial({ color: 0xb191f2, side: THREE.DoubleSide, transparent: true, opacity: 0.68 })); mesh.rotation.x = -Math.PI / 2; content.add(mesh); return mesh; });
    const raycaster = new THREE.Raycaster(); raycaster.params.Line!.threshold = 0.34;
    const current: SceneState = { scene, renderer, perspective, top, content, raycaster, pointer: new THREE.Vector2(), orbit: { az: 1.14, el: 1.0, distance: 56, pan: new THREE.Vector3() }, drag: null, focus: new THREE.Vector3(), mode: 'algorithm', course, courseBase, detections, detectionBase, graph, car, frontWheels, lookahead, obstacle, stop, envelope, fov, fovRange: -1, fovAngle: -1, disk, rings, labels: [], frame: null, selection: { kind: 'kart', id: 'vehicle' }, animationStage: 0, stageStarted: performance.now(), courseCenter: new THREE.Vector3() };
    state.current = current;
    const resize = new ResizeObserver(() => { const width = Math.max(1, element.clientWidth), height = Math.max(1, element.clientHeight), aspect = width / height; perspective.aspect = aspect; perspective.updateProjectionMatrix(); top.left = -current.orbit.distance * aspect / 2; top.right = current.orbit.distance * aspect / 2; top.top = current.orbit.distance / 2; top.bottom = -current.orbit.distance / 2; top.updateProjectionMatrix(); renderer.setSize(width, height); });
    resize.observe(element);
    const onDown = (event: PointerEvent) => { current.drag = { x: event.clientX, y: event.clientY, startX: event.clientX, startY: event.clientY, button: event.button }; renderer.domElement.setPointerCapture(event.pointerId); };
    const onMove = (event: PointerEvent) => { if (!current.drag || current.mode !== 'free') return; const dx = event.clientX - current.drag.x, dy = event.clientY - current.drag.y; if (current.drag.button === 2 || event.shiftKey) { current.orbit.pan.x -= dx * current.orbit.distance / Math.max(300, element.clientWidth); current.orbit.pan.z -= dy * current.orbit.distance / Math.max(300, element.clientHeight); } else { current.orbit.az -= dx * 0.006; current.orbit.el = THREE.MathUtils.clamp(current.orbit.el + dy * 0.006, 0.15, 1.53); } current.drag.x = event.clientX; current.drag.y = event.clientY; };
    const onUp = (event: PointerEvent) => { const drag = current.drag; current.drag = null; if (!drag || Math.hypot(event.clientX - drag.startX, event.clientY - drag.startY) > 4) return; const rect = renderer.domElement.getBoundingClientRect(); current.pointer.set(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1); raycaster.setFromCamera(current.pointer, current.mode === 'top' ? top : perspective); for (const hit of raycaster.intersectObjects(content.children, true)) { let object: THREE.Object3D | null = hit.object; while (object && object !== content && !object.userData.kind) object = object.parent; if (!object || object === content) continue; if (object.userData.kind === 'cone' && hit.instanceId !== undefined) { callback.current({ kind: 'cone', id: object.userData.ids[hit.instanceId] }); return; } if (object.userData.ids) { const id = object.userData.ids[Math.floor((hit.index ?? 0) / 2)]; if (id) { callback.current({ kind: object.userData.kind, id }); return; } } if (object.userData.kind && object.userData.id) { callback.current({ kind: object.userData.kind, id: object.userData.id }); return; } } };
    const onWheel = (event: WheelEvent) => { event.preventDefault(); current.orbit.distance = THREE.MathUtils.clamp(current.orbit.distance * (event.deltaY > 0 ? 1.09 : 0.91), 8, 190); };
    const onContext = (event: MouseEvent) => event.preventDefault();
    renderer.domElement.addEventListener('pointerdown', onDown); renderer.domElement.addEventListener('pointermove', onMove); renderer.domElement.addEventListener('pointerup', onUp); renderer.domElement.addEventListener('wheel', onWheel, { passive: false }); renderer.domElement.addEventListener('contextmenu', onContext);
    let animation = 0;
    const draw = (now: number) => {
      const visible = current.frame;
      const carPosition = visible ? point3(visible.vehicle, 0) : new THREE.Vector3();
      const longLab = visible?.source === 'LAB' && (visible.run_id.includes('autocross') || visible.run_id.includes('brake'));
      const desired = current.mode === 'top' && visible?.source === 'LAB' ? current.courseCenter.clone() : current.mode === 'algorithm' ? carPosition.clone().lerp(current.courseCenter, visible?.source === 'LAB' && visible.sequence === 0 ? 1 : longLab ? 0.12 : visible?.source === 'DEMO' || visible?.source === 'LAB' ? 0.72 : 0) : carPosition;
      if (current.mode === 'algorithm' && longLab && visible && visible.sequence > 1) current.orbit.distance = THREE.MathUtils.lerp(current.orbit.distance, 57, 0.025);
      if (current.mode === 'free') desired.add(current.orbit.pan);
      current.focus.lerp(desired, 0.09);
      const { az, el, distance } = current.orbit;
      const active = current.mode === 'top' ? top : perspective;
      if (current.mode === 'top') { const aspect = Math.max(1, element.clientWidth) / Math.max(1, element.clientHeight); const topDistance = visible?.source === 'LAB' && visible.run_id.includes('skidpad') ? Math.max(distance, 53) : distance; top.left = -topDistance * aspect / 2; top.right = topDistance * aspect / 2; top.top = topDistance / 2; top.bottom = -topDistance / 2; top.updateProjectionMatrix(); top.position.copy(current.focus).add(new THREE.Vector3(0, 100, 0.001)); top.lookAt(current.focus); }
      else if (current.mode === 'driver' && visible) { const yaw = visible.vehicle.yaw_rad; perspective.position.copy(carPosition).add(new THREE.Vector3(Math.cos(yaw) * 0.62, 1.15, -Math.sin(yaw) * 0.62)); perspective.lookAt(perspective.position.clone().add(new THREE.Vector3(Math.cos(yaw) * 10, -0.1, -Math.sin(yaw) * 10))); }
      else if (current.mode === 'chase' && visible) { const yaw = visible.vehicle.yaw_rad; const target = carPosition.clone().add(new THREE.Vector3(-Math.cos(yaw) * 11, 7.5, Math.sin(yaw) * 11)); perspective.position.lerp(target, 0.09); perspective.lookAt(carPosition); }
      else { perspective.position.copy(current.focus).add(new THREE.Vector3(Math.cos(az) * Math.cos(el) * distance, Math.sin(el) * distance, Math.sin(az) * Math.cos(el) * distance)); perspective.lookAt(current.focus); }
      if (current.animationStage) { const fraction = THREE.MathUtils.clamp((now - current.stageStarted) / 680, 0, 1); const stage = current.animationStage; const key = stage === 2 ? 'delaunay' : stage === 3 ? 'accepted' : stage === 4 ? 'gates' : stage === 5 ? 'candidates' : stage === 7 ? 'selected' : null; if (key) current.graph[key].object.geometry.setDrawRange(0, Math.floor(current.graph[key].count * fraction) * 2); }
      current.labels.forEach(({ element: label, point }) => { const projected = point.clone().project(active); const inView = projected.z > -1 && projected.z < 1 && Math.abs(projected.x) < 1 && Math.abs(projected.y) < 1; label.style.display = inView ? 'block' : 'none'; if (inView) { label.style.left = `${(projected.x * 0.5 + 0.5) * element.clientWidth}px`; label.style.top = `${(-projected.y * 0.5 + 0.5) * element.clientHeight}px`; } });
      renderer.render(scene, active); animation = requestAnimationFrame(draw);
    };
    animation = requestAnimationFrame(draw);
    return () => { cancelAnimationFrame(animation); resize.disconnect(); renderer.domElement.removeEventListener('pointerdown', onDown); renderer.domElement.removeEventListener('pointermove', onMove); renderer.domElement.removeEventListener('pointerup', onUp); renderer.domElement.removeEventListener('wheel', onWheel); renderer.domElement.removeEventListener('contextmenu', onContext); dispose(content); ground.geometry.dispose(); (ground.material as THREE.Material).dispose(); renderer.dispose(); labelsHost.remove(); renderer.domElement.remove(); state.current = null; };
  }, []);

  useEffect(() => {
    const current = state.current; if (!current) return;
    current.frame = frame; current.selection = selection; current.mode = cameraMode;
    if (current.animationStage !== animationStage) { current.animationStage = animationStage; current.stageStarted = performance.now(); }
    const clear = () => { Object.values(current.graph).forEach((pool) => updateLines(pool, [])); [current.course, current.courseBase, current.detections, current.detectionBase].forEach((mesh) => { mesh.count = 0; }); current.labels.forEach(({ element }) => element.remove()); current.labels = []; current.lookahead.visible = current.obstacle.visible = current.stop.visible = current.envelope.visible = current.fov.visible = current.disk.visible = false; current.rings.forEach((ring) => { ring.visible = false; }); };
    clear(); current.car.visible = !!frame; if (!frame) return;
    const host = current.renderer.domElement.parentElement!;
    const labelsHost = host.querySelector('.mc-scene-labels')!;
    const addLabel = (name: string, point: Point, className = '') => { if (current.labels.length >= 36) return; const label = document.createElement('div'); label.className = `mc-world-label ${className}`; label.textContent = name; labelsHost.appendChild(label); current.labels.push({ element: label, point: point3(point, 0.8) }); };
    const dummy = new THREE.Object3D();
    const fillCones = (cones: Cone[], mesh: THREE.InstancedMesh, base: THREE.InstancedMesh, faded: boolean) => {
      const count = Math.min(cones.length, mesh.instanceMatrix.count);
      mesh.count = base.count = count;
      mesh.userData = { kind: 'cone', ids: cones.slice(0, count).map((cone) => cone.id) };
      for (let i = 0; i < count; i++) { const cone = cones[i]; const special = coneColor(cone.class) === VISUAL_COLORS.ORANGE; const scale = faded && !special ? 0.78 : 1; dummy.position.copy(point3(cone, 0.47)); dummy.scale.setScalar(scale); dummy.updateMatrix(); mesh.setMatrixAt(i, dummy.matrix); mesh.setColorAt(i, new THREE.Color(coneColor(cone.class)).multiplyScalar(faded && !special ? 0.45 : 1)); dummy.position.y = 0.02; dummy.scale.setScalar(scale); dummy.updateMatrix(); base.setMatrixAt(i, dummy.matrix); }
      mesh.instanceMatrix.needsUpdate = base.instanceMatrix.needsUpdate = true; if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    };
    const contextCones = frame.source === 'DEMO' || frame.source === 'LAB' ? frame.visualization?.course_cones ?? [] : layers.tracked && frame.perception.tracked_cones?.length ? frame.perception.tracked_cones : layers.groundTruth ? frame.perception.ground_truth_cones ?? [] : [];
    fillCones(contextCones, current.course, current.courseBase, frame.source !== 'LAB' || frame.sequence > 0);
    if (layers.detections) fillCones(frame.perception.cones, current.detections, current.detectionBase, false);
    if (contextCones.length) { const xs = contextCones.map((cone) => cone.x_m), ys = contextCones.map((cone) => cone.y_m); const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys); current.courseCenter.set((minX + maxX) / 2, 0, -(minY + maxY) / 2); if (frame.sequence === 0) current.orbit.distance = frame.source === 'LAB' && frame.run_id.includes('skidpad') ? 44 : Math.min(220, Math.max(55, Math.hypot(maxX - minX, maxY - minY) * 1.2)); }
    else current.courseCenter.copy(point3(frame.vehicle, 0));
    const coneMap = new Map(frame.perception.cones.map((cone) => [cone.id, cone]));
    if (layers.delaunay) { const result = edgeSegments(frame.planning.delaunay_edges, coneMap); updateLines(current.graph.delaunay, result.segments, result.ids, 'edge'); }
    if (layers.gabriel) { for (const [key, accepted] of [['accepted', true], ['rejected', false]] as const) { const result = edgeSegments(frame.planning.gabriel_edges.filter((edge) => edge.accepted === accepted), coneMap); updateLines(current.graph[key], result.segments, result.ids, 'edge'); } }
    if (layers.gates) { const segments: Array<[Point, Point]> = []; const ids: string[] = []; const nearby = [...frame.planning.gates].sort((a, b) => Math.hypot(a.midpoint.x_m - frame.vehicle.x_m, a.midpoint.y_m - frame.vehicle.y_m) - Math.hypot(b.midpoint.x_m - frame.vehicle.x_m, b.midpoint.y_m - frame.vehicle.y_m)).slice(0, 3); frame.planning.gates.forEach((gate) => { const angle = gate.heading_rad ?? frame.vehicle.yaw_rad; const nx = -Math.sin(angle) * gate.width_m / 2, ny = Math.cos(angle) * gate.width_m / 2; segments.push([{ x_m: gate.midpoint.x_m + nx, y_m: gate.midpoint.y_m + ny }, { x_m: gate.midpoint.x_m - nx, y_m: gate.midpoint.y_m - ny }]); ids.push(gate.id); if (nearby.includes(gate)) addLabel(`${gate.id} · ${gate.valid ? 'VALID' : gate.reason ?? 'UNQUALIFIED'}`, gate.midpoint, 'gate'); }); updateLines(current.graph.gates, segments, ids, 'gate'); }
    if (layers.candidates) { const segments: Array<[Point, Point]> = [], ids: string[] = []; frame.planning.candidate_paths.forEach((candidate) => { for (const segment of polySegments(candidate.points)) { segments.push(segment); ids.push(candidate.id); } if (candidate.points.length) addLabel(`${candidate.id} · ${candidate.total_cost.toFixed(2)}`, candidate.points.at(-1)!, candidate.selected ? 'lookahead' : 'gate'); }); updateLines(current.graph.candidates, segments, ids, 'candidate'); }
    if (layers.selected) { updateLines(current.graph.selected, polySegments(frame.planning.selected_path)); if (frame.planning.lookahead) { current.lookahead.visible = true; current.lookahead.position.copy(point3(frame.planning.lookahead, 0.35)); current.lookahead.userData = { kind: 'lookahead', id: 'lookahead' }; addLabel('LOOKAHEAD', frame.planning.lookahead, 'lookahead'); } }
    if (frame.visualization?.trail) updateLines(current.graph.trail, polySegments(frame.visualization.trail));
    if (frame.visualization?.course_guide) updateLines(current.graph.guide, polySegments(frame.visualization.course_guide));
    if (layers.boundaries) {
      updateLines(current.graph.boundaryLeft, polySegments(frame.visualization?.boundary_left ?? []));
      updateLines(current.graph.boundaryRight, polySegments(frame.visualization?.boundary_right ?? []));
    }
    // LAB never renders its scenario centreline; only observed planner output is shown.
    frame.visualization?.special_markers?.forEach((marker) => addLabel(marker.label, marker, 'special'));
    current.car.position.copy(point3(frame.vehicle, 0.17)); current.car.rotation.y = frame.vehicle.yaw_rad; current.car.scale.setScalar(frame.source === 'LAB' ? 1.22 : 1); current.frontWheels.forEach((wheel) => { wheel.rotation.y = -(frame.vehicle.measured_steering_rad ?? 0); });
    const obstacle = frame.perception.obstacles?.[0];
    if (layers.safety && obstacle) { current.obstacle.visible = true; current.obstacle.position.copy(point3(obstacle, 0.48)); current.obstacle.scale.set(obstacle.radius_m * 2, 1, obstacle.radius_m * 2); current.obstacle.userData.id = obstacle.id; updateLines(current.graph.distance, [[frame.vehicle, obstacle]]); addLabel(`${obstacle.id} · ${obstacle.distance_m?.toFixed(1) ?? '—'} m`, obstacle, 'obstacle'); const stopDistance = frame.safety.obstacle?.stopping_distance_m; if (stopDistance != null) { const distance = Math.max(0.5, stopDistance); current.envelope.visible = true; current.envelope.position.copy(point3(frame.vehicle, 0.065)).add(new THREE.Vector3(Math.cos(frame.vehicle.yaw_rad) * distance / 2, 0, -Math.sin(frame.vehicle.yaw_rad) * distance / 2)); current.envelope.rotation.z = 0; current.envelope.rotation.y = frame.vehicle.yaw_rad; current.envelope.scale.set(distance, 2.3, 1); (current.envelope.material as THREE.MeshBasicMaterial).color.setHex(frame.safety.obstacle?.brake_trigger ? VISUAL_COLORS.REJECTED : VISUAL_COLORS.ACCEPTED); } }
    if (layers.safety && frame.safety.obstacle?.predicted_stop) { current.stop.visible = true; current.stop.position.copy(point3(frame.safety.obstacle.predicted_stop, 0.08)); addLabel('PREDICTED STOP', frame.safety.obstacle.predicted_stop, 'obstacle'); }
    if (layers.sensor && frame.perception.sensor_range_m && frame.perception.sensor_fov_rad) { const range = frame.perception.sensor_range_m, angle = frame.perception.sensor_fov_rad; if (range !== current.fovRange || angle !== current.fovAngle) { const points = [new THREE.Vector3(0, 0, 0)]; for (let i = 0; i <= 40; i++) { const theta = -angle / 2 + i * angle / 40; points.push(new THREE.Vector3(Math.cos(theta) * range, 0, -Math.sin(theta) * range)); } const geometry = new THREE.BufferGeometry().setFromPoints(points); geometry.setIndex(Array.from({ length: 40 }, (_, i) => [0, i + 1, i + 2]).flat()); current.fov.geometry.dispose(); current.fov.geometry = geometry; current.fovRange = range; current.fovAngle = angle; } current.fov.visible = true; current.fov.position.copy(point3(frame.vehicle, 0.025)); current.fov.rotation.y = frame.vehicle.yaw_rad; }
    if (layers.gabriel && (selection.kind === 'edge' || animationStage === 3)) { const edge = selection.kind === 'edge' ? frame.planning.gabriel_edges.find((item) => item.id === selection.id) : frame.planning.gabriel_edges[Math.floor(performance.now() / 360) % frame.planning.gabriel_edges.length]; const a = edge && coneMap.get(edge.a), b = edge && coneMap.get(edge.b); if (a && b) { const radius = Math.hypot(a.x_m - b.x_m, a.y_m - b.y_m) / 2; current.disk.visible = true; current.disk.position.set((a.x_m + b.x_m) / 2, 0.07, -(a.y_m + b.y_m) / 2); current.disk.scale.setScalar(radius); (current.disk.material as THREE.MeshBasicMaterial).color.setHex(edge?.accepted === undefined ? VISUAL_COLORS.DELAUNAY : edge.accepted ? VISUAL_COLORS.ACCEPTED : VISUAL_COLORS.REJECTED); if (edge?.blocking_cone_id) { const blocker = coneMap.get(edge.blocking_cone_id); if (blocker) addLabel(`BLOCKER ${blocker.id}`, blocker, 'obstacle'); } } }
    const skidpad = frame.planning.skidpad; const demoPad = frame.visualization?.skidpad;
    const centers = skidpad ? [[skidpad.left_center, skidpad.left_radius_m], [skidpad.right_center, skidpad.right_radius_m]] as const : demoPad ? [[demoPad.left_center, demoPad.radius_m], [demoPad.right_center, demoPad.radius_m]] as const : [];
    centers.forEach(([center, radius], index) => { if (center && radius) { const ring = current.rings[index]; ring.visible = true; ring.position.copy(point3(center, 0.06)); ring.scale.set(radius, radius, 1); addLabel(index === 0 ? 'LEFT LOOP' : 'RIGHT LOOP', center, 'loop'); const winding = index === 0 ? skidpad?.w_left ?? demoPad?.w_left : skidpad?.w_right ?? demoPad?.w_right; const fraction = Math.min(1, Math.abs(winding ?? 0) % 1); const arc: Point[] = []; const samples = Math.ceil(fraction * 72); for (let sample = 0; sample <= samples; sample++) { const angle = -Math.PI / 2 + Math.sign(winding ?? 1) * (sample / 72) * Math.PI * 2; arc.push({ x_m: center.x_m + Math.cos(angle) * radius, y_m: center.y_m + Math.sin(angle) * radius }); } updateLines(current.graph[index === 0 ? 'leftArc' : 'rightArc'], polySegments(arc)); } });
    const nearest = centers.map(([center]) => center ? Math.hypot(frame.vehicle.x_m - center.x_m, frame.vehicle.y_m - center.y_m) : Infinity);
    const closest = nearest[0] <= nearest[1] ? centers[0]?.[0] : centers[1]?.[0];
    if (closest && Math.min(...nearest) < 18) updateLines(current.graph.angle, [[closest, frame.vehicle]]);
    if (skidpad?.branches?.length) { const valid: Array<[Point, Point]> = [], invalid: Array<[Point, Point]> = []; skidpad.branches.forEach((branch) => { const segments = polySegments(branch.points); (branch.valid ? valid : invalid).push(...segments); if (branch.points.length) addLabel(`${branch.name} ${branch.selected ? '✓ SELECTED' : branch.valid ? 'AVAILABLE' : 'REJECTED'}`, branch.points.at(-1)!, branch.selected ? 'lookahead' : branch.valid ? 'gate' : 'obstacle'); }); updateLines(current.graph.branchValid, valid); updateLines(current.graph.branchInvalid, invalid); }
  }, [frame, layers, cameraMode, selection, animationStage]);

  return <div className="mission-scene" ref={host} aria-label="Mission Control 3D digital twin" />;
}
