import { useEffect, useRef } from 'react';
import type { AutonomyEngine } from '../engine/engine';
import { KART_HALF_WIDTH, OBSTACLE_CLEARANCE } from '../engine/avoidance';
import { pointToSegmentDistance } from '../engine/math';
import type { EditTool, InspectionSelection, PlanningSnapshot, Point2D } from '../engine/types';

interface Props {
  engine: AutonomyEngine;
  selection: InspectionSelection;
  tool: EditTool;
  onSelect: (selection: InspectionSelection) => void;
  onWorldEdit: () => void;
}

interface ViewTransform {
  scale: number;
  ox: number;
  oy: number;
  width: number;
  height: number;
}

function createTransform(snapshot: PlanningSnapshot, width: number, height: number): ViewTransform {
  const points = snapshot.world.cones.length > 0 ? snapshot.world.cones : [snapshot.vehicle];
  const minX = Math.min(...points.map((point) => point.x)) - 2.8;
  const maxX = Math.max(...points.map((point) => point.x)) + 2.8;
  const minY = Math.min(...points.map((point) => point.y)) - 2.8;
  const maxY = Math.max(...points.map((point) => point.y)) + 2.8;
  const scale = Math.min(width / (maxX - minX), height / (maxY - minY));
  return { scale, ox: (width - (maxX - minX) * scale) / 2 - minX * scale, oy: (height - (maxY - minY) * scale) / 2 + maxY * scale, width, height };
}

function toScreen(point: Point2D, view: ViewTransform): Point2D {
  return { x: view.ox + point.x * view.scale, y: view.oy - point.y * view.scale };
}

function toWorld(point: Point2D, view: ViewTransform): Point2D {
  return { x: (point.x - view.ox) / view.scale, y: (view.oy - point.y) / view.scale };
}

function line(ctx: CanvasRenderingContext2D, points: Point2D[], view: ViewTransform): void {
  if (points.length === 0) return;
  const first = toScreen(points[0], view);
  ctx.beginPath();
  ctx.moveTo(first.x, first.y);
  for (let i = 1; i < points.length; i += 1) {
    const point = toScreen(points[i], view);
    ctx.lineTo(point.x, point.y);
  }
  ctx.stroke();
}

function selected(selection: InspectionSelection, kind: InspectionSelection['kind'], id: string): boolean {
  return selection.kind === kind && selection.id === id;
}

function drawCone(ctx: CanvasRenderingContext2D, point: Point2D, color: string, visible: boolean, active: boolean, view: ViewTransform): void {
  const screen = toScreen(point, view);
  const size = Math.max(4, view.scale * 0.16);
  ctx.beginPath();
  ctx.moveTo(screen.x, screen.y - size);
  ctx.lineTo(screen.x - size * 0.72, screen.y + size);
  ctx.lineTo(screen.x + size * 0.72, screen.y + size);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.globalAlpha = visible ? 1 : 0.35;
  ctx.fill();
  ctx.globalAlpha = 1;
  if (active) {
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 3;
    ctx.stroke();
  }
}

function drawKart(ctx: CanvasRenderingContext2D, snapshot: PlanningSnapshot, view: ViewTransform, active: boolean): void {
  const vehicle = snapshot.vehicle;
  const center = toScreen(vehicle, view);
  ctx.save();
  ctx.translate(center.x, center.y);
  ctx.rotate(-vehicle.heading);
  const length = view.scale * 1.25;
  const width = view.scale * 0.72;
  ctx.fillStyle = '#e7edf3';
  ctx.strokeStyle = active ? '#ffcf52' : '#071018';
  ctx.lineWidth = active ? 4 : 2;
  ctx.beginPath();
  ctx.roundRect(-length / 2, -width / 2, length, width, Math.max(3, view.scale * 0.08));
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = '#fb5c45';
  ctx.fillRect(length * 0.18, -width * 0.38, length * 0.2, width * 0.76);
  const wheelLength = view.scale * 0.3;
  const wheelWidth = Math.max(3, view.scale * 0.09);
  const wheelX = length * 0.32;
  for (const side of [-1, 1]) {
    ctx.save();
    ctx.translate(wheelX, side * width * 0.52);
    ctx.rotate(-vehicle.steeringAngle);
    ctx.fillStyle = '#000000';
    ctx.fillRect(-wheelLength / 2, -wheelWidth / 2, wheelLength, wheelWidth);
    ctx.restore();
    ctx.fillRect(-length * 0.38 - wheelLength / 2, side * width * 0.52 - wheelWidth / 2, wheelLength, wheelWidth);
  }
  ctx.restore();
}

function drawWorld(
  ctx: CanvasRenderingContext2D,
  snapshot: PlanningSnapshot,
  selection: InspectionSelection,
  view: ViewTransform,
): void {
  ctx.clearRect(0, 0, view.width, view.height);
  ctx.fillStyle = '#000000';
  ctx.fillRect(0, 0, view.width, view.height);

  ctx.strokeStyle = 'rgba(175, 188, 195, 0.055)';
  ctx.lineWidth = 1;
  const grid = 2;
  for (let x = -20; x <= 20; x += grid) line(ctx, [{ x, y: -15 }, { x, y: 15 }], view);
  for (let y = -14; y <= 14; y += grid) line(ctx, [{ x: -20, y }, { x: 20, y }], view);

  const vehicle = snapshot.sensor.vehicleAtCalculation;
  const fovLeft = vehicle.heading + snapshot.sensor.fov / 2;
  const fovRight = vehicle.heading - snapshot.sensor.fov / 2;
  const fovPoints = [
    vehicle,
    { x: vehicle.x + Math.cos(fovRight) * snapshot.sensor.range, y: vehicle.y + Math.sin(fovRight) * snapshot.sensor.range },
  ];
  const arcSamples = 24;
  for (let i = 0; i <= arcSamples; i += 1) {
    const angle = fovRight + (i / arcSamples) * (fovLeft - fovRight);
    fovPoints.push({ x: vehicle.x + Math.cos(angle) * snapshot.sensor.range, y: vehicle.y + Math.sin(angle) * snapshot.sensor.range });
  }
  fovPoints.push(vehicle);
  ctx.fillStyle = 'rgba(71, 201, 255, 0.055)';
  ctx.strokeStyle = 'rgba(71, 201, 255, 0.32)';
  ctx.lineWidth = 1;
  const start = toScreen(fovPoints[0], view);
  ctx.beginPath();
  ctx.moveTo(start.x, start.y);
  for (let i = 1; i < fovPoints.length; i += 1) {
    const point = toScreen(fovPoints[i], view);
    ctx.lineTo(point.x, point.y);
  }
  ctx.closePath();
  ctx.fill();
  ctx.stroke();

  ctx.strokeStyle = 'rgba(147, 189, 208, 0.28)';
  ctx.lineWidth = Math.max(1, view.scale * 0.04);
  line(ctx, snapshot.world.trail, view);

  for (const triangle of snapshot.filtering.exteriorTriangles) {
    const isActive = selected(selection, 'triangle', triangle.id);
    ctx.strokeStyle = isActive ? '#ffcf52' : 'rgba(242, 91, 83, 0.34)';
    ctx.fillStyle = isActive ? 'rgba(255, 207, 82, 0.2)' : 'rgba(242, 91, 83, 0.035)';
    ctx.lineWidth = isActive ? 4 : 1;
    ctx.setLineDash([5, 4]);
    const vertices = [...triangle.vertices, triangle.vertices[0]];
    line(ctx, vertices, view);
    const first = toScreen(vertices[0], view);
    ctx.beginPath();
    ctx.moveTo(first.x, first.y);
    vertices.slice(1).forEach((vertex) => { const p = toScreen(vertex, view); ctx.lineTo(p.x, p.y); });
    ctx.fill();
  }
  ctx.setLineDash([]);
  for (const triangle of snapshot.filtering.interiorTriangles) {
    const isActive = selected(selection, 'triangle', triangle.id);
    ctx.strokeStyle = isActive ? '#ffcf52' : 'rgba(69, 190, 175, 0.62)';
    ctx.fillStyle = isActive ? 'rgba(255, 207, 82, 0.23)' : 'rgba(69, 190, 175, 0.055)';
    ctx.lineWidth = isActive ? 4 : 1.2;
    const vertices = [...triangle.vertices, triangle.vertices[0]];
    line(ctx, vertices, view);
    const first = toScreen(vertices[0], view);
    ctx.beginPath();
    ctx.moveTo(first.x, first.y);
    vertices.slice(1).forEach((vertex) => { const p = toScreen(vertex, view); ctx.lineTo(p.x, p.y); });
    ctx.fill();
  }

  for (const edge of snapshot.edges.crossTrackEdges) {
    const isActive = selected(selection, 'edge', edge.id) ||
      (selection.kind === 'midpoint' && snapshot.midpoints.find((midpoint) => midpoint.id === selection.id)?.sourceEdgeId === edge.id);
    ctx.strokeStyle = isActive ? '#ffcf52' : 'rgba(244, 176, 67, 0.72)';
    ctx.lineWidth = isActive ? 5 : 2;
    line(ctx, [edge.a, edge.b], view);
  }

  if (snapshot.avoidance.active) {
    ctx.strokeStyle = 'rgba(132, 151, 160, 0.52)';
    ctx.lineWidth = 1.3;
    ctx.setLineDash([5, 5]);
    line(ctx, snapshot.interpolation.basePath, view);
    ctx.setLineDash([]);
    for (const candidate of snapshot.avoidance.candidates) {
      const isSelected = candidate.side === snapshot.avoidance.selectedSide;
      ctx.strokeStyle = isSelected
        ? snapshot.avoidance.phase === 'BLOCKED' ? '#ef6b61' : '#55e6c7'
        : candidate.feasible ? 'rgba(100, 148, 170, 0.42)' : 'rgba(241, 100, 87, 0.42)';
      ctx.lineWidth = isSelected ? 3.5 : 1.1;
      if (!isSelected) ctx.setLineDash([3, 5]);
      line(ctx, candidate.path, view);
      ctx.setLineDash([]);
    }
  }
  ctx.strokeStyle = snapshot.avoidance.active
    ? snapshot.avoidance.phase === 'BLOCKED' ? '#ef6b61' : '#55e6c7'
    : '#f7f8f2';
  ctx.lineWidth = snapshot.avoidance.active ? 3 : 2.4;
  line(ctx, snapshot.interpolation.path, view);
  for (const point of snapshot.midpoints) {
    const isActive = selected(selection, 'midpoint', point.id);
    const screen = toScreen(point, view);
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, isActive ? 7 : 3.8, 0, Math.PI * 2);
    ctx.fillStyle = isActive ? '#ffcf52' : '#ffffff';
    ctx.fill();
  }

  if (snapshot.controller.target) {
    ctx.strokeStyle = '#ff5f4a';
    ctx.lineWidth = 2;
    line(ctx, [snapshot.controller.vehicleAtCalculation, snapshot.controller.target], view);
    const target = toScreen(snapshot.controller.target, view);
    ctx.beginPath();
    ctx.arc(target.x, target.y, selected(selection, 'target', 'lookahead') ? 10 : 7, 0, Math.PI * 2);
    ctx.strokeStyle = '#ff5f4a';
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(target.x - 10, target.y);
    ctx.lineTo(target.x + 10, target.y);
    ctx.moveTo(target.x, target.y - 10);
    ctx.lineTo(target.x, target.y + 10);
    ctx.stroke();
  }

  if (selection.kind === 'path') {
    const pathPoint = snapshot.interpolation.path[Number(selection.id)];
    if (pathPoint) {
      const screen = toScreen(pathPoint, view);
      ctx.beginPath();
      ctx.arc(screen.x, screen.y, 7, 0, Math.PI * 2);
      ctx.fillStyle = '#ffcf52';
      ctx.fill();
      ctx.strokeStyle = '#071017';
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }

  const visibleIds = new Set(snapshot.sensor.visibleCones.map((cone) => cone.id));
  const selectedMidpoint = selection.kind === 'midpoint' ? snapshot.midpoints.find((item) => item.id === selection.id) : null;
  for (const cone of snapshot.world.cones) {
    const parentActive = selectedMidpoint && (selectedMidpoint.blue.id === cone.id || selectedMidpoint.yellow.id === cone.id);
    drawCone(ctx, cone, cone.side === 'BLUE' ? '#378ee6' : '#ffd34e', visibleIds.has(cone.id), selected(selection, 'cone', cone.id) || Boolean(parentActive), view);
  }
  for (const obstacle of snapshot.world.obstacles) {
    const screen = toScreen(obstacle, view);
    const isAvoided = snapshot.avoidance.obstacle?.obstacle.id === obstacle.id;
    const isActive = selected(selection, 'obstacle', obstacle.id);
    if (isAvoided || isActive) {
      ctx.strokeStyle = isActive ? '#ffffff' : 'rgba(255, 118, 104, 0.65)';
      ctx.lineWidth = isActive ? 3 : 1.5;
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.arc(screen.x, screen.y, (obstacle.radius + KART_HALF_WIDTH + OBSTACLE_CLEARANCE) * view.scale, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.fillStyle = '#ed5f59';
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, obstacle.radius * view.scale, 0, Math.PI * 2);
    ctx.fill();
  }
  drawKart(ctx, snapshot, view, selected(selection, 'kart', 'vehicle'));

  ctx.fillStyle = 'rgba(0, 0, 0, 0.9)';
  ctx.fillRect(12, 12, 238, snapshot.avoidance.active ? 76 : 56);
  ctx.fillStyle = '#eaf2f6';
  ctx.font = '600 12px "JetBrains Mono Variable", ui-monospace, monospace';
  ctx.fillText(`t ${snapshot.simulationTime.toFixed(2)} s  |  frame ${snapshot.frame}`, 24, 34);
  ctx.fillStyle = '#ffffff';
  ctx.fillText(`${snapshot.sensor.visibleCones.length} visible  |  plan #${snapshot.cycle}`, 24, 54);
  if (snapshot.avoidance.active) {
    ctx.fillStyle = snapshot.avoidance.phase === 'MANEUVERING' ? '#55e6c7' : '#ef6b61';
    ctx.fillText(`${snapshot.avoidance.phase} · ${snapshot.avoidance.selectedSide} ROUTE`, 24, 76);
  }
}

function hitTest(point: Point2D, snapshot: PlanningSnapshot, view: ViewTransform): InspectionSelection {
  const world = toWorld(point, view);
  if (Math.hypot(world.x - snapshot.vehicle.x, world.y - snapshot.vehicle.y) < 0.75) return { kind: 'kart', id: 'vehicle' };
  if (snapshot.controller.target && Math.hypot(world.x - snapshot.controller.target.x, world.y - snapshot.controller.target.y) < 0.36) return { kind: 'target', id: 'lookahead' };
  const cone = snapshot.world.cones.find((item) => Math.hypot(world.x - item.x, world.y - item.y) < 0.34);
  if (cone) return { kind: 'cone', id: cone.id };
  const obstacle = snapshot.world.obstacles.find((item) => Math.hypot(world.x - item.x, world.y - item.y) < item.radius + 0.2);
  if (obstacle) return { kind: 'obstacle', id: obstacle.id };
  const midpoint = snapshot.midpoints.find((item) => Math.hypot(world.x - item.x, world.y - item.y) < 0.3);
  if (midpoint) return { kind: 'midpoint', id: midpoint.id };
  let bestPath = { index: -1, distance: 0.24 };
  snapshot.interpolation.path.forEach((item) => {
    const d = Math.hypot(world.x - item.x, world.y - item.y);
    if (d < bestPath.distance) bestPath = { index: item.index, distance: d };
  });
  if (bestPath.index >= 0) return { kind: 'path', id: String(bestPath.index) };
  const edge = snapshot.edges.uniqueEdges.find((item) => pointToSegmentDistance(world, item.a, item.b) < 0.18);
  if (edge) return { kind: 'edge', id: edge.id };
  const triangle = [...snapshot.delaunay.triangles].reverse().find((item) => {
    const [a, b, c] = item.vertices;
    const area = (p1: Point2D, p2: Point2D, p3: Point2D) => Math.abs((p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y)) / 2);
    return Math.abs(area(a, b, c) - area(world, a, b) - area(world, b, c) - area(world, c, a)) < 0.02;
  });
  return triangle ? { kind: 'triangle', id: triangle.id } : { kind: 'kart', id: 'vehicle' };
}

export function SimulationCanvas({ engine, selection, tool, onSelect, onWorldEdit }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const selectionRef = useRef(selection);
  selectionRef.current = selection;
  const toolRef = useRef(tool);
  toolRef.current = tool;
  const viewRef = useRef<ViewTransform | null>(null);

  useEffect(() => {
    let animationFrame = 0;
    const render = (wallTime: number) => {
      const renderStart = performance.now();
      const canvas = canvasRef.current;
      if (canvas) {
        const rect = canvas.getBoundingClientRect();
        // Engineering geometry stays crisp without paying the full cost of 3×/4× displays.
        const ratio = Math.min(window.devicePixelRatio || 1, 1.5);
        const width = Math.max(1, Math.round(rect.width));
        const height = Math.max(1, Math.round(rect.height));
        if (canvas.width !== width * ratio || canvas.height !== height * ratio) {
          canvas.width = width * ratio;
          canvas.height = height * ratio;
        }
        const ctx = canvas.getContext('2d');
        if (ctx) {
          ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
          const snapshot = engine.getSnapshot();
          const view = createTransform(snapshot, width, height);
          viewRef.current = view;
          drawWorld(ctx, snapshot, selectionRef.current, view);
        }
      }
      engine.recordRenderCycle(wallTime, performance.now() - renderStart);
      animationFrame = requestAnimationFrame(render);
    };
    animationFrame = requestAnimationFrame(render);
    return () => cancelAnimationFrame(animationFrame);
  }, [engine]);

  const onPointerDown = (event: React.PointerEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    const view = viewRef.current;
    if (!canvas || !view) return;
    const rect = canvas.getBoundingClientRect();
    const point = { x: event.clientX - rect.left, y: event.clientY - rect.top };
    const snapshot = engine.getSnapshot();
    if (toolRef.current === 'inspect') onSelect(hitTest(point, snapshot, view));
    else {
      engine.editWorld(toolRef.current, toWorld(point, view));
      onWorldEdit();
    }
  };

  return <canvas ref={canvasRef} className="simulation-canvas" onPointerDown={onPointerDown} aria-label="Live simulation world" />;
}
