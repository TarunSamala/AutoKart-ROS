export type ConeSide = 'BLUE' | 'YELLOW';

export interface Point2D {
  x: number;
  y: number;
}

export interface Cone extends Point2D {
  id: string;
  side: ConeSide;
}

export interface Obstacle extends Point2D {
  id: string;
  radius: number;
}

export interface VehicleState extends Point2D {
  heading: number;
  velocity: number;
  acceleration: number;
  steeringAngle: number;
  yawRate: number;
  speedCommand: number;
}

export interface ConeDetection {
  cone: Cone;
  dx: number;
  dy: number;
  longitudinal: number;
  lateral: number;
  distance: number;
  bearing: number;
  rangePass: boolean;
  fovPass: boolean;
  forwardPass: boolean;
  detected: boolean;
}

export interface ObstacleDetection {
  obstacle: Obstacle;
  dx: number;
  dy: number;
  longitudinal: number;
  lateral: number;
  distance: number;
  bearing: number;
  detected: boolean;
  unsafe: boolean;
}

export interface SensorSnapshot {
  vehicleAtCalculation: VehicleState;
  range: number;
  fov: number;
  worldConeCount: number;
  detections: ConeDetection[];
  visibleCones: Cone[];
  visibleObstacles: ObstacleDetection[];
  blueVisible: number;
  yellowVisible: number;
}

export interface Triangle {
  id: string;
  indices: [number, number, number];
  vertices: [Cone, Cone, Cone];
  centroid: Point2D;
  interior: boolean;
  filterReason: string;
}

export type EdgeClassification =
  | 'VALID CROSS-TRACK EDGE'
  | 'REJECT — SAME SIDE'
  | 'REJECT — WIDTH';

export interface Edge {
  id: string;
  aIndex: number;
  bIndex: number;
  a: Cone;
  b: Cone;
  triangleIds: string[];
  length: number;
  crossTrack: boolean;
  widthPass: boolean;
  classification: EdgeClassification;
}

export interface Midpoint extends Point2D {
  id: string;
  sourceEdgeId: string;
  blue: Cone;
  yellow: Cone;
}

export interface OrderingCheck {
  candidateId: string;
  dx: number;
  dy: number;
  forwardProgress: boolean;
  distance: number;
  headingChange: number;
  connectivityPass: boolean;
  score: number;
  selected: boolean;
}

export interface OrderedMidpoint extends Midpoint {
  order: number;
  distanceFromPrevious: number;
  headingFromPrevious: number;
  selectionReason: OrderingCheck;
}

export interface PathPoint extends Point2D {
  index: number;
  s: number;
  sourceSegment: number;
}

export type AvoidanceSide = 'LEFT' | 'RIGHT';
export type AvoidancePhase = 'CLEAR' | 'MANEUVERING' | 'BLOCKED';

export interface AvoidanceCandidate {
  side: AvoidanceSide;
  feasible: boolean;
  desiredOffset: number;
  maxAppliedOffset: number;
  corridorWidth: number;
  minimumObstacleClearance: number;
  minimumBoundaryClearance: number;
  requiredObstacleClearance: number;
  requiredBoundaryClearance: number;
  score: number;
  reason: string;
  path: PathPoint[];
}

export interface AvoidanceSnapshot {
  active: boolean;
  phase: AvoidancePhase;
  trackingSource: 'LIVE SENSOR' | 'COMMITTED TRACK' | null;
  obstacle: ObstacleDetection | null;
  obstaclePathIndex: number;
  obstaclePathS: number;
  obstacleLateral: number;
  currentClearance: number;
  brakingDistance: number;
  candidates: AvoidanceCandidate[];
  selectedSide: AvoidanceSide | null;
  selectedPath: PathPoint[];
}

export interface ControllerSnapshot {
  vehicleAtCalculation: VehicleState;
  nearestIndex: number;
  nearest: PathPoint | null;
  targetIndex: number;
  target: PathPoint | null;
  desiredLookahead: number;
  dx: number;
  dy: number;
  longitudinal: number;
  lateral: number;
  lookaheadDistance: number;
  alpha: number;
  curvature: number;
  steeringAngle: number;
  crossTrackError: number;
  speedCommand: number;
  stopReason: DecisionState | null;
  holdReason: DecisionState | null;
  avoidanceActive: boolean;
  avoidanceSide: AvoidanceSide | null;
  avoidancePhase: AvoidancePhase;
}

export interface MovementSnapshot {
  dt: number;
  oldState: VehicleState;
  newState: VehicleState;
  yawRate: number;
  deltaHeading: number;
  deltaX: number;
  deltaY: number;
}

export interface PlanningTiming {
  sensorMs: number;
  delaunayMs: number;
  filterMs: number;
  edgeMs: number;
  midpointMs: number;
  orderMs: number;
  splineMs: number;
  avoidanceMs: number;
  controllerMs: number;
  physicsMs: number;
  renderMs: number;
  totalPlanMs: number;
}

export type LoopName = 'physics' | 'controller' | 'sensor' | 'planner' | 'ui' | 'render';

export interface LoopMetric {
  targetHz: number;
  measuredHz: number;
  executionMs: number;
  cycles: number;
}

export type LoopMetrics = Record<LoopName, LoopMetric>;

export interface HistorySample {
  time: number;
  steeringAngle: number;
  crossTrackError: number;
  velocity: number;
  visibleCones: number;
  plannerMs: number;
}

export type DecisionState =
  | 'FOLLOW PATH'
  | 'GO STRAIGHT'
  | 'STEER LEFT'
  | 'STEER RIGHT'
  | 'MANEUVER LEFT — OBSTACLE'
  | 'MANEUVER RIGHT — OBSTACLE'
  | 'STOP — NO SAFE OBSTACLE ROUTE'
  | 'STOP — INSUFFICIENT CONES'
  | 'STOP — NO CORRIDOR'
  | 'STOP — PATH LOST';

export interface DecisionTrace {
  decision: DecisionState;
  sensorCount: number;
  triangleCount: number;
  interiorCount: number;
  rejectedCount: number;
  crossTrackCount: number;
  midpointCount: number;
  pathCount: number;
  target: Point2D | null;
  alpha: number;
  curvature: number;
  steeringAngle: number;
  yawRate: number;
  newPose: VehicleState;
}

export interface PlanningSnapshot {
  timestamp: number;
  simulationTime: number;
  frame: number;
  cycle: number;
  controllerCycle: number;
  running: boolean;
  simulationSpeed: number;
  vehicle: VehicleState;
  world: {
    cones: Cone[];
    obstacles: Obstacle[];
    trail: Point2D[];
  };
  sensor: SensorSnapshot;
  delaunay: {
    points: Cone[];
    triangles: Triangle[];
    connectivity: [number, number, number][];
  };
  filtering: {
    interiorTriangles: Triangle[];
    exteriorTriangles: Triangle[];
    flags: boolean[];
    corridorPolygon: Point2D[];
  };
  edges: {
    uniqueEdges: Edge[];
    crossTrackEdges: Edge[];
  };
  midpoints: Midpoint[];
  orderedMidpoints: OrderedMidpoint[];
  orderingChecks: OrderingCheck[];
  interpolation: {
    cumulativeDistance: number[];
    segmentDistances: number[];
    basePath: PathPoint[];
    path: PathPoint[];
  };
  avoidance: AvoidanceSnapshot;
  controller: ControllerSnapshot;
  movement: MovementSnapshot;
  timing: PlanningTiming;
  loops: LoopMetrics;
  history: HistorySample[];
  trace: DecisionTrace;
}

export type InspectableKind =
  | 'kart'
  | 'cone'
  | 'obstacle'
  | 'triangle'
  | 'edge'
  | 'midpoint'
  | 'path'
  | 'target';

export interface InspectionSelection {
  kind: InspectableKind;
  id: string;
}

export type EditTool = 'inspect' | 'blue' | 'yellow' | 'obstacle' | 'erase';

export interface CoursePreset {
  id: string;
  name: string;
  cones: Cone[];
  obstacles: Obstacle[];
  start: VehicleState;
}
