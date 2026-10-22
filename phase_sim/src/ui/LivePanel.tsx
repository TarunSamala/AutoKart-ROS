import { useState, type ReactNode } from 'react';
import { WHEELBASE } from '../engine/controller';
import { radToDeg } from '../engine/math';
import { MAX_TRACK_WIDTH, MIN_TRACK_WIDTH } from '../engine/planner';
import type { ConeDetection, InspectionSelection, PlanningSnapshot } from '../engine/types';
import { degrees, direction, fixed, meters, pass, radians, seconds } from './format';

function Section({ title, children, open = false, badge }: { title: string; children: ReactNode; open?: boolean; badge?: string | number }) {
  return (
    <details className="calc-section" open={open}>
      <summary><span>{title}</span>{badge !== undefined && <span className="section-badge">{badge}</span>}</summary>
      <div className="section-body">{children}</div>
    </details>
  );
}

function Row({ label, value, testId }: { label: string; value: ReactNode; testId?: string }) {
  return <div className="calc-row"><span>{label}</span><output data-testid={testId}>{value}</output></div>;
}

function Status({ children, good = true }: { children: ReactNode; good?: boolean }) {
  return <div className={good ? 'result result-good' : 'result result-bad'}>{children}</div>;
}

function SensorCalculation({ detection, snapshot }: { detection: ConeDetection | undefined; snapshot: PlanningSnapshot }) {
  if (!detection) return <p className="empty">No cone selected or available.</p>;
  const cone = detection.cone;
  const vehicle = snapshot.sensor.vehicleAtCalculation;
  return (
    <div className="equation-block">
      <h4>WORLD CONE · {cone.id}</h4>
      <div>Cx = {fixed(cone.x)} m</div><div>Cy = {fixed(cone.y)} m</div>
      <h4>VEHICLE AT SENSOR SAMPLE</h4>
      <div>Vx = {fixed(vehicle.x)} m</div><div>Vy = {fixed(vehicle.y)} m</div><div>ψ = {degrees(vehicle.heading)}</div>
      <h4>RELATIVE</h4>
      <div>dx = Cx − Vx = {fixed(cone.x)} − {fixed(vehicle.x)} = {fixed(detection.dx)} m</div>
      <div>dy = Cy − Vy = {fixed(cone.y)} − {fixed(vehicle.y)} = {fixed(detection.dy)} m</div>
      <div>forward = cos(ψ)dx + sin(ψ)dy = {fixed(detection.longitudinal)} m</div>
      <div>lateral = −sin(ψ)dx + cos(ψ)dy = {fixed(detection.lateral)} m</div>
      <h4>DISTANCE / BEARING</h4>
      <div>r = √(dx² + dy²) = {fixed(detection.distance)} m</div>
      <div>β = atan2(lateral, forward) = {degrees(detection.bearing)}</div>
      <h4>CHECK</h4>
      <div>r ≤ {fixed(snapshot.sensor.range, 2)} m <b>{pass(detection.rangePass)}</b></div>
      <div>|β| ≤ {fixed(radToDeg(snapshot.sensor.fov / 2), 1)}° <b>{pass(detection.fovPass)}</b></div>
      <div>forward &gt; 0 <b>{pass(detection.forwardPass)}</b></div>
      <Status good={detection.detected}>{detection.detected ? 'CONE DETECTED' : 'CONE NOT DETECTED'}</Status>
    </div>
  );
}

export function LivePanel({ snapshot, selection, onSelect }: { snapshot: PlanningSnapshot; selection: InspectionSelection; onSelect?: (selection: InspectionSelection) => void }) {
  const [matrixExpanded, setMatrixExpanded] = useState(false);
  const selectedDetection = snapshot.sensor.detections.find((item) => item.cone.id === selection.id)
    ?? snapshot.sensor.detections.find((item) => item.detected);
  const triangle = snapshot.delaunay.triangles.find((item) => item.id === selection.id) ?? snapshot.delaunay.triangles[0];
  const edge = snapshot.edges.uniqueEdges.find((item) => item.id === selection.id) ?? snapshot.edges.crossTrackEdges[0] ?? snapshot.edges.uniqueEdges[0];
  const midpoint = snapshot.midpoints.find((item) => item.id === selection.id) ?? snapshot.midpoints[0];
  const selectedPathIndex = selection.kind === 'path' ? Number(selection.id) : snapshot.controller.targetIndex;
  const pathPoint = snapshot.interpolation.path[selectedPathIndex];
  const ordered = snapshot.orderedMidpoints.find((item) => item.id === selection.id) ?? snapshot.orderedMidpoints[1] ?? snapshot.orderedMidpoints[0];
  const c = snapshot.controller;
  const cv = c.vehicleAtCalculation;
  const movement = snapshot.movement;
  const selectedAvoidance = snapshot.avoidance.candidates.find((candidate) => candidate.side === snapshot.avoidance.selectedSide);
  const inspectedObstacle = snapshot.world.obstacles.find((item) => item.id === selection.id);
  const decisionClass = snapshot.trace.decision.startsWith('STOP') ? 'decision-stop' : snapshot.trace.decision.includes('LEFT') ? 'decision-left' : snapshot.trace.decision.includes('RIGHT') ? 'decision-right' : 'decision-follow';
  const pointsShown = matrixExpanded ? snapshot.delaunay.points : snapshot.delaunay.points.slice(0, 7);

  return (
    <aside className="live-panel" aria-label="Live autonomy computation">
      <div className="panel-heading">
        <div><span className="eyebrow">LIVE DATA · NO REPLAY</span><h2>LIVE AUTONOMY COMPUTATION</h2></div>
        <span className={snapshot.running ? 'live-dot live' : 'live-dot'}>{snapshot.running ? 'RUNNING' : 'FROZEN'}</span>
      </div>

      <section className={`decision-card ${decisionClass}`}>
        <span>CURRENT DECISION · CYCLE #{snapshot.cycle}</span>
        <strong>{snapshot.trace.decision}</strong>
        <div>{direction(c.steeringAngle)} {fixed(Math.abs(radToDeg(c.steeringAngle)), 2)}° · speed cmd {fixed(c.speedCommand, 2)} m/s</div>
      </section>

      <section className="inspect-card">
        <span>INSPECTING</span><b>{selection.kind.toUpperCase()} · {selection.id}</b>
        {selection.kind === 'path' && pathPoint && <small>sample {pathPoint.index}: ({fixed(pathPoint.x)}, {fixed(pathPoint.y)}), s = {fixed(pathPoint.s)} m</small>}
        {selection.kind === 'target' && c.target && <small>lookahead sample {c.targetIndex}: ({fixed(c.target.x)}, {fixed(c.target.y)})</small>}
        {selection.kind === 'obstacle' && inspectedObstacle && <small>world position ({fixed(inspectedObstacle.x)}, {fixed(inspectedObstacle.y)}) · radius {fixed(inspectedObstacle.radius)} m</small>}
        {selection.kind === 'kart' && <small>complete current vehicle state is shown below</small>}
      </section>

      <Section title="A · VEHICLE STATE" open badge={`${fixed(snapshot.vehicle.velocity, 2)} m/s`}>
        <Row label="Simulation time t" value={seconds(snapshot.simulationTime)} />
        <Row label="X position" value={meters(snapshot.vehicle.x)} testId="vehicle-x" />
        <Row label="Y position" value={meters(snapshot.vehicle.y)} testId="vehicle-y" />
        <Row label="Heading ψ" value={degrees(snapshot.vehicle.heading)} />
        <Row label="Velocity v" value={`${fixed(snapshot.vehicle.velocity)} m/s`} />
        <Row label="Acceleration a" value={`${radians(snapshot.vehicle.acceleration)} m/s²`} />
        <Row label="Steering δ" value={degrees(snapshot.vehicle.steeringAngle)} testId="steering-angle" />
        <Row label="Yaw rate" value={`${radians(snapshot.vehicle.yawRate)} rad/s`} />
        <Row label="Speed command" value={`${fixed(snapshot.vehicle.speedCommand)} m/s`} />
        <Row label="Frame / plan / control" value={`${snapshot.frame} / ${snapshot.cycle} / ${snapshot.controllerCycle}`} />
      </Section>

      <Section title="B · SENSOR CALCULATION" badge={snapshot.sensor.visibleCones.length}>
        <Row label="Sensor range" value={meters(snapshot.sensor.range, 2)} />
        <Row label="Field of view" value={`${fixed(radToDeg(snapshot.sensor.fov), 1)}°`} />
        <Row label="World cones" value={snapshot.sensor.worldConeCount} />
        <Row label="Visible cones" value={snapshot.sensor.visibleCones.length} testId="visible-cone-count" />
        <Row label="Blue / yellow visible" value={`${snapshot.sensor.blueVisible} / ${snapshot.sensor.yellowVisible}`} />
        <Row label="Visible obstacles" value={snapshot.sensor.visibleObstacles.length} />
        {snapshot.sensor.visibleObstacles[0] && <div className="equation-block">
          <h4>OBSTACLE {snapshot.sensor.visibleObstacles[0].obstacle.id}</h4>
          <div>relative forward = {fixed(snapshot.sensor.visibleObstacles[0].longitudinal)} m</div>
          <div>relative lateral = {fixed(snapshot.sensor.visibleObstacles[0].lateral)} m</div>
          <div>surface distance = {fixed(snapshot.sensor.visibleObstacles[0].distance)} m</div>
          <Status good={!snapshot.sensor.visibleObstacles[0].unsafe}>{snapshot.sensor.visibleObstacles[0].unsafe ? 'ROUTE MANEUVER REQUIRED' : 'DETECTED · MONITORING'}</Status>
        </div>}
        <SensorCalculation detection={selectedDetection} snapshot={snapshot} />
      </Section>

      <Section title="C · POINT MATRIX P" badge={`${snapshot.delaunay.points.length} × 2`}>
        <p className="note">This is the exact cone array supplied to Delaunator.</p>
        <table className="matrix-table"><thead><tr><th>IDX</th><th>ID</th><th>SIDE</th><th>X</th><th>Y</th></tr></thead>
          <tbody>{pointsShown.map((point, index) => <tr className={selection.kind === 'cone' && selection.id === point.id ? 'interactive-row selected' : 'interactive-row'} onClick={() => onSelect?.({ kind: 'cone', id: point.id })} key={point.id}><td>{index}</td><td>{point.id}</td><td className={point.side.toLowerCase()}>{point.side}</td><td>{fixed(point.x, 2)}</td><td>{fixed(point.y, 2)}</td></tr>)}</tbody>
        </table>
        {snapshot.delaunay.points.length > 7 && <button className="text-button" onClick={() => setMatrixExpanded((value) => !value)}>{matrixExpanded ? 'Compact matrix' : `Expand all ${snapshot.delaunay.points.length} rows`}</button>}
      </Section>

      <Section title="D · DELAUNAY TRIANGULATION" badge={snapshot.delaunay.triangles.length}>
        <Row label="Points" value={snapshot.delaunay.points.length} />
        <Row label="Triangles" value={snapshot.delaunay.triangles.length} testId="triangle-count" />
        <div className="array-block inspectable-list">{snapshot.delaunay.connectivity.slice(0, 12).map((indices, index) => <button className={selection.kind === 'triangle' && selection.id === `T${index + 1}` ? 'selected' : ''} onClick={() => onSelect?.({ kind: 'triangle', id: `T${index + 1}` })} key={index}>T{index + 1} = [{indices.join(', ')}]</button>)}</div>
        {triangle && <div className="equation-block">
          <h4>TRIANGLE {triangle.id}</h4>
          {triangle.vertices.map((vertex, index) => <div key={vertex.id}>Vertex {String.fromCharCode(65 + index)} = P[{triangle.indices[index]}] = {vertex.id} · {vertex.side} · ({fixed(vertex.x)}, {fixed(vertex.y)})</div>)}
          <h4>CENTROID</h4>
          <div>Cx = ({triangle.vertices.map((item) => fixed(item.x)).join(' + ')}) / 3 = {fixed(triangle.centroid.x)} m</div>
          <div>Cy = ({triangle.vertices.map((item) => fixed(item.y)).join(' + ')}) / 3 = {fixed(triangle.centroid.y)} m</div>
        </div>}
      </Section>

      <Section title="E · EXTERIOR TRIANGLE FILTER" badge={`${snapshot.filtering.interiorTriangles.length}/${snapshot.delaunay.triangles.length}`}>
        <Row label="Total triangles" value={snapshot.delaunay.triangles.length} />
        <Row label="Interior" value={snapshot.filtering.interiorTriangles.length} testId="interior-count" />
        <Row label="Rejected" value={snapshot.filtering.exteriorTriangles.length} />
        <div className="array-block">isInterior =<br />[{snapshot.filtering.flags.map((flag) => flag ? 1 : 0).join(' ')}]</div>
        {triangle && <div className="equation-block"><h4>{triangle.id} POINT-IN-CORRIDOR TEST</h4><div>centroid = ({fixed(triangle.centroid.x)}, {fixed(triangle.centroid.y)})</div><div>{triangle.filterReason}</div><Status good={triangle.interior}>RESULT = {triangle.interior ? 'INTERIOR' : 'EXTERIOR'}</Status></div>}
      </Section>

      <Section title="F · UNIQUE EDGE EXTRACTION" badge={snapshot.edges.crossTrackEdges.length}>
        <Row label="Unique edges" value={snapshot.edges.uniqueEdges.length} />
        <Row label="Valid cross-track" value={snapshot.edges.crossTrackEdges.length} testId="edge-count" />
        <div className="array-block edge-list inspectable-list">{snapshot.edges.uniqueEdges.slice(0, 14).map((item) => <button className={selection.kind === 'edge' && selection.id === item.id ? 'selected' : ''} onClick={() => onSelect?.({ kind: 'edge', id: item.id })} key={item.id}><b>{item.id}</b> = {item.a.id} ↔ {item.b.id} · {item.classification}</button>)}</div>
        {edge && <div className="equation-block">
          <h4>EDGE {edge.id}</h4>
          <div>{edge.a.id} ({edge.a.side}) ↔ {edge.b.id} ({edge.b.side})</div>
          <div>w = √(({fixed(edge.b.x)} − {fixed(edge.a.x)})² + ({fixed(edge.b.y)} − {fixed(edge.a.y)})²)</div>
          <div>w = {fixed(edge.length)} m</div>
          <div>{fixed(MIN_TRACK_WIDTH, 2)} ≤ {fixed(edge.length)} ≤ {fixed(MAX_TRACK_WIDTH, 2)} <b>{pass(edge.widthPass)}</b></div>
          <Status good={edge.classification === 'VALID CROSS-TRACK EDGE'}>{edge.classification}</Status>
        </div>}
      </Section>

      <Section title="G · MIDPOINT CALCULATION" badge={snapshot.midpoints.length}>
        <Row label="Center estimates" value={snapshot.midpoints.length} testId="midpoint-count" />
        {midpoint ? <div className="equation-block">
          <h4>{midpoint.id} FROM {midpoint.sourceEdgeId}</h4>
          <div>BLUE = ({fixed(midpoint.blue.x)}, {fixed(midpoint.blue.y)})</div>
          <div>YELLOW = ({fixed(midpoint.yellow.x)}, {fixed(midpoint.yellow.y)})</div>
          <div>xmp = ({fixed(midpoint.blue.x)} + {fixed(midpoint.yellow.x)}) / 2 = {fixed(midpoint.x)} m</div>
          <div>ymp = ({fixed(midpoint.blue.y)} + {fixed(midpoint.yellow.y)}) / 2 = {fixed(midpoint.y)} m</div>
          <Status>MIDPOINT = ({fixed(midpoint.x)}, {fixed(midpoint.y)})</Status>
        </div> : <p className="empty">No valid cross-track midpoint.</p>}
      </Section>

      <Section title="H · MIDPOINT ORDERING" badge={snapshot.orderedMidpoints.length}>
        <div className="array-block inspectable-list">{snapshot.orderedMidpoints.map((item) => <button className={selection.kind === 'midpoint' && selection.id === item.id ? 'selected' : ''} onClick={() => onSelect?.({ kind: 'midpoint', id: item.id })} key={item.id}>M{item.order} = {item.id} · ({fixed(item.x, 2)}, {fixed(item.y, 2)})</button>)}</div>
        {ordered && <div className="equation-block">
          <h4>SELECT ORDER {ordered.order} · {ordered.id}</h4>
          <div>dx = {fixed(ordered.selectionReason.dx)} m · dy = {fixed(ordered.selectionReason.dy)} m</div>
          <div>d = √({fixed(ordered.selectionReason.dx)}² + {fixed(ordered.selectionReason.dy)}²) = {fixed(ordered.distanceFromPrevious)} m</div>
          <div>heading from previous = {degrees(ordered.headingFromPrevious)}</div>
          <div>Forward progress <b>{pass(ordered.selectionReason.forwardProgress)}</b></div>
          <div>Heading change = {fixed(Math.abs(radToDeg(ordered.selectionReason.headingChange)), 2)}°</div>
          <div>Connectivity <b>{pass(ordered.selectionReason.connectivityPass)}</b></div>
          <div>selection score = {fixed(ordered.selectionReason.score)}</div>
          <Status>SELECT {ordered.id}</Status>
        </div>}
      </Section>

      <Section title="I · CUMULATIVE DISTANCE" badge={snapshot.interpolation.cumulativeDistance.length}>
        <div className="equation-block">
          {snapshot.interpolation.segmentDistances.slice(0, 10).map((value, index) => <div key={index}>d{index + 1} = {fixed(value)} m</div>)}
          <h4>s =</h4>
          {snapshot.interpolation.cumulativeDistance.slice(0, 12).map((value, index) => <div key={index}>s{index} = {index === 0 ? '0' : `s${index - 1} + d${index} = ${fixed(value)}`} m</div>)}
          <Status>totalDistance = {fixed(snapshot.interpolation.cumulativeDistance.at(-1) ?? 0)} m</Status>
        </div>
      </Section>

      <Section title="J · SPLINE INTERPOLATION" badge={snapshot.interpolation.path.length}>
        <Row label="Raw midpoint count" value={snapshot.orderedMidpoints.length} />
        <Row label="Spline sample count" value={snapshot.interpolation.path.length} />
        <div className="array-block inspectable-list">{snapshot.interpolation.path.slice(0, 7).map((point) => <button className={selection.kind === 'path' && selection.id === String(point.index) ? 'selected' : ''} onClick={() => onSelect?.({ kind: 'path', id: String(point.index) })} key={point.index}>S{point.index}: x = {fixed(point.x)}, y = {fixed(point.y)}, s = {fixed(point.s)}</button>)}</div>
        {pathPoint && <Status>SELECTED S{pathPoint.index} = ({fixed(pathPoint.x)}, {fixed(pathPoint.y)})</Status>}
      </Section>

      <Section title="OBSTACLE ROUTE SELECTION" open={snapshot.avoidance.active} badge={snapshot.avoidance.active ? snapshot.avoidance.selectedSide ?? 'ACTIVE' : 'CLEAR'}>
        {!snapshot.avoidance.active && <p className="empty">No obstacle intersects the live centerline safety envelope. Pure Pursuit receives the base spline.</p>}
        {snapshot.avoidance.active && snapshot.avoidance.obstacle && <>
          <Row label="Tracking source" value={snapshot.avoidance.trackingSource} />
          <Row label="Safety phase" value={snapshot.avoidance.phase} />
          <div className="clickable-value" onClick={() => onSelect?.({ kind: 'obstacle', id: snapshot.avoidance.obstacle!.obstacle.id })}><Row label="Obstacle" value={`${snapshot.avoidance.obstacle.obstacle.id} · CLICK TO INSPECT`} /></div>
          <Row label="Obstacle world position" value={`(${fixed(snapshot.avoidance.obstacle.obstacle.x)}, ${fixed(snapshot.avoidance.obstacle.obstacle.y)})`} />
          <Row label="Projected path index" value={snapshot.avoidance.obstaclePathIndex} />
          <Row label="Projected path s" value={meters(snapshot.avoidance.obstaclePathS)} />
          <Row label="Obstacle lateral" value={meters(snapshot.avoidance.obstacleLateral)} />
          <Row label="Current body clearance" value={meters(snapshot.avoidance.currentClearance)} />
          <Row label="Emergency stopping distance" value={meters(snapshot.avoidance.brakingDistance)} />
          <p className="note">Both routes use a cosine-blended offset with a flat clearance zone. Lower feasible score wins; the selected side remains committed through the pass.</p>
          {snapshot.avoidance.candidates.map((candidate) => <div className={`route-candidate ${candidate.side === snapshot.avoidance.selectedSide ? 'route-selected' : ''}`} key={candidate.side}>
            <h4>{candidate.side} CANDIDATE {candidate.side === snapshot.avoidance.selectedSide && '· SELECTED'}</h4>
            <Row label="Corridor width" value={meters(candidate.corridorWidth)} />
            <Row label="Desired lateral offset" value={meters(candidate.desiredOffset)} />
            <Row label="Max applied offset" value={meters(candidate.maxAppliedOffset)} />
            <Row label="Obstacle clearance" value={`${fixed(candidate.minimumObstacleClearance)} ≥ ${fixed(candidate.requiredObstacleClearance)} m · ${pass(candidate.minimumObstacleClearance >= candidate.requiredObstacleClearance - 0.025)}`} />
            <Row label="Boundary clearance" value={`${fixed(candidate.minimumBoundaryClearance)} ≥ ${fixed(candidate.requiredBoundaryClearance)} m · ${pass(candidate.minimumBoundaryClearance >= candidate.requiredBoundaryClearance)}`} />
            <Row label="Route score" value={fixed(candidate.score)} />
            <Status good={candidate.feasible}>{candidate.feasible ? 'FEASIBLE ROUTE' : 'REJECTED'} · {candidate.reason}</Status>
          </div>)}
          {selectedAvoidance && <Status good={snapshot.avoidance.phase !== 'BLOCKED'}>{snapshot.avoidance.phase === 'BLOCKED'
              ? 'STOP · NO ROUTE PASSES BOTH SAFETY LIMITS'
              : `MANEUVER ${selectedAvoidance.side} · ${snapshot.avoidance.selectedPath.length} PATH SAMPLES → PURE PURSUIT`}</Status>}
        </>}
      </Section>

      <Section title="PURE PURSUIT · LIVE CALCULATION" open badge={degrees(c.steeringAngle)}>
        <div className="equation-block">
          <h4>NEAREST PATH POINT</h4>
          <div>index = {c.nearestIndex}</div><div>coordinates = {c.nearest ? `(${fixed(c.nearest.x)}, ${fixed(c.nearest.y)})` : 'none'}</div>
          <h4>LOOKAHEAD SEARCH</h4>
          <div>Ld_desired = {fixed(c.desiredLookahead)} m</div><div>target index = {c.targetIndex}</div>
          <div>target = {c.target ? `(${fixed(c.target.x)}, ${fixed(c.target.y)})` : 'none'}</div>
          <h4>WORLD → VEHICLE FRAME</h4>
          <div>Convention: +longitudinal = forward · +lateral / +δ = LEFT (CCW)</div>
          <div>controller pose = ({fixed(cv.x)}, {fixed(cv.y)}, {degrees(cv.heading)})</div>
          <div>dx = Tx − VehicleX = {fixed(c.dx)} m</div><div>dy = Ty − VehicleY = {fixed(c.dy)} m</div>
          <div>x_vehicle = cos(ψ)dx + sin(ψ)dy = {fixed(c.longitudinal)} m</div>
          <div>y_vehicle = −sin(ψ)dx + cos(ψ)dy = {fixed(c.lateral)} m</div>
          <h4>LOOKAHEAD DISTANCE</h4>
          <div>Ld = √({fixed(c.longitudinal)}² + {fixed(c.lateral)}²) = <span data-testid="lookahead-distance">{fixed(c.lookaheadDistance)}</span> m</div>
          <h4>TARGET ANGLE</h4><div>α = atan2(y_vehicle, x_vehicle) = {degrees(c.alpha)}</div>
          <h4>CURVATURE</h4><div>κ = 2 sin({degrees(c.alpha)}) / {fixed(c.lookaheadDistance)} = <span data-testid="curvature">{fixed(c.curvature)}</span> m⁻¹</div>
          <h4>STEERING ANGLE</h4><div>δ = atan({fixed(WHEELBASE, 2)} × {fixed(c.curvature)}) = {degrees(c.steeringAngle)}</div>
          <Status good={!c.stopReason && !c.holdReason}>{c.holdReason ? `${c.holdReason} · SPEED COMMAND 0.000 m/s` : `STEERING COMMAND · ${direction(c.steeringAngle)} ${fixed(Math.abs(radToDeg(c.steeringAngle)), 2)}°`}</Status>
        </div>
      </Section>

      <Section title="VEHICLE MOVEMENT CALCULATION" open badge={`${fixed(movement.dt, 4)} s`}>
        <div className="equation-block">
          <div>yawRate = v / L × tan(δ)</div>
          <div>= {fixed(movement.oldState.velocity)} / {fixed(WHEELBASE, 2)} × tan({degrees(movement.oldState.steeringAngle)})</div>
          <div>= {fixed(movement.yawRate)} rad/s</div>
          <div>Δψ = yawRate × dt = {fixed(movement.deltaHeading, 5)} rad = {fixed(radToDeg(movement.deltaHeading), 3)}°</div>
          <div>Δx = v cos(ψ) dt = {fixed(movement.deltaX, 5)} m</div>
          <div>Δy = v sin(ψ) dt = {fixed(movement.deltaY, 5)} m</div>
          <h4>OLD</h4><div>x = {fixed(movement.oldState.x)} · y = {fixed(movement.oldState.y)} · ψ = {degrees(movement.oldState.heading, 3)}</div>
          <h4>NEW</h4><div>x = {fixed(movement.newState.x)} · y = {fixed(movement.newState.y)} · ψ = {degrees(movement.newState.heading, 3)}</div>
        </div>
      </Section>

      <Section title="WHY DID THE KART DO THIS?" open badge={`#${snapshot.cycle}`}>
        <div className="decision-trace">
          <div><b>✓ SENSOR UPDATE</b><span>{snapshot.trace.sensorCount} cones visible</span></div><i>↓</i>
          <div><b>✓ DELAUNAY</b><span>{snapshot.trace.triangleCount} triangles generated</span></div><i>↓</i>
          <div><b>✓ EXTERIOR FILTER</b><span>{snapshot.trace.interiorCount} valid · {snapshot.trace.rejectedCount} rejected</span></div><i>↓</i>
          <div><b>✓ EDGE EXTRACTION</b><span>{snapshot.trace.crossTrackCount} BLUE↔YELLOW edges</span></div><i>↓</i>
          <div><b>✓ MIDPOINTS / SPLINE</b><span>{snapshot.trace.midpointCount} estimates · {snapshot.trace.pathCount} samples</span></div><i>↓</i>
          {snapshot.avoidance.active && <><div><b>✓ OBSTACLE ROUTE · {snapshot.avoidance.phase}</b><span>{snapshot.avoidance.selectedSide} selected · route clearance {fixed(selectedAvoidance?.minimumObstacleClearance ?? 0)} m · live clearance {fixed(snapshot.avoidance.currentClearance)} m</span></div><i>↓</i></>}
          <div><b>✓ LOOKAHEAD</b><span>{snapshot.trace.target ? `target = (${fixed(snapshot.trace.target.x)}, ${fixed(snapshot.trace.target.y)})` : 'no target'}</span></div><i>↓</i>
          <div><b>✓ PURE PURSUIT</b><span>α {degrees(snapshot.trace.alpha)} · κ {radians(snapshot.trace.curvature)} m⁻¹ · δ {degrees(snapshot.trace.steeringAngle)}</span></div><i>↓</i>
          <div className="trace-decision"><b>DECISION</b><span>{snapshot.trace.decision}</span></div><i>↓</i>
          <div><b>VEHICLE MODEL</b><span>yawRate = {radians(snapshot.trace.yawRate)} rad/s</span></div><i>↓</i>
          <div><b>MOVE</b><span>({fixed(snapshot.trace.newPose.x)}, {fixed(snapshot.trace.newPose.y)}, {degrees(snapshot.trace.newPose.heading)})</span></div>
        </div>
      </Section>

      <Section title="REAL-TIME PIPELINE STATUS" open badge={`${fixed(snapshot.timing.totalPlanMs, 2)} ms`}>
        <div className="pipeline-timing">
          <Row label="SENSOR" value={`${fixed(snapshot.timing.sensorMs, 3)} ms`} />
          <Row label="DELAUNAY" value={`${fixed(snapshot.timing.delaunayMs, 3)} ms`} />
          <Row label="FILTER" value={`${fixed(snapshot.timing.filterMs, 3)} ms`} />
          <Row label="EDGES" value={`${fixed(snapshot.timing.edgeMs, 3)} ms`} />
          <Row label="MIDPOINTS" value={`${fixed(snapshot.timing.midpointMs, 3)} ms`} />
          <Row label="ORDER" value={`${fixed(snapshot.timing.orderMs, 3)} ms`} />
          <Row label="SPLINE" value={`${fixed(snapshot.timing.splineMs, 3)} ms`} />
          <Row label="AVOIDANCE" value={`${fixed(snapshot.timing.avoidanceMs, 3)} ms`} />
          <Row label="CONTROL" value={`${fixed(snapshot.timing.controllerMs, 3)} ms`} />
          <Row label="TOTAL PLAN" value={`${fixed(snapshot.timing.totalPlanMs, 3)} ms`} />
        </div>
        <h4 className="subhead">MEASURED LOOPS</h4>
        {(Object.entries(snapshot.loops) as [string, typeof snapshot.loops.physics][]).map(([name, loop]) => <div className="loop-row" key={name}><b>{name}</b><span>{fixed(loop.measuredHz, 1)} Hz <small>/ {loop.targetHz}</small></span><span>{fixed(loop.executionMs, 3)} ms</span></div>)}
      </Section>
    </aside>
  );
}
