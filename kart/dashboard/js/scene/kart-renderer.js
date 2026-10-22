function rotateLocal(local, pose) {
  const [x, y, z] = local;
  const cr = Math.cos(pose.roll_rad), sr = Math.sin(pose.roll_rad);
  const cp = Math.cos(pose.pitch_rad), sp = Math.sin(pose.pitch_rad);
  const cy = Math.cos(pose.yaw_rad), sy = Math.sin(pose.yaw_rad);
  const roll = [x, y * cr - z * sr, y * sr + z * cr];
  const pitch = [roll[0] * cp + roll[2] * sp, roll[1], -roll[0] * sp + roll[2] * cp];
  return [
    pose.x_m + pitch[0] * cy - pitch[1] * sy,
    pose.y_m + pitch[0] * sy + pitch[1] * cy,
    pose.z_m + pitch[2],
  ];
}

function polygon(context, project, localPoints, pose, fill, stroke = null, width = 1) {
  const points = localPoints.map(point => project(rotateLocal(point, pose))).filter(Boolean);
  if (points.length < 3) return;
  context.beginPath();
  context.moveTo(points[0].x, points[0].y);
  for (const point of points.slice(1)) context.lineTo(point.x, point.y);
  context.closePath();
  context.fillStyle = fill;
  context.fill();
  if (stroke) {
    context.strokeStyle = stroke;
    context.lineWidth = width;
    context.stroke();
  }
}

function localRectangle(cx, cy, length, width, angle = 0, z = 0.08) {
  const corners = [[length / 2, width / 2], [length / 2, -width / 2], [-length / 2, -width / 2], [-length / 2, width / 2]];
  return corners.map(([x, y]) => [
    cx + x * Math.cos(angle) - y * Math.sin(angle),
    cy + x * Math.sin(angle) + y * Math.cos(angle),
    z,
  ]);
}

function line(context, points, stroke, width = 1) {
  const valid = points.filter(Boolean);
  if (valid.length < 2) return;
  context.beginPath();
  context.moveTo(valid[0].x, valid[0].y);
  for (const point of valid.slice(1)) context.lineTo(point.x, point.y);
  context.strokeStyle = stroke;
  context.lineWidth = width;
  context.stroke();
}

export class KartRenderer {
  constructor() {
    this.phases = {front_left: 0, front_right: 0, rear_left: 0, rear_right: 0};
    this.previousTime = null;
  }

  draw(context, frame, kart, project, overlays) {
    if (!frame?.valid || !kart?.dimensions) return;
    const pose = frame.pose;
    const dimensions = kart.dimensions;
    const wheels = frame.subsystems?.four_wheel?.wheels ?? {};
    const dt = this.previousTime == null ? 0 : Math.max(0, Math.min(0.1, frame.timestamp_s - this.previousTime));
    this.previousTime = frame.timestamp_s;
    for (const [name, wheel] of Object.entries(wheels)) {
      this.phases[name] = (this.phases[name] + Number(wheel.angular_speed_radps ?? 0) * dt) % (Math.PI * 2);
    }

    if (overlays.footprint) {
      polygon(context, project, localRectangle(0, 0, dimensions.chassis_length_m, dimensions.chassis_width_m, 0, 0.018), pose, "rgba(91,192,216,.07)", "rgba(91,192,216,.55)", 1);
    }

    // Steel tube chassis and floor.
    polygon(context, project, localRectangle(-0.03, 0, dimensions.wheelbase_m + 0.3, dimensions.rear_track_m * 0.68, 0, 0.10), pose, "#303b43", "#9babb7", 1.2);
    polygon(context, project, localRectangle(-0.12, 0, 0.60, 0.48, 0, 0.135), pose, "#111820", "#4e5d69", 1);

    // Battery, motor, VCU, Jetson, seat, and sensor mast. These are explicitly
    // procedural engineering placeholders scaled from the kart configuration.
    polygon(context, project, localRectangle(-0.2, 0.29, 0.52, 0.22, 0, 0.18), pose, "#2f6690", "#74b3df");
    polygon(context, project, localRectangle(-0.46, -0.31, 0.24, 0.20, 0, 0.18), pose, "#9a5d36", "#dc9a6f");
    polygon(context, project, localRectangle(0.12, -0.31, 0.18, 0.14, 0, 0.19), pose, "#495561", "#91a0ad");
    polygon(context, project, localRectangle(0.34, 0.28, 0.20, 0.15, 0, 0.22), pose, "#425f4c", "#7bb28c");
    polygon(context, project, localRectangle(0.07, 0, 0.35, 0.39, 0, 0.30), pose, "#20282f", "#65727c");

    const halfWheelbase = dimensions.wheelbase_m / 2;
    const wheelLocations = {
      front_left: [halfWheelbase, dimensions.front_track_m / 2],
      front_right: [halfWheelbase, -dimensions.front_track_m / 2],
      rear_left: [-halfWheelbase, dimensions.rear_track_m / 2],
      rear_right: [-halfWheelbase, -dimensions.rear_track_m / 2],
    };
    for (const [name, [x, y]] of Object.entries(wheelLocations)) {
      const steering = Number(wheels[name]?.steering_angle_rad ?? 0);
      const contact = wheels[name]?.contact_valid !== false;
      polygon(
        context,
        project,
        localRectangle(x, y, dimensions.wheel_radius_m * 2, dimensions.wheel_width_m, steering, dimensions.wheel_radius_m),
        pose,
        contact ? "#111417" : "#7e2430",
        contact ? "#9ba8b2" : "#ff5262",
        1.2,
      );
      const spokeAngle = steering + this.phases[name];
      line(context, [
        project(rotateLocal([x - Math.cos(spokeAngle) * dimensions.wheel_radius_m * .7, y - Math.sin(spokeAngle) * dimensions.wheel_radius_m * .7, dimensions.wheel_radius_m + .01], pose)),
        project(rotateLocal([x + Math.cos(spokeAngle) * dimensions.wheel_radius_m * .7, y + Math.sin(spokeAngle) * dimensions.wheel_radius_m * .7, dimensions.wheel_radius_m + .01], pose)),
      ], "#d5dce1", 1);
    }

    // Rear axle and steering column.
    line(context, [
      project(rotateLocal([-halfWheelbase, dimensions.rear_track_m / 2, dimensions.wheel_radius_m], pose)),
      project(rotateLocal([-halfWheelbase, -dimensions.rear_track_m / 2, dimensions.wheel_radius_m], pose)),
    ], "#c0c9cf", 2);
    line(context, [
      project(rotateLocal([0.16, 0, .24], pose)),
      project(rotateLocal([0.48, 0, .48], pose)),
    ], "#b8c3ca", 2);

    // LiDAR puck, camera bar, and physical E-stop indication.
    const mast = project(rotateLocal([0.25, 0, .58], pose));
    if (mast) {
      context.beginPath();
      context.arc(mast.x, mast.y, 4, 0, Math.PI * 2);
      context.fillStyle = "#5bc0d8";
      context.fill();
    }
    const estop = frame.vcu?.state === "EMERGENCY_STOP";
    const estopPoint = project(rotateLocal([-0.1, -0.33, .35], pose));
    if (estopPoint) {
      context.beginPath();
      context.arc(estopPoint.x, estopPoint.y, estop ? 6 : 3.5, 0, Math.PI * 2);
      context.fillStyle = estop ? "#ff3347" : "#d44c58";
      context.fill();
    }

    if (overlays.cg) {
      const cg = project(rotateLocal([kart.cg.x_m, kart.cg.y_m, kart.cg.z_m], pose));
      if (cg) {
        context.strokeStyle = "#f5d35a";
        context.lineWidth = 1;
        context.beginPath();
        context.arc(cg.x, cg.y, 5, 0, Math.PI * 2);
        context.moveTo(cg.x - 8, cg.y); context.lineTo(cg.x + 8, cg.y);
        context.moveTo(cg.x, cg.y - 8); context.lineTo(cg.x, cg.y + 8);
        context.stroke();
      }
    }

    if (overlays.velocity) {
      const length = Math.min(8, frame.motion.speed_mps * 0.8);
      line(context, [
        project(rotateLocal([0, 0, .2], pose)),
        project(rotateLocal([length, 0, .2], pose)),
      ], "#5bc0d8", 2);
    }

    if (overlays.stopping) {
      const distance = Number(frame.vcu?.stopping_distance?.total_m ?? 0);
      line(context, [
        project(rotateLocal([0, 0, .03], pose)),
        project(rotateLocal([distance, 0, .03], pose)),
      ], "#e6ae45", 3);
    }

    if (overlays.forces) {
      for (const [name, [x, y]] of Object.entries(wheelLocations)) {
        const wheel = wheels[name] ?? {};
        const fx = Number(wheel.longitudinal_force_n ?? 0) / 1200;
        const fy = Number(wheel.lateral_force_n ?? 0) / 1200;
        line(context, [
          project(rotateLocal([x, y, .08], pose)),
          project(rotateLocal([x + fx, y + fy, .08], pose)),
        ], Number(wheel.utilisation ?? 0) > .9 ? "#ef5d68" : "#4ac087", 2);
      }
    }
  }
}
