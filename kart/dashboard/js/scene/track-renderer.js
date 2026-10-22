function pathPolygon(context, projected) {
  const valid = projected.filter(Boolean);
  if (valid.length < 3) return false;
  context.beginPath();
  context.moveTo(valid[0].x, valid[0].y);
  for (const point of valid.slice(1)) context.lineTo(point.x, point.y);
  context.closePath();
  return true;
}

function line(context, projected, stroke, width = 1, dash = []) {
  const valid = projected.filter(Boolean);
  if (valid.length < 2) return;
  context.beginPath();
  context.moveTo(valid[0].x, valid[0].y);
  for (const point of valid.slice(1)) context.lineTo(point.x, point.y);
  context.strokeStyle = stroke;
  context.lineWidth = width;
  context.setLineDash(dash);
  context.stroke();
  context.setLineDash([]);
}

function offsetPoint(point, lateral) {
  return [
    point.x_m - lateral * Math.sin(point.heading_rad),
    point.y_m + lateral * Math.cos(point.heading_rad),
    point.z_m + 0.012,
  ];
}

export class TrackRenderer {
  draw(context, track, project, overlays, faultEvents = []) {
    if (!track?.points?.length) return;
    const points = track.points;
    const left = points.map(point => project([point.left.x_m, point.left.y_m, point.left.z_m + 0.01]));
    const right = points.map(point => project([point.right.x_m, point.right.y_m, point.right.z_m + 0.01]));
    const ribbon = [...left, ...right.slice().reverse()];
    if (pathPolygon(context, ribbon)) {
      context.fillStyle = "#222b33";
      context.fill();
    }

    // Alternating arc-length bands give the surface scale without pretending
    // to be a friction source.
    for (let index = 0; index < points.length - 1; index += 1) {
      const quad = [left[index], left[index + 1], right[index + 1], right[index]];
      if (!pathPolygon(context, quad)) continue;
      context.fillStyle = index % 2 ? "#242e37" : "#27313a";
      context.fill();
    }

    if (overlays.surface) {
      for (const region of track.surface_regions ?? []) {
        const regionPoints = points.filter(point =>
          point.s_m >= Number(region.start_s_m) && point.s_m <= Number(region.end_s_m)
        );
        if (regionPoints.length < 2) continue;
        const low = regionPoints.map(point => project(offsetPoint(point, Number(region.min_lateral_m))));
        const high = regionPoints.map(point => project(offsetPoint(point, Number(region.max_lateral_m))));
        if (pathPolygon(context, [...low, ...high.reverse()])) {
          const material = String(region.material ?? "");
          context.fillStyle = material.includes("wet") ? "rgba(43,114,150,.48)" :
            material.includes("gravel") ? "rgba(142,116,70,.55)" : "rgba(133,99,58,.4)";
          context.fill();
        }
      }
    }

    if (overlays.boundaries) {
      line(context, left, "#dce6ec", 1.4);
      line(context, right, "#dce6ec", 1.4);
      line(context, points.map(point => project([point.x_m, point.y_m, point.z_m + 0.018])), "#526372", 1, [6, 8]);
    }

    // Start/finish stripe.
    const start = points[0];
    const startLine = [
      project(offsetPoint(start, start.left_width_m)),
      project(offsetPoint(start, -start.right_width_m)),
    ];
    line(context, startLine, "#f2f5f7", 3);

    if (overlays.corners) {
      context.textAlign = "center";
      context.font = "700 9px Inter, sans-serif";
      for (const corner of track.corners ?? []) {
        const apex = points.reduce((best, point) =>
          Math.abs(point.s_m - corner.geometric_apex_s_m) < Math.abs(best.s_m - corner.geometric_apex_s_m) ? point : best
        );
        const marker = project([apex.x_m, apex.y_m, apex.z_m + 0.25]);
        if (!marker) continue;
        context.beginPath();
        context.arc(marker.x, marker.y, 6, 0, Math.PI * 2);
        context.fillStyle = "#5bc0d8";
        context.fill();
        context.fillStyle = "#cceaf0";
        context.fillText(corner.id, marker.x, marker.y - 10);
      }
    }

    if (overlays.faults) {
      for (const event of faultEvents) {
        const pose = event.record.pose;
        const marker = project([pose.x_m, pose.y_m, pose.z_m + 0.4]);
        if (!marker) continue;
        context.beginPath();
        context.moveTo(marker.x, marker.y - 8);
        context.lineTo(marker.x - 7, marker.y + 6);
        context.lineTo(marker.x + 7, marker.y + 6);
        context.closePath();
        context.fillStyle = "#ef5d68";
        context.fill();
      }
    }
  }
}
