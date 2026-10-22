const add = (a, b) => a.map((value, index) => value + b[index]);
const sub = (a, b) => a.map((value, index) => value - b[index]);
const scale = (a, value) => a.map(component => component * value);
const dot = (a, b) => a.reduce((sum, value, index) => sum + value * b[index], 0);
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const normalize = vector => {
  const magnitude = Math.hypot(...vector) || 1;
  return scale(vector, 1 / magnitude);
};

export class CameraController {
  constructor(canvas) {
    this.canvas = canvas;
    this.mode = "orbit";
    this.azimuth = -0.85;
    this.elevation = 0.78;
    this.distance = 27;
    this.target = [0, 0, 0];
    this.drag = null;
    this.trackCenter = [0, 0, 0];
    this.trackRadius = 10;
    this.bindControls();
  }

  bindControls() {
    this.canvas.addEventListener("pointerdown", event => {
      this.drag = {x: event.clientX, y: event.clientY, button: event.button};
      this.canvas.setPointerCapture(event.pointerId);
    });
    this.canvas.addEventListener("pointermove", event => {
      if (!this.drag) return;
      const dx = event.clientX - this.drag.x;
      const dy = event.clientY - this.drag.y;
      this.drag.x = event.clientX;
      this.drag.y = event.clientY;
      if (this.drag.button === 0) {
        this.azimuth -= dx * 0.006;
        this.elevation = Math.max(0.08, Math.min(1.48, this.elevation + dy * 0.005));
      } else {
        const factor = this.distance * 0.0018;
        this.target[0] -= dx * factor;
        this.target[1] += dy * factor;
      }
      this.mode = "free";
    });
    this.canvas.addEventListener("pointerup", () => { this.drag = null; });
    this.canvas.addEventListener("wheel", event => {
      event.preventDefault();
      this.distance = Math.max(1.2, Math.min(250, this.distance * Math.exp(event.deltaY * 0.001)));
      this.mode = "free";
    }, {passive: false});
  }

  configureTrack(track) {
    if (!track?.points?.length) return;
    const xs = track.points.map(point => point.x_m);
    const ys = track.points.map(point => point.y_m);
    const zs = track.points.map(point => point.z_m);
    this.trackCenter = [
      (Math.min(...xs) + Math.max(...xs)) / 2,
      (Math.min(...ys) + Math.max(...ys)) / 2,
      (Math.min(...zs) + Math.max(...zs)) / 2,
    ];
    this.trackRadius = Math.max(5, Math.hypot(Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys)) / 2);
    this.reset();
  }

  setMode(mode) { this.mode = mode; }

  reset() {
    this.target = [...this.trackCenter];
    this.azimuth = -0.85;
    this.elevation = 0.78;
    this.distance = this.trackRadius * 2.5;
  }

  pose(frame, track) {
    const pose = frame?.pose ?? {x_m: 0, y_m: 0, z_m: 0, yaw_rad: 0};
    const kart = [pose.x_m, pose.y_m, pose.z_m + 0.15];
    let target = this.target;
    let eye;
    if (this.mode === "chase") {
      target = add(kart, [Math.cos(pose.yaw_rad) * 2.2, Math.sin(pose.yaw_rad) * 2.2, 0.2]);
      eye = add(kart, [-Math.cos(pose.yaw_rad) * 5.8, -Math.sin(pose.yaw_rad) * 5.8, 3.1]);
    } else if (this.mode === "top") {
      target = [...kart];
      eye = add(kart, [0.001, 0, Math.max(15, this.trackRadius * 2.3)]);
    } else if (this.mode === "front") {
      eye = add(kart, [Math.cos(pose.yaw_rad) * 0.7, Math.sin(pose.yaw_rad) * 0.7, 0.42]);
      target = add(eye, [Math.cos(pose.yaw_rad) * 12, Math.sin(pose.yaw_rad) * 12, -0.8]);
    } else if (this.mode === "driver") {
      eye = add(kart, [-Math.cos(pose.yaw_rad) * 0.2, -Math.sin(pose.yaw_rad) * 0.2, 0.62]);
      target = add(eye, [Math.cos(pose.yaw_rad) * 10, Math.sin(pose.yaw_rad) * 10, -0.5]);
    } else if (this.mode === "corner") {
      const corner = (track?.corners ?? [])[0];
      const point = corner ? track.points.reduce((best, candidate) =>
        Math.abs(candidate.s_m - corner.geometric_apex_s_m) < Math.abs(best.s_m - corner.geometric_apex_s_m) ? candidate : best
      ) : null;
      target = point ? [point.x_m, point.y_m, point.z_m] : [...kart];
      eye = add(target, [7, -7, 5]);
    } else {
      const horizontal = this.distance * Math.cos(this.elevation);
      eye = [
        target[0] + horizontal * Math.cos(this.azimuth),
        target[1] + horizontal * Math.sin(this.azimuth),
        target[2] + this.distance * Math.sin(this.elevation),
      ];
    }
    return {eye, target};
  }

  projector(frame, track, width, height) {
    const {eye, target} = this.pose(frame, track);
    const forward = normalize(sub(target, eye));
    let right = normalize(cross(forward, [0, 0, 1]));
    if (Math.hypot(...right) < 0.1) right = [1, 0, 0];
    const up = normalize(cross(right, forward));
    const focal = Math.min(width, height) * 0.82;
    return point => {
      const delta = sub(point, eye);
      const depth = dot(delta, forward);
      if (depth <= 0.03) return null;
      const perspective = focal / depth;
      return {
        x: width / 2 + dot(delta, right) * perspective,
        y: height / 2 - dot(delta, up) * perspective,
        depth,
        scale: perspective,
      };
    };
  }
}
