import {CameraController} from "./camera-controller.js";
import {TrackRenderer} from "./track-renderer.js";
import {KartRenderer} from "./kart-renderer.js";

export class EngineeringScene extends EventTarget {
  constructor(canvas, store) {
    super();
    this.canvas = canvas;
    this.context = canvas.getContext("2d", {alpha: false});
    this.store = store;
    this.camera = new CameraController(canvas);
    this.trackRenderer = new TrackRenderer();
    this.kartRenderer = new KartRenderer();
    this.overlays = {
      trajectory: true, boundaries: true, surface: true, footprint: true,
      forces: false, velocity: false, stopping: false, corners: true,
      faults: true, cg: false,
    };
    this.frameTimes = [];
    this.fps = 0;
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas);
    this.resize();
    this.animate();
  }

  resize() {
    const ratio = Math.min(2, window.devicePixelRatio || 1);
    const rect = this.canvas.getBoundingClientRect();
    this.canvas.width = Math.max(1, Math.floor(rect.width * ratio));
    this.canvas.height = Math.max(1, Math.floor(rect.height * ratio));
    this.context.setTransform(ratio, 0, 0, ratio, 0, 0);
    this.width = rect.width;
    this.height = rect.height;
  }

  setTrack(track) {
    this.camera.configureTrack(track);
  }

  setCameraMode(mode) { this.camera.setMode(mode); }
  resetCamera() { this.camera.reset(); }
  setOverlay(name, value) { this.overlays[name] = value; }

  drawGround(project) {
    const ctx = this.context;
    const extent = Math.max(30, this.camera.trackRadius * 2.1);
    ctx.fillStyle = "#081016";
    ctx.fillRect(0, 0, this.width, this.height);
    ctx.strokeStyle = "#14212a";
    ctx.lineWidth = 1;
    const step = extent > 70 ? 10 : 5;
    for (let value = -extent; value <= extent; value += step) {
      const horizontal = [project([-extent, value, -0.03]), project([extent, value, -0.03])].filter(Boolean);
      const vertical = [project([value, -extent, -0.03]), project([value, extent, -0.03])].filter(Boolean);
      for (const line of [horizontal, vertical]) {
        if (line.length !== 2) continue;
        ctx.beginPath(); ctx.moveTo(line[0].x, line[0].y); ctx.lineTo(line[1].x, line[1].y); ctx.stroke();
      }
    }
  }

  drawTrajectory(project) {
    if (!this.overlays.trajectory || !this.store.records.length) return;
    const currentTime = this.store.current?.timestamp_s ?? Infinity;
    const sampled = this.store.records.filter((_, index) => index % Math.max(1, Math.floor(this.store.records.length / 1200)) === 0)
      .filter(record => record.timestamp_s <= currentTime)
      .map(record => project([record.pose.x_m, record.pose.y_m, record.pose.z_m + .035]))
      .filter(Boolean);
    if (sampled.length < 2) return;
    this.context.beginPath();
    this.context.moveTo(sampled[0].x, sampled[0].y);
    for (const point of sampled.slice(1)) this.context.lineTo(point.x, point.y);
    this.context.strokeStyle = "#5bc0d8";
    this.context.lineWidth = 2;
    this.context.stroke();
    if (this.store.comparison?.length) {
      const comparison = this.store.comparison
        .map(record => project([record.right_x_m, record.right_y_m, .055]))
        .filter(Boolean);
      if (comparison.length >= 2) {
        this.context.beginPath();
        this.context.moveTo(comparison[0].x, comparison[0].y);
        for (const point of comparison.slice(1)) this.context.lineTo(point.x, point.y);
        this.context.strokeStyle = "#a98cf4";
        this.context.lineWidth = 1.5;
        this.context.setLineDash([5, 4]);
        this.context.stroke();
        this.context.setLineDash([]);
      }
    }
  }

  animate(now = performance.now()) {
    this.frameTimes.push(now);
    while (this.frameTimes.length && now - this.frameTimes[0] > 1000) this.frameTimes.shift();
    this.fps = this.frameTimes.length;
    const frame = this.store.current;
    const project = this.camera.projector(frame, this.store.track, this.width, this.height);
    this.drawGround(project);
    this.trackRenderer.draw(this.context, this.store.track, project, this.overlays, this.store.faultEvents);
    this.drawTrajectory(project);
    this.kartRenderer.draw(this.context, frame, this.store.kart, project, this.overlays);
    requestAnimationFrame(time => this.animate(time));
  }
}
