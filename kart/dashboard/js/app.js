import {DashboardStateStore} from "./state-store.js";
import {ReplayController} from "./replay/replay-controller.js";
import {TelemetryConnection} from "./websocket.js";
import {EngineeringScene} from "./scene/scene.js";
import {PanelRenderer} from "./panels/panels.js";
import {FaultTimeline, TimeSeriesPlot} from "./plots/time-series.js";

const $ = id => document.getElementById(id);

async function json(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}: ${url}`);
  return response.json();
}

function csv(records) {
  const header = [
    "measurement_timestamp_s", "x_m", "y_m", "z_m", "yaw_rad", "speed_mps",
    "ax_mps2", "ay_mps2", "track_s_m", "lateral_error_m", "vcu_state",
    "active_faults", "motor_torque_nm", "brake_pressure_pa",
  ];
  const rows = records.map(record => [
    record.timestamp_s, record.pose.x_m, record.pose.y_m, record.pose.z_m,
    record.pose.yaw_rad, record.motion.speed_mps,
    record.motion.longitudinal_acceleration_mps2, record.motion.lateral_acceleration_mps2,
    record.track.s_m, record.track.lateral_m, record.vcu?.state,
    (record.vcu?.faults ?? []).join(";"),
    record.subsystems?.motor?.actual_torque_nm,
    record.subsystems?.brake?.hydraulic_pressure_pa,
  ]);
  return [header, ...rows].map(row => row.map(value => `"${String(value ?? "").replaceAll('"', '""')}"`).join(",")).join("\n");
}

function download(name, content, type) {
  const link = document.createElement("a");
  link.download = name;
  link.href = URL.createObjectURL(new Blob([content], {type}));
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 1000);
}

async function boot() {
  const store = new DashboardStateStore();
  let manifest, track, kart, telemetry;
  try {
    [manifest, track, kart, telemetry] = await Promise.all([
      json("/api/manifest"), json("/api/track"), json("/api/kart"), json("/api/telemetry?max_points=20000"),
    ]);
  } catch {
    [manifest, track, kart, telemetry] = await Promise.all([
      json("data/manifest.json"), json("data/track.json"), json("data/kart.json"), json("data/telemetry.json"),
    ]);
  }
  store.setStaticData({manifest, track, kart});
  store.replace(telemetry.records);
  store.setConnection(manifest.mode === "live" ? "connecting" : "replay");

  const replay = new ReplayController(store);
  const scene = new EngineeringScene($("scene-canvas"), store);
  scene.setTrack(track);
  const panels = new PanelRenderer(store);
  panels.staticData({manifest, track, kart});
  panels.connection();
  const plot = new TimeSeriesPlot($("timeseries-canvas"), store);
  plot.dispatchSeek = time => replay.seek(time);
  new FaultTimeline($("fault-timeline"), store, time => replay.seek(time));
  const connection = new TelemetryConnection(store);
  if (manifest.mode === "live") {
    connection.addEventListener("telemetry", event => {
      const end = replay.bounds().end;
      $("replay-range").max = end;
      $("replay-end").textContent = `${Number(end).toFixed(3)} s`;
      if (replay.playing || replay.time >= end - .1 || !store.current) replay.seek(event.detail.timestamp_s);
    });
    connection.connect();
  }

  const bounds = replay.bounds();
  replay.time = bounds.start;
  replay.seek(bounds.start);
  replay.start();
  $("replay-range").min = bounds.start;
  $("replay-range").max = bounds.end;
  $("replay-range").step = Math.max(.001, (bounds.end - bounds.start) / 10000);

  replay.addEventListener("play", event => { $("play-pause").textContent = event.detail ? "❚❚" : "▶"; });
  replay.addEventListener("seek", event => {
    $("replay-current").textContent = Number(event.detail).toFixed(3);
    $("replay-range").value = event.detail;
  });
  $("play-pause").addEventListener("click", () => replay.toggle());
  $("restart-replay").addEventListener("click", () => replay.restart());
  $("step-back").addEventListener("click", () => replay.step(-1));
  $("step-forward").addEventListener("click", () => replay.step(1));
  $("replay-range").addEventListener("input", event => { replay.setPlaying(false); replay.seek(Number(event.target.value)); });
  $("replay-speed").addEventListener("change", event => {
    replay.speed = Number(event.target.value);
    $("rt-factor").textContent = `${Number(event.target.value).toFixed(2)}×`;
  });
  $("replay-loop").addEventListener("change", event => { replay.loop = event.target.checked; });
  $("jump-fault").addEventListener("click", () => replay.jumpToFault());

  $("camera-mode").addEventListener("change", event => scene.setCameraMode(event.target.value));
  $("reset-camera").addEventListener("click", () => scene.resetCamera());
  $("overlay-toggle").addEventListener("click", () => $("overlay-menu").classList.toggle("open"));
  document.querySelectorAll("[data-overlay]").forEach(input => {
    input.addEventListener("change", () => scene.setOverlay(input.dataset.overlay, input.checked));
  });
  document.querySelectorAll("[data-panel]").forEach(button => button.addEventListener("click", () => {
    document.querySelectorAll("[data-panel]").forEach(item => item.classList.toggle("active", item === button));
    document.querySelectorAll("[data-page]").forEach(page => page.classList.toggle("active", page.dataset.page === button.dataset.panel));
  }));
  document.querySelectorAll("[data-plot]").forEach(button => button.addEventListener("click", () => {
    document.querySelectorAll("[data-plot]").forEach(item => item.classList.toggle("active", item === button));
    plot.setGroup(button.dataset.plot);
  }));
  $("export-csv").addEventListener("click", () => download(`${manifest.run_id}-telemetry.csv`, csv(store.records), "text/csv"));
  $("export-png").addEventListener("click", () => plot.exportPng());
  $("compare-run").addEventListener("click", async () => {
    const run = window.prompt("Comparison run directory name (beneath the current runs folder):");
    if (!run) return;
    try {
      const result = await json(`/api/comparison?other=${encodeURIComponent(run)}`);
      store.comparison = result.records;
      $("global-alert").textContent = `Comparison loaded: ${run} · ${result.records.length} distance-aligned samples`;
    } catch (error) {
      $("global-alert").textContent = `Comparison unavailable: ${error.message}`;
    }
  });

  let lastPerformancePoll = 0;
  const statusLoop = async now => {
    const stale = store.isStale();
    $("stale-overlay").classList.toggle("hidden", !stale);
    if (now - lastPerformancePoll > 2000) {
      lastPerformancePoll = now;
      try {
        const metrics = await json("/api/performance");
        $("performance-line").textContent = `render ${scene.fps} fps · stream ${Number(metrics.message_rate_hz).toFixed(1)} Hz · dropped ${metrics.dropped_messages} · bounded ${metrics.queue_bound} msgs`;
        const task = metrics.task_timing ?? {};
        const number = (value, scale = 1, digits = 2) =>
          Number.isFinite(Number(value)) ? (Number(value) * scale).toFixed(digits) : "—";
        $("perf-class").textContent = metrics.qualification;
        $("perf-status").textContent = metrics.qualification;
        $("performance-state-card").className =
          `safety-state-card ${metrics.qualification === "REALTIME_QUALIFIED" ? "qualified" : "degraded"}`;
        $("perf-rtf").textContent = number(metrics.simulation_rtf, 1, 3);
        $("perf-min-rtf").textContent = number(metrics.minimum_window_rtf, 1, 3);
        $("perf-physics").textContent = number(task.physics?.p99_s, 1000, 3);
        $("perf-vcu").textContent = number(task.vcu?.p99_s, 1000, 3);
        $("perf-safety").textContent = number(task.independent_safety?.maximum_s, 1000, 3);
        $("perf-follower").textContent = number(metrics.gazebo_follower_latency_s, 1000, 1);
        $("perf-cpu").textContent = number(metrics.cpu_time_s, 1, 2);
        $("perf-memory").textContent = number(metrics.memory_mib, 1, 1);
        $("perf-physics-misses").textContent = metrics.deadline_misses?.physics ?? "—";
        $("perf-vcu-misses").textContent = metrics.deadline_misses?.vcu ?? "—";
        $("perf-safety-misses").textContent = metrics.deadline_misses?.independent_safety ?? "—";
        $("perf-queue").textContent = `${Math.max(0, ...Object.values(metrics.sensor_queue_depth ?? {}).map(Number))} frames`;
        $("perf-sensor-latency").textContent = `${number(Math.max(0, ...Object.values(metrics.sensor_latency_s ?? {}).map(Number)), 1000, 1)} ms`;
        const reasons = Object.values(metrics.sensor_drops_by_reason ?? {}).flatMap(Object.entries);
        $("perf-drops").textContent = `${reasons.reduce((sum, [, count]) => sum + Number(count), 0)} categorized`;
        $("perf-ros").textContent = metrics.ros_callback_latency_s == null ? "NOT MEASURED" : `${number(metrics.ros_callback_latency_s, 1000, 2)} ms`;
        $("perf-gpu").textContent = String(metrics.gpu || "UNAVAILABLE").slice(0, 24);
      } catch {
        $("performance-line").textContent = `render ${scene.fps} fps · server metrics unavailable`;
      }
    }
    requestAnimationFrame(statusLoop);
  };
  requestAnimationFrame(statusLoop);

  // Start replay immediately so the physical telemetry visibly moves the kart.
  replay.setPlaying(true);
}

boot().catch(error => {
  console.error(error);
  $("global-alert").textContent = `DASHBOARD INITIALIZATION FAILED: ${error.message}`;
  $("connection-state").textContent = "FAILED";
  $("connection-state").className = "status-pill emergency-stop";
});
