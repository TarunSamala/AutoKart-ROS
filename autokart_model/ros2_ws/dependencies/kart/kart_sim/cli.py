"""Command line entry points for the implemented headless baseline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from threading import Thread
import pytest
from dataclasses import asdict

from .config import load_kart_config, load_yaml
from .scenario import run_scenario
from .track import Track, segment_corners
from .track_formats import export_track, validate_track_source
from .ros2_sil import Ros2Runtime, load_sil_configuration
from .fast_env import AutoKartFastEnv
from .multi_kart import MultiKartRace
from .racing_autonomy import FollowTheGapController, track_trajectory
from .autonomy import PurePursuitController, StanleyController
from .sim_studio import load_preset
from .studio_scenario import validate_studio_scenario
from .robot_description import validate_sdf, validate_urdf
from .studio_performance import measure_supported_tiers, performance_payload
from .dashboard import DashboardExporter, TelemetryStore, export_dashboard_bundle
from .dashboard_server import DashboardServerConfig, serve_dashboard
from .benchmark import CampaignCase, CampaignRunner, coverage, load_catalog
from .safety_config import load_safety_configuration
from .safety_metrics import derive_safety_metrics
import yaml


def main() -> int:
    parser = argparse.ArgumentParser(prog="kart-sim")
    subcommands = parser.add_subparsers(dest="command", required=True)
    validate = subcommands.add_parser("validate", help="validate a kart YAML configuration")
    validate.add_argument("configuration", type=Path)
    run = subcommands.add_parser("run", help="run a deterministic headless scenario")
    run.add_argument("scenario", type=Path)
    run.add_argument("--output-dir", default="runs")
    model_test = subcommands.add_parser("test-model", help="run focused deterministic model tests")
    model_test.add_argument(
        "model",
        choices=("steering", "brake", "powertrain", "thermal", "autonomy", "vcu", "controlled-stop", "emergency-stop", "safety-envelope"),
    )
    vcu_tool = subcommands.add_parser("vcu", help="VCU inspection tools")
    vcu_sub = vcu_tool.add_subparsers(dest="vcu_command", required=True)
    state_machine = vcu_sub.add_parser("state-machine")
    state_machine.add_argument("--diagram", action="store_true")
    ros2 = subcommands.add_parser("ros2", help="optional ROS 2/SIL adapter tools")
    ros2_sub = ros2.add_subparsers(dest="ros2_command", required=True)
    ros2_sub.add_parser("status", help="detect optional ROS 2 runtime")
    ros2_validate = ros2_sub.add_parser("validate", help="validate ROS 2/SIL configuration")
    ros2_validate.add_argument("configuration", type=Path)
    dashboard = subcommands.add_parser("dashboard", help="headless engineering dashboard exports")
    dashboard_sub = dashboard.add_subparsers(dest="dashboard_command", required=True)
    dashboard_inspect = dashboard_sub.add_parser("inspect", help="summarize a telemetry JSONL file")
    dashboard_inspect.add_argument("telemetry", type=Path)
    dashboard_export = dashboard_sub.add_parser("export", help="export static HTML, CSV, and JSON dashboard data")
    dashboard_export.add_argument("telemetry", type=Path); dashboard_export.add_argument("--output-dir", type=Path, required=True)
    dashboard_export.add_argument("--track", type=Path, default=Path("configs/tracks/research_circle.yaml"))
    dashboard_export.add_argument("--kart", type=Path, default=Path("configs/karts/prototype_dynamic.yaml"))
    dashboard_serve = dashboard_sub.add_parser("serve", help="serve the live/replay engineering dashboard")
    dashboard_serve.add_argument("telemetry", type=Path)
    dashboard_serve.add_argument("--track", type=Path, default=Path("configs/tracks/research_circle.yaml"))
    dashboard_serve.add_argument("--kart", type=Path, default=Path("configs/karts/prototype_dynamic.yaml"))
    dashboard_serve.add_argument("--host", default="127.0.0.1")
    dashboard_serve.add_argument("--port", type=int, default=8000)
    dashboard_serve.add_argument("--rate-hz", type=float, default=30.0)
    dashboard_serve.add_argument("--live", action="store_true", help="tail a telemetry file that is still growing")
    dashboard_launch = dashboard_sub.add_parser("launch", help="run a real-time scenario with the live dashboard")
    dashboard_launch.add_argument("scenario", type=Path)
    dashboard_launch.add_argument("--output-dir", type=Path, default=Path("runs/dashboard-live"))
    dashboard_launch.add_argument("--track", type=Path)
    dashboard_launch.add_argument("--kart", type=Path)
    dashboard_launch.add_argument("--host", default="127.0.0.1")
    dashboard_launch.add_argument("--port", type=int, default=8000)
    dashboard_launch.add_argument("--rate-hz", type=float, default=30.0)
    benchmark = subcommands.add_parser("benchmark", help="deterministic safety fault benchmark")
    benchmark_sub = benchmark.add_subparsers(dest="benchmark_command", required=True)
    benchmark_run = benchmark_sub.add_parser("critical-smoke", help="run critical VCU fault campaign")
    benchmark_run.add_argument("--catalog", type=Path, default=Path("configs/benchmarks/critical_smoke.yaml")); benchmark_run.add_argument("--output-dir", type=Path, default=Path("runs/benchmark"))
    safety = subcommands.add_parser("safety", help="safety configuration and campaign tools")
    safety_sub = safety.add_subparsers(dest="safety_command", required=True)
    safety_validate = safety_sub.add_parser("validate")
    safety_validate.add_argument("configuration", type=Path)
    safety_inspect = safety_sub.add_parser("inspect")
    safety_inspect.add_argument("telemetry", type=Path)
    safety_campaign = safety_sub.add_parser("campaign")
    safety_campaign.add_argument("--catalog", type=Path, default=Path("configs/benchmarks/critical_smoke.yaml"))
    safety_campaign.add_argument("--output-dir", type=Path, default=Path("runs/safety-campaign"))
    report = subcommands.add_parser("report", help="generate a machine-readable run report")
    report.add_argument("run", type=Path)
    report.add_argument("--safety", action="store_true")
    track = subcommands.add_parser("track", help="headless track tools")
    track_sub = track.add_subparsers(dest="track_command", required=True)
    for name in ("validate", "inspect", "segment-corners"):
        item=track_sub.add_parser(name); item.add_argument("track_file", type=Path)
        if name=="segment-corners": item.add_argument("--output", type=Path)
    query=track_sub.add_parser("query"); query.add_argument("track_file", type=Path); query.add_argument("--s", type=float, required=True); query.add_argument("--lateral", type=float, default=0.0)
    track_export = track_sub.add_parser("export")
    track_export.add_argument("track_file", type=Path)
    track_export.add_argument("--output-dir", type=Path, required=True)
    fast = subcommands.add_parser("fast", help="run VCU-routed F1TENTH-style fast environment")
    fast.add_argument("--track", type=Path, default=Path("configs/tracks/cone_track.yaml"))
    fast.add_argument("--kart", type=Path, default=Path("configs/karts/prototype_dynamic.yaml"))
    fast.add_argument("--steps", type=int, default=1000)
    fast.add_argument("--seed", type=int, default=20260729)
    fast.add_argument("--speed", type=float, default=3.0)
    fast.add_argument("--steering", type=float, default=0.0)
    fast.add_argument("--controller", choices=("pure_pursuit", "stanley", "follow_the_gap", "constant"), default="pure_pursuit")
    fast.add_argument("--output", type=Path)
    multi = subcommands.add_parser("multi-race", help="run namespaced multi-kart fast race")
    multi.add_argument("--track", type=Path, default=Path("configs/tracks/cone_track.yaml"))
    multi.add_argument("--kart", type=Path, default=Path("configs/karts/prototype_dynamic.yaml"))
    multi.add_argument("--agents", type=int, default=2)
    multi.add_argument("--steps", type=int, default=200)
    multi.add_argument("--seed", type=int, default=20260729)
    studio = subcommands.add_parser("studio", help="AutoKart Sim Studio configuration tools")
    studio_sub = studio.add_subparsers(dest="studio_command", required=True)
    preset_validate = studio_sub.add_parser("validate-preset")
    preset_validate.add_argument("preset", type=Path)
    scenario_validate = studio_sub.add_parser("validate-scenario")
    scenario_validate.add_argument("scenario", type=Path)
    robot = subcommands.add_parser("robot", help="robot description validation")
    robot_sub = robot.add_subparsers(dest="robot_command", required=True)
    robot_validate = robot_sub.add_parser("validate")
    robot_validate.add_argument("description", type=Path)
    performance = subcommands.add_parser("performance", help="measure local simulation fidelity tiers")
    performance.add_argument("--output-dir", type=Path, default=Path("runs/performance"))
    arguments = parser.parse_args()
    if arguments.command == "validate":
        config = load_kart_config(arguments.configuration)
        print(f"valid: {arguments.configuration} ({config.kart.name})")
        return 0
    if arguments.command == "test-model":
        paths = ["tests/unit/test_phase4_subsystems.py", "tests/integration/test_dynamic_kart.py"]
        if arguments.model == "autonomy": paths = ["tests/unit/test_autonomy.py", "tests/integration/test_ros2_sil.py"]
        if arguments.model == "vcu": paths = ["tests/unit/test_vcu.py", "tests/unit/test_vcu_state_guards.py", "tests/integration/test_vcu_scenario.py"]
        if arguments.model == "controlled-stop": paths = ["tests/unit/test_controlled_stop_reset.py", "tests/integration/test_controlled_stop_scenarios.py"]
        if arguments.model == "emergency-stop": paths = ["tests/integration/test_phase7_estop_envelope.py"]
        if arguments.model == "safety-envelope": paths = ["tests/unit/test_vcu_command_envelope.py"]
        return pytest.main(["-q", *paths])
    if arguments.command == "vcu":
        diagram = """stateDiagram-v2
  POWER_OFF --> BOOT
  BOOT --> SELF_TEST
  SELF_TEST --> SAFE
  SAFE --> READY
  READY --> AUTONOMOUS: guarded request
  AUTONOMOUS --> DEGRADED
  AUTONOMOUS --> CONTROLLED_STOP
  CONTROLLED_STOP --> SAFE: measured stop
  CONTROLLED_STOP --> EMERGENCY_STOP: failure
  EMERGENCY_STOP --> SELF_TEST: guarded reset
  FAULT_LOCKOUT --> SELF_TEST: guarded reset"""
        print(diagram if arguments.diagram else "VCU state machine available; use --diagram")
        return 0
    if arguments.command == "ros2":
        if arguments.ros2_command == "status":
            runtime = Ros2Runtime.detect()
            print(yaml.safe_dump({"ros2_cli_available": runtime.cli_available, "rclpy_available": runtime.rclpy_available}, sort_keys=True))
            return 0
        configuration = load_sil_configuration(arguments.configuration)
        print(f"valid ROS 2/SIL configuration: {arguments.configuration} ({configuration.controller.lateral})")
        return 0
    if arguments.command == "dashboard":
        if arguments.dashboard_command == "launch":
            scenario_document = load_yaml(arguments.scenario)
            scenario_name = str(scenario_document["name"])
            destination = arguments.output_dir / scenario_name
            destination.mkdir(parents=True, exist_ok=True)
            live_telemetry = destination / "telemetry-live.jsonl"
            live_telemetry.write_text("", encoding="utf-8")
            track_path = arguments.track or (
                arguments.scenario.parent.parent / scenario_document.get("track_config", "tracks/research_circle.yaml")
            )
            kart_path = arguments.kart or (
                arguments.scenario.parent.parent / scenario_document["kart_config"]
            )
            (destination / "metadata.json").write_text(
                json.dumps(
                    {
                        "scenario": scenario_name,
                        "physics_dt_s": None,
                        "seed": None,
                        "control_mode": scenario_document.get("control", {}).get("mode", "direct_test"),
                        "live": True,
                    },
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )

            def simulate() -> None:
                with live_telemetry.open("w", encoding="utf-8", buffering=1) as stream:
                    run_scenario(
                        arguments.scenario,
                        arguments.output_dir,
                        realtime=True,
                        record_callback=lambda record: stream.write(json.dumps(record, sort_keys=True) + "\n"),
                    )

            Thread(target=simulate, name="kart-sim-live", daemon=True).start()
            serve_dashboard(
                DashboardServerConfig(
                    telemetry=live_telemetry,
                    track=Path(track_path),
                    kart=Path(kart_path),
                    static_root=Path(__file__).resolve().parent.parent / "dashboard",
                    host=arguments.host,
                    port=arguments.port,
                    telemetry_rate_hz=arguments.rate_hz,
                    live_tail=True,
                )
            )
            return 0
        if arguments.dashboard_command == "serve":
            serve_dashboard(
                DashboardServerConfig(
                    telemetry=arguments.telemetry,
                    track=arguments.track,
                    kart=arguments.kart,
                    static_root=Path(__file__).resolve().parent.parent / "dashboard",
                    host=arguments.host,
                    port=arguments.port,
                    telemetry_rate_hz=arguments.rate_hz,
                    live_tail=arguments.live,
                )
            )
            return 0
        store = TelemetryStore.from_jsonl(arguments.telemetry)
        if arguments.dashboard_command == "inspect":
            latest = store.records[-1] if store.records else None
            print(yaml.safe_dump({"records": len(store.records), "latest_simulation_time_s": latest.timestamp_s if latest else None,
                                  "fault_events": len(store.fault_events())}, sort_keys=True))
            return 0
        arguments.output_dir.mkdir(parents=True, exist_ok=True)
        export_dashboard_bundle(
            store,
            arguments.output_dir,
            track=Track.from_yaml(arguments.track),
            kart=load_kart_config(arguments.kart),
            static_root=Path(__file__).resolve().parent.parent / "dashboard",
            run_id=arguments.telemetry.parent.name,
        )
        DashboardExporter.export_csv(store, arguments.output_dir / "telemetry.csv")
        DashboardExporter.export_json(store, arguments.output_dir / "telemetry.json")
        print(f"exported: {arguments.output_dir}")
        return 0
    if arguments.command == "benchmark":
        catalog = load_catalog(arguments.catalog)
        cases = [
            CampaignCase("brake_cable_break", "configs/scenarios/vcu_brake_cable_break.yaml", ("BRAKE-CABLE-BREAK",), 1),
            CampaignCase("motor_stuck_torque", "configs/scenarios/vcu_motor_stuck_torque.yaml", ("MOTOR-STUCK-TORQUE",), 1),
            CampaignCase("vcu_freeze", "configs/scenarios/vcu_task_freeze.yaml", ("VCU-FREEZE",), 1),
        ]
        results = CampaignRunner(catalog, arguments.output_dir).run(cases)
        summary = coverage(catalog, results, cases)
        print(yaml.safe_dump(summary, sort_keys=True))
        return 0 if not summary["uncovered_faults"] and all(result.passed for result in results) else 1
    if arguments.command == "safety":
        if arguments.safety_command == "validate":
            config = load_safety_configuration(arguments.configuration)
            print(f"valid simulation safety configuration: {arguments.configuration} ({config.vcu_rate_hz:g} Hz)")
            return 0
        if arguments.safety_command == "inspect":
            rows = [json.loads(line) for line in arguments.telemetry.read_text(encoding="utf-8").splitlines()]
            print(yaml.safe_dump(asdict(derive_safety_metrics(rows)), sort_keys=True))
            return 0
        catalog = load_catalog(arguments.catalog)
        cases = [
            CampaignCase("brake_cable_break", "configs/scenarios/vcu_brake_cable_break.yaml", ("BRAKE-CABLE-BREAK",), 1),
            CampaignCase("motor_stuck_torque", "configs/scenarios/vcu_motor_stuck_torque.yaml", ("MOTOR-STUCK-TORQUE",), 1),
            CampaignCase("vcu_freeze", "configs/scenarios/vcu_task_freeze.yaml", ("VCU-FREEZE",), 1),
        ]
        results = CampaignRunner(catalog, arguments.output_dir).run(cases)
        summary = coverage(catalog, results, cases)
        print(yaml.safe_dump(summary, sort_keys=True))
        return 0 if all(result.passed for result in results) else 1
    if arguments.command == "report":
        telemetry = arguments.run / "telemetry.jsonl" if arguments.run.is_dir() else arguments.run
        rows = [json.loads(line) for line in telemetry.read_text(encoding="utf-8").splitlines()]
        if arguments.safety:
            print(yaml.safe_dump(asdict(derive_safety_metrics(rows)), sort_keys=True))
            return 0
        print(yaml.safe_dump({"records": len(rows)}, sort_keys=True))
        return 0
    if arguments.command == "track":
        if arguments.track_command == "export":
            paths = export_track(arguments.track_file, arguments.output_dir)
            print(yaml.safe_dump({name: str(path) for name, path in paths.items()}, sort_keys=True))
            return 0
        model=Track.from_yaml(arguments.track_file)
        if arguments.track_command=="validate":
            report = validate_track_source(arguments.track_file)
            issues=report["issues"]; print(yaml.safe_dump(report, sort_keys=True)); return 1 if any(x.startswith("ERROR") for x in issues) else 0
        if arguments.track_command=="inspect":
            print(yaml.safe_dump({"name":model.name,"length_m":model.length_m,"closed":model.closed,"sample":model.sample(0).__dict__},sort_keys=True)); return 0
        if arguments.track_command=="query":
            print(yaml.safe_dump({"geometry":asdict(model.sample(arguments.s)),"surface":asdict(model.surface_at(arguments.s,arguments.lateral))},sort_keys=True)); return 0
        corners=segment_corners(model); text=yaml.safe_dump({"corners":corners},sort_keys=False)
        if arguments.output: arguments.output.write_text(text,encoding="utf-8")
        else: print(text)
        return 0
    if arguments.command == "fast":
        if arguments.steps <= 0:
            raise ValueError("--steps must be positive")
        env = AutoKartFastEnv(arguments.track, arguments.kart, maximum_steps=arguments.steps)
        observation, info = env.reset(
            seed=arguments.seed,
            options={"arm": True, "operator_acknowledged": True},
        )
        trajectory = track_trajectory(env.track, timestamp_s=0.0, speed_mps=arguments.speed)
        lateral = PurePursuitController(
            wheelbase_m=env.config.kart.wheelbase_m,
            maximum_steering_rate_radps=4.0,
        ) if arguments.controller == "pure_pursuit" else StanleyController(maximum_steering_rate_radps=4.0)
        gap = FollowTheGapController(maximum_speed_mps=arguments.speed)
        total_reward = 0.0
        records = []
        terminated = truncated = False
        for step_index in range(arguments.steps):
            if arguments.controller == "follow_the_gap":
                gap_output = gap.compute(observation["lidar"])
                action = (gap_output.target_speed_mps, gap_output.steering_rad)
            elif arguments.controller in {"pure_pursuit", "stanley"}:
                output = lateral.compute(env.state, trajectory, env.control_dt_s)
                action = (arguments.speed, output.steering_rad)
            else:
                action = (arguments.speed, arguments.steering)
            observation, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            records.append({
                "step": step_index,
                "state": asdict(env.state),
                "track_s_m": info["track_s_m"],
                "lateral_m": info["lateral_m"],
                "action": action,
                "reward": reward,
                "safety_state": observation["safety_state"],
                "faults": info["vcu_faults"],
            })
            if terminated or truncated:
                break
        summary = {
            "steps": len(records),
            "seed": arguments.seed,
            "controller": arguments.controller,
            "physics_backend": "internal",
            "vcu_routed": True,
            "total_reward": total_reward,
            "terminated": terminated,
            "truncated": truncated,
            "laps": info["lap_count"],
            "final": records[-1] if records else None,
        }
        if arguments.output:
            arguments.output.parent.mkdir(parents=True, exist_ok=True)
            arguments.output.write_text(
                "\n".join(json.dumps(record, sort_keys=True) for record in records) + "\n",
                encoding="utf-8",
            )
        print(yaml.safe_dump(summary, sort_keys=True))
        return 0 if not info["vcu_faults"] else 1
    if arguments.command == "multi-race":
        race = MultiKartRace(arguments.track, arguments.kart, agent_count=arguments.agents)
        race.reset(seed=arguments.seed)
        collisions = 0
        for _ in range(arguments.steps):
            results = race.step({env.namespace: (2.0, 0.0) for env in race.environments})
            collisions += sum(bool(result[4].get("collision")) for result in results.values())
            if all(result[2] or result[3] for result in results.values()):
                break
        print(yaml.safe_dump({
            "agents": arguments.agents,
            "namespaces": [env.namespace for env in race.environments],
            "collisions": collisions,
            "race_control": race.race_control.snapshot(),
        }, sort_keys=True))
        return 0
    if arguments.command == "studio":
        if arguments.studio_command == "validate-preset":
            preset = load_preset(arguments.preset)
            payload = asdict(preset)
            payload["mode"] = preset.mode.value
            payload["physics_backend"] = preset.physics_backend.value
            payload["gazebo_role"] = preset.gazebo_role.value
            print(yaml.safe_dump(payload, sort_keys=True))
            return 0
        print(yaml.safe_dump(validate_studio_scenario(arguments.scenario, root=Path.cwd()), sort_keys=True))
        return 0
    if arguments.command == "robot":
        report = validate_urdf(arguments.description) if arguments.description.suffix in {".urdf", ".xacro"} else validate_sdf(arguments.description)
        print(yaml.safe_dump(asdict(report), sort_keys=True))
        return 1 if report.issues else 0
    if arguments.command == "performance":
        reports = measure_supported_tiers(Path.cwd(), arguments.output_dir)
        payload = performance_payload(reports)
        arguments.output_dir.mkdir(parents=True, exist_ok=True)
        (arguments.output_dir / "performance.yaml").write_text(
            yaml.safe_dump(payload, sort_keys=False),
            encoding="utf-8",
        )
        print(yaml.safe_dump(payload, sort_keys=False))
        return 0
    output = run_scenario(arguments.scenario, arguments.output_dir)
    print(f"completed: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
