"""Step 9: runs my_track over the nominal case plus degraded-sensor /
degraded-capability variants, both kart profiles, and writes a CSV plus a
one-page markdown summary with a Limits section. Mirrors the shape of
docs/my_track_spec/MY_TRACK_SCENARIO_MATRIX.csv (the prototype draft's own
reference run) so the two are directly comparable.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path

from .my_track_runner import default_run_config, run_my_track_scenario


@dataclass(frozen=True)
class Variant:
    name: str
    seeds: int
    apply: object  # Callable[[RunConfig], RunConfig]


def _nominal(config):
    return config


def _latency_0_5s(config):
    config.camera_config = replace(config.camera_config, latency_s=0.5)
    return config


def _range_3m(config):
    config.camera_config = replace(config.camera_config, zmax_m=3.0)
    return config


def _hfov_58(config):
    config.camera_config = replace(config.camera_config, hfov_deg=58.0)
    return config


def _depth_noise_x2(config):
    config.camera_config = replace(
        config.camera_config,
        depth_sigma_r0_m=config.camera_config.depth_sigma_r0_m * 2,
        depth_sigma_r1_m_per_m2=config.camera_config.depth_sigma_r1_m_per_m2 * 2,
    )
    return config


def _recall_0_75(config):
    config.camera_config = replace(config.camera_config, recall_peak=0.75)
    return config


def _label_swap_8pct(config):
    config.camera_config = replace(config.camera_config, label_swap_p=0.08)
    return config


def _gyro_bias_x3(config):
    # NOT IMPLEMENTED: this runner uses the kart's true pose/speed directly
    # (no separate encoder/gyro dead-reckoning estimate was built -- see
    # docs/MY_TRACK.md). This variant is a documented no-op, not a silent
    # skip: it runs the nominal case again so the matrix still reports a
    # row for it, flagged in the Limits section rather than omitted.
    return config


def _detour_disabled(config):
    config.detour_enabled = False
    return config


def _speed_0_8(config):
    config.cruise_speed_mps = 0.8
    return config


VARIANTS = (
    Variant("nominal", 12, _nominal),
    Variant("latency_0.5s", 6, _latency_0_5s),
    Variant("range_3m", 6, _range_3m),
    Variant("hfov_58", 6, _hfov_58),
    Variant("depth_noise_x2", 6, _depth_noise_x2),
    Variant("recall_0.75", 6, _recall_0_75),
    Variant("label_swap_8pct", 6, _label_swap_8pct),
    Variant("gyro_bias_x3", 6, _gyro_bias_x3),  # documented no-op, see above
    Variant("detour_disabled", 6, _detour_disabled),
    Variant("speed_0.8", 6, _speed_0_8),
)


def run_matrix(profiles: tuple[str, ...] = ("shared", "built"), variants: tuple[Variant, ...] = VARIANTS) -> list[dict]:
    rows = []
    for profile in profiles:
        for variant in variants:
            outcomes = []
            for seed in range(1, variant.seeds + 1):
                config = default_run_config(profile, seed=seed)
                config = variant.apply(config)
                result = run_my_track_scenario(config)
                outcomes.append(result.metrics)
            completed = sum(1 for m in outcomes if m.outcome == "COMPLETE")
            collisions = sum(m.collisions for m in outcomes)
            rows.append({
                "kart_profile": profile, "scenario": variant.name, "runs": variant.seeds,
                "completed": completed, "collisions": collisions,
                "min_clearance_m": round(min(m.min_clearance_m for m in outcomes), 3),
                "max_speed_mps": round(max(m.max_speed_mps for m in outcomes), 3),
                "max_steer_deg": round(max(m.max_steer_deg for m in outcomes), 1),
            })
    return rows


def write_matrix_csv(rows: list[dict], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_matrix_summary_md(rows: list[dict], path: Path, *, assumed_values: list[str]) -> None:
    lines = [
        "# my_track scenario matrix", "",
        f"Generated {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')}.", "",
        "| profile | scenario | runs | completed | collisions | min_clearance_m | max_speed_mps | max_steer_deg |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for row in rows:
        lines.append(
            f"| {row['kart_profile']} | {row['scenario']} | {row['runs']} | {row['completed']} | {row['collisions']} | "
            f"{row['min_clearance_m']} | {row['max_speed_mps']} | {row['max_steer_deg']} |"
        )
    total_collisions = sum(row["collisions"] for row in rows)
    total_runs = sum(row["runs"] for row in rows)
    total_completed = sum(row["completed"] for row in rows)
    lines += [
        "", f"**Totals: {total_completed}/{total_runs} completed, {total_collisions} collisions.**", "",
        "## Limits", "",
        "- Values still ASSUMED (not measured on the real kart/HP60C): mass, tyre cornering "
        "stiffness, drive/brake force limits, steering rate/lag, brake dead-time/ramp, cone "
        "height, detector recall curve, colour-swap rate, depth/lateral noise coefficients, "
        "and the finish-detection gap thresholds (recalibrated against this layout's own "
        "map-blending behaviour -- see my_track_algorithm.py). Full list in docs/MY_TRACK.md.",
        "- Sunlight / structured-light degradation risk: the HP60C is a structured-light depth "
        "sensor; direct sunlight is a known failure mode for that technology. The simulated "
        "switch (off by default here) only scales effective range and adds dropout "
        "probability -- it has not been validated against a real outdoor test.",
        "- Simulation success here does not prove the real kart is safe. This is a software-in-the-loop "
        "model with ASSUMED dynamics, actuator, and perception parameters; it has not been "
        "validated against the physical vehicle.",
        "- An independent hardware E-stop is required for any powered test, regardless of what "
        "this simulation reports.",
        "- The first powered test of the real vehicle must be a supervised creep at very low speed, "
        "not a run of this scenario at cruise speed.",
    ]
    if assumed_values:
        lines += ["", "## Known failure modes from this matrix", ""] + [f"- {item}" for item in assumed_values]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(description="Run the my_track scenario matrix (Step 9).")
    parser.add_argument("--out", type=Path, default=Path("results/my_track/matrix"))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    rows = run_matrix()
    write_matrix_csv(rows, args.out / "matrix.csv")
    write_matrix_summary_md(rows, args.out / "SUMMARY.md", assumed_values=[
        "gyro_bias_x3 is a documented no-op (re-runs nominal): no separate encoder/gyro dead-reckoning "
        "estimate exists in this runner, only the kart's true pose/speed.",
        "A residual ~12.5% real-collision rate was observed in the nominal matrix during development "
        "(3/24 seeds), traced to Pure Pursuit tracking error against a once-computed, locked detour "
        "offset; raising the planning margin did not reliably fix it (it only reshuffled which seeds "
        "collide vs. abort). See docs/MY_TRACK.md for the root cause and what a real fix needs.",
    ])
    print(f"Wrote {args.out / 'matrix.csv'} and {args.out / 'SUMMARY.md'}")


if __name__ == "__main__":
    main()
