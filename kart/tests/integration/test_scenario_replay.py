from pathlib import Path

from kart_sim.scenario import run_scenario

ROOT = Path(__file__).parents[2]


def test_empty_scenario_replays_bit_for_bit(tmp_path: Path) -> None:
    scenario = ROOT / "configs/scenarios/empty.yaml"
    first = run_scenario(scenario, tmp_path / "first")
    second = run_scenario(scenario, tmp_path / "second")
    assert (first / "metadata.json").read_bytes() == (second / "metadata.json").read_bytes()
    assert (first / "telemetry.jsonl").read_bytes() == (second / "telemetry.jsonl").read_bytes()
