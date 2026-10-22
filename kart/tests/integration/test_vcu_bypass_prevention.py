from pathlib import Path
import pytest
from kart_sim.scenario import run_scenario

ROOT = Path(__file__).parents[2]


def test_vcu_mode_rejects_direct_actuator_override_and_autonomy_rejects_direct_mode(tmp_path):
    base = """
name: bypass_attempt
kart_config: karts/prototype_dynamic.yaml
duration_s: 0.1
initial_state: {}
command: {drive: 0.5, brake: 0.0, steering_rad: 0.0}
"""
    scenario_dir = tmp_path / "configs" / "scenarios"
    kart_dir = tmp_path / "configs" / "karts"
    scenario_dir.mkdir(parents=True); kart_dir.mkdir(parents=True)
    (kart_dir / "prototype_dynamic.yaml").write_text((ROOT / "configs/karts/prototype_dynamic.yaml").read_text())
    bypass = scenario_dir / "bypass.yaml"
    bypass.write_text(base + "control: {mode: vcu, direct_actuator_command: {drive: 1.0}}\n")
    with pytest.raises(ValueError, match="prohibited"):
        run_scenario(bypass, tmp_path / "runs")
    direct = scenario_dir / "direct.yaml"
    direct.write_text(base + "control: {mode: direct_test, autonomous: true}\n")
    with pytest.raises(ValueError, match="VCU mode"):
        run_scenario(direct, tmp_path / "runs")
