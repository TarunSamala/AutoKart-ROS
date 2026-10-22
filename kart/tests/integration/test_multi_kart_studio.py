from pathlib import Path

from kart_sim.multi_kart import MultiKartRace
from kart_sim.studio_scenario import validate_studio_scenario


ROOT = Path(__file__).parents[2]


def test_scenario_editor_contract_and_multi_kart_namespaces() -> None:
    report = validate_studio_scenario(ROOT / "configs/studio/cone_track_demo.yaml", root=ROOT)
    assert report["valid"] and report["physics_authority"] == "internal"
    race = MultiKartRace(
        ROOT / "configs/tracks/cone_track.yaml",
        ROOT / "configs/karts/prototype_dynamic.yaml",
        agent_count=2,
        control_dt_s=0.01,
    )
    reset = race.reset(seed=11)
    assert set(reset) == {"/kart_01", "/kart_02"}
    results = race.step({"/kart_01": (0.0, 0.0), "/kart_02": (0.0, 0.0)})
    assert set(results) == set(reset)
    assert all(result[4]["namespace"] in reset for result in results.values())
    assert all("race_control" in result[4] for result in results.values())
