from pathlib import Path

from kart_sim.benchmark import CampaignCase, CampaignRunner, coverage, load_catalog

ROOT = Path(__file__).parents[2]


def test_critical_campaign_is_deterministic_and_records_oracle_evidence(tmp_path):
    catalog = load_catalog(ROOT / "configs/benchmarks/critical_smoke.yaml")
    cases = [
        CampaignCase("brake", str(ROOT / "configs/scenarios/vcu_brake_cable_break.yaml"), ("BRAKE-CABLE-BREAK",), 7),
        CampaignCase("torque", str(ROOT / "configs/scenarios/vcu_motor_stuck_torque.yaml"), ("MOTOR-STUCK-TORQUE",), 7),
        CampaignCase("freeze", str(ROOT / "configs/scenarios/vcu_task_freeze.yaml"), ("VCU-FREEZE",), 7),
    ]
    results = CampaignRunner(catalog, tmp_path).run(cases)
    assert all(result.passed for result in results)
    assert CampaignRunner(catalog, tmp_path).run(cases, resume=True) == results
    report = coverage(catalog, results, cases)
    assert report["uncovered_faults"] == []
