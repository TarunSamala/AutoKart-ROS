from kart_sim.my_track_runner import default_run_config, run_my_track_scenario


def test_short_run_produces_telemetry_and_metrics_for_both_profiles() -> None:
    for profile in ("shared", "built"):
        config = default_run_config(profile, seed=1)
        config.max_duration_s = 5.0
        result = run_my_track_scenario(config)
        assert result.metrics.kart_profile == profile
        assert len(result.telemetry_rows) > 0
        assert result.metrics.frames_captured > 0
        assert result.metrics.min_clearance_m > -5.0  # sanity: not a degenerate/garbage run


def test_run_never_reports_a_pass_it_cannot_show() -> None:
    config = default_run_config("shared", seed=1)
    config.max_duration_s = 2.0  # far too short to finish; must be honestly reported as aborted
    result = run_my_track_scenario(config)
    assert result.metrics.outcome != "COMPLETE"
    assert result.metrics.outcome.startswith("ABORTED")
