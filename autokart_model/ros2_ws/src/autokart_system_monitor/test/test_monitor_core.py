"""Unit tests for the pure health-evaluation logic. No ROS needed."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_system_monitor.monitor_core import (  # noqa: E402
    ERROR, OK, STALE, WARN, HealthReport, StartupLatch, evaluate_health,
)
from autokart_system_monitor.resource_backends import ResourceSample  # noqa: E402


def _sample(available=True, cpu=10.0, ram=20.0, reason="OK"):
    return ResourceSample(available=available, cpu_percent=cpu, ram_percent=ram, reason=reason)


def test_all_fresh_and_resources_fine_is_overall_ok():
    report = evaluate_health({"a": 0.1, "b": 0.2}, {"a": 1.0, "b": 1.0}, _sample())
    assert report.overall_level == OK
    assert all(c.level == OK for c in report.components)


def test_never_seen_heartbeat_is_stale_not_crash():
    report = evaluate_health({"a": None}, {"a": 1.0}, _sample())
    assert report.overall_level == STALE
    assert report.components[0].message == "never seen"


def test_stale_heartbeat_beyond_timeout_is_stale():
    report = evaluate_health({"a": 5.0}, {"a": 1.0}, _sample())
    assert report.overall_level == STALE
    assert "stale" in report.components[0].message


def test_dependency_not_running_degrades_gracefully_not_as_error():
    """A dependency node that simply isn't running (never published) must
    show up as STALE/never-seen, not crash this monitor or be silently
    ignored as if everything were fine."""
    report = evaluate_health({"hp60c": None, "safety": None}, {"hp60c": 1.0, "safety": 1.0}, _sample())
    assert report.overall_level == STALE
    names = {c.name: c.level for c in report.components}
    assert names["hp60c"] == STALE and names["safety"] == STALE


def test_resource_unavailable_reports_warn_not_fabricated_numbers():
    report = evaluate_health({}, {}, _sample(available=False, cpu=None, ram=None, reason="PSUTIL_NOT_INSTALLED"))
    resources = [c for c in report.components if c.name == "resources"][0]
    assert resources.level == WARN
    assert "PSUTIL_NOT_INSTALLED" in resources.message
    assert resources.values["cpu_percent"] == "null"


def test_high_cpu_or_ram_triggers_warn():
    report = evaluate_health({}, {}, _sample(cpu=95.0, ram=10.0), cpu_warn_pct=90.0, ram_warn_pct=90.0)
    resources = [c for c in report.components if c.name == "resources"][0]
    assert resources.level == WARN
    assert "cpu" in resources.message


def test_a_dependency_node_that_stops_publishing_shows_up_as_stale():
    """Explicit regression test for the mission's own fault-injection
    scenario: a dependency that WAS fresh and then stops must show stale,
    not silently continue looking fine."""
    fresh = evaluate_health({"hp60c": 0.05}, {"hp60c": 1.0}, _sample())
    assert fresh.overall_level == OK
    now_stopped = evaluate_health({"hp60c": 30.0}, {"hp60c": 1.0}, _sample())
    assert now_stopped.overall_level == STALE


def test_startup_latch_never_seen_stays_not_started():
    latch = StartupLatch(["hp60c", "safety"])
    latch.observe({"hp60c": None, "safety": None})
    assert not latch.fully_started
    status = latch.status()
    assert status.overall_level == STALE


def test_startup_latch_latches_true_and_never_regresses():
    latch = StartupLatch(["hp60c", "safety"])
    latch.observe({"hp60c": 0.1, "safety": None})
    assert not latch.fully_started
    latch.observe({"hp60c": 0.1, "safety": 0.1})
    assert latch.fully_started
    # hp60c goes stale/disappears afterwards -- startup latch must NOT
    # un-latch; that is /system/health's job, not /system/startup_status's.
    latch.observe({"hp60c": None, "safety": None})
    assert latch.fully_started
    assert latch.status().overall_level == OK
