from types import SimpleNamespace as NS

from autokart_perception_ros.node import qualified


def result(calibration_status, reasons=(), **diagnostics):
    perception = NS(safety=NS(reasons=list(reasons)), status="OK",
                    diagnostics={"calibration_status": calibration_status, **diagnostics})
    return NS(perception_result=perception)


LIVE = NS(source="LIVE", calibration={})


def test_live_unverified_calibration_never_qualifies():
    ok, reasons = qualified(result("MOUNT_CONFIGURED_UNVERIFIED"), LIVE)
    assert not ok and reasons[0] == "LIVE_METRIC_CALIBRATION_UNVERIFIED"


def test_live_verified_calibration_qualifies_even_with_track_geometry_reasons():
    ok, reasons = qualified(result("MOUNT_VERIFIED", ["GATE_AMBIGUOUS", "CENTERLINE_AMBIGUOUS"]), LIVE)
    assert ok and reasons == []


def test_live_verified_still_blocked_by_stale_frozen_or_unverified_model():
    for blocker in ("FRAME_TOO_OLD", "MODEL_UNVERIFIED", "DEPTH_UNAVAILABLE", "INFERENCE_OVERLOAD"):
        ok, reasons = qualified(result("MOUNT_VERIFIED", [blocker]), LIVE)
        assert not ok and blocker in reasons
    ok, reasons = qualified(result("MOUNT_VERIFIED", frame_frozen=True), LIVE)
    assert not ok and "NONMONOTONE_FRAME" in reasons
