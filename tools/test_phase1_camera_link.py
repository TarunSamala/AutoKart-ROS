from tools.phase1_camera_link import decision_code, fields


def test_observation_fields_keep_bumper_to_center_target_and_stop():
    row = {"stamp_ns": 1_217_000_000, "cone_id": 7, "cone_forward_m": 1.2168,
           "bumper_to_center_m": 0.15, "decision": "AT_OR_INSIDE_TARGET"}
    assert fields(row) == (1217, 7, 1217, 150, 2)
    assert decision_code(row) == 2
    missing = {"stamp_ns": 0, "cone_id": 7, "cone_forward_m": None,
               "bumper_to_center_m": None, "decision": "STOP"}
    assert fields(missing) == (0, -1, -1, -1, 0)
