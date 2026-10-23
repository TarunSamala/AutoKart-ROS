from autokart_perception.release_audit import build_report
from test_pipeline import CFG


def test_release_report_is_blocked_and_unmeasured_gates_are_explicit(tmp_path):
    model=tmp_path/'fake.onnx';model.write_bytes(b'test fixture only')
    tests={'exit_code':0,'passed':53,'failed':0,'elapsed_s':2.0}
    negative={'runs':[{'ground_truth_cones':0,'false_positives':0,'precision':None,'recall':None}]}
    report=build_report(CFG,model,tests,negative,None)
    assert report['vehicle_operation_approval']=='BLOCKED'
    assert not report['actuator_interface_present'] and report['speed_limit_advisory_mps']==0
    assert report['gates']['automated_tests']['status']=='PASS'
    assert report['gates']['positive_cone_detection']['status']=='NOT_EXECUTED'
    assert report['gates']['surveyed_localization']['status']=='NOT_EXECUTED'
    assert report['gates']['target_pi_latency_p50_p95_p99']['status']=='NOT_EXECUTED'
    assert report['gates']['braking_and_emergency_stop']['status']=='NOT_EXECUTED'
