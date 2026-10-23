import pytest
rclpy=pytest.importorskip("rclpy")
from autokart_vcu_bridge.node import legacy_envelopes
from autokart_vcu_bridge.backends import SerialVcuBackend,VcuBackendError

def test_missing_telemetry_never_qualifies_stop_or_wheel():
    wheel,stop=legacy_envelopes(None,"LIVE",1000,False)
    assert not wheel["valid"] and not wheel["clock_valid"]
    assert wheel["sample_time_ns"]==0
    assert not stop["ground_valid"] and not stop["ground_independent"]
    assert not stop["brake_valid"] and not stop["wheel_valid"]
    assert "STALE_TELEMETRY" in stop["faults"]

@pytest.mark.parametrize("method,args",[("send_command",(1,2000,0)),("send_request",(1,"ARM")),("send_request",(1,"RST"))])
def test_serial_outbound_is_disabled_even_if_called_directly(method,args):
    backend=object.__new__(SerialVcuBackend)
    with pytest.raises(VcuBackendError,match="DISABLED"):
        getattr(backend,method)(*args)
