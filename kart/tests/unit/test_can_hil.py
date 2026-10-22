import pytest
from kart_sim.can_hil import MotionCanMessage,bus_load
from kart_sim.types import SimulationError
def test_can_round_trip_crc_and_bus_load():
 m=MotionCanMessage(2,300,-50,100,100); assert MotionCanMessage.decode(m.encode())==m
 bad=m.encode();
 with pytest.raises(SimulationError): MotionCanMessage.decode(type(bad)(bad.arbitration_id,bad.data[:-1]+b'\0',0))
 assert 0<bus_load(100,8,500000)<1
