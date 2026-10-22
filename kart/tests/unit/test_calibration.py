from kart_sim.calibration import bounded_linear_gain
from kart_sim.types import SimulationError
import pytest
def test_synthetic_bounded_recovery(): assert bounded_linear_gain([{'x':1,'y':2},{'x':2,'y':4}],'x','y',0,3)==2
def test_unidentifiable_rejected():
 with pytest.raises(SimulationError): bounded_linear_gain([{'x':0,'y':1}],'x','y',0,3)
