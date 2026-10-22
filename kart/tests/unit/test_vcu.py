from kart_sim.vcu import *
def test_startup_invalid_transition_and_autonomy_command():
 v=VCU();assert not v.transition(VcuState.AUTONOMOUS);assert v.transition(VcuState.BOOT);assert v.transition(VcuState.SELF_TEST);assert v.transition(VcuState.SAFE);assert v.transition(VcuState.READY);assert v.request_autonomous(AutonomousGuards());v.independent.pulse(0);c=MotionCommand("jetson",1,0,.1,20,.1).signed();assert v.update(0,c).drive==1
def test_stale_and_independent_watchdog_stop():
 v=VCU();v.state=VcuState.AUTONOMOUS;v.independent.pulse(0);assert v.update(.2,None).brake>0;v=VCU();v.state=VcuState.AUTONOMOUS;assert v.update(.1,None).brake==1
def test_reset_guards_and_physical_stop_completion():
 v=VCU();v.state=VcuState.EMERGENCY_STOP
 assert not v.reset(authorized=True,vehicle_speed_mps=1,motor_torque_nm=0,estop_active=False,brake_pressure_pa=2e5,feedback_healthy=True).accepted
 assert v.reset(authorized=True,vehicle_speed_mps=0,motor_torque_nm=0,estop_active=False,brake_pressure_pa=2e5,feedback_healthy=True).next_state==VcuState.SELF_TEST
 v=VCU();v.state=VcuState.CONTROLLED_STOP;v.independent.pulse(0)
 for t in (.0,.06):
  v.independent.pulse(t); v.cycle(VcuInputs(t,None,0,0,2e5,0,0,True))
 assert v.state==VcuState.SAFE
