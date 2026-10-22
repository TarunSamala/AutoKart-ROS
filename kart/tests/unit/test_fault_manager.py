from kart_sim.fault_manager import FaultManager, FaultStatus


def test_fault_manager_latches_counts_and_retains_history_after_reset():
    manager = FaultManager()
    record = manager.confirm("MOTOR-003", 1.0, severity="CRITICAL", response="EMERGENCY_STOP")
    manager.confirm("MOTOR-003", 1.1, severity="CRITICAL", response="EMERGENCY_STOP")
    assert record.occurrence_count == 2 and record.status == FaultStatus.ACTIVE
    assert manager.reset_latched(False) == ("MOTOR-003",)
    manager.reset_latched(True)
    assert not manager.active and manager.history["MOTOR-003"].status == FaultStatus.HISTORICAL
