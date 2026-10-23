"""RC-06 audit follow-up: runtime parameter rejection and data_source provenance.

Unlike test_envelope.py (pure classification logic), this instantiates the
real rclpy node to exercise its on_set_parameters callback and data_source
handling directly -- not a text-contract check.
"""
from __future__ import annotations

from contextlib import contextmanager

import rclpy
from rclpy.parameter import Parameter
import pytest

from autokart_safety_bridge_ros.node import FrontSafetyEnvelopeMonitor


def string_param(name: str, value: str) -> Parameter:
    # Node(parameter_overrides=...) takes rclpy.parameter.Parameter instances,
    # not the rcl_interfaces.msg.Parameter wire message.
    return Parameter(name, value=value)


@contextmanager
def running_node(**kwargs):
    rclpy.init()
    try:
        node = FrontSafetyEnvelopeMonitor(**kwargs)
        try:
            yield node
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_refuses_to_start_with_vehicle_authority_not_off():
    # The on_set_parameters callback is registered before declare_parameter, so a launch-time
    # override is rejected there first (InvalidParameterValueException), before the node's own
    # explicit startup check would even run -- an even earlier refusal than RC-05 established.
    with pytest.raises(rclpy.exceptions.InvalidParameterValueException, match="cannot be changed from 'OFF'"):
        with running_node(parameter_overrides=[string_param("vehicle_authority", "ARMED")]):
            pass


def test_refuses_to_start_with_safety_monitor_mode_not_diagnostic_only():
    with pytest.raises(rclpy.exceptions.InvalidParameterValueException, match="cannot be changed from 'DIAGNOSTIC_ONLY'"):
        with running_node(parameter_overrides=[string_param("safety_monitor_mode", "ACTIVE")]):
            pass


def test_rejects_runtime_change_to_vehicle_authority():
    with running_node() as node:
        before = node.get_parameter("vehicle_authority").value
        result = node.set_parameters([Parameter("vehicle_authority", value="ARMED")])
        assert result[0].successful is False
        assert "cannot be changed from 'OFF'" in result[0].reason
        after = node.get_parameter("vehicle_authority").value
        assert after == before == "OFF"


def test_rejects_runtime_change_to_safety_monitor_mode():
    with running_node() as node:
        result = node.set_parameters([Parameter("safety_monitor_mode", value="ACTIVE")])
        assert result[0].successful is False
        assert node.get_parameter("safety_monitor_mode").value == "DIAGNOSTIC_ONLY"


def test_accepts_runtime_change_to_an_unrelated_parameter():
    with running_node() as node:
        result = node.set_parameters([Parameter("minimum_bumper_clearance_m", value=1.5)])
        assert result[0].successful is True
        assert node.get_parameter("minimum_bumper_clearance_m").value == 1.5


def test_data_source_defaults_to_unverified_not_live():
    with running_node() as node:
        assert node.data_source == "UNVERIFIED"


def test_invalid_data_source_falls_back_to_unverified_rather_than_live():
    with running_node(parameter_overrides=[string_param("data_source", "DEFINITELY_REAL")]) as node:
        assert node.data_source == "UNVERIFIED"


@pytest.mark.parametrize("value", ["LIVE", "SIMULATED", "REPLAY"])
def test_recognized_data_source_values_pass_through_unchanged(value):
    with running_node(parameter_overrides=[string_param("data_source", value)]) as node:
        assert node.data_source == value
