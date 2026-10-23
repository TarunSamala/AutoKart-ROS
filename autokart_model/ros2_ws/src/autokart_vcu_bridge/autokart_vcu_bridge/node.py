"""Read-only legacy VCU adapter. No outbound command or arm API."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from autokart_msgs.msg import SensorDeliveryMetadata, ActuatorFeedback, VcuState
from .backends import build_backend, VcuBackendError


def legacy_envelopes(tel, source, stamp_ns, fresh=True):
    """Legacy v2 has no boot, clock relation, calibrated steering or ground sensor."""
    faults = ["LEGACY_PROTOCOL_UNQUALIFIED", "BOOT_ID_UNAVAILABLE", "CLOCK_UNALIGNED",
              "ENCODER_CALIBRATION_UNVERIFIED", "INDEPENDENT_GROUND_SPEED_UNAVAILABLE"]
    if not fresh or tel is None:
        faults.append("STALE_TELEMETRY")
    common = dict(schema_version=1, source=source, sample_time_ns=0, arrival_time_ns=stamp_ns,
                  sequence=tel.fwseq if tel else 0, boot_id=None, clock_valid=False,
                  valid=False, faults=faults)
    wheel = dict(common, sample_time_us=tel.t_ms*1000 if tel else 0,
                 axle_count_total=tel.counts if tel else None, steering_actual_rad=None,
                 calibration_verified=False, calibration_hash="", steering_valid=False)
    stop = dict(common, wheel_speed_mps=tel.speed_mmps/1000.0 if tel and tel.speed_mmps >= 0 else None,
                wheel_valid=False, ground_speed_mps=None, ground_valid=False, ground_independent=False, ground_speed_source="UNAVAILABLE",
                brake_instrumented=False, brake_valid=False, brake_applied=False,
                vcu_valid=False, disarmed=False, stop_active=False, estop_active=bool(tel and tel.estop_input_fitted and not tel.estop_ok))
    return wheel, stop


class VcuBridgeNode(Node):
    def __init__(self, start_timer=True, **kwargs):
        super().__init__("autokart_vcu_bridge", **kwargs)
        for key, val in {"backend":"synthetic", "hardware_enabled":False,
                         "algorithm_package_path":"", "serial_port":"/dev/ttyACM0",
                         "serial_baud":115200, "tick_hz":20.0}.items():
            self.declare_parameter(key,val)
        self.wheel_pub=self.create_publisher(String,"/vehicle/wheel_encoder",10)
        self.stop_pub=self.create_publisher(String,"/vehicle/stop_evidence",10)
        self.telemetry_pub=self.create_publisher(String,"/vehicle/telemetry",10)
        self.status_pub=self.create_publisher(SensorDeliveryMetadata,"/vehicle/status",10)
        self.compat_status_pub=self.create_publisher(SensorDeliveryMetadata,"/vcu_bridge/status",10)
        self.feedback_pub=self.create_publisher(ActuatorFeedback,"/vehicle/actuator_feedback",10)
        self.vcu_pub=self.create_publisher(VcuState,"/vcu/state",10)
        self.backend=None
        self.seq=0
        self.last_tel=None
        self.last_arrival=None
        self.startup_error=None
        name=self.get_parameter("backend").value
        self.source="SYNTHETIC" if name=="synthetic" else "LIVE"
        try:
            if name!="synthetic" and not self.get_parameter("hardware_enabled").value:
                raise VcuBackendError("HARDWARE_ACCESS_DISABLED")
            self.backend=build_backend(name, algorithm_package_path=self.get_parameter("algorithm_package_path").value,
                                       port=self.get_parameter("serial_port").value,
                                       baud=self.get_parameter("serial_baud").value)
            self.backend.start()
        except Exception as exc:
            self.startup_error=str(exc)
            if self.backend:
                self.backend.close()
            self.backend=None
        self._timer=self.create_timer(1/max(1.0,self.get_parameter("tick_hz").value),self._tick) if start_timer else None

    def _tick(self):
        self.seq+=1
        error=self.startup_error
        if self.backend:
            try:
                for kind,value in self.backend.poll():
                    if kind=="TEL":
                        self.last_tel=value
                        self.last_arrival=time.monotonic()
            except Exception as exc:
                error=str(exc)
        fresh=self.last_arrival is not None and time.monotonic()-self.last_arrival<=0.3 and not error
        now=self.get_clock().now().nanoseconds
        wheel,stop=legacy_envelopes(self.last_tel,self.source,now,fresh)
        if error:
            wheel["faults"].append(error)
        self.wheel_pub.publish(String(data=json.dumps(wheel,allow_nan=False)))
        self.stop_pub.publish(String(data=json.dumps(stop,allow_nan=False)))
        tel={**wheel,"outputs_inhibited":True,"transport_read_only":True,
             "ack_seq": self.last_tel.ack_seq if self.last_tel else None,
             "firmware_command_age_ms":self.last_tel.link_age_ms if self.last_tel else None}
        self.telemetry_pub.publish(String(data=json.dumps(tel,allow_nan=False)))
        status=SensorDeliveryMetadata()
        status.header.stamp=self.get_clock().now().to_msg()
        status.sensor_id="vcu_bridge"
        status.sequence=self.seq
        status.encoding=self.source
        status.quality="INVALID"
        status.fault_state=error or "LEGACY_PROTOCOL_UNQUALIFIED"
        status.payload_present=bool(fresh)
        self.status_pub.publish(status)
        self.compat_status_pub.publish(status)
        feedback=ActuatorFeedback()
        feedback.measurement_time=status.header.stamp
        feedback.delivery_time=status.header.stamp
        feedback.valid=False
        # Unknown physical values stay invalid; never put linear m/s in angular rad/s.
        self.feedback_pub.publish(feedback)
        state=VcuState()
        state.timestamp=status.header.stamp
        state.state=self.last_tel.state_name if fresh else "UNKNOWN"
        state.motor_enable=bool(fresh and self.last_tel.en)
        state.contactor_open=False  # unknown; no measured contactor feedback in this protocol
        state.emergency_stop_active=stop["estop_active"]
        if hasattr(state,"estop_input_fitted"):
            state.estop_input_fitted=bool(fresh and self.last_tel.estop_input_fitted)
        state.limiting_reason="READ_ONLY_UNQUALIFIED"
        state.active_faults=list(wheel["faults"])
        self.vcu_pub.publish(state)

    def destroy_node(self):
        if self._timer: self._timer.cancel()
        if self.backend: self.backend.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node=VcuBridgeNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
