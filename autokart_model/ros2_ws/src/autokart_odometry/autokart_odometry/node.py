"""Sole /odom and odom->base_link writer; no pose on unqualified sensors."""
import json
import math
import time
import fcntl
import os
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from autokart_msgs.msg import SensorDeliveryMetadata
from tf2_ros import TransformBroadcaster
from .core import Config, Estimator

class OdometryNode(Node):
    def __init__(self, **kwargs):
        super().__init__("autokart_odometry",**kwargs)
        self._owner=open(f"/tmp/autokart_odometry_{os.getuid()}.lock","w")
        try: fcntl.flock(self._owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            self._owner.close()
            raise RuntimeError("ODOMETRY_OWNER_ALREADY_RUNNING")
        for key,value in Config().__dict__.items(): self.declare_parameter(key,value)
        self.declare_parameter("imu_frame_id","imu_link")
        self.declare_parameter("imu_axes_aligned",False)
        self.estimator=Estimator(Config(**{k:self.get_parameter(k).value for k in Config().__dict__}))
        self.odom_pub=self.create_publisher(Odometry,"/odom",10)
        self.status_pub=self.create_publisher(String,"/vehicle/odometry_status",10)
        self.tf=TransformBroadcaster(self)
        self.imu_status=None
        self._imu_samples={}
        self._imu_states={}
        self.queue=deque(maxlen=32)
        self.last_valid_ns=0
        self.last_sample_ns=0
        self.create_subscription(Imu,"/imu/data_raw",self._imu,qos_profile_sensor_data)
        self.create_subscription(SensorDeliveryMetadata,"/imu/status",self._status,10)
        self.create_subscription(String,"/vehicle/wheel_encoder",self._wheel,10)
        self.timer=self.create_timer(0.01,self._tick)

    @staticmethod
    def _ns(stamp): return stamp.sec*10**9+stamp.nanosec

    def _status(self,msg):
        self.imu_status=(msg,time.monotonic())
        if msg.quality!="OK" or not msg.payload_present:
            self.estimator.add_imu(0,0.0,"UNVERIFIED",False)
            self._imu_samples.clear(); self._imu_states.clear()
            return
        stamp=self._ns(msg.capture_time)
        if stamp<=0:
            self.estimator.add_imu(0,0.0,"UNVERIFIED",False)
            return
        self._imu_states[stamp]=msg
        self._pair_imu(stamp)

    def _imu(self,msg):
        stamp=self._ns(msg.header.stamp)
        self._imu_samples[stamp]=msg
        self._pair_imu(stamp)

    def _pair_imu(self,stamp):
        # Per-sample acquisition metadata, rather than last received status,
        # must qualify this exact rate sample. Both ROS arrival orders work.
        if stamp in self._imu_samples and stamp in self._imu_states:
            msg=self._imu_samples.pop(stamp); status=self._imu_states.pop(stamp)
            now=self.get_clock().now().nanoseconds
            valid=bool(status.quality=="OK" and status.payload_present and
                       self.get_parameter("imu_axes_aligned").value and
                       msg.header.frame_id==self.get_parameter("imu_frame_id").value and
                       0<=now-stamp<=100_000_000)
            source=status.encoding
            if source=="LIVE_PHYSICAL": source="LIVE"
            self.estimator.add_imu(stamp,msg.angular_velocity.z,source,valid)
        for pending in (self._imu_samples,self._imu_states):
            if len(pending)>128:
                pending.clear()
                self.estimator.add_imu(0,0.0,"UNVERIFIED",False)

    def _wheel(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict): raise ValueError()
            if len(self.queue)==self.queue.maxlen:
                self.estimator.continuity_lost=True
                self.estimator.reason="WHEEL_QUEUE_OVERFLOW"
            self.queue.append((value,time.monotonic()))
        except (ValueError,TypeError):
            self.estimator.continuity_lost=True
            self.estimator.reason="WHEEL_MALFORMED"

    def _tick(self):
        now=self.get_clock().now().nanoseconds
        if self.queue:
            wheel,arrival=self.queue[0]
            end=wheel.get("sample_time_ns",0)
            if type(end) is not int: end=0
            if self.estimator.imu and self.estimator.imu[-1][0]>=end or time.monotonic()-arrival>0.1:
                self.queue.popleft()
                status=self.imu_status
                imu_ok=bool(status and time.monotonic()-status[1]<0.1 and status[0].quality=="OK")
                if not imu_ok:
                    self.estimator.reason="IMU_STATUS_INVALID_OR_STALE"
                    self.estimator.continuity_lost |= self.estimator.previous is not None
                    value=None
                else:
                    value=self.estimator.update(wheel,now)
                if value:
                    self.last_valid_ns=now
                    self.last_sample_ns=value["stamp_ns"]
                    self._publish(value)
        stale=now-self.last_valid_ns>200_000_000
        self.status_pub.publish(String(data=json.dumps(dict(schema_version=1,source=self.estimator.source or "UNVERIFIED",valid=self.estimator.reason=="OK" and not stale, reason="STALE" if stale and self.estimator.reason=="OK" else self.estimator.reason,sample_time_ns=self.last_sample_ns, expires_ns=self.last_sample_ns+200_000_000, physical_qualified=False, rpm=self.estimator.rpm, speed_mps=self.estimator.speed, signed_distance_m=self.estimator.signed_distance, distance_travelled_total=self.estimator.distance_total, x_m=self.estimator.x, y_m=self.estimator.y, yaw_rad=self.estimator.yaw, continuity_lost=self.estimator.continuity_lost))))

    def _publish(self,v):
        msg=Odometry()
        msg.header.stamp=Time(nanoseconds=v["stamp_ns"]).to_msg()
        msg.header.frame_id="odom"; msg.child_frame_id="base_link"
        msg.pose.pose.position.x=v["x"]; msg.pose.pose.position.y=v["y"]
        msg.pose.pose.orientation.z=math.sin(v["yaw"]/2); msg.pose.pose.orientation.w=math.cos(v["yaw"]/2)
        msg.twist.twist.linear.x=v["speed"]; msg.twist.twist.angular.z=v["yaw_rate"]
        for idx in (0,7): msg.pose.covariance[idx]=v["position_variance"]
        msg.pose.covariance[35]=v["yaw_variance"]
        for idx in (14,21,28): msg.pose.covariance[idx]=1e6
        msg.twist.covariance[0]=self.estimator.cfg.distance_variance_per_m
        msg.twist.covariance[35]=self.estimator.cfg.yaw_variance_per_s
        for idx in (7,14,21,28): msg.twist.covariance[idx]=1e6
        self.odom_pub.publish(msg)
        tf=TransformStamped(); tf.header=msg.header; tf.child_frame_id=msg.child_frame_id
        tf.transform.translation.x=v["x"]; tf.transform.translation.y=v["y"]
        tf.transform.rotation=msg.pose.pose.orientation
        self.tf.sendTransform(tf)

    def destroy_node(self):
        self.timer.cancel()
        self._owner.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node=OdometryNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
