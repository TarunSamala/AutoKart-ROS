import json
import time
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from autokart_msgs.msg import SensorDeliveryMetadata
from autokart_odometry.node import OdometryNode


def test_default_unverified_node_never_publishes_odom_and_has_single_owner():
    rclpy.init()
    node=OdometryNode()
    try:
        with pytest.raises(RuntimeError,match='OWNER_ALREADY_RUNNING'):
            OdometryNode()
        assert not node.estimator.cfg.calibration_verified
        node._wheel(String(data='{}'))
        for _ in range(12): node._tick(); time.sleep(.01)
        assert node.last_valid_ns==0
    finally: node.destroy_node(); rclpy.shutdown()


def test_ros_wrapper_publishes_aligned_synthetic_pose_and_stops_on_invalid_imu():
    rclpy.init()
    values=dict(counts_per_rev=100.0,circumference_m=1.0,wheelbase_m=1.0,calibration_verified=True,calibration_hash='fixture',imu_axes_aligned=True)
    node=OdometryNode(parameter_overrides=[Parameter(k,value=v) for k,v in values.items()])
    sub=rclpy.create_node('test_odometry_receiver')
    got=[]
    sub.create_subscription(Odometry,'/odom',got.append,10)
    try:
        deadline=time.monotonic()+1
        while node.odom_pub.get_subscription_count()<1 and time.monotonic()<deadline:
            rclpy.spin_once(sub,timeout_sec=.02)
        status=SensorDeliveryMetadata(); status.quality='OK'; status.encoding='SYNTHETIC'; status.payload_present=True
        end=node.get_clock().now().nanoseconds-20_000_000
        begin=end-50_000_000
        for t in range(begin,end+20_000_000,10_000_000):
            msg=Imu(); msg.header.stamp=Time(nanoseconds=t).to_msg(); msg.header.frame_id='imu_link'
            status.capture_time=msg.header.stamp
            node._status(status)
            node._imu(msg)
        def wheel(tick,stamp,count,seq):
            return String(data=json.dumps(dict(schema_version=1,source='SYNTHETIC',valid=True,clock_valid=True,calibration_verified=True,calibration_hash='fixture',steering_valid=True,steering_actual_rad=0.0,boot_id=1,sequence=seq,sample_time_us=tick,sample_time_ns=stamp,axle_count_total=count)))
        node._wheel(wheel(0,begin,0,0)); node._tick()
        node._wheel(wheel(50000,end,5,1)); node._tick()
        deadline=time.monotonic()+1
        while not got and time.monotonic()<deadline: rclpy.spin_once(sub,timeout_sec=.02)
        assert got and got[-1].pose.pose.position.x==pytest.approx(.05)
        assert got[-1].header.frame_id=='odom'
        assert got[-1].child_frame_id=='base_link'
        assert got[-1].twist.twist.linear.x==pytest.approx(1)
        status.quality='INVALID'; node._status(status)
        node._wheel(wheel(60000,end+10_000_000,6,2)); node._tick()
        assert node.estimator.continuity_lost
        assert node.estimator.reason in ('IMU_INVALID','IMU_STATUS_INVALID_OR_STALE')
    finally: node.destroy_node(); sub.destroy_node(); rclpy.shutdown()


def test_imu_requires_exact_capture_metadata_in_either_arrival_order():
    rclpy.init()
    node=OdometryNode(parameter_overrides=[Parameter('imu_axes_aligned',value=True)])
    try:
        now=node.get_clock().now().nanoseconds-10_000_000
        msg=Imu(); msg.header.frame_id='imu_link'; msg.header.stamp=Time(nanoseconds=now).to_msg()
        status=SensorDeliveryMetadata(); status.quality='OK'; status.payload_present=True; status.encoding='SYNTHETIC'
        status.capture_time=Time(nanoseconds=now-1).to_msg()
        node._status(status); node._imu(msg)
        assert not node.estimator.imu
        status.capture_time=msg.header.stamp
        node._status(status)
        assert len(node.estimator.imu)==1
        second=Imu(); second.header.frame_id='imu_link'; second.header.stamp=Time(nanoseconds=now+1).to_msg()
        node._imu(second)
        assert len(node.estimator.imu)==1
        status.capture_time=second.header.stamp
        node._status(status)
        assert len(node.estimator.imu)==2
    finally: node.destroy_node(); rclpy.shutdown()
