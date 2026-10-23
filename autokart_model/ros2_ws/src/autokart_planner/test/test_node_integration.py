"""ROS pub/sub verification of mission -> planner -> controller; synthetic inputs only."""
import json
import time
import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from autokart_msgs.msg import MissionDebug, ConeArray, ActuatorFeedback
from autokart_planner.node import PlannerNode, ros_time
from autokart_controller.node import ControllerNode


def params(values):
    return [Parameter(k, value=v) for k, v in values.items()]


@pytest.fixture
def chain():
    rclpy.init()
    planner = PlannerNode(parameter_overrides=params(dict(a_brake_min_mps2=2.,
        a_lat_max_mps2=1., t_total_latency_s=.2, acceleration_limit_mps2=.5,
        maximum_curvature_m_inv=.5, front_extent_m=1., rear_extent_m=.5,
        source='SYNTHETIC', cone_radius_m=.15, cone_position_uncertainty_m=.05)))
    controller = ControllerNode(parameter_overrides=params(dict(source='SYNTHETIC')))
    observer = rclpy.create_node('planner_controller_observer')
    received = []
    observer.create_subscription(MissionDebug, '/control/request', lambda m: received.append(json.loads(m.payload_json)), 10)
    executor = rclpy.executors.SingleThreadedExecutor()
    for node in (planner, controller, observer): executor.add_node(node)
    yield planner, controller, observer, executor, received
    executor.shutdown()
    for node in (planner, controller, observer): node.destroy_node()
    rclpy.shutdown()


def seed(planner, controller, *, feedback=True, coverage=True):
    stamp = planner.get_clock().now().nanoseconds
    odom = Odometry()
    odom.header.stamp = ros_time(stamp)
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.orientation.w = 1.
    planner.on_odom(odom)
    controller.on_odom(odom)
    status = String(data=json.dumps(dict(schema_version=1, sample_time_ns=stamp,
        expires_ns=stamp+200_000_000, source='SYNTHETIC', valid=True,
        continuity_lost=False, physical_qualified=False)))
    planner.on_odometry_status(status)
    controller.on_odometry_status(status)
    if feedback:
        actuator = ActuatorFeedback()
        actuator.measurement_time = ros_time(stamp)
        actuator.valid = True
        controller.on_feedback(actuator)
    if coverage:
        obstacles = ConeArray()
        obstacles.measurement_time = ros_time(stamp)
        obstacles.frame_id = 'odom'
        planner.on_obstacles(obstacles)
        status = MissionDebug()
        status.measurement_time = ros_time(stamp)
        status.payload_json = json.dumps(dict(schema_version=1, sample_time_ns=stamp,
            expires_ns=stamp+100_000_000, source='SYNTHETIC', frame_id='odom',
            coverage_complete=True, valid=True))
        planner.on_coverage(status)
    mapping = MissionDebug()
    mapping.measurement_time = ros_time(stamp)
    mapping.payload_json = json.dumps(dict(schema_version=1, sample_time_ns=stamp,
        expires_ns=stamp+150_000_000, source='SYNTHETIC', frame_id='odom', valid=True,
        geometry={'observation_cones_odom':[{'x_m':3.,'y_m':3.}]}))
    planner.on_mapping(mapping)
    mission = MissionDebug()
    mission.measurement_time = ros_time(stamp)
    mission.payload_json = json.dumps(dict(schema_version=1, sample_time_ns=stamp,
        expires_ns=stamp+100_000_000, source='SYNTHETIC', frame_id='odom', valid=True,
        path_odom=[[float(i),0.] for i in range(11)], target_speed_mps=1.))
    return mission


def spin_until(executor, predicate, timeout=2.):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        executor.spin_once(timeout_sec=.01)
        if predicate(): return True
    return False


def test_synthetic_mission_to_expiring_candidate_then_stop(chain):
    planner, controller, _, executor, received = chain
    # Discovery must complete before the short-lived source sample is issued.
    spin_until(executor, lambda: planner.status_pub.get_subscription_count() > 0)
    planner.on_mission(seed(planner, controller))
    assert spin_until(executor, lambda: any(p['valid'] for p in received))
    candidate = next(p for p in received if p['valid'])
    assert candidate['expires_ns'] > candidate['issued_ns']
    assert candidate['expires_ns'] <= candidate['issued_ns']+100_000_000
    assert not candidate['vehicle_authorized']
    assert candidate['output_inhibited']
    assert spin_until(executor, lambda: any(p['stop_requested'] for p in received))
    assert received[-1]['target_speed_mps'] == 0.


def test_missing_obstacle_coverage_never_produces_valid_plan(chain):
    planner, controller, _, executor, received = chain
    mission = seed(planner, controller, coverage=False)
    planner.on_mission(mission)
    assert planner.obstacles_received is None
    assert spin_until(executor, lambda: bool(received))
    assert all(not p['valid'] for p in received)


def test_missing_feedback_never_produces_valid_control(chain):
    planner, controller, _, executor, received = chain
    planner.on_mission(seed(planner, controller, feedback=False))
    assert spin_until(executor, lambda: bool(received))
    assert all(not p['valid'] for p in received)


def test_fresh_odometry_with_invalid_status_is_vetoed(chain):
    planner, controller, _, executor, received = chain
    mission = seed(planner, controller)
    planner.odom_status['valid'] = False
    captured = []
    planner.publish = captured.append
    planner.on_mission(mission)
    assert captured[-1]['reason_codes'] == ['ODOMETRY_STATUS_UNQUALIFIED']


def test_stale_status_vetoes_even_fresh_pose(chain):
    planner, controller, _, executor, received = chain
    mission = seed(planner, controller)
    controller.odom_status['expires_ns'] = 0
    captured = []
    controller.publish = captured.append
    controller._process([[0.,0.],[2.,0.]], planner.get_clock().now().nanoseconds)
    assert captured[-1]['reason_codes'] == ['ODOMETRY_STATUS_UNQUALIFIED']


@pytest.mark.parametrize('value', ['false', 1, [], None])
def test_malformed_validity_boolean_rejected_by_both_nodes(chain, value):
    planner, controller, _, _, _ = chain
    mission = seed(planner,controller)
    payload=json.loads(mission.payload_json)
    payload['valid']=value
    mission.payload_json=json.dumps(payload)
    planner.on_mission(mission)
    assert planner.mission is None
    payload.update(speed_profile_mps=[0.]*len(payload['path_odom']))
    mission.payload_json=json.dumps(payload)
    controller.on_planning_status(mission)
    assert controller.latest_path is None


@pytest.mark.parametrize('field,value', [('expires_ns','bad'),('coverage_complete','false'),('valid',1)])
def test_malformed_coverage_status_dropped(chain,field,value):
    planner, controller, _, _, _=chain
    seed(planner,controller)
    payload=dict(planner.coverage)
    payload[field]=value
    status=MissionDebug()
    status.measurement_time=ros_time(payload['sample_time_ns'])
    status.payload_json=json.dumps(payload)
    planner.on_coverage(status)
    assert planner.coverage is None


def test_source_age_not_extended_by_recent_receipt(chain):
    planner, controller, _, _, _=chain
    seed(planner,controller)
    for node in (planner,controller):
        stamp=node.odom_stamp_ns
        node.odom_status['expires_ns']=stamp+1_000_000_000
        assert node.odometry_qualified(stamp+149_000_000)
        assert not node.odometry_qualified(stamp+151_000_000)



def test_mapped_cone_on_path_requests_stop(chain):
    planner,controller,_,_,_=chain
    mission=seed(planner,controller)
    planner.mapping['geometry']['observation_cones_odom']=[{'x_m':.5,'y_m':0.}]
    captured=[]
    planner.publish=captured.append
    planner.on_mission(mission)
    assert captured[-1]['valid'] is False
    assert captured[-1]['reason_codes']==['CORRIDOR_BLOCKED']


def test_missing_mapping_companion_vetoes_plan(chain):
    planner,controller,_,_,_=chain
    mission=seed(planner,controller)
    planner.mapping=None
    captured=[]
    planner.publish=captured.append
    planner.on_mission(mission)
    assert captured[-1]['reason_codes']==['MAPPING_COMPANION_MISSING_INVALID_OR_STALE']


def test_simulated_and_synthetic_are_same_nonphysical_odometry_regime(chain):
    planner,controller,_,_,_=chain
    seed(planner,controller)
    for node in (planner,controller):
        node.odom_status['source']='SIMULATED'
        assert node.odometry_qualified(node.odom_stamp_ns)
        node.odom_status['source']='LIVE'
        assert not node.odometry_qualified(node.odom_stamp_ns)


@pytest.mark.parametrize('backend_parameter,node_index', [('algorithm_backend',0),('pursuit_backend',1)])
def test_live_builtin_backend_is_inhibited_before_processing(chain,backend_parameter,node_index):
    node=chain[node_index]
    node.source='LIVE_PHYSICAL'
    faults=[]
    node._publish_fault=faults.append
    node._process([[0.,0.],[2.,0.]],node.get_clock().now().nanoseconds)
    assert faults==['LIVE_REUSED_BACKEND_REQUIRED']



def test_synthetic_ground_truth_keeps_companion_gate(chain):
    planner,controller,_,_,_=chain
    planner.mission=json.loads(seed(planner,controller).payload_json)
    obstacles=ConeArray()
    obstacles.measurement_time=ros_time(planner.odom_stamp_ns)
    obstacles.frame_id='odom'
    obstacles.ground_truth=True
    planner.on_obstacles(obstacles)
    assert planner.obstacles_received==planner.odom_stamp_ns
    planner.coverage=None
    captured=[]
    planner.publish=captured.append
    planner._process([[0.,0.],[2.,0.]],planner.odom_stamp_ns)
    assert captured[-1]['reason_codes']==['OBSTACLE_COVERAGE_UNQUALIFIED']
    planner.source='LIVE_PHYSICAL'
    planner.on_obstacles(obstacles)
    assert planner.obstacles_received is None
