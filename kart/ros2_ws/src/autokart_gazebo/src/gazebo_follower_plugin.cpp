#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#include <autokart_msgs/msg/authoritative_vehicle_snapshot.hpp>
#include <autokart_msgs/msg/gazebo_follower_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace autokart_gazebo
{
using Snapshot = autokart_msgs::msg::AuthoritativeVehicleSnapshot;

builtin_interfaces::msg::Time Stamp(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  const double bounded = std::max(0.0, seconds);
  stamp.sec = static_cast<int32_t>(std::floor(bounded));
  stamp.nanosec = static_cast<uint32_t>(
    std::llround((bounded - static_cast<double>(stamp.sec)) * 1e9));
  if (stamp.nanosec >= 1000000000u) {
    ++stamp.sec;
    stamp.nanosec -= 1000000000u;
  }
  return stamp;
}

diagnostic_msgs::msg::KeyValue Key(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

geometry_msgs::msg::TransformStamped Transform(
  const std::string & parent, const std::string & child,
  double timestamp, const ignition::math::Vector3d & translation,
  const ignition::math::Quaterniond & rotation)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = Stamp(timestamp);
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.x = translation.X();
  transform.transform.translation.y = translation.Y();
  transform.transform.translation.z = translation.Z();
  transform.transform.rotation.x = rotation.X();
  transform.transform.rotation.y = rotation.Y();
  transform.transform.rotation.z = rotation.Z();
  transform.transform.rotation.w = rotation.W();
  return transform;
}

class GazeboFollowerPlugin final : public gazebo::ModelPlugin
{
public:
  GazeboFollowerPlugin() = default;

  ~GazeboFollowerPlugin() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    world_ = model_->GetWorld();
    if (!model_->IsStatic()) {
      gzerr << "AUTOKART DOUBLE PHYSICS: follower model must be static\n";
      world_->SetPaused(true);
      invalid_.store(true);
      return;
    }
    const std::string model_xml = model_->GetSDF()->ToString("");
    const std::vector<std::string> forbidden{
      "diff_drive", "ackermann", "gazebo_ros2_control", "vehicle_dynamics",
      "steering_controller", "brake_controller", "motor_controller"};
    for (const auto & token : forbidden) {
      if (model_xml.find(token) != std::string::npos) {
        gzerr << "AUTOKART DOUBLE PHYSICS: prohibited plugin token " << token << "\n";
        world_->SetPaused(true);
        invalid_.store(true);
        return;
      }
    }
    const std::string authority =
      sdf->HasElement("physics_authority") ?
      sdf->Get<std::string>("physics_authority") : "";
    const std::string role =
      sdf->HasElement("gazebo_role") ? sdf->Get<std::string>("gazebo_role") : "";
    if (authority != "internal" || role != "visual_sensor_follower") {
      gzerr << "AUTOKART DOUBLE PHYSICS: invalid authority/role\n";
      world_->SetPaused(true);
      invalid_.store(true);
      return;
    }
    maximum_state_age_s_ =
      sdf->HasElement("maximum_state_age_s") ?
      sdf->Get<double>("maximum_state_age_s") : 0.020;
    maximum_position_error_m_ =
      sdf->HasElement("maximum_position_error_m") ?
      sdf->Get<double>("maximum_position_error_m") : 0.005;
    maximum_orientation_error_rad_ =
      sdf->HasElement("maximum_orientation_error_rad") ?
      sdf->Get<double>("maximum_orientation_error_rad") : 0.005;
    maximum_joint_error_rad_ =
      sdf->HasElement("maximum_joint_error_rad") ?
      sdf->Get<double>("maximum_joint_error_rad") : 0.005;
    maximum_wheel_error_rad_ =
      sdf->HasElement("maximum_wheel_error_rad") ?
      sdf->Get<double>("maximum_wheel_error_rad") : 0.020;
    maximum_clock_offset_s_ =
      sdf->HasElement("maximum_clock_offset_s") ?
      sdf->Get<double>("maximum_clock_offset_s") : 0.002;
    maximum_wall_snapshot_gap_s_ =
      sdf->HasElement("maximum_wall_snapshot_gap_s") ?
      sdf->Get<double>("maximum_wall_snapshot_gap_s") : 0.5;
    ros_output_rate_hz_ =
      sdf->HasElement("ros_output_rate_hz") ?
      sdf->Get<double>("ros_output_rate_hz") : 100.0;
    if (ros_output_rate_hz_ <= 0.0) {
      gzerr << "AutoKart ROS output rate must be positive\n";
      world_->SetPaused(true);
      invalid_.store(true);
      return;
    }
    ros_output_period_s_ = 1.0 / ros_output_rate_hz_;

    const std::vector<std::string> required_joints{
      "front_left_steering_joint", "front_right_steering_joint",
      "front_left_wheel_joint", "front_right_wheel_joint",
      "rear_solid_axle_joint"};
    for (const auto & name : required_joints) {
      auto joint = model_->GetJoint(name);
      if (!joint) {
        gzerr << "AUTOKART follower missing joint " << name << "\n";
        invalid_.store(true);
        world_->SetPaused(true);
        return;
      }
      joints_.push_back(joint);
    }

    node_ = gazebo_ros::Node::Get(sdf);
    const auto snapshot_qos = rclcpp::QoS(rclcpp::KeepLast(64)).reliable();
    snapshot_subscription_ = node_->create_subscription<Snapshot>(
      "/autokart/follower/authoritative_snapshot", snapshot_qos,
      std::bind(&GazeboFollowerPlugin::OnSnapshot, this, std::placeholders::_1));
    follower_publisher_ = node_->create_publisher<autokart_msgs::msg::GazeboFollowerState>(
      "/autokart/follower/state", rclcpp::QoS(64).reliable());
    clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(1).reliable());
    joint_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS());
    odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/vehicle/odometry", rclcpp::SensorDataQoS());
    diagnostics_publisher_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    static_transform_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
    PublishStaticTransforms();

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboFollowerPlugin::OnUpdate, this, std::placeholders::_1));
    watchdog_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&GazeboFollowerPlugin::OnWallWatchdog, this));
    world_->SetPaused(true);
    gzmsg << "AutoKart follower loaded: internal authority, static visual/sensor mirror\n";
  }

private:
  static double InterpolateAngle(double first, double second, double fraction)
  {
    const double delta = std::atan2(
      std::sin(second - first), std::cos(second - first));
    return first + fraction * delta;
  }

  static Snapshot Interpolate(
    const Snapshot & first, const Snapshot & second, double timestamp)
  {
    const double duration = second.simulation_time_s - first.simulation_time_s;
    const double fraction = std::clamp(
      (timestamp - first.simulation_time_s) / duration, 0.0, 1.0);
    const auto scalar = [fraction](double a, double b) {
        return a + fraction * (b - a);
      };
    Snapshot result = second;
    result.simulation_time_s = timestamp;
    result.position_x_m = scalar(first.position_x_m, second.position_x_m);
    result.position_y_m = scalar(first.position_y_m, second.position_y_m);
    result.position_z_m = scalar(first.position_z_m, second.position_z_m);
    result.roll_rad = InterpolateAngle(first.roll_rad, second.roll_rad, fraction);
    result.pitch_rad = InterpolateAngle(first.pitch_rad, second.pitch_rad, fraction);
    result.yaw_rad = InterpolateAngle(first.yaw_rad, second.yaw_rad, fraction);
    for (std::size_t index = 0; index < 3u; ++index) {
      result.linear_velocity_body_mps[index] = scalar(
        first.linear_velocity_body_mps[index], second.linear_velocity_body_mps[index]);
      result.angular_velocity_body_radps[index] = scalar(
        first.angular_velocity_body_radps[index], second.angular_velocity_body_radps[index]);
    }
    result.front_left_steering_rad = scalar(
      first.front_left_steering_rad, second.front_left_steering_rad);
    result.front_right_steering_rad = scalar(
      first.front_right_steering_rad, second.front_right_steering_rad);
    result.front_left_wheel_rotation_rad = scalar(
      first.front_left_wheel_rotation_rad, second.front_left_wheel_rotation_rad);
    result.front_right_wheel_rotation_rad = scalar(
      first.front_right_wheel_rotation_rad, second.front_right_wheel_rotation_rad);
    result.rear_left_wheel_rotation_rad = scalar(
      first.rear_left_wheel_rotation_rad, second.rear_left_wheel_rotation_rad);
    result.rear_right_wheel_rotation_rad = scalar(
      first.rear_right_wheel_rotation_rad, second.rear_right_wheel_rotation_rad);
    result.front_left_wheel_speed_radps = scalar(
      first.front_left_wheel_speed_radps, second.front_left_wheel_speed_radps);
    result.front_right_wheel_speed_radps = scalar(
      first.front_right_wheel_speed_radps, second.front_right_wheel_speed_radps);
    result.rear_left_wheel_speed_radps = scalar(
      first.rear_left_wheel_speed_radps, second.rear_left_wheel_speed_radps);
    result.rear_right_wheel_speed_radps = scalar(
      first.rear_right_wheel_speed_radps, second.rear_right_wheel_speed_radps);
    result.rear_axle_rotation_rad = scalar(
      first.rear_axle_rotation_rad, second.rear_axle_rotation_rad);
    result.rear_axle_speed_radps = scalar(
      first.rear_axle_speed_radps, second.rear_axle_speed_radps);
    return result;
  }

  void OnSnapshot(const Snapshot::SharedPtr message)
  {
    if (invalid_.load()) {
      return;
    }
    if (!message->valid || message->configuration_hash.size() != 64u ||
      !std::isfinite(message->simulation_time_s))
    {
      Critical("invalid authoritative snapshot", false);
      return;
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (have_last_received_) {
      if (message->reset_epoch < last_received_.reset_epoch ||
        (message->reset_epoch == last_received_.reset_epoch &&
        (message->sequence <= last_received_.sequence ||
        message->simulation_time_s <= last_received_.simulation_time_s)))
      {
        Critical("non-monotonic snapshot within reset epoch", false);
        return;
      }
      if (message->reset_epoch == last_received_.reset_epoch &&
        message->configuration_hash != last_received_.configuration_hash)
      {
        Critical("configuration hash changed within reset epoch", false);
        return;
      }
      if (message->reset_epoch > last_received_.reset_epoch) {
        snapshot_queue_.clear();
      }
    }
    if (snapshot_queue_.size() >= 64u) {
      Critical("authoritative snapshot bridge queue overflow", false);
      return;
    }
    snapshot_queue_.push_back(*message);
    last_received_ = *message;
    have_last_received_ = true;
    received_snapshot_.store(true);
    last_snapshot_wall_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    watchdog_reported_.store(false);
    world_->SetPaused(false);
  }

  void OnWallWatchdog()
  {
    if (invalid_.load() || !received_snapshot_.load() || watchdog_reported_.load()) {
      return;
    }
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const double gap_s =
      static_cast<double>(now_ns - last_snapshot_wall_ns_.load()) * 1e-9;
    if (gap_s <= maximum_wall_snapshot_gap_s_) {
      return;
    }
    const bool disconnected = snapshot_subscription_->get_publisher_count() == 0u;
    autokart_msgs::msg::GazeboFollowerState state;
    {
      std::lock_guard<std::mutex> lock(output_mutex_);
      if (!have_last_follower_output_) {
        return;
      }
      state = last_follower_output_;
    }
    state.follower_valid = !disconnected;
    state.follower_frozen = true;
    state.state_age_s = gap_s;
    follower_publisher_->publish(state);
    PublishDiagnostics(
      disconnected ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      disconnected ? "snapshot bridge disconnected; follower frozen and invalid" :
      "internal simulation paused; follower frozen at authoritative state",
      0.0, 0.0, 0.0, 0.0);
    watchdog_reported_.store(true);
  }

  void OnUpdate(const gazebo::common::UpdateInfo &)
  {
    if (invalid_.load()) {
      world_->SetPaused(true);
      return;
    }
    if (!have_applied_) {
      Snapshot first;
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        if (snapshot_queue_.empty()) {
          world_->SetPaused(true);
          return;
        }
        first = snapshot_queue_.front();
        snapshot_queue_.pop_front();
      }
      ApplyAuthoritative(first, true);
      world_->SetPaused(true);
      return;
    }

    if (!have_target_) {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      if (!snapshot_queue_.empty()) {
        target_ = snapshot_queue_.front();
        snapshot_queue_.pop_front();
        have_target_ = true;
      }
    }
    if (!have_target_) {
      world_->SetPaused(true);
      return;
    }
    if (target_.reset_epoch != applied_.reset_epoch) {
      ApplyAuthoritative(target_, true);
      have_target_ = false;
      world_->SetPaused(true);
      return;
    }
    const double interval = target_.simulation_time_s - applied_.simulation_time_s;
    if (interval <= 0.0 || interval > maximum_state_age_s_ + 1e-12) {
      Critical(
        "snapshot interval exceeds bounded interpolation policy", false, interval);
      return;
    }

    const double gazebo_time = world_->SimTime().Double();
    if (gazebo_time >= target_.simulation_time_s - 1e-12) {
      ApplyAuthoritative(target_, false);
      have_target_ = false;
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      if (snapshot_queue_.empty()) {
        world_->SetPaused(true);
      }
      return;
    }
    const auto visual = Interpolate(applied_, target_, gazebo_time);
    ApplyVisual(visual);
  }

  void ApplyAuthoritative(const Snapshot & snapshot, bool set_world_time)
  {
    const bool reset = !have_applied_ || snapshot.reset_epoch != applied_.reset_epoch;
    if (!reset && snapshot.simulation_time_s <= applied_.simulation_time_s) {
      Critical("Gazebo follower rewind prohibited within an epoch", false);
      return;
    }
    if (set_world_time) {
      world_->SetSimTime(gazebo::common::Time(snapshot.simulation_time_s));
    }
    ignition::math::Pose3d target_pose;
    std::vector<double> positions;
    if (!ApplyVisual(snapshot, &target_pose, &positions)) {
      return;
    }
    applied_ = snapshot;
    have_applied_ = true;
    ++applied_update_count_;
    PublishOutputs(snapshot, target_pose, positions);
  }

  bool ApplyVisual(
    const Snapshot & snapshot, ignition::math::Pose3d * output_pose = nullptr,
    std::vector<double> * output_positions = nullptr)
  {
    const ignition::math::Quaterniond orientation(
      snapshot.roll_rad, snapshot.pitch_rad, snapshot.yaw_rad);
    const ignition::math::Pose3d target_pose(
      snapshot.position_x_m, snapshot.position_y_m, snapshot.position_z_m,
      orientation.Roll(), orientation.Pitch(), orientation.Yaw());
    model_->SetWorldPose(target_pose);

    const double cy = std::cos(snapshot.yaw_rad);
    const double sy = std::sin(snapshot.yaw_rad);
    const double u = snapshot.linear_velocity_body_mps[0];
    const double v = snapshot.linear_velocity_body_mps[1];
    model_->SetLinearVel(ignition::math::Vector3d(cy * u - sy * v, sy * u + cy * v,
      snapshot.linear_velocity_body_mps[2]));
    model_->SetAngularVel(ignition::math::Vector3d(
      snapshot.angular_velocity_body_radps[0],
      snapshot.angular_velocity_body_radps[1],
      snapshot.angular_velocity_body_radps[2]));

    const std::vector<double> positions{
      snapshot.front_left_steering_rad,
      snapshot.front_right_steering_rad,
      -snapshot.front_left_wheel_rotation_rad,
      -snapshot.front_right_wheel_rotation_rad,
      -snapshot.rear_axle_rotation_rad};
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      if (!joints_[index]->SetPosition(0, positions[index], false)) {
        Critical("Gazebo joint follower SetPosition failed", false);
        return false;
      }
    }
    if (output_pose) {
      *output_pose = target_pose;
    }
    if (output_positions) {
      *output_positions = positions;
    }
    return true;
  }

  void PublishOutputs(
    const Snapshot & snapshot, const ignition::math::Pose3d & target_pose,
    const std::vector<double> & positions)
  {
    const double timestamp = snapshot.simulation_time_s;
    const double gazebo_timestamp = world_->SimTime().Double();
    const double clock_offset = gazebo_timestamp - timestamp;
    const auto observed_pose = model_->WorldPose();
    const double position_error = observed_pose.Pos().Distance(target_pose.Pos());
    const double quaternion_dot = std::abs(
      observed_pose.Rot().W() * target_pose.Rot().W() +
      observed_pose.Rot().X() * target_pose.Rot().X() +
      observed_pose.Rot().Y() * target_pose.Rot().Y() +
      observed_pose.Rot().Z() * target_pose.Rot().Z());
    const double orientation_error =
      2.0 * std::acos(std::clamp(quaternion_dot, 0.0, 1.0));
    double steering_error = 0.0;
    double wheel_error = 0.0;
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      const double error = std::abs(joints_[index]->Position(0) - positions[index]);
      if (index < 2u) {
        steering_error = std::max(steering_error, error);
      } else {
        wheel_error = std::max(wheel_error, error);
      }
    }
    if (position_error > maximum_position_error_m_ ||
      orientation_error > maximum_orientation_error_rad_ ||
      steering_error > maximum_joint_error_rad_ ||
      wheel_error > maximum_wheel_error_rad_ ||
      std::abs(clock_offset) > maximum_clock_offset_s_)
    {
      Critical("follower mismatch exceeds simulation acceptance threshold", false);
      return;
    }

    rosgraph_msgs::msg::Clock clock;
    clock.clock = Stamp(timestamp);
    clock_publisher_->publish(clock);
    last_output_sim_time_s_.store(timestamp);

    if (!have_ros_output_schedule_) {
      next_ros_output_s_ = timestamp;
      have_ros_output_schedule_ = true;
    }
    if (timestamp + 1e-12 < next_ros_output_s_) {
      return;
    }
    do {
      next_ros_output_s_ += ros_output_period_s_;
    } while (next_ros_output_s_ <= timestamp + 1e-12);

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = Stamp(timestamp);
    joints.header.frame_id = "base_link";
    joints.name = {
      "front_left_steering_joint", "front_right_steering_joint",
      "front_left_wheel_joint", "front_right_wheel_joint",
      "rear_solid_axle_joint"};
    joints.position = positions;
    joints.velocity = {
      0.0, 0.0,
      -snapshot.front_left_wheel_speed_radps,
      -snapshot.front_right_wheel_speed_radps,
      -snapshot.rear_axle_speed_radps};
    joint_publisher_->publish(joints);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = Stamp(timestamp);
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = observed_pose.Pos().X();
    odometry.pose.pose.position.y = observed_pose.Pos().Y();
    odometry.pose.pose.position.z = observed_pose.Pos().Z();
    odometry.pose.pose.orientation.x = observed_pose.Rot().X();
    odometry.pose.pose.orientation.y = observed_pose.Rot().Y();
    odometry.pose.pose.orientation.z = observed_pose.Rot().Z();
    odometry.pose.pose.orientation.w = observed_pose.Rot().W();
    odometry.twist.twist.linear.x = snapshot.linear_velocity_body_mps[0];
    odometry.twist.twist.linear.y = snapshot.linear_velocity_body_mps[1];
    odometry.twist.twist.linear.z = snapshot.linear_velocity_body_mps[2];
    odometry.twist.twist.angular.x = snapshot.angular_velocity_body_radps[0];
    odometry.twist.twist.angular.y = snapshot.angular_velocity_body_radps[1];
    odometry.twist.twist.angular.z = snapshot.angular_velocity_body_radps[2];
    odometry_publisher_->publish(odometry);

    auto follower = autokart_msgs::msg::GazeboFollowerState();
    follower.header = odometry.header;
    follower.simulation_time_s = timestamp;
    follower.gazebo_simulation_time_s = gazebo_timestamp;
    follower.clock_offset_s = clock_offset;
    follower.sequence = snapshot.sequence;
    follower.reset_epoch = snapshot.reset_epoch;
    follower.applied_updates = applied_update_count_;
    follower.pose = odometry.pose.pose;
    follower.twist = odometry.twist.twist;
    follower.joint_names = joints.name;
    follower.joint_positions_rad = joints.position;
    follower.joint_velocities_radps = joints.velocity;
    follower.follower_valid = true;
    follower.follower_frozen = false;
    follower.state_age_s = 0.0;
    follower.pose_source = "authoritative_internal";
    follower.joint_source = "authoritative_internal";
    follower_publisher_->publish(follower);
    {
      std::lock_guard<std::mutex> lock(output_mutex_);
      last_follower_output_ = follower;
      have_last_follower_output_ = true;
    }

    PublishDynamicTransforms(snapshot, observed_pose, positions);
    PublishDiagnostics(
      diagnostic_msgs::msg::DiagnosticStatus::OK, "follower synchronized",
      position_error, orientation_error, steering_error, wheel_error);
  }

  void PublishStaticTransforms()
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(Transform(
      "map", "odom", 0.0, ignition::math::Vector3d::Zero,
      ignition::math::Quaterniond::Identity));
    transforms.push_back(Transform(
      "base_link", "chassis_link", 0.0, ignition::math::Vector3d::Zero,
      ignition::math::Quaterniond::Identity));
    transforms.push_back(Transform(
      "base_link", "camera_link", 0.0, {0.64, 0.0, 0.72},
      ignition::math::Quaterniond::Identity));
    transforms.push_back(Transform(
      "camera_link", "camera_optical_frame", 0.0, ignition::math::Vector3d::Zero,
      ignition::math::Quaterniond(-M_PI_2, 0.0, -M_PI_2)));
    transforms.push_back(Transform(
      "base_link", "depth_camera_link", 0.0, {0.60, 0.0, 0.64},
      ignition::math::Quaterniond::Identity));
    transforms.push_back(Transform(
      "depth_camera_link", "depth_camera_optical_frame", 0.0,
      ignition::math::Vector3d::Zero,
      ignition::math::Quaterniond(-M_PI_2, 0.0, -M_PI_2)));
    transforms.push_back(Transform(
      "base_link", "lidar_link", 0.0, {0.20, 0.0, 0.82},
      ignition::math::Quaterniond::Identity));
    transforms.push_back(Transform(
      "base_link", "imu_link", 0.0, {0.05, 0.0, 0.30},
      ignition::math::Quaterniond::Identity));
    transforms.push_back(Transform(
      "rear_axle_link", "rear_left_wheel_link", 0.0, {0.0, 0.46, 0.0},
      ignition::math::Quaterniond::Identity));
    transforms.push_back(Transform(
      "rear_axle_link", "rear_right_wheel_link", 0.0, {0.0, -0.46, 0.0},
      ignition::math::Quaterniond::Identity));
    static_transform_broadcaster_->sendTransform(transforms);
  }

  void PublishDynamicTransforms(
    const Snapshot & snapshot, const ignition::math::Pose3d & pose,
    const std::vector<double> & positions)
  {
    const double stamp = snapshot.simulation_time_s;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(Transform("odom", "base_link", stamp, pose.Pos(), pose.Rot()));
    transforms.push_back(Transform(
      "base_link", "front_left_steer_link", stamp, {0.575, 0.475, 0.14},
      ignition::math::Quaterniond(0.0, 0.0, positions[0])));
    transforms.push_back(Transform(
      "base_link", "front_right_steer_link", stamp, {0.575, -0.475, 0.14},
      ignition::math::Quaterniond(0.0, 0.0, positions[1])));
    transforms.push_back(Transform(
      "front_left_steer_link", "front_left_wheel_link", stamp,
      ignition::math::Vector3d::Zero,
      ignition::math::Quaterniond(M_PI_2, positions[2], 0.0)));
    transforms.push_back(Transform(
      "front_right_steer_link", "front_right_wheel_link", stamp,
      ignition::math::Vector3d::Zero,
      ignition::math::Quaterniond(M_PI_2, positions[3], 0.0)));
    transforms.push_back(Transform(
      "base_link", "rear_axle_link", stamp, {-0.575, 0.0, 0.14},
      ignition::math::Quaterniond(M_PI_2, positions[4], 0.0)));
    transform_broadcaster_->sendTransform(transforms);
  }

  void PublishDiagnostics(
    uint8_t level, const std::string & message, double position_error,
    double orientation_error, double steering_error, double wheel_error)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = Stamp(last_output_sim_time_s_.load());
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = "autokart/gazebo_follower";
    status.hardware_id = "SIMULATION";
    status.message = message;
    status.values = {
      Key("physics_authority", "internal"),
      Key("gazebo_role", "visual_sensor_follower"),
      Key(
        "double_physics_detected",
        double_physics_detected_.load() ? "true" : "false"),
      Key("pose_source", "authoritative_internal"),
      Key("joint_source", "authoritative_internal"),
      Key("applied_updates", std::to_string(applied_update_count_)),
      Key(
        "follower_frozen",
        (invalid_.load() || watchdog_reported_.load()) ? "true" : "false"),
      Key("gazebo_actuation_authority", "disabled"),
      Key("gazebo_contacts", "visual_only"),
      Key("position_error_m", std::to_string(position_error)),
      Key("orientation_error_rad", std::to_string(orientation_error)),
      Key("steering_error_rad", std::to_string(steering_error)),
      Key("wheel_error_rad", std::to_string(wheel_error))};
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  void Critical(
    const std::string & reason, bool double_physics, double state_age_s = 0.0)
  {
    invalid_.store(true);
    if (double_physics) {
      double_physics_detected_.store(true);
    }
    if (world_) {
      world_->SetPaused(true);
    }
    gzerr << "AUTOKART FOLLOWER CRITICAL: " << reason << "\n";
    if (follower_publisher_) {
      autokart_msgs::msg::GazeboFollowerState state;
      bool have_state = false;
      {
        std::lock_guard<std::mutex> lock(output_mutex_);
        if (have_last_follower_output_) {
          state = last_follower_output_;
          state.follower_valid = false;
          state.follower_frozen = true;
          state.state_age_s = std::max(state.state_age_s, state_age_s);
          last_follower_output_ = state;
          have_state = true;
        }
      }
      if (have_state) {
        follower_publisher_->publish(state);
      }
    }
    if (diagnostics_publisher_) {
      PublishDiagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        double_physics ? "double physics detected: " + reason : reason,
        0.0, 0.0, 0.0, 0.0);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  std::vector<gazebo::physics::JointPtr> joints_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<Snapshot>::SharedPtr snapshot_subscription_;
  rclcpp::Publisher<autokart_msgs::msg::GazeboFollowerState>::SharedPtr follower_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::mutex snapshot_mutex_;
  std::mutex output_mutex_;
  std::deque<Snapshot> snapshot_queue_;
  Snapshot target_;
  Snapshot applied_;
  Snapshot last_received_;
  autokart_msgs::msg::GazeboFollowerState last_follower_output_;
  bool have_target_{false};
  bool have_applied_{false};
  bool have_last_received_{false};
  bool have_last_follower_output_{false};
  std::atomic<bool> invalid_{false};
  std::atomic<bool> double_physics_detected_{false};
  std::atomic<bool> received_snapshot_{false};
  std::atomic<bool> watchdog_reported_{false};
  std::atomic<int64_t> last_snapshot_wall_ns_{0};
  std::atomic<double> last_output_sim_time_s_{0.0};
  double maximum_state_age_s_{0.020};
  double maximum_position_error_m_{0.005};
  double maximum_orientation_error_rad_{0.005};
  double maximum_joint_error_rad_{0.005};
  double maximum_wheel_error_rad_{0.020};
  double maximum_clock_offset_s_{0.002};
  double maximum_wall_snapshot_gap_s_{0.5};
  double ros_output_rate_hz_{100.0};
  double ros_output_period_s_{0.010};
  double next_ros_output_s_{0.0};
  bool have_ros_output_schedule_{false};
  uint64_t applied_update_count_{0};
};

GZ_REGISTER_MODEL_PLUGIN(GazeboFollowerPlugin)
}  // namespace autokart_gazebo
