#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo_ros/node.hpp>

#include <autokart_msgs/msg/authoritative_vehicle_snapshot.hpp>
#include <autokart_msgs/msg/sensor_delivery_metadata.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace autokart_gazebo
{
using Snapshot = autokart_msgs::msg::AuthoritativeVehicleSnapshot;

builtin_interfaces::msg::Time SensorStamp(double seconds)
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

struct QueuedDelivery
{
  uint64_t sequence;
  uint32_t reset_epoch;
  double capture_time_s;
  double processing_completion_time_s;
  double due_time_s;
  std::size_t payload_size;
  std::string encoding;
  uint32_t width;
  uint32_t height;
  std::function<void()> publish_payload;
};

class GazeboSensorDeliveryPlugin final : public gazebo::SensorPlugin
{
public:
  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
  {
    sensor_ = std::move(sensor);
    world_ = gazebo::physics::get_world(sensor_->WorldName());
    if (!world_) {
      gzerr << "AutoKart sensor delivery cannot resolve Gazebo world\n";
      return;
    }
    sensor_id_ = sdf->Get<std::string>("sensor_id");
    frame_id_ = sdf->Get<std::string>("frame_id");
    topic_ = sdf->Get<std::string>("ros_topic");
    metadata_topic_ =
      sdf->HasElement("metadata_topic") ?
      sdf->Get<std::string>("metadata_topic") : "/sensors/delivery_metadata";
    calibration_version_ =
      sdf->HasElement("calibration_version") ?
      sdf->Get<std::string>("calibration_version") : "SIM-CAL-1";
    processing_latency_s_ =
      sdf->HasElement("processing_latency_s") ?
      sdf->Get<double>("processing_latency_s") : 0.0;
    transport_latency_s_ =
      sdf->HasElement("transport_latency_s") ?
      sdf->Get<double>("transport_latency_s") : 0.0;
    capacity_ =
      sdf->HasElement("queue_capacity") ?
      std::max(1, sdf->Get<int>("queue_capacity")) : 8;
    drop_every_n_ =
      sdf->HasElement("drop_every_n") ?
      std::max(0, sdf->Get<int>("drop_every_n")) : 0;
    maximum_capture_state_lag_s_ =
      sdf->HasElement("maximum_capture_state_lag_s") ?
      sdf->Get<double>("maximum_capture_state_lag_s") : 0.020;
    if (processing_latency_s_ < 0.0 || transport_latency_s_ < 0.0) {
      gzerr << "AutoKart sensor delivery latency cannot be negative\n";
      return;
    }
    if (maximum_capture_state_lag_s_ <= 0.0) {
      gzerr << "AutoKart sensor capture state-age limit must be positive\n";
      return;
    }

    node_ = gazebo_ros::Node::Get(sdf);
    metadata_publisher_ =
      node_->create_publisher<autokart_msgs::msg::SensorDeliveryMetadata>(
      metadata_topic_, rclcpp::QoS(8192).reliable());
    snapshot_subscription_ = node_->create_subscription<Snapshot>(
      "/autokart/follower/authoritative_snapshot", rclcpp::QoS(64).reliable(),
      std::bind(&GazeboSensorDeliveryPlugin::OnSnapshot, this, std::placeholders::_1));
    clock_subscription_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(1).reliable(),
      std::bind(&GazeboSensorDeliveryPlugin::OnClock, this, std::placeholders::_1));

    camera_ = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(sensor_);
    depth_ = std::dynamic_pointer_cast<gazebo::sensors::DepthCameraSensor>(sensor_);
    ray_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor_);
    if (depth_) {
      image_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(
        topic_, rclcpp::SensorDataQoS());
      auto rendering_camera = depth_->DepthCamera();
      if (!rendering_camera) {
        gzerr << "AutoKart depth delivery cannot resolve the rendering camera\n";
        return;
      }
      rendering_camera->SetCaptureData(true);
      sensor_connection_ = rendering_camera->ConnectNewDepthFrame(
        std::bind(
          &GazeboSensorDeliveryPlugin::OnDepth, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
    } else if (camera_) {
      image_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(
        topic_, rclcpp::SensorDataQoS());
      auto rendering_camera = camera_->Camera();
      if (!rendering_camera) {
        gzerr << "AutoKart RGB delivery cannot resolve the rendering camera\n";
        return;
      }
      rendering_camera->SetCaptureData(true);
      sensor_connection_ = rendering_camera->ConnectNewImageFrame(
        std::bind(
          &GazeboSensorDeliveryPlugin::OnImage, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
    } else if (ray_) {
      laser_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
        topic_, rclcpp::SensorDataQoS());
      sensor_connection_ = sensor_->ConnectUpdated(
        std::bind(&GazeboSensorDeliveryPlugin::OnLaser, this));
    } else {
      gzerr << "AutoKart sensor delivery supports camera, depth, and ray sensors only\n";
      return;
    }
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&GazeboSensorDeliveryPlugin::OnWorldUpdate, this));
    sensor_->SetActive(true);
    gzmsg << "AutoKart timestamped sensor delivery active for " << sensor_id_ << "\n";
  }

private:
  void OnSnapshot(const Snapshot::SharedPtr snapshot)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_epoch_ || snapshot->reset_epoch != reset_epoch_) {
      queue_.clear();
      reset_epoch_ = snapshot->reset_epoch;
      sequence_ = 0;
      have_epoch_ = true;
      have_clock_ = false;
    }
  }

  void OnClock(const rosgraph_msgs::msg::Clock::SharedPtr clock)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_authoritative_time_s_ =
      static_cast<double>(clock->clock.sec) +
      static_cast<double>(clock->clock.nanosec) * 1e-9;
    have_clock_ = true;
  }

  bool BeginCapture(double capture_time_s)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_epoch_ || !have_clock_) {
      RecordDrop(warmup_rejections_);
      return false;
    }
    ++captured_;
    const double state_lag_s = latest_authoritative_time_s_ - capture_time_s;
    if (state_lag_s > maximum_capture_state_lag_s_ + 1e-12) {
      RecordDrop(stale_capture_rejections_);
      if (stale_capture_rejections_ == 1u) {
        gzwarn << "AutoKart " << sensor_id_ << " rejected rendered capture "
               << state_lag_s << " s behind authoritative state\n";
      }
      return false;
    }
    if (drop_every_n_ > 0 && captured_ % static_cast<uint64_t>(drop_every_n_) == 0u) {
      RecordDrop(intentional_rate_decimations_);
      return false;
    }
    return true;
  }

  void RecordDrop(uint64_t & reason_counter)
  {
    ++dropped_;
    ++reason_counter;
    ++consecutive_drops_;
    maximum_consecutive_drops_ =
      std::max(maximum_consecutive_drops_, consecutive_drops_);
  }

  void OnImage(
    const unsigned char * data, unsigned int width, unsigned int height,
    unsigned int depth, const std::string &)
  {
    const double capture_time = sensor_->LastMeasurementTime().Double();
    if (!data || width == 0u || height == 0u || depth == 0u || !BeginCapture(capture_time)) {
      return;
    }
    auto message = std::make_shared<sensor_msgs::msg::Image>();
    message->header.stamp = SensorStamp(capture_time);
    message->header.frame_id = frame_id_;
    message->width = width;
    message->height = height;
    message->encoding = depth == 3u ? "rgb8" : "mono8";
    message->is_bigendian = false;
    message->step = width * depth;
    message->data.assign(data, data + static_cast<std::size_t>(message->step) * height);
    Enqueue(
      capture_time, message->data.size(), message->encoding, width, height,
      [publisher = image_publisher_, message]() {publisher->publish(*message);});
  }

  void OnDepth(
    const float * data, unsigned int width, unsigned int height,
    unsigned int, const std::string &)
  {
    const double capture_time = sensor_->LastMeasurementTime().Double();
    if (!data || width == 0u || height == 0u || !BeginCapture(capture_time)) {
      return;
    }
    auto message = std::make_shared<sensor_msgs::msg::Image>();
    message->header.stamp = SensorStamp(capture_time);
    message->header.frame_id = frame_id_;
    message->width = width;
    message->height = height;
    message->encoding = "32FC1";
    message->is_bigendian = false;
    message->step = width * sizeof(float);
    const std::size_t bytes = static_cast<std::size_t>(message->step) * height;
    message->data.resize(bytes);
    std::memcpy(message->data.data(), data, bytes);
    Enqueue(
      capture_time, bytes, message->encoding, width, height,
      [publisher = image_publisher_, message]() {publisher->publish(*message);});
  }

  void OnLaser()
  {
    const double capture_time = sensor_->LastMeasurementTime().Double();
    if (!BeginCapture(capture_time)) {
      return;
    }
    std::vector<double> raw_ranges;
    ray_->Ranges(raw_ranges);
    if (raw_ranges.empty()) {
      ++empty_payloads_;
      return;
    }
    auto message = std::make_shared<sensor_msgs::msg::LaserScan>();
    message->header.stamp = SensorStamp(capture_time);
    message->header.frame_id = frame_id_;
    message->angle_min = ray_->AngleMin().Radian();
    message->angle_max = ray_->AngleMax().Radian();
    message->angle_increment = ray_->AngleResolution();
    message->time_increment =
      raw_ranges.size() > 1u && sensor_->UpdateRate() > 0.0 ?
      1.0 / (sensor_->UpdateRate() * raw_ranges.size()) : 0.0;
    message->scan_time = sensor_->UpdateRate() > 0.0 ? 1.0 / sensor_->UpdateRate() : 0.0;
    message->range_min = ray_->RangeMin();
    message->range_max = ray_->RangeMax();
    message->ranges.reserve(raw_ranges.size());
    for (const double range : raw_ranges) {
      message->ranges.push_back(static_cast<float>(range));
    }
    const std::size_t bytes = message->ranges.size() * sizeof(float);
    Enqueue(
      capture_time, bytes, "32FC1_RANGES", static_cast<uint32_t>(message->ranges.size()), 1u,
      [publisher = laser_publisher_, message]() {publisher->publish(*message);});
  }

  void Enqueue(
    double capture_time, std::size_t payload_size, std::string encoding,
    uint32_t width, uint32_t height, std::function<void()> publish)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_epoch_) {
      return;
    }
    if (queue_.size() >= static_cast<std::size_t>(capacity_)) {
      queue_.pop_front();
      RecordDrop(queue_overflow_drops_);
    }
    ++sequence_;
    queue_.push_back(QueuedDelivery{
      sequence_, reset_epoch_, capture_time,
      capture_time + processing_latency_s_,
      capture_time + processing_latency_s_ + transport_latency_s_,
      payload_size, std::move(encoding), width, height, std::move(publish)});
    maximum_queue_depth_ = std::max(maximum_queue_depth_, queue_.size());
  }

  void OnWorldUpdate()
  {
    const double now = world_->SimTime().Double();
    std::deque<QueuedDelivery> due;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      while (!queue_.empty() && queue_.front().due_time_s <= now + 1e-12) {
        due.push_back(std::move(queue_.front()));
        queue_.pop_front();
      }
    }
    for (auto & frame : due) {
      uint64_t cumulative_captured;
      uint64_t cumulative_delivered;
      uint64_t cumulative_dropped;
      uint64_t warmup_rejections;
      uint64_t intentional_rate_decimations;
      uint64_t queue_overflow_drops;
      uint64_t processing_deadline_misses;
      uint64_t transport_drops;
      uint64_t stale_rejections;
      uint32_t consecutive_drops;
      uint32_t maximum_consecutive_drops;
      std::size_t queue_depth;
      std::size_t maximum_queue_depth;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (frame.reset_epoch != reset_epoch_) {
          RecordDrop(stale_rejections_);
          continue;
        }
        ++delivered_;
        consecutive_drops_ = 0u;
        maximum_latency_s_ = std::max(maximum_latency_s_, now - frame.capture_time_s);
        cumulative_captured = captured_;
        cumulative_delivered = delivered_;
        cumulative_dropped = dropped_;
        warmup_rejections = warmup_rejections_;
        intentional_rate_decimations = intentional_rate_decimations_;
        queue_overflow_drops = queue_overflow_drops_;
        processing_deadline_misses = processing_deadline_misses_;
        transport_drops = transport_drops_;
        stale_rejections = stale_rejections_ + stale_capture_rejections_;
        consecutive_drops = consecutive_drops_;
        maximum_consecutive_drops = maximum_consecutive_drops_;
        queue_depth = queue_.size();
        maximum_queue_depth = maximum_queue_depth_;
      }
      frame.publish_payload();
      autokart_msgs::msg::SensorDeliveryMetadata metadata;
      metadata.header.stamp = SensorStamp(frame.capture_time_s);
      metadata.header.frame_id = frame_id_;
      metadata.sensor_id = sensor_id_;
      metadata.sequence = frame.sequence;
      metadata.reset_epoch = frame.reset_epoch;
      metadata.capture_time = SensorStamp(frame.capture_time_s);
      metadata.processing_completion_time = SensorStamp(frame.processing_completion_time_s);
      metadata.delivery_time = SensorStamp(now);
      metadata.calibration_version = calibration_version_;
      metadata.quality = "VALID";
      metadata.fault_state = "NONE";
      metadata.payload_present = frame.payload_size > 0u;
      metadata.payload_size = frame.payload_size;
      metadata.encoding = frame.encoding;
      metadata.width = frame.width;
      metadata.height = frame.height;
      metadata.cumulative_captured = cumulative_captured;
      metadata.cumulative_delivered = cumulative_delivered;
      metadata.cumulative_dropped = cumulative_dropped;
      metadata.warmup_rejections = warmup_rejections;
      metadata.intentional_rate_decimations = intentional_rate_decimations;
      metadata.queue_overflow_drops = queue_overflow_drops;
      metadata.processing_deadline_misses = processing_deadline_misses;
      metadata.transport_drops = transport_drops;
      metadata.stale_rejections = stale_rejections;
      metadata.consecutive_drops = consecutive_drops;
      metadata.maximum_consecutive_drops = maximum_consecutive_drops;
      metadata.queue_depth = static_cast<uint32_t>(queue_depth);
      metadata.maximum_queue_depth = static_cast<uint32_t>(maximum_queue_depth);
      metadata_publisher_->publish(metadata);
    }
  }

  gazebo::sensors::SensorPtr sensor_;
  gazebo::sensors::CameraSensorPtr camera_;
  gazebo::sensors::DepthCameraSensorPtr depth_;
  gazebo::sensors::RaySensorPtr ray_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr sensor_connection_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<Snapshot>::SharedPtr snapshot_subscription_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
  rclcpp::Publisher<autokart_msgs::msg::SensorDeliveryMetadata>::SharedPtr metadata_publisher_;
  std::mutex mutex_;
  std::deque<QueuedDelivery> queue_;
  std::string sensor_id_;
  std::string frame_id_;
  std::string topic_;
  std::string metadata_topic_;
  std::string calibration_version_;
  double processing_latency_s_{0.0};
  double transport_latency_s_{0.0};
  double maximum_capture_state_lag_s_{0.020};
  int capacity_{8};
  int drop_every_n_{0};
  uint32_t reset_epoch_{0};
  uint64_t sequence_{0};
  uint64_t captured_{0};
  uint64_t delivered_{0};
  uint64_t dropped_{0};
  uint64_t empty_payloads_{0};
  uint64_t warmup_rejections_{0};
  uint64_t intentional_rate_decimations_{0};
  uint64_t queue_overflow_drops_{0};
  uint64_t processing_deadline_misses_{0};
  uint64_t transport_drops_{0};
  uint64_t stale_rejections_{0};
  uint64_t stale_capture_rejections_{0};
  uint32_t consecutive_drops_{0};
  uint32_t maximum_consecutive_drops_{0};
  std::size_t maximum_queue_depth_{0};
  double maximum_latency_s_{0.0};
  double latest_authoritative_time_s_{0.0};
  bool have_epoch_{false};
  bool have_clock_{false};
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorDeliveryPlugin)
}  // namespace autokart_gazebo
