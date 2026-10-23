#include "ascamera_composition.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "LogRedirectBuffer.h"


using namespace std::chrono_literals;

namespace composition
{

// Create a AsCamera "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
AsCamera::AsCamera(const rclcpp::NodeOptions &options)
    : CameraPublisher(options)
{
    auto logger = this->get_logger();
    log_buff_ = std::make_shared<LogRedirectBuffer>(logger);
    std::cout.rdbuf(log_buff_.get());
    std::cerr.rdbuf(log_buff_.get());

    // Use a timer to schedule periodic message publishing.
    timer_ = create_wall_timer(1s, std::bind(&AsCamera::on_timer, this));
    start();
}

void AsCamera::on_timer()
{

}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::AsCamera)
