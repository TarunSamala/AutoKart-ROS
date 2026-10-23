#pragma once

#include <memory>
#include "CameraPublisher.h"
#include "LogRedirectBuffer.h"

namespace composition
{

class AsCamera : public CameraPublisher
{
public:
  explicit AsCamera(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<LogRedirectBuffer> log_buff_;
};

}  // namespace composition
