#pragma once
#include <thread>
#include <memory>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "CameraPublisher.h"
#include "LogRedirectBuffer.h"

using namespace std;

namespace AngstrongRos
{
class CameraNodelet : public nodelet::Nodelet
{
public:
    CameraNodelet();
    ~CameraNodelet();
private:
    virtual void onInit() override;

private:
    int process();

private:
    CameraPublisher *m_publisher = nullptr;
    ros::NodeHandle m_nh;
    bool m_running = false;
    std::thread m_process;
    std::shared_ptr<LogRedirectBuffer> m_log_buff;
};
}
