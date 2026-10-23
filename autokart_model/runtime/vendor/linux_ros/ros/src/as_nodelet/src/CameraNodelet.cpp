/**
 * @file      CameraNodelet.cpp
 * @brief     angstrong camera ROS nodelet demo program.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2024/02/05
 * @version   1.0

 */


#include <iostream>
#include <chrono>
#include "CameraNodelet.h"

namespace AngstrongRos
{

CameraNodelet::CameraNodelet()
{
    m_log_buff = std::make_shared<LogRedirectBuffer>();
    std::cout.rdbuf(m_log_buff.get());
    std::cerr.rdbuf(m_log_buff.get());
}

CameraNodelet::~CameraNodelet()
{
    m_running = false;
    if (m_process.joinable()) {
        m_process.join();
    }

    if (m_publisher != nullptr) {
        delete m_publisher;
        m_publisher = nullptr;
    }
}

void CameraNodelet::onInit()
{
    NODELET_DEBUG("Initializing nodelet...");
    ROS_INFO("Nodelet is ready, nodelet begin running");

    m_nh = getPrivateNodeHandle();
    m_publisher = new CameraPublisher(m_nh);
    m_running = true;
    m_process = std::thread(&CameraNodelet::process, this);
}

int CameraNodelet::process()
{
    int ret = 0;
    ROS_INFO("start process...");

    m_publisher->start();

    while (m_running) {
        fd_set readFdSet;
        struct timeval timeout;
        char ch;
        while (ros::ok()) {
            FD_ZERO(&readFdSet);
            FD_SET(fileno(stdin), &readFdSet);
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;
            int selectResult = select(fileno(stdin) + 1, &readFdSet, nullptr, nullptr, &timeout);
            if (FD_ISSET(fileno(stdin), &readFdSet)) {
                ch = getchar();
                if (ch == 'q') {
                    break;
                } else if (ch == 's') {
                    m_publisher->saveImage();
                } else if (ch == 'f') {
                    m_publisher->logFps(!m_publisher->getLogFps());
                } else if (ch == 'l') {
                    m_publisher->logCfgParameter();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    m_publisher->stop();

    return 0;
}

}

PLUGINLIB_EXPORT_CLASS(AngstrongRos::CameraNodelet, nodelet::Nodelet)
