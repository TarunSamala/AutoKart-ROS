#!/usr/bin/env bash
# Stops the stack. The VCU also disarms by itself if commands stop (300 ms timeout).
pkill -f "ros2 launch autokart_bringup" 2>/dev/null || true
pkill -f "dashboard/ros_gateway/gateway.py" 2>/dev/null || true
echo "stopped"
