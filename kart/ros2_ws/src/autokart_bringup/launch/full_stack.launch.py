from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="kart_01"),
        Node(
            package="autokart_sim_bridge",
            executable="vcu_bridge",
            name="vcu_bridge",
            parameters=[{"use_sim_time": True, "integrated_mode": True, "direct_actuator_commands": False}],
            output="screen",
        ),
    ])
