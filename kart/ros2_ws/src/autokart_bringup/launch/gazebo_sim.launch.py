from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Internal physics remains authoritative; Gazebo is a visual/sensor follower.
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="simulation/worlds/autokart_empty_test_world.sdf"),
        DeclareLaunchArgument("physics_backend", default_value="internal"),
        ExecuteProcess(cmd=["gazebo", "--verbose", LaunchConfiguration("world")], output="screen"),
    ])
