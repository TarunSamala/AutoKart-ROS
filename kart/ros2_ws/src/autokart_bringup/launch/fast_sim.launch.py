from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("track", default_value="configs/tracks/cone_track.yaml"),
        DeclareLaunchArgument("steps", default_value="1000"),
        ExecuteProcess(cmd=[
            "python3", "-m", "kart_sim.cli", "fast",
            "--track", LaunchConfiguration("track"),
            "--steps", LaunchConfiguration("steps"),
        ], output="screen"),
    ])
