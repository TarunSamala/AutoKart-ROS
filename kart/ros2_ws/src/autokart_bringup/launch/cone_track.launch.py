from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([ExecuteProcess(
        cmd=["python3", "-m", "kart_sim.cli", "fast", "--track", "configs/tracks/cone_track.yaml", "--steps", "2000"],
        output="screen",
    )])
