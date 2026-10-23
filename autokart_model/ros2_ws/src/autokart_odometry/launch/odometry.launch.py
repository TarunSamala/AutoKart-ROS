from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(package='autokart_odometry',executable='odometry_node',parameters=[str(Path(__file__).resolve().parent.parent/'config/odometry.yaml')])])
