from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([ExecuteProcess(
        cmd=["./scripts/run_formula_student_event.sh", "skidpad"], output="screen"
    )])
