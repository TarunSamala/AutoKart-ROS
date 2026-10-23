import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ascamera_component',
                    plugin='composition::AsCamera',
                    name='AsCamera',
                    namespace='ascamera',
                    parameters=[
                        {"confiPath": "./composition/configurationfiles"},
                        # {"usb_bus_no": 3},
                        # {"usb_path": "2.2"}
                    ]),
                # ComposableNode(
                #     package='ascamera_component',
                #     plugin='composition::AsCamera',
                #     name='AsCamera2',
                #     namespace='ascamera2',
                #     parameters=[
                #         {"confiPath": "./composition/configurationfiles"},
                #         {"usb_bus_no": 3},
                #         {"usb_path": "2.1"}
                #     ]),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])