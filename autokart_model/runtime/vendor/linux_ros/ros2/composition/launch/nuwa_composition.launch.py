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
                    namespace='ascamera_nuwa',
                    parameters=[
                        {"confiPath": "./composition/configurationfiles"},
                        {"usb_bus_no": -1},
                        {"usb_path": "null"},
                        {"color_pcl": False},
                        # {"depth_width": 640},
                        # {"depth_height": 400},
                        # {"fps": 30},
                    ]),
                # ComposableNode(
                #     package='ascamera_component',
                #     plugin='composition::AsCamera',
                #     name='AsCamera',
                #     namespace='ascamera_nuwa',
                #     parameters=[
                #         {"confiPath": "./composition/configurationfiles"},
                #         {"usb_bus_no": -1},
                #         {"usb_path": "null"},
                #         {"color_pcl": False},
                #         {"depth_width": 640},
                #         {"depth_height": 400},
                #         {"fps": 30},
                #     ]),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])