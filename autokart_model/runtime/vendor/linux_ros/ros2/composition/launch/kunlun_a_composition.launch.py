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
                    namespace='ascamera_kunlunA',
                    parameters=[
                        {"confiPath": "./composition/configurationfiles"},
                        {"depth_width": -1},
                        {"depth_height": -1},
                        {"peak_width": -1},
                        {"peak_height": -1},
                        {"fps": -1},
                        {"usb_bus_no": -1},
                        {"usb_path": "null"},
                        {"color_pcl": False},
                    ]),
                # ComposableNode(
                #     package='ascamera_component',
                #     plugin='composition::AsCamera',
                #     name='AsCamera',
                # 	namespace='ascamera_kunlunA',
                #     parameters=[
                #         {"confiPath": "./composition/configurationfiles"},
                #         {"depth_width": -1},
                #         {"depth_height": -1},
                #         {"peak_width": -1},
                #         {"peak_height": -1},
                #         {"fps": -1},
                #         {"usb_bus_no": 2},
                #         {"usb_path": "2.2"},
                #         {"color_pcl": False},
                #     ]),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])