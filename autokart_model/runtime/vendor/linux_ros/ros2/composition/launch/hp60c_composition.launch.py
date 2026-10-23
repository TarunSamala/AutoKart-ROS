from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    ascamera_node = Node(
        namespace= "ascamera_hp60c",
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": "./ascamera/configurationfiles"},
            {"color_pcl": False},
            {"pub_tfTree": True},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 15},
        ],
        remappings=[]
    )

    # ascamera_node2 = Node(
    #     namespace= "ascamera_hp60c_2",
    #     package='ascamera',
    #     executable='ascamera_node',
    #     respawn=True,
    #     output='both',
    #     parameters=[
    #         {"usb_bus_no": 3},    # set your usb_bus_no
    #         {"usb_path": "2.2"},  # set your usb_path
    #         {"confiPath": "./ascamera/configurationfiles"},
    #         {"color_pcl": False},
    #         {"depth_width": 640},
    #         {"depth_height": 480},
    #         {"rgb_width": 640},
    #         {"rgb_height": 480},
    #         {"fps": 15},
    #     ],
    #     remappings=[]
    # )

    ld.add_action(ascamera_node)
    # ld.add_action(ascamera_node2)

    return ld




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
                    namespace='ascamera_hp60c',
                    parameters=[
                        {"confiPath": "./composition/configurationfiles"},
                        {"usb_bus_no": -1},
                        {"usb_path": "null"},
                        {"color_pcl": False},
                        {"pub_tfTree": True},
                        {"depth_width": 640},
                        {"depth_height": 480},
                        {"rgb_width": 640},
                        {"rgb_height": 480},
                        {"fps": 15}
                    ]),
                # ComposableNode(
                #     package='ascamera_component',
                #     plugin='composition::AsCamera',
                #     name='AsCamera2',
                #     namespace='ascamera_hp60c2',
                #     parameters=[
                #         {"confiPath": "./composition/configurationfiles"},
                #         {"usb_bus_no": 3},
                #         {"usb_path": "2.1"},
                #         {"color_pcl": False},
                #         {"pub_tfTree": True},
                #         {"depth_width": 640},
                #         {"depth_height": 480},
                #         {"rgb_width": 640},
                #         {"rgb_height": 480},
                #         {"fps": 15}
                #     ]),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])



