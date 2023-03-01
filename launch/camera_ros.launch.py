import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        # The name can be anything.
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                # Names defined in CMakelists.txt
                package="camera_ros",
                plugin="camera::CameraNode",
                name="camera_node",
                remappings=[("/image_raw", "/camera/image")],
                # The important parameters are:
                # Other settings are available but this will do for now.
                # "format": "RGB888",
                # These two can be set to anything that the camera supports.
                # 'width': 320, 'height': 240
                parameters=[
                    {"camera": 0, "format": "RGB888", "width": 320, "height": 240}
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
    )

    return launch.LaunchDescription([container])
