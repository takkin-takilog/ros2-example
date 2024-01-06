import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # YAML file
    BB_PARAM_YAML = "bb_param.yaml"

    # Get the package directory
    package_dir = get_package_share_directory("ros2_example")

    # Get the YAML file full path
    yaml_file = os.path.join(package_dir, "params", BB_PARAM_YAML)

    # Create the node action
    node_act1 = Node(
        package="ros2_example",
        executable="bb_param",
        name=None,
        namespace="name_space_1",
        parameters=[yaml_file],
        output="screen",
    )
    node_act2 = Node(
        package="ros2_example",
        executable="bb_param",
        name=None,
        namespace="name_space_2",
        parameters=[yaml_file],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the nodes
    ld.add_action(node_act1)
    ld.add_action(node_act2)

    return ld
