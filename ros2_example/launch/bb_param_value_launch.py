from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Create the node action
    node_act1 = Node(
        package="ros2_example",
        executable="bb_param",
        name="bb_param",
        namespace="name_space_1",
        parameters=[{"sma": 5, "sigma": 1.0}],
        output="screen",
    )
    node_act2 = Node(
        package="ros2_example",
        executable="bb_param",
        name="bb_param",
        namespace="name_space_2",
        parameters=[{"sma": 10, "sigma": 2.0}],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the nodes
    ld.add_action(node_act1)
    ld.add_action(node_act2)

    return ld
