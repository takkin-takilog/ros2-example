import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # YAML file
    BB_PARAM_YAML = "bb_param.yaml"

    # Launch configuration variable names
    VN_BB_PARAM = "bb_param"

    # Get the package directory
    package_dir = get_package_share_directory("ros2_example")

    # Create the launch configuration variables
    lc_bb_param_yaml = LaunchConfiguration(VN_BB_PARAM)

    # Declare the launch arguments
    #   ※"default_value"を設定した場合、外部から呼び出されたときに引数が定義されていないと
    #   　"default_value"の値が初期値として設定される
    #   ※"default_value"を設定しない場合、外部から呼び出されたときに引数が定義されていないと
    #   　エラーとなり、処理を停止する。
    declare_bb_params_cmd = DeclareLaunchArgument(
        VN_BB_PARAM,
        default_value=os.path.join(package_dir, "params", BB_PARAM_YAML),
        description="Full path to the ROS2 parameters file to use for the launched node",
    )

    # Create the node action
    node_act1 = Node(
        package="ros2_example",
        executable="bb_param",
        name="bb_param",
        namespace="name_space_1",
        parameters=[lc_bb_param_yaml],
        output="screen",
    )
    node_act2 = Node(
        package="ros2_example",
        executable="bb_param",
        name="bb_param",
        namespace="name_space_2",
        parameters=[lc_bb_param_yaml],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_bb_params_cmd)

    # Add the actions to launch all of the nodes
    ld.add_action(node_act1)
    ld.add_action(node_act2)

    return ld
