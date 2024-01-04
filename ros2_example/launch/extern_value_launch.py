import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    # Launch configuration variable names
    VN_NS1_SMA = "ns1_sma"
    VN_NS1_SIGMA = "ns1_sigma"
    VN_NS2_SMA = "ns2_sma"
    VN_NS2_SIGMA = "ns2_sigma"

    # Get the directory and full-path
    package_dir = get_package_share_directory("ros2_example")
    launch_dir = os.path.join(package_dir, "launch")

    # Specify the actions
    launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "bb_param_value_ext_launch.py")
        ),
        launch_arguments={
            VN_NS1_SMA: "100",
            VN_NS1_SIGMA: "100.11",
            VN_NS2_SMA: "300",
            VN_NS2_SIGMA: "300.33",
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of nodes
    ld.add_action(launch_cmd)

    return ld
