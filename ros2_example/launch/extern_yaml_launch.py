import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    # YAML file
    BB_PARAM_YAML = "bb_param_ext.yaml"

    # Launch configuration variable names
    VN_BB_PARAM = "bb_param"

    # Get the directory and full-path
    package_dir = get_package_share_directory("ros2_example")
    launch_dir = os.path.join(package_dir, "launch")
    yaml_fullpath = os.path.join(package_dir, "params", BB_PARAM_YAML)

    # Specify the actions
    launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "bb_param_yaml_ext_launch.py")
        ),
        launch_arguments={
            VN_BB_PARAM: yaml_fullpath,
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of nodes
    ld.add_action(launch_cmd)

    return ld
