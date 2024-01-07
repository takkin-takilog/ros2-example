import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    # Launch引数名の定義
    VN_NS1_SMA = "ns1_sma"
    VN_NS1_SIGMA = "ns1_sigma"
    VN_NS2_SMA = "ns2_sma"
    VN_NS2_SIGMA = "ns2_sigma"

    # パッケージとLaunchディレクトリの取得
    package_dir = get_package_share_directory("ros2_example")
    launch_dir = os.path.join(package_dir, "launch")

    # Launchアクションの定義
    # ロードするLaunchファイルを指定する
    # Launch引数はlaunch_argumentsで指定する
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

    # LaunchDescriptionオブジェクトの生成
    ld = LaunchDescription()

    # Launchアクションの追加
    ld.add_action(launch_cmd)

    return ld
