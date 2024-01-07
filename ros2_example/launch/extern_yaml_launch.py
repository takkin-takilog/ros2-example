import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    # 読み込ませるYAMLファイル名の定義
    BB_PARAM_YAML = "bb_param_ext.yaml"

    # Launch引数名の定義
    VN_BB_PARAM = "bb_param"

    # パッケージとLaunchディレクトリの取得
    package_dir = get_package_share_directory("ros2_example")
    launch_dir = os.path.join(package_dir, "launch")

    # YAMLファイル格納場所のフルパスを取得
    yaml_fullpath = os.path.join(package_dir, "params", BB_PARAM_YAML)

    # Launchアクションの定義
    # ロードするLaunchファイルを指定する
    # Launch引数はlaunch_argumentsで指定する
    launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "bb_param_yaml_ext_launch.py")
        ),
        launch_arguments={
            VN_BB_PARAM: yaml_fullpath,
        }.items(),
    )

    # LaunchDescriptionオブジェクトの生成
    ld = LaunchDescription()

    # Launchアクションの追加
    ld.add_action(launch_cmd)

    return ld
