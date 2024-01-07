import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 読み込ませるYAMLファイル名の定義
    BB_PARAM_YAML = "bb_param.yaml"

    # Launch引数名の定義
    VN_BB_PARAM = "bb_param"

    # パッケージ・ディレクトリの取得
    package_dir = get_package_share_directory("ros2_example")

    # Launch引数の値を受け取るための変数定義
    lc_bb_param_yaml = LaunchConfiguration(VN_BB_PARAM)

    # Launch引数の宣言
    #   ※"default_value"を設定した場合、外部から呼び出されたときに引数が指定されていないと
    #   　"default_value"の値が初期値として設定される
    #   ※"default_value"を設定しない場合、外部から呼び出されたときに引数が指定されていないと
    #   　エラーとなり、処理を停止する。
    declare_bb_params_cmd = DeclareLaunchArgument(
        VN_BB_PARAM,
        default_value=os.path.join(package_dir, "params", BB_PARAM_YAML),
        description="Full path to the ROS2 parameters file to use for the launched node",
    )

    # ノード・アクションの定義
    node_act1 = Node(
        package="ros2_example",
        executable="bb_param",
        name=None,
        namespace="name_space_1",
        parameters=[lc_bb_param_yaml],
        output="screen",
    )
    node_act2 = Node(
        package="ros2_example",
        executable="bb_param",
        name=None,
        namespace="name_space_2",
        parameters=[lc_bb_param_yaml],
        output="screen",
    )

    # LaunchDescriptionオブジェクトの生成
    ld = LaunchDescription()

    # Launch引数アクションの追加
    ld.add_action(declare_bb_params_cmd)

    # ノード・アクションの追加
    ld.add_action(node_act1)
    ld.add_action(node_act2)

    return ld
