import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 読み込ませるYAMLファイル名の定義
    BB_PARAM_YAML = "bb_param_without_ns.yaml"

    # パッケージ・ディレクトリの取得
    package_dir = get_package_share_directory("ros2_example")

    # YAMLファイル格納場所のフルパスを取得
    yaml_file = os.path.join(package_dir, "params", BB_PARAM_YAML)

    # ノード・アクションの定義
    node_act1 = Node(
        package="ros2_example",
        executable="bb_param",
        name="bb_param_1",
        namespace=None,
        parameters=[yaml_file],
        output="screen",
    )
    node_act2 = Node(
        package="ros2_example",
        executable="bb_param",
        name="bb_param_2",
        namespace=None,
        parameters=[yaml_file],
        output="screen",
    )

    # LaunchDescriptionオブジェクトの生成
    ld = LaunchDescription()

    # ノード・アクションの追加
    ld.add_action(node_act1)
    ld.add_action(node_act2)

    return ld
