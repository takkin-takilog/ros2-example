from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ノード・アクションの定義
    node_act1 = Node(
        package="ros2_example",
        executable="bb_param",
        name=None,
        namespace="name_space_1",
        parameters=[{"sma": 5, "sigma": 1.0}],
        output="screen",
    )
    node_act2 = Node(
        package="ros2_example",
        executable="bb_param",
        name=None,
        namespace="name_space_2",
        parameters=[{"sma": 10, "sigma": 2.0}],
        output="screen",
    )

    # LaunchDescriptionオブジェクトの生成
    ld = LaunchDescription()

    # ノード・アクションの追加
    ld.add_action(node_act1)
    ld.add_action(node_act2)

    return ld
