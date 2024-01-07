from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Launch引数名の定義
    VN_NS1_SMA = "ns1_sma"
    VN_NS1_SIGMA = "ns1_sigma"
    VN_NS2_SMA = "ns2_sma"
    VN_NS2_SIGMA = "ns2_sigma"

    # Launch引数の値を受け取るための変数定義
    lc_ns1sma = LaunchConfiguration(VN_NS1_SMA)
    lc_ns1sigma = LaunchConfiguration(VN_NS1_SIGMA)
    lc_ns2sma = LaunchConfiguration(VN_NS2_SMA)
    lc_ns2sigma = LaunchConfiguration(VN_NS2_SIGMA)

    # Launch引数の宣言
    #   ※"default_value"を設定した場合、外部から呼び出されたときに引数が指定されていないと
    #   　"default_value"の値が初期値として設定される
    #   ※"default_value"を設定しない場合、外部から呼び出されたときに引数が指定されていないと
    #   　エラーとなり、処理を停止する。
    declare_ns1sma_cmd = DeclareLaunchArgument(
        VN_NS1_SMA,
        default_value="33",
        description="Bollinger-Bands SMA in namespace1",
    )
    declare_ns1sigma_cmd = DeclareLaunchArgument(
        VN_NS1_SIGMA,
        default_value="3.3",
        description="Bollinger-Bands Sigma in namespace1",
    )
    declare_ns2sma_cmd = DeclareLaunchArgument(
        VN_NS2_SMA,
        # default_value="55",
        description="Bollinger-Bands SMA in namespace2",
    )
    declare_ns2sigma_cmd = DeclareLaunchArgument(
        VN_NS2_SIGMA,
        # default_value="5.5",
        description="Bollinger-Bands Sigma in namespace2",
    )

    # ノード・アクションの定義
    node_act1 = Node(
        package="ros2_example",
        executable="bb_param",
        name=None,
        namespace="name_space_1",
        parameters=[{"sma": lc_ns1sma, "sigma": lc_ns1sigma}],
        output="screen",
    )
    node_act2 = Node(
        package="ros2_example",
        executable="bb_param",
        name=None,
        namespace="name_space_2",
        parameters=[{"sma": lc_ns2sma, "sigma": lc_ns2sigma}],
        output="screen",
    )

    # LaunchDescriptionオブジェクトの生成
    ld = LaunchDescription()

    # Launch引数アクションの追加
    ld.add_action(declare_ns1sma_cmd)
    ld.add_action(declare_ns1sigma_cmd)
    ld.add_action(declare_ns2sma_cmd)
    ld.add_action(declare_ns2sigma_cmd)

    # ノード・アクションの追加
    ld.add_action(node_act1)
    ld.add_action(node_act2)

    return ld
