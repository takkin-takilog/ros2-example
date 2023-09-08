import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from ros2_example_msgs.msg import CustomPricing


class CustomPricingSubscriber(Node):
    """
    独自型為替レートトピックSubscriberノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("custom_pricing_subscriber")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL, reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 独自型のcustom_pricingトピックを受信するsubscriptionの定義
        # （listener_callbackは受信毎に呼び出されるコールバック関数）
        self.sub = self.create_subscription(
            CustomPricing, "custom_pricing", self._listener_callback, qos_profile
        )

    def _listener_callback(self, msg: CustomPricing) -> None:
        """
        subscriptionコールバック
        """
        # 受信msgの中身をログ出力
        self.get_logger().info(
            "＜受信＞現在レート:Ask={:.3f}, Bid={:.3f}".format(msg.ask, msg.bid)
        )


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # custom_pricing_subscriberノードの作成
    cps = CustomPricingSubscriber()
    cps.get_logger().info("custom_pricing_subscriber start!")

    try:
        # ノードの実行開始
        rclpy.spin(cps)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        cps.destroy_node()
