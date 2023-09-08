import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Float32


class SimplePricingSubscriber(Node):
    """
    シンプル為替レートトピックSubscriberノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("simple_pricing_subscriber")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL, reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Float32型のsimple_pricingトピックを受信するsubscriptionの定義
        # （listener_callbackは受信毎に呼び出されるコールバック関数）
        self.sub = self.create_subscription(
            Float32, "simple_pricing", self._listener_callback, qos_profile
        )

    def _listener_callback(self, msg: Float32) -> None:
        """
        subscriptionコールバック
        """
        # 受信msgの中身をログ出力
        self.get_logger().info("＜受信＞現在レート:{:.3f}".format(msg.data))


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # simple_pricing_subscriberノードの作成
    sps = SimplePricingSubscriber()
    sps.get_logger().info("simple_pricing_subscriber start!")

    try:
        # ノードの実行開始
        rclpy.spin(sps)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        sps.destroy_node()
