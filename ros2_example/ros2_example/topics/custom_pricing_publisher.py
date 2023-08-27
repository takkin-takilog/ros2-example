import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from ros2_example_msgs.msg import CustomPricing
from ..utils.waveform import SineWave


class CustomPricingPublisher(Node):
    """
    独自型為替レートトピックPublisherノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("custom_pricing_publisher")

        # 為替レートを正弦波で生成
        self._sin_wave = SineWave(30, 0.05, 100)

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL, reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 独自型のcustom_pricingトピックを送信するpublisherの定義
        self.pub = self.create_publisher(CustomPricing, "custom_pricing", qos_profile)

        # 送信周期毎にtimer_callbackを呼び出し（送信周期は0.1秒）
        self.timer = self.create_timer(0.5, self.timer_callback)

        self._start_time = time.time()

    def timer_callback(self) -> None:
        """
        ROSタイマーコールバック
        """
        elapsed_time = time.time() - self._start_time
        # 現在為替レートを取得
        pricing = self._sin_wave.get_value(elapsed_time)

        # custom_pricingトピックにmsgを送信
        msg = CustomPricing()
        msg.ask = pricing + 1.0
        msg.bid = pricing - 1.0
        self.pub.publish(msg)

        # 送信msgの中身をログ出力
        self.get_logger().info(
            "＜送信＞経過時間:[{:.2f}]　現在レート:Ask={:.3f}, Bid={:.3f}".format(
                elapsed_time, msg.ask, msg.bid
            )
        )


def main(args: list[str] | None = None) -> None:
    # Pythonクライアントライブラリの初期化
    rclpy.init(args=args)
    # custom_pricing_publisherノードの作成
    spp = CustomPricingPublisher()
    spp.get_logger().info("custom_pricing_publisher start!")

    try:
        # ノードの実行開始
        rclpy.spin(spp)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # Pythonクライアントライブラリの終了
        rclpy.shutdown()
    finally:
        # ノードの破棄
        spp.destroy_node()
