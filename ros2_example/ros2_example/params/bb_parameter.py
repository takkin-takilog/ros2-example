import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter


class BbParameter(Node):
    """
    ボリンジャーバンド(BB)パラメータノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("bb_param")

        # ロガー取得
        self.logger = self.get_logger()

        # パラメータの定義
        self.declare_parameter("sma", Parameter.Type.INTEGER)
        self.declare_parameter("sigma", Parameter.Type.DOUBLE)

        # 1秒周期で実行されるROSタイマーの定義
        self.timer = self.create_timer(1, self._timer_callback)

    def _timer_callback(self) -> None:
        """
        timerコールバック
        """
        # パラメータの値を取得
        sma = self.get_parameter("sma").value
        sigma = self.get_parameter("sigma").value
        # 取得したパラメータの値をログ出力
        self.logger.info("sma:[{}] sigma:[{}]".format(sma, sigma))


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # ノードの作成
    bb = BbParameter()
    bb.get_logger().info("bb_param start!")

    try:
        # ノードの実行開始
        rclpy.spin(bb)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        bb.destroy_node()
