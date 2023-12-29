import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter


class SmaParameter(Node):
    """
    パラメータノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("sma_parameter")

        # ロガー取得
        self.logger = self.get_logger()

        # パラメータの定義
        self.declare_parameter("window_size", Parameter.Type.INTEGER)

        # 1秒周期で実行されるROSタイマーの定義
        self.timer = self.create_timer(1, self._timer_callback)

    def _timer_callback(self) -> None:
        """
        timerコールバック
        """
        # パラメータの値を取得
        window_size = self.get_parameter("window_size").value
        # 取得したパラメータの値をログ出力
        self.logger.info("window_size:[{}]".format(window_size))


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # ノードの作成
    sp = SmaParameter()
    sp.get_logger().info("sma_parameter start!")

    try:
        # ノードの実行開始
        rclpy.spin(sp)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        sp.destroy_node()
