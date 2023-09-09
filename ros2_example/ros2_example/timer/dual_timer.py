import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import datetime


class DualTimer(Node):
    """
    デュアルTimerノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("dual_timer")

        # ロガー取得
        self.logger = self.get_logger()

        # 1.0秒周期で実行されるROSタイマー①の定義
        # （timer1_callbackは1.0秒経過する度に呼び出されるコールバック関数）
        self.timer1 = self.create_timer(1.0, self._timer1_callback)

        # 3.0秒周期で実行されるROSタイマー②の定義
        # （timer2_callbackは3.0秒経過する度に呼び出されるコールバック関数）
        self.timer2 = self.create_timer(3.0, self._timer2_callback)

    def _timer1_callback(self) -> None:
        """
        timer①コールバック
        """
        # 現在時刻を取得
        now = datetime.datetime.now()
        # 現在時刻をログ出力
        self.logger.info("＜タイマー１＞現在時刻:{}".format(now))

    def _timer2_callback(self) -> None:
        """
        timer②コールバック
        """
        # 現在時刻を取得
        now = datetime.datetime.now()
        # 現在時刻をログ出力
        self.logger.info("＜タイマー２＞現在時刻:{}".format(now))


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # dual_timerノードの作成
    dt = DualTimer()
    dt.get_logger().info("dual_timer start!")

    try:
        # ノードの実行開始
        rclpy.spin(dt)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        dt.destroy_node()
