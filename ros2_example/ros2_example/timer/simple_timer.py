import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import datetime


class SimpleTimer(Node):
    """
    シンプルTimerノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("simple_timer")

        # ロガー取得
        self.logger = self.get_logger()

        # 1.0秒周期で実行されるROSタイマーの定義
        # （timer_callbackは1.0秒経過する度に呼び出されるコールバック関数）
        self.timer = self.create_timer(1.0, self._timer_callback)

    def _timer_callback(self) -> None:
        """
        timerコールバック
        """
        # 現在時刻を取得
        now = datetime.datetime.now()
        # 現在時刻をログ出力
        self.logger.info("現在時刻:{}".format(now))


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # simple_timerノードの作成
    st = SimpleTimer()
    st.get_logger().info("simple_timer start!")

    try:
        # ノードの実行開始
        rclpy.spin(st)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        st.destroy_node()
