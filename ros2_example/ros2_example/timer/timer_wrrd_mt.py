import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import datetime
import time


class TimerWrRd(Node):
    """
    Timer(Write/Read)ノード

      並列処理（Multi-Thread）
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("timer_wrrd")

        # 再入可能コールバックGroupを生成
        cb_grp = ReentrantCallbackGroup()

        # 5.0秒周期でcounter1,2,3を書き込むROSタイマーの定義
        # （timer_wr_callbackは5.0秒経過する度に呼び出されるコールバック関数）
        # ※コールバック関数は再入可能に設定
        self.timer_wr = self.create_timer(5.0, self._timer_wr_callback, cb_grp)

        # 読み出しタイマーを書き込みタイマーより0.5秒遅らせて実行させる
        time.sleep(0.5)  # 0.5秒停止

        # 5.0秒周期でcounter1,2,3を読み出すROSタイマーの定義
        # （timer_rd_callbackは5.0秒経過する度に呼び出されるコールバック関数）
        # ※コールバック関数は再入可能に設定
        self.timer_rd = self.create_timer(5.0, self._timer_rd_callback, cb_grp)

        # counter1,2,3のインスタンス変数を定義
        self._counter1 = 0
        self._counter2 = 0
        self._counter3 = 0

    def _timer_wr_callback(self) -> None:
        """
        書き込みtimerコールバック
        """
        # 開始時刻を取得
        now = datetime.datetime.now()
        self.get_logger().info("[{}]timer_wr_callback:開始".format(now))

        self._counter1 += 1  # カウンター１を+1
        self.get_logger().info("  Wirte counter1:[{}] ".format(self._counter1))

        time.sleep(1)  # １秒停止

        self._counter2 += 1  # カウンター２を+1
        self.get_logger().info("  Wirte counter2:[{}]".format(self._counter2))
        time.sleep(1)  # １秒停止

        self._counter3 += 1  # カウンター３を+1
        self.get_logger().info("  Wirte counter3:[{}]".format(self._counter3))

        # 終了時刻を取得
        now = datetime.datetime.now()
        self.get_logger().info("[{}]timer_wr_callback:終了".format(now))

    def _timer_rd_callback(self) -> None:
        """
        読み出しtimerコールバック
        """
        # 開始時刻を取得
        now = datetime.datetime.now()
        self.get_logger().info("[{}]timer_rd_callback:開始".format(now))

        self.get_logger().info("  Read counter1:[{}]".format(self._counter1))
        self.get_logger().info("  Read counter2:[{}]".format(self._counter2))
        self.get_logger().info("  Read counter3:[{}]".format(self._counter3))

        # 終了時刻を取得
        now = datetime.datetime.now()
        self.get_logger().info("[{}]timer_rd_callback:終了".format(now))


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # timer_wrrdノードの作成
    twr = TimerWrRd()
    twr.get_logger().info("timer_wrrd start!")
    # マルチスレッドExecutorを生成
    executor = MultiThreadedExecutor()
    try:
        # ノードの実行開始
        # （マルチスレッドで実行させる）
        rclpy.spin(twr, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        twr.destroy_node()
