import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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

        # ロガー取得
        self.logger = self.get_logger()

        # 開始時刻を取得
        self._start_time = datetime.datetime.now()

        # 書き込み用コールバックGroupを生成
        cb_grp_wr = MutuallyExclusiveCallbackGroup()
        # 読み出し用コールバックGroupを生成
        cb_grp_rd = MutuallyExclusiveCallbackGroup()

        # 5.0秒周期でcounter1,2,3を書き込むROSタイマーの定義
        # （timer_wr_callbackは5.0秒経過する度に呼び出されるコールバック関数）
        # 書き込み用コールバックGroupを設定
        self.timer_wr = self.create_timer(5.0, self._timer_wr_callback, cb_grp_wr)

        # 読み出しタイマーを書き込みタイマーより0.5秒遅らせて実行させる
        time.sleep(0.5)  # 0.5秒停止

        # 5.0秒周期でcounter1,2,3を読み出すROSタイマーの定義
        # （timer_rd_callbackは5.0秒経過する度に呼び出されるコールバック関数）
        # 読み出し用コールバックGroupを設定
        self.timer_rd = self.create_timer(5.0, self._timer_rd_callback, cb_grp_rd)

        # counter1,2,3のインスタンス変数を定義
        self._counter1 = 0
        self._counter2 = 0
        self._counter3 = 0

    def _timer_wr_callback(self) -> None:
        """
        書き込みtimerコールバック
        """
        # 開始経過時刻を取得
        elps = datetime.datetime.now() - self._start_time
        self.logger.info("[{}]timer_wr_callback:開始".format(elps))

        self._counter1 += 1  # カウンター１を+1
        self.logger.info("[{}]Wirte counter1:[{}]".format(elps, self._counter1))
        time.sleep(1)  # １秒停止

        self._counter2 += 1  # カウンター２を+1
        self.logger.info("[{}]Wirte counter2:[{}]".format(elps, self._counter2))
        time.sleep(1)  # １秒停止

        self._counter3 += 1  # カウンター３を+1
        self.logger.info("[{}]Wirte counter3:[{}]".format(elps, self._counter3))
        time.sleep(1)  # １秒停止

        # 終了経過時間を取得
        elps = datetime.datetime.now() - self._start_time
        self.logger.info("[{}]timer_wr_callback:終了".format(elps))

    def _timer_rd_callback(self) -> None:
        """
        読み出しtimerコールバック
        """
        # 開始経過時刻を取得
        elps = datetime.datetime.now() - self._start_time
        self.logger.info("[{}]timer_rd_callback:開始".format(elps))

        self.logger.info("[{}]Read counter1:[{}]".format(elps, self._counter1))
        self.logger.info("[{}]Read counter2:[{}]".format(elps, self._counter2))
        self.logger.info("[{}]Read counter3:[{}]".format(elps, self._counter3))

        # 終了経過時間を取得
        elps = datetime.datetime.now() - self._start_time
        self.logger.info("[{}]timer_rd_callback:終了".format(elps))


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
