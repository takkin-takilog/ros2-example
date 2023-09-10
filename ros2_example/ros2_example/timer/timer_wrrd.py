import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import datetime
import time


class TimerWrRd(Node):
    """
    Timer(Write/Read)ノード
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

        # 5.0秒周期でcounter1,2,3を書き込むROSタイマーの定義
        # （timer_wr_callbackは5.0秒経過する度に呼び出されるコールバック関数）
        self.timer_wr = self.create_timer(5.0, self._timer_wr_callback)

        # 読み出しタイマーを書き込みタイマーより0.5秒遅らせて実行させる
        time.sleep(0.5)  # 0.5秒停止

        # 5.0秒周期でcounter1,2,3を読み出すROSタイマーの定義
        # （timer_rd_callbackは5.0秒経過する度に呼び出されるコールバック関数）
        self.timer_rd = self.create_timer(5.0, self._timer_rd_callback)

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

        # 経過時刻を取得
        elps = datetime.datetime.now() - self._start_time

        self._counter2 += 1  # カウンター２を+1
        self.logger.info("[{}]Wirte counter2:[{}]".format(elps, self._counter2))
        time.sleep(1)  # １秒停止

        # 経過時刻を取得
        elps = datetime.datetime.now() - self._start_time

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

    try:
        # ノードの実行開始
        rclpy.spin(twr)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        twr.destroy_node()
