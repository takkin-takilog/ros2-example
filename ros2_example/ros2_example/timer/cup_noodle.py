import rclpy
from rclpy.node import Node
import datetime
import time
from std_srvs.srv import Empty


class CupNoodle(Node):
    """
    カップ麺ノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("cup_noodle")

        # ロガー取得
        self.logger = self.get_logger()

        # 3分周期で実行されるROSタイマーの定義
        # （timer_callbackは3分経過する度に呼び出されるコールバック関数）
        self.timer = self.create_timer(3 * 60, self._timer_callback)

        # 電話"phone"ROSサービスの定義
        # （take_callはROSサービスクライアントからリクエストを受けた際に
        #  呼び出されるコールバック関数）
        self.phone = self.create_service(Empty, "phone", self._take_call)

    def _timer_callback(self) -> None:
        """
        timerコールバック
        """
        # 現在時刻を取得
        now = datetime.datetime.now()
        # 現在時刻をログ出力
        self.logger.info("[{}]3分経過しました。".format(now))

        # タイマー停止
        self.timer.cancel()

        # システム終了
        raise SystemExit

    def _take_call(self, req, rsp) -> None:
        """
        "phone"serviceコールバック
        """
        # 現在時刻を取得
        now = datetime.datetime.now()
        # ログ出力
        self.logger.info("[{}]電話に出ました。".format(now))

        # 重たい処理の代わり(10秒停止)
        time.sleep(10)

        # 現在時刻を取得
        now = datetime.datetime.now()
        # ログ出力
        self.logger.info("[{}]電話を切りました。".format(now))

        return rsp


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # simple_timerノードの作成
    cn = CupNoodle()
    cn.get_logger().info("カップ麺ノードを実行開始します。")

    # 「現在時間」を取得
    now = datetime.datetime.now()
    cn.get_logger().info("[{}]カップ麺にお湯を入れました。".format(now))

    try:
        # ノードの実行開始
        rclpy.spin(cn)
    except SystemExit:
        cn.get_logger().info("カップ麺ノードを実行終了します。")

    # ROSのシャットダウン
    rclpy.shutdown()
    # ノードの破棄
    cn.destroy_node()
