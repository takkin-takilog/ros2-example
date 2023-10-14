import datetime
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from ros2_example_msgs.srv import PriceQuery


class PriceClient(Node):
    """
    為替レートサービスClientノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("price_client_async")

        # ロガー取得
        self.logger = self.get_logger()

        # circleサービスclientの定義
        self.cli = self.create_client(PriceQuery, "price_query")

        # サービスServerが有効になるまで待機
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info("サービスServerが有効になるまで待機中・・・")

        # 1.0秒周期で実行されるROSタイマーの定義
        self.timer = self.create_timer(1.0, self._main_routine)

        # 5.0秒周期で実行されるROSタイマーの定義
        self.timer = self.create_timer(5.0, self._service_call_async)

        # Future初期化
        self._future = Future()
        # 為替レート初期値
        self._price = 100.0

    def _main_routine(self) -> None:
        """
        timerコールバック(メインルーチン)
        """
        # price_queryサービスの応答がある場合は、為替レートを更新する。
        if self._future.done():
            rsp = self._future.result()
            self._future = Future()
            if rsp is not None:
                self.logger.info("＜サービス応答＞為替レート更新")
                self._price = rsp.price

        # 現在時刻をログ出力
        now = datetime.datetime.now()
        self.logger.info("＜メインルーチン処理＞[{}]為替レート:{:.3f}".format(now.time(), self._price))

    def _service_call_async(self) -> None:
        """
        price_queryサービス呼び出し（非同期）
        """
        # price_queryサービスの引数
        req = PriceQuery.Request()

        # サービスの非同期実行
        self.logger.info("＜サービス要求＞送信")
        self._future = self.cli.call_async(req)


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # price_clientノードの作成
    pc = PriceClient()
    pc.get_logger().info("price_client start!")

    try:
        # ノードの実行開始
        rclpy.spin(pc)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        pc.destroy_node()
