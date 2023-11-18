import time
import datetime
import random
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros2_example_msgs.srv import PriceQuery


class PriceServer(Node):
    """
    為替レートサービスServerノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("price_server")

        # ロガー取得
        self.logger = self.get_logger()

        # circleサービスserverの定義
        self.srv = self.create_service(PriceQuery, "price_query", self._get_price)

        # 為替レート初期値
        self._price = 100.0

    def _get_price(self, req, rsp):
        """
        price_queryサービスコールバック
        """
        now = datetime.datetime.now()
        self.logger.info("[{}]サービス処理中・・・".format(now.time()))

        # 重たい処理の代わり
        time.sleep(3)  # ３秒停止

        # 乱数で為替レートの変動を模擬
        self._price += random.uniform(10.0, -10.0)
        rsp.price = self._price

        now = datetime.datetime.now()
        self.logger.info("[{}]完了".format(now.time()))

        return rsp


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # price_serverノードの作成
    ps = PriceServer()
    ps.get_logger().info("price_server start!")

    try:
        # ノードの実行開始
        rclpy.spin(ps)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        ps.destroy_node()
