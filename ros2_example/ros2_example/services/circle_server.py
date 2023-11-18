import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros2_example_msgs.srv import Circle


class CircleServer(Node):
    """
    円サービスServerノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("circle_server")

        # ロガー取得
        self.logger = self.get_logger()

        # circleサービスserverの定義
        self.srv = self.create_service(
            Circle,
            "circle",
            self._calc_circle,
        )

    def _calc_circle(self, req, rsp):
        """
        circleサービスコールバック
        """
        # 受信requestの中身をログ出力
        self.logger.info("＜リクエスト＞")
        self.logger.info("  - 半径:[{:.2f}]".format(req.radius))

        # 円周の計算(直径×π)
        rsp.circumference = req.radius * 2 * math.pi
        # 円面積の計算(半径^2×π)
        rsp.area = req.radius**2 * math.pi

        self.logger.info("＜レスポンス＞")
        self.logger.info("  - 円周:[{:.2f}]".format(rsp.circumference))
        self.logger.info("  - 円面積:[{:.2f}]".format(rsp.area))

        return rsp


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # ohlc_serverノードの作成
    cs = CircleServer()
    cs.get_logger().info("circle_server start!")

    try:
        # ノードの実行開始
        rclpy.spin(cs)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        cs.destroy_node()
