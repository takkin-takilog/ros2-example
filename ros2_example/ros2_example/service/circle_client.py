import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros2_example_msgs.srv import Circle


class CircleClient(Node):
    """
    円サービスClientノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("circle_client")

        # ロガー取得
        self.logger = self.get_logger()

        # circleサービスclientの定義
        self.cli = self.create_client(Circle, "circle")

        # サービスServerが有効になるまで待機
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info("サービスServerが有効になるまで待機中・・・")

    def service_call(self, radius: float):
        """
        circleサービス呼び出し（同期）
        """
        self.logger.info("半径:[{:.2f}]".format(radius))

        # circleサービスの引数
        req = Circle.Request()
        # circleサービスの引数に半径の値を設定
        req.radius = radius

        # サービスの非同期実行
        future = self.cli.call_async(req)

        # サービスの非同期実行が完了するまで待機
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # circle_clientノードの作成
    cc = CircleClient()
    logger = cc.get_logger()
    logger.info("circle_client start!")

    try:
        while rclpy.ok():
            # 半径をキーボード入力
            radius = float(input("input radius: "))
            # サービスリクエストを送信
            rsp = cc.service_call(radius)
            # circleサービスの返り値をログ出力
            if rsp is not None:
                logger.info("----------")
                logger.info("円周:[{:.2f}]".format(rsp.circumference))
                logger.info("円面積:[{:.2f}]".format(rsp.area))

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        cc.destroy_node()
