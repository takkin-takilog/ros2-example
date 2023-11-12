import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from ros2_example_msgs.action import SimpleMovingAverage


class SimpleMovingAverageClient(Node):
    """
    単純移動平均計算アクションClientノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("sma_client_async")

        # ロガー取得
        self.logger = self.get_logger()

        # smaアクションclientの定義
        self.act_cli = ActionClient(self, SimpleMovingAverage, "sma")

        # サービスServerが有効になるまで待機
        while not self.act_cli.wait_for_server(timeout_sec=1.0):
            self.logger.info("アクションServerが有効になるまで待機中・・・")

        # Future初期化
        self._goal_future = Future()
        self._result_future = Future()

        self._price_raw_list = []
        self._window = 0

    def send_action_goal_async(self, window: int, price_raw_list: list[float]) -> None:
        """
        アクション・ゴールの送信（非同期）
        """
        # smaアクションの引数
        goal_msg = SimpleMovingAverage.Goal()
        goal_msg.window = window
        for price_raw in price_raw_list:
            goal_msg.price_raw_list.append(price_raw)

        # アクションの非同期実行
        self.logger.info("＜アクション：Goal送信＞")
        self._goal_future = self.act_cli.send_goal_async(
            goal_msg, self._feedback_callback
        )
        self._goal_future.add_done_callback(self._goal_response_callback)

        self._price_raw_list = price_raw_list
        self._window = window

    def _feedback_callback(self, feedback) -> None:
        """
        アクション・フィードバックコールバック
        """
        self.logger.info("フィードバック受信：進捗{}%".format(feedback.feedback.progress))

    def _goal_response_callback(self, future) -> None:
        """
        アクション・Goal応答コールバック
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info("　Goal拒否されました。")
            # Shutdown after receiving a result
            raise ExternalShutdownException

        self.logger.info("　Goal受理されました。")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        """
        アクション・Result応答コールバック
        """
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.info("Goal成功")
        else:
            self.logger.info("Goal失敗 (status: {})".format(status))

        # ********** 描写 **********
        label_sma = "SMA(window:" + str(self._window) + ")"
        plt.plot(self._price_raw_list, label="Raw value")
        plt.scatter(range(len(self._price_raw_list)), self._price_raw_list)
        plt.plot(result.price_sma_list, label=label_sma)
        plt.scatter(range(len(result.price_sma_list)), result.price_sma_list)
        plt.legend()
        plt.title("Simple Moving Average(SMA)")
        plt.grid(linestyle="dashed")
        plt.show()

        # Shutdown after receiving a result
        raise ExternalShutdownException


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # sma_client_asyncノードの作成
    mac = SimpleMovingAverageClient()
    logger = mac.get_logger()
    logger.info("sma_client start!")

    # 単純移動平均計算用の生データ
    price_raw_list = [
        100.0,
        102.0,
        105.0,
        112.0,
        120.0,
        122.0,
        118.0,
        110.0,
        98.0,
        88.0,
        85.0,
        90.0,
        110.0,
        125.0,
    ]

    # 区切り範囲をキーボード入力
    window = int(input("windows size: "))

    # アクション・ゴールの送信（非同期）
    mac.send_action_goal_async(window, price_raw_list)

    try:
        # ノードの実行開始
        rclpy.spin(mac)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        mac.destroy_node()
