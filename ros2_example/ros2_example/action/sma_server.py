import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from ros2_example_msgs.action import SimpleMovingAverage


class SimpleMovingAverageServer(Node):
    """
    単純移動平均計算アクションServerノード
    """

    def __init__(self) -> None:
        """
        ノードの初期化
        """
        super().__init__("sma_server")

        # ロガー取得
        self.logger = self.get_logger()

        # smaアクションserverの定義
        self.act_srv = ActionServer(
            self,
            SimpleMovingAverage,
            "sma",
            self._execute_callback,
            goal_callback=self._goal_callback,
        )

    def _goal_callback(self, goal_request) -> GoalResponse:
        """
        アクション・Goal要求コールバック
        """
        self.logger.info("＜アクション：Goal受信＞")
        self.logger.info("　区切り範囲(window):[{}]".format(goal_request.window))
        self.logger.info("　データ長:[{}]".format(len(goal_request.price_raw_list)))

        if goal_request.window < 1:
            self.logger.warn("　Goal Request: REJECT")
            self.logger.warn("　　区切り範囲(window)が0以下です。")
            return GoalResponse.REJECT

        if len(goal_request.price_raw_list) < goal_request.window:
            self.logger.warn("　Goal Request: REJECT")
            self.logger.warn("　　データ長が区切り範囲(window)より小さいです。")
            return GoalResponse.REJECT

        self.logger.info("　Goal Request: ACCEPT")
        return GoalResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        """
        アクション実行コールバック
        """
        # アクションの実行
        self.logger.info("アクション実行中・・・")

        # Feedbackメッセージの定義
        fb_msg = SimpleMovingAverage.Feedback()
        fb_msg.progress = 0

        # Resultの定義
        result = SimpleMovingAverage.Result()

        goal = goal_handle.request
        price_len = len(goal.price_raw_list)
        prg_rate = float(100.0 / (price_len - goal.window + 1))

        # リスト位置 0〜goal.window-1 は計算不可のためNaNで埋める
        for i in range(0, goal.window - 1):
            result.price_sma_list.append(np.nan)

        # リスト位置 goal.window-1 以降で計算
        for i in range(goal.window, price_len + 1):
            # 移動平均の計算
            price_sum = 0.0
            for j in range(i - goal.window, i):
                price_sum += goal.price_raw_list[j]
            result.price_sma_list.append(price_sum / goal.window)

            # 0.5秒待機（重たい処理の代わり）
            time.sleep(0.5)

            # アクションのフィードバックの送信
            fb_msg.progress = int(prg_rate * (i - goal.window + 1))
            goal_handle.publish_feedback(fb_msg)
            self.logger.info("フィードバック送信：進捗{}%".format(fb_msg.progress))

        # アクションステータスに"成功"をセット
        goal_handle.succeed()
        self.get_logger().info("Goal成功")
        # アクション実行結果を送信
        return result


def main(args: list[str] | None = None) -> None:
    # ROSの初期化
    rclpy.init(args=args)
    # sma_serverノードの作成
    mas = SimpleMovingAverageServer()
    mas.get_logger().info("sma_server start!")

    try:
        # ノードの実行開始
        rclpy.spin(mas)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        # ROSのシャットダウン
        rclpy.shutdown()
    finally:
        # ノードの破棄
        mas.destroy_node()
