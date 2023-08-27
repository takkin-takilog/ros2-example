import numpy as np


class SineWave:
    """
    正弦波(SIN波)を生成する
    """

    def __init__(self, amp: float, frq: float, ofs: float) -> None:
        """
        初期化

        :param amp: 振幅
        :param frq: 周波数[Hz]
        :param ofs: 出力値オフセット
        """
        self._AMP = amp
        self._OFS = ofs
        self._GAIN = 2 * np.pi * frq

    def get_value(self, elapsed_time: float) -> float:
        """
        正弦波出力値を取得する

        :param elapsed_time: 経過時間[秒]
        """
        return self._AMP * np.sin(self._GAIN * elapsed_time) + self._OFS
