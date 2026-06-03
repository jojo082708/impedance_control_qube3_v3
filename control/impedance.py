"""
control/impedance.py — 離散阻抗動力學。

完整方程式：M·x'' + B·x' + K·x = F_ext
  x = theta_cmd − theta_d  （虛擬位移）
  theta_cmd = theta_d + x   （修正目標角度）

M = 0 時退化為靜態彈簧：x = F_ext / K（B 無效）。
數值方法：顯式 Euler，穩定條件 dt < 2/ω_n。
"""


class ImpedanceDynamics:

    def __init__(self):
        self._x    = 0.0   # 虛擬位移
        self._xdot = 0.0   # 虛擬速度

    def reset(self):
        self._x    = 0.0
        self._xdot = 0.0

    def update(self, F_ext: float, K: float, B: float,
               M: float, dt: float) -> float:
        """回傳本步的虛擬位移 x。"""
        K_safe = max(K, 0.01)

        if M < 1e-6:                          # 靜態彈簧退化模式
            self._x    = F_ext / K_safe
            self._xdot = 0.0
            return self._x

        xddot       = (F_ext - B * self._xdot - K_safe * self._x) / M
        self._xdot += xddot * dt
        self._x    += self._xdot * dt
        return self._x

    @property
    def velocity(self) -> float:
        """虛擬速度，用於內迴路 PD 的速度誤差項。"""
        return self._xdot
