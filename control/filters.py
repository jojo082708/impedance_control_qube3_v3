"""
control/filters.py — 一階 IIR 低通濾波器。
"""
import math


class LowPassFilter:
    """y[k] = α·y[k-1] + (1-α)·x[k]"""

    def __init__(self, fc: float, dt: float):
        dt_safe     = max(dt, 1e-9)          # 防止 dt=0 造成 alpha=1
        self._alpha = math.exp(-2 * math.pi * fc * dt_safe)
        self._y     = 0.0

    def reset(self, value: float = 0.0):
        self._y = value

    def update(self, x: float) -> float:
        self._y = self._alpha * self._y + (1 - self._alpha) * x
        return self._y
