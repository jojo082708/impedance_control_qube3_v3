"""
control/safety.py — 安全限制檢查器。

每個控制週期呼叫一次 check()。
- 角度 / 速度 / 電流超限 → 觸發緊急停止（設定 emergency event）
- 外力估算異常         → 只警告 + 本週期外力歸零，不停止
- 暖機期               → 跳過電流 / 外力檢查
"""
import math
from config import (ANGLE_LIMIT_RAD, SPEED_LIMIT_RADS,
                    CURRENT_LIMIT, FORCE_EST_LIMIT, WARMUP_CYCLES)


class SafetyChecker:

    def __init__(self, log_cb, emergency_event):
        self._log       = log_cb
        self._emergency = emergency_event
        self._triggered = False
        self._cycle     = 0

    def check(self, theta: float, omega: float,
              current: float, force_est: float) -> tuple:
        """回傳 (safe: bool, reason: str)。"""
        if self._triggered:
            return False, "緊急停止已觸發"

        self._cycle += 1
        in_warmup = self._cycle <= WARMUP_CYCLES

        if abs(theta) > ANGLE_LIMIT_RAD:
            return self._trigger(
                f"角度超限 {math.degrees(theta):.1f}° "
                f"(限制 ±{math.degrees(ANGLE_LIMIT_RAD):.0f}°)")

        if abs(omega) > SPEED_LIMIT_RADS:
            return self._trigger(
                f"速度超限 {omega:.2f} rad/s "
                f"(限制 ±{SPEED_LIMIT_RADS} rad/s)")

        if not in_warmup and abs(current) > CURRENT_LIMIT:
            return self._trigger(
                f"電流異常 {current:.3f} A (限制 {CURRENT_LIMIT} A)")

        if not in_warmup and abs(force_est) > FORCE_EST_LIMIT:
            self._log(
                f"[WARN] 外力估算異常 {force_est:.4f} N·m "
                f"(限制 {FORCE_EST_LIMIT} N·m)，本週期歸零")
            return True, "force_zero"

        return True, ""

    def _trigger(self, msg: str) -> tuple:
        self._triggered = True
        self._emergency.set()
        self._log(f"[EMERGENCY] {msg}")
        return False, msg

    def reset(self):
        self._triggered = False
        self._cycle     = 0
        self._emergency.clear()
