"""
control/step.py — 單一控制步驟的計算邏輯。

將感測器讀值 → 外力估算 → 阻抗動力學 → PD 控制器 → 電壓輸出
包裝成一個純計算函式 run_step()，方便測試與替換。
"""
import numpy as np
from config import Kt, J_motor, B_friction, VOLTAGE_LIMIT, ROW_FIELDS


class StepContext:
    """每輪實驗開始時建立，儲存跨步驟的可變狀態（濾波器、積分器）。"""

    def __init__(self, spd_f, acc_f, frc_f, imp_dyn):
        self.prev_theta = None   # None 表示本輪第一步
        self.prev_omega = 0.0
        self.spd_f      = spd_f
        self.acc_f      = acc_f
        self.frc_f      = frc_f
        self.imp_dyn    = imp_dyn


def run_step(theta: float, current: float,
             ctx: StepContext, params: tuple,
             safety, dt: float,
             rnd: int, timestamp: float) -> tuple:
    """
    執行一個控制週期的完整計算。

    Args:
        theta:     本步馬達角度 [rad]
        current:   本步馬達電流 [A]
        ctx:       StepContext（含濾波器與積分器狀態）
        params:    (K, B, M, Kp, Kd, theta_d)
        safety:    SafetyChecker 實例
        dt:        取樣時間 [s]
        rnd:       輪次編號
        timestamp: 實驗經過時間 [s]

    Returns:
        (safe, reason, voltage, row)
        safe=False 時 voltage=0.0、row=None。
    """
    K, B, M, Kp, Kd, theta_d = params

    # ── 速度 / 加速度估算 ──────────────────────────────────────────────────
    if ctx.prev_theta is None:
        ctx.prev_theta = theta
    omega_raw      = (theta - ctx.prev_theta) / dt
    ctx.prev_theta = theta

    omega          = ctx.spd_f.update(omega_raw)
    alpha_raw      = (omega - ctx.prev_omega) / dt
    alpha          = ctx.acc_f.update(alpha_raw)
    ctx.prev_omega = omega

    # ── 外力估算：τ_ext = Kt·I − (J·α + Bf·ω) ─────────────────────────────
    tau_motor = Kt * current
    tau_model = J_motor * alpha + B_friction * omega
    force_est = ctx.frc_f.update(tau_motor - tau_model)

    # ── 安全檢查 ───────────────────────────────────────────────────────────
    safe, reason = safety.check(theta, omega, current, force_est)
    if not safe:
        return False, reason, 0.0, None
    if reason == "force_zero":
        force_est = 0.0

    # ── 阻抗動力學 → 修正目標角度 ─────────────────────────────────────────
    x_imp     = ctx.imp_dyn.update(force_est, K, B, M, dt)
    theta_cmd = theta_d + x_imp

    # ── 內迴路 PD 控制器 ───────────────────────────────────────────────────
    error_pos   = theta_cmd - theta
    error_vel   = ctx.imp_dyn.velocity - omega
    voltage     = float(np.clip(Kp * error_pos + Kd * error_vel,
                                -VOLTAGE_LIMIT, VOLTAGE_LIMIT))

    row = _make_row(rnd, timestamp, theta, theta_d, theta_cmd,
                    omega, voltage, current, force_est, K, B, M, Kp, Kd)
    return True, reason, voltage, row


def _make_row(rnd, t, theta, theta_d, theta_cmd, omega, voltage,
              current, force_est, K, B, M, Kp, Kd) -> dict:
    return {
        "round":         rnd,
        "time":          round(t,         4),
        "theta_rad":     round(theta,     6),
        "theta_d_rad":   round(theta_d,   6),
        "theta_cmd_rad": round(theta_cmd, 6),
        "omega_rads":    round(omega,     6),
        "voltage_V":     round(voltage,   6),
        "current_A":     round(current,   6),
        "force_est_Nm":  round(force_est, 6),
        "K_Nm_rad":      round(K,         3),
        "B_Nms_rad":     round(B,         4),
        "M_kgm2":        round(M,         4),
        "Kp":            round(Kp,        3),
        "Kd":            round(Kd,        3),
    }
