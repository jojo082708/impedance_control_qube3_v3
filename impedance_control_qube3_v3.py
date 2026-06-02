## impedance_control_qube3.py
# QUBE-Servo 3 — 阻抗控制  v3
# ─ 架構：位置內迴路阻抗控制
# ─ 外力估算：τ_ext = Kt × I - (J × α + B_friction × ω)
# ─ 安全機制：角度限制 / 速度限制 / 電壓限制 / 緊急停止
# ─ 末端：紅色旋臂（不掛擺錘，無重力補償）
# ─────────────────────────────────────────────────────────────────────────────
# [v3 修正]
#  1. 阻抗模型修正：完整實作 M·ẍ + B·ẋ + K·x = F_ext 離散積分
#     → M / B 參數現在真正影響控制行為，不再是擺設
#  2. 速度估算不再依賴繪圖 buffer（跨輪污染問題修正）
#     → 改用控制迴圈內部 prev_theta 局部變數
#  3. 安全檢查每週期只呼叫一次，修正 _cycle 計數器雙重遞增 bug
#  4. round_history 賦值改在鎖內執行，消除 GUI / 控制 thread 的 race condition
#  5. _make_row 改用固定欄位名稱，避免動態 key 導致 CSV/Excel 資料靜默丟失
#  6. KILL / PAUSE / EMERGENCY 改用 threading.Event，語意明確
#  7. FORCE_EST_LIMIT 調整為合理值（QUBE 額定扭矩量級）
#  8. 魔術數字提升為具名常數
#  9. M=0 時阻抗模型退化為靜態彈簧，並在 GUI 顯示警告
# 10. 暖機期結束後，LowPassFilter 已穩定，外力限制再啟用
# ─────────────────────────────────────────────────────────────────────────────

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import time
import csv
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
import datetime
import openpyxl
from openpyxl.styles import Font, PatternFill, Alignment
import math

# ═══════════════════════════════════════════════════════════════════════════════
# MOTOR PARAMETERS  ← 辨識完成後在此修改
# ═══════════════════════════════════════════════════════════════════════════════
Kt          = 0.042      # 力矩常數   N·m/A
J_motor     = 4.0e-6    # 轉子慣量   kg·m²
B_friction  = 1.0e-5    # 黏性摩擦   N·m·s/rad

# ═══════════════════════════════════════════════════════════════════════════════
# SAFETY LIMITS
# ═══════════════════════════════════════════════════════════════════════════════
ANGLE_LIMIT_RAD   = math.radians(270)  # ±270°
SPEED_LIMIT_RADS  = 50.0               # rad/s
VOLTAGE_LIMIT     = 10.0               # V
CURRENT_LIMIT     = 4.0                # A

# [v3] 外力上限設定為合理值（QUBE 額定扭矩 ~0.1 N·m 量級）
FORCE_EST_LIMIT   = 1.0                # N·m

# [v3] 具名常數取代魔術數字
WARMUP_CYCLES       = 50      # 暖機週期數（dt=0.002 → 100ms）
PLOT_DOWNSAMPLE_PTS = 800     # 繪圖最大取樣點數
POLL_INTERVAL_MS    = 200     # 控制 thread 輪詢間隔 ms
PLOT_INTERVAL_MS    = 80      # 繪圖更新間隔 ms
SIM_VOLT_TO_CURR    = 0.15    # 虛擬模式：電壓→電流近似係數 A/V
THREAD_WAIT_MAX     = 30      # 等待舊 thread 最多次數（×POLL_INTERVAL_MS）

# ═══════════════════════════════════════════════════════════════════════════════
# FILTER
# ═══════════════════════════════════════════════════════════════════════════════
FC_SPEED  = 40.0
FC_ACCEL  = 15.0
FC_FORCE  = 15.0

# ═══════════════════════════════════════════════════════════════════════════════
# THREAD EVENTS  ← [v3] 以 threading.Event 取代裸全域布林
# ═══════════════════════════════════════════════════════════════════════════════
_kill_event      = threading.Event()   # set() → 要求停止
_pause_event     = threading.Event()   # set() → 暫停中
_emergency_event = threading.Event()   # set() → 緊急停止

data_lock      = threading.Lock()
control_thread = None

# ═══════════════════════════════════════════════════════════════════════════════
# DATA BUFFERS
# ═══════════════════════════════════════════════════════════════════════════════
BUFFER_SIZE   = 4000
time_buf      = deque(maxlen=BUFFER_SIZE)
pos_buf       = deque(maxlen=BUFFER_SIZE)
desired_buf   = deque(maxlen=BUFFER_SIZE)
voltage_buf   = deque(maxlen=BUFFER_SIZE)
speed_buf     = deque(maxlen=BUFFER_SIZE)
force_est_buf = deque(maxlen=BUFFER_SIZE)
cmd_pos_buf   = deque(maxlen=BUFFER_SIZE)

all_rounds_history = []
round_history      = []
round_counter      = 0
_last_render_len   = 0

# 即時阻抗參數（熱更新，由 GUI 寫入，控制 thread 讀取）
live_K       = 1.0
live_B       = 0.1
live_M       = 0.05
live_Kp      = 20.0
live_Kd      = 0.5
live_theta_d = 0.0

# ═══════════════════════════════════════════════════════════════════════════════
# LOW-PASS FILTER
# ═══════════════════════════════════════════════════════════════════════════════

class LowPassFilter:
    """一階 IIR 低通濾波器  y[k] = α·y[k-1] + (1-α)·x[k]"""
    def __init__(self, fc, dt):
        # [v3] 防範 dt=0 造成 alpha=1（輸出永遠為 0）
        dt_safe = max(dt, 1e-9)
        self._alpha = math.exp(-2 * math.pi * fc * dt_safe)
        self._y = 0.0

    def reset(self, value=0.0):
        self._y = value

    def update(self, x):
        self._y = self._alpha * self._y + (1 - self._alpha) * x
        return self._y

# ═══════════════════════════════════════════════════════════════════════════════
# IMPEDANCE DYNAMICS  ← [v3] 新增：正確的阻抗模型
# ═══════════════════════════════════════════════════════════════════════════════

class ImpedanceDynamics:
    """
    離散積分阻抗動力學：M·ẍ + B·ẋ + K·x = F_ext

    x      = theta_cmd - theta_d（阻抗位移）
    theta_cmd = theta_d + x

    M=0 時退化為靜態彈簧：x = F_ext / K
    """
    def __init__(self):
        self._x    = 0.0   # 阻抗位移
        self._xdot = 0.0   # 阻抗速度

    def reset(self):
        self._x    = 0.0
        self._xdot = 0.0

    def update(self, F_ext, K, B, M, dt):
        K_safe = max(K, 0.01)

        if M < 1e-6:
            # 靜態彈簧退化模式
            self._x    = F_ext / K_safe
            self._xdot = 0.0
            return self._x

        # 顯式 Euler 積分
        xddot      = (F_ext - B * self._xdot - K_safe * self._x) / M
        self._xdot += xddot * dt
        self._x    += self._xdot * dt
        return self._x

    @property
    def displacement(self):
        return self._x

    @property
    def velocity(self):
        return self._xdot

# ═══════════════════════════════════════════════════════════════════════════════
# SAFETY CHECKER
# ═══════════════════════════════════════════════════════════════════════════════

class SafetyChecker:
    """
    每個控制週期呼叫一次 check()。
    回傳 (safe: bool, reason: str)。

    [v3] 修正：每週期只呼叫一次，_cycle 計數器不再雙重遞增。
    [v3] 外力上限改為合理值（FORCE_EST_LIMIT）。
    """
    def __init__(self, log_cb):
        self._log       = log_cb
        self._triggered = False
        self._cycle     = 0

    def check(self, theta, omega, voltage, current, force_est):
        if self._triggered:
            return False, "緊急停止已觸發"

        self._cycle += 1
        in_warmup = self._cycle <= WARMUP_CYCLES

        # 角度 / 速度：暖機期也要檢查
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

        # 外力超限：只警告歸零，不停止
        if not in_warmup and abs(force_est) > FORCE_EST_LIMIT:
            self._log(
                f"[WARN] 外力估算異常 {force_est:.4f} N·m "
                f"(限制 {FORCE_EST_LIMIT} N·m)，本週期外力歸零")
            return True, "force_zero"

        return True, ""

    def _trigger(self, msg):
        self._triggered = True
        _emergency_event.set()
        self._log(f"[EMERGENCY] {msg}")
        return False, msg

    def reset(self):
        self._triggered = False
        self._cycle     = 0
        _emergency_event.clear()

# ═══════════════════════════════════════════════════════════════════════════════
# QUBE IMPORT
# ═══════════════════════════════════════════════════════════════════════════════

def try_import_qube():
    try:
        from pal.products.qube import QubeServo3
        return QubeServo3
    except ImportError:
        return None

# ═══════════════════════════════════════════════════════════════════════════════
# DATA HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

# [v3] 固定欄位名稱，不再用動態 key
_ROW_FIELDS = [
    "round", "time",
    "theta_rad", "theta_d_rad", "theta_cmd_rad",
    "omega_rads", "voltage_V", "current_A", "force_est_Nm",
    "K_Nm_rad", "B_Nms_rad", "M_kgm2", "Kp", "Kd",
]

def _make_row(rnd, t, theta, theta_d, theta_cmd, omega, voltage,
              current, force_est, K, B, M, Kp, Kd):
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

def _clear_plot_buffers():
    time_buf.clear(); pos_buf.clear(); desired_buf.clear()
    voltage_buf.clear(); speed_buf.clear()
    force_est_buf.clear(); cmd_pos_buf.clear()

def _push_buffers(t, theta, theta_d, theta_cmd, omega, voltage, force_est):
    time_buf.append(t)
    pos_buf.append(theta)
    desired_buf.append(theta_d)
    cmd_pos_buf.append(theta_cmd)
    voltage_buf.append(voltage)
    speed_buf.append(omega)
    force_est_buf.append(force_est)

# ═══════════════════════════════════════════════════════════════════════════════
# IMPEDANCE CONTROL LOOP
# ═══════════════════════════════════════════════════════════════════════════════

def control_loop(params, log_cb, status_cb, round_done_cb,
                 all_done_cb, new_round_cb, safety_alert_cb):
    global all_rounds_history, round_history, round_counter

    dt           = params["sample_time"]
    total_rounds = params["total_rounds"]
    exp_time     = params["exp_time"]
    hardware     = params["hardware"]

    safety    = SafetyChecker(log_cb)
    imp_dyn   = ImpedanceDynamics()

    def make_filters():
        return (LowPassFilter(FC_SPEED, dt),
                LowPassFilter(FC_ACCEL, dt),
                LowPassFilter(FC_FORCE, dt))

    QubeServo3 = try_import_qube()

    if hardware == 1 and QubeServo3 is None:
        msg = "找不到 pal 套件，請切換為虛擬模式。"
        log_cb(f"[ERROR] {msg}")
        status_cb(False, "None", msg)
        all_done_cb(); return

    log_cb(f"[DEBUG] exp_time={exp_time}s  dt={dt}s  rounds={total_rounds}  "
           f"hardware={hardware}  WARMUP_CYCLES={WARMUP_CYCLES}")
    log_cb(f"[DEBUG] CURRENT_LIMIT={CURRENT_LIMIT}A  "
           f"FORCE_EST_LIMIT={FORCE_EST_LIMIT}N·m")
    log_cb(f"[INFO] 模式：{'實體 QubeServo3' if hardware == 1 else '虛擬模擬'} | "
           f"共 {total_rounds} 次實驗")

    # ── 實體模式 ──────────────────────────────────────────────────────────────
    if hardware == 1:
        try:
            device_ctx = QubeServo3(hardware=1, pendulum=0, readMode=0)
        except Exception as e:
            msg = f"實體連線失敗：{e}"
            log_cb(f"[ERROR] {msg}")
            status_cb(False, "None", msg)
            all_done_cb(); return

        status_cb(True, "QubeServo3", "")

        with device_ctx as qube:
            _has_current = hasattr(qube, 'motorCurrent')
            if not _has_current:
                log_cb("[WARN] qube.motorCurrent 屬性不存在，電流固定設為 0.0A，"
                       "外力估算將停用")

            for rnd_idx in range(total_rounds):
                if _kill_event.is_set() or _emergency_event.is_set():
                    break
                safety.reset()
                imp_dyn.reset()

                clear_event = threading.Event()
                new_round_cb(rnd_idx + 1, clear_event)
                clear_event.wait(timeout=2.0)

                spd_f, acc_f, frc_f = make_filters()

                # [v3] 速度估算用局部變數，不依賴 pos_buf
                prev_theta = None
                prev_omega = 0.0

                # [v3] round_history 在鎖內重置
                with data_lock:
                    round_history.clear()

                startTime = time.time()
                timeStamp = 0.0

                log_cb(f"[ROUND {rnd_idx+1}/{total_rounds}] 開始 | "
                       f"K={live_K:.3f}  B={live_B:.4f}  M={live_M:.4f}  "
                       f"暖機週期={WARMUP_CYCLES}")

                while (timeStamp < exp_time
                       and not _kill_event.is_set()
                       and not _emergency_event.is_set()):

                    while _pause_event.is_set() and not _kill_event.is_set():
                        time.sleep(0.05)

                    t0 = time.time()

                    try:
                        qube.read_outputs()
                        theta   = float(np.asarray(qube.motorPosition).flat[0])
                        current = (float(np.asarray(qube.motorCurrent).flat[0])
                                   if _has_current else 0.0)

                        # [v3] 速度估算：使用局部 prev_theta
                        if prev_theta is None:
                            prev_theta = theta
                        omega_raw  = (theta - prev_theta) / dt
                        prev_theta = theta

                        omega     = spd_f.update(omega_raw)
                        alpha_raw = (omega - prev_omega) / dt
                        alpha     = acc_f.update(alpha_raw)
                        prev_omega = omega

                        tau_motor = Kt * current
                        tau_model = J_motor * alpha + B_friction * omega
                        force_raw = tau_motor - tau_model
                        force_est = frc_f.update(force_raw)

                        # [v3] 安全檢查每週期只呼叫一次（修正雙重計數 bug）
                        safe, reason = safety.check(
                            theta, omega, 0.0, current, force_est)
                        if not safe:
                            qube.write_voltage(0.0)
                            safety_alert_cb(reason)
                            break
                        if reason == "force_zero":
                            force_est = 0.0

                        K = live_K; B = live_B; M = live_M
                        Kp = live_Kp; Kd = live_Kd
                        theta_d = live_theta_d

                        # [v3] 完整阻抗動力學（M / B 現在真正有作用）
                        x_imp     = imp_dyn.update(force_est, K, B, M, dt)
                        theta_cmd = theta_d + x_imp

                        error_pos   = theta_cmd - theta
                        error_vel   = imp_dyn.velocity - omega
                        voltage_raw = Kp * error_pos + Kd * error_vel
                        voltage     = float(np.clip(voltage_raw,
                                                    -VOLTAGE_LIMIT, VOLTAGE_LIMIT))

                        qube.write_voltage(voltage)

                        row = _make_row(rnd_idx+1, timeStamp, theta, theta_d,
                                        theta_cmd, omega, voltage,
                                        current, force_est, K, B, M, Kp, Kd)
                        with data_lock:
                            _push_buffers(timeStamp, theta, theta_d,
                                          theta_cmd, omega, voltage, force_est)
                            round_history.append(row)

                    except Exception as e:
                        log_cb(f"[ERROR] 控制迴圈例外：{type(e).__name__}: {e}")
                        try:
                            qube.write_voltage(0.0)
                        except Exception:
                            pass
                        safety_alert_cb(f"控制迴圈例外：{type(e).__name__}: {e}")
                        break

                    elapsed = time.time() - t0
                    time.sleep(max(0.0, dt - elapsed))
                    timeStamp = time.time() - startTime

                qube.write_voltage(0.0)
                if not _kill_event.is_set() and not _emergency_event.is_set():
                    round_counter += 1
                    with data_lock:
                        all_rounds_history.append(list(round_history))
                    log_cb(f"[ROUND {round_counter}] 完成")
                    round_done_cb(list(round_history), round_counter)

        status_cb(False, "None", "")
        log_cb("[INFO] 控制迴圈結束（實體）")
        all_done_cb()
        return

    # ── 虛擬模式 ──────────────────────────────────────────────────────────────
    status_cb(True, "虛擬模擬", "")
    Km_sim  = 0.042; J_sim = 4e-6; B_sim = 1e-5
    sim_theta = 0.0; sim_omega = 0.0

    for rnd_idx in range(total_rounds):
        if _kill_event.is_set() or _emergency_event.is_set():
            break
        safety.reset()
        imp_dyn.reset()

        clear_event = threading.Event()
        new_round_cb(rnd_idx + 1, clear_event)
        clear_event.wait(timeout=2.0)

        spd_f, acc_f, frc_f = make_filters()

        # [v3] 速度估算用局部變數
        prev_theta = None
        prev_omega = 0.0

        with data_lock:
            round_history.clear()

        sim_theta    = 0.0
        sim_omega    = 0.0
        last_voltage = 0.0

        startTime = time.time()
        timeStamp = 0.0

        log_cb(f"[ROUND {rnd_idx+1}/{total_rounds}] 開始（虛擬）| "
               f"K={live_K:.3f}  B={live_B:.4f}  M={live_M:.4f}  "
               f"暖機週期={WARMUP_CYCLES}")

        while (timeStamp < exp_time
               and not _kill_event.is_set()
               and not _emergency_event.is_set()):

            while _pause_event.is_set() and not _kill_event.is_set():
                time.sleep(0.05)

            t0    = time.time()
            theta = sim_theta

            # [v3] 虛擬電流：用具名常數 SIM_VOLT_TO_CURR
            current = last_voltage * SIM_VOLT_TO_CURR

            # [v3] 速度估算：使用局部 prev_theta
            if prev_theta is None:
                prev_theta = theta
            omega_raw  = (theta - prev_theta) / dt
            prev_theta = theta

            omega     = spd_f.update(omega_raw)
            alpha_raw = (omega - prev_omega) / dt
            alpha     = acc_f.update(alpha_raw)
            prev_omega = omega

            tau_motor = Kt * current
            tau_model = J_motor * alpha + B_friction * omega
            force_raw = tau_motor - tau_model
            force_est = frc_f.update(force_raw)

            # [v3] 安全檢查每週期只呼叫一次
            safe, reason = safety.check(
                theta, omega, 0.0, current, force_est)
            if not safe:
                safety_alert_cb(reason); break
            if reason == "force_zero":
                force_est = 0.0

            K = live_K; B = live_B; M = live_M
            Kp = live_Kp; Kd = live_Kd
            theta_d = live_theta_d

            # [v3] 完整阻抗動力學
            x_imp     = imp_dyn.update(force_est, K, B, M, dt)
            theta_cmd = theta_d + x_imp

            error_pos   = theta_cmd - theta
            error_vel   = imp_dyn.velocity - omega
            voltage_raw = Kp * error_pos + Kd * error_vel
            voltage     = float(np.clip(voltage_raw, -VOLTAGE_LIMIT, VOLTAGE_LIMIT))

            # 虛擬馬達動力學
            torque    = Km_sim * voltage - B_sim * sim_omega
            accel_sim = torque / max(J_sim, 1e-9)
            sim_omega += accel_sim * dt
            sim_theta += sim_omega * dt
            last_voltage = voltage

            row = _make_row(rnd_idx+1, timeStamp, theta, theta_d,
                            theta_cmd, omega, voltage,
                            current, force_est, K, B, M, Kp, Kd)
            with data_lock:
                _push_buffers(timeStamp, theta, theta_d,
                              theta_cmd, omega, voltage, force_est)
                round_history.append(row)

            elapsed = time.time() - t0
            time.sleep(max(0.0, dt - elapsed))
            timeStamp = time.time() - startTime

        if not _kill_event.is_set() and not _emergency_event.is_set():
            round_counter += 1
            with data_lock:
                all_rounds_history.append(list(round_history))
            log_cb(f"[ROUND {round_counter}] 完成（虛擬）")
            round_done_cb(list(round_history), round_counter)

    status_cb(False, "None", "")
    log_cb("[INFO] 控制迴圈結束（虛擬）")
    all_done_cb()

# ═══════════════════════════════════════════════════════════════════════════════
# PARAM WIDGET
# ═══════════════════════════════════════════════════════════════════════════════

class ParamWidget(tk.Frame):
    BG       = "#0b0f14"; ENTRY_BG = "#060a0f"
    ACCENT   = "#00c8ff"; DANGER   = "#ff4d6a"
    SUBTEXT  = "#7a8899"; TEXT     = "#dde6f0"; BORDER = "#1e2a38"

    def __init__(self, parent, label, var, from_, to,
                 resolution=0.001, unit="", fmt="{:.3f}",
                 slider_length=110, **kw):
        super().__init__(parent, bg=self.BG, **kw)
        self._var = var; self._from = from_; self._to = to
        self._res = resolution; self._fmt = fmt; self._updating = False

        tk.Label(self, text=label, font=("Consolas", 9), bg=self.BG,
                 fg=self.SUBTEXT, width=22, anchor="w").pack(side="left")
        self._slider = tk.Scale(
            self, variable=var, from_=from_, to=to, resolution=resolution,
            orient="horizontal", showvalue=False, bg=self.BG, fg=self.TEXT,
            troughcolor="#060a0f", highlightthickness=0, sliderrelief="flat",
            activebackground=self.ACCENT, command=self._on_slider,
            length=slider_length)
        self._slider.pack(side="left", padx=(4, 6))
        self._entry_var = tk.StringVar(value=fmt.format(var.get()))
        self._entry = tk.Entry(
            self, textvariable=self._entry_var, width=9,
            font=("Consolas", 10), bg=self.ENTRY_BG, fg=self.TEXT,
            insertbackground=self.TEXT, relief="flat", highlightthickness=1,
            highlightbackground=self.BORDER, highlightcolor=self.ACCENT)
        self._entry.pack(side="left")
        for ev in ("<Return>", "<KP_Enter>", "<FocusOut>", "<Tab>"):
            self._entry.bind(ev, self._on_entry_commit)
        if unit:
            tk.Label(self, text=unit, font=("Consolas", 9), bg=self.BG,
                     fg=self.SUBTEXT, width=7,
                     anchor="w").pack(side="left", padx=(4, 0))

    def _on_slider(self, val):
        if self._updating: return
        self._updating = True
        self._entry_var.set(self._fmt.format(float(val)))
        self._flash_normal(); self._updating = False

    def _on_entry_commit(self, event=None):
        if self._updating: return
        raw = self._entry_var.get().strip()
        try:
            value = float(raw)
        except ValueError:
            self._flash_error()
            self._entry_var.set(self._fmt.format(self._var.get()))
            return
        clamped = max(self._from, min(self._to, value))
        self._updating = True
        self._var.set(round(clamped / self._res) * self._res)
        self._entry_var.set(self._fmt.format(clamped))
        self._flash_normal(); self._updating = False

    def _flash_error(self):
        self._entry.config(highlightbackground=self.DANGER,
                           highlightcolor=self.DANGER)
        self.after(800, self._flash_normal)

    def _flash_normal(self):
        self._entry.config(highlightbackground=self.BORDER,
                           highlightcolor=self.ACCENT)

    def get(self): return self._var.get()

# ═══════════════════════════════════════════════════════════════════════════════
# MAIN GUI
# ═══════════════════════════════════════════════════════════════════════════════

class ImpedanceControlPanel(tk.Tk):

    BG      = "#0b0f14"; PANEL   = "#111820"
    BORDER  = "#1e2a38"; ACCENT  = "#00c8ff"; ACCENT2 = "#00e5a0"
    WARN    = "#f0a800"; DANGER  = "#ff4d6a"; PURPLE  = "#a78bfa"
    TEXT    = "#dde6f0"; SUBTEXT = "#7a8899"; GRID_C  = "#141e28"
    ORANGE  = "#ff7b42"

    CLR_POS  = "#00c8ff"
    CLR_DES  = "#00e5a0"
    CLR_CMD  = "#f0a800"
    CLR_SPD  = "#a78bfa"
    CLR_FRC  = "#ff7b42"
    CLR_VOLT = "#ff4d6a"

    FONT_MONO = ("Consolas", 10)
    FONT_BODY = ("Segoe UI", 10)
    FONT_H2   = ("Segoe UI", 11, "bold")
    FONT_H1   = ("Segoe UI", 13, "bold")

    def __init__(self):
        super().__init__()
        self.title("QUBE-Servo 3 — 阻抗控制  v3")
        self.configure(bg=self.BG)
        self.geometry("1720x1020")
        self.minsize(1400, 850)
        self._emergency_shown = False
        self._build_ui()
        self._start_plot_animation()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Layout ────────────────────────────────────────────────────────────────

    def _build_ui(self):
        top = tk.Frame(self, bg=self.BG, pady=5)
        top.pack(fill="x", padx=16)
        tf = tk.Frame(top, bg=self.BG); tf.pack(side="left")
        tk.Label(tf, text="◈  QUBE-Servo 3",
                 font=("Consolas", 16, "bold"),
                 bg=self.BG, fg=self.ACCENT).pack(side="left")
        tk.Label(tf, text="  阻抗控制",
                 font=("Segoe UI", 16, "bold"),
                 bg=self.BG, fg=self.TEXT).pack(side="left")
        tk.Label(tf, text="  v3",
                 font=("Consolas", 11),
                 bg=self.BG, fg=self.SUBTEXT).pack(side="left")

        rt = tk.Frame(top, bg=self.BG); rt.pack(side="right")
        self._clock_var = tk.StringVar()
        tk.Label(rt, textvariable=self._clock_var, font=("Consolas", 11),
                 bg=self.BG, fg=self.SUBTEXT).pack(side="right", padx=8)
        self._status_cv = tk.Canvas(rt, width=12, height=12,
                                    bg=self.BG, highlightthickness=0)
        self._status_cv.pack(side="right", padx=(0, 4))
        self._led = self._status_cv.create_oval(1, 1, 11, 11,
                                                fill=self.DANGER, outline="")
        self._status_text = tk.StringVar(value="未連線")
        tk.Label(rt, textvariable=self._status_text, font=self.FONT_BODY,
                 bg=self.BG, fg=self.TEXT).pack(side="right", padx=4)
        self._tick_clock()

        self._emergency_banner = tk.Frame(self, bg=self.DANGER, pady=4)
        self._emergency_msg = tk.StringVar(value="")
        tk.Label(self._emergency_banner, textvariable=self._emergency_msg,
                 font=("Consolas", 11, "bold"),
                 bg=self.DANGER, fg="#ffffff").pack()

        tk.Frame(self, bg=self.BORDER, height=1).pack(fill="x")

        body = tk.Frame(self, bg=self.BG)
        body.pack(fill="both", expand=True, padx=10, pady=6)

        lo = tk.Frame(body, bg=self.BG, width=360)
        lo.pack(side="left", fill="y", padx=(0, 6))
        lo.pack_propagate(False)
        lc = tk.Canvas(lo, bg=self.BG, highlightthickness=0, width=350)
        ls = ttk.Scrollbar(lo, orient="vertical", command=lc.yview)
        lc.configure(yscrollcommand=ls.set)
        ls.pack(side="right", fill="y")
        lc.pack(side="left", fill="both", expand=True)
        left = tk.Frame(lc, bg=self.BG)
        lw = lc.create_window((0, 0), window=left, anchor="nw")

        def _cfg(e):
            lc.configure(scrollregion=lc.bbox("all"))
            lc.itemconfig(lw, width=lc.winfo_width())
        left.bind("<Configure>", _cfg)
        lc.bind_all("<MouseWheel>",
                    lambda e: lc.yview_scroll(int(-1*(e.delta/120)), "units"))
        lc.bind_all("<Button-4>", lambda e: lc.yview_scroll(-1, "units"))
        lc.bind_all("<Button-5>", lambda e: lc.yview_scroll(1, "units"))

        centre = tk.Frame(body, bg=self.BG)
        centre.pack(side="left", fill="both", expand=True)
        right = tk.Frame(body, bg=self.BG, width=300)
        right.pack(side="right", fill="y", padx=(6, 0))
        right.pack_propagate(False)

        self._build_left(left)
        self._build_centre(centre)
        self._build_right(right)

    # ── Left panel ────────────────────────────────────────────────────────────

    def _build_left(self, parent):
        c = self._card(parent, "⚙  硬體設定")
        self._hw_mode = self._labeled_combo(
            c, "硬體模式", ["0 — 虛擬", "1 — 實體"], "1 — 實體")
        self._device_var = tk.StringVar(value="裝置：—")
        tk.Label(c, textvariable=self._device_var, font=self.FONT_MONO,
                 bg=self.PANEL, fg=self.SUBTEXT).pack(anchor="w", pady=(4, 0))
        self._error_var = tk.StringVar(value="")
        tk.Label(c, textvariable=self._error_var, font=("Consolas", 9),
                 bg=self.PANEL, fg=self.DANGER,
                 wraplength=320, justify="left").pack(anchor="w")

        cp = self._card(parent, "🔧  馬達參數（常數）")
        for lbl, val, unit in [
            ("Kt  力矩常數",  f"{Kt:.4f}",       "N·m/A"),
            ("J   轉子慣量",  f"{J_motor:.2e}",   "kg·m²"),
            ("Bf  黏性摩擦",  f"{B_friction:.2e}", "N·m·s/rad"),
        ]:
            row = tk.Frame(cp, bg=self.PANEL); row.pack(fill="x", pady=2)
            tk.Label(row, text=lbl, font=("Consolas", 9),
                     bg=self.PANEL, fg=self.SUBTEXT,
                     width=16, anchor="w").pack(side="left")
            tk.Label(row, text=val, font=("Consolas", 10, "bold"),
                     bg=self.PANEL, fg=self.ACCENT).pack(side="left", padx=4)
            tk.Label(row, text=unit, font=("Consolas", 9),
                     bg=self.PANEL, fg=self.SUBTEXT).pack(side="left")
        tk.Label(cp,
                 text="※ 修改請直接編輯程式頂部 MOTOR PARAMETERS",
                 font=("Consolas", 8), bg=self.PANEL,
                 fg=self.SUBTEXT, wraplength=320,
                 justify="left").pack(anchor="w", pady=(4, 0))

        ci = self._card(parent, "🎛  阻抗參數  （即時生效）")
        self._K_var = tk.DoubleVar(value=1.0)
        self._B_var = tk.DoubleVar(value=0.1)
        self._M_var = tk.DoubleVar(value=0.05)

        ParamWidget(ci, "K  剛性", self._K_var,
                    0.0, 5.0, 0.01, unit="N·m/rad",
                    fmt="{:.3f}", slider_length=95).pack(fill="x", pady=3)
        ParamWidget(ci, "B  阻尼", self._B_var,
                    0.0, 1.0, 0.001, unit="N·m·s/rad",
                    fmt="{:.3f}", slider_length=95).pack(fill="x", pady=3)
        ParamWidget(ci, "M  虛擬慣量", self._M_var,
                    0.0, 0.5, 0.001, unit="kg·m²",
                    fmt="{:.3f}", slider_length=95).pack(fill="x", pady=3)

        self._stability_var = tk.StringVar(value="")
        tk.Label(ci, textvariable=self._stability_var,
                 font=("Consolas", 9), bg=self.PANEL,
                 fg=self.WARN, wraplength=320,
                 justify="left").pack(anchor="w", pady=(2, 0))

        for v in [self._K_var, self._B_var, self._M_var]:
            v.trace_add("write", lambda *a: self._push_live_impedance())

        cpd = self._card(parent, "⚡  內迴路 PD 控制器")
        self._Kp_var = tk.DoubleVar(value=20.0)
        self._Kd_var = tk.DoubleVar(value=0.5)
        ParamWidget(cpd, "Kp  比例增益", self._Kp_var,
                    0.0, 50.0, 0.5, fmt="{:.1f}",
                    slider_length=95).pack(fill="x", pady=3)
        ParamWidget(cpd, "Kd  微分增益", self._Kd_var,
                    0.0, 5.0, 0.05, fmt="{:.2f}",
                    slider_length=95).pack(fill="x", pady=3)
        for v in [self._Kp_var, self._Kd_var]:
            v.trace_add("write", lambda *a: self._push_live_pd())

        cth = self._card(parent, "🎯  目標角度")
        self._theta_d_var = tk.DoubleVar(value=0.0)
        ParamWidget(cth, "θ_d  目標角度", self._theta_d_var,
                    -math.pi, math.pi, 0.01,
                    unit="rad", fmt="{:.3f}",
                    slider_length=95).pack(fill="x", pady=3)
        self._theta_d_var.trace_add("write",
                                    lambda *a: self._push_live_theta_d())

        ce = self._card(parent, "🔁  實驗設定")
        self._exp_time_var     = tk.DoubleVar(value=30.0)
        self._sample_time_var  = tk.DoubleVar(value=0.002)
        self._total_rounds_var = tk.IntVar(value=1)
        ParamWidget(ce, "實驗時長", self._exp_time_var,
                    5.0, 120.0, 1.0, unit="s", fmt="{:.0f}",
                    slider_length=95).pack(fill="x", pady=3)
        ParamWidget(ce, "取樣時間", self._sample_time_var,
                    0.001, 0.01, 0.001, unit="s", fmt="{:.3f}",
                    slider_length=95).pack(fill="x", pady=3)
        rr = tk.Frame(ce, bg=self.PANEL); rr.pack(fill="x", pady=3)
        tk.Label(rr, text="實驗次數", width=12, anchor="w",
                 font=self.FONT_BODY, bg=self.PANEL,
                 fg=self.SUBTEXT).pack(side="left")
        tk.Spinbox(rr, textvariable=self._total_rounds_var,
                   from_=1, to=20, width=5, font=self.FONT_MONO,
                   bg="#060a0f", fg=self.TEXT,
                   buttonbackground=self.BORDER, relief="flat",
                   highlightthickness=1, highlightbackground=self.BORDER,
                   highlightcolor=self.ACCENT).pack(side="left", padx=4)
        tk.Label(rr, text="次", font=self.FONT_BODY,
                 bg=self.PANEL, fg=self.SUBTEXT).pack(side="left")
        self._round_var = tk.StringVar(value="進度：—")
        tk.Label(ce, textvariable=self._round_var,
                 font=("Consolas", 10, "bold"),
                 bg=self.PANEL, fg=self.ACCENT2).pack(anchor="w", pady=(4, 0))

        cs = self._card(parent, "🛡  安全限制（固定）")
        for lbl, val in [
            ("角度限制",  f"±{math.degrees(ANGLE_LIMIT_RAD):.0f}°"),
            ("速度限制",  f"±{SPEED_LIMIT_RADS:.0f} rad/s"),
            ("電壓限制",  f"±{VOLTAGE_LIMIT:.0f} V"),
            ("電流異常",  f">{CURRENT_LIMIT:.1f} A"),
            ("外力異常",  f">{FORCE_EST_LIMIT:.2f} N·m"),
        ]:
            row = tk.Frame(cs, bg=self.PANEL); row.pack(fill="x", pady=1)
            tk.Label(row, text=lbl, font=("Consolas", 9),
                     bg=self.PANEL, fg=self.SUBTEXT,
                     width=12, anchor="w").pack(side="left")
            tk.Label(row, text=val, font=("Consolas", 9, "bold"),
                     bg=self.PANEL, fg=self.WARN).pack(side="left")

        cb = self._card(parent, "")
        self._start_btn = self._btn(
            cb, "▶  開始實驗", self._start_control, self.ACCENT2, "#000000")
        self._pause_btn = self._btn(
            cb, "⏸  暫停", self._pause_control, self.WARN, "#000000")
        self._pause_btn.config(state="disabled")
        self._stop_btn = self._btn(
            cb, "■  停止", self._stop_control, self.DANGER, "#ffffff")
        self._stop_btn.config(state="disabled")
        self._reset_btn = self._btn(
            cb, "🔄  解除緊急停止", self._reset_emergency,
            self.ORANGE, "#000000")
        self._reset_btn.config(state="disabled")
        tk.Frame(cb, bg=self.BORDER, height=1).pack(fill="x", pady=6)
        self._btn(cb, "⬇  儲存 Excel（全部輪次）",
                  self._download_excel_all, self.ACCENT, "#000000")
        self._btn(cb, "⬇  儲存 CSV（最後一輪）",
                  self._download_csv_last, self.PURPLE, "#ffffff")
        self._btn(cb, "🗑  清除所有資料",
                  self._clear_data, self.DANGER, "#ffffff")

    # ── Centre panel ──────────────────────────────────────────────────────────

    def _build_centre(self, parent):
        hdr = tk.Frame(parent, bg=self.BG); hdr.pack(fill="x", pady=(0, 4))
        tk.Label(hdr, text="📊  即時波形（當前輪）",
                 font=self.FONT_H1, bg=self.BG, fg=self.TEXT).pack(side="left")
        self._plot_round_var = tk.StringVar(value="")
        tk.Label(hdr, textvariable=self._plot_round_var,
                 font=("Consolas", 10, "bold"),
                 bg=self.BG, fg=self.ACCENT2).pack(side="left", padx=10)

        tf = tk.Frame(hdr, bg=self.BG); tf.pack(side="right", padx=4)
        self._show_vars = {}
        for lbl, key, clr in [
            ("實際角度", "pos",  self.CLR_POS),
            ("修正目標", "cmd",  self.CLR_CMD),
            ("角速度",   "spd",  self.CLR_SPD),
            ("外力估算", "frc",  self.CLR_FRC),
            ("電壓",     "volt", self.CLR_VOLT),
        ]:
            v = tk.BooleanVar(value=True)
            tk.Checkbutton(tf, text=lbl, variable=v,
                           bg=self.BG, fg=clr, selectcolor=self.BG,
                           activebackground=self.BG, activeforeground=clr,
                           font=("Consolas", 9), cursor="hand2").pack(
                side="left", padx=2)
            self._show_vars[key] = v

        fig_bg = "#0b0f14"; ax_bg = "#0e151d"
        self._fig, axes = plt.subplots(5, 1, figsize=(7, 9.5),
                                       facecolor=fig_bg)
        self._fig.subplots_adjust(left=0.10, right=0.985,
                                  top=0.985, bottom=0.04, hspace=0.48)

        plot_cfg = [
            ("角度追蹤 (rad)",     ax_bg, "rad"),
            ("控制電壓 (V)",       ax_bg, "V"),
            ("角速度 (rad/s)",     ax_bg, "rad/s"),
            ("外力估算 (N·m)",     ax_bg, "N·m"),
            ("修正目標角度 (rad)", ax_bg, "rad"),
        ]
        self._axes = axes
        for ax, (title, bg, ylab) in zip(axes, plot_cfg):
            ax.set_facecolor(bg)
            ax.set_title(title, color=self.TEXT, fontsize=8.5,
                         pad=3, loc="left")
            ax.set_ylabel(ylab, color=self.SUBTEXT, fontsize=7.5)
            ax.tick_params(colors=self.SUBTEXT, labelsize=7, length=2)
            for sp in ax.spines.values(): sp.set_edgecolor(self.BORDER)
            ax.grid(True, color=self.GRID_C, linewidth=0.6, linestyle="-")
            ax.set_xlim(0, 1); ax.set_ylim(-1, 1)

        ln_pos,   = axes[0].plot([], [], color=self.CLR_POS,
                                 lw=1.4, label="實際 θ")
        ln_des,   = axes[0].plot([], [], color=self.CLR_DES,
                                 lw=1.1, ls="--", alpha=0.7, label="目標 θ_d")
        ln_cmd_0, = axes[0].plot([], [], color=self.CLR_CMD,
                                 lw=1.1, ls=":", alpha=0.8, label="修正 θ_cmd")
        axes[0].legend(fontsize=7, facecolor=ax_bg, edgecolor=self.BORDER,
                       labelcolor=self.TEXT, loc="upper right", framealpha=0.8)

        ln_volt, = axes[1].plot([], [], color=self.CLR_VOLT, lw=1.3)
        ln_spd,  = axes[2].plot([], [], color=self.CLR_SPD,  lw=1.3)
        ln_frc,  = axes[3].plot([], [], color=self.CLR_FRC,  lw=1.3)
        ln_cmd,  = axes[4].plot([], [], color=self.CLR_CMD,  lw=1.3)

        self._lines = [ln_pos, ln_des, ln_cmd_0,
                       ln_volt, ln_spd, ln_frc, ln_cmd]

        self._hlines = []
        for ax, clr in zip(axes, [self.CLR_POS, self.CLR_VOLT,
                                   self.CLR_SPD, self.CLR_FRC, self.CLR_CMD]):
            hl = ax.axhline(y=0, color=clr, lw=0.6, ls=":", alpha=0.5)
            self._hlines.append(hl)

        canvas = FigureCanvasTkAgg(self._fig, master=parent)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas = canvas

    # ── Right panel ───────────────────────────────────────────────────────────

    def _build_right(self, parent):
        vc = self._card(parent, "📈  即時數值")
        self._stat_vars = {}
        for key, lbl, unit, clr in [
            ("theta", "實際角度", "rad",   self.CLR_POS),
            ("omega", "角速度",   "rad/s", self.CLR_SPD),
            ("force", "外力估算", "N·m",   self.CLR_FRC),
            ("volt",  "控制電壓", "V",     self.CLR_VOLT),
            ("cmd",   "修正目標", "rad",   self.CLR_CMD),
            ("n",     "樣本數",   "",      self.SUBTEXT),
        ]:
            row = tk.Frame(vc, bg=self.PANEL); row.pack(fill="x", pady=2)
            tk.Label(row, text=lbl, font=("Consolas", 9),
                     bg=self.PANEL, fg=self.SUBTEXT,
                     width=10, anchor="w").pack(side="left")
            sv = tk.StringVar(value="—")
            tk.Label(row, textvariable=sv,
                     font=("Consolas", 12, "bold"),
                     bg=self.PANEL, fg=clr).pack(side="left", padx=4)
            if unit:
                tk.Label(row, text=unit, font=("Consolas", 8),
                         bg=self.PANEL, fg=self.SUBTEXT).pack(side="left")
            self._stat_vars[key] = sv

        self._alert_label = tk.Label(
            vc, text="", font=("Consolas", 9, "bold"),
            bg=self.PANEL, fg=self.DANGER,
            wraplength=280, justify="left")
        self._alert_label.pack(anchor="w", pady=3)

        lc = self._card(parent, "📋  系統日誌")
        lc.pack_configure(fill="both", expand=True)
        self._log_text = tk.Text(
            lc, bg="#060a0f", fg=self.SUBTEXT,
            font=("Consolas", 9), height=18, wrap="word",
            state="disabled", relief="flat",
            insertbackground=self.TEXT)
        self._log_text.pack(fill="both", expand=True, side="left")
        scr = ttk.Scrollbar(lc, command=self._log_text.yview)
        self._log_text.config(yscrollcommand=scr.set)
        scr.pack(side="right", fill="y")
        self._btn(parent, "🗑  清除日誌",
                  self._clear_log, self.BORDER, self.TEXT)

    # ── Plot animation ────────────────────────────────────────────────────────

    def _start_plot_animation(self):
        self._canvas.draw()
        self._schedule_plot()

    def _schedule_plot(self):
        self._update_plots()
        self.after(PLOT_INTERVAL_MS, self._schedule_plot)

    def _update_plots(self):
        global _last_render_len
        with data_lock:
            n = len(time_buf)
            if n == 0:
                return
            if n == _last_render_len:
                self._update_stats_only(); return
            _last_render_len = n
            step = max(1, n // PLOT_DOWNSAMPLE_PTS)
            ts   = list(time_buf)[::step]
            pos  = list(pos_buf)[::step]
            des  = list(desired_buf)[::step]
            cmd  = list(cmd_pos_buf)[::step]
            volt = list(voltage_buf)[::step]
            spd  = list(speed_buf)[::step]
            frc  = list(force_est_buf)[::step]

        vis = {k: v.get() for k, v in self._show_vars.items()}

        def _set(ln, arr, visible):
            if visible and ts:
                ln.set_data(ts, arr); ln.set_visible(True)
            else:
                ln.set_visible(False)

        _set(self._lines[0], pos,  vis["pos"])
        _set(self._lines[1], des,  vis["pos"])
        _set(self._lines[2], cmd,  vis["cmd"])
        _set(self._lines[3], volt, vis["volt"])
        _set(self._lines[4], spd,  vis["spd"])
        _set(self._lines[5], frc,  vis["frc"])
        _set(self._lines[6], cmd,  vis["cmd"])

        if not ts: return
        x0, x1 = ts[0], ts[-1] + 0.01

        for ax, arr in zip(self._axes, [
            np.concatenate([pos, des, cmd]),
            np.asarray(volt), np.asarray(spd),
            np.asarray(frc),  np.asarray(cmd)
        ]):
            if arr.size:
                lo, hi = float(arr.min()), float(arr.max())
                m = max(0.03, (hi - lo) * 0.12)
                ax.set_xlim(x0, x1); ax.set_ylim(lo - m, hi + m)

        latest = [pos[-1] if pos else 0, volt[-1] if volt else 0,
                  spd[-1] if spd else 0, frc[-1]  if frc  else 0,
                  cmd[-1] if cmd else 0]
        for hl, val in zip(self._hlines, latest):
            hl.set_ydata([val])

        self._stat_vars["theta"].set(f"{pos[-1]:.4f}"  if pos  else "—")
        self._stat_vars["omega"].set(f"{spd[-1]:.3f}"  if spd  else "—")
        self._stat_vars["force"].set(f"{frc[-1]:.4f}"  if frc  else "—")
        self._stat_vars["volt"].set( f"{volt[-1]:.3f}" if volt else "—")
        self._stat_vars["cmd"].set(  f"{cmd[-1]:.4f}"  if cmd  else "—")
        self._stat_vars["n"].set(str(n))

        alerts = []
        if volt and abs(volt[-1]) > VOLTAGE_LIMIT * 0.9:
            alerts.append(f"⚠ 電壓接近上限 {volt[-1]:.2f} V")
        if spd and abs(spd[-1]) > SPEED_LIMIT_RADS * 0.8:
            alerts.append(f"⚠ 速度接近上限 {spd[-1]:.2f} rad/s")
        self._alert_label.config(text="\n".join(alerts))
        self._canvas.draw_idle()

    def _update_stats_only(self):
        if not pos_buf: return
        self._stat_vars["theta"].set(f"{pos_buf[-1]:.4f}")
        self._stat_vars["omega"].set(f"{speed_buf[-1]:.3f}"     if speed_buf     else "—")
        self._stat_vars["force"].set(f"{force_est_buf[-1]:.4f}" if force_est_buf else "—")
        self._stat_vars["volt"].set( f"{voltage_buf[-1]:.3f}"   if voltage_buf   else "—")
        self._stat_vars["cmd"].set(  f"{cmd_pos_buf[-1]:.4f}"   if cmd_pos_buf   else "—")
        self._stat_vars["n"].set(str(len(time_buf)))

    # ── Live update ───────────────────────────────────────────────────────────

    def _push_live_impedance(self):
        global live_K, live_B, live_M
        live_K = self._K_var.get()
        live_B = self._B_var.get()
        live_M = self._M_var.get()

        # [v3] M=0 時特別說明退化行為
        if live_M < 1e-6:
            self._stability_var.set(
                "ℹ M=0：阻抗退化為靜態彈簧  θ_cmd = θ_d + F/K\n"
                "  B 參數在此模式下無效")
            return

        b2  = live_B ** 2
        mk4 = 4 * live_M * live_K
        if b2 < mk4:
            self._stability_var.set(
                f"⚠ 欠阻尼：B²={b2:.4f} < 4MK={mk4:.4f}\n"
                f"  建議增大 B 或減小 K/M")
        else:
            self._stability_var.set("✓ 阻尼條件滿足")

    def _push_live_pd(self):
        global live_Kp, live_Kd
        live_Kp = self._Kp_var.get()
        live_Kd = self._Kd_var.get()

    def _push_live_theta_d(self):
        global live_theta_d
        live_theta_d = self._theta_d_var.get()

    # ── Control thread ────────────────────────────────────────────────────────

    def _parse_params(self):
        return {
            "hardware":      int(self._hw_mode.get()[0]),
            "exp_time":      self._exp_time_var.get(),
            "sample_time":   self._sample_time_var.get(),
            "total_rounds":  self._total_rounds_var.get(),
        }

    def _start_control(self):
        global control_thread, round_counter, all_rounds_history

        self._start_btn.config(state="disabled")

        if _emergency_event.is_set():
            messagebox.showwarning("警告", "請先解除緊急停止！")
            self._start_btn.config(state="normal"); return

        if control_thread and control_thread.is_alive():
            self._log("[INFO] 送出停止訊號，等待舊控制 thread 結束...")
            _kill_event.set()
            self._wait_for_thread_then_start(retry=0)
            return

        self._do_start_control()

    def _wait_for_thread_then_start(self, retry):
        if control_thread and control_thread.is_alive():
            if retry >= THREAD_WAIT_MAX:
                messagebox.showwarning(
                    "警告",
                    f"舊控制 thread 無法在 {THREAD_WAIT_MAX * POLL_INTERVAL_MS // 1000} 秒內結束，"
                    "請重新啟動程式。")
                self._start_btn.config(state="normal"); return
            self.after(POLL_INTERVAL_MS,
                       lambda: self._wait_for_thread_then_start(retry + 1))
            return
        self._log("[INFO] 舊 thread 已結束，啟動新實驗")
        self._do_start_control()

    def _do_start_control(self):
        global control_thread, round_counter, all_rounds_history

        _kill_event.clear()
        _pause_event.clear()
        round_counter = 0
        all_rounds_history.clear()
        self._emergency_shown = False
        self._push_live_impedance()
        self._push_live_pd()
        self._push_live_theta_d()

        params = self._parse_params()
        n = params["total_rounds"]
        self._round_var.set(f"進度：0 / {n}")
        self._log(f"[START] K={live_K:.3f}  B={live_B:.4f}  M={live_M:.4f}  "
                  f"Kp={live_Kp:.1f}  Kd={live_Kd:.2f}  "
                  f"θ_d={live_theta_d:.3f} rad  共 {n} 次")

        control_thread = threading.Thread(
            target=control_loop,
            args=(params, self._log, self._update_status_threadsafe,
                  self._on_round_done, self._on_all_done,
                  self._on_new_round, self._on_safety_alert),
            daemon=True)
        control_thread.start()
        self._pause_btn.config(state="normal")
        self._stop_btn.config(state="normal")
        self._poll_thread()

    def _pause_control(self):
        if _pause_event.is_set():
            _pause_event.clear()
            self._pause_btn.config(text="⏸  暫停")
            self._log("[RESUME]")
        else:
            _pause_event.set()
            self._pause_btn.config(text="▶  繼續")
            self._log("[PAUSE]")

    def _stop_control(self):
        _kill_event.set()
        self._log("[STOP] 使用者中止")

    def _reset_emergency(self):
        _emergency_event.clear()
        self._emergency_shown = False
        self._emergency_banner.pack_forget()
        self._reset_btn.config(state="disabled")
        self._log("[INFO] 緊急停止已解除")

    def _poll_thread(self):
        global control_thread
        if control_thread and control_thread.is_alive():
            self.after(POLL_INTERVAL_MS, self._poll_thread)
        else:
            self._start_btn.config(state="normal")
            self._pause_btn.config(state="disabled", text="⏸  暫停")
            self._stop_btn.config(state="disabled")

    def _update_status_threadsafe(self, connected, device, error):
        self.after(0, lambda: self._apply_status(connected, device, error))

    def _apply_status(self, connected, device, error):
        if connected:
            self._status_cv.itemconfig(self._led, fill=self.ACCENT2)
            self._status_text.set("已連線")
            self._device_var.set(f"裝置：{device}")
            self._error_var.set("")
        else:
            self._status_cv.itemconfig(self._led, fill=self.DANGER)
            self._status_text.set("未連線")
            self._device_var.set("裝置：—")
            self._error_var.set(error)

    # ── 安全警報 ──────────────────────────────────────────────────────────────

    def _on_safety_alert(self, reason):
        self.after(0, lambda: self._show_emergency(reason))

    def _show_emergency(self, reason):
        if self._emergency_shown: return
        self._emergency_shown = True
        self._emergency_msg.set(f"🚨  緊急停止  |  {reason}")
        self._emergency_banner.pack(fill="x")
        self._reset_btn.config(state="normal")
        self._log(f"[EMERGENCY] 已觸發安全停止：{reason}")
        # 用 after() 延遲彈出視窗，避免阻塞主迴圈中的 after 任務
        self.after(100, lambda: messagebox.showerror(
            "緊急停止",
            f"已觸發安全停止：\n{reason}\n\n"
            "請確認旋臂狀態後按「解除緊急停止」"))

    # ── New round ─────────────────────────────────────────────────────────────

    def _on_new_round(self, rnd, clear_event):
        self.after(0, lambda: self._clear_plot_gui(rnd, clear_event))

    def _clear_plot_gui(self, rnd, clear_event):
        global _last_render_len
        with data_lock:
            _clear_plot_buffers()
        _last_render_len = 0
        for ln in self._lines: ln.set_data([], [])
        for ax in self._axes: ax.set_xlim(0, 1); ax.set_ylim(-1, 1)
        self._canvas.draw_idle()
        n = self._total_rounds_var.get()
        self._plot_round_var.set(f"第 {rnd} / {n} 輪")
        clear_event.set()

    # ── Round / all done ──────────────────────────────────────────────────────

    def _on_round_done(self, data, rnd):
        self.after(0, lambda: self._handle_round_done(data, rnd))

    def _handle_round_done(self, data, rnd):
        n = self._total_rounds_var.get()
        self._round_var.set(f"進度：{rnd} / {n}")
        self._log(f"[ROUND {rnd}/{n}] 完成  ({len(data)} 筆)")

    def _on_all_done(self):
        self.after(0, self._handle_all_done)

    def _handle_all_done(self):
        n = self._total_rounds_var.get()
        total = sum(len(r) for r in all_rounds_history)
        self._log(f"[DONE] 全部 {n} 次實驗完成（共 {total} 筆）")
        self._round_var.set(f"進度：{n} / {n}  ✓")
        if all_rounds_history:
            if messagebox.askyesno(
                    "實驗完成",
                    f"已完成 {n} 次實驗（共 {total} 筆）。\n\n"
                    "現在儲存 Excel（每輪一個工作表）嗎？"):
                self._download_excel_all()

    # ── Logging ───────────────────────────────────────────────────────────────

    def _log(self, msg):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.after(0, lambda: self._append_log(f"[{ts}]  {msg}\n"))

    def _append_log(self, line):
        self._log_text.config(state="normal")
        self._log_text.insert("end", line)
        self._log_text.see("end")
        self._log_text.config(state="disabled")

    def _clear_log(self):
        self._log_text.config(state="normal")
        self._log_text.delete("1.0", "end")
        self._log_text.config(state="disabled")

    # ── Excel / CSV ───────────────────────────────────────────────────────────

    def _download_excel_all(self):
        with data_lock:
            rounds = [list(r) for r in all_rounds_history]
        if not rounds:
            messagebox.showinfo("提示", "尚無資料"); return

        ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = (f"impedance_K{live_K:.3f}_B{live_B:.4f}"
                        f"_M{live_M:.4f}_{len(rounds)}rounds_{ts_str}.xlsx")
        path = filedialog.asksaveasfilename(
            initialfile=default_name, defaultextension=".xlsx",
            filetypes=[("Excel", "*.xlsx")],
            title="儲存全部輪次 Excel")
        if not path: return

        wb = openpyxl.Workbook()
        wb.remove(wb.active)
        hdr_fill  = PatternFill("solid", fgColor="1e2a38")
        hdr_font  = Font(name="Consolas", bold=True, color="00c8ff")
        hdr_align = Alignment(horizontal="center")

        for idx, rdata in enumerate(rounds):
            if not rdata: continue
            ws = wb.create_sheet(title=f"Round_{idx+1:02d}")
            # [v3] 使用固定欄位順序，確保所有輪次結構一致
            headers = _ROW_FIELDS
            for ci, h in enumerate(headers, 1):
                cell = ws.cell(row=1, column=ci, value=h)
                cell.fill = hdr_fill; cell.font = hdr_font
                cell.alignment = hdr_align
            for ri, row in enumerate(rdata, 2):
                for ci, h in enumerate(headers, 1):
                    ws.cell(row=ri, column=ci, value=row.get(h, ""))
            for col in ws.columns:
                w = max(len(str(c.value)) if c.value else 0 for c in col)
                ws.column_dimensions[
                    col[0].column_letter].width = min(w + 2, 24)

        wb.save(path)
        total = sum(len(r) for r in rounds)
        self._log(f"[SAVE] Excel → {path}  ({len(rounds)} 輪, {total} 筆)")
        messagebox.showinfo("完成",
                            f"Excel 已儲存（{len(rounds)} 個工作表）\n{path}")

    def _download_csv_last(self):
        with data_lock:
            data = list(round_history)
        if not data:
            messagebox.showinfo("提示", "尚無最後一輪資料"); return
        ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = (f"impedance_K{live_K:.3f}_B{live_B:.4f}"
                        f"_M{live_M:.4f}_round{round_counter:03d}"
                        f"_{ts_str}.csv")
        path = filedialog.asksaveasfilename(
            initialfile=default_name, defaultextension=".csv",
            filetypes=[("CSV", "*.csv")], title="儲存最後一輪 CSV")
        if not path: return
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=_ROW_FIELDS,
                               extrasaction="ignore")
            w.writeheader(); w.writerows(data)
        self._log(f"[SAVE] 最後一輪 CSV → {path}  ({len(data)} 筆)")
        messagebox.showinfo("完成", f"CSV 已儲存\n{path}")

    def _clear_data(self):
        global all_rounds_history, round_history, round_counter, _last_render_len
        if not messagebox.askyesno(
                "確認", "確定要清除所有資料？此動作無法復原。"):
            return
        with data_lock:
            all_rounds_history.clear()
            round_history.clear()
            _clear_plot_buffers()
        round_counter = 0; _last_render_len = 0
        for ln in self._lines: ln.set_data([], [])
        for ax in self._axes: ax.set_xlim(0, 1); ax.set_ylim(-1, 1)
        self._canvas.draw_idle()
        n = self._total_rounds_var.get()
        self._round_var.set(f"進度：0 / {n}")
        self._plot_round_var.set("")
        self._log("[CLEAR] 所有資料已清除")

    # ── Widget helpers ────────────────────────────────────────────────────────

    def _card(self, parent, title):
        f = tk.Frame(parent, bg=self.PANEL,
                     highlightbackground=self.BORDER,
                     highlightthickness=1, padx=10, pady=6)
        f.pack(fill="x", pady=3)
        if title:
            tk.Label(f, text=title, font=self.FONT_H2,
                     bg=self.PANEL, fg=self.TEXT).pack(
                anchor="w", pady=(0, 4))
        return f

    def _btn(self, parent, text, command, bg, fg):
        b = tk.Button(parent, text=text, command=command,
                      bg=bg, fg=fg, font=("Segoe UI", 10, "bold"),
                      relief="flat", padx=10, pady=5,
                      activebackground=self.BORDER, cursor="hand2")
        b.pack(fill="x", pady=2)
        return b

    def _labeled_combo(self, parent, label, values, default):
        row = tk.Frame(parent, bg=self.PANEL); row.pack(fill="x", pady=2)
        tk.Label(row, text=label, width=10, anchor="w",
                 font=self.FONT_BODY, bg=self.PANEL,
                 fg=self.SUBTEXT).pack(side="left")
        var = tk.StringVar(value=default)
        ttk.Combobox(row, textvariable=var, values=values,
                     state="readonly", width=14).pack(side="left", padx=4)
        return var

    def _tick_clock(self):
        self._clock_var.set(
            datetime.datetime.now().strftime("%Y-%m-%d  %H:%M:%S"))
        self.after(1000, self._tick_clock)

    def _on_close(self):
        _kill_event.set()
        plt.close("all")
        self.destroy()


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = ImpedanceControlPanel()
    app.mainloop()
