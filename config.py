"""
config.py — 所有常數集中在此，修改參數只需改這一個檔案。
"""
import math

# ── 馬達參數（系統辨識後在此修改）────────────────────────────────────────────
Kt         = 0.042      # 力矩常數   [N·m/A]
J_motor    = 4.0e-6    # 轉子慣量   [kg·m²]
B_friction = 1.0e-5    # 黏性摩擦   [N·m·s/rad]

# ── 安全限制 ──────────────────────────────────────────────────────────────────
ANGLE_LIMIT_RAD  = math.radians(270)   # ±270°
SPEED_LIMIT_RADS = 50.0                # [rad/s]
VOLTAGE_LIMIT    = 10.0                # [V]
CURRENT_LIMIT    = 4.0                 # [A]
FORCE_EST_LIMIT  = 1.0                 # [N·m]
WARMUP_CYCLES    = 50                  # 暖機週期數（dt=0.002s → 100 ms）

# ── 濾波器截止頻率 [Hz] ────────────────────────────────────────────────────────
FC_SPEED = 40.0
FC_ACCEL = 15.0
FC_FORCE = 15.0

# ── 資料緩衝 ──────────────────────────────────────────────────────────────────
BUFFER_SIZE = 4000

# 固定 CSV / Excel 欄位順序（不隨參數值改變）
ROW_FIELDS = [
    "round", "time",
    "theta_rad", "theta_d_rad", "theta_cmd_rad",
    "omega_rads", "voltage_V", "current_A", "force_est_Nm",
    "K_Nm_rad", "B_Nms_rad", "M_kgm2", "Kp", "Kd",
]

# ── GUI 更新速率 ───────────────────────────────────────────────────────────────
PLOT_DOWNSAMPLE_PTS = 800   # 繪圖最大取樣點數
POLL_INTERVAL_MS    = 200   # 控制 thread 輪詢間隔 [ms]
PLOT_INTERVAL_MS    = 80    # 繪圖更新間隔 [ms]
THREAD_WAIT_MAX     = 30    # 最多等待舊 thread 幾次（× POLL_INTERVAL_MS）
