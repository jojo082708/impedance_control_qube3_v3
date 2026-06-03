"""
gui/app.py — 主 GUI 視窗。

不使用全域變數，所有執行緒間的共享狀態都透過 SharedState 存取。
"""
import math
import datetime
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from gui.widgets   import ParamWidget
from control.loop  import control_loop
from data.export   import save_excel, save_csv, make_default_filename
from config import (
    Kt, J_motor, B_friction,
    ANGLE_LIMIT_RAD, SPEED_LIMIT_RADS, VOLTAGE_LIMIT,
    CURRENT_LIMIT, FORCE_EST_LIMIT,
    PLOT_DOWNSAMPLE_PTS, POLL_INTERVAL_MS, PLOT_INTERVAL_MS, THREAD_WAIT_MAX,
)


class ImpedanceControlPanel(tk.Tk):

    # ── 色票 ──────────────────────────────────────────────────────────────────
    BG      = "#0b0f14"; PANEL   = "#111820"
    BORDER  = "#1e2a38"; ACCENT  = "#00c8ff"; ACCENT2 = "#00e5a0"
    WARN    = "#f0a800"; DANGER  = "#ff4d6a"; PURPLE  = "#a78bfa"
    TEXT    = "#dde6f0"; SUBTEXT = "#7a8899"; GRID_C  = "#141e28"
    ORANGE  = "#ff7b42"

    CLR_POS  = "#00c8ff"; CLR_DES  = "#00e5a0"; CLR_CMD  = "#f0a800"
    CLR_SPD  = "#a78bfa"; CLR_FRC  = "#ff7b42"; CLR_VOLT = "#ff4d6a"

    FONT_MONO = ("Consolas", 10)
    FONT_BODY = ("Segoe UI", 10)
    FONT_H2   = ("Segoe UI", 11, "bold")
    FONT_H1   = ("Segoe UI", 13, "bold")

    def __init__(self, state):
        super().__init__()
        self.state            = state          # 唯一的共享狀態物件
        self._ctrl_thread     = None           # 控制 thread（不用全域變數）
        self._emergency_shown = False
        self._last_render_len = 0              # 繪圖效能優化

        self.title("QUBE-Servo 3 — 阻抗控制  v3")
        self.configure(bg=self.BG)
        self.geometry("1720x1020")
        self.minsize(1400, 850)

        self._build_ui()
        self._start_plot_animation()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ══════════════════════════════════════════════════════════════════════════
    # UI 建構
    # ══════════════════════════════════════════════════════════════════════════

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
        tk.Label(rt, textvariable=self._clock_var,
                 font=("Consolas", 11),
                 bg=self.BG, fg=self.SUBTEXT).pack(side="right", padx=8)
        self._status_cv = tk.Canvas(rt, width=12, height=12,
                                    bg=self.BG, highlightthickness=0)
        self._status_cv.pack(side="right", padx=(0, 4))
        self._led = self._status_cv.create_oval(
            1, 1, 11, 11, fill=self.DANGER, outline="")
        self._status_text = tk.StringVar(value="未連線")
        tk.Label(rt, textvariable=self._status_text,
                 font=self.FONT_BODY, bg=self.BG,
                 fg=self.TEXT).pack(side="right", padx=4)
        self._tick_clock()

        self._emergency_banner = tk.Frame(self, bg=self.DANGER, pady=4)
        self._emergency_msg    = tk.StringVar(value="")
        tk.Label(self._emergency_banner,
                 textvariable=self._emergency_msg,
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
        lw   = lc.create_window((0, 0), window=left, anchor="nw")

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

    # ── 左欄 ──────────────────────────────────────────────────────────────────

    def _build_left(self, parent):
        self._build_motor_params_card(parent)
        self._build_impedance_card(parent)
        self._build_pd_card(parent)
        self._build_target_angle_card(parent)
        self._build_experiment_card(parent)
        self._build_safety_info_card(parent)
        self._build_action_buttons(parent)

    def _build_motor_params_card(self, parent):
        cp = self._card(parent, "🔧  馬達參數（常數）")
        for lbl, val, unit in [
            ("Kt  力矩常數", f"{Kt:.4f}",       "N·m/A"),
            ("J   轉子慣量", f"{J_motor:.2e}",   "kg·m²"),
            ("Bf  黏性摩擦", f"{B_friction:.2e}", "N·m·s/rad"),
        ]:
            row = tk.Frame(cp, bg=self.PANEL); row.pack(fill="x", pady=2)
            tk.Label(row, text=lbl, font=("Consolas", 9),
                     bg=self.PANEL, fg=self.SUBTEXT,
                     width=16, anchor="w").pack(side="left")
            tk.Label(row, text=val, font=("Consolas", 10, "bold"),
                     bg=self.PANEL, fg=self.ACCENT).pack(side="left", padx=4)
            tk.Label(row, text=unit, font=("Consolas", 9),
                     bg=self.PANEL, fg=self.SUBTEXT).pack(side="left")
        tk.Label(cp, text="※ 修改請編輯 config.py — MOTOR PARAMETERS",
                 font=("Consolas", 8), bg=self.PANEL, fg=self.SUBTEXT,
                 wraplength=320, justify="left").pack(anchor="w", pady=(4, 0))

    def _build_impedance_card(self, parent):
        ci = self._card(parent, "🎛  阻抗參數  （即時生效）")
        self._K_var  = tk.DoubleVar(value=1.0)
        self._B_var  = tk.DoubleVar(value=0.1)
        self._M_var  = tk.DoubleVar(value=0.05)
        ParamWidget(ci, "K  剛性", self._K_var, 0.0, 5.0, 0.01,
                    unit="N·m/rad", fmt="{:.3f}",
                    slider_length=95).pack(fill="x", pady=3)
        ParamWidget(ci, "B  阻尼", self._B_var, 0.0, 1.0, 0.001,
                    unit="N·m·s/rad", fmt="{:.3f}",
                    slider_length=95).pack(fill="x", pady=3)
        ParamWidget(ci, "M  虛擬慣量", self._M_var, 0.0, 0.5, 0.001,
                    unit="kg·m²", fmt="{:.3f}",
                    slider_length=95).pack(fill="x", pady=3)
        self._stability_var = tk.StringVar(value="")
        tk.Label(ci, textvariable=self._stability_var,
                 font=("Consolas", 9), bg=self.PANEL,
                 fg=self.WARN, wraplength=320,
                 justify="left").pack(anchor="w", pady=(2, 0))
        for v in [self._K_var, self._B_var, self._M_var]:
            v.trace_add("write", lambda *_: self._sync_params())

    def _build_pd_card(self, parent):
        cpd = self._card(parent, "⚡  內迴路 PD 控制器")
        self._Kp_var = tk.DoubleVar(value=20.0)
        self._Kd_var = tk.DoubleVar(value=0.5)
        ParamWidget(cpd, "Kp  比例增益", self._Kp_var, 0.0, 50.0, 0.5,
                    fmt="{:.1f}", slider_length=95).pack(fill="x", pady=3)
        ParamWidget(cpd, "Kd  微分增益", self._Kd_var, 0.0, 5.0, 0.05,
                    fmt="{:.2f}", slider_length=95).pack(fill="x", pady=3)
        for v in [self._Kp_var, self._Kd_var]:
            v.trace_add("write", lambda *_: self._sync_params())

    def _build_target_angle_card(self, parent):
        cth = self._card(parent, "🎯  目標角度")
        self._theta_d_var = tk.DoubleVar(value=0.0)
        ParamWidget(cth, "θ_d  目標角度", self._theta_d_var,
                    -math.pi, math.pi, 0.01, unit="rad",
                    fmt="{:.3f}", slider_length=95).pack(fill="x", pady=3)
        self._theta_d_var.trace_add("write", lambda *_: self._sync_params())

    def _build_experiment_card(self, parent):
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

    def _build_safety_info_card(self, parent):
        cs = self._card(parent, "🛡  安全限制（固定）")
        for lbl, val in [
            ("角度限制", f"±{math.degrees(ANGLE_LIMIT_RAD):.0f}°"),
            ("速度限制", f"±{SPEED_LIMIT_RADS:.0f} rad/s"),
            ("電壓限制", f"±{VOLTAGE_LIMIT:.0f} V"),
            ("電流異常", f">{CURRENT_LIMIT:.1f} A"),
            ("外力異常", f">{FORCE_EST_LIMIT:.2f} N·m"),
        ]:
            row = tk.Frame(cs, bg=self.PANEL); row.pack(fill="x", pady=1)
            tk.Label(row, text=lbl, font=("Consolas", 9),
                     bg=self.PANEL, fg=self.SUBTEXT,
                     width=12, anchor="w").pack(side="left")
            tk.Label(row, text=val, font=("Consolas", 9, "bold"),
                     bg=self.PANEL, fg=self.WARN).pack(side="left")

    def _build_action_buttons(self, parent):
        cb = self._card(parent, "")
        self._device_var = tk.StringVar(value="裝置：—")
        tk.Label(cb, textvariable=self._device_var,
                 font=self.FONT_MONO, bg=self.PANEL,
                 fg=self.SUBTEXT).pack(anchor="w", pady=(0, 4))
        self._error_var = tk.StringVar(value="")
        tk.Label(cb, textvariable=self._error_var,
                 font=("Consolas", 9), bg=self.PANEL, fg=self.DANGER,
                 wraplength=320, justify="left").pack(anchor="w")

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

    # ── 中欄（波形）─────────────────────────────────────────────────────────

    def _build_centre(self, parent):
        hdr = tk.Frame(parent, bg=self.BG)
        hdr.pack(fill="x", pady=(0, 4))
        tk.Label(hdr, text="📊  即時波形（當前輪）",
                 font=self.FONT_H1, bg=self.BG,
                 fg=self.TEXT).pack(side="left")
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
                           font=("Consolas", 9),
                           cursor="hand2").pack(side="left", padx=2)
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
            ax.set_title(title, color=self.TEXT, fontsize=8.5, pad=3, loc="left")
            ax.set_ylabel(ylab, color=self.SUBTEXT, fontsize=7.5)
            ax.tick_params(colors=self.SUBTEXT, labelsize=7, length=2)
            for sp in ax.spines.values():
                sp.set_edgecolor(self.BORDER)
            ax.grid(True, color=self.GRID_C, linewidth=0.6, linestyle="-")
            ax.set_xlim(0, 1); ax.set_ylim(-1, 1)

        ln_pos,   = axes[0].plot([], [], color=self.CLR_POS,
                                 lw=1.4, label="實際 θ")
        ln_des,   = axes[0].plot([], [], color=self.CLR_DES,
                                 lw=1.1, ls="--", alpha=0.7, label="目標 θ_d")
        ln_cmd_0, = axes[0].plot([], [], color=self.CLR_CMD,
                                 lw=1.1, ls=":",  alpha=0.8, label="修正 θ_cmd")
        axes[0].legend(fontsize=7, facecolor=ax_bg, edgecolor=self.BORDER,
                       labelcolor=self.TEXT, loc="upper right", framealpha=0.8)
        ln_volt, = axes[1].plot([], [], color=self.CLR_VOLT, lw=1.3)
        ln_spd,  = axes[2].plot([], [], color=self.CLR_SPD,  lw=1.3)
        ln_frc,  = axes[3].plot([], [], color=self.CLR_FRC,  lw=1.3)
        ln_cmd,  = axes[4].plot([], [], color=self.CLR_CMD,  lw=1.3)
        self._lines = [ln_pos, ln_des, ln_cmd_0, ln_volt, ln_spd, ln_frc, ln_cmd]

        self._hlines = []
        for ax, clr in zip(axes, [self.CLR_POS, self.CLR_VOLT,
                                   self.CLR_SPD, self.CLR_FRC, self.CLR_CMD]):
            hl = ax.axhline(y=0, color=clr, lw=0.6, ls=":", alpha=0.5)
            self._hlines.append(hl)

        canvas = FigureCanvasTkAgg(self._fig, master=parent)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas = canvas

    # ── 右欄（即時數值 + 日誌）───────────────────────────────────────────────

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

    # ══════════════════════════════════════════════════════════════════════════
    # 即時參數同步
    # ══════════════════════════════════════════════════════════════════════════

    def _sync_params(self):
        """從 GUI 變數讀取所有控制參數，原子寫入 SharedState。"""
        K  = self._K_var.get()
        B  = self._B_var.get()
        M  = self._M_var.get()
        Kp = self._Kp_var.get()
        Kd = self._Kd_var.get()
        td = self._theta_d_var.get()
        self.state.set_params(K, B, M, Kp, Kd, td)
        self._update_stability_hint(K, B, M)

    def _update_stability_hint(self, K, B, M):
        if M < 1e-6:
            self._stability_var.set(
                "ℹ M=0：靜態彈簧模式  θ_cmd = θ_d + F/K\n  B 參數無效")
            return
        b2  = B ** 2
        mk4 = 4 * M * K
        if b2 < mk4:
            self._stability_var.set(
                f"⚠ 欠阻尼：B²={b2:.4f} < 4MK={mk4:.4f}\n"
                f"  建議增大 B 或減小 K/M")
        else:
            self._stability_var.set("✓ 阻尼條件滿足")

    # ══════════════════════════════════════════════════════════════════════════
    # 控制 thread 管理
    # ══════════════════════════════════════════════════════════════════════════

    def _start_control(self):
        self._start_btn.config(state="disabled")

        if self.state.emergency.is_set():
            messagebox.showwarning("警告", "請先解除緊急停止！")
            self._start_btn.config(state="normal"); return

        if self._ctrl_thread and self._ctrl_thread.is_alive():
            self._log("[INFO] 送出停止訊號，等待舊控制 thread 結束...")
            self.state.kill.set()
            self._wait_then_start(retry=0)
            return

        self._do_start()

    def _wait_then_start(self, retry: int):
        if self._ctrl_thread and self._ctrl_thread.is_alive():
            if retry >= THREAD_WAIT_MAX:
                messagebox.showwarning(
                    "警告",
                    f"舊 thread 無法在 "
                    f"{THREAD_WAIT_MAX * POLL_INTERVAL_MS // 1000} 秒內結束。")
                self._start_btn.config(state="normal"); return
            self.after(POLL_INTERVAL_MS,
                       lambda: self._wait_then_start(retry + 1))
            return
        self._log("[INFO] 舊 thread 已結束，啟動新實驗")
        self._do_start()

    def _do_start(self):
        n = self._total_rounds_var.get()
        self.state.reset_for_experiment()
        self._emergency_shown = False
        self._sync_params()
        self._round_var.set(f"進度：0 / {n}")

        K, B, M, Kp, Kd, td = self.state.get_params()
        self._log(f"[START] K={K:.3f}  B={B:.4f}  M={M:.4f}  "
                  f"Kp={Kp:.1f}  Kd={Kd:.2f}  θ_d={td:.3f} rad  共 {n} 次")

        params = {
            "sample_time":  self._sample_time_var.get(),
            "exp_time":     self._exp_time_var.get(),
            "total_rounds": n,
        }
        self._ctrl_thread = threading.Thread(
            target=control_loop,
            args=(params, self.state,
                  self._log, self._update_status_safe,
                  self._on_round_done, self._on_all_done,
                  self._on_new_round, self._on_safety_alert),
            daemon=True)
        self._ctrl_thread.start()
        self._pause_btn.config(state="normal")
        self._stop_btn.config(state="normal")
        self._poll_thread()

    def _pause_control(self):
        if self.state.pause.is_set():
            self.state.pause.clear()
            self._pause_btn.config(text="⏸  暫停")
            self._log("[RESUME]")
        else:
            self.state.pause.set()
            self._pause_btn.config(text="▶  繼續")
            self._log("[PAUSE]")

    def _stop_control(self):
        self.state.kill.set()
        self._log("[STOP] 使用者中止")

    def _reset_emergency(self):
        self.state.emergency.clear()
        self._emergency_shown = False
        self._emergency_banner.pack_forget()
        self._reset_btn.config(state="disabled")
        self._log("[INFO] 緊急停止已解除")

    def _poll_thread(self):
        if self._ctrl_thread and self._ctrl_thread.is_alive():
            self.after(POLL_INTERVAL_MS, self._poll_thread)
        else:
            self._start_btn.config(state="normal")
            self._pause_btn.config(state="disabled", text="⏸  暫停")
            self._stop_btn.config(state="disabled")

    # ══════════════════════════════════════════════════════════════════════════
    # Callbacks（由控制 thread 呼叫，透過 after() 切回主 thread）
    # ══════════════════════════════════════════════════════════════════════════

    def _update_status_safe(self, connected, device, error):
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

    def _on_safety_alert(self, reason: str):
        self.after(0, lambda: self._show_emergency(reason))

    def _show_emergency(self, reason: str):
        if self._emergency_shown:
            return
        self._emergency_shown = True
        self._emergency_msg.set(f"🚨  緊急停止  |  {reason}")
        self._emergency_banner.pack(fill="x")
        self._reset_btn.config(state="normal")
        self._log(f"[EMERGENCY] 已觸發安全停止：{reason}")
        self.after(100, lambda: messagebox.showerror(
            "緊急停止",
            f"已觸發安全停止：\n{reason}\n\n"
            "請確認旋臂狀態後按「解除緊急停止」"))

    def _on_new_round(self, rnd: int, clear_event: threading.Event):
        self.after(0, lambda: self._clear_plot_gui(rnd, clear_event))

    def _clear_plot_gui(self, rnd: int, clear_event: threading.Event):
        with self.state.data_lock:
            self.state.clear_buffers()
        self._last_render_len = 0
        for ln in self._lines:
            ln.set_data([], [])
        for ax in self._axes:
            ax.set_xlim(0, 1); ax.set_ylim(-1, 1)
        self._canvas.draw_idle()
        n = self._total_rounds_var.get()
        self._plot_round_var.set(f"第 {rnd} / {n} 輪")
        clear_event.set()

    def _on_round_done(self, data: list, rnd: int):
        self.after(0, lambda: self._handle_round_done(data, rnd))

    def _handle_round_done(self, data: list, rnd: int):
        n = self._total_rounds_var.get()
        self._round_var.set(f"進度：{rnd} / {n}")
        self._log(f"[ROUND {rnd}/{n}] 完成  ({len(data)} 筆)")

    def _on_all_done(self):
        self.after(0, self._handle_all_done)

    def _handle_all_done(self):
        n     = self._total_rounds_var.get()
        total = sum(len(r) for r in self.state.all_rounds_history)
        self._log(f"[DONE] 全部 {n} 次實驗完成（共 {total} 筆）")
        self._round_var.set(f"進度：{n} / {n}  ✓")
        if self.state.all_rounds_history:
            if messagebox.askyesno(
                    "實驗完成",
                    f"已完成 {n} 次實驗（共 {total} 筆）。\n\n"
                    "現在儲存 Excel 嗎？"):
                self._download_excel_all()

    # ══════════════════════════════════════════════════════════════════════════
    # 繪圖更新
    # ══════════════════════════════════════════════════════════════════════════

    def _start_plot_animation(self):
        self._canvas.draw()
        self._schedule_plot()

    def _schedule_plot(self):
        self._update_plots()
        self.after(PLOT_INTERVAL_MS, self._schedule_plot)

    def _update_plots(self):
        with self.state.data_lock:
            n = len(self.state.time_buf)
            if n == 0:
                return
            if n == self._last_render_len:
                self._update_stats_only(); return
            self._last_render_len = n
            step = max(1, n // PLOT_DOWNSAMPLE_PTS)
            ts   = list(self.state.time_buf)[::step]
            pos  = list(self.state.pos_buf)[::step]
            des  = list(self.state.desired_buf)[::step]
            cmd  = list(self.state.cmd_pos_buf)[::step]
            volt = list(self.state.voltage_buf)[::step]
            spd  = list(self.state.speed_buf)[::step]
            frc  = list(self.state.force_est_buf)[::step]

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
            np.asarray(frc),  np.asarray(cmd),
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
        """資料量未增加時只更新數字顯示，跳過昂貴的 matplotlib 繪圖。"""
        s = self.state
        if not s.pos_buf: return
        self._stat_vars["theta"].set(f"{s.pos_buf[-1]:.4f}")
        self._stat_vars["omega"].set(
            f"{s.speed_buf[-1]:.3f}"     if s.speed_buf     else "—")
        self._stat_vars["force"].set(
            f"{s.force_est_buf[-1]:.4f}" if s.force_est_buf else "—")
        self._stat_vars["volt"].set(
            f"{s.voltage_buf[-1]:.3f}"   if s.voltage_buf   else "—")
        self._stat_vars["cmd"].set(
            f"{s.cmd_pos_buf[-1]:.4f}"   if s.cmd_pos_buf   else "—")
        self._stat_vars["n"].set(str(len(s.time_buf)))

    # ══════════════════════════════════════════════════════════════════════════
    # 資料匯出
    # ══════════════════════════════════════════════════════════════════════════

    def _download_excel_all(self):
        with self.state.data_lock:
            rounds = [list(r) for r in self.state.all_rounds_history]
        if not rounds:
            messagebox.showinfo("提示", "尚無資料"); return

        default = make_default_filename(self.state, len(rounds), "xlsx")
        path = filedialog.asksaveasfilename(
            initialfile=default, defaultextension=".xlsx",
            filetypes=[("Excel", "*.xlsx")],
            title="儲存全部輪次 Excel")
        if not path: return

        save_excel(rounds, path, self._log)
        messagebox.showinfo("完成",
                            f"Excel 已儲存（{len(rounds)} 個工作表）\n{path}")

    def _download_csv_last(self):
        with self.state.data_lock:
            data = list(self.state.round_history)
        if not data:
            messagebox.showinfo("提示", "尚無最後一輪資料"); return

        default = make_default_filename(
            self.state, self.state.round_counter, "csv")
        path = filedialog.asksaveasfilename(
            initialfile=default, defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
            title="儲存最後一輪 CSV")
        if not path: return

        save_csv(data, path, self._log)
        messagebox.showinfo("完成", f"CSV 已儲存\n{path}")

    def _clear_data(self):
        if not messagebox.askyesno(
                "確認", "確定要清除所有資料？此動作無法復原。"):
            return
        with self.state.data_lock:
            self.state.all_rounds_history.clear()
            self.state.round_history.clear()
            self.state.round_counter = 0
            self.state.clear_buffers()
        self._last_render_len = 0
        for ln in self._lines: ln.set_data([], [])
        for ax in self._axes:
            ax.set_xlim(0, 1); ax.set_ylim(-1, 1)
        self._canvas.draw_idle()
        n = self._total_rounds_var.get()
        self._round_var.set(f"進度：0 / {n}")
        self._plot_round_var.set("")
        self._log("[CLEAR] 所有資料已清除")

    # ══════════════════════════════════════════════════════════════════════════
    # 日誌
    # ══════════════════════════════════════════════════════════════════════════

    def _log(self, msg: str):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.after(0, lambda: self._append_log(f"[{ts}]  {msg}\n"))

    def _append_log(self, line: str):
        self._log_text.config(state="normal")
        self._log_text.insert("end", line)
        self._log_text.see("end")
        self._log_text.config(state="disabled")

    def _clear_log(self):
        self._log_text.config(state="normal")
        self._log_text.delete("1.0", "end")
        self._log_text.config(state="disabled")

    # ══════════════════════════════════════════════════════════════════════════
    # Helper 元件
    # ══════════════════════════════════════════════════════════════════════════

    def _card(self, parent, title: str) -> tk.Frame:
        f = tk.Frame(parent, bg=self.PANEL,
                     highlightbackground=self.BORDER,
                     highlightthickness=1, padx=10, pady=6)
        f.pack(fill="x", pady=3)
        if title:
            tk.Label(f, text=title, font=self.FONT_H2,
                     bg=self.PANEL, fg=self.TEXT).pack(
                anchor="w", pady=(0, 4))
        return f

    def _btn(self, parent, text: str, command,
             bg: str, fg: str) -> tk.Button:
        b = tk.Button(parent, text=text, command=command,
                      bg=bg, fg=fg, font=("Segoe UI", 10, "bold"),
                      relief="flat", padx=10, pady=5,
                      activebackground=self.BORDER, cursor="hand2")
        b.pack(fill="x", pady=2)
        return b

    def _tick_clock(self):
        self._clock_var.set(
            datetime.datetime.now().strftime("%Y-%m-%d  %H:%M:%S"))
        self.after(1000, self._tick_clock)

    def _on_close(self):
        self.state.kill.set()
        plt.close("all")
        self.destroy()
