"""
control/loop.py — 主控制迴圈（在 daemon thread 內執行）。

依賴 SharedState 進行執行緒間通訊，不使用任何全域變數。
虛擬模式已移除，僅支援實體 QUBE-Servo 3。
"""
import time
import threading
from dataclasses import dataclass
from typing import Callable, Optional
import numpy as np

from control.filters   import LowPassFilter
from control.impedance import ImpedanceDynamics
from control.safety    import SafetyChecker
from control.step      import StepContext, run_step
from config            import FC_SPEED, FC_ACCEL, FC_FORCE, WARMUP_CYCLES


@dataclass
class RoundConfig:
    """每輪實驗的不可變設定，用來減少 _run_one_round 的參數數量。"""
    rnd_idx:       int
    total_rounds:  int
    exp_time:      float
    dt:            float
    qube:          object
    has_current:   bool


def _try_import_qube():
    try:
        from pal.products.qube import QubeServo3
        return QubeServo3
    except ImportError:
        return None


def control_loop(params: dict, state,
                 log_cb:          Callable[[str], None],
                 status_cb:       Callable[[bool, str, str], None],
                 round_done_cb:   Callable[[list, int], None],
                 all_done_cb:     Callable[[], None],
                 new_round_cb:    Callable[[int, threading.Event], None],
                 safety_alert_cb: Callable[[str], None]):
    """
    主控制迴圈。

    Args:
        params:         {"sample_time", "exp_time", "total_rounds"}
        state:          SharedState 實例
        log_cb:         寫日誌的 callback
        status_cb:      更新連線狀態的 callback (connected, device, error)
        round_done_cb:  每輪完成後的 callback (round_data, round_number)
        all_done_cb:    全部輪次完成後的 callback
        new_round_cb:   新輪開始前的 callback (round_number, clear_event)
        safety_alert_cb: 觸發緊急停止的 callback (reason)
    """
    dt           = params["sample_time"]
    total_rounds = params["total_rounds"]
    exp_time     = params["exp_time"]

    safety  = SafetyChecker(log_cb, state.emergency)
    imp_dyn = ImpedanceDynamics()

    QubeServo3 = _try_import_qube()
    if QubeServo3 is None:
        msg = "找不到 pal 套件，請確認已安裝 QUBE 驅動。"
        log_cb(f"[ERROR] {msg}")
        status_cb(False, "None", msg)
        all_done_cb(); return

    log_cb(f"[INFO] 實體模式 | dt={dt}s  exp={exp_time}s  "
           f"rounds={total_rounds}  warmup={WARMUP_CYCLES}cycles")

    try:
        device_ctx = QubeServo3(hardware=1, pendulum=0, readMode=0)
    except Exception as e:
        msg = f"實體連線失敗：{e}"
        log_cb(f"[ERROR] {msg}")
        status_cb(False, "None", msg)
        all_done_cb(); return

    status_cb(True, "QubeServo3", "")

    with device_ctx as qube:
        _has_current = hasattr(qube, "motorCurrent")
        if not _has_current:
            log_cb("[WARN] 無 motorCurrent 屬性，電流固定為 0.0A，外力估算停用")

        for rnd_idx in range(total_rounds):
            if state.kill.is_set() or state.emergency.is_set():
                break

            cfg = RoundConfig(rnd_idx, total_rounds, exp_time, dt,
                              qube, _has_current)
            _run_one_round(cfg, safety, imp_dyn, state,
                           log_cb, round_done_cb, new_round_cb, safety_alert_cb)

    status_cb(False, "None", "")
    log_cb("[INFO] 控制迴圈結束")
    all_done_cb()


def _run_one_round(cfg: RoundConfig,
                   safety, imp_dyn, state,
                   log_cb:          Callable[[str], None],
                   round_done_cb:   Callable[[list, int], None],
                   new_round_cb:    Callable[[int, threading.Event], None],
                   safety_alert_cb: Callable[[str], None]):
    """執行一輪實驗，將每輪的設定與清理封裝在此。"""
    safety.reset()
    imp_dyn.reset()

    clear_event = threading.Event()
    new_round_cb(cfg.rnd_idx + 1, clear_event)
    clear_event.wait(timeout=2.0)

    ctx = StepContext(
        LowPassFilter(FC_SPEED, cfg.dt),
        LowPassFilter(FC_ACCEL, cfg.dt),
        LowPassFilter(FC_FORCE, cfg.dt),
        imp_dyn,
    )

    with state.data_lock:
        state.round_history.clear()

    log_cb(f"[ROUND {cfg.rnd_idx + 1}/{cfg.total_rounds}] 開始")

    start_time = time.time()
    timestamp  = 0.0

    while (timestamp < cfg.exp_time
           and not state.kill.is_set()
           and not state.emergency.is_set()):

        while state.pause.is_set() and not state.kill.is_set():
            time.sleep(0.05)

        t0 = time.time()

        try:
            cfg.qube.read_outputs()
            theta   = float(np.asarray(cfg.qube.motorPosition).flat[0])
            current = (float(np.asarray(cfg.qube.motorCurrent).flat[0])
                       if cfg.has_current else 0.0)

            ctrl_params = state.get_params()
            safe, reason, voltage, row = run_step(
                theta, current, ctx, ctrl_params,
                safety, cfg.dt, cfg.rnd_idx + 1, timestamp)

            if not safe:
                cfg.qube.write_voltage(0.0)
                safety_alert_cb(reason)
                return

            cfg.qube.write_voltage(voltage)

            _, _, _, Kp, Kd, theta_d = ctrl_params
            with state.data_lock:
                state.push_buffers(timestamp, theta, theta_d,
                                   row["theta_cmd_rad"], row["omega_rads"],
                                   voltage, row["force_est_Nm"])
                state.round_history.append(row)

        except Exception as e:
            log_cb(f"[ERROR] {type(e).__name__}: {e}")
            try:
                cfg.qube.write_voltage(0.0)
            except Exception:
                pass
            safety_alert_cb(f"迴圈例外：{type(e).__name__}: {e}")
            return

        elapsed   = time.time() - t0
        time.sleep(max(0.0, cfg.dt - elapsed))
        timestamp = time.time() - start_time

    cfg.qube.write_voltage(0.0)

    if not state.kill.is_set() and not state.emergency.is_set():
        with state.data_lock:
            state.round_counter += 1
            state.all_rounds_history.append(list(state.round_history))
            rnd = state.round_counter

        log_cb(f"[ROUND {rnd}] 完成")
        round_done_cb(list(state.round_history), rnd)
