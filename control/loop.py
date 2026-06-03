"""
control/loop.py — 主控制迴圈（在 daemon thread 內執行）。

依賴 SharedState 進行執行緒間通訊，不使用任何全域變數。
虛擬模式已移除，僅支援實體 QUBE-Servo 3。
"""
import time
import threading
import numpy as np

from control.filters   import LowPassFilter
from control.impedance import ImpedanceDynamics
from control.safety    import SafetyChecker
from control.step      import StepContext, run_step
from config            import FC_SPEED, FC_ACCEL, FC_FORCE


def _try_import_qube():
    try:
        from pal.products.qube import QubeServo3
        return QubeServo3
    except ImportError:
        return None


def control_loop(params: dict, state,
                 log_cb, status_cb,
                 round_done_cb, all_done_cb,
                 new_round_cb, safety_alert_cb):
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
           f"rounds={total_rounds}  warmup={50}cycles")

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

            _run_one_round(
                rnd_idx, total_rounds, exp_time, dt,
                qube, _has_current,
                safety, imp_dyn, state,
                log_cb, round_done_cb, new_round_cb, safety_alert_cb)

    status_cb(False, "None", "")
    log_cb("[INFO] 控制迴圈結束")
    all_done_cb()


def _run_one_round(rnd_idx, total_rounds, exp_time, dt,
                   qube, has_current,
                   safety, imp_dyn, state,
                   log_cb, round_done_cb, new_round_cb, safety_alert_cb):
    """執行一輪實驗，將每輪的設定與清理封裝在此。"""
    safety.reset()
    imp_dyn.reset()

    clear_event = threading.Event()
    new_round_cb(rnd_idx + 1, clear_event)
    clear_event.wait(timeout=2.0)

    ctx = StepContext(
        LowPassFilter(FC_SPEED, dt),
        LowPassFilter(FC_ACCEL, dt),
        LowPassFilter(FC_FORCE, dt),
        imp_dyn,
    )

    with state.data_lock:
        state.round_history.clear()

    log_cb(f"[ROUND {rnd_idx + 1}/{total_rounds}] 開始")

    start_time = time.time()
    timestamp  = 0.0

    while (timestamp < exp_time
           and not state.kill.is_set()
           and not state.emergency.is_set()):

        while state.pause.is_set() and not state.kill.is_set():
            time.sleep(0.05)

        t0 = time.time()

        try:
            qube.read_outputs()
            theta   = float(np.asarray(qube.motorPosition).flat[0])
            current = (float(np.asarray(qube.motorCurrent).flat[0])
                       if has_current else 0.0)

            ctrl_params = state.get_params()
            safe, reason, voltage, row = run_step(
                theta, current, ctx, ctrl_params,
                safety, dt, rnd_idx + 1, timestamp)

            if not safe:
                qube.write_voltage(0.0)
                safety_alert_cb(reason)
                return

            qube.write_voltage(voltage)

            _, _, _, Kp, Kd, theta_d = ctrl_params
            with state.data_lock:
                state.push_buffers(timestamp, theta, theta_d,
                                   row["theta_cmd_rad"], row["omega_rads"],
                                   voltage, row["force_est_Nm"])
                state.round_history.append(row)

        except Exception as e:
            log_cb(f"[ERROR] {type(e).__name__}: {e}")
            try:
                qube.write_voltage(0.0)
            except Exception:
                pass
            safety_alert_cb(f"迴圈例外：{type(e).__name__}: {e}")
            return

        elapsed   = time.time() - t0
        time.sleep(max(0.0, dt - elapsed))
        timestamp = time.time() - start_time

    qube.write_voltage(0.0)

    if not state.kill.is_set() and not state.emergency.is_set():
        with state.data_lock:
            state.round_counter += 1
            state.all_rounds_history.append(list(state.round_history))
            rnd = state.round_counter

        log_cb(f"[ROUND {rnd}] 完成")
        round_done_cb(list(state.round_history), rnd)
