"""
data/state.py — GUI thread 與控制 thread 共享的所有狀態。

取代散落各處的全域變數，集中在 SharedState 一個物件內管理。
- data_lock   保護繪圖 buffer 與歷史資料
- params_lock 保護阻抗控制參數（原子讀寫整組參數）
- kill / pause / emergency 為 threading.Event，取代裸 bool flag
"""
import threading
from collections import deque
from config import BUFFER_SIZE


class SharedState:
    def __init__(self):
        # ── 執行緒同步 ─────────────────────────────────────────────────────
        self.data_lock   = threading.Lock()
        self.params_lock = threading.Lock()
        self.kill        = threading.Event()
        self.pause       = threading.Event()
        self.emergency   = threading.Event()

        # ── 繪圖 buffer（需在 data_lock 內讀寫）──────────────────────────
        self.time_buf      = deque(maxlen=BUFFER_SIZE)
        self.pos_buf       = deque(maxlen=BUFFER_SIZE)
        self.desired_buf   = deque(maxlen=BUFFER_SIZE)
        self.voltage_buf   = deque(maxlen=BUFFER_SIZE)
        self.speed_buf     = deque(maxlen=BUFFER_SIZE)
        self.force_est_buf = deque(maxlen=BUFFER_SIZE)
        self.cmd_pos_buf   = deque(maxlen=BUFFER_SIZE)

        # ── 實驗歷史資料（需在 data_lock 內讀寫）──────────────────────────
        self.round_history      = []
        self.all_rounds_history = []
        self.round_counter      = 0

        # ── 即時控制參數（需在 params_lock 內讀寫）────────────────────────
        self._K       = 1.0
        self._B       = 0.1
        self._M       = 0.05
        self._Kp      = 20.0
        self._Kd      = 0.5
        self._theta_d = 0.0

    # ── 控制參數原子存取 ───────────────────────────────────────────────────────

    def get_params(self) -> tuple:
        """一次原子讀取所有控制參數，避免同一週期讀到不同時間點的值。"""
        with self.params_lock:
            return (self._K, self._B, self._M,
                    self._Kp, self._Kd, self._theta_d)

    def set_params(self, K, B, M, Kp, Kd, theta_d):
        with self.params_lock:
            self._K = K; self._B = B; self._M = M
            self._Kp = Kp; self._Kd = Kd; self._theta_d = theta_d

    # ── Buffer 操作（呼叫前需持有 data_lock）─────────────────────────────────

    def clear_buffers(self):
        self.time_buf.clear()
        self.pos_buf.clear()
        self.desired_buf.clear()
        self.voltage_buf.clear()
        self.speed_buf.clear()
        self.force_est_buf.clear()
        self.cmd_pos_buf.clear()

    def push_buffers(self, t, theta, theta_d, theta_cmd, omega, voltage, force_est):
        self.time_buf.append(t)
        self.pos_buf.append(theta)
        self.desired_buf.append(theta_d)
        self.cmd_pos_buf.append(theta_cmd)
        self.voltage_buf.append(voltage)
        self.speed_buf.append(omega)
        self.force_est_buf.append(force_est)

    # ── 實驗開始時重置 ────────────────────────────────────────────────────────

    def reset_for_experiment(self):
        """開始新實驗前呼叫，清空歷史資料並重置所有旗標。"""
        with self.data_lock:
            self.round_history.clear()
            self.all_rounds_history.clear()
            self.round_counter = 0
        self.kill.clear()
        self.pause.clear()
        self.emergency.clear()
