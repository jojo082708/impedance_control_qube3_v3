"""
tests/test_control.py — 核心控制邏輯的單元測試。

執行：
    python -m pytest tests/
"""
import math
import threading
import pytest

from control.filters   import LowPassFilter
from control.impedance import ImpedanceDynamics
from control.safety    import SafetyChecker
from control.step      import StepContext, run_step


# ── LowPassFilter ─────────────────────────────────────────────────────────────

class TestLowPassFilter:
    def test_zero_input_stays_zero(self):
        f = LowPassFilter(fc=20.0, dt=0.002)
        for _ in range(100):
            assert f.update(0.0) == pytest.approx(0.0)

    def test_step_converges(self):
        f = LowPassFilter(fc=20.0, dt=0.002)
        for _ in range(500):
            f.update(1.0)
        assert f.update(1.0) == pytest.approx(1.0, abs=1e-3)

    def test_dt_zero_safe(self):
        # dt=0 should not raise; alpha clamps via max(dt, 1e-9)
        f = LowPassFilter(fc=20.0, dt=0.0)
        assert math.isfinite(f.update(1.0))


# ── ImpedanceDynamics ─────────────────────────────────────────────────────────

class TestImpedanceDynamics:
    def test_zero_force_stays_at_origin(self):
        dyn = ImpedanceDynamics()
        dt = 0.002
        for _ in range(200):
            x = dyn.update(0.0, K=1.0, B=0.5, M=0.05, dt=dt)
        assert x == pytest.approx(0.0, abs=1e-6)

    def test_static_spring_mode(self):
        # M < 1e-6 → x = F_ext / K
        dyn = ImpedanceDynamics()
        x = dyn.update(2.0, K=4.0, B=0.1, M=0.0, dt=0.002)
        assert x == pytest.approx(0.5, abs=1e-6)

    def test_reset_clears_state(self):
        dyn = ImpedanceDynamics()
        for _ in range(100):
            dyn.update(1.0, K=1.0, B=0.1, M=0.05, dt=0.002)
        dyn.reset()
        x = dyn.update(0.0, K=1.0, B=0.1, M=0.05, dt=0.002)
        assert x == pytest.approx(0.0, abs=1e-9)

    def test_constant_force_settles(self):
        # With sufficient damping, x → F/K
        dyn = ImpedanceDynamics()
        K, B, M, F = 2.0, 2.0, 0.05, 1.0
        dt = 0.002
        for _ in range(5000):
            x = dyn.update(F, K=K, B=B, M=M, dt=dt)
        assert x == pytest.approx(F / K, abs=0.01)


# ── SafetyChecker ─────────────────────────────────────────────────────────────

class TestSafetyChecker:
    def _make_checker(self):
        logs = []
        emergency = threading.Event()
        checker = SafetyChecker(log_cb=logs.append, emergency_event=emergency)
        return checker, logs, emergency

    def test_normal_values_pass(self):
        checker, _, _ = self._make_checker()
        # skip warmup
        for _ in range(60):
            checker.check(0.0, 0.0, 0.0, 0.0)
        safe, reason = checker.check(0.1, 0.5, 0.5, 0.1)
        assert safe is True

    def test_angle_limit_triggers(self):
        checker, _, emergency = self._make_checker()
        safe, _ = checker.check(100.0, 0.0, 0.0, 0.0)
        assert safe is False
        assert emergency.is_set()

    def test_speed_limit_triggers(self):
        checker, _, emergency = self._make_checker()
        safe, _ = checker.check(0.0, 60.0, 0.0, 0.0)
        assert safe is False
        assert emergency.is_set()

    def test_current_skipped_during_warmup(self):
        checker, _, emergency = self._make_checker()
        # During warmup, large current should NOT trigger
        safe, _ = checker.check(0.0, 0.0, 100.0, 0.0)
        assert safe is True
        assert not emergency.is_set()

    def test_current_triggers_after_warmup(self):
        checker, _, emergency = self._make_checker()
        for _ in range(60):
            checker.check(0.0, 0.0, 0.0, 0.0)
        safe, _ = checker.check(0.0, 0.0, 100.0, 0.0)
        assert safe is False
        assert emergency.is_set()

    def test_force_zeroed_not_stopped(self):
        checker, _, emergency = self._make_checker()
        for _ in range(60):
            checker.check(0.0, 0.0, 0.0, 0.0)
        safe, reason = checker.check(0.0, 0.0, 0.0, 999.0)
        assert safe is True
        assert reason == "force_zero"
        assert not emergency.is_set()

    def test_reset_clears_cycle_count(self):
        checker, _, _ = self._make_checker()
        for _ in range(60):
            checker.check(0.0, 0.0, 0.0, 0.0)
        checker.reset()
        # After reset, current check is suppressed again (warmup restart)
        safe, _ = checker.check(0.0, 0.0, 100.0, 0.0)
        assert safe is True


# ── run_step ──────────────────────────────────────────────────────────────────

class TestRunStep:
    def _make_ctx(self, dt=0.002):
        dyn = ImpedanceDynamics()
        return StepContext(
            LowPassFilter(40, dt),
            LowPassFilter(15, dt),
            LowPassFilter(15, dt),
            dyn,
        )

    def _make_safety(self):
        emergency = threading.Event()
        checker = SafetyChecker(log_cb=lambda _: None,
                                emergency_event=emergency)
        # advance past warmup
        for _ in range(60):
            checker.check(0.0, 0.0, 0.0, 0.0)
        return checker

    def test_returns_four_tuple(self):
        ctx    = self._make_ctx()
        safety = self._make_safety()
        result = run_step(0.0, 0.0, ctx, (1.0, 0.1, 0.05, 20.0, 0.5, 0.0),
                          safety, 0.002, 1, 0.0)
        assert len(result) == 4

    def test_safe_step_produces_row_with_correct_keys(self):
        from config import ROW_FIELDS
        ctx    = self._make_ctx()
        safety = self._make_safety()
        safe, reason, voltage, row = run_step(
            0.0, 0.0, ctx, (1.0, 0.1, 0.05, 20.0, 0.5, 0.0),
            safety, 0.002, 1, 0.0)
        assert safe is True
        assert row is not None
        assert set(row.keys()) == set(ROW_FIELDS)

    def test_voltage_clamped(self):
        from config import VOLTAGE_LIMIT
        ctx    = self._make_ctx()
        safety = self._make_safety()
        # Very high Kp with large error → voltage should be clipped
        safe, _, voltage, _ = run_step(
            5.0, 0.0, ctx, (1.0, 0.1, 0.05, 1000.0, 0.0, 0.0),
            safety, 0.002, 1, 0.0)
        if safe:
            assert abs(voltage) <= VOLTAGE_LIMIT + 1e-9

    def test_unsafe_angle_returns_false(self):
        ctx    = self._make_ctx()
        safety = self._make_safety()
        safe, reason, voltage, row = run_step(
            100.0, 0.0, ctx, (1.0, 0.1, 0.05, 20.0, 0.5, 0.0),
            safety, 0.002, 1, 0.0)
        assert safe is False
        assert voltage == 0.0
        assert row is None
