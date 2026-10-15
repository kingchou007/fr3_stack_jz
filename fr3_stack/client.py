"""Pose-centric facade over :class:`fr3_stack.Robot` for policy loops."""
from __future__ import annotations

import math
import time
from contextlib import contextmanager
from typing import Iterator, Optional, Sequence, Union

import numpy as np

from .geometry import Pose
from .measure import Recorder
from .robot import MOVE_TO_Q_MAX_SPEED_DEFAULT, Robot
from .state import Observation
from .wire import aslist

ArrayLike = Union[np.ndarray, Sequence[float]]
_AXIS = {"x": 0, "y": 1, "z": 2}

# Arm.move_to settle detection: arrival means the EE is within the pose
# tolerances of the goal (impedance leaves a small steady-state error) and
# every joint speed stays below _SETTLE_DQ for _SETTLE_HOLD s. Held-arm dq
# noise is ~1e-3 rad/s.
_SETTLE_POS  = 0.02   # m
_SETTLE_ROT  = 0.1    # rad
_SETTLE_DQ   = 0.01   # rad/s
_SETTLE_HOLD = 0.1    # s
_SETTLE_POLL = 0.01   # s


class Arm:
    """Pose-centric facade. Drop through to ``arm.robot`` for admittance,
    hybrid, joint impedance, or direct streaming kwargs.
    """

    def __init__(self, host: str, **robot_kwargs):
        self._robot = Robot(host, **robot_kwargs)

    # ---- escape hatch -----------------------------------------------------

    @property
    def robot(self) -> Robot:
        return self._robot

    # ---- lifecycle --------------------------------------------------------

    def connect(self) -> None:
        self._robot.connect()

    def close(self) -> None:
        self._robot.close()

    def __enter__(self) -> "Arm":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- observation ------------------------------------------------------

    def observe(self, *, timeout: float = 5.0) -> Observation:
        """Latest state. Raises if ``state.last_error`` is set (e.g. protective
        stop) — drop to ``arm.robot.state`` for a no-raise view."""
        s = self._robot.wait_for_state(timeout=timeout)
        if s.last_error:
            raise RuntimeError(f"daemon error: {s.last_error}")
        wrench = s.wrench_ft if s.has_ft_sensor else s.wrench_ext
        return Observation(
            pose      = Pose(s.pos.copy(), s.quat_xyzw.copy()),
            q         = s.q.copy(),
            dq        = s.dq.copy(),
            wrench    = wrench.copy(),
            has_ft    = s.has_ft_sensor,
            timestamp = s.timestamp,
        )

    # ---- streaming target -------------------------------------------------

    def send(
        self,
        target: Pose,
        *,
        stiffness: ArrayLike | None = None,
    ) -> None:
        """Push one Cartesian-impedance target. ``stiffness`` is sticky."""
        if stiffness is not None:
            self._robot._cart_cache["K"] = aslist(stiffness, 6)
            self._robot._adm_cache["K"]  = aslist(stiffness, 6)
        self._robot.send_cartesian_impedance(
            target_pos       = target.pos,
            target_quat_xyzw = target.quat,
        )

    # ---- blocking reset ---------------------------------------------------

    def move_to(
        self,
        target: Pose,
        *,
        duration: float = 2.0,
        q_max_speed: float | None = MOVE_TO_Q_MAX_SPEED_DEFAULT,
        settle_timeout: float = 30.0,
    ) -> Observation:
        """Min-jerk move to ``target``; returns the post-move observation.

        The daemon may lengthen the move beyond ``duration`` to keep the peak
        joint speed under ``q_max_speed`` (see ``Robot.send_move_to``), so
        after ``duration`` this keeps blocking until the EE is within
        ``_SETTLE_POS`` / ``_SETTLE_ROT`` of ``target`` and every joint speed
        stays below ``_SETTLE_DQ`` for ``_SETTLE_HOLD`` s. Raises
        ``TimeoutError`` if that has not happened ``settle_timeout`` s after
        ``duration`` (e.g. the arm is blocked by contact).
        """
        self._robot.send_move_to(
            target_pos       = target.pos,
            target_quat_xyzw = target.quat,
            run_time         = duration,
            q_max_speed      = q_max_speed,
        )
        time.sleep(duration)
        deadline = time.monotonic() + settle_timeout
        still_since = None
        while True:
            obs = self.observe()
            now = time.monotonic()
            arrived = obs.pose.approx_equal(
                target, pos_tol=_SETTLE_POS, rot_tol=_SETTLE_ROT)
            if arrived and np.max(np.abs(obs.dq)) < _SETTLE_DQ:
                still_since = now if still_since is None else still_since
                if now - still_since >= _SETTLE_HOLD:
                    return obs
            else:
                still_since = None
            if now > deadline:
                raise TimeoutError(
                    f"move_to: arm not settled at target {settle_timeout:g} s "
                    f"after duration elapsed (max |dq| "
                    f"{np.max(np.abs(obs.dq)):.3g} rad/s, position error "
                    f"{np.linalg.norm(obs.pose.pos - target.pos):.3g} m)")
            time.sleep(_SETTLE_POLL)

    # ---- state switches ---------------------------------------------------

    def hold(self) -> Observation:
        """Lock at the current EE pose. Returns the lock-target observation."""
        obs = self.observe()
        self._robot.send_cartesian_impedance(
            target_pos       = obs.pose.pos,
            target_quat_xyzw = obs.pose.quat,
        )
        return obs

    def relax(self) -> None:
        """Drop torque to gravity-comp only (free-drive)."""
        self._robot.send_idle()

    # ---- parameters (sticky) ---------------------------------------------

    def set_stiffness(
        self,
        K: ArrayLike | None = None,
        D: ArrayLike | None = None,
        *,
        damp_ratio: float | None = None,
    ) -> None:
        """Cache cartesian K/D (also seeds admittance inner loop).

        When only ``K`` and ``damp_ratio`` are given, ``D = 2·sqrt(K)·ratio``
        (assumes M_eff=1, matching default tuning K=200/D=28).
        """
        if K is not None:
            self._robot._cart_cache["K"] = aslist(K, 6)
            self._robot._adm_cache["K"]  = aslist(K, 6)
        if D is None and damp_ratio is not None:
            K_eff = np.asarray(self._robot._cart_cache["K"], dtype=float)
            D = (2.0 * np.sqrt(K_eff) * damp_ratio).tolist()
        if D is not None:
            self._robot._cart_cache["D"] = aslist(D, 6)
            self._robot._adm_cache["D"]  = aslist(D, 6)

    def use_profile(self, name: str, controller: str = "cartesian_impedance") -> None:
        """Switch one controller's profile (``configs/<controller>.<name>.yaml``).

        Defaults to ``cartesian_impedance`` for the common case. Pass
        ``controller="hybrid"`` / ``"admittance"`` / etc. to swap profiles
        for the other controllers without dropping to ``arm.robot.set_profile``.
        """
        self._robot.set_profile(controller, name)

    def set_smoothing(
        self,
        *,
        linear_interp: bool | None = None,
        ema:           bool | None = None,
        filter_alpha:  float | None = None,
    ) -> None:
        """Toggle the two cartesian target-smoothing stages (sticky).

        ``linear_interp``: daemon-side LERP/SLERP between received cmds.
        ``ema``: first-order LP in the controller (~19 ms lag at α=0.05).
        With ``linear_interp=True`` at <5 Hz updates, ``ema=False`` usually
        wins — LERP already gives a continuous 1 kHz target.
        """
        c = self._robot._cart_cache
        if linear_interp is not None: c["linear_interp"] = bool(linear_interp)
        if ema is not None:           c["ema"]           = bool(ema)
        if filter_alpha is not None:  c["filter_alpha"]  = float(filter_alpha)

    # ---- experiment harness ----------------------------------------------

    @contextmanager
    def record(self, *, rate_hz: float = 100.0) -> Iterator[Recorder]:
        """Recording context for the mode-aware helpers (oscillate/step/...).
        Daemon publishes ~200 Hz; rate_hz higher than that gives duplicates."""
        rec = Recorder(rate_hz=rate_hz)
        prev = getattr(self, "_recorder", None)
        self._recorder = rec
        try:
            yield rec
        finally:
            self._recorder = prev

    def _run_loop(self, mode: str, axis_idx: int,
                  duration: float, target_fn) -> None:
        """``target_fn(t, p0_xyz) -> p_d_xyz``."""
        rec: Optional[Recorder] = getattr(self, "_recorder", None)
        rate = rec.rate_hz if rec is not None else 100.0
        period = 1.0 / rate
        if rec is not None:
            rec.mode = mode
            rec.axis = axis_idx

        obs0 = self.observe()
        p0   = obs0.pose.pos.copy()
        quat = obs0.pose.quat.copy()

        t_start = time.monotonic()
        next_t  = t_start
        while True:
            now = time.monotonic()
            t = now - t_start
            if duration > 0.0 and t >= duration:
                break
            p_d = target_fn(t, p0)
            self.send(Pose(p_d, quat))
            if rec is not None:
                obs = self.observe()
                rec.push(t, p_d, obs)
            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.monotonic()

    def oscillate(self, *, axis: str = "z", amp: float = 0.05,
                  freq: float = 0.25, duration: float = 30.0) -> None:
        """Sinusoid on one axis around the current pose. Uses cached stiffness."""
        i = _AXIS[axis]
        def target(t: float, p0: np.ndarray) -> np.ndarray:
            p = p0.copy()
            p[i] += amp * math.sin(2.0 * math.pi * freq * t)
            return p
        self._run_loop("osc", i, duration, target)

    def step(self, *, axis: str = "z", delta: float = 0.05,
             duration: float = 8.0, settle: float = 2.0) -> None:
        """Hold for ``settle`` s, then commit a ``delta`` m step on ``axis``."""
        i = _AXIS[axis]
        def target(t: float, p0: np.ndarray) -> np.ndarray:
            p = p0.copy()
            if t >= settle:
                p[i] += delta
            return p
        self._run_loop("step", i, duration, target)

    def disturb(self, *, duration: float = 30.0) -> None:
        """Hold pose while the operator pushes the EE. Tags recording mode="disturb"."""
        def target(_t: float, p0: np.ndarray) -> np.ndarray:
            return p0
        self._run_loop("disturb", 2, duration, target)
