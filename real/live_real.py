"""dimos's live backend with gripper calls that wait for the jaw: the default of ``run_program_real.py``.

``Arm.open_gripper``/``close_gripper``/``set_gripper_position`` return as soon as the command is accepted, and the
sim-found programs wait for the jaw with a fixed number of ``arm.pose()`` reads (v24: 30 after a release). That
"dwell" is only as long as the RPC round trips: about 30 ms each on the loaded sim lab where v24 was tuned, about 5 ms
on a lighter stack, so 30 reads shrink from about 1 s to 0.15 s and the arm starts rising while the jaw is still
opening. In the MuJoCo measured-scene runs that dragged and tipped the just-placed cube. Here each gripper call
returns only once the jaw has stopped moving (``gripper_position`` within ``SETTLE_TOL`` for ``SETTLE_S``), after at
least ``MIN_S`` and at most ``MAX_S`` seconds. Everything else is ``dimos.autoresearch.live:backend`` unchanged.
"""

from __future__ import annotations

from pathlib import Path
import time

from dimos.autoresearch.live import backend as live_backend
from dimos.autoresearch.program import DimosBackend, RobotBackend

MIN_S = 0.3
MAX_S = 3.0
SETTLE_S = 0.25
SETTLE_TOL = 0.002  # in the units gripper_position reports (metres of joint7, or normalized travel)
POLL_S = 0.03


class JawWaitingBackend:
    def __init__(self, inner: DimosBackend) -> None:
        self._inner = inner
        self._arm = inner._arm

    def __getattr__(self, name: str):
        return getattr(self._inner, name)

    def _jaw(self) -> float | None:
        try:
            return self._arm.state().gripper_position
        except Exception:
            return None

    def _wait_for_jaw(self) -> None:
        start = time.monotonic()
        last, still_since = self._jaw(), time.monotonic()
        while time.monotonic() - start < MAX_S:
            time.sleep(POLL_S)
            now_value = self._jaw()
            now = time.monotonic()
            if now_value is None or last is None or abs(now_value - last) > SETTLE_TOL:
                last, still_since = now_value, now
            elif now - still_since >= SETTLE_S and now - start >= MIN_S:
                return

    def open_gripper(self) -> None:
        self._inner.open_gripper()
        self._wait_for_jaw()

    def close_gripper(self) -> None:
        self._inner.close_gripper()
        self._wait_for_jaw()

    def set_gripper(self, joint_value: float) -> None:
        self._inner.set_gripper(joint_value)
        self._wait_for_jaw()


def backend(trial_dir: Path) -> RobotBackend:
    """The factory ``run_program`` names: the live backend, with gripper calls that wait for the jaw."""
    return JawWaitingBackend(live_backend(trial_dir))  # type: ignore[return-value]
