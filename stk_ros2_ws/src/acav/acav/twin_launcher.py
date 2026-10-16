"""Start and stop the MATLAB twin from the run orchestrator.

``deploy_run_cli --twin-launch matlab-batch`` starts one fresh MATLAB
process per run::

    matlab -batch "run_twin_session('<run config>', '<stop file>')"
        -logfile <run>.logs\\matlab.log

from ``src/stk_matlab``. The driver (run_twin_session.m) builds the twin
for the run config, starts the simulation and runs until the stop file
appears; then, and on any error, it stops the simulation, closes the
model and writes ``<run>.logs\\matlab_session.json`` (MATLAB version,
pid, times, how it exited). :meth:`MatlabBatchTwin.stop` writes the stop
file and waits for the process; if it does not exit in time, it kills
the process tree it started -- by its own PID, never another MATLAB.

The runner's ack wait, the twin identity check and the free-run start
gate stay authoritative: this module only replaces the person who built
the model and pressed Play.
"""

from __future__ import annotations

import os
import subprocess
import time
from dataclasses import asdict, dataclass
from typing import Callable, List, Optional

STOP_FILE = "twin.stop"
MATLAB_LOG_FILE = "matlab.log"
MATLAB_SESSION_FILE = "matlab_session.json"

#: MATLAB start-up plus the twin's build and simulation start, seconds:
#: the larger of two headless runs of run_twin_session on the deploy
#: machine (R2023b Update 6, 2026-10-16: 126.3 s and 118.0 s; build
#: about 91 s of that).
MEASURED_STARTUP_BUILD_S = 126.3
#: The ack wait in matlab-batch mode: twice the measurement, at least
#: 3 minutes. summary.json records the value used.
AUTOMATED_TWIN_ACK_TIMEOUT_S = max(2 * MEASURED_STARTUP_BUILD_S, 180.0)
#: From the stop file to the process's exit; it took 27.7 and 29.3 s in
#: the same runs (closing the model, then MATLAB itself).
STOP_TIMEOUT_S = 120.0
POLL_S = 0.5


def kill_process_tree(pid: int) -> None:
    """Kill ``pid`` and its children (Windows): only that tree."""
    subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                   check=False)


def _default_matlab_dir() -> str:
    acav_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(os.path.dirname(acav_root), "stk_matlab")


@dataclass(frozen=True)
class SessionResult:
    """How the MATLAB process ended.

    ``how``: "clean_stop" (exit 0 after the stop file), "error_exit"
    (non-zero after the stop file), "early_exit" (it exited before the
    stop file was written), "timeout_killed" (it did not exit after the
    stop file and its tree was killed) or "not_started".
    """

    how: str
    exit_code: Optional[int]
    detail: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


class MatlabBatchTwin:
    def __init__(self, run_config: str, log_dir: str, *,
                 popen: Callable[..., "subprocess.Popen"] = subprocess.Popen,
                 kill_tree: Callable[[int], None] = kill_process_tree,
                 sleep: Callable[[float], None] = time.sleep,
                 clock: Callable[[], float] = time.monotonic,
                 matlab: str = "matlab", matlab_dir: str = ""):
        self.log_dir = log_dir
        self.stop_file = os.path.join(log_dir, STOP_FILE)
        self.log_file = os.path.join(log_dir, MATLAB_LOG_FILE)
        self.session_file = os.path.join(log_dir, MATLAB_SESSION_FILE)
        self.matlab_dir = matlab_dir or _default_matlab_dir()
        self._popen, self._kill_tree = popen, kill_tree
        self._sleep, self._clock = sleep, clock
        quoted = [path.replace("'", "''")
                  for path in (run_config, self.stop_file)]
        self.argv: List[str] = [
            matlab, "-batch",
            f"run_twin_session('{quoted[0]}', '{quoted[1]}')",
            "-logfile", self.log_file]
        self.process = None
        self._result: Optional[SessionResult] = None

    def start(self) -> None:
        os.makedirs(self.log_dir, exist_ok=True)
        if os.path.exists(self.stop_file):
            os.remove(self.stop_file)
        self.process = self._popen(self.argv, cwd=self.matlab_dir)

    def exit_code(self) -> Optional[int]:
        """The process's exit code if it has exited, else None."""
        return None if self.process is None else self.process.poll()

    def stop(self, timeout_s: float = STOP_TIMEOUT_S) -> SessionResult:
        """Write the stop file, wait for the exit, kill the tree on timeout.

        Idempotent: a second call returns the first result.
        """
        if self._result is not None:
            return self._result
        if self.process is None:
            self._result = SessionResult("not_started", None)
            return self._result
        early = self.process.poll()
        if early is not None:
            self._result = SessionResult(
                "early_exit", early,
                "MATLAB exited before the runner asked it to stop")
            return self._result
        with open(self.stop_file, "w", encoding="utf-8"):
            pass
        deadline = self._clock() + timeout_s
        while self._clock() < deadline:
            code = self.process.poll()
            if code is not None:
                self._result = SessionResult(
                    "clean_stop" if code == 0 else "error_exit", code)
                return self._result
            self._sleep(POLL_S)
        self._kill_tree(self.process.pid)
        self._result = SessionResult(
            "timeout_killed", self.process.poll(),
            f"no exit within {timeout_s:.0f} s of the stop file; killed "
            f"the process tree of PID {self.process.pid}")
        return self._result
