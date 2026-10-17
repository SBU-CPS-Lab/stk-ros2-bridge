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
from typing import Callable, Iterable, List, Optional, Tuple

STOP_FILE = "twin.stop"
MATLAB_LOG_FILE = "matlab.log"
MATLAB_SESSION_FILE = "matlab_session.json"
#: MATLAB's console output: written here, never to the shared console,
#: where a paused console could hold the twin.
MATLAB_CONSOLE_FILE = "matlab_console.log"

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
    try:
        subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       check=False, timeout=30)
    except subprocess.TimeoutExpired:
        pass


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
    #: The PID of the process the runner started (the MATLAB launcher).
    pid: Optional[int] = None

    def to_dict(self) -> dict:
        return asdict(self)


class MatlabBatchTwin:
    def __init__(self, run_config: str, log_dir: str, *,
                 popen: Callable[..., "subprocess.Popen"] = subprocess.Popen,
                 kill_tree: Callable[[int], None] = kill_process_tree,
                 sleep: Callable[[float], None] = time.sleep,
                 clock: Callable[[], float] = time.monotonic,
                 matlab: str = "matlab", matlab_dir: str = ""):
        # Absolute here, the one place paths are handed to MATLAB: it runs
        # from src/stk_matlab, where a relative path means another file
        # (live session on d0bf796: simulink_diagnostics.txt not writable,
        # no matlab.log).
        run_config = os.path.abspath(run_config)
        log_dir = os.path.abspath(log_dir)
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
        self._console = None
        self._result: Optional[SessionResult] = None

    def start(self) -> None:
        os.makedirs(self.log_dir, exist_ok=True)
        if os.path.exists(self.stop_file):
            os.remove(self.stop_file)
        self._console = open(os.path.join(self.log_dir, MATLAB_CONSOLE_FILE),
                             "w", encoding="utf-8")
        self.process = self._popen(self.argv, cwd=self.matlab_dir,
                                   stdout=self._console,
                                   stderr=subprocess.STDOUT)

    def exit_code(self) -> Optional[int]:
        """The process's exit code if it has exited, else None."""
        return None if self.process is None else self.process.poll()

    def stop(self, timeout_s: float = STOP_TIMEOUT_S) -> SessionResult:
        """Write the stop file, wait for the exit, kill the tree on timeout.

        Idempotent: a second call returns the first result.
        """
        if self._result is None:
            self._result = self._stop(timeout_s)
            if self._console is not None:
                self._console.close()
        return self._result

    def _stop(self, timeout_s: float) -> SessionResult:
        if self._result is not None:
            return self._result
        if self.process is None:
            self._result = SessionResult("not_started", None)
            return self._result
        pid = getattr(self.process, "pid", None)
        early = self.process.poll()
        if early is not None:
            self._result = SessionResult(
                "early_exit", early,
                "MATLAB exited before the runner asked it to stop", pid)
            return self._result
        with open(self.stop_file, "w", encoding="utf-8"):
            pass
        deadline = self._clock() + timeout_s
        while self._clock() < deadline:
            code = self.process.poll()
            if code is not None:
                self._result = SessionResult(
                    "clean_stop" if code == 0 else "error_exit", code,
                    pid=pid)
                return self._result
            self._sleep(POLL_S)
        self._kill_tree(self.process.pid)
        self._result = SessionResult(
            "timeout_killed", self.process.poll(),
            f"no exit within {timeout_s:.0f} s of the stop file; killed "
            f"the process tree of PID {self.process.pid}", pid)
        return self._result


#: The processes a twin session leaves behind if it does not exit.
TWIN_IMAGES = ("matlab.exe", "libmwros2server.exe")


def processes_of(pids: Iterable[int]) -> List[Tuple[int, str]]:
    """(pid, image name) of the running processes that are ``pids`` or
    were started by one of them (Windows, via Win32_Process)."""
    ids = ",".join(str(int(p)) for p in pids)
    if not ids:
        return []
    script = (f"$ids = @({ids}); Get-CimInstance Win32_Process | "
              "Where-Object { $ids -contains $_.ProcessId -or "
              "$ids -contains $_.ParentProcessId } | "
              "ForEach-Object { \"$($_.ProcessId),$($_.Name)\" }")
    completed = subprocess.run(
        ["powershell", "-NoProfile", "-NonInteractive", "-Command", script],
        capture_output=True, text=True, check=False)
    found = []
    for line in completed.stdout.splitlines():
        pid, _, name = line.strip().partition(",")
        if pid.isdigit() and name:
            found.append((int(pid), name))
    return found


def session_pids(log_dir: str) -> List[int]:
    """The PIDs a run recorded for its twin session: the process the
    runner started (summary.json) and MATLAB's own (matlab_session.json)."""
    import json

    pids = []
    for name, key in (("summary.json", "matlab_session_result"),
                      (MATLAB_SESSION_FILE, None)):
        try:
            with open(os.path.join(log_dir, name), encoding="utf-8") as h:
                data = json.load(h)
        except (OSError, ValueError):
            continue
        pid = (data.get(key) or {}).get("pid") if key else data.get("pid")
        if isinstance(pid, int):
            pids.append(pid)
    return pids


def leftover_twin_processes(log_dir: str,
                            find: Callable[[Iterable[int]],
                                           List[Tuple[int, str]]]
                            = processes_of) -> List[Tuple[int, str]]:
    """Twin processes (TWIN_IMAGES) still running from the run whose logs
    are ``log_dir``: those PIDs, or processes they started. Anything else
    that happens to hold one of those PIDs now is left alone."""
    pids = session_pids(log_dir)
    if not pids:
        return []
    return [(pid, name) for pid, name in find(pids)
            if name.lower() in TWIN_IMAGES]
