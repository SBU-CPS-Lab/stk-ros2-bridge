"""Which STK Desktop is running, and whether the operator started it.

An STK Desktop the operator starts is a plain AgUiApplication.exe; one
started through COM (STKDesktop.StartApplication, as build_acav_scenario
did when none was running) is a COM server: its command line ends in
``-Embedding`` and its parent is svchost (DcomLaunch). In the P10 live
session E6 on such an STK stepped at 95.7 ms against 60.0-61.1 ms on the
P9 runs, and a free-run bridge's validation on it ran past 90 s. That is
one run, so a correlation; final runs use an operator-started STK by
protocol, and every COM run records which STK it had.

Windows only (Win32_Process through PowerShell). The script has no
double quotes: Windows PowerShell 5.1 strips them from native-command
arguments.
"""

from __future__ import annotations

import json
import subprocess
from typing import Callable, Dict, List, Optional, Tuple

STK_IMAGE = "AgUiApplication.exe"

_SCRIPT = (
    "$all = @(Get-CimInstance Win32_Process); $names = @{}; "
    "foreach ($p in $all) { $names[[int]$p.ProcessId] = $p.Name }; "
    "$now = Get-Date; "
    "$found = @($all | Where-Object { $_.Name -eq '" + STK_IMAGE + "' } | "
    "ForEach-Object { [pscustomobject]@{ pid = [int]$_.ProcessId; "
    "parent_pid = [int]$_.ParentProcessId; "
    "parent_name = [string]$names[[int]$_.ParentProcessId]; "
    "command_line = [string]$_.CommandLine; "
    "started_at = $_.CreationDate.ToString('yyyy-MM-ddTHH:mm:ss'); "
    "uptime_s = [math]::Round(($now - $_.CreationDate).TotalSeconds, 1) "
    "} }); "
    "ConvertTo-Json -InputObject $found -Compress")


def started_through_com(command_line: str) -> bool:
    """A COM-activated server is started with -Embedding (/Embedding)."""
    tokens = (command_line or "").lower().split()
    return "-embedding" in tokens or "/embedding" in tokens


def started_by(process: Dict) -> str:
    return ("com" if started_through_com(process.get("command_line", ""))
            else "operator")


def stk_processes(run: Callable = subprocess.run) -> List[Dict]:
    """Every running STK Desktop, each with ``started_by``; [] if none
    (or if the query failed: the caller cannot tell those apart, and
    treats both as "cannot identify")."""
    completed = run(
        ["powershell", "-NoProfile", "-NonInteractive", "-Command", _SCRIPT],
        capture_output=True, text=True, check=False)
    try:
        found = json.loads(completed.stdout or "[]")
    except ValueError:
        return []
    if isinstance(found, dict):
        found = [found]
    return [dict(process, started_by=started_by(process))
            for process in found if isinstance(process, dict)]


def assess(processes: List[Dict], allow_com_started: bool
           ) -> Tuple[Dict, bool, str]:
    """(stk_process record, ok, detail) for a COM run's STK.

    One operator-started STK passes. One started through COM, none, or
    several (the bridge attached to one of them, but which is not
    knowable from here) fail unless ``allow_com_started``.
    """
    flag = "--allow-com-started-stk"
    if len(processes) == 1:
        process = dict(processes[0])
        where = (f"STK Desktop pid {process.get('pid')}, started "
                 f"{process.get('started_at')} by "
                 f"{process.get('parent_name') or '?'}")
        if process["started_by"] == "operator":
            return process, True, f"{where}: operator-started"
        detail = (f"{where}, was started through COM (-Embedding), not by "
                  f"the operator; E6 on such an STK ran 95.7 ms against "
                  f"about 60 ms. Close it, start STK Desktop yourself, "
                  f"build the scenario in it, then run")
        if allow_com_started:
            return process, True, f"{detail} -- allowed by {flag}"
        return process, False, f"{detail}; or pass {flag}"
    if not processes:
        problem = (f"no STK Desktop process ({STK_IMAGE}) found, or the "
                   f"process query failed; cannot tell which STK the "
                   f"bridge attached to")
    else:
        problem = (f"{len(processes)} STK Desktop processes are running "
                   f"(pids {', '.join(str(p.get('pid')) for p in processes)})"
                   f"; cannot tell which one the bridge attached to. Close "
                   f"all but the one you started")
    record = {"started_by": "unknown",
              "candidates": [p.get("pid") for p in processes],
              "processes": [dict(p) for p in processes]}
    if allow_com_started:
        return record, True, f"{problem} -- allowed by {flag}"
    return record, False, f"{problem}; or pass {flag}"


def start_mode(record: Optional[Dict]) -> str:
    """"operator", "com", "unknown", or "" for a record without one."""
    return str((record or {}).get("started_by") or "")
