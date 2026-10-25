"""What a run that left no record actually was, from its summary.json.

"No record" covers very different things, and only one of them is a
mismatch being detected:

    interrupted              a KeyboardInterrupt in summary.json errors:
                             no result, never a pass, never a fail-stop
    infrastructure_failure   the twin's ROS node/server did not start
                             ("Server failed to start"), or the simulation
                             never reached "running" (run_twin_session's
                             waitFor "not reached"): not a detection;
                             retry it
    guard_fail_stop          the twin stopped on its own attitude guard
                             (adcs:simulink_step:AttitudeMismatch in
                             +adcs/simulink_step.m)
    fail_stop                anything else that ended by itself

Read only from what summary.json already records (errors and the
matlab_session copy of matlab_session.json).
"""

from __future__ import annotations

from typing import Dict

GUARD_ID = "adcs:simulink_step:AttitudeMismatch"
INFRASTRUCTURE_MARKERS = ("Server failed to start",)


def _first_line_with(text: str, needle: str) -> str:
    for line in text.replace("\r", "").splitlines():
        if needle in line:
            return line.strip()
    return ""


def classify(summary: Dict) -> Dict[str, str]:
    """{"kind": ..., "reason": ...} for a run without a record."""
    for error in summary.get("errors") or []:
        if error.get("type") == "KeyboardInterrupt":
            return {"kind": "interrupted",
                    "reason": f"KeyboardInterrupt during step "
                              f"{error.get('step')!r}"}
    session = summary.get("matlab_session") or {}
    diagnostics = str(session.get("simulink_diagnostics") or "")
    error = str(session.get("error") or "")
    text = diagnostics + "\n" + error
    for marker in INFRASTRUCTURE_MARKERS:
        if marker in text:
            return {"kind": "infrastructure_failure",
                    "reason": _first_line_with(text, marker)}
    if "waitFor" in error and "not reached" in error:
        return {"kind": "infrastructure_failure",
                "reason": "the simulation never reached 'running' "
                          "(run_twin_session waitFor not reached)"}
    if GUARD_ID in text:
        detail = _first_line_with(diagnostics or error,
                                  "does not match") or GUARD_ID
        detail = detail.replace("Error: ", "", 1)
        return {"kind": "guard_fail_stop", "reason": f"{GUARD_ID}: {detail}"}
    failed = [s.get("step") for s in summary.get("steps") or []
              if not s.get("ok")]
    return {"kind": "fail_stop",
            "reason": "failed steps: " + ", ".join(map(str, failed))}
