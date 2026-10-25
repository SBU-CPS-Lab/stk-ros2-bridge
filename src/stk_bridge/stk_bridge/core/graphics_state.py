"""STK's graphics state during a run, as a recorded run condition.

STK's 2D and 3D Graphics windows redraw on every lockstep step. On
2026-10-15 (code f2f5746) closing every one of them and minimising STK
cut an E6 step from 120.3 to 70.2 ms with an identical determinism
signature, so timing from runs in different graphics states must never
be pooled.

A state is read from STK Desktop through documented, read-only calls
(ComBackend.graphics_state):

    {"windows_2d": int, "windows_3d": int,
     "main_window": "minimized" | "normal" | "maximized", "visible": bool}

"off" is exactly the measured condition -- no 2D window, no 3D window and
the main window minimised; which of those matters was not separated.
Anything else is "on"; no state at all is "unavailable".

The bridge samples at startup, every SUMMARY_EVERY_STEPS steps and at
the terminal epoch, and rewrites the file after each sample (GraphicsLog);
the run orchestrator reads it (summarize).
"""

import json
import os
from typing import Any, Dict, Optional

OFF = "off"
ON = "on"
UNAVAILABLE = "unavailable"
MIXED = "mixed"
UNVERIFIED = "unverified"


def classify(state: Optional[Dict[str, Any]]) -> str:
    if not state or "windows_2d" not in state:
        return UNAVAILABLE
    if (state["windows_2d"] == 0 and state["windows_3d"] == 0
            and state["main_window"] == "minimized"):
        return OFF
    return ON


def describe(entry: Dict[str, Any]) -> str:
    if "windows_2d" not in entry:
        return f"{entry.get('mode', UNAVAILABLE)} ({entry.get('error', '')})"
    return (f"{entry['mode']} (2D windows {entry['windows_2d']}, 3D windows "
            f"{entry['windows_3d']}, main window {entry['main_window']}, "
            f"visible {entry['visible']})")


class GraphicsLog:
    """Samples of one run, written after each so a killed bridge loses
    nothing already measured."""

    def __init__(self, path: str):
        self.path = path
        self.data: Dict[str, Any] = {"startup": None, "samples": [],
                                     "end": None}

    def add(self, phase: str, state: Optional[Dict[str, Any]], step: int,
            error: str = "") -> Dict[str, Any]:
        entry = dict(state or {})
        entry["mode"] = classify(state)
        entry["step"] = step
        if error:
            entry["error"] = error
        if phase == "startup":
            self.data["startup"] = entry
        elif phase == "end":
            self.data["end"] = entry
        else:
            self.data["samples"].append(entry)
        self._write()
        return entry

    def _write(self) -> None:
        if not self.path:
            return
        temporary = f"{self.path}.partial"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(self.data, handle, indent=2)
            handle.write("\n")
        os.replace(temporary, self.path)


def summarize(data: Dict[str, Any]) -> Optional[str]:
    """The run's graphics mode; None when the startup state is missing.

    "mixed" when any sample differs from another, "unverified" when the
    terminal sample is missing (the run did not reach its end, or the
    bridge was stopped first).
    """
    startup = data.get("startup")
    if not startup:
        return None
    end = data.get("end")
    modes = {startup["mode"]}
    modes.update(sample["mode"] for sample in data.get("samples") or [])
    if end:
        modes.add(end["mode"])
    if len(modes) > 1:
        return MIXED
    if not end:
        return UNVERIFIED
    return startup["mode"]


def read(path: str) -> Optional[Dict[str, Any]]:
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None
