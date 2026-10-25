"""Keep the run orchestrators from blocking on their console.

On Windows, a selection in a console window with QuickEdit on pauses
every write to that console until a key is pressed, which can stall a
run for as long as nobody notices. Two measures:

- :func:`disable_quickedit` turns QuickEdit off for the console the
  process runs in (it is a property of the console, shared with the
  campaign and its runs) and returns a function that restores it;
- :class:`BackgroundConsoleHandler` writes log records to the console
  from a daemon thread, so a paused console delays the console's copy of
  a line but never the program; files are still written synchronously.
"""

from __future__ import annotations

import ctypes
import logging
import os
import queue
import sys
import threading
from typing import Callable, Optional, Tuple

STD_INPUT_HANDLE = -10
ENABLE_QUICK_EDIT_MODE = 0x0040
ENABLE_EXTENDED_FLAGS = 0x0080


def disable_quickedit(kernel32=None
                      ) -> Tuple[str, Optional[Callable[[], None]]]:
    """Turn QuickEdit off for this process's console.

    Returns (status, restore): status is "disabled", "already off",
    "no console (...)", "not windows" or "failed (...)"; restore puts the
    previous mode back, and is None when nothing was changed.
    """
    if kernel32 is None:
        if os.name != "nt":
            return "not windows", None
        kernel32 = ctypes.windll.kernel32
    try:
        handle = kernel32.GetStdHandle(STD_INPUT_HANDLE)
        mode = ctypes.c_uint32()
        if not kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            return "no console (standard input is not a console)", None
        original = mode.value
        if not original & ENABLE_QUICK_EDIT_MODE:
            return "already off", None
        wanted = (original & ~ENABLE_QUICK_EDIT_MODE) | ENABLE_EXTENDED_FLAGS
        if not kernel32.SetConsoleMode(handle, wanted):
            return "failed (SetConsoleMode refused)", None
    except Exception as error:                  # noqa: BLE001 - recorded
        return f"failed ({type(error).__name__}: {error})", None

    def restore() -> None:
        kernel32.SetConsoleMode(handle, original)

    return "disabled", restore


class BackgroundConsoleHandler(logging.Handler):
    """A StreamHandler whose writes happen on a daemon thread."""

    _STOP = object()

    def __init__(self, stream=None):
        super().__init__()
        self._stream = stream if stream is not None else sys.stderr
        self._queue: "queue.SimpleQueue" = queue.SimpleQueue()
        self._thread = threading.Thread(target=self._drain, daemon=True,
                                        name="console log writer")
        self._thread.start()

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._queue.put(self.format(record) + "\n")
        except Exception:                       # noqa: BLE001
            self.handleError(record)

    def _drain(self) -> None:
        while True:
            item = self._queue.get()
            if item is self._STOP:
                return
            try:
                self._stream.write(item)
                self._stream.flush()
            except Exception:                   # noqa: BLE001
                pass

    def close(self, timeout_s: float = 2.0) -> None:
        """Let queued lines out for up to ``timeout_s``, never longer."""
        self._queue.put(self._STOP)
        self._thread.join(timeout_s)
        super().close()


def configure_console_logging(level: int = logging.INFO,
                              fmt: str = "%(asctime)s %(levelname)s "
                                         "%(message)s") -> logging.Handler:
    """logging.basicConfig's console output, without blocking on it."""
    handler = BackgroundConsoleHandler()
    handler.setFormatter(logging.Formatter(fmt))
    root = logging.getLogger()
    root.addHandler(handler)
    root.setLevel(level)
    return handler
