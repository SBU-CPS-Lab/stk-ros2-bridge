"""STK Engine backend: headless, in-process, Windows or Linux.

Subclasses :class:`~stk_bridge.backends.com_backend.ComBackend` because
once a root object exists the Object Model surface is identical -- the
same data providers, the same ``ExecuteCommand``, the same
``SetPosition``/``AddAttitude`` path. Only *acquisition* differs
(``STKEngine.StartApplication`` + ``NewObjectRoot()`` rather than
``STKDesktop`` + ``.Root``), plus the absence of graphics.

Why this matters for STK 12.6
-----------------------------
This is the only way to run the whole bridge natively on Linux without a
Windows machine in the loop, because STK 12.6 has no gRPC remoting. It
requires an STK Engine entitlement, and STK Engine runs in-process with
the Python interpreter -- one engine per process, no attaching to a
separate running instance.

Constraints in no-graphics mode
-------------------------------
The analysis engine is fully functional (propagation, data providers,
access, real-time position and attitude). What is unavailable:

* 2D/3D windows and any VO/graphics Connect command, which NACK.
* Anything that depends on the Desktop UI, including the Attitude
  Simulator's interactive dialogs.

``core/scenario_manager.py`` consults ``capabilities.graphics`` before
issuing anything graphics-related, so a headless run degrades cleanly
rather than accumulating NACKs.
"""

from __future__ import annotations

import logging
import platform
import sys
from typing import Any

from stk_bridge.backends.base import BackendInfo, BackendType
from stk_bridge.backends.com_backend import ComBackend
from stk_bridge.exceptions import BackendUnavailableError

LOGGER = logging.getLogger(__name__)


class EngineBackend(ComBackend):
    """Headless STK Engine, in-process with this Python interpreter."""

    def __init__(self, no_graphics: bool = True):
        # ComBackend's desktop arguments do not apply to an in-process
        # engine; only the backend type and info differ.
        super().__init__(attach_to_existing=False, visible=False,
                         user_control=False)
        self._backend_type = BackendType.ENGINE
        self._no_graphics = no_graphics
        self._info = BackendInfo(backend_type=BackendType.ENGINE)

    def connect(self) -> None:
        if self._connected:
            return

        try:
            from agi.stk12.stkengine import STKEngine
        except ImportError as exc:
            raise BackendUnavailableError(
                BackendType.ENGINE.value,
                "cannot import agi.stk12.stkengine. Install the wheel from "
                "<STK install>/bin/AgPythonAPI. On Linux this also requires "
                "an STK Engine installation and an entitlement that permits "
                f"Engine on Linux. Underlying error: {exc}",
            ) from exc

        try:
            self._stk = STKEngine.StartApplication(
                noGraphics=self._no_graphics
            )
            self._root = self._stk.NewObjectRoot()
        except Exception as exc:
            raise BackendUnavailableError(
                BackendType.ENGINE.value,
                f"failed to start STK Engine: {exc}. Check the "
                f"stk_engine_runtime entitlement and, on Linux, that the "
                f"STK Engine shared libraries are on the loader path.",
            ) from exc

        self._connected = True
        try:
            self._load_enums()
            self._probe_version()
            self._probe_capabilities()
            self._apply_headless_capabilities()
            self.configure_units()
        except Exception:
            self.disconnect()
            raise

        LOGGER.info(
            "STK Engine started in-process (graphics=%s, platform=%s, STK %s)",
            not self._no_graphics, platform.system(), self._info.stk_version,
        )

    def _apply_headless_capabilities(self) -> None:
        """Record what headless mode removes.

        Done after ``_probe_capabilities`` so the licence probe's answers
        stand and only the graphics-dependent flags are overridden.
        """
        if self._no_graphics:
            self._capabilities.graphics = False
            self._capabilities.unlicensed_detail["graphics"] = (
                "STK Engine started with noGraphics=True; 2D/3D windows "
                "and VO Connect commands are unavailable in this session"
            )
        self._capabilities.async_connect = False

    def disconnect(self) -> None:
        """Shut the engine down.

        Unlike the Desktop backend there is no user to inconvenience --
        the engine lives inside this process, so it is always ours to
        terminate, and leaking it would hold the licence until the
        process exits.
        """
        self._connected = False
        self._root = None
        if self._stk is not None:
            try:
                self._stk.ShutDown()
            except Exception:  # pragma: no cover - best-effort teardown
                LOGGER.debug("engine shutdown raised; ignoring", exc_info=True)
            self._stk = None
