"""Backend construction from configuration.

The single place where a backend *type* string becomes a concrete object.
Keeping this in one function means the deployment topology (T1-T4 in
docs/00_architecture.md section 2) is genuinely a config change: nothing
else in the bridge names a concrete backend class.
"""

from __future__ import annotations

import logging
import platform
from typing import Any, Dict, Optional

from stk_bridge.backends.base import BackendType, StkBackend
from stk_bridge.backends.com_backend import ComBackend
from stk_bridge.backends.connect_backend import ConnectBackend
from stk_bridge.backends.engine_backend import EngineBackend
from stk_bridge.backends.mock_backend import MockBackend
from stk_bridge.backends import connect_protocol as proto
from stk_bridge.exceptions import BackendUnavailableError

LOGGER = logging.getLogger(__name__)

_BUILDERS = {
    BackendType.COM: "_build_com",
    BackendType.ENGINE: "_build_engine",
    BackendType.CONNECT: "_build_connect",
    BackendType.MOCK: "_build_mock",
}


def create_backend(config: Optional[Dict[str, Any]] = None) -> StkBackend:
    """Build a backend from a ``backend:`` config block.

    Expected shape (see ``config/bridge_params.yaml``, Phase 6)::

        backend:
          type: com          # com | engine | connect | mock | auto
          com:
            attach_to_existing: true
            start_stk: false   # true: start STK if none is running
            visible: true
            user_control: true
          engine:
            no_graphics: true
          connect:
            host: 192.168.1.50
            port: 5001
            timeout_s: 10.0
          mock:
            satellite_name: SSS-1

    ``type: auto`` picks the best available backend for the current
    platform, preferring fidelity over convenience: COM on Windows,
    Engine elsewhere, and never Mock -- silently substituting fake
    physics for real STK would be the single most dangerous default this
    bridge could have.
    """
    config = config or {}
    backend_config = config.get("backend", config)
    requested = str(backend_config.get("type", "auto")).strip().lower()

    if requested == "auto":
        requested = _autodetect()
        LOGGER.info("backend type 'auto' resolved to %r", requested)

    try:
        backend_type = BackendType(requested)
    except ValueError:
        raise BackendUnavailableError(
            requested,
            f"unknown backend type {requested!r}; valid values are "
            f"{[t.value for t in BackendType]} or 'auto'",
        ) from None

    builder = globals()[_BUILDERS[backend_type]]
    backend = builder(backend_config.get(backend_type.value, {}) or {})
    LOGGER.info("constructed %s", backend)
    return backend


def _autodetect() -> str:
    """Choose a backend for this platform without connecting to anything.

    Deliberately does not probe (no socket attempts, no COM activation):
    autodetection runs before logging is fully configured and a hanging
    probe here would look like a silent startup failure. Probing belongs
    in ``scripts/check_environment.py``, which exists to be run
    explicitly.
    """
    if platform.system() == "Windows":
        return BackendType.COM.value
    return BackendType.ENGINE.value


def _build_com(options: Dict[str, Any]) -> StkBackend:
    return ComBackend(
        attach_to_existing=bool(options.get("attach_to_existing", True)),
        visible=bool(options.get("visible", True)),
        user_control=bool(options.get("user_control", True)),
        start_stk=bool(options.get("start_stk", False)),
    )


def _build_engine(options: Dict[str, Any]) -> StkBackend:
    return EngineBackend(
        no_graphics=bool(options.get("no_graphics", True)),
    )


def _build_connect(options: Dict[str, Any]) -> StkBackend:
    return ConnectBackend(
        host=str(options.get("host", "localhost")),
        port=int(options.get("port", proto.DEFAULT_CONNECT_PORT)),
        timeout_s=float(options.get("timeout_s", 10.0)),
        connect_timeout_s=float(options.get("connect_timeout_s", 5.0)),
    )


def _build_mock(options: Dict[str, Any]) -> StkBackend:
    return MockBackend(
        scenario_epoch=float(options.get("scenario_epoch", 1_717_200_000.0)),
        satellite_name=str(options.get("satellite_name", "SSS-1")),
    )
