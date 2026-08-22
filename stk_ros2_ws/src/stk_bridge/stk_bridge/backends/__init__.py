"""STK access backends.

This is the *only* subpackage permitted to import ``agi.stk12``,
``win32com``, ``pythoncom``, or socket-level Connect machinery. Everything
above it codes against :class:`~stk_bridge.backends.base.StkBackend`.

Available implementations:

===========  ==========================  ================================
Type         Transport                   Use
===========  ==========================  ================================
``com``      in-process COM (Desktop)    Default. Windows, full fidelity.
``engine``   in-process STK Engine       Headless; Windows or Linux.
``connect``  TCP socket, port 5001       Cross-machine / cross-OS.
``mock``     none (analytic Kepler)      CI and STK-free development.
===========  ==========================  ================================

Construct via :func:`~stk_bridge.backends.factory.create_backend` rather
than instantiating a class directly, so that deployment topology stays a
configuration concern.

Note that ``com_backend`` and ``engine_backend`` import ``agi.stk12``
lazily, inside ``connect()``, so that importing this package on a machine
without STK installed succeeds. That is what lets the mock backend and
the whole test suite run anywhere.
"""

from stk_bridge.backends.base import (  # noqa: F401
    AccessIntervalData,
    AngularRateSample,
    AttitudeSample,
    BackendCapabilities,
    BackendInfo,
    BackendType,
    ConnectResponse,
    DataProviderData,
    DataProviderQuery,
    EclipseCondition,
    EclipseSample,
    LLASample,
    LinkBudgetData,
    ObjectInfo,
    PropagatorType,
    StateSample,
    StkBackend,
    StkTime,
)
from stk_bridge.backends.factory import create_backend  # noqa: F401

__all__ = [
    "AccessIntervalData",
    "AngularRateSample",
    "AttitudeSample",
    "BackendCapabilities",
    "BackendInfo",
    "BackendType",
    "ConnectResponse",
    "DataProviderData",
    "DataProviderQuery",
    "EclipseCondition",
    "EclipseSample",
    "LLASample",
    "LinkBudgetData",
    "ObjectInfo",
    "PropagatorType",
    "StateSample",
    "StkBackend",
    "StkTime",
    "create_backend",
]
