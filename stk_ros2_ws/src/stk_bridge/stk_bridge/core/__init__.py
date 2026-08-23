"""Backend-agnostic domain logic.

Everything here codes against ``backends.StkBackend`` and never imports
``agi.stk12``, ``win32com``, or socket-level Connect machinery. ROS
imports are lazy (inside functions) so that the majority of this package
is unit-testable without a ROS installation.

Phase 3 modules
---------------
``conversions``     Units, quaternions, ROS time, STK path canonicalisation.
``frames``          Frame naming, aliasing, tf tree, command-frame validation.
``data_providers``  Typed provider specs, element-name resolution, empty-result
                    detection.
``rate_group``      Deterministic multi-rate scheduling on simulation time.
``com_thread``      Single COM-apartment worker; the bridge's whole
                    concurrency model.
``validation``      Startup self-tests for units, quaternion order, and data
                    presence.

Phase 4 adds ``time_sync``, ``sensing``, ``actuation``, ``scenario_manager``,
and ``health``.
"""

from stk_bridge.core import conversions  # noqa: F401
from stk_bridge.core.com_thread import BackendWorker, QueueFullPolicy  # noqa: F401
from stk_bridge.core.frames import FrameRegistry  # noqa: F401
from stk_bridge.core.rate_group import RateGroup, RateGroupScheduler  # noqa: F401
from stk_bridge.core.validation import (  # noqa: F401
    ConventionValidator,
    ValidationReport,
    validate_backend,
)

__all__ = [
    "BackendWorker",
    "ConventionValidator",
    "FrameRegistry",
    "QueueFullPolicy",
    "RateGroup",
    "RateGroupScheduler",
    "ValidationReport",
    "conversions",
    "validate_backend",
]
