"""Backend-agnostic domain logic.

Everything here codes against ``backends.StkBackend`` and never imports
``agi.stk12``, ``win32com``, or socket-level Connect machinery. ROS
imports are lazy (inside functions) so most of this package is
unit-testable without a ROS installation.

Modules
-------
``conversions``       Units, quaternions, ROS time, STK path canonicalisation.
``frames``            Frame naming, aliasing, tf tree, command-frame validation.
``data_providers``    Typed provider specs, element resolution, discovery.
``rate_group``        Deterministic multi-rate scheduling on simulation time.
``com_thread``        Single COM-apartment worker; the concurrency model.
``validation``        Startup self-tests for units, frames, and quaternion order.
``time_sync``         Execution modes, clock authority, lockstep barrier.
``sensing``           STK data providers -> typed samples.
``actuation``         ROS commands -> STK, with monotonicity guards.
``scenario_manager``  Scenario lifecycle, propagation, capability gating.
``health``            Bridge state machine and reconnection policy.
``ephemeris_writer``  STK .e / CCSDS OEM writers for the batch path.
"""

from stk_bridge.core import conversions  # noqa: F401
from stk_bridge.core.actuation import ActuationDispatcher  # noqa: F401
from stk_bridge.core.com_thread import BackendWorker, QueueFullPolicy  # noqa: F401
from stk_bridge.core.frames import FrameRegistry  # noqa: F401
from stk_bridge.core.health import BridgeState, HealthConfig, HealthMonitor  # noqa: F401
from stk_bridge.core.rate_group import RateGroup, RateGroupScheduler  # noqa: F401
from stk_bridge.core.scenario_manager import (  # noqa: F401
    ScenarioConfig,
    ScenarioManager,
)
from stk_bridge.core.sensing import SensingConfig, SensingPipeline  # noqa: F401
from stk_bridge.core.time_sync import (  # noqa: F401
    BarrierTimeoutPolicy,
    ExecutionMode,
    LockstepBarrier,
    TimeSync,
    TimeSyncConfig,
)
from stk_bridge.core.validation import (  # noqa: F401
    ConventionValidator,
    ValidationReport,
    validate_backend,
)

__all__ = [
    "ActuationDispatcher", "BackendWorker", "BarrierTimeoutPolicy",
    "BridgeState", "ConventionValidator", "ExecutionMode", "FrameRegistry",
    "HealthConfig", "HealthMonitor", "LockstepBarrier", "QueueFullPolicy",
    "RateGroup", "RateGroupScheduler", "ScenarioConfig", "ScenarioManager",
    "SensingConfig", "SensingPipeline", "TimeSync", "TimeSyncConfig",
    "ValidationReport", "conversions", "validate_backend",
]
