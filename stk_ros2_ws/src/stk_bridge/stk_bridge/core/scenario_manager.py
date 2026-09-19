"""Scenario lifecycle and object management.

Owns everything that happens once at setup rather than per control step:
loading or creating a scenario, ensuring objects exist and are
propagated, checking that required capabilities are licensed, and
running the convention validation that gates startup.

The propagation rule
--------------------
A satellite created through ``Children.New`` has a propagator and default
orbital elements but **no ephemeris**, and every data provider on it
returns zero rows until ``Propagate()`` runs. That is not obvious, it
raises nothing, and it cost a full debugging cycle during bring-up. So
:meth:`ScenarioManager.ensure_propagated` is called on every object the
sensing pipeline will read, and its result is asserted rather than
assumed.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from stk_bridge.backends.base import (
    ObjectInfo,
    PropagatorType,
    StkBackend,
    StkTime,
)
from stk_bridge.core import conversions as conv
from stk_bridge.core import data_providers as dp
from stk_bridge.core.frames import FrameRegistry, validate_against_backend
from stk_bridge.core.validation import ValidationReport, validate_backend
from stk_bridge.exceptions import (
    LicenseError,
    ObjectNotFoundError,
    QueryError,
    StkBridgeError,
)

LOGGER = logging.getLogger(__name__)


@dataclass
class ScenarioConfig:
    #: Path to an existing .sc to load; empty creates unless use_existing.
    scenario_path: str = ""
    scenario_name: str = "SSS1_DigitalTwin"
    #: Objects to create if absent, as {"type": ..., "name": ...}.
    ensure_objects: List[Dict[str, str]] = field(default_factory=list)
    #: Objects to place under real-time control.
    realtime_objects: List[str] = field(default_factory=list)
    look_ahead_s: float = 3600.0
    look_behind_s: float = 3600.0
    require_astrogator: bool = False
    require_coverage: bool = False
    require_communications: bool = False
    use_existing: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.use_existing, bool):
            raise ValueError("scenario.use_existing must be a boolean")
        if self.use_existing and self.scenario_path:
            raise ValueError("scenario.use_existing conflicts with scenario_path")
        if self.use_existing and not self.scenario_name.strip():
            raise ValueError("scenario.use_existing requires scenario_name")

    @classmethod
    def from_config(cls, config: Optional[Dict] = None) -> "ScenarioConfig":
        section = (config or {}).get("scenario", {}) or {}
        return cls(
            use_existing=section.get("use_existing", False),
            scenario_path=str(section.get("scenario_path", "")),
            scenario_name=str(section.get("scenario_name",
                                          "SSS1_DigitalTwin")),
            ensure_objects=list(section.get("ensure_objects") or []),
            realtime_objects=[
                conv.canonical_object_path(p)
                for p in (section.get("realtime_objects") or [])
            ],
            look_ahead_s=float(section.get("look_ahead_s", 3600.0)),
            look_behind_s=float(section.get("look_behind_s", 3600.0)),
            require_astrogator=bool(section.get("require_astrogator", False)),
            require_coverage=bool(section.get("require_coverage", False)),
            require_communications=bool(
                section.get("require_communications", False)),
        )


class ScenarioManager:
    """Brings a scenario to the state the bridge needs, then keeps track."""

    def __init__(self, backend: StkBackend, frames: FrameRegistry,
                 config: ScenarioConfig):
        self._backend = backend
        self._frames = frames
        self._config = config
        self._epoch: Optional[StkTime] = None
        self._objects: Dict[str, ObjectInfo] = {}
        self._propagated: Dict[str, bool] = {}
        self._report = ValidationReport()

    # -- properties --------------------------------------------------------

    @property
    def epoch(self) -> StkTime:
        if self._epoch is None:
            raise RuntimeError("scenario has not been set up")
        return self._epoch

    @property
    def validation_report(self) -> ValidationReport:
        return self._report

    def objects(self) -> Dict[str, ObjectInfo]:
        return dict(self._objects)

    # -- setup -------------------------------------------------------------

    def setup(self, run_validation: bool = True,
              validation_config: Optional[Dict] = None) -> ValidationReport:
        """Full startup sequence. Returns the validation report.

        Order matters: capabilities are checked before anything is
        created, so an unlicensed install fails in two seconds with a
        clear message rather than part-way through building a scenario.
        """
        self._check_capabilities()
        self._open_scenario()
        self._epoch = self._backend.get_scenario_epoch()
        self._refresh_objects()
        self._ensure_objects()
        self._check_frames()

        if run_validation:
            self._report = validate_backend(
                self._backend, "", validation_config or {}
            )
            self._report.raise_if_failed()
            LOGGER.info("convention validation passed:\n%s",
                        self._report.summary())

        self._prepare_realtime_objects()
        return self._report

    def _check_capabilities(self) -> None:
        capabilities = self._backend.capabilities
        capabilities.require("integration")
        if self._config.require_astrogator:
            capabilities.require("astrogator")
        if self._config.require_coverage:
            capabilities.require("coverage")
        if self._config.require_communications:
            capabilities.require("communications")

        if self._config.realtime_objects and not capabilities.realtime_propagator:
            raise LicenseError(
                "realtime_propagator",
                "required because scenario.realtime_objects is non-empty; "
                "without it only the batch StkExternal ephemeris path is "
                "available and there is no live control loop",
            )

    def _open_scenario(self) -> None:
        if self._config.use_existing:
            current = self._backend.get_current_scenario_name()
            expected = self._config.scenario_name
            if current is None:
                raise QueryError("CurrentScenario", "/",
                                 f"no scenario is open; build/open {expected!r} first")
            if current != expected:
                raise QueryError("CurrentScenario", "/",
                                 f"expected {expected!r}, but {current!r} is open; "
                                 "the existing scenario was not replaced")
            LOGGER.info("reusing open scenario %r", current)
        elif self._config.scenario_path:
            LOGGER.info("loading scenario %s", self._config.scenario_path)
            self._backend.load_scenario(self._config.scenario_path)
        else:
            LOGGER.info("creating scenario %r", self._config.scenario_name)
            self._backend.new_scenario(self._config.scenario_name)

    def _refresh_objects(self) -> None:
        self._objects = {
            conv.canonical_object_path(info.object_path): info
            for info in self._backend.list_objects()
        }

    def _ensure_objects(self) -> None:
        for spec in self._config.ensure_objects:
            object_type = str(spec.get("type", "Satellite"))
            name = str(spec.get("name", "")).strip()
            if not name:
                continue
            parent = str(spec.get("parent", ""))
            path = conv.child_path(parent or "*", object_type, name)
            if path in self._objects:
                LOGGER.debug("%s already exists", path)
                continue
            created = self._backend.create_object(object_type, name, parent)
            LOGGER.info("created %s", created)
        self._refresh_objects()

    def _check_frames(self) -> None:
        """Warn about configured frames this install does not offer.

        Frame group membership varies by STK version -- "J2000" is a
        valid Cartesian Position member on some installs and absent on
        12.6 -- and an unavailable frame fails only on first query,
        typically inside a running loop.
        """
        sample = next(iter(self._objects), None)
        if sample is None:
            return
        availability = validate_against_backend(self._frames, self._backend,
                                                sample)
        missing = [name for name, ok in availability.items() if not ok]
        if missing:
            LOGGER.warning(
                "frames declared in the registry that this STK install does "
                "not offer as Cartesian Position group members: %s. "
                "Requesting one of these will fail at query time.", missing,
            )

    def _prepare_realtime_objects(self) -> None:
        for path in self._config.realtime_objects:
            if path not in self._objects:
                raise ObjectNotFoundError(path)
            self._backend.set_propagator(
                path, PropagatorType.REALTIME,
                look_ahead_s=self._config.look_ahead_s,
                look_behind_s=self._config.look_behind_s,
            )
            self._backend.enable_realtime_attitude(path)
            self._propagated[path] = False
            LOGGER.info(
                "%s placed under real-time control; its analytic ephemeris "
                "is discarded and state now comes only from injected "
                "samples", path,
            )
        self._refresh_objects()

    # -- propagation -------------------------------------------------------

    def ensure_propagated(self, object_path: str,
                          verify: bool = True) -> bool:
        """Generate ephemeris for an object, and confirm it worked.

        Returns True if the object has queryable state afterwards. For
        real-time objects this returns False until samples are injected,
        which is correct rather than a failure.
        """
        path = conv.canonical_object_path(object_path)
        if self._propagated.get(path):
            return True

        info = self._objects.get(path)
        is_realtime = bool(info and (info.propagator_type or "").lower()
                           in ("realtime", "stkrealtime"))

        propagate = getattr(self._backend, "propagate", None)
        if propagate is not None and not is_realtime:
            propagate(path)

        if not verify:
            self._propagated[path] = True
            return True

        try:
            data, _ = dp.execute(
                self._backend, dp.CARTESIAN_POSITION, path,
                frame=self._frames.stk_name("icrf"),
                start=self.epoch, stop=self.epoch,
                allow_empty=is_realtime,
            )
        except (QueryError, StkBridgeError) as exc:
            LOGGER.warning("%s has no queryable state after propagation: %s",
                           path, exc)
            self._propagated[path] = False
            return False

        ok = data.row_count > 0
        self._propagated[path] = ok
        if not ok and is_realtime:
            LOGGER.info(
                "%s is on the RealTime propagator and has no samples yet; "
                "state will appear once commands are injected", path,
            )
        return ok

    def ensure_all_propagated(self, object_paths: List[str]) -> Dict[str, bool]:
        return {path: self.ensure_propagated(path) for path in object_paths}

    # -- runtime operations ------------------------------------------------

    def create_object(self, object_type: str, name: str,
                      parent_path: str = "") -> str:
        path = self._backend.create_object(object_type, name, parent_path)
        self._refresh_objects()
        return conv.canonical_object_path(path)

    def delete_object(self, object_path: str) -> None:
        path = conv.canonical_object_path(object_path)
        self._backend.delete_object(path)
        self._objects.pop(path, None)
        self._propagated.pop(path, None)

    def reset(self, clear_realtime_data: bool = True) -> None:
        """Return the scenario to its post-setup state."""
        if clear_realtime_data:
            for path in self._config.realtime_objects:
                try:
                    self._backend.clear_attitude_data(path)
                except StkBridgeError:
                    LOGGER.debug("could not clear attitude on %s", path,
                                 exc_info=True)
        self._backend.set_current_time(self.epoch)
        self._propagated.clear()
        LOGGER.info("scenario reset to epoch")

    def statistics(self) -> Dict:
        return {
            "scenario_name": self._config.scenario_name,
            "epoch": self._epoch.epoch_seconds if self._epoch else None,
            "object_count": len(self._objects),
            "propagated": dict(self._propagated),
            "attitude_provider": self._report.attitude_provider,
            "angular_rate_provider": self._report.angular_rate_provider,
            "angle_rate_scale": self._report.angle_rate_scale.source,
        }
