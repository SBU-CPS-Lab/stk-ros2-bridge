"""STK Object Model backend via the ``agi.stk12`` Python API (STK Desktop).

The recommended backend when the bridge can run on the same Windows host
as STK 12.6. Uses in-process COM, gives typed access to Data Providers
(no text parsing), and supports bulk extraction via ``ToNumpyArray()``,
available from STK 12.6 onward.

Threading
---------
COM is apartment-threaded. Every method here must be called from the same
thread that called :meth:`connect`, and that thread must have initialised
COM. ``core/com_thread.py`` (Phase 3) owns that thread and marshals work
onto it; this class deliberately takes no locks so that a violation
surfaces as a clear COM error during development rather than being masked
into an intermittent deadlock in production.

Version drift
-------------
This is the file most exposed to STK-version differences: enum names,
data-provider group structure, and a few property spellings changed
across the 12.x line, and no model has reliable recall of 12.6's exact
type library. Every such touch point is routed through
:meth:`_resolve_enum` or an explicit try/except with a named fallback,
and :meth:`_probe_capabilities` records what it could not resolve. Run
``scripts/check_environment.py`` (below) against your install before
trusting this module; ``core/validation.py`` (Phase 3) additionally
cross-checks conventions at every startup.
"""

from __future__ import annotations

import logging
import math
import platform
import sys
import time
from contextlib import contextmanager
from typing import Any, Dict, List, Optional, Tuple

from stk_bridge.backends.base import (
    AccessIntervalData,
    BackendCapabilities,
    BackendInfo,
    BackendType,
    ConnectResponse,
    DataProviderData,
    DataProviderQuery,
    LinkBudgetData,
    ObjectInfo,
    PropagatorType,
    StkBackend,
    StkTime,
)
from stk_bridge.backends import connect_protocol as proto
from stk_bridge.exceptions import (
    BackendUnavailableError,
    CommandError,
    ConnectionLostError,
    ObjectNotFoundError,
    QueryError,
)

LOGGER = logging.getLogger(__name__)


class ComBackend(StkBackend):
    """In-process COM access to STK Desktop through ``agi.stk12``."""

    def __init__(self, attach_to_existing: bool = True,
                 visible: bool = True, user_control: bool = True,
                 start_stk: bool = False):
        super().__init__(BackendType.COM)
        self._attach_to_existing = attach_to_existing
        #: Start an STK when there is none to attach to. Off by default:
        #: an STK started through COM ran E6 at 95.7 ms against about
        #: 60 ms (P10 live session), and runs must use the operator's.
        self._start_stk = start_stk
        self._visible = visible
        self._user_control = user_control
        self._stk: Any = None
        self._root: Any = None
        self._enums: Dict[str, Any] = {}
        self._info = BackendInfo(backend_type=BackendType.COM)
        # COM handles only, never telemetry values. Owned by the COM worker.
        self._provider_cache = {}
        # Structural revision; see StkBackend.revision.
        self._revision = 0
        self._query_timings = {}
        self._query_timings_since = time.perf_counter()
        self._extraction_counts = {"bulk": 0, "fallback": 0, "bulk_failed": 0}
        self._extraction_timings = {}
        self._extraction_since = time.perf_counter()

    # -- lifecycle ---------------------------------------------------------

    def _obtain_application(self):
        """The operator's STK Desktop; a new one only when configured.

        Attaching is the only default. Starting STK (start_stk, or
        attach_to_existing false) is logged as a warning: that STK is a
        COM server (AgUiApplication.exe -Embedding), which the run
        orchestrator refuses for reportable runs.
        """
        from agi.stk12.stkdesktop import STKDesktop

        if self._attach_to_existing:
            try:
                application = STKDesktop.AttachToApplication()
                LOGGER.info("attached to a running STK Desktop instance")
                return application
            except Exception as exc:
                if not self._start_stk:
                    raise BackendUnavailableError(
                        BackendType.COM.value,
                        "no running STK Desktop to attach to. Start STK "
                        "Desktop yourself (runs must use an STK the "
                        "operator started), or set backend.com.start_stk: "
                        "true (build/verify scripts: --start-stk) to let "
                        f"this start one. Underlying error: {exc}",
                    ) from exc
        try:
            application = STKDesktop.StartApplication(
                visible=self._visible, userControl=self._user_control)
        except Exception as exc:
            raise BackendUnavailableError(
                BackendType.COM.value, f"failed to start STK Desktop: {exc}"
            ) from exc
        LOGGER.warning(
            "STARTED A NEW STK Desktop through COM (%s). It is not an "
            "operator-started STK: the run orchestrator refuses reportable "
            "runs on it unless --allow-com-started-stk.",
            "start_stk" if self._attach_to_existing
            else "attach_to_existing is false")
        return application

    def connect(self) -> None:
        if self._connected:
            return
        if platform.system() != "Windows":
            raise BackendUnavailableError(
                BackendType.COM.value,
                "the COM backend requires Windows (STK Desktop's Object "
                "Model is COM-based). On Linux use the 'engine' backend "
                "with STK Engine, or the 'connect' backend against a "
                "remote Windows STK.",
            )

        try:
            import agi.stk12.stkdesktop  # noqa: F401 - is it installed?
        except ImportError as exc:
            raise BackendUnavailableError(
                BackendType.COM.value,
                "cannot import agi.stk12. Install the wheel shipped with "
                "STK 12.6 from <STK install>\\bin\\AgPythonAPI (see "
                "docs/01_installation_windows.md). Note STK 12.6's Python "
                f"API is in-process only -- there is no gRPC remoting until "
                f"STK 12.8. Underlying error: {exc}",
            ) from exc

        self._stk = self._obtain_application()
        try:
            self._root = self._stk.Root
        except Exception as exc:
            raise BackendUnavailableError(
                BackendType.COM.value, f"failed to obtain STK root: {exc}"
            ) from exc

        self._connected = True
        try:
            self._load_enums()
            self._probe_version()
            self._probe_capabilities()
            self.configure_units()
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        self._flush_extraction_profile()
        self._provider_cache.clear()
        self._revision += 1
        self._connected = False
        self._root = None
        if self._stk is not None:
            try:
                # ShutDown() only if we own the process; if we attached to
                # a user's running STK, closing it out from under them
                # would be hostile. userControl=True on StartApplication
                # already means STK stays up for the user.
                if not self._attach_to_existing and not self._user_control:
                    self._stk.ShutDown()
            except Exception:  # pragma: no cover - best-effort teardown
                LOGGER.debug("STK shutdown raised; ignoring", exc_info=True)
            self._stk = None

    def configure_units(self) -> None:
        """Force STK's unit preferences to SI, then check they took.

        ``SetCurrentUnit`` raises rather than returning a status if the
        dimension or unit name is unknown, and the exact spellings have
        shifted slightly across STK versions, so each is attempted
        independently and failures are logged rather than aborting the
        connection -- a session missing one unit preference is degraded,
        not unusable.

        Two things measured on a live STK 12.6 install shape this:

        * **Latitude and Longitude are their own dimensions**, separate
          from Angle. Setting Angle to radians succeeded and left "LLA
          State" reporting *degrees*, because that provider's columns
          are Latitude/Longitude-dimensioned. The bridge reported SI
          units while handing out degrees, and only a case-study check
          that compared LLA against a Cartesian position caught it.
        * **Acceptance is not application.** Every preference is now
          read back, because the failure above raised nothing and
          logged a success.
        """
        preferences = (
            ("Distance", "m"),
            ("Time", "sec"),
            ("Angle", "rad"),
            # Separate dimensions from Angle. Without these, every
            # geodetic latitude and longitude the bridge reports is in
            # degrees while claiming to be radians.
            ("Latitude", "rad"),
            ("Longitude", "rad"),
            ("Mass", "kg"),
            ("DateFormat", "UTCG"),
            ("AngleRate", "rad/sec"),
        )
        applied, failed = [], []
        for dimension, unit in preferences:
            try:
                self._root.UnitPreferences.SetCurrentUnit(dimension, unit)
                applied.append(f"{dimension}={unit}")
            except Exception as exc:
                failed.append(f"{dimension}={unit} ({exc})")

        LOGGER.info("unit preferences applied: %s", ", ".join(applied))
        if failed:
            LOGGER.warning(
                "unit preferences NOT applied: %s -- values in these "
                "dimensions may not be SI; verify with "
                "core/validation.py's startup self-test",
                ", ".join(failed),
            )

        mismatched = self._verify_units(preferences)
        if mismatched:
            LOGGER.warning(
                "unit preferences did not take effect: %s. SetCurrentUnit "
                "accepted these and STK is still reporting the old unit, so "
                "data providers in these dimensions are NOT SI. Anything "
                "reading them must measure the scale rather than trust this "
                "call.", ", ".join(mismatched))

    def _verify_units(self, preferences) -> List[str]:
        """Read each preference back; return the ones that disagree.

        ``GetCurrentUnitAbbrv`` is the only direct evidence that a
        preference applied. If the call itself is unavailable the list
        comes back empty and the caller is no worse off than before.
        """
        mismatched: List[str] = []
        for dimension, unit in preferences:
            try:
                actual = str(self._root.UnitPreferences.GetCurrentUnitAbbrv(
                    dimension))
            except Exception:
                continue
            if actual.strip().lower() != unit.strip().lower():
                mismatched.append(f"{dimension}: asked {unit}, got {actual}")
        return mismatched

    # -- version / capability probing --------------------------------------

    def _load_enums(self) -> None:
        """Import the STK enum module once, tolerating naming drift.

        STK 12.x exposes enums as ``AgE*`` names in
        ``agi.stk12.stkobjects``. Later releases introduced un-prefixed
        aliases. Both spellings are cached so :meth:`_resolve_enum` can
        find whichever this install provides.
        """
        try:
            from agi.stk12 import stkobjects
        except ImportError as exc:  # pragma: no cover
            raise BackendUnavailableError(
                BackendType.COM.value,
                f"agi.stk12.stkobjects unavailable: {exc}",
            ) from exc
        self._enums["module"] = stkobjects

    def _resolve_enum(self, *candidate_names: str) -> Any:
        """Return the first enum/constant that exists under any given name.

        Raises ``AttributeError`` naming all candidates if none resolve,
        which produces a far more debuggable failure than an
        ``AttributeError`` on a single guessed spelling.
        """
        module = self._enums.get("module")
        for name in candidate_names:
            if module is not None and hasattr(module, name):
                return getattr(module, name)
        raise AttributeError(
            "none of these STK enum names exist in this install: "
            + ", ".join(candidate_names)
            + " -- run scripts/check_environment.py and report the STK version"
        )

    def _probe_version(self) -> None:
        self._info.python_version = sys.version.split()[0]
        try:
            self._info.stk_version = str(self._root.ExecuteCommand(
                "GetSTKVersion /").Item(0)).strip()
        except Exception:
            try:
                self._info.stk_version = str(self._stk.Version)
            except Exception:
                LOGGER.debug("could not determine STK version", exc_info=True)
        try:
            import agi.stk12 as _pkg

            self._info.agi_stk_wheel_version = getattr(
                _pkg, "__version__", "unknown")
        except Exception:
            pass

    def _probe_capabilities(self) -> None:
        """Determine licensed capabilities by asking STK directly.

        Uses the ``GetLicense`` Connect command through
        ``ExecuteCommand``, which is available to every Integration-
        licensed install. Anything that cannot be determined is recorded
        as unknown-but-assumed-available with the reason captured in
        ``unlicensed_detail``, so a later failure carries context.
        """
        caps = BackendCapabilities(
            integration=True,          # we are speaking Object Model already
            graphics=True,
            async_connect=False,        # not supported over COM, per AGI
            numpy_bulk_extraction=True,  # ToNumpyArray, STK 12.6+
        )

        probes = {
            "astrogator": "Astrogator",
            "realtime_propagator": "RealTimePropagator",
            "coverage": "Coverage",
            "communications": "Communications",
        }
        for attribute, feature in probes.items():
            try:
                result = self._root.ExecuteCommand(f"GetLicense / {feature}")
                text = " ".join(str(result.Item(i))
                                for i in range(result.Count)).lower()
                setattr(caps, attribute, "yes" in text or "true" in text
                        or "1" in text)
            except Exception as exc:
                setattr(caps, attribute, True)
                caps.unlicensed_detail[attribute] = (
                    f"could not probe ({exc}); assumed available and "
                    f"verified at point of use"
                )
        self._capabilities = caps

    # -- helpers -----------------------------------------------------------

    def _get_object(self, object_path: str) -> Any:
        self.require_connected()
        try:
            return self._root.GetObjectFromPath(object_path)
        except Exception as exc:
            raise ObjectNotFoundError(object_path) from exc

    def _execute(self, command: str) -> ConnectResponse:
        """Run a Connect command through the Object Model.

        ``ExecuteCommand`` raises on NACK rather than returning a status,
        so failures arrive as exceptions and are translated to the
        bridge's taxonomy by callers.
        """
        self.require_connected()
        try:
            result = self._root.ExecuteCommand(command)
        except Exception as exc:
            message = str(exc)
            if "not connected" in message.lower() or "RPC" in message:
                self._connected = False
                raise ConnectionLostError(
                    BackendType.COM.value, message
                ) from exc
            raise CommandError(command.split(" ", 1)[0], "-", message) from exc

        messages = []
        try:
            messages = [str(result.Item(i)) for i in range(result.Count)]
        except Exception:
            pass
        return ConnectResponse(command=command, ack=True, messages=messages,
                               raw="\n".join(messages))

    # -- scenario ----------------------------------------------------------

    def get_current_scenario_name(self) -> Optional[str]:
        """Inspect the attached application's scenario without replacing it."""
        self.require_connected()
        try:
            scenario = self._root.CurrentScenario
            return None if scenario is None else str(scenario.InstanceName)
        except Exception as exc:
            raise QueryError("CurrentScenario", "/", str(exc)) from exc

    def new_scenario(self, name: str) -> None:
        self._provider_cache.clear()
        self._revision += 1
        self.require_connected()
        try:
            self._root.NewScenario(name)
        except Exception as exc:
            raise CommandError("NewScenario", name, str(exc)) from exc

    def load_scenario(self, path: str) -> None:
        self._provider_cache.clear()
        self._revision += 1
        self.require_connected()
        try:
            self._root.LoadScenario(path)
        except Exception as exc:
            raise CommandError("LoadScenario", path, str(exc)) from exc

    def save_scenario(self, path: Optional[str] = None) -> None:
        self.require_connected()
        try:
            if path:
                self._root.SaveScenarioAs(path)
            else:
                self._root.SaveScenario()
        except Exception as exc:
            raise CommandError("SaveScenario", path or "(current)",
                               str(exc)) from exc

    def close_scenario(self) -> None:
        self._provider_cache.clear()
        self._revision += 1
        self.require_connected()
        try:
            self._root.CloseScenario()
        except Exception:
            LOGGER.debug("CloseScenario raised (likely none open)",
                         exc_info=True)

    def get_scenario_epoch(self) -> StkTime:
        self.require_connected()
        scenario = self._root.CurrentScenario
        if scenario is None:
            raise QueryError("ScenarioEpoch", "/", "no scenario loaded")
        try:
            return proto.parse_utcg(str(scenario.StartTime))
        except Exception as exc:
            raise QueryError("ScenarioEpoch", "/", str(exc)) from exc

    @property
    def revision(self) -> int:
        return self._revision

    def get_propagator_type(self, object_path: str) -> Optional[str]:
        # IAgSatellite.PropagatorType (AgEVePropagatorType); the live
        # 12.6 inspection read 'ePropagatorHPOP' this way. PropagatorName,
        # used by list_objects, is not on the 12.6 Object Model.
        obj = self._get_object(object_path)
        try:
            name = getattr(obj.PropagatorType, "name", None)
        except Exception:                   # noqa: BLE001 - not a vehicle
            return None
        return name if isinstance(name, str) else None

    def graphics_state(self) -> Optional[Dict[str, Any]]:
        """Open 2D/3D Graphics windows and the main window's state.

        Read-only, documented calls: IAgUiApplication.Windows,
        IAgUiWindow.GetServiceByType (null when the window is not of that
        type), IAgUiApplication.WindowState and Visible (agi.stk12
        uiapplication.py / uicore.py). None when there is no STK
        Desktop application object.
        """
        if self._stk is None:
            return None
        from agi.stk12.uicore import AgEWindowService, AgEWindowState

        windows_2d = windows_3d = 0
        for window in self._stk.Windows:
            if window.GetServiceByType(
                    AgEWindowService.eWindowService2DWindow) is not None:
                windows_2d += 1
            if window.GetServiceByType(
                    AgEWindowService.eWindowService3DWindow) is not None:
                windows_3d += 1
        state = {AgEWindowState.eWindowStateMinimized: "minimized",
                 AgEWindowState.eWindowStateMaximized: "maximized",
                 AgEWindowState.eWindowStateNormal: "normal"}.get(
                     self._stk.WindowState, str(self._stk.WindowState))
        return {"windows_2d": windows_2d, "windows_3d": windows_3d,
                "main_window": state, "visible": bool(self._stk.Visible)}

    def set_graphics_off(self) -> None:
        """Close every 2D/3D Graphics window and minimise STK Desktop.

        Called only by the scenario builder after a build (never during
        a run); the caller reads :meth:`graphics_state` back to check.
        IAgUiWindow.Close and IAgUiApplication.WindowState (agi.stk12
        uicore.py / uiapplication.py).
        """
        if self._stk is None:
            return
        from agi.stk12.uicore import AgEWindowService, AgEWindowState

        graphics = [window for window in self._stk.Windows
                    if window.GetServiceByType(
                        AgEWindowService.eWindowService2DWindow) is not None
                    or window.GetServiceByType(
                        AgEWindowService.eWindowService3DWindow) is not None]
        for window in graphics:
            window.Close()
        self._stk.WindowState = AgEWindowState.eWindowStateMinimized

    def get_scenario_stop_time(self) -> StkTime:
        self.require_connected()
        scenario = self._root.CurrentScenario
        if scenario is None:
            raise QueryError("ScenarioStopTime", "/", "no scenario loaded")
        try:
            return proto.parse_utcg(str(scenario.StopTime))
        except Exception as exc:
            raise QueryError("ScenarioStopTime", "/", str(exc)) from exc

    def list_objects(self) -> List[ObjectInfo]:
        self.require_connected()
        scenario = self._root.CurrentScenario
        if scenario is None:
            return []
        objects: List[ObjectInfo] = []
        try:
            children = scenario.Children
            for index in range(children.Count):
                child = children.Item(index)
                objects.append(ObjectInfo(
                    object_path=str(child.Path),
                    object_type=str(child.ClassName),
                    propagator_type=_safe_propagator_name(child),
                ))
        except Exception as exc:
            raise QueryError("Children", "/", str(exc)) from exc
        return objects

    def object_exists(self, object_path: str) -> bool:
        try:
            self._get_object(object_path)
        except ObjectNotFoundError:
            return False
        return True

    def create_object(self, object_type: str, name: str,
                      parent_path: str = "") -> str:
        self._provider_cache.clear()
        self._revision += 1
        self.require_connected()
        parent = (self._get_object(parent_path) if parent_path
                  else self._root.CurrentScenario)
        if parent is None:
            raise CommandError("CreateObject", name, "no scenario loaded")
        try:
            enum_value = self._resolve_enum(
                f"AgESTKObjectType",  # 12.x spelling
                f"STKObjectType",      # later alias
            )
            type_value = getattr(enum_value, f"e{object_type}")
            child = parent.Children.New(type_value, name)
        except AttributeError:
            # Fall back to the string-typed overload, which STK also
            # accepts and which is immune to enum-name drift.
            try:
                child = parent.Children.New(f"e{object_type}", name)
            except Exception as exc:
                raise CommandError("CreateObject", name, str(exc)) from exc
        except Exception as exc:
            raise CommandError("CreateObject", name, str(exc)) from exc
        return str(child.Path)

    def delete_object(self, object_path: str) -> None:
        self._provider_cache.clear()
        self._revision += 1
        obj = self._get_object(object_path)
        try:
            obj.Unload()
        except Exception as exc:
            raise CommandError("Unload", object_path, str(exc)) from exc

    # -- time --------------------------------------------------------------

    def get_current_time(self) -> StkTime:
        self.require_connected()
        try:
            return proto.parse_utcg(str(self._root.CurrentTime))
        except Exception:
            response = self._execute("GetAnimationTime /")
            if not response.messages:
                raise QueryError("CurrentTime", "/", "no reply")
            return proto.parse_utcg(response.messages[0].strip().strip('"'))

    def set_current_time(self, when: StkTime) -> None:
        self._execute(
            f"Animate * SetValues "
            f"{proto.quote(proto.format_utcg(when))} 1 Reset"
        )

    # -- sensing -----------------------------------------------------------

    def query_data_provider(self, query: DataProviderQuery) -> DataProviderData:
        """Typed Data Provider access.

        Handles the group/element distinction explicitly: many providers
        (notably "Cartesian Position") are *groups* whose members are
        reference frames, and calling ``Exec`` on the group rather than
        the member is the single most common STK scripting mistake. The
        ``Group`` attribute is probed rather than assumed, because
        whether a given provider is grouped varies by object type.
        """
        self.require_connected()
        key = (query.object_path, query.provider_name, query.group_element)
        if key not in self._provider_cache:
            obj = self._get_object(query.object_path)
            try:
                provider = obj.DataProviders.Item(query.provider_name)
            except Exception as exc:
                raise QueryError(query.provider_name, query.object_path,
                                 f"no such data provider ({exc})") from exc
            provider = self._select_group_member(provider, query)
            fixed_position = (query.provider_name == "Cartesian Position" and
                              query.group_element and not hasattr(provider, "ExecSingle"))
            if len(self._provider_cache) >= 256:
                self._provider_cache.clear()
            self._provider_cache[key] = (obj, provider, fixed_position)
        obj, provider, fixed_position = self._provider_cache[key]
        try:
            if fixed_position:
                # Still resolve fixed-object geometry at EVERY requested epoch.
                return self._fixed_position_in_frame(obj, query)
            started = time.perf_counter()
            try:
                result = self._exec_provider(provider, query)
            finally:
                self._record_query_timing(query, "execute", time.perf_counter() - started)
            started = time.perf_counter()
            try:
                return self._extract(result, query)
            finally:
                self._record_query_timing(query, "extract", time.perf_counter() - started)
        except Exception:
            # Do not hide an execution failure or return old telemetry. Resolve
            # fresh handles on the next query in case the object was replaced.
            self._provider_cache.pop(key, None)
            raise

    def _record_query_timing(self, query, phase, elapsed):
        """Aggregate attempted provider calls, including failures, on the COM worker.

        Execution excludes handle resolution; extraction includes metadata and
        Python conversion. Fixed-object LocateInSystem queries are not included.
        No telemetry or COM result objects are retained.
        """
        key = (query.provider_name, query.group_element, phase)
        count, total = self._query_timings.get(key, (0, 0.0))
        self._query_timings[key] = (count + 1, total + elapsed)
        now = time.perf_counter()
        if now - self._query_timings_since >= 60.0:
            details = "; ".join(
                f"{name}/{frame or '-'} {part}: n={n}, mean_ms={1000 * seconds / n:.3f}"
                for (name, frame, part), (n, seconds) in self._query_timings.items())
            LOGGER.info("COM provider profile (interval): %s; extraction paths=%s",
                        details, self._extraction_counts)
            self._query_timings.clear()
            self._extraction_counts = dict.fromkeys(self._extraction_counts, 0)
            self._query_timings_since = now

    def _fixed_position_in_frame(self, obj, query):
        try:
            body = self._root.CentralBodies.Item(obj.CentralBodyName)
            system = body.Vgt.Systems.Item(query.group_element)
            point = obj.Vgt.Points.Item("Center")
            start = query.start or self.get_scenario_epoch()
            stop = query.stop or start
            if not query.is_instantaneous and query.step_s <= 0:
                raise ValueError("step_s must be positive")
            count = (1 if query.is_instantaneous else
                     int(math.floor((stop - start) / query.step_s + 1e-7)) + 1)
            columns = {name: [] for name in (query.elements or ("Time", "x", "y", "z"))}
            for i in range(count):
                stamp = proto.format_utcg(start + i * query.step_s)
                position = point.LocateInSystem(stamp, system).Position
                row = {"Time": stamp, "x": position.X,
                       "y": position.Y, "z": position.Z}
                for name in columns:
                    columns[name].append(row[name])
            return DataProviderData(object_path=query.object_path,
                                    provider_name=query.provider_name,
                                    columns=columns, row_count=count)
        except Exception as exc:
            raise QueryError(query.provider_name, query.object_path,
                             f"cannot resolve fixed-object position in "
                             f"{query.group_element!r}: {exc}") from exc

    def _select_group_member(self, provider: Any,
                             query: DataProviderQuery) -> Any:
        has_group = False
        try:
            has_group = bool(provider.Group) and provider.Group.Count > 0
        except Exception:
            has_group = False

        if not has_group:
            return provider

        element = query.group_element
        if not element:
            available = []
            try:
                available = [str(provider.Group.Item(i).Name)
                             for i in range(provider.Group.Count)]
            except Exception:
                pass
            raise QueryError(
                query.provider_name, query.object_path,
                "this is a grouped data provider and no group_element was "
                f"given; available members: {available}",
            )
        try:
            return provider.Group.Item(element)
        except Exception as exc:
            raise QueryError(
                query.provider_name, query.object_path,
                f"group member {element!r} not found ({exc})",
            ) from exc

    def _exec_provider(self, provider: Any, query: DataProviderQuery) -> Any:
        """Execute a provider, time-varying or fixed.

        STK has two data-provider interfaces and they are not
        interchangeable. ``AgDataPrvTimeVar`` takes epochs --
        ``ExecSingle(time)`` / ``Exec(start, stop, step)``. A **fixed**
        provider (``AgDataPrvFixed``) describes something that does not
        vary with time, so it takes no epochs at all: ``Exec()``.

        Static objects -- Target, Facility, Place -- serve *every*
        provider through the fixed interface, position included.
        Confirmed on a live STK 12.6 install, where reading a target's
        position failed with ``'AgDataPrvFixed' object has no attribute
        'ExecSingle'``. That single missing branch made every target and
        facility position unreadable, which in turn made target
        placement unverifiable and hid a unit error that put the whole
        scenario in the wrong place.
        """
        if not hasattr(provider, "ExecSingle"):
            try:
                if query.elements:
                    return provider.ExecElements(list(query.elements))
                return provider.Exec()
            except Exception as exc:
                raise QueryError(
                    query.provider_name, query.object_path,
                    f"fixed-provider execution failed: {exc}") from exc

        start = (proto.format_utcg(query.start) if query.start
                 else str(self._root.CurrentScenario.StartTime))
        # Instantaneous reads do not need StopTime: avoid two COM property
        # accesses on every single-epoch telemetry query.
        stop = None
        if not query.is_instantaneous:
            stop = (proto.format_utcg(query.stop) if query.stop
                    else str(self._root.CurrentScenario.StopTime))
        try:
            if query.elements:
                if query.is_instantaneous:
                    return provider.ExecSingleElements(start,
                                                       list(query.elements))
                return provider.ExecElements(start, stop, query.step_s,
                                             list(query.elements))
            if query.is_instantaneous:
                return provider.ExecSingle(start)
            return provider.Exec(start, stop, query.step_s)
        except Exception as exc:
            raise QueryError(query.provider_name, query.object_path,
                             f"execution failed: {exc}") from exc

    @contextmanager
    def _extraction_stage(self, query, stage):
        started = time.perf_counter()
        failed = 0
        try:
            yield
        except Exception:
            failed = 1
            raise
        finally:
            elapsed = time.perf_counter() - started
            key = (query.provider_name, query.group_element, stage)
            n, total, peak, errors = self._extraction_timings.get(key, (0, 0., 0., 0))
            self._extraction_timings[key] = (n + 1, total + elapsed,
                                            max(peak, elapsed), errors + failed)

    def _flush_extraction_profile(self):
        if not self._extraction_timings:
            return
        for (name, frame, stage), (n, total, peak, errors) in self._extraction_timings.items():
            LOGGER.info("COM extraction detail: provider=%s frame=%s stage=%s "
                        "n=%d mean_ms=%.4f max_ms=%.4f errors=%d",
                        name, frame or '-', stage, n, 1000 * total / n,
                        1000 * peak, errors)
        self._extraction_timings.clear()
        self._extraction_since = time.perf_counter()

    def _extract(self, result: Any, query: DataProviderQuery) -> DataProviderData:
        try:
            with self._extraction_stage(query, 'total'):
                return self._extract_profiled(result, query)
        finally:
            # Flush only after a complete extraction, never within a stage.
            if time.perf_counter() - self._extraction_since >= 60.0:
                self._flush_extraction_profile()

    def _extract_profiled(self, result: Any, query: DataProviderQuery
                 ) -> DataProviderData:
        """Pull a DR result into plain Python columns.

        Prefers ``ToNumpyArray()`` (STK 12.6+) for multi-row results
        because the per-element ``GetValues()`` path makes one COM call
        per column and that dominates latency on wide providers. Falls
        back to ``GetValues()`` when NumPy extraction is unavailable, so
        the backend still works if the install predates it.
        """
        columns: Dict[str, List] = {}

        datasets = None
        try:
            with self._extraction_stage(query, 'datasets'):
                datasets = result.DataSets
        except Exception as exc:
            raise QueryError(query.provider_name, query.object_path,
                             f"result has no DataSets ({exc})") from exc

        names: List[str] = []
        selected_datasets = []
        try:
            # STK 12.6 exposes the complete ordered names in one COM call.
            # Read them afresh: no cross-epoch schema cache or stale mapping.
            with self._extraction_stage(query, 'element_names'):
                names = [str(name) for name in datasets.ElementNames]
        except Exception:
            try:
                with self._extraction_stage(query, 'count'):
                    count = datasets.Count
                for index in range(count):
                    with self._extraction_stage(query, 'item'):
                        dataset = datasets.Item(index)
                    with self._extraction_stage(query, 'name'):
                        names.append(str(dataset.ElementName))
                    selected_datasets.append(dataset)
            except Exception:
                names = list(query.elements)
                selected_datasets = []

        if self._capabilities.numpy_bulk_extraction and len(names) > 1:
            try:
                with self._extraction_stage(query, 'bulk_fetch'):
                    array = datasets.ToNumpyArray()
                    if array.ndim != 2 or array.shape[1] != len(names):
                        raise ValueError('Bulk data shape does not match element names')
                with self._extraction_stage(query, 'python_conversion'):
                    for index, name in enumerate(names):
                        columns[name] = [_coerce(v) for v in array[:, index]]
                    row_count = len(next(iter(columns.values()), []))
                self._extraction_counts["bulk"] += 1
                with self._extraction_stage(query, 'output'):
                    return DataProviderData(
                        object_path=query.object_path,
                        provider_name=query.provider_name,
                        columns=columns, row_count=row_count,
                    )
            except Exception:
                self._extraction_counts["bulk_failed"] += 1
                LOGGER.debug("ToNumpyArray unavailable; using GetValues",
                             exc_info=True)
                self._capabilities.numpy_bulk_extraction = False

        try:
            self._extraction_counts["fallback"] += 1
            columns = {}
            if not selected_datasets:
                with self._extraction_stage(query, 'fallback_metadata'):
                    selected_datasets = [datasets.Item(i) for i in range(datasets.Count)]
                    names = [str(dataset.ElementName) for dataset in selected_datasets]
            for name, dataset in zip(names, selected_datasets):
                with self._extraction_stage(query, 'fallback_fetch'):
                    values = dataset.GetValues()
                with self._extraction_stage(query, 'python_conversion'):
                    columns[name] = [_coerce(v) for v in values]
        except Exception as exc:
            raise QueryError(query.provider_name, query.object_path,
                             f"could not read datasets ({exc})") from exc

        row_count = len(next(iter(columns.values()), []))
        with self._extraction_stage(query, 'output'):
            return DataProviderData(
                object_path=query.object_path,
                provider_name=query.provider_name,
                columns=columns, row_count=row_count,
            )

    def list_data_providers(self, object_path: str) -> Dict[str, List[str]]:
        """Enumerate this object's data providers and their group members.

        Walks ``obj.DataProviders`` by index. Individual providers that
        raise while being inspected are skipped rather than aborting the
        walk, because a single unavailable provider (unlicensed module,
        wrong object type) must not hide the other eighty.
        """
        obj = self._get_object(object_path)
        providers: Dict[str, List[str]] = {}
        try:
            collection = obj.DataProviders
            count = collection.Count
        except Exception as exc:
            LOGGER.debug("cannot enumerate data providers on %s: %s",
                         object_path, exc)
            return {}

        for index in range(count):
            try:
                item = collection.Item(index)
                name = str(item.Name)
            except Exception:
                continue
            members: List[str] = []
            try:
                group = item.Group
                for group_index in range(group.Count):
                    try:
                        members.append(str(group.Item(group_index).Name))
                    except Exception:
                        continue
            except Exception:
                members = []
            providers[name] = members
        return providers

    def compute_access(self, from_path: str, to_path: str,
                       start: StkTime, stop: StkTime
                       ) -> List[AccessIntervalData]:
        source = self._get_object(from_path)
        try:
            target = self._get_object(to_path)
            access = source.GetAccessToObject(target)
            access.ComputeAccess()
            intervals = access.ComputedAccessIntervalTimes
            out: List[AccessIntervalData] = []
            for index in range(intervals.Count):
                begin, end = intervals.GetInterval(index)
                out.append(AccessIntervalData(
                    start=proto.parse_utcg(str(begin)),
                    stop=proto.parse_utcg(str(end)),
                ))
            return out
        except Exception as exc:
            raise QueryError("Access", from_path, str(exc)) from exc

    def get_link_budget(self, tx_path: str, rx_path: str,
                        when: StkTime) -> LinkBudgetData:
        data = self.query_data_provider(DataProviderQuery(
            object_path=rx_path, provider_name="Link Information",
            start=when, stop=when,
        ))
        return LinkBudgetData(
            carrier_to_noise_db=_first_of(data, "C/N", "C/No"),
            eb_no_db=_first_of(data, "Eb/No"),
            margin_db=_first_of(data, "Link Margin", "Eb/No Margin"),
            received_power_dbw=_first_of(data, "Rcvd. Iso. Power"),
            free_space_loss_db=_first_of(data, "Free Space Loss"),
            link_closed=_first_of(data, "Link Margin") > 0.0,
        )

    # -- actuation ---------------------------------------------------------

    def set_propagator(self, object_path: str, propagator: PropagatorType,
                       look_ahead_s: float = 3600.0,
                       look_behind_s: float = 3600.0,
                       external_file: str = "") -> None:
        """Switch propagator.

        Implemented through Connect commands rather than the typed
        ``SetPropagatorType`` API. That is a deliberate choice: the enum
        names for propagator types moved between 12.x releases, whereas
        the Connect spellings did not, and this operation happens once at
        setup where the extra microseconds are irrelevant. The hot path
        (SetPosition/AddAttitude) is likewise Connect-based for the same
        reason -- see the note on :meth:`set_position`.
        """
        self._provider_cache.clear()
        self._revision += 1
        if propagator is PropagatorType.REALTIME:
            self._capabilities.require("realtime_propagator")
            # Confirmed against STK 12.6: the `SetPropagator ... RealTime`
            # Connect command fails ("Command has failed."), but the typed
            # Object Model path works. Enum spelling verified on a live
            # 12.6 install: AgEVePropagatorType.ePropagatorRealtime --
            # note the lowercase 't' in "Realtime", which differs from the
            # "RealTime" spelling used elsewhere in STK's own docs.
            try:
                stkobjects = self._enums.get("module")
                enum_type = self._resolve_enum("AgEVePropagatorType",
                                               "VePropagatorType")
                satellite = self._root.GetObjectFromPath(object_path)
                satellite.SetPropagatorType(
                    getattr(enum_type, "ePropagatorRealtime")
                )
            except Exception as exc:
                raise CommandError("SetPropagator", object_path,
                                   str(exc)) from exc

            # SetPropagatorType leaves Look-Ahead/Look-Behind at STK's
            # defaults. Those durations decide how far the real-time
            # propagator extrapolates past the newest injected sample and
            # how much history it retains, which directly determines
            # whether a data provider queried at "now" returns anything at
            # all. Applying them is therefore not optional, but the
            # property names are version-sensitive, so failure here is a
            # warning rather than a hard error -- the propagator still
            # works, just with defaults.
            self._configure_realtime_durations(
                object_path, look_ahead_s, look_behind_s)
        elif propagator is PropagatorType.EXTERNAL:
            if not external_file:
                raise CommandError("SetPropagator", object_path,
                                   "EXTERNAL propagator requires external_file")
            self.load_external_ephemeris(object_path, external_file)
        else:
            # Confirmed on a live STK 12.6 install for RealTime: the
            # Connect form of SetPropagator fails ("Command has failed.")
            # while SetPropagatorType on the Object Model succeeds. The
            # same is assumed to apply to the analytic propagators, so
            # the typed path is tried FIRST and Connect is the fallback
            # -- the reverse of the original ordering, which would have
            # spent a failed round trip on every call.
            enum_member = {
                PropagatorType.TWO_BODY: "ePropagatorTwoBody",
                PropagatorType.J4_PERTURBATION: "ePropagatorJ4Perturbation",
                PropagatorType.HPOP: "ePropagatorHPOP",
            }.get(propagator)

            applied = False
            if enum_member is not None:
                try:
                    enum_type = self._resolve_enum("AgEVePropagatorType",
                                                   "VePropagatorType")
                    satellite = self._get_object(object_path)
                    satellite.SetPropagatorType(getattr(enum_type,
                                                        enum_member))
                    applied = True
                except Exception as exc:
                    LOGGER.debug("SetPropagatorType(%s) failed on %s: %s; "
                                 "falling back to Connect",
                                 enum_member, object_path, exc)

            if not applied:
                self._execute(
                    f"SetPropagator {object_path} {propagator.value}")

    def _configure_realtime_durations(self, object_path: str,
                                      look_ahead_s: float,
                                      look_behind_s: float) -> None:
        """Set Look-Ahead / Look-Behind on a real-time-propagated object.

        STK applies these durations to real-time orbit propagation,
        real-time attitude, and model articulations alike -- they are one
        shared setting, not three. Property spelling has moved across
        12.x, so several candidates are attempted.
        """
        satellite = self._get_object(object_path)
        try:
            propagator = satellite.Propagator
        except Exception as exc:
            LOGGER.warning("cannot reach Propagator on %s: %s",
                           object_path, exc)
            return

        applied = []
        for candidates, value in (
            # "Duration" confirmed present on STK 12.6's real-time
            # propagator via dir(); the LookAhead*/LookBehind* spellings
            # are not. Duration is tried first for look-ahead. Note that
            # 12.6 exposes no separate look-behind property here, so
            # look_behind_s may be unsettable -- reported, not silently
            # ignored.
            (("Duration", "LookAheadDuration", "LookAheadTime",
              "LookAhead"), look_ahead_s),
            (("LookBehindDuration", "LookBehindTime", "LookBehind"),
             look_behind_s),
        ):
            for name in candidates:
                try:
                    setattr(propagator, name, value)
                    applied.append(f"{name}={value}")
                    break
                except Exception:
                    continue

        if applied:
            LOGGER.info("real-time durations on %s: %s", object_path,
                        ", ".join(applied))
            return

        # None of the candidate spellings worked. Rather than leave the
        # operator to guess, enumerate what the propagator object
        # actually exposes -- that turns a dead-end warning into the
        # exact list of names to add above.
        discovered = []
        try:
            discovered = [
                name for name in dir(propagator)
                if not name.startswith("_")
                and any(token in name.lower()
                        for token in ("look", "ahead", "behind", "duration"))
            ]
        except Exception:
            pass

        LOGGER.warning(
            "could not set Look-Ahead/Look-Behind on %s; STK defaults "
            "apply. Candidate properties on this propagator that look "
            "relevant: %s. Note that Look-Ahead/Look-Behind bound the "
            "interval over which a real-time object has ANY ephemeris, so "
            "a too-small default is a likely cause of empty data provider "
            "results near the newest injected sample.",
            object_path, discovered or "(none found)",
        )

    def propagate(self, object_path: str) -> bool:
        """Force ephemeris generation for an object.

        A satellite created via ``Children.New`` has a propagator and
        default elements but *no ephemeris* until propagated, and every
        data provider on it returns zero rows until then. This is the
        single most common cause of a silently empty Data Provider
        result, so it gets a dedicated, explicitly-named method rather
        than being buried in a query path.

        Returns True if propagation ran, False if the object has no
        propagator that supports it (e.g. real-time, which generates
        ephemeris from injected samples instead).
        """
        obj = self._get_object(object_path)
        self._revision += 1             # the ephemeris may be regenerated
        try:
            obj.Propagator.Propagate()
            return True
        except Exception as exc:
            LOGGER.debug("Propagate() unavailable on %s: %s",
                         object_path, exc)
            return False

    def set_sensor_cone(self, object_path: str, half_angle_rad: float,
                        angular_resolution_rad: float = 0.017453) -> str:
        """Set a simple-conic pattern, Object Model first.

        The Connect ``Define ... SimpleConic`` form was rejected on a
        live STK 12.6 install in both angle units, leaving the sensor at
        STK's default half-angle. That is the same shape of failure as
        ``SetPropagator ... RealTime``, whose typed equivalent worked,
        so the typed path is tried first here too.

        Each candidate is confirmed by reading the cone angle back
        rather than by its return value: the units these accept follow
        STK's Angle preference, and that preference was observed being
        honoured for data providers while Connect commands ignored it.
        """
        try:
            sensor = self._get_object(object_path)
        except Exception as exc:                          # noqa: BLE001
            raise CommandError("SetSensorCone", object_path, str(exc))

        wanted_deg = math.degrees(half_angle_rad)
        reasons: List[str] = []

        for label, value, resolution in (
            ("CommonTasks.SetPatternSimpleConic (radians)",
             half_angle_rad, angular_resolution_rad),
            ("CommonTasks.SetPatternSimpleConic (degrees)",
             wanted_deg, math.degrees(angular_resolution_rad)),
        ):
            try:
                sensor.CommonTasks.SetPatternSimpleConic(value, resolution)
            except Exception as exc:                      # noqa: BLE001
                reasons.append(f"{label}: {type(exc).__name__}: {exc}")
                continue
            applied = self.get_sensor_cone_half_angle_rad(object_path)
            if applied is None:
                # Cannot confirm, but the call did not raise. Accept it
                # and let the caller's own measurement arbitrate.
                return label + " (unconfirmed)"
            if abs(math.degrees(applied) - wanted_deg) < 0.01:
                return label
            reasons.append(
                f"{label}: applied but reads back as "
                f"{math.degrees(applied):.4f} deg, not {wanted_deg:.4f}")

        try:
            return super().set_sensor_cone(object_path, half_angle_rad,
                                           angular_resolution_rad)
        except CommandError as exc:
            reasons.append(str(exc))
        raise CommandError("SetSensorCone", object_path, "; ".join(reasons))

    def set_az_el_mask(self, object_path: str, mode: str,
                       value: float = 0.0) -> str:
        """Object Model first, then Connect.

        Every Connect ``SetAzElMask`` form is rejected on STK 12.6 with
        the generic "Command has failed."; the typed path works. Member
        spellings are measured, not guessed -- a live install reports
        ``eAzElMaskNone``, ``eMaskFile`` and ``eTerrainData``, and the
        first attempt at ``eAzElMaskTerrainData`` failed with an
        AttributeError that listed them.
        """
        members = {
            "terrain": ("eTerrainData", "eAzElMaskTerrainData"),
            "none": ("eAzElMaskNone", "eNone"),
            "file": ("eMaskFile", "eAzElMaskFile"),
        }.get(mode)
        if members is None:
            raise CommandError("SetAzElMask", object_path,
                               f"unknown mask mode {mode!r}")

        reasons: List[str] = []
        obj = enum_type = None
        try:
            obj = self._get_object(object_path)
        except Exception as exc:                          # noqa: BLE001
            reasons.append(f"object unavailable: {exc}")
        try:
            enum_type = self._resolve_enum("AgEAzElMaskType", "AzElMaskType")
        except Exception as exc:                          # noqa: BLE001
            reasons.append(f"AgEAzElMaskType unavailable: {exc}")

        if obj is not None and enum_type is not None:
            for member in members:
                enum_value = getattr(enum_type, member, None)
                if enum_value is None:
                    continue
                try:
                    obj.SetAzElMask(enum_value, value)
                    return f"Object Model SetAzElMask [{member}]"
                except Exception as exc:                  # noqa: BLE001
                    reasons.append(f"SetAzElMask [{member}]: "
                                   f"{type(exc).__name__}: {exc}")

        try:
            return super().set_az_el_mask(object_path, mode, value)
        except CommandError as exc:
            reasons.append(str(exc))

        reasons.append(self._describe_mask_api(obj, enum_type))
        raise CommandError("SetAzElMask", object_path, "; ".join(reasons))

    def get_az_el_mask(self, object_path: str) -> Optional[str]:
        try:
            obj = self._get_object(object_path)
            return str(obj.GetAzElMask())
        except Exception as exc:                          # noqa: BLE001
            LOGGER.debug("could not read the az-el mask on %s: %s",
                         object_path, exc)
            return None

    @staticmethod
    def _describe_mask_api(obj: Any, enum_type: Any) -> str:
        """What this install actually offers, for the next attempt."""
        parts = []
        if enum_type is not None:
            members = [name for name in dir(enum_type)
                       if name.startswith("e")]
            parts.append(f"AgEAzElMaskType members: {members or '(none)'}")
        if obj is not None:
            attributes = [name for name in dir(obj)
                          if "mask" in name.lower()]
            parts.append(f"object mask attributes: "
                         f"{attributes or '(none)'}")
        return " | ".join(parts) or "no Object Model introspection available"

    def get_sensor_cone_half_angle_rad(self, object_path: str
                                       ) -> Optional[float]:
        """Read the configured cone half-angle back through the Object Model.

        Returns radians. The property is reported in STK's current Angle
        unit, which configure_units() sets to radians -- but that is
        exactly the assumption that has already been wrong once, so a
        value too large to be a plausible radian half-angle is treated
        as degrees rather than believed.
        """
        try:
            sensor = self._get_object(object_path)
            value = float(sensor.Pattern.ConeAngle)
        except Exception as exc:                          # noqa: BLE001
            LOGGER.debug("could not read ConeAngle on %s: %s",
                         object_path, exc)
            return None
        # A conic sensor half-angle cannot exceed 90 deg = 1.5708 rad, so
        # anything above pi/2 is being reported in degrees.
        return math.radians(value) if value > math.pi / 2.0 else value

    def set_position(self, object_path: str, when: StkTime, frame: str,
                     position_m: Tuple[float, float, float],
                     velocity_mps: Optional[Tuple[float, float, float]] = None
                     ) -> None:
        """Append one real-time position sample.

        ``SetPosition`` has no typed Object Model equivalent -- it is a
        Connect-only command -- so even the COM backend issues it through
        ``ExecuteCommand``. This is in-process, so it does not incur the
        socket round-trip that the Connect backend does.
        """
        args = [
            "SetPosition", object_path, proto.quote(proto.format_utcg(when)),
            frame,
            f"{position_m[0]!r}", f"{position_m[1]!r}", f"{position_m[2]!r}",
        ]
        if velocity_mps is not None:
            args += [f"{velocity_mps[0]!r}", f"{velocity_mps[1]!r}",
                     f"{velocity_mps[2]!r}"]
        self._revision += 1             # an orbit write
        self._execute(" ".join(args))

    def enable_realtime_attitude(self, object_path: str) -> None:
        self._provider_cache.clear()
        self._revision += 1
        self._execute(f"SetAttitude {object_path} Standard")

    def clear_attitude_data(self, object_path: str) -> None:
        self._revision += 1
        try:
            self._execute(f"SetAttitude {object_path} ClearData")
        except CommandError:
            LOGGER.debug("ClearData rejected (likely no real-time data yet)")

    def add_attitude_quaternion(self, object_path: str, when: StkTime,
                                quaternion_xyzw: Tuple[float, float, float, float],
                                inertial: bool = False) -> None:
        qx, qy, qz, qw = quaternion_xyzw
        keyword = "Quat" if inertial else "CBFQuat"
        self._execute(
            f"AddAttitude {object_path} {keyword} "
            f"{proto.quote(proto.format_utcg(when))} "
            f"{qx!r} {qy!r} {qz!r} {qw!r}"
        )

    def add_attitude_angular_velocity(
        self, object_path: str, when: StkTime,
        angular_velocity_radps: Tuple[float, float, float],
    ) -> None:
        # Degrees/second is fixed by the command definition, independent
        # of the session's Angle unit preference.
        wx, wy, wz = (math.degrees(v) for v in angular_velocity_radps)
        self._execute(
            f"AddAttitude {object_path} AngVels "
            f"{proto.quote(proto.format_utcg(when))} {wx!r} {wy!r} {wz!r}"
        )

    def load_external_ephemeris(self, object_path: str,
                                file_path: str) -> None:
        self._revision += 1             # an orbit write
        self._execute(
            f"SetState {object_path} FromFile {proto.quote(file_path)}"
        )

    # -- escape hatch ------------------------------------------------------

    def execute_connect(self, command: str,
                        timeout_s: Optional[float] = None) -> ConnectResponse:
        self._provider_cache.clear()  # arbitrary commands may replace objects
        self._revision += 1
        # timeout_s is accepted for interface symmetry but has no meaning
        # in-process: ExecuteCommand is a blocking COM call with no
        # client-side timeout available.
        try:
            return self._execute(command)
        except CommandError as exc:
            return ConnectResponse(command=command, ack=False,
                                   messages=[exc.reason], raw=exc.reason)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _coerce(value: Any):
    try:
        return float(value)
    except (TypeError, ValueError):
        return str(value)


def _first_of(data: DataProviderData, *names: str) -> float:
    for name in names:
        if name in data.columns and data.columns[name]:
            value = data.columns[name][0]
            if isinstance(value, float):
                return value
    return 0.0


def _safe_propagator_name(child: Any) -> str:
    try:
        return str(child.Propagator.PropagatorName)
    except Exception:
        return ""
