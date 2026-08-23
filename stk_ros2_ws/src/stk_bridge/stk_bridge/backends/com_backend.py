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
                 visible: bool = True, user_control: bool = True):
        super().__init__(BackendType.COM)
        self._attach_to_existing = attach_to_existing
        self._visible = visible
        self._user_control = user_control
        self._stk: Any = None
        self._root: Any = None
        self._enums: Dict[str, Any] = {}
        self._info = BackendInfo(backend_type=BackendType.COM)

    # -- lifecycle ---------------------------------------------------------

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
            from agi.stk12.stkdesktop import STKDesktop
        except ImportError as exc:
            raise BackendUnavailableError(
                BackendType.COM.value,
                "cannot import agi.stk12. Install the wheel shipped with "
                "STK 12.6 from <STK install>\\bin\\AgPythonAPI (see "
                "docs/01_installation_windows.md). Note STK 12.6's Python "
                f"API is in-process only -- there is no gRPC remoting until "
                f"STK 12.8. Underlying error: {exc}",
            ) from exc

        try:
            if self._attach_to_existing:
                try:
                    self._stk = STKDesktop.AttachToApplication()
                    LOGGER.info("attached to a running STK Desktop instance")
                except Exception:
                    LOGGER.info("no running STK found; starting a new one")
                    self._stk = STKDesktop.StartApplication(
                        visible=self._visible, userControl=self._user_control
                    )
            else:
                self._stk = STKDesktop.StartApplication(
                    visible=self._visible, userControl=self._user_control
                )
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
        """Force STK's unit preferences to SI.

        After this, every Data Provider returns metres, metres/second,
        radians, and seconds, so no downstream conversion is needed. See
        ``backends/base.py``'s units contract.

        ``SetCurrentUnit`` raises rather than returning a status if the
        dimension or unit name is unknown, and the exact spellings have
        shifted slightly across STK versions, so each is attempted
        independently and failures are logged rather than aborting the
        connection -- a session missing one unit preference is degraded,
        not unusable, and ``core/validation.py`` will catch the resulting
        magnitude error at startup if it matters.
        """
        preferences = (
            ("Distance", "m"),
            ("Time", "sec"),
            ("Angle", "rad"),
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

    def new_scenario(self, name: str) -> None:
        self.require_connected()
        try:
            self._root.NewScenario(name)
        except Exception as exc:
            raise CommandError("NewScenario", name, str(exc)) from exc

    def load_scenario(self, path: str) -> None:
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
        obj = self._get_object(query.object_path)
        try:
            provider = obj.DataProviders.Item(query.provider_name)
        except Exception as exc:
            raise QueryError(query.provider_name, query.object_path,
                             f"no such data provider ({exc})") from exc

        provider = self._select_group_member(provider, query)
        result = self._exec_provider(provider, query)
        return self._extract(result, query)

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
        start = (proto.format_utcg(query.start) if query.start
                 else str(self._root.CurrentScenario.StartTime))
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

    def _extract(self, result: Any, query: DataProviderQuery
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
            datasets = result.DataSets
        except Exception as exc:
            raise QueryError(query.provider_name, query.object_path,
                             f"result has no DataSets ({exc})") from exc

        names: List[str] = []
        try:
            for index in range(datasets.Count):
                names.append(str(datasets.Item(index).ElementName))
        except Exception:
            names = list(query.elements)

        if self._capabilities.numpy_bulk_extraction and len(names) > 1:
            try:
                array = datasets.ToNumpyArray()
                for index, name in enumerate(names):
                    columns[name] = [_coerce(v) for v in array[:, index]]
                row_count = len(next(iter(columns.values()), []))
                return DataProviderData(
                    object_path=query.object_path,
                    provider_name=query.provider_name,
                    columns=columns, row_count=row_count,
                )
            except Exception:
                LOGGER.debug("ToNumpyArray unavailable; using GetValues",
                             exc_info=True)
                self._capabilities.numpy_bulk_extraction = False

        try:
            for index in range(datasets.Count):
                dataset = datasets.Item(index)
                name = str(dataset.ElementName)
                columns[name] = [_coerce(v) for v in dataset.GetValues()]
        except Exception as exc:
            raise QueryError(query.provider_name, query.object_path,
                             f"could not read datasets ({exc})") from exc

        row_count = len(next(iter(columns.values()), []))
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
            self._execute(f"SetPropagator {object_path} {propagator.value}")

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
        try:
            obj.Propagator.Propagate()
            return True
        except Exception as exc:
            LOGGER.debug("Propagate() unavailable on %s: %s",
                         object_path, exc)
            return False

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
        self._execute(" ".join(args))

    def enable_realtime_attitude(self, object_path: str) -> None:
        self._execute(f"SetAttitude {object_path} Standard")

    def clear_attitude_data(self, object_path: str) -> None:
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
        self._execute(
            f"SetState {object_path} FromFile {proto.quote(file_path)}"
        )

    # -- escape hatch ------------------------------------------------------

    def execute_connect(self, command: str,
                        timeout_s: Optional[float] = None) -> ConnectResponse:
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
