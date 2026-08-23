"""Service servers.

Every handler routes STK access through ``BackendWorker.call``, which
marshals onto the COM apartment thread and blocks until the result
returns. Services are the right place for that blocking: a service
caller is already synchronous by contract, and the executor's other
threads continue serving telemetry and commands meanwhile.

Failure convention
------------------
Handlers do not raise. A service that raises produces an opaque
transport-level failure on the caller's side with nothing actionable in
it. Instead every response carries ``success`` and ``message``, and the
message is the exception text -- so a MATLAB script that calls
``SetPropagator`` on an unlicensed install gets the licence error
verbatim rather than "service call failed".
"""

from __future__ import annotations

import logging
from typing import Callable, Dict, List, Optional

from stk_bridge.backends.base import (
    DataProviderQuery,
    PropagatorType,
    StkTime,
)
from stk_bridge.core import conversions as conv
from stk_bridge.core.time_sync import ExecutionMode
from stk_bridge.exceptions import StkBridgeError

LOGGER = logging.getLogger(__name__)

_PROPAGATOR_BY_INDEX = {
    0: PropagatorType.TWO_BODY,
    1: PropagatorType.J4_PERTURBATION,
    2: PropagatorType.HPOP,
    3: PropagatorType.REALTIME,
    4: PropagatorType.EXTERNAL,
}


class ServiceServers:
    """Creates and owns every service the bridge exposes."""

    def __init__(self, node, worker, context, prefix: str = "stk",
                 callback_group=None):
        """``context`` bundles the pieces handlers need.

        Passed as one object rather than a dozen constructor arguments
        because handlers legitimately need access to most of the node's
        state, and threading each through individually makes adding a
        service a five-file change.
        """
        from stk_msgs.srv import (
            ComputeAccess,
            CreateObject,
            DeleteObject,
            ExecuteConnect,
            GetLinkBudget,
            LoadScenario,
            QueryDataProvider,
            ResetScenario,
            SaveScenario,
            SetAnimationTime,
            SetExecutionMode,
            SetPropagator,
            StepSimulation,
        )

        self._node = node
        self._worker = worker
        self._ctx = context
        self._services = []

        definitions = (
            (LoadScenario, "load_scenario", self._load_scenario),
            (SaveScenario, "save_scenario", self._save_scenario),
            (ResetScenario, "reset_scenario", self._reset_scenario),
            (CreateObject, "create_object", self._create_object),
            (DeleteObject, "delete_object", self._delete_object),
            (SetPropagator, "set_propagator", self._set_propagator),
            (QueryDataProvider, "query_data_provider",
             self._query_data_provider),
            (ComputeAccess, "compute_access", self._compute_access),
            (GetLinkBudget, "get_link_budget", self._get_link_budget),
            (SetAnimationTime, "set_animation_time", self._set_animation_time),
            (StepSimulation, "step_simulation", self._step_simulation),
            (SetExecutionMode, "set_execution_mode", self._set_execution_mode),
            (ExecuteConnect, "execute_connect", self._execute_connect),
        )

        for service_type, name, handler in definitions:
            self._services.append(node.create_service(
                service_type, f"{prefix}/{name}", handler,
                callback_group=callback_group,
            ))

        LOGGER.info("%d services created under %s/", len(self._services),
                    prefix)

    # -- helpers -----------------------------------------------------------

    def _call(self, function: Callable, description: str,
              timeout_s: Optional[float] = None):
        return self._worker.call(function, description, timeout_s)

    @staticmethod
    def _fail(response, exc: Exception, operation: str):
        LOGGER.warning("%s failed: %s", operation, exc)
        response.success = False
        response.message = f"{type(exc).__name__}: {exc}"
        return response

    # -- scenario ----------------------------------------------------------

    def _load_scenario(self, request, response):
        try:
            def work(backend):
                if request.close_existing_first:
                    backend.close_scenario()
                backend.load_scenario(request.scenario_path)
                return backend.get_scenario_epoch()

            epoch = self._call(work, "LoadScenario", timeout_s=120.0)
            self._ctx.on_scenario_changed(epoch)
            response.success = True
            response.message = f"loaded {request.scenario_path}"
            response.scenario_name = request.scenario_path
            response.scenario_epoch = conv.stk_time_to_ros(epoch)
        except Exception as exc:
            self._fail(response, exc, "LoadScenario")
        return response

    def _save_scenario(self, request, response):
        try:
            self._call(
                lambda backend: backend.save_scenario(request.save_path or None),
                "SaveScenario", timeout_s=120.0)
            response.success = True
            response.message = "saved"
        except Exception as exc:
            self._fail(response, exc, "SaveScenario")
        return response

    def _reset_scenario(self, request, response):
        try:
            def work(backend):
                self._ctx.scenario.reset(request.clear_realtime_data)
                return True

            self._call(work, "ResetScenario", timeout_s=60.0)
            # Guard state must be cleared alongside STK's: after
            # ClearData, STK accepts earlier timestamps again, and a
            # stale monotonicity guard would reject every valid sample
            # for the rest of the run.
            self._ctx.actuation.reset()
            self._ctx.inbox.clear()
            response.success = True
            response.message = "scenario reset"
        except Exception as exc:
            self._fail(response, exc, "ResetScenario")
        return response

    def _create_object(self, request, response):
        try:
            path = self._call(
                lambda backend: self._ctx.scenario.create_object(
                    request.object_type, request.object_name,
                    request.parent_path),
                "CreateObject")
            response.success = True
            response.message = "created"
            response.object_path = path
        except Exception as exc:
            self._fail(response, exc, "CreateObject")
            response.object_path = ""
        return response

    def _delete_object(self, request, response):
        try:
            self._call(
                lambda backend: self._ctx.scenario.delete_object(
                    request.object_path),
                "DeleteObject")
            response.success = True
            response.message = "deleted"
        except Exception as exc:
            self._fail(response, exc, "DeleteObject")
        return response

    def _set_propagator(self, request, response):
        propagator = _PROPAGATOR_BY_INDEX.get(int(request.propagator_type))
        if propagator is None:
            response.success = False
            response.message = (
                f"unknown propagator_type {request.propagator_type}; "
                f"valid values are 0..4"
            )
            return response
        try:
            self._call(
                lambda backend: backend.set_propagator(
                    request.object_path, propagator,
                    look_ahead_s=request.realtime_look_ahead_s or 3600.0,
                    look_behind_s=request.realtime_look_behind_s or 3600.0,
                    external_file=request.external_ephemeris_path,
                ),
                "SetPropagator", timeout_s=60.0)
            # The sensing pipeline caches propagator type to avoid a
            # per-read query; that cache is now wrong for this object.
            self._ctx.sensing.invalidate_propagator_cache(request.object_path)
            response.success = True
            response.message = f"propagator set to {propagator.value}"
            if propagator is PropagatorType.REALTIME:
                response.message += (
                    "; note the analytic ephemeris is now discarded and "
                    "state exists only where samples are injected"
                )
        except Exception as exc:
            self._fail(response, exc, "SetPropagator")
        return response

    # -- queries -----------------------------------------------------------

    def _query_data_provider(self, request, response):
        from stk_msgs.msg import DataProviderResult

        result = DataProviderResult()
        source = request.request
        result.object_path = source.object_path
        result.provider_name = source.provider_name

        try:
            query = DataProviderQuery(
                object_path=source.object_path,
                provider_name=source.provider_name,
                group_element=source.group_name or None,
                elements=tuple(source.element_names),
                start=conv.ros_time_to_stk(source.start_time),
                stop=conv.ros_time_to_stk(source.stop_time),
                step_s=source.step_s or 1.0,
            )
            data = self._call(
                lambda backend: backend.query_data_provider(query),
                f"QueryDataProvider({source.provider_name})", timeout_s=120.0)

            names = list(data.columns)
            result.column_names = names
            result.column_units_scale = [1.0] * len(names)
            result.row_count = data.row_count

            flattened: List[float] = []
            for row in range(data.row_count):
                for name in names:
                    column = data.columns[name]
                    value = column[row] if row < len(column) else float("nan")
                    flattened.append(conv.as_float(value,
                                                   default=float("nan")))
            result.flattened_data = flattened
            result.success = True
        except Exception as exc:
            LOGGER.warning("QueryDataProvider failed: %s", exc)
            result.success = False
            result.error_message = f"{type(exc).__name__}: {exc}"

        response.result = result
        return response

    def _compute_access(self, request, response):
        from stk_msgs.msg import AccessInterval, AccessIntervalArray

        try:
            intervals = self._call(
                lambda backend: backend.compute_access(
                    request.from_object_path, request.to_object_path,
                    conv.ros_time_to_stk(request.start_time),
                    conv.ros_time_to_stk(request.stop_time)),
                "ComputeAccess", timeout_s=300.0)

            array = AccessIntervalArray()
            array.from_object_path = request.from_object_path
            array.to_object_path = request.to_object_path
            for interval in intervals:
                item = AccessInterval()
                item.start_time = conv.stk_time_to_ros(interval.start)
                item.stop_time = conv.stk_time_to_ros(interval.stop)
                item.duration_s = interval.duration_s
                array.intervals.append(item)

            response.accesses = array
            response.success = True
            response.message = f"{len(intervals)} interval(s)"
        except Exception as exc:
            self._fail(response, exc, "ComputeAccess")
        return response

    def _get_link_budget(self, request, response):
        from stk_msgs.msg import LinkBudget

        try:
            budget = self._call(
                lambda backend: backend.get_link_budget(
                    request.transmitter_path, request.receiver_path,
                    conv.ros_time_to_stk(request.at_time)),
                "GetLinkBudget", timeout_s=60.0)
            message = LinkBudget()
            message.transmitter_path = request.transmitter_path
            message.receiver_path = request.receiver_path
            message.carrier_to_noise_db = budget.carrier_to_noise_db
            message.eb_no_db = budget.eb_no_db
            message.margin_db = budget.margin_db
            message.received_power_dbw = budget.received_power_dbw
            message.free_space_loss_db = budget.free_space_loss_db
            message.link_closed = budget.link_closed
            response.link = message
            response.success = True
            response.message = "ok"
        except Exception as exc:
            self._fail(response, exc, "GetLinkBudget")
        return response

    # -- time --------------------------------------------------------------

    def _set_animation_time(self, request, response):
        time_sync = self._ctx.time_sync
        if time_sync.mode is ExecutionMode.LOCKSTEP:
            response.success = False
            response.message = (
                "SetAnimationTime is not valid in lockstep mode: the clock "
                "is advanced exclusively by the step loop, and moving it "
                "from outside would desynchronise every barrier "
                "participant. Use StepSimulation, or switch mode first."
            )
            return response
        try:
            target = conv.ros_time_to_stk(request.target_time)
            self._call(lambda backend: backend.set_current_time(target),
                       "SetAnimationTime")
            response.success = True
            response.message = "animation time set"
        except Exception as exc:
            self._fail(response, exc, "SetAnimationTime")
        return response

    def _step_simulation(self, request, response):
        time_sync = self._ctx.time_sync
        if time_sync.mode is not ExecutionMode.LOCKSTEP:
            response.success = False
            response.message = (
                f"StepSimulation is only valid in lockstep mode; current "
                f"mode is {time_sync.mode.name.lower()}"
            )
            response.step_index = time_sync.step_index
            return response
        try:
            now = self._ctx.request_step(request.step_s or 0.0)
            response.success = True
            response.message = "stepped"
            response.new_sim_time = conv.stk_time_to_ros(now)
            response.step_index = time_sync.step_index
        except Exception as exc:
            self._fail(response, exc, "StepSimulation")
            response.step_index = time_sync.step_index
        return response

    def _set_execution_mode(self, request, response):
        try:
            mode = ExecutionMode(int(request.mode))
            self._ctx.time_sync.set_mode(
                mode,
                real_time_factor=(request.real_time_factor
                                  if request.real_time_factor > 0 else None),
                participants=(list(request.lockstep_barrier_nodes)
                              if request.lockstep_barrier_nodes else None),
            )
            response.success = True
            response.message = f"execution mode set to {mode.name.lower()}"
        except Exception as exc:
            self._fail(response, exc, "SetExecutionMode")
        return response

    # -- escape hatch ------------------------------------------------------

    def _execute_connect(self, request, response):
        LOGGER.warning(
            "raw Connect command via ExecuteConnect: %r. This bypasses all "
            "typed validation -- frames, units, and monotonicity guards do "
            "not apply.", request.command,
        )
        try:
            result = self._call(
                lambda backend: backend.execute_connect(
                    request.command,
                    request.timeout_s if request.timeout_s > 0 else None),
                "ExecuteConnect",
                timeout_s=(request.timeout_s if request.timeout_s > 0
                           else 60.0))
            response.success = result.ack
            response.raw_response = "\n".join(result.messages)
            response.was_nack = not result.ack
        except Exception as exc:
            LOGGER.warning("ExecuteConnect failed: %s", exc)
            response.success = False
            response.raw_response = f"{type(exc).__name__}: {exc}"
            response.was_nack = True
        return response
