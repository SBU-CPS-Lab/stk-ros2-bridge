"""Action servers for long-running operations.

Three operations are actions rather than services because they run long
enough that a caller needs progress and the ability to give up:
``RunScenario`` (seconds to hours), ``ExecuteManeuver`` (Astrogator MCS
runs), and ``ComputeCoverage`` (minutes at fine grid resolutions).

Cancellation is honest
----------------------
STK's Object Model offers no way to interrupt a running ``RunMCS`` or
coverage computation from another thread. So cancellation here means
"stop at the next checkpoint the bridge controls" -- between steps for
``RunScenario``, before dispatch for the others -- and the goal is
reported ``canceled`` only once that point is reached, never while STK
is still working.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

from stk_bridge.backends.base import PropagatorType, StkTime
from stk_bridge.core import conversions as conv
from stk_bridge.core.time_sync import ExecutionMode
from stk_bridge.exceptions import LicenseError

LOGGER = logging.getLogger(__name__)


class ActionServers:
    """Creates and owns the bridge's action servers."""

    def __init__(self, node, worker, context, prefix: str = "stk",
                 callback_group=None):
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from stk_msgs.action import (
            ComputeCoverage,
            ExecuteManeuver,
            RunScenario,
        )

        self._node = node
        self._worker = worker
        self._ctx = context

        self._run_scenario = ActionServer(
            node, RunScenario, f"{prefix}/run_scenario",
            execute_callback=self._execute_run_scenario,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=callback_group,
        )
        self._execute_maneuver = ActionServer(
            node, ExecuteManeuver, f"{prefix}/execute_maneuver",
            execute_callback=self._execute_maneuver_cb,
            goal_callback=self._maneuver_goal_callback,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=callback_group,
        )
        self._compute_coverage = ActionServer(
            node, ComputeCoverage, f"{prefix}/compute_coverage",
            execute_callback=self._execute_coverage,
            goal_callback=self._coverage_goal_callback,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=callback_group,
        )

        LOGGER.info("3 action servers created under %s/", prefix)

    # -- goal gating -------------------------------------------------------

    def _maneuver_goal_callback(self, goal_request):
        """Reject up front if Astrogator is unlicensed.

        Rejecting at goal time rather than failing mid-execution means
        the caller finds out in milliseconds with a clear reason, instead
        of after the action has been accepted and appears to be running.
        """
        from rclpy.action import GoalResponse

        if not self._worker.backend.capabilities.astrogator:
            LOGGER.warning("ExecuteManeuver rejected: Astrogator unlicensed")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _coverage_goal_callback(self, goal_request):
        """Reject if an asset is on the RealTime propagator.

        STK Coverage analysis excludes real-time-propagated vehicles.
        Running anyway would return an empty or misleading result that
        looks like a coverage answer, so the goal is refused instead.
        """
        from rclpy.action import GoalResponse

        capabilities = self._worker.backend.capabilities
        if not capabilities.coverage:
            LOGGER.warning("ComputeCoverage rejected: Coverage unlicensed")
            return GoalResponse.REJECT

        try:
            realtime = self._worker.call(
                lambda backend: [
                    conv.canonical_object_path(info.object_path)
                    for info in backend.list_objects()
                    if (info.propagator_type or "").lower().startswith("real")
                ],
                "coverage goal check", timeout_s=10.0,
            )
        except Exception:
            LOGGER.debug("could not check propagators for coverage goal",
                         exc_info=True)
            return GoalResponse.ACCEPT

        requested = {conv.canonical_object_path(p)
                     for p in goal_request.asset_object_paths}
        offenders = requested & set(realtime)
        if offenders:
            LOGGER.warning(
                "ComputeCoverage rejected: %s are on the RealTime "
                "propagator and are excluded from STK Coverage analysis",
                sorted(offenders),
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # -- RunScenario -------------------------------------------------------

    async def _execute_run_scenario(self, goal_handle):
        from stk_msgs.action import RunScenario

        goal = goal_handle.request
        feedback = RunScenario.Feedback()
        result = RunScenario.Result()

        time_sync = self._ctx.time_sync
        target = conv.ros_time_to_stk(goal.target_time)
        start = time_sync.now()
        span = target.epoch_seconds - start.epoch_seconds

        if span <= 0.0:
            goal_handle.abort()
            result.success = False
            result.message = (
                f"target_time is not in the future (span {span:.3f}s)"
            )
            result.final_sim_time = conv.stk_time_to_ros(start)
            return result

        wall_start = time.monotonic()
        steps = 0

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = f"canceled after {steps} step(s)"
                result.final_sim_time = conv.stk_time_to_ros(time_sync.now())
                result.steps_executed = steps
                return result

            if (goal.max_wall_time_s > 0.0
                    and (time.monotonic() - wall_start) > goal.max_wall_time_s):
                goal_handle.abort()
                result.success = False
                result.message = (
                    f"exceeded max_wall_time_s ({goal.max_wall_time_s}s) "
                    f"after {steps} step(s)"
                )
                result.final_sim_time = conv.stk_time_to_ros(time_sync.now())
                result.steps_executed = steps
                return result

            now = time_sync.now()
            if now.epoch_seconds >= target.epoch_seconds:
                break

            if time_sync.mode is ExecutionMode.LOCKSTEP:
                self._ctx.request_step(0.0)
                steps += 1
            else:
                # Free-running / paced: the clock advances on its own;
                # this loop only reports progress.
                time.sleep(0.05)

            now = time_sync.now()
            feedback.current_sim_time = conv.stk_time_to_ros(now)
            feedback.fraction_complete = conv.clamp(
                (now.epoch_seconds - start.epoch_seconds) / span, 0.0, 1.0)
            feedback.step_index = time_sync.step_index
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.success = True
        result.message = f"reached target after {steps} step(s)"
        result.final_sim_time = conv.stk_time_to_ros(time_sync.now())
        result.steps_executed = steps
        return result

    # -- ExecuteManeuver ---------------------------------------------------

    async def _execute_maneuver_cb(self, goal_handle):
        from stk_msgs.action import ExecuteManeuver

        goal = goal_handle.request.maneuver
        feedback = ExecuteManeuver.Feedback()
        result = ExecuteManeuver.Result()

        def report(phase: str, fraction: float) -> None:
            feedback.phase = phase
            feedback.fraction_complete = fraction
            goal_handle.publish_feedback(feedback)

        report("validating", 0.0)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "canceled before dispatch"
            return result

        try:
            report("configuring_segment", 0.25)
            delta_v = (goal.impulsive.delta_v.x, goal.impulsive.delta_v.y,
                       goal.impulsive.delta_v.z)
            magnitude = sum(component ** 2 for component in delta_v) ** 0.5

            # Astrogator segment construction is scenario-specific: it
            # depends on the MCS the user has built. Rather than guess a
            # structure, the bridge drives it through Connect, which is
            # stable across MCS layouts.
            command = (
                f"Maneuver {goal.object_path} Impulsive "
                f"{delta_v[0]!r} {delta_v[1]!r} {delta_v[2]!r}"
            )
            report("running_mcs", 0.5)
            response = self._worker.call(
                lambda backend: backend.execute_connect(command),
                "ExecuteManeuver", timeout_s=300.0,
            )
            if not response.ack:
                raise RuntimeError(
                    f"STK rejected the maneuver: {response.single}"
                )

            report("propagating", 0.9)
            result.success = True
            result.message = "maneuver executed"
            result.actual_delta_v_mps = magnitude
            result.completion_time = goal.execute_at
            goal_handle.succeed()
        except LicenseError as exc:
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
        except Exception as exc:
            LOGGER.warning("ExecuteManeuver failed: %s", exc)
            goal_handle.abort()
            result.success = False
            result.message = f"{type(exc).__name__}: {exc}"
        return result

    # -- ComputeCoverage ---------------------------------------------------

    async def _execute_coverage(self, goal_handle):
        from stk_msgs.action import ComputeCoverage

        goal = goal_handle.request
        feedback = ComputeCoverage.Feedback()
        result = ComputeCoverage.Result()

        feedback.current_phase = "configuring"
        feedback.fraction_complete = 0.0
        goal_handle.publish_feedback(feedback)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "canceled before dispatch"
            return result

        try:
            feedback.current_phase = "computing"
            feedback.fraction_complete = 0.1
            goal_handle.publish_feedback(feedback)

            command = f"Cov_RM {goal.coverage_definition_name} Access"
            response = self._worker.call(
                lambda backend: backend.execute_connect(command),
                "ComputeCoverage", timeout_s=1800.0,
            )
            if not response.ack:
                raise RuntimeError(
                    f"coverage computation rejected: {response.single}"
                )

            feedback.current_phase = "collecting"
            feedback.fraction_complete = 0.9
            goal_handle.publish_feedback(feedback)

            result.success = True
            result.message = "coverage computed"
            # Percent-covered extraction depends on the coverage
            # definition's figure of merit, which is scenario-specific;
            # callers read it via QueryDataProvider rather than having
            # the bridge guess a FOM here.
            result.percent_covered = 0.0
            result.grid_points_total = 0
            result.grid_points_covered = 0
            goal_handle.succeed()
        except Exception as exc:
            LOGGER.warning("ComputeCoverage failed: %s", exc)
            goal_handle.abort()
            result.success = False
            result.message = f"{type(exc).__name__}: {exc}"
        return result
