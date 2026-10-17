"""Orchestrates ONE campaign run end to end, on the Windows deploy machine.

This is the launch command ``CommandLauncher`` (``acav/campaign_runner.py``)
runs per run -- every placeholder its argv template substitutes
(``{run_id}``, ``{experiment}``, ``{scheduler}``, ``{execution_mode}``,
``{seed}``, ``{output}``, ``{scenario}``, ``{backend}``,
``{attitude_provider}``, ``{run_config}``, ``{gains_json}``,
``{fault_json}``) is accepted here as a same-named CLI flag. See
``docs/08_deploy_runbook.md`` for how a colleague without this project's
context actually runs it.

Topology started, in this order, and why
------------------------------------------
1. **stk_bridge** (direct Python module process) -- the twin and the scheduler are
   both lockstep barrier participants *against* the bridge, so nothing
   else can usefully start first.
2. **the MATLAB/Simulink twin -- NOT started by this script.**
   ``build_acav_twin``'s model runs with ``StopTime = 'inf'`` (see
   ``simulink/build_acav_twin.m``): it runs for exactly as long as the
   bridge's ``/clock`` does, the same reason
   ``stk_bridge/launch/full_twin.launch.py`` leaves MATLAB out of its
   own launch file. This project has never run that model unattended,
   headless, end to end against a live bridge -- only interactively
   (``docs/C3_adcs_twin.md`` section 6). Scripting an unattended MATLAB
   step this project has not verified and calling it done would replace
   a measurement with a self-consistent, untested guess -- the specific
   failure mode this project has repeatedly had to catch and fix
   elsewhere (see e.g. ``docs/C6_analysis.md``). So this script stops
   and asks a human to start it, then *confirms* over ROS that the twin
   is actually acking the barrier before continuing -- it does not
   trust a keypress alone.
3. **acav_scheduler**, once the bridge is CONNECTED and the twin is
   observed acking the barrier. Starting it any earlier would spend the
   scheduler's own barrier-timeout budget waiting on a participant that
   was never going to show up in time, turning a "twin not started yet"
   situation into an "abort: barrier timeout" one instead.

What is captured
-----------------
Every subprocess's stdout/stderr goes to its own file under
``<output>.logs/`` (created next to the run's output JSON), plus this
script's own narrative in ``run.log`` and a ``summary.json`` recording
timestamps, exit codes, and the backend actually observed on
``BridgeStatus`` (so a COM run that silently fell back to mock is
visible in the log bundle, not just in the record's own provenance).
That is the "trace/logs for errors and metrics" a report from this
script is expected to carry back for analysis.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import logging
import os
import subprocess
import sys
import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

LOGGER = logging.getLogger(__name__)

#: Exit codes, matching the sense (not the values) of run_acav_campaign's
#: own convention: 0 completed, 1 a known step failed cleanly (readable
#: reason in the log), 2 the invocation itself was wrong.
EXIT_OK = 0
EXIT_STEP_FAILED = 1
EXIT_USAGE = 2

#: Launch to BridgeStatus CONNECTED took 43.7-76.1 s over 40 runs (P10
#: live session report); 90 s failed a run still inside validation. 180 s
#: is about 2.4x the slowest, and the wait ends early if the bridge exits.
BRIDGE_CONNECT_TIMEOUT_S = 180.0


@dataclass
class RunConfig:
    run_id: str
    experiment: str
    scheduler: str
    execution_mode: str
    seed: str
    output: str
    scenario: str
    backend: str
    attitude_provider: str
    run_config: str
    gains_json: str
    fault_json: str
    bridge_config: str
    prefix: str = "stk"
    log_dir: str = ""
    bridge_connect_timeout_s: float = BRIDGE_CONNECT_TIMEOUT_S
    twin_ack_timeout_s: float = 60.0
    ros_domain_id: str = ""
    #: STK graphics state the run requires: "off" (no 2D/3D Graphics
    #: window, STK minimised) unless "on" is asked for explicitly.
    graphics_mode: str = "off"
    #: time.real_time_factor for the bridge; None keeps the base config's.
    real_time_factor: Optional[float] = None
    #: A one-off measurement that report must never pool (summary.json).
    diagnostic_label: str = ""
    #: "manual" (a person builds the model and presses Play, as always)
    #: or "matlab-batch" (the runner starts and stops MATLAB itself, see
    #: acav/twin_launcher.py).
    twin_launch: str = "manual"
    #: Run on an STK Desktop started through COM, or one the runner
    #: cannot identify (acav/stk_process.py); refused otherwise.
    allow_com_started_stk: bool = False

    def resolved_log_dir(self) -> str:
        if self.log_dir:
            return self.log_dir
        base, _ = os.path.splitext(self.output)
        return f"{base}.logs"


@dataclass
class Deps:
    """Everything about the environment this orchestration touches.

    Bundled and injectable so the state machine below (bridge up -> twin
    confirmed -> scheduler run -> teardown) is testable without ROS, a
    subprocess, or a human at a keyboard -- see test_deploy_run_cli.py.
    """

    start_recorder: Callable = None
    check_idle: Callable[[str], None] = None
    popen: Callable[..., "subprocess.Popen"] = subprocess.Popen
    #: (prefix, timeout_s, ros_domain_id, bridge exit code or None)
    wait_for_bridge_connected: Callable[
        [str, float, str, Callable[[], Optional[int]]],
        "ConnectResult"] = None
    wait_for_twin_ack: Callable[[str, float], bool] = None
    #: (run config path, timeout) -> (identity text, problems); empty
    #: problems means the twin loaded this run's config and runs this
    #: checkout's code (see acav/twin_identity.py).
    confirm_twin_identity: Callable[[str, float],
                                    Tuple[str, List[str]]] = None
    #: (file, "startup" | "end", timeout) -> the bridge's graphics samples
    #: once that phase is present, else what is there (or None).
    read_graphics_state: Callable[[str, str, float],
                                  Optional[dict]] = None
    prompt: Callable[[str], None] = lambda message: input(message)  # noqa: E731
    sleep: Callable[[float], None] = time.sleep
    #: (run config path, log dir) -> a twin process with start(), stop()
    #: and exit_code(), for --twin-launch matlab-batch.
    launch_twin: Callable[[str, str], object] = None
    #: () -> (status, restore or None): QuickEdit off for this console
    #: (acav.console.disable_quickedit).
    disable_quickedit: Callable[[], Tuple[str, Optional[Callable]]] = None
    #: () -> every running STK Desktop process, each with started_by
    #: (acav.stk_process.stk_processes).
    find_stk_processes: Callable[[], List[dict]] = None

    def __post_init__(self) -> None:
        if self.start_recorder is None:
            from acav.topic_recorder import TopicRecorder
            self.start_recorder = TopicRecorder
        if self.check_idle is None:
            self.check_idle = _check_graph_idle
        if self.wait_for_bridge_connected is None:
            self.wait_for_bridge_connected = _wait_for_bridge_connected_ros
        if self.wait_for_twin_ack is None:
            self.wait_for_twin_ack = _wait_for_twin_ack_ros
        if self.confirm_twin_identity is None:
            self.confirm_twin_identity = _confirm_twin_identity_ros
        if self.read_graphics_state is None:
            self.read_graphics_state = _read_graphics_state_file
        if self.launch_twin is None:
            from acav.twin_launcher import MatlabBatchTwin
            self.launch_twin = MatlabBatchTwin
        if self.disable_quickedit is None:
            from acav.console import disable_quickedit
            self.disable_quickedit = disable_quickedit
        if self.find_stk_processes is None:
            self.find_stk_processes = _find_stk_processes


@dataclass
class ConnectResult:
    connected: bool
    backend_type: str = ""
    detail: str = ""


@contextlib.contextmanager
def _private_ros_node(name: str):
    """A node and executor on an rclpy Context of their own.

    Every ROS wait below runs in one. rclpy's default context is shared
    by the whole process, and rclpy.init() on it installs SIGINT/SIGTERM
    handlers that shut it down; each wait then ended in
    ``rclpy.shutdown()``, which raised "rcl_shutdown already called on
    the given context" and replaced the wait's own error (P10 live
    session, the first E6 attempt's idle_check). Here no signal handler
    is installed, and try_shutdown does nothing to a context that is
    already shut down or never started, so the wait's own exception is
    the one that propagates. Cleanup errors are not allowed to hide it.
    """
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    context = Context()
    node = executor = None
    try:
        rclpy.init(args=None, context=context,
                   signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node(name, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        yield node, executor
    finally:
        for close in (executor and executor.shutdown,
                      node and node.destroy_node):
            if close:
                try:
                    close()
                except Exception:       # noqa: BLE001 - must not mask
                    logging.getLogger(__name__).warning(
                        "%s: cleanup failed", name, exc_info=True)
        rclpy.try_shutdown(context=context)


def _probe_stk_processes() -> List[dict]:
    from acav.stk_process import stk_processes
    return stk_processes()


#: Deps' default; offline tests replace this hook (conftest.py).
_find_stk_processes = _probe_stk_processes


def _check_graph_idle(prefix: str) -> None:
    """Reject a shared graph before a new run can accept another run's status."""
    with _private_ros_node("acav_run_preflight") as (node, executor):
        deadline = time.monotonic() + 3.0
        topics = [f"/{prefix.strip('/')}/status",
                  f"/{prefix.strip('/')}/scenario_time",
                  f"/{prefix.strip('/')}/lockstep/ack", "/clock"]
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            active = [topic for topic in topics if node.count_publishers(topic)]
            if active:
                raise RuntimeError("Another run is active on " + ", ".join(active)
                                   + "; stop its bridge, scheduler and twin first")


def bridge_launch_argv(config: RunConfig,
                       config_file: Optional[str] = None) -> List[str]:
    # Own the actual node process, not a ros2 launcher whose children survive
    # terminate() on Windows. Keep the same interpreter and ROS environment.
    config_file = config_file or config.bridge_config
    return [sys.executable, "-m", "stk_bridge.nodes.bridge_node", "--ros-args",
            "-p", f"config_file:={json.dumps(config_file)}"]


#: The per-run bridge configuration, written beside the run's logs.
RUN_BRIDGE_CONFIG = "bridge_params.yaml"
#: The execution modes a run may request: E3 compares exactly these two.
EXECUTION_MODES = ("lockstep", "free_run")
#: The bridge's STK graphics samples for the run, beside the run's logs.
GRAPHICS_STATE_FILE = "stk_graphics.json"
#: A free-run twin's sub-step statistics, written by adcs.simulink_step.
TWIN_SUBSTEPS_FILE = "twin_substeps.json"
#: A free-run bridge's pacing summary (cycles, overruns, period_s).
FREE_RUN_PACING_FILE = "free_run_pacing.json"
#: When a free-run bridge's start gate opened, and the wall time it held.
STARTUP_GATE_FILE = "startup_gate.json"
#: Each bridge startup phase's duration, written by the bridge as it goes
#: (stk_bridge core/startup_phases.py).
STARTUP_PHASES_FILE = "bridge_startup_phases.json"
#: BridgeStatus.state names, for failure details.
BRIDGE_STATES = {0: "STARTING", 1: "CONNECTED", 2: "DEGRADED",
                 3: "DISCONNECTED", 4: "ERROR"}
#: The startup sample is written before the bridge reports CONNECTED;
#: the terminal one at the last step, before the scheduler can finish.
GRAPHICS_STARTUP_TIMEOUT_S = 10.0
GRAPHICS_END_TIMEOUT_S = 30.0


def write_bridge_config(config: RunConfig, log_dir: str, logger) -> str:
    """Copy ``--bridge-config`` with this run's attitude provider applied.

    ``sensing.force_attitude_provider`` is set from the run, always: to
    the forced provider for E1's treatment and to empty for every other
    run. Before this, ``--attitude-provider`` reached only the scheduler
    -- it labelled the record without changing what the bridge read
    through, so a treatment needed a hand-made YAML and a value left in
    a shared YAML would have silently turned a nominal run into one. The
    base file is not edited; the copy stays with the logs as the record
    of what the bridge was given.
    """
    import yaml

    if config.execution_mode not in EXECUTION_MODES:
        raise ValueError(f"unknown execution mode {config.execution_mode!r}; "
                         f"expected one of {list(EXECUTION_MODES)}")
    with open(config.bridge_config, encoding="utf-8-sig") as handle:
        settings = yaml.safe_load(handle) or {}
    if not isinstance(settings, dict):
        raise ValueError(f"{config.bridge_config} is not a YAML mapping")
    # The run's execution mode, always: before this the base file's mode
    # (lockstep) was used whatever the run asked for, so an E3 free-run
    # run silently ran in lockstep.
    settings.setdefault("time", {})
    settings["time"]["mode"] = config.execution_mode
    if config.real_time_factor is not None:
        settings["time"]["real_time_factor"] = config.real_time_factor
    sensing = settings.get("sensing") or {}
    previous = str(sensing.get("force_attitude_provider") or "")
    if previous != config.attitude_provider:
        logger.warning(
            "%s sets sensing.force_attitude_provider=%r; this run uses %r",
            config.bridge_config, previous, config.attitude_provider)
    sensing["force_attitude_provider"] = config.attitude_provider
    # The opt-in equivalence diagnostic writes beside this run's logs.
    if (sensing.get("equivalence_diagnostic")
            and not sensing.get("equivalence_diagnostic_dir")):
        sensing["equivalence_diagnostic_dir"] = os.path.abspath(log_dir)
    # Sensing optimisations (decisions Q2/Q3): E1 compares attitude
    # read-back conventions, so both arms keep the original sensing path,
    # and a forced provider is never combined. The effective values are
    # written explicitly, with the mode label, so the copy beside the logs
    # says which path this run used.
    combine = bool(sensing.get("combine_attitude_rate", False))
    batch = int(sensing.get("orbit_batch_samples", 1) or 1)
    if config.experiment == "E1" or config.attitude_provider:
        combine = False
    if config.experiment == "E1":
        batch = 1
    # Free-run: (B) can never verify (no sense epoch is a written sample)
    # and (C) reads epochs ahead of the wall clock, which an asynchronous
    # loop must not do; written off so the record says what ran.
    if config.execution_mode == "free_run":
        combine = False
        batch = 1
    sensing["combine_attitude_rate"] = combine
    sensing["orbit_batch_samples"] = batch
    sensing["sensing_mode"] = sensing_mode_label(combine)
    # Decision Q4: the bridge checks STK's stop time against the window.
    if batch > 1 and sensing.get("orbit_batch_expected_window_s") is None:
        window = _scenario_window_s(config, logger)
        if window is not None:
            sensing["orbit_batch_expected_window_s"] = window
    settings["sensing"] = sensing
    # Where the bridge records STK's graphics state for this run.
    settings["graphics_state_file"] = os.path.join(
        os.path.abspath(log_dir), GRAPHICS_STATE_FILE)
    # Free-run only: where the bridge records when its start gate opened
    # and its pacing summary.
    if config.execution_mode == "free_run":
        settings["startup_gate_file"] = os.path.join(
            os.path.abspath(log_dir), STARTUP_GATE_FILE)
        settings["free_run_pacing_file"] = os.path.join(
            os.path.abspath(log_dir), FREE_RUN_PACING_FILE)
    # Where the bridge times its startup phases, as they happen.
    settings["startup_phases_file"] = os.path.join(
        os.path.abspath(log_dir), STARTUP_PHASES_FILE)
    path = os.path.join(log_dir, RUN_BRIDGE_CONFIG)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(settings, handle, sort_keys=False)
    return os.path.abspath(path)


def _scenario_window_s(config: RunConfig, logger) -> Optional[float]:
    """The scenario specification's window, or None if it cannot be read."""
    try:
        from acav.spec import AcavSpec

        return float(AcavSpec.load(config.scenario or None).window_duration_s)
    except Exception as error:              # noqa: BLE001 - reported
        logger.warning("could not read the scenario window for the orbit "
                       "batch stop-time cross-check: %s", error)
        return None


def sensing_mode_label(combine_attitude_rate: bool) -> str:
    """The configured sensing mode, as recorded with every run."""
    return "combined_attitude_rate" if combine_attitude_rate else "separate"


def _sensing_labels(bridge_config: str) -> dict:
    """What the per-run bridge config enables, for summary.json.

    ``sensing_mode`` is the configured mode; whether (B) actually
    activated is in bridge.log and in the provider BridgeStatus reports.
    """
    import yaml

    with open(bridge_config, encoding="utf-8") as handle:
        settings = yaml.safe_load(handle) or {}
    sensing = settings.get("sensing") or {}
    settings_time = settings.get("time") or {}
    return {
        "real_time_factor": float(settings_time.get("real_time_factor", 1.0)),
        "equivalence_diagnostic": bool(sensing.get("equivalence_diagnostic")),
        "sensing_mode": sensing_mode_label(
            bool(sensing.get("combine_attitude_rate", False))),
        "orbit_batch_samples": int(sensing.get("orbit_batch_samples", 1) or 1),
    }


class RunConfigError(ValueError):
    """The run's MATLAB run configuration cannot be used or created."""


def _json_argument(text: str, flag: str):
    """A ``--gains-json``/``--fault-json`` value, or None when empty."""
    if not (text or "").strip():
        return None
    try:
        return json.loads(text)
    except ValueError as error:
        raise RunConfigError(f"{flag} is not valid JSON: {error}") from error


def standalone_run_config(config: RunConfig, version: str) -> dict:
    """The run configuration a campaign would have written for this run.

    Same fields as ``campaign_runner.write_run_config``, from this run's
    own arguments: a standalone run has no ``RunSpec``, and its run id is
    the operator's, not one derived from the configuration.
    """
    gains = _json_argument(config.gains_json, "--gains-json")
    if gains is None:
        gains = {}
    if not isinstance(gains, dict):
        raise RunConfigError("--gains-json must be a JSON object")
    fault = _json_argument(config.fault_json, "--fault-json")
    if fault is not None and not isinstance(fault, dict):
        raise RunConfigError("--fault-json must be a JSON object or null")
    return {
        "run_id": config.run_id,
        "gains": gains,
        "fault": fault,
        "experiment": config.experiment,
        "attitude_provider": config.attitude_provider,
        "code_version": version,
        # Selects the twin's integration (+adcs/config.m): fixed lockstep
        # steps, or free-run sub-stepping.
        "execution_mode": config.execution_mode,
        # The campaign writer's rule, which +adcs/config.m enforces:
        # the ablation only for an E1 run with a forced provider.
        "attitude_mismatch_policy": (
            "e1_readback_ablation" if config.experiment == "E1" and
            config.attitude_provider else "strict"),
    }


def prepare_run_config(config: RunConfig, version: str):
    """Make sure the twin's run configuration exists; return its path.

    Returns ``(absolute path, created, run_id_verified)``; the last is
    False only for an existing file without a ``run_id`` key, which
    cannot be shown to belong to this run. A missing file is written from
    this run's arguments, as UTF-8 without a byte-order mark. An existing
    one -- a campaign run's, or an operator's override -- is left
    byte-for-byte unchanged, but must be a JSON object for this run: a
    file the twin cannot read, or one written for another run, is
    rejected here rather than discovered after the bridge has started.
    """
    from acav.campaign_runner import run_config_path

    path = os.path.abspath(config.run_config or run_config_path(config.output))
    if os.path.exists(path):
        try:
            with open(path, "rb") as handle:
                raw = handle.read()
        except OSError as error:
            raise RunConfigError(f"cannot read {path}: {error}") from error
        if raw.startswith(b"\xef\xbb\xbf"):
            raise RunConfigError(
                f"{path} starts with a UTF-8 byte-order mark (Windows "
                f"PowerShell Out-File adds one); re-save it as UTF-8 "
                f"without a byte-order mark, or delete it so it is "
                f"regenerated from this run's arguments")
        try:
            payload = json.loads(raw.decode("utf-8"))
        except ValueError as error:
            raise RunConfigError(
                f"{path} is not valid UTF-8 JSON: {error}") from error
        if not isinstance(payload, dict):
            raise RunConfigError(
                f"{path} must contain a JSON object, not "
                f"{type(payload).__name__}")
        if "run_id" in payload and payload["run_id"] != config.run_id:
            raise RunConfigError(
                f"{path} is for run {payload['run_id']!r}, not "
                f"{config.run_id!r}")
        # Without the key the twin integrates as in lockstep, which is
        # only right for a lockstep run.
        recorded_mode = payload.get("execution_mode", "lockstep")
        if recorded_mode != config.execution_mode:
            raise RunConfigError(
                f"{path} has execution_mode {recorded_mode!r} but this run "
                f"is {config.execution_mode!r}; the twin would integrate "
                f"for the wrong mode")
        return path, False, "run_id" in payload

    payload = standalone_run_config(config, version)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
    return path, True, True


def scheduler_argv(config: RunConfig, twin_identity: str = "") -> List[str]:
    # ROS parses parameter values as YAML. Quote strings explicitly so an
    # empty value, JSON payload, numeric-looking ID or Windows path retains
    # its string type. subprocess receives an argv list, not shell text.
    strings = {
        # The identity the run orchestrator accepted, for the record.
        "twin_identity": twin_identity,
        "scenario_config": config.scenario,
        "scheduler_type": config.scheduler,
        "prefix": config.prefix,
        "output_json": config.output,
        "run_id": config.run_id,
        "experiment": config.experiment,
        "gains_json": config.gains_json,
        "fault_json": config.fault_json,
        "attitude_provider": config.attitude_provider,
    }
    argv = [sys.executable, "-m", "acav.scheduler_node", "--ros-args"]
    for name, value in strings.items():
        argv += ["-p", f"{name}:={json.dumps(value)}"]
    argv += ["-p", "auto_shutdown:=true"]
    if config.seed:
        argv += ["-p", f"seed:={config.seed}"]
    return argv


def _wait_for_bridge_connected_ros(
        prefix: str, timeout_s: float, ros_domain_id: str,
        bridge_exit_code: Optional[Callable[[], Optional[int]]] = None,
) -> ConnectResult:
    """Poll ``{prefix}/status`` for ``state == CONNECTED``.

    Ends at once if ``bridge_exit_code()`` says the bridge process has
    exited: waiting out the bound for a dead bridge only delays the
    failure.

    Imported lazily -- this module must stay importable (for the offline
    unit tests) on a machine with no ROS 2 sourced.

    Uses ``stk_bridge.interfaces.qos.status_qos()`` rather than a
    hand-rolled profile, and that is load-bearing, not a style
    preference: the bridge publishes ``{prefix}/status`` RELIABLE +
    TRANSIENT_LOCAL (interfaces/qos.py's own docstring -- so a
    late-joining watcher like this one sees current state immediately).
    A subscriber whose durability does not request at least
    TRANSIENT_LOCAL never matches that publisher at the DDS layer -- not
    "arrives late", *never arrives*, with no error on either side. An
    earlier version of this function used VOLATILE and reproduced
    exactly that: the bridge was running and CONNECTED the whole time,
    and this function reported "no BridgeStatus received at all" for the
    full timeout regardless.
    """
    from stk_bridge.interfaces.qos import status_qos
    from stk_msgs.msg import BridgeStatus

    seen: List[BridgeStatus] = []

    with _private_ros_node("deploy_run_wait_for_bridge") as (node, executor):
        qos = status_qos(depth=5)
        node.create_subscription(
            BridgeStatus, f"{prefix}/status", seen.append, qos)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.5)
            code = bridge_exit_code() if bridge_exit_code else None
            if code is not None:
                return ConnectResult(
                    False, "", f"the bridge exited (code {code}) before "
                               f"it reported CONNECTED")
            if node.count_publishers(f"{prefix}/status") > 1:
                return ConnectResult(False, "", "multiple bridge status publishers detected")
            if seen:
                latest = seen[-1]
                if latest.state == BridgeStatus.CONNECTED:
                    return ConnectResult(True, latest.backend_type)
                if latest.state == BridgeStatus.ERROR:
                    return ConnectResult(
                        False, latest.backend_type,
                        f"bridge reported ERROR: {latest.last_error}")
        if seen:
            return ConnectResult(
                False, seen[-1].backend_type,
                f"timed out; last state={seen[-1].state} "
                f"({BRIDGE_STATES.get(seen[-1].state, 'unknown')})")
        return ConnectResult(False, "", "no BridgeStatus received at all")


def _wait_for_twin_ack_ros(prefix: str, timeout_s: float) -> bool:
    """Poll ``{prefix}/lockstep/ack`` for a message from ``matlab_twin``.

    Confirms the twin is not merely running but actually participating
    in the barrier -- the specific thing that matters, since a Simulink
    model can be "running" (Play pressed) yet not have its Subscribe/
    Publish blocks wired to the right topics.

    Uses ``stk_bridge.interfaces.qos.command_qos()`` -- the profile the
    bridge's own ack subscriber uses (``ros_subscribers.py``) -- for the
    same reason ``_wait_for_bridge_connected_ros`` now uses
    ``status_qos()``: a durability/reliability mismatch on this topic
    would not fail loudly, it would just never deliver a message, and
    this function has no way to tell that apart from "the twin really
    isn't acking".
    """
    from stk_bridge.interfaces.qos import command_qos, clock_qos
    from stk_msgs.msg import ScenarioTime
    from std_msgs.msg import String

    seen: List[str] = []
    ticks = []

    with _private_ros_node("deploy_run_wait_for_twin") as (node, executor):
        node.create_subscription(ScenarioTime, f"{prefix}/scenario_time",
                                 ticks.append, clock_qos())
        node.create_subscription(
            String, f"{prefix}/lockstep/ack",
            lambda message: seen.append(str(message.data)), command_qos())

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.5)
            if ticks and f"matlab_twin:{ticks[-1].step_index}" in seen:
                return True
        return False


def _read_json_or_none(path: str) -> Optional[dict]:
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None


def _bridge_startup_fields(phases: Optional[dict]) -> dict:
    """summary.json's view of bridge_startup_phases.json (None: no file)."""
    phases = phases or {}
    return {
        "bridge_startup": phases or None,
        "bridge_last_phase": (phases.get("current")
                              or phases.get("last_phase") or None),
        "bridge_phase_durations": {
            entry["phase"]: entry["s"]
            for entry in phases.get("phases") or []},
    }


def _startup_phase_note(phases: Optional[dict]) -> str:
    """Where the bridge's startup had got to, for a step's detail."""
    if not phases:
        return (f"no {STARTUP_PHASES_FILE} (the bridge never began timing "
                f"its startup)")
    if phases.get("current"):
        return (f"bridge startup phase: {phases['current']} since "
                f"{phases.get('current_since', '?')}")
    if phases.get("finished"):
        return (f"bridge startup {phases.get('outcome', '')}"
                + (f": {phases['error']}" if phases.get("error") else ""))
    return f"bridge startup: last phase {phases.get('last_phase') or '-'}"


def _read_graphics_state_file(path: str, phase: str,
                              timeout_s: float) -> Optional[dict]:
    from stk_bridge.core.graphics_state import read

    deadline = time.monotonic() + timeout_s
    while True:
        data = read(path)
        if (data and data.get(phase)) or time.monotonic() >= deadline:
            return data
        time.sleep(0.5)


def _check_graphics_startup(config: RunConfig, data: Optional[dict],
                            summary: dict, record) -> bool:
    from stk_bridge.core.graphics_state import describe

    startup = (data or {}).get("startup")
    summary["graphics_mode_required"] = config.graphics_mode
    summary["graphics_startup"] = startup
    if not startup:
        record("stk_graphics", False,
               "the bridge reported no STK graphics state; this run "
               f"requires '{config.graphics_mode}'")
        return False
    if startup["mode"] != config.graphics_mode:
        if config.graphics_mode == "off":
            action = ("close every 2D and 3D Graphics window and minimise "
                      "STK, or pass --graphics-mode on deliberately")
        else:
            action = "open the graphics windows this run requires"
        record("stk_graphics", False,
               f"STK graphics is {describe(startup)} but this run requires "
               f"'{config.graphics_mode}': {action}")
        return False
    record("stk_graphics", True, describe(startup))
    return True


def _check_graphics_end(config: RunConfig, data: Optional[dict],
                        summary: dict, record) -> bool:
    """The whole run's mode, recorded and written into the record."""
    from stk_bridge.core.graphics_state import UNVERIFIED, summarize

    data = data or {}
    mode = summarize(data) or UNVERIFIED
    summary["graphics_mode"] = mode
    summary["graphics_samples"] = data.get("samples") or []
    summary["graphics_end"] = data.get("end")
    ok = mode == config.graphics_mode
    record("stk_graphics_end", ok,
           mode if ok else f"{mode}; this run required "
                           f"'{config.graphics_mode}' throughout")
    _label_record(config.output, graphics_mode=mode)
    return ok


def _label_record(output: str, **provenance) -> None:
    """Write run conditions into the run's record, if there is one.

    Provenance only: the determinism signature is computed from the
    result, so it is unchanged.
    """
    import dataclasses

    from acav.run_record import read_record, write_record

    if not os.path.exists(output):
        return
    record = read_record(output)
    write_record(output, dataclasses.replace(
        record, provenance=dataclasses.replace(record.provenance,
                                               **provenance)))


#: The twin republishes its identity about once per wall second while
#: ready, so a few seconds after its first ack are plenty.
TWIN_IDENTITY_TIMEOUT_S = 10.0


def _confirm_twin_identity(run_config: str, timeout_s: float,
                           wait: Callable[[float], Optional[str]]
                           ) -> Tuple[str, List[str]]:
    """The twin's identity text, and every reason it does not fit."""
    from acav.twin_identity import (
        IDENTITY_TOPIC,
        check_identity,
        source_root,
    )

    text = wait(timeout_s)
    if text is None:
        return "", [
            f"no twin identity on {IDENTITY_TOPIC} within {timeout_s:.0f} "
            f"s; the model predates the identity check or was not built "
            f"for this code -- rebuild it with build_acav_twin "
            f"('RunConfigPath', '{run_config}')"]
    return text, check_identity(text, run_config, source_root())


def _confirm_twin_identity_ros(run_config: str, timeout_s: float
                               ) -> Tuple[str, List[str]]:
    return _confirm_twin_identity(run_config, timeout_s,
                                  _wait_for_twin_identity_ros)


def _wait_for_twin_identity_ros(timeout_s: float) -> Optional[str]:
    """The latest identity the twin published, or None.

    The twin publishes RELIABLE + VOLATILE, the same as its ack, about
    once per wall second while ready, so the bridge's own command_qos()
    matches it.
    """
    from stk_bridge.interfaces.qos import command_qos
    from std_msgs.msg import String

    from acav.twin_identity import IDENTITY_TOPIC

    seen: List[str] = []
    with _private_ros_node("deploy_run_wait_for_twin_identity") as (
            node, executor):
        node.create_subscription(
            String, IDENTITY_TOPIC,
            lambda message: seen.append(str(message.data)), command_qos())
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not seen:
            executor.spin_once(timeout_sec=0.5)
        return seen[-1] if seen else None


def _open_log(log_dir: str, name: str):
    os.makedirs(log_dir, exist_ok=True)
    return open(os.path.join(log_dir, name), "w", encoding="utf-8")


def _stop_process(process, name, logger):
    if process is not None and process.poll() is None:
        logger.info("stopping %s", name)
        process.terminate()
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                logger.error("%s did not exit after kill", name)


#: Wall-second bounds on the steps after the scheduler exits. A step
#: that has not finished by then is recorded as failed and the run goes
#: on (the step's thread is abandoned), so nothing after the scheduler
#: can hold a campaign. Each bound is well above the step's own timeouts:
#: the graphics end read waits GRAPHICS_END_TIMEOUT_S, stopping a process
#: up to 15 s + 15 s, stopping MATLAB twin_launcher.STOP_TIMEOUT_S plus a
#: kill.
POST_RUN_TIMEOUT_S = {
    "stk_graphics_end": 60.0,
    "bridge_stop": 60.0,
    "scheduler_stop": 60.0,
    "matlab_session": 180.0,
    "topic_capture": 60.0,
}


def _bounded(function, timeout_s: float):
    """Run ``function`` on a daemon thread for at most ``timeout_s``.

    Returns (finished, value); an exception it raised is raised here."""
    box: dict = {}

    def target():
        try:
            box["value"] = function()
        except BaseException as error:          # noqa: BLE001 - re-raised
            box["error"] = error

    thread = threading.Thread(target=target, daemon=True,
                              name="post-run step")
    thread.start()
    thread.join(timeout_s)
    if thread.is_alive():
        return False, None
    if "error" in box:
        raise box["error"]
    return True, box.get("value")


def execute_run(config: RunConfig, deps: Optional[Deps] = None) -> int:
    deps = deps or Deps()
    log_dir = config.resolved_log_dir()
    os.makedirs(log_dir, exist_ok=True)

    run_log_path = os.path.join(log_dir, "run.log")
    run_log = logging.getLogger(f"deploy_run.{config.run_id}")
    run_log.setLevel(logging.INFO)
    handler = logging.FileHandler(run_log_path, encoding="utf-8")
    handler.setFormatter(logging.Formatter(
        "%(asctime)s %(levelname)s %(message)s"))
    run_log.addHandler(handler)

    # Resolved here, from this checkout, and exported so the bridge and
    # scheduler (which may run from an install tree) record the same value.
    from acav import code_version
    version = code_version.describe(prefer_env=False)
    os.environ[code_version.ENV_VAR] = version
    run_log.info("code version: %s", version)
    if not code_version.is_clean(version):
        run_log.warning(
            "code version is %s, not a plain commit: results from this run "
            "cannot be attributed to a committed version of the code",
            version)

    summary = {
        "run_id": config.run_id, "experiment": config.experiment,
        "code_version": version,
        "execution_mode": config.execution_mode,
        "diagnostic_label": config.diagnostic_label,
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "steps": [],
        "errors": [],
    }

    def record(step: str, ok: bool, detail: str = "") -> None:
        summary["steps"].append({"step": step, "ok": ok, "detail": detail})
        (run_log.info if ok else run_log.error)("%s: %s", step, detail or "ok")

    def record_error(step: str, error: BaseException) -> None:
        """The exception itself, kept beside the step it failed."""
        text = "".join(traceback.format_exception(
            type(error), error, error.__traceback__))
        summary["errors"].append({"step": step,
                                  "type": type(error).__name__,
                                  "message": str(error), "traceback": text})
        run_log.error("%s raised %s", step, text.rstrip())

    # A QuickEdit selection in this console once held a run for 11.5 min
    # (E6 seed1, P9b): off while the run lasts, restored afterwards.
    quickedit, restore_quickedit = deps.disable_quickedit()
    summary["console_quickedit"] = quickedit
    post_run_failed = []

    def bounded(step: str, function):
        """A post-run step under POST_RUN_TIMEOUT_S[step]."""
        timeout_s = POST_RUN_TIMEOUT_S[step]
        finished, value = _bounded(function, timeout_s)
        if not finished:
            post_run_failed.append(step)
            record(step, False, f"timed out after {timeout_s:.0f} s; the "
                                f"run went on without it")
        return finished, value

    recorder = None
    bridge_proc = None
    scheduler_proc = None
    scheduler_returncode = EXIT_STEP_FAILED
    twin_state = {"twin": None, "done": False}

    def finish_twin() -> bool:
        """matlab-batch only: stop the twin once, record how it ended."""
        twin = twin_state["twin"]
        if twin is None or twin_state["done"]:
            return True
        twin_state["done"] = True
        from acav.twin_launcher import MATLAB_SESSION_FILE
        finished, result = bounded("matlab_session", twin.stop)
        if not finished:
            return False
        session = _read_json_or_none(
            os.path.join(log_dir, MATLAB_SESSION_FILE))
        summary["matlab_session_result"] = result.to_dict()
        summary["matlab_session"] = session
        version = str((session or {}).get("matlab_version", "") or "")
        summary["matlab_version"] = version
        ok = (result.how == "clean_stop" and session is not None
              and session.get("exit") == "clean_stop")
        record("matlab_session", ok,
               f"{result.how}, exit code {result.exit_code}"
               + (f": {result.detail}" if result.detail else ""))
        if version:
            _label_record(config.output, matlab_version=version)
        return ok

    stopped = set()          # processes already stopped, or abandoned

    def stop(name, process) -> None:
        """Stop a launched process once, under its post-run bound."""
        if name in stopped or process is None:
            return
        stopped.add(name)
        bounded(f"{name}_stop", lambda: _stop_process(process, name, run_log))

    in_flight = None
    try:
        # Before anything is launched: the twin cannot start without it.
        try:
            run_config, created, verified = prepare_run_config(
                config, version)
        except RunConfigError as error:
            record("run_config", False, str(error))
            return EXIT_STEP_FAILED
        summary["run_config"] = run_config
        summary["run_config_run_id_verified"] = verified
        if config.scenario:
            # The twin always reads the reference scenario: a scenario
            # that differs from it anywhere but its targets would give the
            # twin other parameters than the scheduler.
            from acav.run_record import scenario_digest
            from acav.scenarios import (scenario_origin,
                                        twin_parameter_differences)
            try:
                differences = twin_parameter_differences(config.scenario)
            except Exception as error:      # noqa: BLE001 - recorded
                record("scenario", False,
                       f"cannot load {config.scenario}: {error}")
                return EXIT_STEP_FAILED
            if differences:
                record("scenario", False,
                       f"{config.scenario} differs from the reference "
                       f"scenario in {', '.join(differences)}; the twin "
                       f"reads the reference, so this run would give it "
                       f"other parameters")
                return EXIT_STEP_FAILED
            seed, generator_version = scenario_origin(config.scenario)
            summary["scenario"] = {
                "path": config.scenario,
                "sha256": scenario_digest(config.scenario),
                "seed": seed, "generator_version": generator_version}
            record("scenario", True,
                   f"{config.scenario} (seed {seed}); the twin's sections "
                   f"match the reference")
        if not verified:
            run_log.warning("run config has no run_id; cannot verify it "
                            "belongs to this run: %s", run_config)
        run_log.info("run config (%s): %s",
                     "created" if created else "existing, unchanged",
                     run_config)
        try:
            bridge_config = write_bridge_config(config, log_dir, run_log)
        except Exception as error:      # noqa: BLE001 - OSError, YAMLError, a non-mapping file
            record("bridge_config", False,
                   f"could not prepare {config.bridge_config}: {error}")
            return EXIT_STEP_FAILED
        run_log.info("bridge config: %s (force_attitude_provider=%r)",
                     bridge_config, config.attitude_provider)
        summary.update(_sensing_labels(bridge_config))
        run_log.info("sensing mode: %s, orbit_batch_samples=%d",
                     summary["sensing_mode"], summary["orbit_batch_samples"])
        if summary["equivalence_diagnostic"]:
            run_log.warning("sensing equivalence diagnostic ON: this run's "
                            "timings are inflated and it is not an E6 "
                            "measurement")
        # A refusal here (another run still publishing) or a failure of
        # the check itself is a recorded step, not only a console traceback.
        try:
            deps.check_idle(config.prefix)
        except Exception as error:      # noqa: BLE001 - recorded, then fail
            record_error("idle_check", error)
            record("idle_check", False, str(error))
            return EXIT_STEP_FAILED
        record("idle_check", True)
        recorder = deps.start_recorder(config, log_dir)
        run_log.info("topic capture started: %s", log_dir)
        with _open_log(log_dir, "bridge.log") as bridge_log:
            argv = bridge_launch_argv(config, bridge_config)
            run_log.info("launching bridge: %s", " ".join(argv))
            launched = time.monotonic()
            bridge_proc = deps.popen(
                argv, stdout=bridge_log, stderr=subprocess.STDOUT)

            def record_startup(connected: bool) -> str:
                """The bridge's startup record; the phase it was in."""
                phases = _read_json_or_none(
                    os.path.join(log_dir, STARTUP_PHASES_FILE))
                summary.update(_bridge_startup_fields(phases))
                summary["bridge_connect_s"] = (
                    round(time.monotonic() - launched, 1) if connected
                    else None)
                summary["bridge_exit_code"] = bridge_proc.poll()
                return _startup_phase_note(phases)

            try:
                connect_result = deps.wait_for_bridge_connected(
                    config.prefix, config.bridge_connect_timeout_s,
                    config.ros_domain_id, bridge_proc.poll)
            except Exception as error:  # noqa: BLE001 - recorded, then fail
                record_error("bridge_connected", error)
                note = record_startup(False)
                record("bridge_connected", False, f"{error}; {note}")
                return EXIT_STEP_FAILED
            note = record_startup(connect_result.connected)
            if not connect_result.connected:
                record("bridge_connected", False,
                       f"{connect_result.detail}; {note}")
                return EXIT_STEP_FAILED
            record("bridge_connected", True,
                   connect_result.detail or connect_result.backend_type)
            # Which STK the bridge attached to, and who started it: an
            # STK started through COM is refused (acav/stk_process.py).
            summary["stk_process"] = None
            summary["allow_com_started_stk"] = config.allow_com_started_stk
            if config.backend == "com":
                from acav.stk_process import assess
                process, stk_ok, detail = assess(
                    deps.find_stk_processes(),
                    config.allow_com_started_stk)
                summary["stk_process"] = process
                record("stk_process", stk_ok, detail)
                if stk_ok and process.get("started_by") != "operator":
                    run_log.warning("this run's STK is not an operator-"
                                    "started one: %s", detail)
                if not stk_ok:
                    return EXIT_STEP_FAILED
            if (config.backend not in ("mock",)
                    and connect_result.backend_type == "mock"):
                # Not fatal by itself -- some backends legitimately fall
                # back -- but this run's record will be unreportable, and
                # that must be visible in THIS log, not only discovered
                # later when C6 silently excludes it.
                run_log.warning(
                    "requested backend %r but bridge reports 'mock' -- "
                    "this run will not be reportable (see run_record.py)",
                    config.backend)

            # STK's graphics state is a run condition: check it before
            # any handoff time is spent.
            graphics_file = os.path.join(os.path.abspath(log_dir),
                                         GRAPHICS_STATE_FILE)
            if not _check_graphics_startup(
                    config, deps.read_graphics_state(
                        graphics_file, "startup",
                        GRAPHICS_STARTUP_TIMEOUT_S),
                    summary, record):
                return EXIT_STEP_FAILED

            ack_timeout_s = config.twin_ack_timeout_s
            if config.twin_launch == "matlab-batch":
                from acav.twin_launcher import AUTOMATED_TWIN_ACK_TIMEOUT_S
                ack_timeout_s = AUTOMATED_TWIN_ACK_TIMEOUT_S
                summary["twin_launch"] = config.twin_launch
                summary["twin_ack_timeout_s"] = ack_timeout_s
                twin_state["twin"] = deps.launch_twin(run_config, log_dir)
                twin_state["twin"].start()
                record("twin_launch", True,
                       " ".join(twin_state["twin"].argv))
            else:
                deps.prompt(
                    "\n"
                    "=== Start the MATLAB/Simulink twin now ===\n"
                    f"  matlab -batch \"cd('src/stk_matlab'); "
                    f"setup_stk_ros; "
                    f"build_acav_twin('RunConfigPath', '{run_config}')\"\n"
                    "then open the generated acav_twin.slx and press Play.\n"
                    "Press Enter here once it is running "
                    "(this script will then WAIT for its barrier ack, not "
                    "just trust the keypress): ")

            try:
                twin_ok = deps.wait_for_twin_ack(config.prefix,
                                                 ack_timeout_s)
            except Exception as error:  # noqa: BLE001 - recorded, then fail
                record_error("twin_acking", error)
                record("twin_acking", False, str(error))
                return EXIT_STEP_FAILED
            detail = "" if twin_ok else ("no matlab_twin ack seen before "
                                         "timeout -- check the Simulink "
                                         "model's Subscribe/Publish topics")
            if not twin_ok and twin_state["twin"] is not None:
                code = twin_state["twin"].exit_code()
                if code is not None:
                    detail = (f"MATLAB exited (code {code}) before the twin "
                              f"acked; see matlab.log")
            record("twin_acking", twin_ok, detail)
            if not twin_ok:
                return EXIT_STEP_FAILED

            # The twin must have loaded THIS run's config and run this
            # checkout's code; otherwise the scheduler is never started,
            # and the bridge cannot advance without it.
            try:
                identity, problems = deps.confirm_twin_identity(
                    run_config, TWIN_IDENTITY_TIMEOUT_S)
            except Exception as error:  # noqa: BLE001 - recorded, then fail
                record_error("twin_identity", error)
                record("twin_identity", False, str(error))
                return EXIT_STEP_FAILED
            summary["twin_identity"] = identity
            summary["twin_identity_problems"] = list(problems)
            record("twin_identity", not problems,
                   identity if not problems else "; ".join(problems))
            if problems:
                return EXIT_STEP_FAILED

            with _open_log(log_dir, "scheduler.log") as scheduler_log:
                argv = scheduler_argv(config, twin_identity=identity)
                run_log.info("launching scheduler: %s", " ".join(argv))
                scheduler_proc = deps.popen(
                    argv, stdout=scheduler_log, stderr=subprocess.STDOUT)
                while True:
                    try:
                        scheduler_returncode = scheduler_proc.wait(timeout=1.0)
                        break
                    except subprocess.TimeoutExpired:
                        if bridge_proc.poll() is not None:
                            record("bridge_alive", False, "bridge exited while scheduler was running")
                            return EXIT_STEP_FAILED
                record("scheduler_exit", scheduler_returncode == 0,
                       f"returncode={scheduler_returncode}")

            # Before the bridge is stopped: its terminal sample is the last.
            finished, data = bounded("stk_graphics_end", lambda: (
                deps.read_graphics_state(graphics_file, "end",
                                         GRAPHICS_END_TIMEOUT_S)))
            graphics_ok = finished and _check_graphics_end(
                config, data, summary, record)
            _label_record(config.output,
                          real_time_factor=summary["real_time_factor"],
                          stk_process=summary["stk_process"])
            gate_ok = True
            if config.execution_mode == "free_run":
                # The twin's ZOH sub-step statistics (adcs.simulink_step);
                # None when no sub-step event was recorded.
                summary["twin_substeps"] = _read_json_or_none(
                    os.path.join(log_dir, TWIN_SUBSTEPS_FILE))
                # The start gate: the clock ran from t = 0 only once it
                # opened, so a run without it did not run the window.
                gate = _read_json_or_none(
                    os.path.join(log_dir, STARTUP_GATE_FILE))
                summary["free_run_start_gate"] = gate
                gate_ok = bool(gate and gate.get("opened"))
                summary["free_run_pacing"] = _read_json_or_none(
                    os.path.join(log_dir, FREE_RUN_PACING_FILE))
                record("free_run_start_gate", gate_ok,
                       f"opened after {gate['wait_s']:.1f} s" if gate_ok
                       else "the bridge never reported its start gate "
                            "open")

            # The scheduler no longer participates in the barrier. Stop the
            # bridge before waiting for a human, otherwise a successful run
            # can acquire a spurious missing-ack timeout during teardown.
            stop("bridge", bridge_proc)
            matlab_ok = True
            if twin_state["twin"] is None:
                deps.prompt(
                    "\n"
                    "=== Scheduler finished. Stop the Simulink twin now "
                    "(the model has no automatic stop) and press Enter to "
                    "continue: ")
            else:
                matlab_ok = finish_twin()

        return (EXIT_OK if scheduler_returncode == 0 and graphics_ok
                and gate_ok and matlab_ok else EXIT_STEP_FAILED)
    except BaseException as error:
        in_flight = error
        record_error("run", error)
        raise
    finally:
        for name, process in (("scheduler", scheduler_proc), ("bridge", bridge_proc)):
            stop(name, process)
        # A failed matlab-batch run still stops MATLAB and records how.
        finish_twin()
        if recorder is not None:
            finished, _ = bounded("topic_capture", recorder.close)
            if finished:
                record("topic_capture", not recorder.error,
                       recorder.error or "files saved")
        if restore_quickedit is not None:
            restore_quickedit()
        summary["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        summary["output_exists"] = os.path.exists(config.output)
        with open(os.path.join(log_dir, "summary.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2)
        run_log.removeHandler(handler)
        handler.close()
        if post_run_failed and in_flight is None:
            # A post-run step that never finished fails the run.
            return EXIT_STEP_FAILED  # noqa: B012


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    # Mirrors CommandLauncher's placeholders one for one.
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--experiment", default="")
    parser.add_argument("--scheduler", default="dynamics_aware")
    parser.add_argument("--execution-mode", default="lockstep",
                        choices=EXECUTION_MODES)
    parser.add_argument("--seed", default="")
    parser.add_argument("--output", required=True)
    parser.add_argument("--scenario", default="")
    parser.add_argument("--backend", default="com")
    parser.add_argument("--attitude-provider", default="")
    parser.add_argument("--run-config", default="")
    parser.add_argument("--gains-json", default="")
    parser.add_argument("--fault-json", default="")
    # Not a CommandLauncher placeholder -- deploy-machine environment.
    parser.add_argument("--bridge-config", required=True, metavar="PATH",
                        help="stk_bridge config with barrier_participants "
                             "including matlab_twin AND acav_scheduler "
                             "(see docs/08_deploy_runbook.md).")
    parser.add_argument("--prefix", default="stk")
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--bridge-connect-timeout-s", type=float,
                        default=BRIDGE_CONNECT_TIMEOUT_S)
    parser.add_argument("--twin-ack-timeout-s", type=float, default=60.0)
    parser.add_argument("--real-time-factor", type=_positive_finite,
                        default=None, metavar="RTF",
                        help="time.real_time_factor for this run's bridge "
                             "(free-run speed); default: the bridge "
                             "config's value")
    parser.add_argument("--graphics-mode", choices=("off", "on"),
                        default="off",
                        help="STK graphics state this run requires: off (no "
                             "2D/3D Graphics window, STK minimised; the "
                             "default) or on")
    parser.add_argument("--allow-com-started-stk", action="store_true",
                        help="run on an STK Desktop started through COM "
                             "(-Embedding) or one the runner cannot "
                             "identify; refused by default, since final "
                             "runs use an STK the operator started")
    parser.add_argument("--twin-launch", choices=("manual", "matlab-batch"),
                        default="manual",
                        help="manual (the default: build the twin and press "
                             "Play when asked) or matlab-batch (the runner "
                             "starts MATLAB with run_twin_session and stops "
                             "it after the run)")
    parser.add_argument("--diagnostic-label", default="", type=_label,
                        metavar="TEXT",
                        help="mark this run as a one-off diagnostic; "
                             "run_acav_campaign report refuses to pool it")
    return parser


def _positive_finite(text: str) -> float:
    import math
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}")
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite number > 0")
    return value


def _label(text: str) -> str:
    import re
    if text and not re.fullmatch(r"[A-Za-z0-9._-]{1,64}", text):
        raise argparse.ArgumentTypeError(
            "use 1-64 letters, digits, '.', '_' or '-'")
    return text


def main(argv: Optional[Sequence[str]] = None) -> int:
    # Console output on a background thread: a paused console must not
    # hold the run (acav/console.py).
    from acav.console import configure_console_logging
    configure_console_logging(logging.INFO)
    args = _parser().parse_args(argv)
    config = RunConfig(
        run_id=args.run_id, experiment=args.experiment,
        scheduler=args.scheduler, execution_mode=args.execution_mode,
        seed=args.seed, output=args.output, scenario=args.scenario,
        backend=args.backend, attitude_provider=args.attitude_provider,
        run_config=args.run_config, gains_json=args.gains_json,
        fault_json=args.fault_json, bridge_config=args.bridge_config,
        prefix=args.prefix, log_dir=args.log_dir,
        bridge_connect_timeout_s=args.bridge_connect_timeout_s,
        twin_ack_timeout_s=args.twin_ack_timeout_s,
        graphics_mode=args.graphics_mode,
        real_time_factor=args.real_time_factor,
        diagnostic_label=args.diagnostic_label,
        twin_launch=args.twin_launch,
        allow_com_started_stk=args.allow_com_started_stk,
    )
    return execute_run(config)


if __name__ == "__main__":
    sys.exit(main())
