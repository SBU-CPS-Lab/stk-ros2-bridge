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
import json
import logging
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence

LOGGER = logging.getLogger(__name__)

#: Exit codes, matching the sense (not the values) of run_acav_campaign's
#: own convention: 0 completed, 1 a known step failed cleanly (readable
#: reason in the log), 2 the invocation itself was wrong.
EXIT_OK = 0
EXIT_STEP_FAILED = 1
EXIT_USAGE = 2


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
    bridge_connect_timeout_s: float = 90.0
    twin_ack_timeout_s: float = 60.0
    ros_domain_id: str = ""

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
    wait_for_bridge_connected: Callable[[str, float, str], "ConnectResult"] = None
    wait_for_twin_ack: Callable[[str, float], bool] = None
    prompt: Callable[[str], None] = lambda message: input(message)  # noqa: E731
    sleep: Callable[[float], None] = time.sleep

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


@dataclass
class ConnectResult:
    connected: bool
    backend_type: str = ""
    detail: str = ""


def _check_graph_idle(prefix: str) -> None:
    """Reject a shared graph before a new run can accept another run's status."""
    import rclpy
    rclpy.init()
    node = rclpy.create_node("acav_run_preflight")
    try:
        deadline = time.monotonic() + 3.0
        topics = [f"/{prefix.strip('/')}/status",
                  f"/{prefix.strip('/')}/scenario_time",
                  f"/{prefix.strip('/')}/lockstep/ack", "/clock"]
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            active = [topic for topic in topics if node.count_publishers(topic)]
            if active:
                raise RuntimeError("Another run is active on " + ", ".join(active)
                                   + "; stop its bridge, scheduler and twin first")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def bridge_launch_argv(config: RunConfig,
                       config_file: Optional[str] = None) -> List[str]:
    # Own the actual node process, not a ros2 launcher whose children survive
    # terminate() on Windows. Keep the same interpreter and ROS environment.
    config_file = config_file or config.bridge_config
    return [sys.executable, "-m", "stk_bridge.nodes.bridge_node", "--ros-args",
            "-p", f"config_file:={json.dumps(config_file)}"]


#: The per-run bridge configuration, written beside the run's logs.
RUN_BRIDGE_CONFIG = "bridge_params.yaml"


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

    with open(config.bridge_config, encoding="utf-8-sig") as handle:
        settings = yaml.safe_load(handle) or {}
    if not isinstance(settings, dict):
        raise ValueError(f"{config.bridge_config} is not a YAML mapping")
    sensing = settings.get("sensing") or {}
    previous = str(sensing.get("force_attitude_provider") or "")
    if previous != config.attitude_provider:
        logger.warning(
            "%s sets sensing.force_attitude_provider=%r; this run uses %r",
            config.bridge_config, previous, config.attitude_provider)
    sensing["force_attitude_provider"] = config.attitude_provider
    settings["sensing"] = sensing
    path = os.path.join(log_dir, RUN_BRIDGE_CONFIG)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(settings, handle, sort_keys=False)
    return os.path.abspath(path)


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
        # The campaign writer's rule, which +adcs/config.m enforces:
        # the ablation only for an E1 run with a forced provider.
        "attitude_mismatch_policy": (
            "e1_readback_ablation" if config.experiment == "E1" and
            config.attitude_provider else "strict"),
    }


def prepare_run_config(config: RunConfig, version: str):
    """Make sure the twin's run configuration exists; return its path.

    Returns ``(absolute path, created)``. A missing file is written from
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
                f"{path} starts with a UTF-8 byte-order mark; rewrite it as "
                f"UTF-8 without one (Windows PowerShell Out-File adds it)")
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
        return path, False

    payload = standalone_run_config(config, version)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
    return path, True


def scheduler_argv(config: RunConfig) -> List[str]:
    # ROS parses parameter values as YAML. Quote strings explicitly so an
    # empty value, JSON payload, numeric-looking ID or Windows path retains
    # its string type. subprocess receives an argv list, not shell text.
    strings = {
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


def _wait_for_bridge_connected_ros(prefix: str, timeout_s: float,
                                   ros_domain_id: str) -> ConnectResult:
    """Poll ``{prefix}/status`` for ``state == CONNECTED``.

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
    import rclpy
    from stk_bridge.interfaces.qos import status_qos
    from stk_msgs.msg import BridgeStatus

    seen: List[BridgeStatus] = []

    rclpy.init(args=None)
    try:
        node = rclpy.create_node("deploy_run_wait_for_bridge")
        qos = status_qos(depth=5)
        node.create_subscription(
            BridgeStatus, f"{prefix}/status", seen.append, qos)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
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
                f"timed out; last state={seen[-1].state}")
        return ConnectResult(False, "", "no BridgeStatus received at all")
    finally:
        node.destroy_node()
        rclpy.shutdown()


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
    import rclpy
    from stk_bridge.interfaces.qos import command_qos, clock_qos
    from stk_msgs.msg import ScenarioTime
    from std_msgs.msg import String

    seen: List[str] = []
    ticks = []

    rclpy.init(args=None)
    try:
        node = rclpy.create_node("deploy_run_wait_for_twin")
        node.create_subscription(ScenarioTime, f"{prefix}/scenario_time",
                                 ticks.append, clock_qos())
        node.create_subscription(
            String, f"{prefix}/lockstep/ack",
            lambda message: seen.append(str(message.data)), command_qos())

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if ticks and f"matlab_twin:{ticks[-1].step_index}" in seen:
                return True
        return False
    finally:
        node.destroy_node()
        rclpy.shutdown()


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
            process.wait(timeout=15)


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
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "steps": [],
    }

    def record(step: str, ok: bool, detail: str = "") -> None:
        summary["steps"].append({"step": step, "ok": ok, "detail": detail})
        (run_log.info if ok else run_log.error)("%s: %s", step, detail or "ok")

    recorder = None
    bridge_proc = None
    scheduler_proc = None
    scheduler_returncode = EXIT_STEP_FAILED
    try:
        # Before anything is launched: the twin cannot start without it.
        try:
            run_config, created = prepare_run_config(config, version)
        except RunConfigError as error:
            record("run_config", False, str(error))
            return EXIT_STEP_FAILED
        summary["run_config"] = run_config
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
        deps.check_idle(config.prefix)
        recorder = deps.start_recorder(config, log_dir)
        run_log.info("topic capture started: %s", log_dir)
        with _open_log(log_dir, "bridge.log") as bridge_log:
            argv = bridge_launch_argv(config, bridge_config)
            run_log.info("launching bridge: %s", " ".join(argv))
            bridge_proc = deps.popen(
                argv, stdout=bridge_log, stderr=subprocess.STDOUT)

            connect_result = deps.wait_for_bridge_connected(
                config.prefix, config.bridge_connect_timeout_s,
                config.ros_domain_id)
            record("bridge_connected", connect_result.connected,
                   connect_result.detail or connect_result.backend_type)
            if not connect_result.connected:
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

            twin_ok = deps.wait_for_twin_ack(
                config.prefix, config.twin_ack_timeout_s)
            record("twin_acking", twin_ok,
                   "" if twin_ok else "no matlab_twin ack seen before "
                                      "timeout -- check the Simulink "
                                      "model's Subscribe/Publish topics")
            if not twin_ok:
                return EXIT_STEP_FAILED

            with _open_log(log_dir, "scheduler.log") as scheduler_log:
                argv = scheduler_argv(config)
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

            # The scheduler no longer participates in the barrier. Stop the
            # bridge before waiting for a human, otherwise a successful run
            # can acquire a spurious missing-ack timeout during teardown.
            _stop_process(bridge_proc, "bridge", run_log)
            deps.prompt(
                "\n"
                "=== Scheduler finished. Stop the Simulink twin now "
                "(the model has no automatic stop) and press Enter to "
                "continue: ")

        return EXIT_OK if scheduler_returncode == 0 else EXIT_STEP_FAILED
    finally:
        for name, process in (("scheduler", scheduler_proc), ("bridge", bridge_proc)):
            _stop_process(process, name, run_log)
        if recorder is not None:
            recorder.close()
            record("topic_capture", not recorder.error, recorder.error or "files saved")
        summary["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        summary["output_exists"] = os.path.exists(config.output)
        with open(os.path.join(log_dir, "summary.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2)
        run_log.removeHandler(handler)
        handler.close()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    # Mirrors CommandLauncher's placeholders one for one.
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--experiment", default="")
    parser.add_argument("--scheduler", default="dynamics_aware")
    parser.add_argument("--execution-mode", default="lockstep")
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
                        default=90.0)
    parser.add_argument("--twin-ack-timeout-s", type=float, default=60.0)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
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
    )
    return execute_run(config)


if __name__ == "__main__":
    sys.exit(main())
