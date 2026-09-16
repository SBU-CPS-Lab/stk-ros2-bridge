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
1. **stk_bridge** (``ros2 launch``) -- the twin and the scheduler are
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
   step this project has not verified and calling it done would be
   exactly the failure mode CLAUDE.md warns about: a self-consistent,
   untested guess standing in for a measurement. So this script stops
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
    twin_ack_timeout_s: float = 300.0
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

    popen: Callable[..., "subprocess.Popen"] = subprocess.Popen
    wait_for_bridge_connected: Callable[[str, float, str], "ConnectResult"] = None
    wait_for_twin_ack: Callable[[str, float], bool] = None
    prompt: Callable[[str], None] = lambda message: input(message)  # noqa: E731
    sleep: Callable[[float], None] = time.sleep

    def __post_init__(self) -> None:
        if self.wait_for_bridge_connected is None:
            self.wait_for_bridge_connected = _wait_for_bridge_connected_ros
        if self.wait_for_twin_ack is None:
            self.wait_for_twin_ack = _wait_for_twin_ack_ros


@dataclass
class ConnectResult:
    connected: bool
    backend_type: str = ""
    detail: str = ""


def bridge_launch_argv(config: RunConfig) -> List[str]:
    return [
        "ros2", "launch", "stk_bridge", "stk_bridge.launch.py",
        f"config_file:={config.bridge_config}",
    ]


def scheduler_argv(config: RunConfig) -> List[str]:
    argv = [
        "ros2", "run", "acav", "acav_scheduler", "--ros-args",
        "-p", f"scenario_config:={config.scenario}",
        "-p", f"scheduler_type:={config.scheduler}",
        "-p", f"prefix:={config.prefix}",
        "-p", f"output_json:={config.output}",
        "-p", f"run_id:={config.run_id}",
        "-p", f"experiment:={config.experiment}",
        "-p", f"gains_json:={config.gains_json}",
        "-p", f"fault_json:={config.fault_json}",
        "-p", f"attitude_provider:={config.attitude_provider}",
        "-p", "auto_shutdown:=true",
    ]
    if config.seed:
        argv += ["-p", f"seed:={config.seed}"]
    return argv


def _wait_for_bridge_connected_ros(prefix: str, timeout_s: float,
                                   ros_domain_id: str) -> ConnectResult:
    """Poll ``{prefix}/status`` for ``state == CONNECTED``.

    Imported lazily -- this module must stay importable (for the offline
    unit tests) on a machine with no ROS 2 sourced.
    """
    import rclpy
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from stk_msgs.msg import BridgeStatus

    seen: List[BridgeStatus] = []

    rclpy.init(args=None)
    try:
        node = rclpy.create_node("deploy_run_wait_for_bridge")
        qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        node.create_subscription(
            BridgeStatus, f"{prefix}/status", seen.append, qos)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
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
    """
    import rclpy
    from std_msgs.msg import String

    seen: List[str] = []

    rclpy.init(args=None)
    try:
        node = rclpy.create_node("deploy_run_wait_for_twin")
        node.create_subscription(
            String, f"{prefix}/lockstep/ack",
            lambda message: seen.append(str(message.data)), 10)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if any(text.startswith("matlab_twin:") for text in seen):
                return True
        return False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _open_log(log_dir: str, name: str):
    os.makedirs(log_dir, exist_ok=True)
    return open(os.path.join(log_dir, name), "w", encoding="utf-8")


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

    summary = {
        "run_id": config.run_id, "experiment": config.experiment,
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "steps": [],
    }

    def record(step: str, ok: bool, detail: str = "") -> None:
        summary["steps"].append({"step": step, "ok": ok, "detail": detail})
        (run_log.info if ok else run_log.error)("%s: %s", step, detail or "ok")

    bridge_proc = None
    scheduler_returncode = EXIT_STEP_FAILED
    try:
        with _open_log(log_dir, "bridge.log") as bridge_log:
            run_log.info("launching bridge: %s",
                         " ".join(bridge_launch_argv(config)))
            bridge_proc = deps.popen(
                bridge_launch_argv(config),
                stdout=bridge_log, stderr=subprocess.STDOUT)

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
                f"build_acav_twin('RunConfigPath', '{config.run_config}')\"\n"
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
                scheduler_returncode = scheduler_proc.wait()
                record("scheduler_exit", scheduler_returncode == 0,
                       f"returncode={scheduler_returncode}")

            deps.prompt(
                "\n"
                "=== Scheduler finished. Stop the Simulink twin now "
                "(the model has no automatic stop) and press Enter to "
                "continue: ")

        return EXIT_OK if scheduler_returncode == 0 else EXIT_STEP_FAILED
    finally:
        if bridge_proc is not None and bridge_proc.poll() is None:
            run_log.info("stopping bridge")
            bridge_proc.terminate()
            try:
                bridge_proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                bridge_proc.kill()
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
    parser.add_argument("--twin-ack-timeout-s", type=float, default=300.0)
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
