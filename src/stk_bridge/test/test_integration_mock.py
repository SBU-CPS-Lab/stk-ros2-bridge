"""End-to-end test of the core pipeline against the mock backend.

Not a ROS test (no rclpy dependency) -- it drives core/ directly the way
nodes/bridge_node.py's step loop does, minus the ROS publish/subscribe
layer. This is the same scenario manually exercised during
bring-up, captured as an automated regression test.
"""

import math

import pytest

from stk_bridge.backends.mock_backend import MockBackend
from stk_bridge.core.actuation import ActuationDispatcher
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.core import conversions as conv
from stk_bridge.core.health import BridgeState, HealthConfig, HealthMonitor
from stk_bridge.core.rate_group import RateGroupScheduler
from stk_bridge.core.sensing import SensingConfig, SensingPipeline
from stk_bridge.core.time_sync import ExecutionMode, TimeSync, TimeSyncConfig
from stk_bridge.core.validation import ConventionValidator
from stk_bridge.interfaces.ros_subscribers import (
    AttitudeCommand,
    AngularRateCommand,
    CommandInbox,
    apply_commands,
)

SAT = "*/Satellite/SSS-1"


@pytest.fixture
def rig():
    backend = MockBackend()
    backend.connect()
    backend.new_scenario("IntegrationTest")

    frames = FrameRegistry()
    report = ConventionValidator(backend).validate()
    assert report.ok

    sensing = SensingPipeline(backend, frames,
                              SensingConfig(satellites=[SAT]), report)
    actuation = ActuationDispatcher(backend, frames)
    actuation.prepare_object(SAT)

    time_sync = TimeSync(
        TimeSyncConfig(mode=ExecutionMode.LOCKSTEP, step_s=0.1,
                       barrier_participants=["twin"], barrier_timeout_s=0.5,
                       on_barrier_timeout="continue"),
        advance_backend=backend.set_current_time,
    )
    time_sync.start(backend.get_scenario_epoch())

    scheduler = RateGroupScheduler.from_config(
        {"sensing": {"rate_groups": {"fast": {"rate_hz": 10.0}}}})
    health = HealthMonitor(HealthConfig(stale_threshold=3))
    inbox = CommandInbox()

    yield {
        "backend": backend, "frames": frames, "sensing": sensing,
        "actuation": actuation, "time_sync": time_sync,
        "scheduler": scheduler, "health": health, "inbox": inbox,
    }
    backend.disconnect()


class TestClosedLoop:
    def test_commanded_attitude_reads_back_through_full_pipeline(self, rig):
        """The definitive test: command via the inbox/dispatcher path,
        step time, sense via the pipeline, and confirm the value that
        comes back matches what was sent -- through STK's (mock) frame
        handling and quaternion convention, not just Python round-tripping."""
        ts, act, sensing, inbox = (
            rig["time_sync"], rig["actuation"], rig["sensing"], rig["inbox"])

        angle = math.radians(40.0)
        commanded = (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))

        for _ in range(5):
            now = ts.step()
            inbox.put("attitude", AttitudeCommand(SAT, now, commanded))
            commands = inbox.drain()
            apply_commands(act, commands)
            ts.await_barrier()

        sample = sensing.read_attitude(SAT, ts.now())
        assert sample is not None
        error_deg = math.degrees(
            conv.quaternion_angle_between(sample.quaternion_xyzw, commanded))
        assert error_deg < 1e-6

    def test_angular_rate_requires_attitude_first_end_to_end(self, rig):
        act, inbox = rig["actuation"], rig["inbox"]
        epoch = rig["backend"].get_scenario_epoch()

        # Publish rate before attitude, as a real ROS graph might if the
        # two commands arrive on separate topics with no ordering
        # guarantee.
        inbox.put("angular_rate",
                  AngularRateCommand(SAT, epoch + 1.0, (0, 0, 0.1)))
        inbox.put("attitude", AttitudeCommand(SAT, epoch + 1.0, (0, 0, 0, 1)))

        result = apply_commands(act, inbox.drain())
        assert result["applied"]["attitude"] == 1
        assert result["applied"]["angular_rate"] == 1
        assert result["rejected"]["angular_rate"] == 0

    def test_health_escalates_to_degraded_on_persistent_staleness(self, rig):
        sensing, health = rig["sensing"], rig["health"]
        # HealthMonitor only escalates CONNECTED -> DEGRADED (see
        # health.py's update_from_sensing); it starts at STARTING, which
        # is what bridge_node.start() clears with record_success() right
        # after the worker connects, before the loop ever runs.
        health.record_success()

        from stk_bridge.backends.base import PropagatorType
        rig["backend"].set_propagator(SAT, PropagatorType.REALTIME)
        sensing.invalidate_propagator_cache(SAT)
        epoch = rig["backend"].get_scenario_epoch()

        for _ in range(6):
            try:
                sensing.read_orbit(SAT, epoch)   # stale every time, no injection
            except Exception:
                pass
            health.update_from_sensing(sensing.statistics())

        assert health.state == BridgeState.DEGRADED

    def test_reset_clears_actuation_guard_and_inbox(self, rig):
        """Mirrors the real ResetScenario service order (see
        interfaces/ros_services.py._reset_scenario and
        actuation.py's reset() docstring): STK-side data is cleared
        BEFORE the local guard forgets its bookkeeping, not after.
        """
        act, inbox, backend = rig["actuation"], rig["inbox"], rig["backend"]
        epoch = backend.get_scenario_epoch()
        act.set_attitude(SAT, epoch + 10.0, (0, 0, 0, 1))
        inbox.put("attitude", AttitudeCommand(SAT, epoch + 20.0, (0, 0, 0, 1)))

        backend.clear_attitude_data(SAT)
        act.reset(SAT)
        inbox.clear()

        assert inbox.depth == 0
        # Guard forgot the last stamp AND STK forgot its samples, so an
        # earlier time is accepted by both.
        assert act.set_attitude(SAT, epoch + 1.0, (0, 0, 0, 1))
