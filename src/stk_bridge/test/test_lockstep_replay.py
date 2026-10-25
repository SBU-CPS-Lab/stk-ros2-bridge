"""The real per-step lockstep sequence, replayed against a COM-shaped fake.

What a lockstep step does to STK, in order (``nodes/bridge_node.py``
``_lockstep_iteration``): apply the attitude command written for this
epoch (``ActuationDispatcher.set_attitude`` -> ``AddAttitude ... Quat``),
advance the clock (``_advance_stk_clock`` -> ``Animate * SetValues``),
then sense (``SensingPipeline.read_all``). The scheduler's services add
``compute_access`` and ``query_data_provider`` calls during the run.

Everything here is the production code -- ComBackend's own write,
clock and access methods, the real dispatcher and the bridge node's own
methods -- except the COM root, which records commands, and the data
providers, which come from the COM-shaped fake. The point: none of these
per-step calls may change ``ComBackend.revision``, or (B) would re-verify
and (C) would drop its cache every step.
"""

from types import SimpleNamespace

from stk_bridge.backends.base import DataProviderQuery, StkTime
from stk_bridge.backends.com_backend import ComBackend
from stk_bridge.core.actuation import ActuationDispatcher
from stk_bridge.core.frames import FrameRegistry
from stk_bridge.core.orbit_batch import OrbitBatch
from stk_bridge.core.sensing import SensingConfig, SensingPipeline
from stk_bridge.nodes.bridge_node import StkBridgeNode

from test_equivalence_diagnostic import (   # noqa: I100 - shared fake
    EPOCH,
    SAT,
    STOP,
    FakeBackend,
    _attitude,
    _epochs,
    _report,
)

SENSOR = "*/Satellite/SSS-1/Sensor/Imager"
TARGET = "*/Target/T1"


class RecordingRoot:
    """Stands in for the STK root: records every Connect command."""

    def __init__(self):
        self.commands = []

    def ExecuteCommand(self, command):
        self.commands.append(command)
        return SimpleNamespace(Count=0, Item=lambda i: "")

    def GetObjectFromPath(self, path):
        access = SimpleNamespace(
            ComputeAccess=lambda: None,
            ComputedAccessIntervalTimes=SimpleNamespace(Count=0))
        return SimpleNamespace(
            GetAccessToObject=lambda target: access,
            PropagatorType=SimpleNamespace(name="ePropagatorHPOP"))


class ReplayBackend(ComBackend):
    """Real ComBackend writes, clock and access; fake data providers."""

    def __init__(self, providers):
        super().__init__()
        self._providers = providers
        self._connected = True
        self._root = RecordingRoot()

    def query_data_provider(self, query):
        return self._providers.query_data_provider(query)

    def list_objects(self):
        return self._providers.list_objects()


def _replay(steps, access_at=150, batch=False):
    providers = FakeBackend()
    backend = ReplayBackend(providers)
    dispatcher = ActuationDispatcher(backend, FrameRegistry(),
                                     default_attitude_inertial=True)
    node = SimpleNamespace(
        actuation=dispatcher,
        worker=SimpleNamespace(submit=lambda fn, description: fn(backend)))
    pipeline = SensingPipeline(
        backend, FrameRegistry(),
        SensingConfig(satellites=[SAT], combine_attitude_rate=True),
        _report())
    pipeline.set_attitude_write_probe(
        lambda path, when: StkBridgeNode._inertial_sample_written(
            node, path, when))
    if batch:
        pipeline.attach_orbit_batch(OrbitBatch(
            samples=100, step_s=0.1, stop_time=StkTime(STOP)))

    revisions = []
    per_step = []
    for index, when in enumerate(_epochs(steps)):
        before = len(providers.queries)
        if index > 0:
            # The twin's command for this epoch, applied before the step.
            assert dispatcher.set_attitude(
                SAT, when, _attitude(when.epoch_seconds), inertial=True)
        StkBridgeNode._advance_stk_clock(node, when)
        if index == access_at:
            # The scheduler's two services, as ros_services calls them.
            backend.compute_access(SENSOR, TARGET, StkTime(EPOCH + 1.0),
                                   StkTime(EPOCH + 3.0))
            backend.query_data_provider(DataProviderQuery(
                object_path=TARGET, provider_name="Cartesian Position",
                group_element="ICRF", start=StkTime(EPOCH),
                stop=StkTime(EPOCH + 720.0), step_s=10.0))
        pipeline.read_all(SAT, when)
        revisions.append(backend.revision)
        per_step.append([(q.provider_name, q.is_instantaneous)
                         for q in providers.queries[before:]
                         if q.object_path == SAT])
    return backend, pipeline, revisions, per_step


def test_the_per_step_calls_never_change_the_revision():
    backend, pipeline, revisions, _ = _replay(300)
    # SetAttitude Standard + ClearData on the first command, then nothing:
    # AddAttitude, Animate SetValues, compute_access and the target query
    # are not structural.
    assert revisions[0] == 0
    assert set(revisions[1:]) == {2}
    commands = backend._root.commands
    assert sum(c.endswith("Standard") for c in commands) == 1
    assert sum(c.endswith("ClearData") for c in commands) == 1
    assert sum(c.startswith("AddAttitude") for c in commands) == 299
    assert sum(c.startswith("Animate * SetValues") for c in commands) == 300


def test_activation_at_the_first_written_epoch_then_three_queries():
    _, pipeline, _, per_step = _replay(300)
    separate = ["Cartesian Position", "Cartesian Velocity",
                "Attitude Quaternions", "Attitude Segment Quaternions"]
    combined = ["Cartesian Position", "Cartesian Velocity",
                "Attitude Segment Quaternions"]
    names = [[name for name, _ in step] for step in per_step]
    assert names[0] == separate               # t = 0.1 s: nothing written
    assert names[1] == separate               # first written epoch: verify
    assert names[2:] == [combined] * 298      # steady state
    assert pipeline.sensing_mode == "combined_attitude_rate"


def test_no_re_verification_from_clock_or_services():
    _, pipeline, _, _ = _replay(300)
    stats = pipeline.combine_statistics()
    assert stats == {
        "configured": True, "state": "active",
        "steps_combined": 298, "steps_separate": 2,
        "verifications": 1, "verifications_passed": 1, "mismatches": 0,
        "revision_resets": 0, "re_verifications": 0,
        "segment_no_data_fallbacks": 0}


def test_the_summary_line_names_every_counter():
    _, pipeline, _, _ = _replay(10)
    line = pipeline.combine_summary_line()
    assert line.startswith("combined attitude/rate summary:")
    for text in ("steps combined 8", "separate 2", "verifications 1",
                 "passed 1", "mismatches 0", "re-verifications 0",
                 "revision resets 0",
                 "segment NO-DATA fallbacks 0", "state active"):
        assert text in line, text


def test_bridge_stop_logs_the_summary_once(caplog):
    import logging
    import threading

    _, pipeline, _, _ = _replay(5)
    node = StkBridgeNode.__new__(StkBridgeNode)
    node._running = threading.Event()
    node._step_request = threading.Event()
    node._loop_thread = None
    node.worker = SimpleNamespace(stop=lambda timeout_s: None)
    node.sensing = pipeline
    with caplog.at_level(logging.INFO):
        node.stop()
    lines = [r.getMessage() for r in caplog.records
             if r.getMessage().startswith("combined attitude/rate summary")]
    assert lines == [pipeline.combine_summary_line()]
    orbit = [r.getMessage() for r in caplog.records
             if r.getMessage().startswith("orbit batch summary")]
    assert orbit == [pipeline.orbit_batch_summary_line()]


def _orbit_counts(steps):
    """(interval, instantaneous) orbit queries per step."""
    orbit = ("Cartesian Position", "Cartesian Velocity")
    return [(sum(1 for n, inst in step if n in orbit and not inst),
             sum(1 for n, inst in step if n in orbit and inst))
            for step in steps]


def test_batched_replay_has_no_per_step_invalidation():
    backend, pipeline, revisions, per_step = _replay(300, batch=True)
    assert set(revisions[1:]) == {2}
    counts = _orbit_counts(per_step)
    # t = 0.1 s: a block is loaded (2 interval + 4 audit reads). The first
    # command's SetAttitude Standard + ClearData is structural, so at
    # t = 0.2 s that block is dropped and reloaded; after that only the
    # block starts at t = 10.2 s and 20.2 s read the orbit.
    loads = [i for i, c in enumerate(counts) if c != (0, 0)]
    assert loads == [0, 1, 101, 201]
    assert all(counts[i] == (2, 4) for i in loads)
    stats = pipeline.orbit_batch_statistics()
    assert stats["blocks_loaded"] == 4
    assert stats["invalidations_revision"] == 1
    assert stats["audits_passed"] == 4 and stats["audits_failed"] == 0
    assert stats["epochs_cached"] == 300
    # The combined attitude/rate read is unaffected by batching.
    assert pipeline.combine_statistics()["verifications"] == 1


def test_steady_state_is_six_orbit_queries_per_hundred_steps():
    _, _, _, per_step = _replay(300, batch=True)
    counts = _orbit_counts(per_step[101:201])
    assert sum(i for i, _ in counts) == 2
    assert sum(n for _, n in counts) == 4
