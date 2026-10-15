"""Offline free-run timing and coupling analysis from a run's logs only.

acav.coupling_analysis reads a run's record and its <run>.logs folder
(the topic captures cmd.txt / attitude.txt / twin_status.txt /
status.txt, bridge.log and summary.json) -- no STK, no ROS -- and
computes the quantities the E3 pilot analysis (P1) used: effective
control rate, command-gap distribution, sample age at each image epoch,
n > 1 sub-step count and max n, command-stamp offset, write lag, COM
round-trip stalls and the STK read-back deviation beyond the newest
written sample, plus the run conditions. A lockstep run gives 10 Hz,
age 0 and deviation 0.
"""

import csv
import json
import math

import pytest

from acav import coupling_analysis as ca
from acav.metrics import CaptureOutcome, RunResult, TargetResult
from acav.run_record import Provenance, RunRecord, write_record

EPOCH = 1717237800.0
Q0 = (0.0, 0.0, 0.0, 1.0)
# 10 degrees about x.
Q10 = (math.sin(math.radians(5)), 0.0, 0.0, math.cos(math.radians(5)))


def _stamp(t):
    """(sec, nanosec) as the bridge stamps them, without a float sum at
    ~1.7e9 s that would itself lose ~1e-7 s."""
    whole = math.floor(t)
    nsec = int(round((t - whole) * 1e9))
    if nsec >= 1000000000:
        whole, nsec = whole + 1, nsec - 1000000000
    return int(EPOCH) + whole, nsec


def _attitude_doc(t, q):
    sec, nsec = _stamp(t)
    return ("header:\n  stamp:\n    sec: %d\n    nanosec: %d\n  frame_id: ''\n"
            "object_path: '*/Satellite/SSS-1'\norientation:\n  x: %r\n"
            "  y: %r\n  z: %r\n  w: %r\nuse_inertial_frame: true\n"
            % (sec, nsec, *q))


def _status_doc(t, round_trip_ms):
    sec, nsec = _stamp(t)
    return ("header:\n  stamp:\n    sec: %d\n    nanosec: %d\n"
            "  frame_id: stk_bridge\nstate: 1\nbackend_type: com\n"
            "last_backend_round_trip_ms: %r\nmean_sense_ms: 0.0\n"
            % (sec, nsec, round_trip_ms))


def _twin_doc(t, error_deg):
    sec, nsec = _stamp(t)
    return ("header:\n  stamp:\n    sec: %d\n    nanosec: %d\n  frame_id: ''\n"
            "mode: IMAGE\nboresight_error_rad: %r\n"
            % (sec, nsec, math.radians(error_deg)))


def _write(path, docs):
    path.write_text("".join(d + "\n---\n" for d in docs), encoding="utf-8")


def _bridge_log(mode, rtf, pacing=None):
    text = ("2026-10-15 15:24:36,830 INFO    stk_bridge.core.time_sync: time "
            "sync started: mode=%s epoch=%.3f step=0.1000s rtf=%.3f "
            "participants=['acav_scheduler', 'matlab_twin']\n"
            % (mode, EPOCH, rtf))
    if pacing:
        text += ("2026-10-15 15:37:27,960 INFO    __main__: free-run pacing "
                 "summary: cycles %d, overruns %d, period 0.100 s\n" % pacing)
    return text


def _run(folder, name, mode, cmd, att, twin, status, images, summary=None,
         pacing=None, rtf=1.0):
    """cmd/att: [(t, q)]; twin: [(t, err_deg)]; status: [(t, ms)]."""
    targets = [TargetResult(target_id=f"T{i + 1}", attempted=True,
                            outcome=CaptureOutcome.CAPTURED,
                            image_time_s=t, error_at_image_deg=0.1,
                            max_error_in_hold_deg=0.1)
               for i, t in enumerate(images)]
    write_record(str(folder / f"{name}.json"), RunRecord(
        result=RunResult(run_id=name, scheduler="dynamics_aware",
                         targets=targets),
        provenance=Provenance(backend_type="com", execution_mode=mode,
                              graphics_mode="off", real_time_factor=rtf)))
    logs = folder / f"{name}.logs"
    logs.mkdir()
    _write(logs / "cmd.txt", [_attitude_doc(t, q) for t, q in cmd])
    _write(logs / "attitude.txt", [_attitude_doc(t, q) for t, q in att])
    _write(logs / "twin_status.txt", [_twin_doc(t, e) for t, e in twin])
    _write(logs / "status.txt", [_status_doc(t, ms) for t, ms in status])
    (logs / "bridge.log").write_text(
        _bridge_log("LOCKSTEP" if mode == "lockstep" else "FREE_RUN", rtf,
                    pacing), encoding="utf-8")
    (logs / "summary.json").write_text(json.dumps(
        {"run_id": name, **(summary or {})}), encoding="utf-8")
    return folder / f"{name}.json"


@pytest.fixture
def lockstep(tmp_path):
    ts = [round(0.1 * k, 10) for k in range(1, 11)]          # 0.1 .. 1.0
    return _run(tmp_path, "E3-lock", "lockstep",
                cmd=[(t, Q0) for t in ts], att=[(t, Q0) for t in ts],
                twin=[(t - 0.1, 0.01) for t in ts],
                status=[(0.0, 500.0)] + [(t, 30.0) for t in ts],
                images=[0.5])


@pytest.fixture
def free_run(tmp_path):
    # Samples at 0, 0.25, 0.5, 1.8 (a 1.3 s stall); commands stamped +0.1.
    samples = [0.0, 0.25, 0.5, 1.8]
    cmd = [(t + 0.1, Q0) for t in samples]
    # The bridge senses at the sample times (plus one at 1.95): every
    # read-back after the first is beyond the newest written sample,
    # where STK reports an attitude 10 degrees away.
    att = [(0.0, Q0), (0.25, Q10), (0.5, Q10), (1.8, Q10), (1.95, Q10)]
    return _run(tmp_path, "E3-free", "free_run", cmd=cmd, att=att,
                twin=[(t, 0.6) for t in samples],
                status=[(0.0, 800.0), (0.3, 120.0), (1.0, 1300.0),
                        (1.9, 90.0)],
                images=[0.6, 1.9], rtf=0.5, pacing=(40, 3),
                summary={"free_run_start_gate": {"opened": True,
                                                 "wait_s": 12.5},
                         "twin_substeps": {"events": 2, "max_n": 13}})


class TestParsing:
    def test_capture_format(self, tmp_path):
        path = tmp_path / "cmd.txt"
        _write(path, [_attitude_doc(0.1, Q10), _attitude_doc(0.35, Q0)])
        rows = ca.read_attitudes(str(path), EPOCH)
        assert [round(t, 6) for t, _ in rows] == [0.1, 0.35]
        assert rows[0][1] == pytest.approx(Q10)

    def test_bridge_log_facts(self):
        facts = ca.bridge_log_facts(_bridge_log("FREE_RUN", 0.5, (40, 3)))
        assert facts == {"mode": "FREE_RUN", "epoch": EPOCH, "step_s": 0.1,
                         "real_time_factor": 0.5,
                         "pacing_cycles": 40, "pacing_overruns": 3}


class TestLockstep:
    def test_reference_values(self, lockstep):
        m = ca.analyze_run(str(lockstep), window_s=1.0)
        assert m["execution_mode"] == "lockstep"
        assert m["commands"] == 10
        assert m["control_rate_hz"] == pytest.approx(10.0)
        assert m["gap_p50_s"] == pytest.approx(0.1)
        assert m["gaps_over_0_6"] == 0
        assert m["n_gt1_count"] == 0 and m["max_n"] == 1
        assert m["n_source"] == "commands"
        assert m["image_ages"] == [{"target": "T1", "image_time_s": 0.5,
                                    "age_s": pytest.approx(0.0, abs=1e-9),
                                    "error_deg": pytest.approx(0.01)}]
        assert m["offset_affected"] == 0
        assert m["readback_at_written"] == 10
        assert m["readback_after_newest"] == 0
        assert m["com_stalls_over_300ms"] == 0           # t = 0 excluded


class TestFreeRun:
    def test_metrics(self, free_run):
        m = ca.analyze_run(str(free_run), window_s=2.0)
        assert m["commands"] == 4
        assert m["control_rate_hz"] == pytest.approx(2.0)
        assert m["gap_max_s"] == pytest.approx(1.3)
        assert m["gaps_over_0_6"] == 1
        assert m["gap_time_over_0_6_s"] == pytest.approx(1.3)
        # Twin-reported sub-steps win over the command-derived estimate.
        assert m["n_source"] == "twin_substeps"
        assert m["n_gt1_count"] == 2 and m["max_n"] == 13
        ages = {a["target"]: a["age_s"] for a in m["image_ages"]}
        assert ages["T1"] == pytest.approx(0.1)      # newest sample 0.5
        assert ages["T2"] == pytest.approx(0.1)      # newest sample 1.8
        # Only sample 0.25 is off the step grid: stamp 0.35, state 0.3.
        assert m["offset_affected"] == 1
        assert m["offset_max_abs_s"] == pytest.approx(0.05)
        assert m["com_stalls_over_300ms"] == 1
        assert m["com_round_trip_max_ms"] == pytest.approx(1300.0)
        assert m["readback_at_written"] == 0
        assert m["readback_after_newest"] == 4
        assert m["readback_after_median_deg"] == pytest.approx(10.0)
        assert m["real_time_factor"] == 0.5
        assert m["graphics_mode"] == "off"
        assert m["gate_wait_s"] == 12.5
        assert m["pacing_cycles"] == 40 and m["pacing_overruns"] == 3

    def test_write_lag(self, free_run):
        m = ca.analyze_run(str(free_run), window_s=2.0)
        # Command for sample t (stamp t+0.1) is written before the next
        # bridge sample after t: lags 0.15, 0.15, 1.2 and 0.05 s.
        assert m["write_lag_behind"] == 4
        assert m["write_lag_max_s"] == pytest.approx(1.2)


class TestFolder:
    def test_one_json_per_run_and_a_csv(self, lockstep, free_run,
                                        tmp_path):
        (tmp_path / "campaign.json").write_text("{}", encoding="utf-8")
        (tmp_path / "E3-free.runconfig.json").write_text("{}",
                                                         encoding="utf-8")
        out = tmp_path / "_coupling"
        rows = ca.analyze_folder(str(tmp_path), str(out), window_s=2.0)
        assert sorted(r["run_id"] for r in rows) == ["E3-free", "E3-lock"]
        assert (out / "E3-free.coupling.json").exists()
        with open(out / "coupling.csv", encoding="utf-8") as handle:
            table = list(csv.DictReader(handle))
        assert [r["run_id"] for r in table] == ["E3-free", "E3-lock"]
        assert "image_ages" not in table[0]          # scalars only
        assert table[0]["control_rate_hz"] == "2.0"


class TestRegressions:
    def test_stamps_keep_sub_microsecond_precision(self, tmp_path):
        """sec + nsec*1e-9 - epoch at ~1.7e9 loses ~2.4e-7 s, enough to
        push an on-grid lockstep stamp across a step boundary."""
        ts = [round(0.1 * k, 10) for k in range(1, 7201)]
        path = tmp_path / "cmd.txt"
        _write(path, [_attitude_doc(t, Q0) for t in ts])
        rows = ca.read_attitudes(str(path), EPOCH)
        assert max(abs(t - r) for (t, _), r in zip(rows, ts)) < 1e-9

    def test_a_full_lockstep_window_has_no_offset(self, tmp_path):
        ts = [round(0.1 * k, 10) for k in range(1, 7201)]
        run = _run(tmp_path, "E3-long", "lockstep",
                   cmd=[(t, Q0) for t in ts], att=[(t, Q0) for t in ts],
                   twin=[(t - 0.1, 0.01) for t in ts],
                   status=[(0.1, 531.0)] * 20 + [(t, 30.0) for t in ts[1:]],
                   images=[60.0])
        m = ca.analyze_run(str(run), window_s=720.0)
        assert m["offset_affected"] == 0
        assert m["control_rate_hz"] == pytest.approx(10.0)
        # The startup wait republishes the first step (t = 0.1) with its
        # long first round trip: not a stall of the running loop.
        assert m["com_stalls_over_300ms"] == 0
