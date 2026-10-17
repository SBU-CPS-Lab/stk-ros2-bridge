"""E1 with six interface-mismatch types.

frame_pairing is the existing E1 treatment (an Earth-fixed read-back
forced against the inertial command, sensing.force_attitude_provider);
it runs guard-on and guard-ablated. The five new types are injected at
the bridge's publish side (sensing.inject_mismatch,
stk_bridge/core/mismatch_injection.py) and run guard-on only; no twin,
+adcs, stk_msgs or message change.

build_e1_matrix(types, repeats) = control + frame_pairing (guard on,
guard ablated) + one guard-on run per new type, per repeat:
    runs = repeats x (1 + 2 [if frame_pairing] + new types)

Following the provider precedent: the type is in the RunSpec and its run
id, the run config (e1_mismatch), the bridge YAML copy, summary.json and
the record's provenance; it is not in the determinism signature, which
is computed from the result only.

A guard-on attitude mismatch, or the time offset, is expected to stop
the run (no record): report counts such a run as a fail-stop from its
summary.json, by arm, never as a silently missing run.
"""

import json
import math
import os

import pytest

from acav.campaign import MISPAIRED_ATTITUDE_PROVIDER, RunSpec
from acav.run_record import Provenance, RunRecord, read_record, write_record

from test_campaign import _result, covering  # noqa: I100
from test_deploy_run_cli import _config  # noqa: I100

PENDING = pytest.mark.xfail(strict=True, reason="E1 mismatch matrix not "
                                                "written yet")

ALL = ("frame_pairing", "quaternion_order", "quaternion_conjugate",
       "angle_rate_units", "distance_units", "time_offset")


# --- the matrix --------------------------------------------------------------

@PENDING
def test_the_types_are_the_six():
    from acav.campaign import E1_MISMATCH_TYPES
    assert E1_MISMATCH_TYPES == ALL


@PENDING
@pytest.mark.parametrize("types, repeats, expected", [
    (ALL, 1, 1 * (1 + 2 + 5)),
    (ALL, 3, 3 * (1 + 2 + 5)),
    (("frame_pairing",), 1, 3),
    (("distance_units", "time_offset"), 2, 2 * (1 + 2)),
])
def test_the_run_count_formula(types, repeats, expected):
    from acav.campaign import build_e1_matrix, e1_run_count
    campaign = build_e1_matrix(types, repeats)
    assert len(campaign) == expected == e1_run_count(types, repeats)
    assert len({r.run_id for r in campaign.runs}) == expected


@PENDING
def test_the_arms():
    from acav.campaign import build_e1_matrix
    runs = build_e1_matrix(ALL, 1).runs
    arms = [(r.mismatch or "control", r.mismatch_guard) for r in runs]
    assert arms == [("control", ""), ("frame_pairing", "on"),
                    ("frame_pairing", "ablated"),
                    ("quaternion_order", "on"),
                    ("quaternion_conjugate", "on"),
                    ("angle_rate_units", "on"), ("distance_units", "on"),
                    ("time_offset", "on")]
    frame = [r for r in runs if r.mismatch == "frame_pairing"]
    assert all(r.attitude_provider == MISPAIRED_ATTITUDE_PROVIDER
               for r in frame)
    assert all(r.attitude_provider == "" for r in runs
               if r.mismatch not in ("frame_pairing",))
    assert len({(r.scheduler, r.execution_mode) for r in runs}) == 1


@PENDING
def test_one_repeat_keeps_the_original_run_ids():
    """Resuming an old E1 folder must skip its two existing runs."""
    from acav.campaign import build_e1_convention, build_e1_matrix
    old = {r.run_id for r in build_e1_convention().runs}
    new = {r.run_id for r in build_e1_matrix(ALL, 1).runs}
    assert old <= new


@PENDING
def test_repeats_get_seeds_and_distinct_ids():
    from acav.campaign import build_e1_matrix
    runs = build_e1_matrix(("time_offset",), 2).runs
    assert sorted(r.seed for r in runs) == [0, 0, 1, 1]


@PENDING
def test_an_unknown_type_is_refused():
    from acav.campaign import build_e1_matrix
    with pytest.raises(ValueError, match="unknown E1 mismatch"):
        build_e1_matrix(("degrees_everywhere",), 1)


# --- CLI -----------------------------------------------------------------------

@PENDING
def test_plan_prints_the_formula(tmp_path, capsys):
    from acav.campaign_cli import main
    code = main(["plan", "--experiment", "E1", "--repeats", "3",
                 "--output-dir", str(tmp_path)])
    out = capsys.readouterr().out
    assert code == 0
    assert ("E1 runs = N_REPEATS x (1 control + 2 frame_pairing + 5 other "
            "types) = 3 x 8 = 24") in out
    assert len(json.load(open(tmp_path / "campaign.json"))["runs"]) == 24


@PENDING
def test_plan_with_chosen_types(tmp_path, capsys):
    from acav.campaign_cli import main
    code = main(["plan", "--experiment", "E1", "--e1-types",
                 "distance_units,time_offset", "--output-dir",
                 str(tmp_path)])
    out = capsys.readouterr().out
    assert code == 0
    assert ("E1 runs = N_REPEATS x (1 control + 0 frame_pairing + 2 other "
            "types) = 1 x 3 = 3") in out


@PENDING
def test_the_run_cli_flags():
    from acav.deploy_run_cli import _parser
    base = ["--run-id", "r", "--output", "o.json", "--bridge-config", "b"]
    args = _parser().parse_args(base)
    assert (args.mismatch, args.mismatch_guard) == ("", "")
    args = _parser().parse_args(base + ["--mismatch", "time_offset",
                                        "--mismatch-guard", "on"])
    assert (args.mismatch, args.mismatch_guard) == ("time_offset", "on")


@PENDING
@pytest.mark.parametrize("mismatch, guard, provider, injected, policy", [
    ("", "", "", "", "strict"),
    ("frame_pairing", "ablated", MISPAIRED_ATTITUDE_PROVIDER, "",
     "e1_readback_ablation"),
    ("frame_pairing", "on", MISPAIRED_ATTITUDE_PROVIDER, "", "strict"),
    ("quaternion_order", "on", "", "quaternion_order", "strict"),
    ("time_offset", "on", "", "time_offset", "strict"),
])
def test_bridge_config_and_run_config(tmp_path, mismatch, guard, provider,
                                      injected, policy):
    import logging

    import yaml

    from acav.deploy_run_cli import standalone_run_config, write_bridge_config
    config = _config(tmp_path, experiment="E1", mismatch=mismatch,
                     mismatch_guard=guard, attitude_provider=provider)
    (tmp_path / "logs").mkdir()
    bridge = yaml.safe_load(open(write_bridge_config(
        config, str(tmp_path / "logs"), logging.getLogger("t")),
        encoding="utf-8"))
    assert bridge["sensing"]["inject_mismatch"] == injected
    payload = standalone_run_config(config, "abc")
    assert payload["e1_mismatch"] == mismatch
    assert payload["attitude_mismatch_policy"] == policy


@PENDING
def test_the_campaign_run_config_follows_the_same_rule(tmp_path):
    from acav.campaign_runner import run_config_payload_for
    run = RunSpec(experiment="E1", scheduler="dynamics_aware",
                  execution_mode="lockstep",
                  attitude_provider=MISPAIRED_ATTITUDE_PROVIDER,
                  mismatch="frame_pairing", mismatch_guard="on")
    payload = run_config_payload_for(run)
    assert payload["attitude_mismatch_policy"] == "strict"
    assert payload["e1_mismatch"] == "frame_pairing"


@PENDING
def test_the_launcher_has_the_placeholders():
    from acav.campaign_runner import CommandLauncher
    import inspect
    source = inspect.getsource(CommandLauncher.__call__)
    assert '"mismatch": run.mismatch' in source
    assert '"mismatch_guard": run.mismatch_guard' in source


# --- provenance and old records ------------------------------------------------

@PENDING
def test_provenance_round_trips_and_old_records_load(tmp_path):
    record = RunRecord(result=_result("r"), provenance=Provenance(
        mismatch="distance_units", mismatch_guard="on"))
    write_record(str(tmp_path / "r.json"), record)
    back = read_record(str(tmp_path / "r.json")).provenance
    assert (back.mismatch, back.mismatch_guard) == ("distance_units", "on")
    old = record.to_dict()
    old["provenance"].pop("mismatch")
    old["provenance"].pop("mismatch_guard")
    assert Provenance.from_dict(old["provenance"]).mismatch == ""


def test_an_old_style_e1_record_still_loads(tmp_path):
    from test_e4_removed import _old_style
    path = _old_style(tmp_path, "E1", "E1-dynamics_aware-lockstep")
    assert read_record(str(path)).result.run_id == \
        "E1-dynamics_aware-lockstep"


# --- report: arms, fail-stops, the expected-null control ---------------------

def _write(tmp_path, run, captured=True):
    record = covering(RunRecord(
        result=_result(run.run_id, scheduler=run.scheduler,
                       captured=captured),
        provenance=Provenance(backend_type="com", execution_mode="lockstep",
                              attitude_provider=run.attitude_provider,
                              attitude_readback_provider=(
                                  run.attitude_provider or
                                  "Attitude Quaternions"),
                              mismatch=run.mismatch,
                              mismatch_guard=run.mismatch_guard,
                              graphics_mode="off")))
    write_record(str(tmp_path / f"{run.run_id}.json"), record)


def _fail_stop(tmp_path, run, step):
    logs = tmp_path / f"{run.run_id}.logs"
    logs.mkdir()
    (logs / "summary.json").write_text(json.dumps({
        "run_id": run.run_id, "steps": [
            {"step": "bridge_connected", "ok": True, "detail": ""},
            {"step": step, "ok": False, "detail": "stopped"}]}),
        encoding="utf-8")


@PENDING
def test_report_counts_fail_stops_by_arm(tmp_path, capsys):
    from acav.campaign import build_e1_matrix
    from acav.campaign_cli import main
    runs = build_e1_matrix(ALL, 1).runs
    stops = {("frame_pairing", "on"): "scheduler_exit",
             ("quaternion_order", "on"): "scheduler_exit",
             ("quaternion_conjugate", "on"): "scheduler_exit",
             ("time_offset", "on"): "twin_acking"}
    for run in runs:
        key = (run.mismatch, run.mismatch_guard)
        if key in stops:
            _fail_stop(tmp_path, run, stops[key])
        else:
            _write(tmp_path, run, captured=run.mismatch in ("",
                                                             "angle_rate_units"))
    code = main(["report", "--experiment", "E1", "--no-figures",
                 "--output-dir", str(tmp_path)])
    out = capsys.readouterr().out
    report = json.loads(out[:out.rindex("}") + 1])
    arms = report["arms"]
    assert arms["time_offset/on"]["fail_stops"] == [
        {"run_id": runs[-1].run_id, "failed_steps": ["twin_acking"]}]
    assert arms["frame_pairing/ablated"]["runs"] == 1
    assert arms["control"]["runs"] == 1
    assert report["missing_runs"] == []
    assert arms["angle_rate_units/on"]["role"] == (
        "expected-null control: the twin takes STK's body rate only to "
        "initialise, then integrates its own; no harm is claimed")
    assert code == 0


@PENDING
def test_a_run_with_neither_record_nor_summary_is_missing(tmp_path, capsys):
    from acav.campaign import build_e1_matrix
    from acav.campaign_cli import main
    runs = build_e1_matrix(("distance_units",), 1).runs
    _write(tmp_path, runs[0])
    code = main(["report", "--experiment", "E1", "--e1-types",
                 "distance_units", "--no-figures", "--output-dir",
                 str(tmp_path)])
    out = capsys.readouterr().out
    report = json.loads(out[:out.rindex("}") + 1])
    assert report["missing_runs"] == [runs[1].run_id]
    assert code == 1


# --- offline reference checks (not the Simulink model) ---------------------------
#
# The twin's guard (src/stk_matlab/+adcs/simulink_step.m:237-241) stops a
# lockstep run when the read-back quaternion differs from its previous
# achieved command: residual = min(|q_read - q_end|, |q_read + q_end|)
# > 5e-5. Its telemetry hold (build_acav_twin.m, UnpackTelemetry: the
# offsetNs check) keeps it from stepping while any telemetry stamp is
# more than 1000 ns from ScenarioTime. Both are mirrored here in Python;
# the injections are the bridge's own (stk_bridge.core.mismatch_injection).

GUARD = 5e-5
HOLD_NS = 1000


def _residual(read, endpoint):
    minus = math.sqrt(sum((a - b) ** 2 for a, b in zip(read, endpoint)))
    plus = math.sqrt(sum((a + b) ** 2 for a, b in zip(read, endpoint)))
    return min(minus, plus)


def _attitudes():
    """Unit quaternions (x, y, z, w) of off-nadir pointing: rotations of
    5-45 deg about assorted axes (synthetic test fixtures)."""
    out = []
    for angle_deg in (5, 19.7, 30, 45):
        for axis in ((1, 0, 0), (0, 1, 0), (0.6, 0.8, 0), (0.48, 0.6, 0.64)):
            half = math.radians(angle_deg) / 2
            out.append(tuple(a * math.sin(half) for a in axis)
                       + (math.cos(half),))
    return out


def _injected(kind, quaternion):
    from stk_bridge.backends.base import AttitudeSample, StkTime
    from stk_bridge.core.mismatch_injection import inject
    sample = AttitudeSample(StkTime(1717237800.0), "icrf", quaternion)
    return inject(kind, {"attitude": sample})["attitude"]


@PENDING
@pytest.mark.parametrize("kind", ["quaternion_order", "quaternion_conjugate"])
def test_offline_the_guard_sees_the_quaternion_types(kind):
    for q in _attitudes():
        assert _residual(_injected(kind, q).quaternion_xyzw, q) > GUARD


def test_offline_the_guard_sees_the_frame_pairing():
    """Read through the Earth-fixed provider, the attitude is off by the
    Earth-rotation angle about z (here 1 deg, far less than a real
    epoch's), so the residual exceeds the threshold."""
    theta = math.radians(1.0)
    rz = (0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2))

    def mul(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz)
    for q in _attitudes():
        assert _residual(mul(q, rz), q) > GUARD


@PENDING
@pytest.mark.parametrize("kind", ["angle_rate_units", "distance_units",
                                  "time_offset"])
def test_offline_the_guard_cannot_see_the_other_types(kind):
    """They leave the quaternion untouched: residual 0."""
    for q in _attitudes():
        assert _residual(_injected(kind, q).quaternion_xyzw, q) == 0.0


@PENDING
def test_offline_the_time_offset_holds_the_twin():
    """Every telemetry stamp is 37 s from ScenarioTime, far beyond the
    1000 ns hold: the twin never steps or acks, so the run stops at the
    twin-ack wait (fail-stop, no record)."""
    from stk_bridge.backends.base import StkTime
    q = _attitudes()[0]
    sample = _injected("time_offset", q)
    offset_ns = (sample.time.epoch_seconds - StkTime(1717237800.0)
                 .epoch_seconds) * 1e9
    assert abs(offset_ns) > HOLD_NS


@PENDING
def test_offline_degenerate_attitudes_are_named():
    """The quaternion guard is blind at measure-zero attitudes: the
    conjugate of a 180 deg rotation (w = 0) equals its negation."""
    q = (0.0, 0.0, 1.0, 0.0)
    assert _residual(_injected("quaternion_conjugate", q).quaternion_xyzw,
                     q) == 0.0
