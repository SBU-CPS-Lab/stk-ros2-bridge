"""`report` never pools records made in different STK graphics modes.

The graphics mode changes the step time (E6: 120.3 vs 70.2 ms), so a
directory whose records carry more than one provenance.graphics_mode --
including a known mode next to an older record that has none ("") -- is
refused, naming each mode and its runs.
"""

import json

from acav.campaign_cli import graphics_modes, main
from acav.run_record import Provenance, RunRecord, write_record

from test_campaign import _result  # noqa: I100 - shared helper


def _directory(tmp_path, *modes):
    for seed, mode in enumerate(modes):
        name = f"E6-dynamics_aware-lockstep-seed{seed}"
        write_record(str(tmp_path / f"{name}.json"), RunRecord(
            result=_result(name, seed=seed),
            provenance=Provenance(backend_type="com",
                                  execution_mode="lockstep",
                                  graphics_mode=mode)))
        logs = tmp_path / f"{name}.logs"
        logs.mkdir()
        (logs / "summary.json").write_text(json.dumps({"run_id": name}),
                                           encoding="utf-8")


def _report(tmp_path, runs):
    return main(["report", "--experiment", "E6", "--repeats", str(runs),
                 "--no-figures", "--output-dir", str(tmp_path)])


def test_one_mode_is_reported(tmp_path):
    _directory(tmp_path, "off", "off")
    assert _report(tmp_path, 2) == 0


def test_older_records_without_a_mode_still_report(tmp_path):
    _directory(tmp_path, "", "")
    assert _report(tmp_path, 2) == 0


def test_two_modes_are_refused(tmp_path, caplog):
    _directory(tmp_path, "off", "on")
    assert _report(tmp_path, 2) == 1
    assert "graphics mode" in caplog.text
    assert "'off'" in caplog.text and "'on'" in caplog.text
    assert "E6-dynamics_aware-lockstep-seed1" in caplog.text


def test_known_with_unknown_is_refused(tmp_path, caplog):
    _directory(tmp_path, "off", "")
    assert _report(tmp_path, 2) == 1
    assert "not recorded" in caplog.text


def test_graphics_modes_lists_runs_per_mode(tmp_path):
    from acav.campaign_cli import load_results
    _directory(tmp_path, "off", "mixed", "off")
    assert graphics_modes(load_results(str(tmp_path))) == {
        "off": ["E6-dynamics_aware-lockstep-seed0",
                "E6-dynamics_aware-lockstep-seed2"],
        "mixed": ["E6-dynamics_aware-lockstep-seed1"]}
