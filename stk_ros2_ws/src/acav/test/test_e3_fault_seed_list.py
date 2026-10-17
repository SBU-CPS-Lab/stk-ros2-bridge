"""E3: an explicit fault-seed list, and the old comparison labelled.

--fault-seeds K keeps meaning seeds 0..K-1; with a comma it is the list
itself (0,1,4; one seed: 4,). The old single-signature determinism
comparison stays in the E3 report, labelled as superseded by the
per-cell replay check.
"""

import json

import pytest

from acav.fault_injection import plan_fault

WINDOW = 720.0


def test_a_seed_list_plans_exactly_those_faults():
    from acav.campaign import build_e3_matrix
    runs = build_e3_matrix(1, (0, 1, 4), WINDOW).runs
    faults = {r.fault for r in runs if r.fault is not None}
    assert {f.seed for f in faults} == {0, 1, 4}
    assert plan_fault(4, WINDOW) in faults
    assert len(runs) == (1 + 3) * (2 + 2)


def test_a_count_still_means_zero_to_k_minus_one():
    from acav.campaign import build_e3_matrix
    runs = build_e3_matrix(1, 2, WINDOW).runs
    assert {r.fault.seed for r in runs if r.fault} == {0, 1}


@pytest.mark.parametrize("text, seeds", [("3", (0, 1, 2)), ("0,1,4", (0, 1, 4)),
                                         ("4,", (4,)), ("0", ())])
def test_the_cli_parses_both_forms(text, seeds):
    from acav.campaign_cli import _parser
    args = _parser().parse_args(["plan", "--experiment", "E3",
                                 "--output-dir", "o", "--fault-seeds", text])
    assert args.fault_seeds == seeds


def test_plan_with_a_seed_list(tmp_path, capsys):
    from acav.campaign_cli import main
    assert main(["plan", "--experiment", "E3", "--repeats", "1",
                 "--fault-seeds", "4,", "--output-dir", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "= (1 + 1) x (2 + 2 x 1) = 8" in out
    assert "fault seeds: 4" in out


def test_the_old_comparison_is_labelled(tmp_path, capsys):
    from acav.campaign import build_e3_matrix
    from acav.campaign_cli import main
    from test_e3_matrix import _record
    for run in build_e3_matrix(1, 0, WINDOW).runs:
        _record(tmp_path, run)
    main(["report", "--experiment", "E3", "--repeats", "1", "--no-figures",
          "--output-dir", str(tmp_path)])
    report = json.loads(capsys.readouterr().out)
    assert report["determinism_note"] == (
        "informational; superseded by the per-cell replay check")
    assert "determinism" in report
