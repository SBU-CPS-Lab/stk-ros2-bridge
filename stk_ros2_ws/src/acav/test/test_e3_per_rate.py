"""E3 repeats per execution mode.

--repeats-rtf05 sets the free-run rtf 0.5 repeats (default: --repeats),
--repeats-lockstep the lockstep repeats (default 2, as before). The
author's final configuration is lockstep 3, rtf 1.0 5, rtf 0.5 3, five
fault seeds: (1 + 5) x (3 + 5 + 3) = 66 runs.
"""

from acav.campaign import build_e3_matrix, e3_run_count



def test_the_final_configuration_has_66_runs():
    campaign = build_e3_matrix(5, 5, 720.0, lockstep_repeats=3,
                               rtf05_repeats=3)
    assert len(campaign) == 66 == e3_run_count(5, 5, lockstep_repeats=3,
                                               rtf05_repeats=3)
    modes = [(r.execution_mode, r.real_time_factor) for r in campaign.runs]
    assert modes.count(("lockstep", 1.0)) == 18
    assert modes.count(("free_run", 1.0)) == 30
    assert modes.count(("free_run", 0.5)) == 18


def test_the_defaults_are_unchanged():
    assert len(build_e3_matrix(3, 3, 720.0)) == 32 == e3_run_count(3, 3)


def test_the_plan_takes_both_options(tmp_path, capsys):
    from acav.campaign_cli import main
    assert main(["plan", "--experiment", "E3", "--repeats", "5",
                 "--fault-seeds", "5", "--repeats-lockstep", "3",
                 "--repeats-rtf05", "3", "--output-dir", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert ("E3 runs = (1 + N_FAULT_SEEDS) x (3 lockstep + 5 rtf 1.0 + 3 "
            "rtf 0.5) = (1 + 5) x 11 = 66") in out
    assert "E3: 18 free-run rtf 0.5 runs: 491.4-491.4 min" in out
