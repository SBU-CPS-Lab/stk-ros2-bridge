"""A campaign is not started from modified or unidentifiable code.

Every record of such a campaign would carry `<commit>+dirty` (or
`unknown (...)`), and `report` could then only warn that the results
cannot be cited. Refusing before the first run costs nothing; finding
out after a 100-run campaign costs the campaign. --allow-dirty is the
explicit way to run anyway, e.g. for a trial.
"""

import pytest

from acav import campaign_cli, code_version
from acav.campaign import Campaign, RunSpec
from acav.campaign_runner import CampaignRefused, RunnerOptions, run_campaign

CLEAN = "a" * 40


def _campaign():
    return Campaign("c", "E2", "", runs=(
        RunSpec(experiment="E2", scheduler="kinematic"),))


def _never_launched(run, campaign, path):
    raise AssertionError("nothing may be launched")


@pytest.mark.parametrize("version", [
    CLEAN + "+dirty", CLEAN + "+unknown-status",
    "unknown (no git checkout found)"])
def test_modified_or_unknown_code_is_refused(tmp_path, version):
    with pytest.raises(CampaignRefused) as refused:
        run_campaign(_campaign(), _never_launched, RunnerOptions(
            output_dir=str(tmp_path), code_version=version))
    assert version in str(refused.value)
    assert "--allow-dirty" in str(refused.value)


def test_the_flag_runs_anyway_with_a_warning(tmp_path, caplog):
    launched = []
    run_campaign(_campaign(), lambda run, c, path: launched.append(run),
                 RunnerOptions(output_dir=str(tmp_path),
                               code_version=CLEAN + "+dirty",
                               allow_dirty=True))
    assert len(launched) == 1
    assert "not a clean commit" in caplog.text


def test_a_clean_commit_runs(tmp_path):
    launched = []
    run_campaign(_campaign(), lambda run, c, path: launched.append(run),
                 RunnerOptions(output_dir=str(tmp_path), code_version=CLEAN))
    assert len(launched) == 1


def test_a_dry_run_is_never_refused(tmp_path):
    outcome = run_campaign(_campaign(), _never_launched, RunnerOptions(
        output_dir=str(tmp_path), code_version=CLEAN + "+dirty",
        dry_run=True))
    assert [o.status for o in outcome.outcomes] == ["skipped"]


def test_without_a_version_the_guard_is_off(tmp_path):
    launched = []
    run_campaign(_campaign(), lambda run, c, path: launched.append(run),
                 RunnerOptions(output_dir=str(tmp_path)))
    assert len(launched) == 1


def test_the_cli_flag_reaches_the_runner(monkeypatch, tmp_path):
    captured = {}

    def fake_run_campaign(campaign, launcher, options):
        captured["options"] = options
        raise CampaignRefused("refused for the test")

    monkeypatch.setattr(campaign_cli, "run_campaign", fake_run_campaign)
    monkeypatch.setattr(code_version, "describe",
                        lambda prefer_env=True: CLEAN + "+dirty")
    base = ["run", "--experiment", "E2", "--output-dir", str(tmp_path)]
    assert campaign_cli.main(base + ["--", "launch"]) == 2
    assert captured["options"].allow_dirty is False
    campaign_cli.main(base + ["--allow-dirty", "--", "launch"])
    assert captured["options"].allow_dirty is True
