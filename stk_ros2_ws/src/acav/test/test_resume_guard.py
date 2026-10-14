"""A campaign must not resume over records made by other code.

Resume skips a run whose record is on disk, readable and matches the
requested run (campaign_runner._load_if_usable). check_record() does not
look at the code version, so before this guard a campaign restarted after
a code change silently mixed records from two versions of the bridge,
scheduler and twin -- a mixture `report` can only warn about afterwards.

Code is "the same" when both versions are clean commits with the same git
tree: a message-only history rewrite changes the commit hash but not the
code, and must not invalidate finished runs.
"""

import subprocess

import pytest

from acav import campaign_cli, code_version
from acav.campaign import Campaign, RunSpec
from acav.campaign_runner import CampaignRefused, RunnerOptions, run_campaign
from acav.metrics import CaptureOutcome, RunResult, TargetResult
from acav.run_record import Provenance, RunRecord, write_record

A = "a" * 40            # clean commits; A and A2 share one tree
A2 = "b" * 40
B = "c" * 40            # a different tree
TREES = {A: "t1", A2: "t1", B: "t2"}


def _tree_of(commit):
    return TREES.get(commit)


def _launcher(version, launched=None):
    def launch(run, campaign, output_path):
        if launched is not None:
            launched.append(run.run_id)
        result = RunResult(
            run_id=run.run_id, scheduler=run.scheduler, seed=run.seed,
            targets=[TargetResult(
                target_id="T1", attempted=True,
                outcome=CaptureOutcome.CAPTURED, image_time_s=1.0,
                error_at_image_deg=0.1, max_error_in_hold_deg=0.1)])
        provenance = Provenance(backend_type="com",
                                execution_mode=run.execution_mode,
                                code_version=version)
        write_record(output_path, RunRecord(result=result,
                                            provenance=provenance,
                                            experiment=run.experiment))
    return launch


def _campaign():
    return Campaign("c", "E2", "", runs=(
        RunSpec(experiment="E2", scheduler="kinematic"),
        RunSpec(experiment="E2", scheduler="dynamics_aware")))


def _options(tmp_path, current, **extra):
    return RunnerOptions(output_dir=str(tmp_path), code_version=current,
                         tree_of=_tree_of, **extra)


def _second_campaign(tmp_path, recorded, current, **extra):
    """Finish the campaign with `recorded` code, then restart it."""
    run_campaign(_campaign(), _launcher(recorded),
                 RunnerOptions(output_dir=str(tmp_path)))
    launched = []
    outcome = run_campaign(_campaign(), _launcher(current, launched),
                           _options(tmp_path, current, **extra))
    return outcome, launched


class TestSameCode:
    @pytest.mark.parametrize("recorded, current, expected", [
        (A, A, True),
        (A, A2, True),                      # same tree, rewritten message
        (A, B, False),
        (A + "+dirty", A, False),
        (A, A + "+dirty", False),
        ("", A, False),                     # not recorded
        ("unknown (no git checkout found)", A, False),
        ("d" * 40, A, False),               # not resolvable here
    ])
    def test_same_code(self, recorded, current, expected):
        assert code_version.same_code(recorded, current,
                                      tree_of=_tree_of) is expected

    def test_tree_of_a_real_checkout(self, tmp_path):
        def git(*args):
            return subprocess.run(
                ["git", "-C", str(tmp_path), *args], check=True,
                capture_output=True, text=True).stdout.strip()

        git("init", "-q")
        git("config", "user.email", "t@example.org")
        git("config", "user.name", "t")
        (tmp_path / "a.txt").write_text("one\n")
        git("add", "a.txt")
        git("commit", "-q", "-m", "first")
        first = git("rev-parse", "HEAD")
        git("commit", "-q", "--amend", "-m", "first, reworded")
        reworded = git("rev-parse", "HEAD")
        (tmp_path / "a.txt").write_text("two\n")
        git("commit", "-q", "-am", "second")
        second = git("rev-parse", "HEAD")

        def tree_of(commit):
            return code_version.tree_of(commit, str(tmp_path))

        assert first != reworded
        assert code_version.same_code(first, reworded, tree_of=tree_of)
        assert not code_version.same_code(first, second, tree_of=tree_of)
        assert code_version.tree_of("0" * 40, str(tmp_path)) is None


class TestResumeGuard:
    def test_same_commit_resumes(self, tmp_path):
        outcome, launched = _second_campaign(tmp_path, A, A)
        assert launched == []
        assert outcome.complete

    def test_same_tree_after_a_message_rewrite_resumes(self, tmp_path):
        outcome, launched = _second_campaign(tmp_path, A, A2)
        assert launched == []
        assert outcome.complete

    @pytest.mark.parametrize("recorded", [B, A + "+dirty", ""])
    def test_other_code_is_refused_before_anything_runs(self, tmp_path,
                                                        recorded):
        with pytest.raises(CampaignRefused) as refused:
            _second_campaign(tmp_path, recorded, A)
        message = str(refused.value)
        assert "--resume-across-code-versions" in message
        for run in _campaign().runs:
            assert run.run_id in message
        assert (recorded or "not recorded") in message

    def test_nothing_is_launched_when_refused(self, tmp_path):
        run_campaign(_campaign(), _launcher(B),
                     RunnerOptions(output_dir=str(tmp_path)))
        launched = []
        with pytest.raises(CampaignRefused):
            run_campaign(_campaign(), _launcher(A, launched),
                         _options(tmp_path, A))
        assert launched == []

    def test_the_explicit_flag_resumes_with_a_warning(self, tmp_path,
                                                      caplog):
        outcome, launched = _second_campaign(
            tmp_path, B, A, allow_code_version_mismatch=True)
        assert launched == []
        assert outcome.complete
        assert "resuming over a record from other code" in caplog.text

    def test_without_resume_nothing_is_refused(self, tmp_path):
        outcome, launched = _second_campaign(tmp_path, B, A, resume=False)
        assert len(launched) == 2
        assert outcome.complete

    def test_a_record_that_will_be_re_run_anyway_is_not_refused(
            self, tmp_path):
        """A record that does not match the run is re-run, not resumed
        over, whatever code made it."""
        campaign = Campaign("c", "E5", "", runs=(
            RunSpec(experiment="E5", seed=4),))
        wrong_seed = _launcher(B)

        def launch_with_wrong_seed(run, c, path):
            wrong_seed(RunSpec(experiment="E5", seed=99), c, path)

        run_campaign(campaign, launch_with_wrong_seed,
                     RunnerOptions(output_dir=str(tmp_path)))
        launched = []
        run_campaign(campaign, _launcher(A, launched), _options(tmp_path, A))
        assert launched == [campaign.runs[0].run_id]

    def test_without_a_current_version_the_guard_is_off(self, tmp_path):
        """Programmatic callers that pass no version keep the old
        behaviour; the CLI always passes one."""
        run_campaign(_campaign(), _launcher(B),
                     RunnerOptions(output_dir=str(tmp_path)))
        launched = []
        run_campaign(_campaign(), _launcher(A, launched),
                     RunnerOptions(output_dir=str(tmp_path)))
        assert launched == []


class TestCli:
    def _captured_options(self, monkeypatch, argv):
        captured = {}

        def fake_run_campaign(campaign, launcher, options):
            captured["options"] = options
            raise CampaignRefused("refused for the test")

        monkeypatch.setattr(campaign_cli, "run_campaign", fake_run_campaign)
        monkeypatch.setattr(code_version, "describe",
                            lambda prefer_env=True: A)
        code = campaign_cli.main(argv)
        return code, captured["options"]

    def test_the_cli_passes_its_own_code_version(self, monkeypatch,
                                                 tmp_path):
        code, options = self._captured_options(monkeypatch, [
            "run", "--experiment", "E2", "--output-dir", str(tmp_path),
            "--", "launch", "{run_id}"])
        assert code == 2                      # refused: invocation error
        assert options.code_version == A
        assert options.allow_code_version_mismatch is False

    def test_the_flag_reaches_the_runner(self, monkeypatch, tmp_path):
        _, options = self._captured_options(monkeypatch, [
            "run", "--experiment", "E2", "--output-dir", str(tmp_path),
            "--resume-across-code-versions", "--", "launch", "{run_id}"])
        assert options.allow_code_version_mismatch is True
