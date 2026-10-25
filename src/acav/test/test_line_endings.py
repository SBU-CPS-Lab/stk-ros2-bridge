"""Line endings do not change the code fingerprint or the scenario digest.

Both hashed each file's bytes as stored, and this checkout keeps CRLF in
the working tree (core.autocrlf) over an LF index, so the same commit
checked out with other settings gave another fingerprint and another
scenario digest. Both now hash the text with CRLF turned into LF.

Every record so far names the scenario by its raw CRLF digest; a
committed map (run_record.LEGACY_SCENARIO_DIGESTS) takes that legacy
digest to the normalised one, so those records still cover their
scenario. Any other digest is still another scenario.
"""

from acav import twin_identity as ti
from acav.metrics import CaptureOutcome, RunResult, TargetResult
from acav.run_record import (
    LEGACY_SCENARIO_DIGESTS,
    Provenance,
    RunRecord,
    scenario_digest,
    scenario_targets,
    target_coverage_problems,
)

LEGACY = "c8825803c3073f744ec76f25e3e889606a4c09fcb040aba985cb7e2371706ba9"
NORMALISED = "c294ae8024093e02b174817e8dcb9ee38fb45dbb783fc60b2cd8cc610e45b28e"


def _tree(root, files):
    for relative, data in files.items():
        path = root.joinpath(*relative.split("/"))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    return root


def _code(eol):
    return {
        "stk_matlab/+adcs/a.m": b"x = 1;" + eol + b"y = 2;" + eol,
        "stk_matlab/simulink/build_acav_twin.m": b"build" + eol,
        "acav/config/acav_scenario.yaml": b"step_s: 0.1" + eol,
    }


class TestFingerprint:
    def test_crlf_and_lf_copies_give_one_fingerprint(self, tmp_path):
        lf = _tree(tmp_path / "lf", _code(b"\n"))
        crlf = _tree(tmp_path / "crlf", _code(b"\r\n"))
        assert ti.code_fingerprint(str(lf)) == ti.code_fingerprint(str(crlf))

    def test_other_bytes_still_change_it(self, tmp_path):
        a = _tree(tmp_path / "a", _code(b"\n"))
        files = _code(b"\n")
        files["stk_matlab/+adcs/a.m"] = b"x = 1;\ny = 3;\n"
        b = _tree(tmp_path / "b", files)
        assert ti.code_fingerprint(str(a)) != ti.code_fingerprint(str(b))

    def test_a_lone_cr_is_not_rewritten(self, tmp_path):
        a = _tree(tmp_path / "a", {**_code(b"\n"),
                                   "stk_matlab/+adcs/c.m": b"a\rb"})
        b = _tree(tmp_path / "b", {**_code(b"\n"),
                                   "stk_matlab/+adcs/c.m": b"a\nb"})
        assert ti.code_fingerprint(str(a)) != ti.code_fingerprint(str(b))


class TestScenarioDigest:
    def test_crlf_and_lf_copies_give_one_digest(self, tmp_path):
        (tmp_path / "lf.yaml").write_bytes(b"step_s: 0.1\nx: 2\n")
        (tmp_path / "crlf.yaml").write_bytes(b"step_s: 0.1\r\nx: 2\r\n")
        assert scenario_digest(str(tmp_path / "lf.yaml")) == \
            scenario_digest(str(tmp_path / "crlf.yaml"))

    def test_the_installed_scenario(self):
        assert scenario_targets().sha256 == NORMALISED

    def test_the_legacy_map(self):
        assert LEGACY_SCENARIO_DIGESTS == {LEGACY: NORMALISED}


def _record(sha):
    ids = scenario_targets().target_ids
    return RunRecord(
        result=RunResult(run_id="r", scheduler="dynamics_aware", targets=[
            TargetResult(target_id=i, attempted=False,
                         outcome=CaptureOutcome.NOT_ATTEMPTED,
                         image_time_s=0.0) for i in ids]),
        provenance=Provenance(backend_type="com", scenario_sha256=sha))


class TestCoverage:
    def test_a_legacy_digest_covers_its_scenario(self):
        assert target_coverage_problems(_record(LEGACY),
                                        scenario_targets()) == []

    def test_the_normalised_digest_covers_it(self):
        assert target_coverage_problems(_record(NORMALISED),
                                        scenario_targets()) == []

    def test_an_unknown_digest_is_another_scenario(self):
        problems = target_coverage_problems(_record("0" * 64),
                                            scenario_targets())
        assert len(problems) == 1 and "another scenario" in problems[0]

    def test_a_legacy_digest_does_not_cover_a_changed_scenario(
            self, tmp_path):
        text = open(scenario_targets().path, "rb").read()
        changed = tmp_path / "changed.yaml"
        changed.write_bytes(text + b"# a comment\n")
        problems = target_coverage_problems(
            _record(LEGACY), scenario_targets(str(changed)))
        assert len(problems) == 1 and "another scenario" in problems[0]
