"""The twin's identity: which run config it loaded and which code it runs.

The Simulink model has its run-config path baked in, so pressing Play on
the PREVIOUS run's model silently loads the previous run's config, and
nothing noticed. The twin now publishes, once ready, on
/acav/twin_identity (std_msgs/String):

    v1 <sha256 of the run-config bytes it parsed> <build fp> <runtime fp>

build fp is the code fingerprint written into the model when it was
built; runtime fp is recomputed from disk when the simulation starts.
The runner checks all three before it starts the scheduler.

The fingerprint must be computed identically in MATLAB
(+adcs/code_fingerprint.m) and here, so both test suites pin the same
vector (FINGERPRINT_VECTOR, also in test_twin_identity.m).
"""

import hashlib
import os

import pytest

from acav import twin_identity as ti

#: The synthetic tree both languages hash; see test_twin_identity.m.
VECTOR_FILES = {
    "stk_matlab/+adcs/a.m": b"a\r\n",
    "stk_matlab/+adcs/+internal/b.m": b"b",
    "stk_matlab/simulink/build_acav_twin.m": b"build",
    "acav/config/acav_scenario.yaml": b"step_s: 0.1\n",
    "stk_msgs/msg/X.msg": b"int32 x\n",
}
#: Not part of the twin's code: must not change the fingerprint.
DECOY_FILES = {
    "stk_matlab/+adcs/notes.txt": b"notes",
    "stk_matlab/other.m": b"other",
    "stk_matlab/simulink/build_twin_template.m": b"template",
    "acav/config/other.yaml": b"x: 1\n",
}
FINGERPRINT_VECTOR = (
    "aecfd2825eaca8f111a228dab00853ec6b496cea8dd98f52773cf3156e476daa")

CONFIG_BYTES = b'{"run_id": "r"}\n'
CONFIG_SHA = "ea0ab5a1b67c36408f52f019e011c48fef54dad1f331ff35d7ec1261bc355fbb"


def _tree(root, files):
    for relative, data in files.items():
        path = root.joinpath(*relative.split("/"))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    return root


@pytest.fixture
def src(tmp_path):
    return _tree(tmp_path / "src", {**VECTOR_FILES, **DECOY_FILES})


@pytest.fixture
def run_config(tmp_path):
    path = tmp_path / "r.runconfig.json"
    path.write_bytes(CONFIG_BYTES)
    return path


def _identity(config=CONFIG_SHA, build=FINGERPRINT_VECTOR[:16],
              runtime=FINGERPRINT_VECTOR[:16]):
    return f"v1 {config} {build} {runtime}"


class TestFingerprint:
    def test_the_shared_vector(self, src):
        assert ti.code_fingerprint(str(src)) == FINGERPRINT_VECTOR

    def test_the_files_covered(self, src):
        assert ti.fingerprint_files(str(src)) == sorted(VECTOR_FILES)

    def test_any_covered_byte_changes_it(self, src):
        (src / "stk_matlab" / "+adcs" / "+internal" / "b.m").write_bytes(
            b"c")
        assert ti.code_fingerprint(str(src)) != FINGERPRINT_VECTOR

    def test_a_new_function_changes_it(self, src):
        (src / "stk_matlab" / "+adcs" / "new.m").write_bytes(b"")
        assert ti.code_fingerprint(str(src)) != FINGERPRINT_VECTOR

    def test_a_missing_required_file_is_an_error(self, src):
        (src / "acav" / "config" / "acav_scenario.yaml").unlink()
        with pytest.raises(FileNotFoundError):
            ti.code_fingerprint(str(src))

    def test_this_checkout_has_a_fingerprint(self):
        root = ti.source_root()
        files = ti.fingerprint_files(root)
        assert "stk_matlab/+adcs/simulink_step.m" in files
        assert "stk_matlab/simulink/build_acav_twin.m" in files
        assert "acav/config/acav_scenario.yaml" in files
        assert len(ti.code_fingerprint(root)) == 64


class TestParse:
    def test_round_trip(self):
        identity = ti.parse_identity(_identity())
        assert identity.config_sha256 == CONFIG_SHA
        assert identity.build_fingerprint == FINGERPRINT_VECTOR[:16]
        assert identity.runtime_fingerprint == FINGERPRINT_VECTOR[:16]
        assert len(_identity()) == 101 <= ti.MAX_IDENTITY_BYTES

    def test_no_run_config_is_a_dash(self):
        assert ti.parse_identity(_identity(config="-")).config_sha256 == "-"

    @pytest.mark.parametrize("text", [
        "", "v2 " + CONFIG_SHA + " " + "0" * 16 + " " + "0" * 16,
        "v1 " + CONFIG_SHA + " " + "0" * 16,
        "v1 " + CONFIG_SHA[:-1] + " " + "0" * 16 + " " + "0" * 16,
        "v1 " + CONFIG_SHA + " " + "0" * 15 + " " + "0" * 16,
        "v1 " + CONFIG_SHA.upper() + " " + "0" * 16 + " " + "0" * 16,
    ])
    def test_malformed(self, text):
        with pytest.raises(ValueError):
            ti.parse_identity(text)


class TestCheck:
    def test_a_matching_twin(self, src, run_config):
        assert ti.check_identity(_identity(), str(run_config),
                                 str(src)) == []

    def test_the_previous_runs_model(self, src, run_config, tmp_path):
        """Its baked path loads the previous run's config."""
        previous = hashlib.sha256(b'{"run_id": "previous"}\n').hexdigest()
        problems = ti.check_identity(_identity(config=previous),
                                     str(run_config), str(src))
        assert len(problems) == 1
        assert "different run configuration" in problems[0]
        assert previous in problems[0] and CONFIG_SHA in problems[0]

    def test_a_config_edited_after_the_twin_loaded_it(self, src,
                                                      run_config):
        run_config.write_bytes(CONFIG_BYTES + b" ")
        problems = ti.check_identity(_identity(), str(run_config), str(src))
        assert len(problems) == 1
        assert "different run configuration" in problems[0]

    def test_no_run_config_loaded(self, src, run_config):
        problems = ti.check_identity(_identity(config="-"), str(run_config),
                                     str(src))
        assert "no run configuration" in problems[0]

    def test_a_model_built_from_other_code(self, src, run_config):
        problems = ti.check_identity(_identity(build="f" * 16),
                                     str(run_config), str(src))
        assert len(problems) == 1
        assert "rebuild" in problems[0]

    def test_twin_code_differs_from_this_checkout(self, src, run_config):
        (src / "stk_matlab" / "+adcs" / "a.m").write_bytes(b"edited")
        problems = ti.check_identity(_identity(), str(run_config), str(src))
        assert len(problems) == 1
        assert "differs from this checkout" in problems[0]

    def test_unreadable(self, src, run_config):
        problems = ti.check_identity("garbage", str(run_config), str(src))
        assert len(problems) == 1 and "not readable" in problems[0]

    def test_a_missing_run_config_file(self, src, tmp_path):
        problems = ti.check_identity(_identity(), str(tmp_path / "x.json"),
                                     str(src))
        assert problems and "cannot read" in problems[0]


class TestModelSource:
    """The Simulink build and the runner must agree on the topic and the
    message capacity; the model itself is only built on the deploy
    machine, so the agreement is checked in the sources."""

    def _source(self, *relative):
        path = os.path.join(ti.source_root(), *relative)
        with open(path, encoding="utf-8") as handle:
            return handle.read()

    def test_the_model_publishes_on_the_runners_topic(self):
        source = self._source("stk_matlab", "simulink", "build_acav_twin.m")
        assert f'"{ti.IDENTITY_TOPIC}"' in source
        assert "TwinIdentityPub" in source
        assert "adcs.code_fingerprint()" in source

    def test_the_adapter_uses_the_runners_capacity(self):
        source = self._source("stk_matlab", "+adcs", "simulink_step.m")
        assert f"zeros(1, {ti.MAX_IDENTITY_BYTES}, 'uint8')" in source
