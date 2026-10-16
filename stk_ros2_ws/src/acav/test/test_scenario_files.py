"""Scenario files other than the reference (Phase 3a, P10a).

The reference scenario (config/acav_scenario.yaml) stays byte-identical:
every existing record names it. Generated scenarios are separate files
without a ground station -- the ground station is optional in the spec,
the builder and the verifier (skipped, not deleted) -- and the build and
verify commands take any scenario file with --scenario (the older
--config still works).
"""

import re

import pytest
from stk_bridge.backends.mock_backend import MockBackend

from acav import build_cli, verify_cli
from acav.run_record import scenario_targets
from acav.scenario_builder import AcavScenarioBuilder
from acav.scenario_verifier import AcavScenarioVerifier
from acav.spec import AcavSpec

REFERENCE_DIGEST = (
    "c294ae8024093e02b174817e8dcb9ee38fb45dbb783fc60b2cd8cc610e45b28e")


def without_ground_station(text: str) -> str:
    """The reference text with its top-level ground_station section cut."""
    lines = text.replace("\r\n", "\n").split("\n")
    start = lines.index("ground_station:")
    end = next(i for i in range(start + 1, len(lines))
               if re.match(r"[A-Za-z_]+:", lines[i]))
    # Keep the banner comment lines above the next section.
    while end > start and lines[end - 1].startswith("#"):
        end -= 1
    return "\n".join(lines[:start] + lines[end:])


@pytest.fixture
def no_station(tmp_path):
    reference = open(AcavSpec.locate(), encoding="utf-8").read()
    path = tmp_path / "no_station.yaml"
    path.write_text(without_ground_station(reference), encoding="utf-8")
    return str(path)


def test_the_reference_is_unchanged():
    assert scenario_targets().sha256 == REFERENCE_DIGEST
    assert AcavSpec.load().ground_station["name"] == "GS1"


def test_a_scenario_without_a_ground_station_loads(no_station):
    spec = AcavSpec.load(no_station)
    assert spec.ground_station == {}
    assert [t.id for t in spec.targets] == [
        t.id for t in AcavSpec.load().targets]


class TestBuildAndVerify:
    def _built(self, path):
        backend = MockBackend()
        backend.connect()
        spec = AcavSpec.load(path)
        return backend, spec, AcavScenarioBuilder(backend, spec).build()

    def test_the_builder_skips_the_ground_station(self, no_station):
        _, _, report = self._built(no_station)
        assert report.ok, report.errors
        assert report.ground_station_path == ""

    def test_the_verifier_skips_its_checks(self, no_station):
        backend, spec, built = self._built(no_station)
        report = AcavScenarioVerifier(backend, spec, built).verify()
        assert report.ok, report.summary()
        skipped = {r.name: r.status for r in report.results
                   if "round station" in r.name or "levation" in r.name
                   or "errain" in r.name}
        assert len(skipped) == 3
        assert set(skipped.values()) == {"SKIP"}

    def test_the_reference_still_builds_its_station(self):
        _, _, report = self._built(AcavSpec.locate())
        assert report.ground_station_path


class TestCommandLine:
    @pytest.mark.parametrize("flag", ["--scenario", "--config"])
    def test_build_takes_a_scenario_file(self, no_station, flag, capsys):
        assert build_cli.main(["--backend", "mock", "--verify",
                               flag, no_station]) == 0
        assert "no ground station" in capsys.readouterr().out

    def test_verify_takes_a_scenario_file(self, no_station, capsys):
        assert verify_cli.main(["--backend", "mock", "--build",
                                "--scenario", no_station]) == 0
        assert "0 failed" in capsys.readouterr().out
