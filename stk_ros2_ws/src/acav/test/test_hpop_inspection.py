"""The HPOP force-model inspection must only READ from STK.

A recording fake stands in for the agi.stk12 Object Model: every attribute
read and method call is logged, and any attribute assignment fails the
test. That is what makes the script's read-only claim checkable here,
without STK.
"""


import pytest

from acav import hpop_inspection as hi


class Fake:
    """A COM wrapper stand-in that records everything done to it."""

    def __init__(self, log, path="root", values=None, errors=()):
        object.__setattr__(self, "_log", log)
        object.__setattr__(self, "_path", path)
        object.__setattr__(self, "_values", values or {})
        object.__setattr__(self, "_errors", set(errors))

    def __getattr__(self, name):
        path = f"{self._path}.{name}"
        self._log.append(("get", path))
        if path in self._errors:
            raise RuntimeError(f"COM error reading {path}")
        if path in self._values:
            return self._values[path]
        return Fake(self._log, path, self._values, self._errors)

    def __setattr__(self, name, value):
        raise AssertionError(f"write to {self._path}.{name}")

    def __call__(self, *args):
        self._log.append(("call", self._path, args))
        path = f"{self._path}({', '.join(map(str, args))})"
        if path in self._errors:
            raise RuntimeError(f"COM error calling {path}")
        if path in self._values:
            return self._values[path]
        return Fake(self._log, path, self._values, self._errors)

    def __str__(self):
        return f"<{self._path}>"


SAT = "root.GetObjectFromPath(*/Satellite/SSS-1)"
FM = SAT + ".Propagator.ForceModel"


def _rows(values=None, errors=()):
    log = []
    rows = hi.inspect(Fake(log, values=values, errors=errors),
                      "*/Satellite/SSS-1")
    return dict(rows), log


def test_only_reads_and_allow_listed_calls_are_made():
    rows, log = _rows(values={FM + ".ThirdBodyGravity.Count": 2})
    calls = {entry[1].rsplit(".", 1)[-1] for entry in log
             if entry[0] == "call"}
    assert calls <= set(hi.ALLOWED_METHOD_CALLS)
    assert "GetObjectFromPath" in calls and "Item" in calls
    # The fake raises on any assignment, so reaching here proves none.


def test_every_requested_quantity_is_reported():
    rows, _ = _rows()
    for label in ("scenario.StartTime", "scenario.StopTime",
                  "satellite.PropagatorType", "satellite.MassProperties.Mass",
                  "gravity.File", "gravity.MaxDegree", "gravity.MaxOrder",
                  "drag.Use", "drag.DragModelType", "drag.DragModel.Cd",
                  "drag.DragModel.AreaMassRatio", "srp.Use",
                  "srp.SRPModel.ModelType", "srp.SRPModel.Model.Cr",
                  "srp.SRPModel.Model.AreaMassRatio",
                  "third_body.Count", "units.Mass",
                  "attitude_dependent_force_assessment"):
        assert label in rows, label


def test_a_failed_read_is_unreadable_never_a_default():
    rows, _ = _rows(errors={FM + ".Drag.DragModel.Cd",
                            SAT + ".MassProperties.Mass"})
    assert rows["drag.DragModel.Cd"].startswith("UNREADABLE")
    assert rows["satellite.MassProperties.Mass"].startswith("UNREADABLE")


def test_third_bodies_are_listed_individually():
    values = {FM + ".ThirdBodyGravity.Count": 2,
              FM + ".ThirdBodyGravity.Item(0).CentralBody": "Sun",
              FM + ".ThirdBodyGravity.Item(1).CentralBody": "Moon"}
    rows, _ = _rows(values=values)
    assert rows["third_body[0].CentralBody"] == "'Sun'"
    assert rows["third_body[1].CentralBody"] == "'Moon'"


@pytest.mark.parametrize("drag_use, drag_type, srp_use, srp_type, verdict", [
    (False, "eDragModelSpherical", False, "eSRPModelSpherical", "NO"),
    (True, "eDragModelSpherical", True, "eSRPModelSpherical", "NO"),
    (True, "eDragModelPlugin", False, "eSRPModelSpherical", "POSSIBLE"),
    (False, "eDragModelSpherical", True, "eSRPModelPlugin", "POSSIBLE"),
    (False, "eDragModelSpherical", True,
     "eSRPModelGPS_BlkIIR_GSPM04a", "POSSIBLE"),
])
def test_assessment_follows_the_model_types(drag_use, drag_type, srp_use,
                                            srp_type, verdict):
    class Enum:
        def __init__(self, name):
            self.name = name

        def __int__(self):
            return 1

    values = {FM + ".Drag.Use": drag_use,
              FM + ".Drag.DragModelType": Enum(drag_type),
              FM + ".SolarRadiationPressure.Use": srp_use,
              FM + ".SolarRadiationPressure.SRPModel.ModelType": Enum(srp_type),
              FM + ".ThirdBodyGravity.Count": 0}
    rows, _ = _rows(values=values)
    assert rows["attitude_dependent_force_assessment"].startswith(verdict)


def test_assessment_is_undetermined_when_a_model_type_is_unreadable():
    rows, _ = _rows(errors={FM + ".Drag.Use"})
    assert rows["attitude_dependent_force_assessment"].startswith(
        "UNDETERMINED")


class FakeApp:
    def __init__(self, root):
        self.Root = root
        self.shut_down = False

    def ShutDown(self):
        self.shut_down = True


def test_main_writes_the_file_and_stdout_and_never_shuts_stk_down(
        tmp_path, capsys):
    app = FakeApp(Fake([]))
    out = tmp_path / "force_model.txt"
    code = hi.main(["--output", str(out)], attach=lambda: app)
    assert code == 0
    text = out.read_text(encoding="utf-8")
    assert "drag.Use" in text and text == capsys.readouterr().out
    assert out.read_bytes()[:3] != b"\xef\xbb\xbf"
    assert app.shut_down is False


def test_main_refuses_to_overwrite_and_does_not_attach(tmp_path):
    out = tmp_path / "force_model.txt"
    out.write_text("previous", encoding="utf-8")
    attached = []
    code = hi.main(["--output", str(out)],
                   attach=lambda: attached.append(1))
    assert code == 2 and attached == []
    assert out.read_text(encoding="utf-8") == "previous"


def test_main_reports_an_attach_failure_without_starting_stk(tmp_path):
    def attach():
        raise RuntimeError("no running STK")
    code = hi.main(["--output", str(tmp_path / "x.txt")], attach=attach)
    assert code == 2
    assert not (tmp_path / "x.txt").exists()
