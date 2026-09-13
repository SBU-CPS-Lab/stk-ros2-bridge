"""Unit tests for the COM backend's adapter logic, without STK.

``ComBackend`` cannot be exercised end to end off Windows, but the parts
that translate between the bridge's contracts and STK's Object Model are
plain Python and are exactly where the version-drift bugs live. Each
test below corresponds to a failure measured on the live STK 12.6
install, reproduced here with a stand-in COM object so it can never cost
a second round trip to the Windows machine.
"""

import math

import pytest

from stk_bridge.backends.base import DataProviderQuery, StkTime
from stk_bridge.backends.com_backend import ComBackend
from stk_bridge.exceptions import QueryError


class FixedProvider:
    """STK's ``AgDataPrvFixed``: no epochs, and **no ExecSingle at all**.

    Static objects -- Target, Facility, Place -- serve every provider
    through this interface, position included.
    """

    def __init__(self):
        self.calls = []

    def Exec(self):                                  # noqa: N802
        self.calls.append(("Exec", ()))
        return "fixed-result"

    def ExecElements(self, elements):                # noqa: N802
        self.calls.append(("ExecElements", tuple(elements)))
        return "fixed-elements-result"


class TimeVarProvider:
    """STK's ``AgDataPrvTimeVar``: everything takes epochs."""

    def __init__(self):
        self.calls = []

    def Exec(self, start, stop, step):               # noqa: N802
        self.calls.append(("Exec", (start, stop, step)))
        return "timevar-span-result"

    def ExecSingle(self, when):                      # noqa: N802
        self.calls.append(("ExecSingle", (when,)))
        return "timevar-single-result"

    def ExecElements(self, start, stop, step, elements):   # noqa: N802
        self.calls.append(("ExecElements", (start, stop, step)))
        return "timevar-elements-result"

    def ExecSingleElements(self, when, elements):    # noqa: N802
        self.calls.append(("ExecSingleElements", (when,)))
        return "timevar-single-elements-result"


def query(**kwargs) -> DataProviderQuery:
    base = dict(object_path="*/Target/T1", provider_name="LLA State",
                start=StkTime(1_717_237_800.0), stop=StkTime(1_717_237_800.0))
    base.update(kwargs)
    return DataProviderQuery(**base)


class TestFixedDataProviders:
    """`'AgDataPrvFixed' object has no attribute 'ExecSingle'`.

    That one missing branch made every target and facility position
    unreadable on the live install. Because the fallback chain then
    failed too, a unit error that had put all seven targets in the wrong
    place could not be detected, and the scenario reported itself as
    merely "UNVERIFIED" rather than wrong.
    """

    def test_a_fixed_provider_is_executed_without_epochs(self):
        backend = ComBackend()
        provider = FixedProvider()
        assert backend._exec_provider(provider, query()) == "fixed-result"
        assert provider.calls == [("Exec", ())]

    def test_a_fixed_provider_never_sees_a_time_argument(self):
        """Passing a span to a fixed provider is a TypeError, not a
        graceful degradation, so the branch must be taken on the
        provider's shape rather than on the query's."""
        backend = ComBackend()
        provider = FixedProvider()
        backend._exec_provider(provider, query(stop=StkTime(1_717_238_520.0),
                                               step_s=10.0))
        assert provider.calls == [("Exec", ())]

    def test_element_selection_still_works_on_a_fixed_provider(self):
        backend = ComBackend()
        provider = FixedProvider()
        result = backend._exec_provider(
            provider, query(elements=("Lat", "Lon")))
        assert result == "fixed-elements-result"
        assert provider.calls == [("ExecElements", ("Lat", "Lon"))]

    def test_a_failing_fixed_provider_reports_which_interface_it_used(self):
        class Broken(FixedProvider):
            def Exec(self):                          # noqa: N802
                raise RuntimeError("no data")

        backend = ComBackend()
        with pytest.raises(QueryError, match="fixed-provider"):
            backend._exec_provider(Broken(), query())


class TestTimeVaryingDataProvidersAreUnaffected:
    """The fix must not change the satellite path, which already worked."""

    def test_instantaneous_query_uses_exec_single(self):
        backend = ComBackend()
        provider = TimeVarProvider()
        assert backend._exec_provider(provider, query()) == \
            "timevar-single-result"
        assert provider.calls[0][0] == "ExecSingle"

    def test_span_query_uses_exec(self):
        backend = ComBackend()
        provider = TimeVarProvider()
        result = backend._exec_provider(
            provider, query(stop=StkTime(1_717_238_520.0), step_s=10.0))
        assert result == "timevar-span-result"
        assert provider.calls[0][0] == "Exec"
        assert provider.calls[0][1][2] == 10.0

    def test_element_queries_pick_the_matching_overload(self):
        backend = ComBackend()
        provider = TimeVarProvider()
        backend._exec_provider(provider, query(elements=("q1",)))
        backend._exec_provider(provider, query(elements=("q1",),
                                               stop=StkTime(1_717_238_520.0)))
        assert [c[0] for c in provider.calls] == [
            "ExecSingleElements", "ExecElements"]


class _UnitPreferences:
    def __init__(self, actual):
        self._actual = actual
        self.set_calls = []

    def SetCurrentUnit(self, dimension, unit):       # noqa: N802
        self.set_calls.append((dimension, unit))

    def GetCurrentUnitAbbrv(self, dimension):        # noqa: N802
        return self._actual.get(dimension, "deg")


class _Root:
    def __init__(self, actual):
        self.UnitPreferences = _UnitPreferences(actual)


class TestUnitPreferencesAreVerified:
    """`SetCurrentUnit("Angle", "rad")` succeeded and LLA stayed in degrees.

    Latitude and Longitude are separate unit dimensions from Angle in
    STK, so setting Angle alone left every geodetic angle in degrees
    while the bridge logged that units were SI.
    """

    WANTED = (("Distance", "m"), ("Angle", "rad"), ("Latitude", "rad"))

    def test_latitude_and_longitude_are_set_not_just_angle(self):
        backend = ComBackend()
        backend._root = _Root({"Distance": "m", "Angle": "rad",
                               "Latitude": "rad", "Longitude": "rad",
                               "Time": "sec", "Mass": "kg",
                               "DateFormat": "UTCG", "AngleRate": "rad/sec"})
        backend.configure_units()
        dimensions = [d for d, _ in backend._root.UnitPreferences.set_calls]
        assert "Latitude" in dimensions and "Longitude" in dimensions

    def test_a_preference_that_did_not_take_is_reported(self):
        backend = ComBackend()
        backend._root = _Root({"Distance": "m", "Angle": "rad",
                               "Latitude": "deg"})
        mismatched = backend._verify_units(self.WANTED)
        assert len(mismatched) == 1
        assert "Latitude" in mismatched[0]

    def test_agreement_is_reported_as_nothing_to_say(self):
        backend = ComBackend()
        backend._root = _Root({"Distance": "m", "Angle": "rad",
                               "Latitude": "rad"})
        assert backend._verify_units(self.WANTED) == []

    def test_an_install_without_the_getter_does_not_fail_the_connect(self):
        """Read-back is a diagnostic, not a requirement."""
        class NoGetter(_UnitPreferences):
            def GetCurrentUnitAbbrv(self, dimension):    # noqa: N802
                raise AttributeError("not available on this build")

        backend = ComBackend()
        backend._root = _Root({})
        backend._root.UnitPreferences = NoGetter({})
        assert backend._verify_units(self.WANTED) == []


class TestSensorConeReadBack:
    """The cone angle is reported in STK's current Angle unit -- which is
    the assumption that has already been wrong once."""

    class _Sensor:
        def __init__(self, cone_angle):
            self.Pattern = type("P", (), {"ConeAngle": cone_angle})()

    def _backend_with(self, cone_angle):
        backend = ComBackend()
        sensor = self._Sensor(cone_angle)
        backend._get_object = lambda path: sensor
        return backend

    def test_a_radian_value_is_taken_as_radians(self):
        backend = self._backend_with(math.radians(2.0))
        assert backend.get_sensor_cone_half_angle_rad("s") == \
            pytest.approx(math.radians(2.0))

    def test_a_value_above_ninety_degrees_must_be_degrees(self):
        """A conic half-angle cannot exceed 90 deg = 1.5708 rad, so a
        larger number is being reported in the other unit."""
        backend = self._backend_with(45.0)
        assert backend.get_sensor_cone_half_angle_rad("s") == \
            pytest.approx(math.radians(45.0))

    def test_an_unreadable_pattern_returns_none_rather_than_guessing(self):
        backend = ComBackend()

        def raise_it(path):
            raise RuntimeError("no such object")

        backend._get_object = raise_it
        assert backend.get_sensor_cone_half_angle_rad("s") is None
