"""Unit tests for the COM backend's adapter logic, without STK.

``ComBackend`` cannot be exercised end to end off Windows, but the parts
that translate between the bridge's contracts and STK's Object Model are
plain Python and are exactly where the version-drift bugs live. Each
test below corresponds to a failure measured on the live STK 12.6
install, reproduced here with a stand-in COM object so it can never cost
a second round trip to the Windows machine.
"""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from stk_bridge.backends.base import DataProviderQuery, StkTime
from stk_bridge.backends.com_backend import ComBackend
from stk_bridge.exceptions import QueryError


def test_instantaneous_query_does_not_read_scenario_stop():
    backend = ComBackend()
    class Root:
        @property
        def CurrentScenario(self):
            raise AssertionError('Unexpected scenario COM property access')
    backend._root = Root()
    provider = TimeVarProvider()
    backend._exec_provider(provider, query(stop=None))
    assert len(provider.calls) == 1


def test_fallback_extract_reads_metadata_once_and_returns_fresh_values():
    backend = ComBackend()
    backend._capabilities.numpy_bulk_extraction = False
    class Column:
        def __init__(self, name, value):
            self.name, self.value, self.name_reads = name, value, 0
        @property
        def ElementName(self):
            self.name_reads += 1
            return self.name
        def GetValues(self):
            return [self.value]
    cols = [Column('x', 1.0), Column('y', 2.0)]
    datasets = SimpleNamespace(Count=2, Item=Mock(side_effect=lambda i: cols[i]))
    result = SimpleNamespace(DataSets=datasets)
    first = backend._extract(result, query())
    assert first.columns == {'x': [1.0], 'y': [2.0]}
    assert datasets.Item.call_count == 2
    assert [c.name_reads for c in cols] == [1, 1]
    cols[0].value = 3.0
    assert backend._extract(result, query()).columns['x'] == [3.0]
    assert first.columns['x'] == [1.0]


def test_bulk_failure_preserves_fallback_values_and_counts():
    backend = ComBackend()
    backend._capabilities.numpy_bulk_extraction = True
    cols = [SimpleNamespace(ElementName=n, GetValues=lambda v=v: [v])
            for n, v in [('x', 1.0), ('y', 2.0)]]
    datasets = SimpleNamespace(Count=2, Item=Mock(side_effect=lambda i: cols[i]),
                              ToNumpyArray=Mock(side_effect=RuntimeError('unsupported')))
    data = backend._extract(SimpleNamespace(DataSets=datasets), query())
    assert data.columns == {'x': [1.0], 'y': [2.0]}
    assert datasets.Item.call_count == 2
    assert backend._extraction_counts == {'bulk': 0, 'fallback': 1, 'bulk_failed': 1}


def test_bulk_extract_and_periodic_profile(monkeypatch, caplog):
    import numpy as np
    backend = ComBackend()
    backend._capabilities.numpy_bulk_extraction = True
    cols = [SimpleNamespace(ElementName=n, GetValues=Mock()) for n in ['x', 'y']]
    datasets = SimpleNamespace(Count=2, Item=Mock(side_effect=lambda i: cols[i]),
                              ToNumpyArray=lambda: np.array([[1., 2.], [3., 4.]]))
    data = backend._extract(SimpleNamespace(DataSets=datasets), query())
    assert data.columns == {'x': [1., 3.], 'y': [2., 4.]}
    assert not any(c.GetValues.called for c in cols)
    assert datasets.Item.call_count == 2
    stages = {key[2]: value for key, value in backend._extraction_timings.items()}
    assert stages['item'][0] == stages['name'][0] == 2
    assert stages['bulk_fetch'][0] == stages['python_conversion'][0] == 1
    assert stages['total'][1] >= sum(v[1] for k, v in stages.items() if k != 'total')
    with caplog.at_level('INFO'):
        backend.disconnect()
    assert 'COM extraction detail:' in caplog.text
    assert not backend._extraction_timings
    backend._query_timings_since = 0
    monkeypatch.setattr('stk_bridge.backends.com_backend.time.perf_counter', lambda: 61.)
    with caplog.at_level('INFO'):
        backend._record_query_timing(query(), 'extract', .002)
    assert 'mean_ms=2.000' in caplog.text
    assert "'bulk': 1" in caplog.text
    assert not backend._query_timings


def test_extraction_failure_is_profiled_without_hiding_error():
    backend = ComBackend()
    with pytest.raises(QueryError):
        backend._extract(object(), query())
    stages = {key[2]: value for key, value in backend._extraction_timings.items()}
    assert stages['datasets'][3] == 1
    assert stages['total'][3] == 1


def test_bulk_names_avoid_items_and_track_reordered_fresh_columns():
    import numpy as np
    backend = ComBackend()
    backend._capabilities.numpy_bulk_extraction = True
    datasets = SimpleNamespace(ElementNames=['x', 'y'],
                              Item=Mock(side_effect=AssertionError('Item used')),
                              ToNumpyArray=Mock(return_value=np.array([[1., 2.]])))
    result = SimpleNamespace(DataSets=datasets)
    assert backend._extract(result, query()).columns == {'x': [1.], 'y': [2.]}
    datasets.ElementNames = ['y', 'x']
    datasets.ToNumpyArray.return_value = np.array([[4., 3.]])
    assert backend._extract(result, query()).columns == {'y': [4.], 'x': [3.]}
    datasets.Item.assert_not_called()


def test_bulk_shape_mismatch_uses_actual_column_metadata():
    import numpy as np
    backend = ComBackend()
    backend._capabilities.numpy_bulk_extraction = True
    cols = [SimpleNamespace(ElementName='x', GetValues=lambda: [7.])]
    datasets = SimpleNamespace(ElementNames=['x', 'y'], Count=1,
                              Item=Mock(side_effect=lambda i: cols[i]),
                              ToNumpyArray=lambda: np.array([[7.]]))
    assert backend._extract(SimpleNamespace(DataSets=datasets), query()).columns == {'x': [7.]}
    assert backend._extraction_counts['bulk_failed'] == 1


def test_provider_handles_are_reused_but_every_epoch_is_executed():
    backend = ComBackend()
    backend._connected = True
    provider = TimeVarProvider()
    obj = SimpleNamespace(DataProviders=Mock())
    obj.DataProviders.Item.return_value = provider
    backend._get_object = Mock(return_value=obj)
    backend._extract = Mock(side_effect=lambda result, request: request.start)
    first = query()
    second = query(start=first.start + .1)
    assert backend.query_data_provider(first) == first.start
    assert backend.query_data_provider(second) == second.start
    assert backend._get_object.call_count == 1
    assert obj.DataProviders.Item.call_count == 1
    assert len(provider.calls) == 2
    assert provider.calls[0] != provider.calls[1]


def test_provider_cache_separates_frames_and_drops_failed_handles():
    backend = ComBackend()
    backend._connected = True
    backend._get_object = Mock(return_value=SimpleNamespace(DataProviders=Mock()))
    backend._select_group_member = Mock(return_value=TimeVarProvider())
    backend._extract = Mock(return_value='fresh')
    backend.query_data_provider(query(group_element='ICRF'))
    backend.query_data_provider(query(group_element='Fixed'))
    assert backend._get_object.call_count == 2
    backend._extract.side_effect = RuntimeError('stale handle')
    with pytest.raises(RuntimeError, match='stale handle'):
        backend.query_data_provider(query(group_element='ICRF'))
    backend._extract.side_effect = None
    backend.query_data_provider(query(group_element='ICRF'))
    assert backend._get_object.call_count == 3


@pytest.mark.parametrize('operation', ['new_scenario', 'load_scenario',
                                      'close_scenario', 'delete_object',
                                      'execute_connect', 'disconnect'])
def test_provider_cache_invalidated_on_lifecycle_changes(operation):
    backend = ComBackend()
    backend._connected = True
    backend._root = Mock()
    backend._execute = Mock()
    backend._provider_cache['old'] = object()
    args = [] if operation in ('close_scenario','disconnect') else ['example']
    getattr(backend, operation)(*args)
    assert not backend._provider_cache


def test_fixed_position_resolves_requested_axes_and_each_epoch():
    backend = ComBackend()
    backend._connected = True
    point = Mock()
    point.LocateInSystem.side_effect = [
        SimpleNamespace(Position=SimpleNamespace(X=x, Y=2, Z=3)) for x in (10, 11, 12)]
    obj = SimpleNamespace(CentralBodyName='Earth',
                          Vgt=SimpleNamespace(Points=Mock()))
    obj.Vgt.Points.Item.return_value = point
    obj.DataProviders = Mock()
    obj.DataProviders.Item.return_value = FixedProvider()
    backend._get_object = Mock(return_value=obj)
    body = Mock()
    backend._root = SimpleNamespace(CentralBodies=Mock())
    backend._root.CentralBodies.Item.return_value = body
    request = query(provider_name='Cartesian Position', group_element='ICRF',
                    stop=StkTime(1717237802), step_s=1)
    data = backend.query_data_provider(request)
    body.Vgt.Systems.Item.assert_called_once_with('ICRF')
    assert data.columns['x'] == [10, 11, 12]
    assert data.row_count == 3
    assert point.LocateInSystem.call_count == 3


def test_missing_requested_fixed_object_frame_is_not_silently_ignored():
    backend = ComBackend()
    backend._root = Mock()
    backend._root.CentralBodies.Item.side_effect = RuntimeError('unknown frame')
    with pytest.raises(QueryError, match='cannot resolve fixed-object position'):
        backend._fixed_position_in_frame(SimpleNamespace(CentralBodyName='Earth'),
            query(provider_name='Cartesian Position', group_element='bad-frame'))


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
