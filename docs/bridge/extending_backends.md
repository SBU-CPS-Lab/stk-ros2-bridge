# Extending Backends

How to add a new `StkBackend` implementation, or a new capability to an
existing one, without breaking the layering the rest of the project
depends on.

## The contract

Everything above `backends/` codes against
`backends/base.py::StkBackend` and the dataclasses declared alongside
it (`StateSample`, `AttitudeSample`, `DataProviderQuery`, etc.). Nothing
in `core/`, `interfaces/`, or `nodes/` may import `agi.stk12`,
`win32com`, `pythoncom`, or socket-level Connect machinery directly --
this is enforced by an AST-based check, which you should re-run after any
change here:

```python
import ast, glob
STK = {'agi', 'win32com', 'pythoncom', 'comtypes'}
for f in glob.glob('stk_bridge/core/*.py') + glob.glob('stk_bridge/interfaces/*.py') + glob.glob('stk_bridge/nodes/*.py'):
    tree = ast.parse(open(f).read())
    imports = {n.split('.')[0] for node in ast.walk(tree)
               for n in (([a.name for a in node.names] if isinstance(node, ast.Import)
                          else [node.module or '']) if isinstance(node, (ast.Import, ast.ImportFrom)) else [])}
    bad = imports & STK
    if bad:
        print(f, bad)
```

The one intentional exception is `core/com_thread.py`'s lazy import of
`pythoncom` to initialise the COM apartment -- documented explicitly at
that import site because apartment setup is a property of the *thread*,
which is what that module owns.

## Units, frames, and quaternions: the contract every backend must honour

* **Return SI.** Call `configure_units()` from `connect()`. If a
  dimension can't be forced (see `AngleRateScale`'s handling of angular
  rate on STK 12.6), document the exception in the backend's
  `configure_units()` docstring the way `com_backend.py` does -- don't
  silently return non-SI values.
* **Quaternions are `(x, y, z, w)`** on this interface, vector part
  first. If the underlying API disagrees, reorder inside the backend;
  never leak the underlying convention upward.
* **Times are `StkTime`** (UTC Unix epoch float). Format/parse
  STK-specific string representations inside the backend.

None of these should be trusted from documentation alone --
`core/validation.py` exists specifically because this project's own
assumptions about STK 12.6's quaternion pairing and angle-rate units
were wrong on first pass and only caught by testing against a live
install. Any new backend should be run through
`scripts/validate_conventions.py --backend <yours>` before being
trusted.

## Adding a new backend

1. Subclass `StkBackend` in a new file under `backends/`.
2. Implement every abstract method. `ComBackend` is the fullest
   reference implementation; `EngineBackend` shows how much can be
   inherited when the underlying API is the same Object Model
   (`EngineBackend` subclasses `ComBackend` and overrides only
   acquisition and graphics capability).
3. Populate `BackendCapabilities` honestly during `connect()` --
   `core/scenario_manager.py` and the action-server goal callbacks
   (`ExecuteManeuver`, `ComputeCoverage`) gate on these, and an
   optimistic-but-wrong capability flag produces a failure much later
   and further from its cause than a `LicenseError` at startup would.
4. Implement `list_data_providers()` if at all possible. Guessing
   provider/element names one at a time does not scale across STK
   versions -- see `core/data_providers.py`'s `discover()` and
   `resolve_spec_provider()`, and the real 190-provider inventory this
   project pulled from a live 12.6 install partway through development,
   which corrected several assumed provider names.
5. Register the type in `backends/base.py::BackendType` and wire
   construction into `backends/factory.py`.
6. Add a `test_<backend>_backend.py` following `test_mock_backend.py`'s
   pattern -- in particular, the tests that verify the backend
   reproduces STK's *bad* behaviours (grouped-provider guard,
   monotonicity rejection, empty-ephemeris-before-propagation) rather
   than being more forgiving than production. A backend that is kinder
   than real STK will pass tests that then fail against the real thing.

## Further contracts

Three parts of `StkBackend` that other features depend on:

* `revision` (`backends/base.py`): a counter that changes on every
  structural change (scenario or object lifecycle, propagator changes,
  real-time attitude enable/ClearData, raw Connect commands) but not on
  attitude sample writes. Orbit batching (`core/orbit_batch.py`) keeps
  its audited blocks only while it is unchanged; a backend that cannot
  tell returns None, and batching then reads instantaneously.
* `get_current_scenario_name` and `graphics_state` are not abstract:
  the defaults refuse scenario reuse and report no graphics. The ACAV
  runs need both from the COM backend (`scenario.use_existing`, the
  graphics-off requirement).
* Starting STK is a decision for the operator, not the backend: the
  COM backend attaches to a running STK Desktop and starts one only with
  `backend.com.start_stk` (`com_backend.py` `_obtain_application`). A
  new desktop-style backend should follow the same rule, since the run
  orchestrator refuses an STK it cannot attribute to the operator
  (`acav/stk_process.py`).

## The gRPC path (STK 12.8+, not yet needed)

STK 12.6 has no gRPC remoting -- `agi.stk12` is in-process only on this
version (see [architecture](architecture.md) section 3). If this project is
ever upgraded past STK 12.8, a `grpc_backend.py` implementing the same
`StkBackend` contract against `agi.stk12.stkengine`'s remotable
`STKRuntime` would slot in as a fifth backend type with **zero changes**
required to `core/`, `interfaces/`, or `nodes/` -- that isolation is the
entire point of the abstraction. Do not special-case gRPC anywhere
above the backend layer in anticipation of this; add it when it's
actually needed, following the same pattern as the four backends here.

## Adding a capability to an existing backend

Follow `com_backend.py`'s pattern for version-uncertain API surface:

* Try multiple candidate spellings (`_resolve_enum`, the Look-Ahead
  property search in `set_propagator`) rather than assuming one.
* Fall back to the Connect string form when the typed Object Model
  path's spelling is uncertain and the operation is not
  latency-sensitive (setup-time operations, not the sense/actuate hot
  path).
* Log at WARNING with the list of things actually tried when nothing
  resolves, not just "failed" -- see `_configure_realtime_durations`'s
  fallback logging for the pattern: dump `dir()` filtered to plausible
  names so the next person (or the next `check_environment.py` run) has
  something concrete to try.
