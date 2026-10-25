#!/usr/bin/env python3
"""Verify the local environment against what stk_bridge assumes.

Run this **before** investing in Phases 3-6. It is the cheapest way to
find out that an assumption baked into the backends does not hold on your
particular STK 12.6 install, and it is designed to tell you *what* is
wrong rather than just failing.

    python scripts/check_environment.py
    python scripts/check_environment.py --backend connect --host 10.0.0.5
    python scripts/check_environment.py --backend mock     # always passes

Every check is independent and non-fatal: the script runs them all and
prints a summary, because knowing that four things are wrong is more
useful than being told about the first one and having to iterate.

Exit codes: 0 all checks passed; 1 one or more FAILED; 2 could not run.
"""

from __future__ import annotations

import argparse
import glob
import os
import platform
import sys
import traceback
from typing import Callable, List, Optional, Tuple

# Allow running from a source checkout without installing the package.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

PASS, WARN, FAIL = "PASS", "WARN", "FAIL"

_RESULTS: List[Tuple[str, str, str]] = []


def record(name: str, status: str, detail: str = "") -> None:
    _RESULTS.append((name, status, detail))
    marker = {PASS: "  ok  ", WARN: " warn ", FAIL: " FAIL "}[status]
    print(f"[{marker}] {name}")
    if detail:
        for line in detail.splitlines():
            print(f"           {line}")


def check(name: str):
    """Decorator turning a function into a recorded, non-fatal check."""
    def decorator(func: Callable[[], Tuple[str, str]]):
        def wrapper():
            try:
                status, detail = func()
            except Exception as exc:  # noqa: BLE001 - report, never abort
                status, detail = FAIL, f"{type(exc).__name__}: {exc}"
            record(name, status, detail)
        return wrapper
    return decorator


# ---------------------------------------------------------------------------
# Environment checks (no STK contact)
# ---------------------------------------------------------------------------

@check("Python version >= 3.8 (required by agi.stk12)")
def check_python():
    version = sys.version_info
    detail = f"found {sys.version.split()[0]} at {sys.executable}"
    if version < (3, 8):
        return FAIL, detail + "\nSTK 12.6's Python API requires 3.8 or greater."
    return PASS, detail


@check("Platform")
def check_platform():
    system = platform.system()
    detail = f"{system} {platform.release()} ({platform.machine()})"
    if system == "Windows":
        return PASS, detail + "\nCOM and Engine backends both available."
    return WARN, (
        detail + "\nThe COM backend requires Windows. On this platform use "
        "the 'engine' backend (needs an STK Engine entitlement) or the "
        "'connect' backend against a remote Windows STK."
    )


@check("agi.stk12 importable")
def check_agi_import():
    try:
        import agi.stk12 as pkg
    except ImportError as exc:
        hint = "\n".join([
            f"import failed: {exc}",
            "The wheel ships inside your STK install, not on PyPI. Install it:",
            r'  pip install "C:\Program Files\AGI\STK 12\bin\AgPythonAPI\agi.stk12-12.6.0-py3-none-any.whl"',
            "Adjust the version in the filename to match what is actually",
            "present in that directory.",
        ])
        return FAIL, hint
    version = getattr(pkg, "__version__", "unknown")
    detail = f"agi.stk12 version {version}"
    if version != "unknown" and not version.startswith("12.6"):
        return WARN, (
            detail + "\nThis bridge targets STK 12.6. Other 12.x versions "
            "will mostly work, but note that gRPC remoting (12.8+) is not "
            "used here and enum spellings drift across releases."
        )
    return PASS, detail


@check("agi.stk12 wheel discoverable in an STK install")
def check_wheel_present():
    candidates = [
        r"C:\Program Files\AGI\STK 12\bin\AgPythonAPI",
        r"C:\Program Files\Ansys Inc\STK 12\bin\AgPythonAPI",
        "/opt/stk/bin/AgPythonAPI",
    ]
    env_root = os.environ.get("STK_INSTALL_DIR")
    if env_root:
        candidates.insert(0, os.path.join(env_root, "bin", "AgPythonAPI"))

    for directory in candidates:
        if not os.path.isdir(directory):
            continue
        wheels = glob.glob(os.path.join(directory, "agi.stk*.whl"))
        if wheels:
            return PASS, f"{directory}\n" + "\n".join(
                os.path.basename(w) for w in wheels)
        return WARN, f"{directory} exists but contains no agi.stk*.whl"
    return WARN, (
        "no AgPythonAPI directory found in the usual locations.\n"
        "Set STK_INSTALL_DIR if STK is installed elsewhere. This is only a "
        "warning: if agi.stk12 already imports, the wheel is installed."
    )


@check("Bridge package imports cleanly")
def check_bridge_import():
    from stk_bridge.backends import BackendType, create_backend  # noqa: F401
    from stk_bridge import exceptions  # noqa: F401
    return PASS, "stk_bridge.backends and stk_bridge.exceptions import"


@check("Connect protocol round-trip (offline)")
def check_protocol():
    from stk_bridge.backends import connect_protocol as proto
    from stk_bridge.backends.base import StkTime

    original = "1 Jun 2024 12:34:56.789"
    parsed = proto.parse_utcg(original)
    formatted = proto.format_utcg(parsed)
    if formatted != original:
        return FAIL, f"UTCG round-trip mismatch: {original!r} -> {formatted!r}"

    reader = proto.ConnectResponseReader(expects_data=False)
    reader.feed("ACK\n")
    if not reader.result().ack:
        return FAIL, "ACK not recognised by the response reader"

    reader = proto.ConnectResponseReader(expects_data=True)
    reader.feed("NACK bad command\n")
    result = reader.result()
    if result.ack or not reader.is_complete:
        return FAIL, "NACK did not terminate a data-expecting read"

    return PASS, f"UTCG round-trip and ACK/NACK framing OK ({original})"


# ---------------------------------------------------------------------------
# Live STK checks
# ---------------------------------------------------------------------------

def run_live_checks(backend_type: str, host: str, port: int) -> None:
    from stk_bridge.backends import create_backend
    from stk_bridge.backends.base import DataProviderQuery

    config = {"backend": {"type": backend_type,
                          "connect": {"host": host, "port": port}}}

    try:
        backend = create_backend(config)
    except Exception as exc:
        record("Backend construction", FAIL, f"{exc}")
        return

    try:
        backend.connect()
    except Exception as exc:
        record(f"Connect to STK via '{backend_type}'", FAIL, str(exc))
        return
    record(f"Connect to STK via '{backend_type}'", PASS,
           f"STK version: {backend.info.stk_version}")

    caps = backend.capabilities
    lines = []
    for attribute in ("integration", "astrogator", "realtime_propagator",
                      "coverage", "communications", "graphics",
                      "async_connect", "numpy_bulk_extraction"):
        value = getattr(caps, attribute)
        note = caps.unlicensed_detail.get(attribute, "")
        lines.append(f"{attribute:24s} {'yes' if value else 'no ':3s} {note}")
    record("Licensed capabilities", PASS, "\n".join(lines))

    if not caps.realtime_propagator:
        record("Real-time propagator", FAIL,
               "Required for the live control path (SetPosition). Without "
               "it only the batch StkExternal path is available.")

    try:
        backend.new_scenario("StkBridgeEnvCheck")
        record("Create scenario", PASS, "scenario 'StkBridgeEnvCheck' created")
    except Exception as exc:
        record("Create scenario", FAIL, str(exc))
        backend.disconnect()
        return

    satellite = ""
    try:
        satellite = backend.create_object("Satellite", "SSS1EnvCheck")
        record("Create satellite", PASS, satellite)
    except Exception as exc:
        record("Create satellite", FAIL, str(exc))

    if satellite:
        # A satellite created via Children.New has a propagator and default
        # elements but NO EPHEMERIS until propagated -- and every data
        # provider on it then returns zero rows while raising nothing.
        # Observed on a live STK 12.6 install. Propagate before querying.
        propagate = getattr(backend, "propagate", None)
        if propagate is not None:
            try:
                ran = propagate(satellite)
                record("Propagate satellite (generate ephemeris)",
                       PASS if ran else WARN,
                       "Propagate() ran" if ran else
                       "Propagate() unavailable on this propagator type")
            except Exception as exc:
                record("Propagate satellite (generate ephemeris)", FAIL,
                       str(exc))

        try:
            epoch = backend.get_scenario_epoch()
            data = backend.query_data_provider(DataProviderQuery(
                object_path=satellite,
                provider_name="Cartesian Position",
                group_element="ICRF",
                start=epoch, stop=epoch,
            ))
            columns = ", ".join(data.columns)
            sample = {k: (v[0] if v else None)
                      for k, v in list(data.columns.items())[:4]}

            # An empty result is a FAILURE, not a pass. The query
            # returning without raising says nothing about whether it
            # returned data, and empty state data feeds an empty control
            # loop. This check previously reported 'ok' for zero columns,
            # which is exactly the kind of false pass a checker must not
            # produce.
            if not data.columns or data.row_count == 0:
                record("Data provider: Cartesian Position/ICRF", FAIL,
                       f"query succeeded but returned NO DATA "
                       f"({len(data.columns)} columns, {data.row_count} "
                       f"rows).\nLikely causes: the object has no ephemeris "
                       f"(propagate it first), the epoch is outside its "
                       f"ephemeris interval, or the group element 'ICRF' is "
                       f"wrong for this provider.\nThe entire sensing path "
                       f"depends on this returning data.")
            else:
                record("Data provider: Cartesian Position/ICRF", PASS,
                       f"columns: {columns}\nfirst row: {sample}")

            magnitude = _magnitude(data)
            if magnitude is None:
                record("Units are SI (metres)", FAIL,
                       "could not extract x/y/z from the result, so units "
                       "could not be verified. Distance correctness is "
                       "UNCHECKED.")
            elif 6.0e6 < magnitude < 5.0e8:
                record("Units are SI (metres)", PASS,
                       f"|r| = {magnitude:.1f} m -- consistent with metres")
            elif 6.0e3 < magnitude < 5.0e5:
                record("Units are SI (metres)", FAIL,
                       f"|r| = {magnitude:.1f} -- this looks like "
                       f"KILOMETRES. configure_units() did not take "
                       f"effect; every distance downstream will be wrong "
                       f"by 1000x.")
            else:
                record("Units are SI (metres)", WARN,
                       f"|r| = {magnitude:.3g}, which matches neither "
                       f"metres nor kilometres for an Earth orbit")
        except Exception as exc:
            record("Data provider: Cartesian Position/ICRF", FAIL, str(exc))

        # Enumerate what this install actually offers. Printed before any
        # propagator change, because switching to RealTime alters what is
        # queryable. This inventory is the authoritative reference for
        # fixing a FieldSpec that stops matching after an STK upgrade.
        try:
            providers = backend.list_data_providers(satellite)
            if providers:
                interesting = {
                    name: members for name, members in providers.items()
                    if any(token in name.lower() for token in
                           ("position", "velocity", "quaternion", "attitude",
                            "angular", "lla", "lighting", "eclipse"))
                }
                lines = [f"{len(providers)} providers total; "
                         f"{len(interesting)} relevant to the ADCS twin:"]
                for name in sorted(interesting):
                    members = interesting[name]
                    lines.append(f"  {name}" +
                                 (f"  [{', '.join(members[:6])}]"
                                  if members else ""))
                record("Data provider inventory", PASS, "\n".join(lines))
            else:
                record("Data provider inventory", WARN,
                       "backend could not enumerate data providers")
        except Exception as exc:
            record("Data provider inventory", WARN, str(exc))

        # NOTE: everything below MUTATES the satellite -- switching to the
        # RealTime propagator DISCARDS its analytic ephemeris, and the
        # attitude samples consume timestamps. Convention validation
        # therefore runs on its own dedicated object (see
        # ConventionValidator), not on this one.
        try:
            from stk_bridge.backends.base import PropagatorType, StkTime

            backend.set_propagator(satellite, PropagatorType.REALTIME)
            record("Set Real-time propagator", PASS, "")
            epoch = backend.get_scenario_epoch()
            backend.set_position(satellite, epoch + 1.0, "ICRF",
                                 (7.0e6, 0.0, 0.0))
            record("SetPosition (real-time orbit injection)", PASS,
                   "one sample accepted")
        except Exception as exc:
            record("Set Real-time propagator / SetPosition", FAIL, str(exc))

        try:
            from stk_bridge.backends.base import StkTime

            backend.enable_realtime_attitude(satellite)
            record("SetAttitude Standard (real-time attitude)", PASS,
                   "real-time attitude profile activated")
            epoch = backend.get_scenario_epoch()
            backend.add_attitude_quaternion(satellite, epoch + 1.0,
                                            (0.0, 0.0, 0.0, 1.0))
            record("AddAttitude CBFQuat", PASS, "one sample accepted")
            backend.add_attitude_angular_velocity(satellite, epoch + 2.0,
                                                  (0.0, 0.0, 0.01))
            record("AddAttitude AngVels", PASS, "one sample accepted")
        except Exception as exc:
            record("Real-time attitude path", FAIL,
                   str(exc) + "\nThis is the primary ADCS control path. If it "
                   "fails, re-check the SetAttitude/AddAttitude command "
                   "spellings against your STK 12.6 Connect documentation.")

    # Phase 3: run the full convention self-test suite. This is the same
    # code path bridge_node uses at startup, so passing here means the
    # bridge will start; failing here means it would refuse to.
    if satellite:
        try:
            from stk_bridge.core.validation import ConventionValidator

            # No object path passed: the validator creates, propagates,
            # and disposes of its own satellite, so it is unaffected by
            # the mutations performed above.
            report = ConventionValidator(backend).validate()
            for result in report.results:
                if result.skipped:
                    status = WARN
                elif result.passed:
                    status = PASS
                elif result.fatal:
                    status = FAIL
                else:
                    status = WARN
                record(f"validation: {result.name}", status, result.detail)
            record("Angular rate scale", PASS if
                   report.angle_rate_scale.measured else WARN,
                   report.angle_rate_scale.source)
        except Exception as exc:
            record("Convention validation", FAIL,
                   f"{type(exc).__name__}: {exc}")

    try:
        backend.close_scenario()
    except Exception:
        pass
    backend.disconnect()


def _magnitude(data) -> Optional[float]:
    import math

    values = []
    for key in ("x", "y", "z"):
        column = data.columns.get(key)
        if not column or not isinstance(column[0], float):
            return None
        values.append(column[0])
    return math.sqrt(sum(v * v for v in values))


# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", default=None,
                        choices=["com", "engine", "connect", "mock"],
                        help="also run live checks against this backend")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=5001)
    args = parser.parse_args()

    print("stk_bridge environment check")
    print("=" * 64)

    check_python()
    check_platform()
    check_agi_import()
    check_wheel_present()
    try:
        check_bridge_import()
        check_protocol()
    except Exception:
        traceback.print_exc()
        return 2

    if args.backend:
        print("-" * 64)
        print(f"Live checks against backend '{args.backend}'")
        print("-" * 64)
        run_live_checks(args.backend, args.host, args.port)

    print("=" * 64)
    failures = [r for r in _RESULTS if r[1] == FAIL]
    warnings = [r for r in _RESULTS if r[1] == WARN]
    print(f"{len(_RESULTS)} checks: "
          f"{len(_RESULTS) - len(failures) - len(warnings)} passed, "
          f"{len(warnings)} warnings, {len(failures)} failed")
    for name, _, _ in failures:
        print(f"  FAILED: {name}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
