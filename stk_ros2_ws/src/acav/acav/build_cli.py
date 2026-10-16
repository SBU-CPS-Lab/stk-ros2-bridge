"""Command-line entry point for building the ACAV STK scenario.

    python scripts/build_acav_scenario.py --backend com
    python scripts/build_acav_scenario.py --backend mock --verify
    ros2 run acav build_acav_scenario --backend com --save C:\\acav.sc

Building and verifying are separate commands because they answer
different questions -- "did the commands run" and "did they do what was
intended" -- and because the second is the one worth re-running after a
manual edit in the STK GUI. ``--verify`` runs both in one process for
convenience; it is exactly equivalent to running the two scripts in
sequence against the same STK session.

Exit codes: 0 the scenario was built (and verified, if asked);
1 the build or the verification reported a hard error; 2 the backend
could not be reached at all.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from typing import Optional

from stk_bridge.backends import create_backend
from stk_bridge.exceptions import StkBridgeError

from acav.scenario_builder import AcavScenarioBuilder
from acav.scenario_verifier import AcavScenarioVerifier
from acav.spec import AcavSpec

MOCK_BANNER = (
    "NOTE: the mock backend has no STK in it. It exercises the builder's "
    "arithmetic, paths and reporting; it models no sensor cone, no terrain "
    "and no real access geometry. No number produced here may appear in "
    "the paper."
)


def build_parser(description: str) -> argparse.ArgumentParser:
    """Argument parser shared with verify_acav_scenario."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--backend", default="com",
                        choices=["com", "engine", "connect", "mock"],
                        help="backend to drive STK through (default: com)")
    parser.add_argument("--host", default="localhost",
                        help="Connect backend host")
    parser.add_argument("--port", type=int, default=5001,
                        help="Connect backend port")
    parser.add_argument("--scenario", "--config", dest="config",
                        default=None, metavar="PATH",
                        help="scenario YAML (default: the reference "
                             "acav_scenario.yaml shipped with this package; "
                             "--config is the older name)")
    parser.add_argument("--json", default=None, metavar="PATH",
                        help="also write the report as JSON, for C6")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="log every STK command that is attempted")
    return parser


def open_backend(args: argparse.Namespace):
    """Construct and connect a backend, or raise SystemExit(2)."""
    config = {"backend": {"type": args.backend,
                          "connect": {"host": args.host, "port": args.port}}}
    try:
        backend = create_backend(config)
        backend.connect()
    except StkBridgeError as exc:
        print(f"could not reach STK via the '{args.backend}' backend: {exc}",
              file=sys.stderr)
        raise SystemExit(2) from exc
    return backend


def load_spec(path: Optional[str]) -> AcavSpec:
    """Load the scenario spec, or raise SystemExit(2)."""
    try:
        return AcavSpec.load(path)
    except (FileNotFoundError, ValueError, KeyError) as exc:
        print(f"could not load the ACAV scenario spec: {exc}",
              file=sys.stderr)
        raise SystemExit(2) from exc


def write_json(path: Optional[str], payload) -> None:
    if not path:
        return
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, default=str)
    print(f"\nwrote {path}")


def main(argv: Optional[list] = None) -> int:
    parser = build_parser(__doc__ or "Build the ACAV STK scenario.")
    parser.add_argument("--save", default=None, metavar="PATH",
                        help="save the scenario to this .sc path afterwards")
    parser.add_argument("--verify", action="store_true",
                        help="run verify_acav_scenario's checks in the same "
                             "process once the build finishes")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)-7s %(name)s: %(message)s")

    spec = load_spec(args.config)
    print(spec.summary())
    print()

    backend = open_backend(args)
    if args.backend == "mock":
        print(MOCK_BANNER)
        print()

    exit_code = 0
    try:
        report = AcavScenarioBuilder(backend, spec).build()
        print(report.summary())
        payload = {"build": _build_payload(report)}
        if not report.ok:
            exit_code = 1

        if args.verify and report.ok:
            print()
            print("-" * 64)
            verification = AcavScenarioVerifier(backend, spec,
                                                report).verify()
            print(verification.summary())
            payload["verification"] = verification.to_dict()
            if not verification.ok:
                exit_code = 1

        if args.save:
            try:
                backend.save_scenario(args.save)
                print(f"\nscenario saved to {args.save}")
            except StkBridgeError as exc:
                print(f"\ncould not save the scenario: {exc}",
                      file=sys.stderr)
                exit_code = 1

        write_json(args.json, payload)
    finally:
        backend.disconnect()

    return exit_code


def _build_payload(report) -> dict:
    return {
        "scenario": report.scenario_name,
        "ok": report.ok,
        "satellite_path": report.satellite_path,
        "propagator_applied": report.propagator_applied,
        "orbit_method": report.orbit_method,
        "sensor_path": report.sensor_path,
        "sensor_pattern_method": report.sensor_pattern_method,
        "sensor_pointing_method": report.sensor_pointing_method,
        "terrain_mask_method": report.terrain_mask_method,
        "targets": [
            {
                "id": placement.target_id,
                "path": placement.object_path,
                "latitude_deg": placement.point.latitude_deg,
                "longitude_deg": placement.point.longitude_deg,
                "requested_cross_track_km":
                    placement.requested_cross_track_km,
                "actual_cross_track_km": placement.actual_cross_track_km,
                "side": placement.side,
            }
            for placement in report.targets
        ],
        "warnings": list(report.warnings),
        "errors": list(report.errors),
    }


if __name__ == "__main__":
    sys.exit(main())
