"""Command-line entry point for verifying a built ACAV scenario.

    python scripts/verify_acav_scenario.py --backend com
    python scripts/verify_acav_scenario.py --backend com --build
    python scripts/verify_acav_scenario.py --backend com --load C:\\acav.sc

Written in the same spirit as ``stk_bridge/scripts/check_environment.py``:
every check is independent and non-fatal, they all run, and the summary
says what is wrong rather than stopping at the first problem. Knowing
that the cone angle *and* the terrain mask are wrong is worth more than
one fix-and-rerun cycle per finding, and against a live STK each cycle
costs minutes.

What the exit code means
------------------------
0  every check passed or warned. Warnings are real: they mean the study
   runs in a degraded form that must be described as such (an
   elevation constraint instead of terrain, say). Read them.
1  at least one check FAILED. The scenario would silently measure
   something other than what was designed. Do not run experiments
   against it.
2  the backend could not be reached, or the spec could not be loaded --
   nothing was measured either way.
"""

from __future__ import annotations

import logging
import sys
from typing import Optional

from stk_bridge.exceptions import StkBridgeError

from acav.build_cli import (
    MOCK_BANNER,
    build_parser,
    load_spec,
    open_backend,
    write_json,
)
from acav.scenario_builder import AcavScenarioBuilder
from acav.scenario_verifier import AcavScenarioVerifier


def main(argv: Optional[list] = None) -> int:
    parser = build_parser(__doc__ or "Verify the ACAV STK scenario.")
    parser.add_argument("--build", action="store_true",
                        help="build the scenario first, in this process, "
                             "instead of verifying whatever is loaded")
    parser.add_argument("--load", default=None, metavar="PATH",
                        help="load this .sc scenario file before verifying")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)-7s %(name)s: %(message)s")

    spec = load_spec(args.config)
    backend = open_backend(args)

    print("ACAV scenario verification")
    print("=" * 64)
    print(f"backend  {args.backend}   scenario  {spec.name}")
    if args.backend == "mock":
        print()
        print(MOCK_BANNER)
    print("=" * 64)

    build_report = None
    try:
        if args.load:
            try:
                backend.load_scenario(args.load)
            except StkBridgeError as exc:
                print(f"could not load {args.load}: {exc}", file=sys.stderr)
                return 2

        if args.build:
            build_report = AcavScenarioBuilder(backend, spec).build()
            print(build_report.summary())
            print("-" * 64)
            if not build_report.ok:
                # Verifying a scenario the builder already knows is
                # broken produces a wall of consequential failures that
                # bury the one that caused them.
                print("the build reported errors; not verifying a scenario "
                      "that is already known to be wrong.")
                return 1

        report = AcavScenarioVerifier(backend, spec, build_report).verify()
        print(report.summary())
        write_json(args.json, report.to_dict())
    finally:
        backend.disconnect()

    return 0 if report.ok else 1


if __name__ == "__main__":
    sys.exit(main())
