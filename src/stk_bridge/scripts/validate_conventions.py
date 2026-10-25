#!/usr/bin/env python3
"""Standalone convention validation, without the rest of check_environment.

Thin wrapper around core/validation.py for two situations
check_environment.py doesn't fit well:

* Re-checking conventions after an STK upgrade, without re-running the
  full environment/platform checks every time.
* CI or a pre-flight hook that wants a single pass/fail exit code and
  machine-parseable output.

    python scripts/validate_conventions.py --backend com
    python scripts/validate_conventions.py --backend mock --json

Exit codes: 0 = all fatal checks passed, 1 = a fatal check failed,
2 = could not connect / could not run.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", default="mock",
                        choices=["com", "engine", "connect", "mock"])
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--json", action="store_true",
                        help="machine-readable output")
    args = parser.parse_args()

    from stk_bridge.backends import create_backend
    from stk_bridge.core.validation import ConventionValidator

    config = {"backend": {"type": args.backend,
                          "connect": {"host": args.host, "port": args.port}}}
    try:
        backend = create_backend(config)
        backend.connect()
        backend.new_scenario("StkBridgeValidateCLI")
    except Exception as exc:
        if args.json:
            print(json.dumps({"ok": False, "stage": "connect",
                              "error": str(exc)}))
        else:
            print(f"could not connect via {args.backend!r} backend: {exc}",
                  file=sys.stderr)
        return 2

    try:
        report = ConventionValidator(backend).validate()
    finally:
        try:
            backend.close_scenario()
        except Exception:
            pass
        backend.disconnect()

    if args.json:
        print(json.dumps({
            "ok": report.ok,
            "attitude_provider": report.attitude_provider,
            "attitude_command_inertial": report.attitude_command_inertial,
            "angular_rate_provider": report.angular_rate_provider,
            "angle_rate_scale_factor": report.angle_rate_scale.factor,
            "angle_rate_scale_measured": report.angle_rate_scale.measured,
            "monotonicity_is_observable": report.monotonicity_is_observable,
            "results": [
                {"name": r.name, "passed": r.passed, "fatal": r.fatal,
                 "detail": r.detail}
                for r in report.results
            ],
        }, indent=2))
    else:
        print(report.summary())
        print()
        print(f"ok={report.ok}  "
              f"attitude={report.attitude_provider!r}  "
              f"rates={report.angular_rate_provider!r}  "
              f"rate_scale={report.angle_rate_scale.source}")

    return 0 if report.ok else 1


if __name__ == "__main__":
    sys.exit(main())
