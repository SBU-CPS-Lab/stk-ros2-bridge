"""Print the validated ACAV scenario summary.

    ros2 run acav describe_scenario
    python -m acav.spec_cli --spec path/to/acav_scenario.yaml

Run this after editing acav_scenario.yaml: it applies every
cross-check in AcavSpec.validate() and prints the per-target
feasibility table, so a spec error surfaces here rather than as a
confusing result three phases later.
"""

from __future__ import annotations

import argparse
import sys


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", default=None,
                        help="path to acav_scenario.yaml")
    args = parser.parse_args(argv)

    from acav.spec import AcavSpec

    try:
        spec = AcavSpec.load(args.spec)
    except (ValueError, KeyError, FileNotFoundError) as exc:
        print(f"scenario spec is invalid:\n{exc}", file=sys.stderr)
        return 1

    print(spec.summary())
    return 0


if __name__ == "__main__":
    sys.exit(main())
