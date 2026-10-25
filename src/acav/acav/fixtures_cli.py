"""Regenerate the cross-language twin fixtures.

    python scripts/export_twin_fixtures.py
    ros2 run acav export_twin_fixtures --output <path>

Writes ``config/twin_fixtures.json``, which the MATLAB twin's self-test
replays. Run it after any change to ``acav/twin_reference.py`` or to the
scenario YAML, and review the diff: a change in an expected value is a
change in what the twin is required to do, and should be as visible as
a change in the code that produced it.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional

from acav.spec import AcavSpec
from acav.twin_fixtures import write_fixtures


def default_output() -> str:
    """The source tree when there is one, the installed share otherwise.

    Writing belongs in the source tree -- the fixtures are checked in so
    a change to them is a reviewable diff. Reading has to work from an
    installed package too, because ``ros2 run acav export_twin_fixtures
    --check`` is the form a CI step would use.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    source = os.path.join(os.path.dirname(here), "config",
                          "twin_fixtures.json")
    if os.path.isdir(os.path.dirname(source)):
        return source
    try:
        from ament_index_python.packages import (
            get_package_share_directory,
        )
        share = get_package_share_directory("acav")
    except Exception:
        return source
    return os.path.join(share, "config", "twin_fixtures.json")


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None,
                        help="path to acav_scenario.yaml")
    parser.add_argument("--output", default=None,
                        help="where to write the fixtures JSON")
    parser.add_argument("--check", action="store_true",
                        help="fail if the file on disk is out of date "
                             "instead of rewriting it")
    args = parser.parse_args(argv)

    spec = AcavSpec.load(args.config)
    output = args.output or default_output()

    if args.check:
        import json
        from acav.twin_fixtures import build_fixtures, compare_fixtures
        if not os.path.isfile(output):
            print(f"{output} does not exist; run without --check",
                  file=sys.stderr)
            return 1
        with open(output, "r", encoding="utf-8") as handle:
            on_disk = json.load(handle)
        differences = compare_fixtures(on_disk, build_fixtures(spec))
        if differences:
            print(f"{output} is out of date with acav/twin_reference.py:",
                  file=sys.stderr)
            for line in differences[:20]:
                print(f"  {line}", file=sys.stderr)
            if len(differences) > 20:
                print(f"  ... and {len(differences) - 20} more",
                      file=sys.stderr)
            print("Regenerate it and review the diff -- a change in an "
                  "expected value is a change in what the twin must do.",
                  file=sys.stderr)
            return 1
        print(f"{output} is up to date")
        return 0

    payload = write_fixtures(output, spec)
    sections = [k for k, v in payload.items() if isinstance(v, list)]
    print(f"wrote {output}")
    for section in sections:
        print(f"  {section}: {len(payload[section])} case(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
