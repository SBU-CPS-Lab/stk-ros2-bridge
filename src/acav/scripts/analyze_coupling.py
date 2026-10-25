"""Timing and coupling analysis of the runs in a results folder.

    python scripts/analyze_coupling.py results/E3 [more ...]

Read-only: uses each run's record and its <run>.logs folder only (no
STK, no ROS); see acav/coupling_analysis.py for every quantity. Writes
one <run>.coupling.json per run and a coupling.csv to <folder>/_coupling
(or --out), and prints a summary table.
"""

import argparse
import os
import sys


def _bootstrap_source_checkout() -> None:
    scripts = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(scripts)          # src/acav
    workspace_src = os.path.dirname(package_root)    # src/
    for candidate in (package_root,
                      os.path.join(workspace_src, "stk_bridge")):
        if os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)


_bootstrap_source_checkout()

from acav.coupling_analysis import analyze_folder  # noqa: E402

COLUMNS = (("run_id", "%-36s"), ("execution_mode", "%-9s"),
           ("real_time_factor", "%4s"), ("captured", "%3s"),
           ("commands", "%5s"), ("control_rate_hz", "%6.2f"),
           ("gap_p50_s", "%6.3f"), ("gap_max_s", "%6.3f"),
           ("gaps_over_0_6", "%4s"), ("max_n", "%3s"),
           ("offset_affected", "%5s"), ("write_lag_p50_s", "%6.3f"),
           ("com_stalls_over_300ms", "%4s"),
           ("readback_after_newest", "%5s"),
           ("readback_after_median_deg", "%7.3f"))


def _cell(fmt, value):
    if value is None:
        return "-"
    try:
        return fmt % value
    except TypeError:
        return str(value)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("folders", nargs="+", metavar="FOLDER")
    parser.add_argument("--out", default="",
                        help="output folder (default: <folder>/_coupling)")
    parser.add_argument("--window-s", type=float, default=None,
                        help="scenario window for the control rate "
                             "(default: the scenario specification's)")
    args = parser.parse_args(argv)
    window = args.window_s
    if window is None:
        from acav.spec import AcavSpec
        window = float(AcavSpec.load(None).window_duration_s)
    print("  ".join(name for name, _ in COLUMNS))
    for folder in args.folders:
        out = args.out or os.path.join(folder, "_coupling")
        for row in analyze_folder(folder, out, window):
            print("  ".join(_cell(fmt, row.get(name))
                            for name, fmt in COLUMNS))
        print(f"# wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
