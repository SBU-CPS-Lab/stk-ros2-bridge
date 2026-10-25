#!/usr/bin/env python3
"""Measure achievable sense/actuate rates against a live backend.

Every rate figure elsewhere in this project (the "1-20 Hz" ballpark in
docs/bridge/architecture.md, the fast/medium/slow defaults in
sensing_pipeline.yaml) is a general expectation, not a measurement of
YOUR scenario. Object count, data-provider complexity, and network
topology (T1 vs T2/T3) all change the real number. Run this before
committing to a control-loop rate.

    python scripts/benchmark_rates.py --backend com --iterations 200
    python scripts/benchmark_rates.py --backend connect --host 10.0.0.5

Reports p50/p95/p99 latency for: a single Cartesian Position query, a
single AddAttitude CBFQuat command, and a combined sense+actuate cycle
(what the bridge's step loop actually does per satellite per step).
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def _percentiles(samples_ms):
    ordered = sorted(samples_ms)
    n = len(ordered)

    def pct(p):
        if n == 0:
            return float("nan")
        idx = min(n - 1, int(round(p / 100.0 * (n - 1))))
        return ordered[idx]

    return {
        "min": ordered[0] if n else float("nan"),
        "p50": pct(50), "p95": pct(95), "p99": pct(99),
        "max": ordered[-1] if n else float("nan"),
        "mean": statistics.fmean(ordered) if n else float("nan"),
    }


def _report(name: str, samples_ms) -> None:
    stats = _percentiles(samples_ms)
    print(f"{name}:")
    print(f"  min={stats['min']:.2f}ms  p50={stats['p50']:.2f}ms  "
          f"p95={stats['p95']:.2f}ms  p99={stats['p99']:.2f}ms  "
          f"max={stats['max']:.2f}ms  mean={stats['mean']:.2f}ms")
    if stats["p95"] > 0:
        print(f"  implied sustainable rate at p95: "
              f"{1000.0 / stats['p95']:.1f} Hz")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", default="mock",
                        choices=["com", "engine", "connect", "mock"])
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--satellite", default="SSS-1")
    args = parser.parse_args()

    from stk_bridge.backends import create_backend
    from stk_bridge.backends.base import DataProviderQuery, PropagatorType
    from stk_bridge.core.validation import ConventionValidator

    config = {"backend": {"type": args.backend,
                          "connect": {"host": args.host, "port": args.port}}}
    backend = create_backend(config)
    backend.connect()
    backend.new_scenario("StkBridgeBenchmark")
    sat = backend.create_object("Satellite", args.satellite)
    propagate = getattr(backend, "propagate", None)
    if propagate is not None:
        propagate(sat)

    report = ConventionValidator(backend).validate()
    if not report.ok:
        print("WARNING: convention validation failed; benchmark numbers "
              "may not reflect a correctly-configured bridge. Run "
              "validate_conventions.py first.", file=sys.stderr)

    backend.set_propagator(sat, PropagatorType.REALTIME)
    backend.enable_realtime_attitude(sat)
    epoch = backend.get_scenario_epoch()

    print(f"Benchmarking '{args.backend}' backend, {args.iterations} "
          f"iterations, satellite={sat}\n")

    # -- position injection + immediate query (the actuate+sense cycle) --
    query_samples = []
    command_samples = []
    combined_samples = []

    for i in range(args.iterations):
        when = epoch + (i + 1) * 0.5

        t0 = time.perf_counter()
        backend.set_position(sat, when, "ICRF",
                             (7.0e6 + i, 0.0, 0.0))
        command_samples.append((time.perf_counter() - t0) * 1000.0)

        t1 = time.perf_counter()
        backend.query_data_provider(DataProviderQuery(
            object_path=sat, provider_name="Cartesian Position",
            group_element="ICRF", start=when, stop=when,
        ))
        query_samples.append((time.perf_counter() - t1) * 1000.0)

        combined_samples.append(command_samples[-1] + query_samples[-1])

    _report("SetPosition (command only)", command_samples)
    print()
    _report("Cartesian Position query (sense only)", query_samples)
    print()
    _report("combined sense+actuate cycle (one satellite, one step)",
           combined_samples)

    print(
        "\nNote: a step with N satellites and multiple channels per "
        "satellite (orbit, attitude, rate) multiplies the 'sense' side "
        "roughly linearly. Re-run with --satellite pointed at your actual "
        "scenario object count in mind, and treat this figure as an "
        "upper bound on control-loop rate, not a target."
    )

    backend.close_scenario()
    backend.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
