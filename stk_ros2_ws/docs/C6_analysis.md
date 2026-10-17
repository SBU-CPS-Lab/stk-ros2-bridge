# C6: Analysis and figures

Turns a campaign directory (C5's `RunRecord` files) into the numbers and
figures the paper needs, via one command: `run_acav_campaign report`.

## 1. Why this is thin

Every real computation — capture rate, coverage rate, the determinism
signature, the gain surface, the latency breakdown — already lives in
`acav/metrics.py` and `acav/campaign.py`, built and tested during C5.
C6 does not re-derive any of them. `acav/report_figures.py` takes the
*same* aggregation dict `run_acav_campaign report`'s JSON output prints
and lays it out as a figure; nothing here computes a number a second
way. Two independently-written computations of one quantity silently
disagreeing is the exact failure this project keeps citing (the slew
model, `docs/C1_case_study_spec.md`) — C6 does not add a fourth place
that could happen.

```
acav/metrics.py, acav/campaign.py   -- the numbers (built in C5)
        |
acav/campaign_cli.py: analyse()     -- picks the right aggregation
        |                              per experiment, prints JSON
        v
acav/report_figures.py              -- lays the SAME dict out as a
        |                              Figure, one function per experiment
        v
run_acav_campaign report            -- writes JSON + figures/ next
                                        to the records it read
```

## 2. Running it

```
run_acav_campaign report --experiment E2 --output-dir results/<E2 folder>
```

Prints the JSON report (same as C5), and — unless `--no-figures` is
given — writes `<E2 folder>/figures/E2_*.{pdf,svg,png}`. Needs no STK,
no MATLAB, no ROS: this is meant to run wherever the analysis actually
happens, which per this project's own environment notes is this dev
machine, not the Windows deploy box.

**matplotlib is not a hard dependency** of the `acav` package —
`pip install 'acav[figures]'` if it is not already present.
`run_acav_campaign report` degrades to JSON-only output with a clear
message if it is missing, rather than failing the whole verb; the
report a colleague actually asked for is the JSON, and a plotting
library being absent on their machine should not cost them that.

## 3. Style, and why it is not mermaid

Every figure goes through `acav/report_style.py`, which is a validated
categorical/sequential/status palette (colorblind-safe adjacent pairs,
checked contrast) applied to matplotlib's rcParams once, plus a couple
of fixed color-role tables (`SCHEDULER_COLOR`, `MODE_COLOR`,
`STATUS_COLOR`) so "dynamics-aware" is the same blue and "captured" is
the same green across every figure in a report, not whatever the next
plot's color cycle happens to assign.

**Auto-laid-out diagram tools (mermaid and similar) are not used for
these, or for any figure or block diagram in this project.** They
produce a recognisable, generic look, and for a heatmap or a stacked
bar the auto-layout algorithms that make sense for a flowchart do not
apply at all — there is no "layout" to solve, only mark choice and
color, which is what `report_style.py`/`report_figures.py` handle by
hand. Any architecture or pipeline diagram in this project's docs (see
`docs/00_architecture.md`) is a hand-authored SVG for the same reason.

## 4. One figure per experiment's question

| Experiment | Figure | Form, and why |
|---|---|---|
| E1 | Control vs treatment capture rate | Two adjacent bars, the drop annotated between their tops. Not spread across a category axis, which would invite reading two points as a trend. |
| E2 | Capture rate vs coverage rate | Grouped bar, same 0-1 scale. **Never** a dual-axis chart — the two metrics have different denominators (`acav/metrics.py`), so two y-scales would let unrelated ranges appear to correlate. |
| E2 | Outcome breakdown | Stacked horizontal bar, status colors. `CaptureOutcome` *is* a status (captured / still slewing / no access / SAFE), so status color, not a generic categorical cycle. |
| E3 | Determinism | Two panels: distinct-signature count (a status fact — deterministic or not) and pointing-error spread (a magnitude, degrees) — different scales, so two panels rather than one dual-axis chart. |
| E5 (E3 fault replay) | Replay reproduction | A status-colored dot per replayed seed. No axis to sum or compare in magnitude, only per-seed pass/fail. |
| E6 | Latency breakdown | One stacked horizontal bar: sense + actuate + barrier-wait. Three parts of one step, so a stack — a grouped bar would wrongly suggest the phases are alternatives rather than components of one total. |

E1's comparison is **not** grouped by scheduler the way E2's is:
`acav.campaign.convention_comparison` splits by whether the attitude
read-back provider was forced (`Provenance.attitude_provider`), because
E1 holds the scheduler fixed and varies the provider instead — grouping
by scheduler the way `compare_schedulers` does would collapse E1's two
runs into one group and hide the entire comparison. That field is
derived from what the bridge reported (`BridgeStatus.attitude_provider`),
not from the launch command.

The E1 figure draws a 0% arm at its true height with a marker on the
baseline (an earlier version floored it at 1.2% so it stayed visible,
which misstated it), labels each bar with captured/attempted and the run
count, gives the drop in percentage points, and names the treatment as a
guard ablation in the title.

## 5. What E6 actually measures

`stk_bridge/core/step_timing.py` times three phases of the bridge's own
step loop — sense, actuate, the lockstep barrier wait — as a rolling
mean over the second since the last `BridgeStatus` publish (always
1 Hz). `acav/capture_tracking.py::LatencyAggregate` folds those
per-window means into one run-level figure, **weighted by how many
step-loop iterations fed each window** — an unweighted average would
let a thin first window (one or two steps, right after the run starts)
count as much as a full second of steady state.

This is deliberately **not** the same number as
`RunResult.mean_step_latency_ms`, which is the *scheduler's* own
tick-to-tick wall clock. In lockstep that already spans a whole
sense/actuate/barrier cycle from the scheduler's vantage, but it also
includes the scheduler's own scheduling jitter and whatever the twin
spent between the ack and the next tick — neither of which a bridge-side
phase timer can see. The two are not expected to sum to the same total,
and both are kept, labelled for what they actually are.

**A latency number from the mock backend is not reportable.** The same
rule as every other metric here (C1 section 8): `latency_breakdown`
excludes any record whose backend was not `com`/`engine`/`connect`
before averaging, because a mock round trip says nothing about the
deploy machine's real STK latency — the entire quantity E6 exists to
report.

## 6. What has been executed

Every figure function is smoke-tested against synthetic records
(`src/acav/test/test_report_figures.py`), including a regression test
for a real bug: an early `e6_latency_figure` read keys the aggregation
does not produce and silently drew two of three phases as zero; the test
now asserts on bar widths. The `run` -> `report` -> figures pipeline has
been run end to end against an injected launcher for every experiment.

`report` also refuses directories it must not pool
([C5_experiment_harness.md](C5_experiment_harness.md) section 7) before
it computes anything. The live numbers, and which records they come
from, are in [09_live_runs_and_results.md](09_live_runs_and_results.md).

## 7. Coupling analysis: `analyze_coupling`

For E3, and for any diagnosis of a free-run run, the records alone do
not say why a run behaved as it did. `scripts/analyze_coupling.py`
reads each run's record and its `<run>.logs` folder (the topic captures,
`bridge.log`, `summary.json`), with no STK and no ROS, and writes one
`<run>.coupling.json` per run and a `coupling.csv` to
`<folder>/_coupling` (or `--out`):

```
python src\acav\scripts\analyze_coupling.py results\E3_P9_rtf1_9fed085 [more folders]
```

What each quantity means (`acav/coupling_analysis.py`):

| Quantity | Meaning |
|---|---|
| `control_rate_hz` | Twin commands per scenario second. Lockstep gives 10 Hz. |
| Command gaps (p50, p90, max, count over 0.6 s) | How long the vehicle went without a new command, in scenario time. |
| `image_ages` | Per attempted target: the image epoch minus the stamp of the newest twin status at or before it, and that status's error. Lockstep gives 0. |
| `n_gt1_count`, `max_n` | Samples the twin integrated over more than one step, from `twin_substeps.json` (or from command spacing). |
| Command-stamp offset | The step the twin actually reached minus the stamp its command carried. |
| Write lag | When the bridge wrote a command (its next sample after the command's source sample) minus the command's stamp. |
| COM stalls over 300 ms | Bridge round trips over 300 ms once the loop is closed. |
| STK read-back | Read-backs at a written stamp, and those beyond the newest written sample with their angle to it (what STK reports past the last command). |
| Twin loop | Ready calls per wall second, the share that were holds (n = 0), and the polling rate from the twin identity. |
| Bridge samples consumed | Distinct bridge samples, and how many the twin answered with a command one step later. |
| Run conditions | Execution mode, graphics mode, real-time factor, start-gate wait, free-run pacing (cycles, overruns). |

Percentiles are `sorted[n // 2]` and `sorted[int(0.9 * n)]`.

## 8. E2 across seeded scenarios: paired analysis

Each seeded scenario is its own E2 folder (one kinematic and one
dynamics-aware run). `report --experiment E2 --output-dir <seed folder>
--include-dir <seed folder> ...` analyses them together as pairs
(`acav/paired_analysis.py`, statistics in `acav/paired_stats.py`): one
pair per scenario seed, difference = dynamics-aware minus kinematic,
incomplete pairs dropped and listed by name.

- Metrics: the records' own `coverage_rate`, `capture_rate`,
  `wheel_energy_J`, `wheel_saturation_events` (`over_promise_gap` is
  1 - `capture_rate` and not tested again).
- Per metric: the differences, median, Hodges-Lehmann estimate, exact
  two-sided Wilcoxon signed-rank p (zeros dropped; average ranks with
  the exact conditional distribution for ties), matched-pairs
  rank-biserial, a percentile bootstrap CI of the median difference
  (seed and B = 10000 written into the output), and Holm-adjusted p over
  the four metrics.
- The output prints the minimum attainable two-sided exact p for the
  number of pairs, 2^(1 - n), and flags it when above 0.05 (n <= 5):
  such a design cannot reach p < 0.05 whatever the data.
- Refusals (each names the records): different code version, twin
  identity (fingerprints and polling rate), STK start mode, execution
  mode or generator version; a record not from a seeded scenario; an
  unreportable record; two records for one seed and scheduler; plus all
  the refusals of section 6.

No paired result exists yet; see [09_live_runs_and_results.md](09_live_runs_and_results.md)
for what has been run.
