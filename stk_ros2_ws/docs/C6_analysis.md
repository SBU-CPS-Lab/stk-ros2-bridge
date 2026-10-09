# C6 — Analysis and figures

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
        |                              Figure, one function per E1-E6
        v
run_acav_campaign report            -- writes JSON + figures/ next
                                        to the records it read
```

## 2. Running it

```
run_acav_campaign report --experiment E2 --output-dir results/E2
```

Prints the JSON report (same as C5), and — unless `--no-figures` is
given — writes `results/E2/figures/E2_*.{pdf,svg,png}`. Needs no STK,
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
| E4 | Gain surface | Heatmap, one sequential hue (blue). Missing gain combinations are left blank (NaN), never interpolated — an interpolated cell would show a value the vehicle was never actually flown at. |
| E5 | Replay reproduction | A status-colored dot per replayed seed. No axis to sum or compare in magnitude, only per-seed pass/fail. |
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

## 6. What has actually been executed

Every figure function is smoke-tested against synthetic `RunRecord`s in
`src/acav/test/test_report_figures.py`, including a regression test for
a real bug the smoke-testing process itself caught: an early version of
`e6_latency_figure` read `mean_sense_ms`/`mean_actuate_ms` while
`campaign.latency_breakdown` produces `mean_sense_latency_ms`/
`mean_actuate_latency_ms`. The mismatched keys fell back to a default of
`0.0` silently — the figure rendered without error, just with two of
three phases missing from the bar. It was caught by actually reading
the rendered PNG, not by the smoke test alone, which is why that
specific check now asserts on bar widths rather than only "did this
raise".

`E1`/`E2`/`E3`/`E5`/`E6`'s full `run` → `report` → figures pipeline has
been run end to end against an injected launcher (not live STK) and
produces `figures/*.{pdf,svg,png}` correctly, including the E5
replay-pairing logic that matches a `<run_id>-replay` record back to its
original by construction (see `RunSpec.run_id`, not by re-matching
faults, which would also pair two *unrelated* runs that happened to
share one).

**One live figure exists: E6**, from the campaign of 2026-10-05 (five
reportable COM runs; measured total 658.1 ms per step, of which sense
621.5 ms, actuate 18.9 ms, barrier wait 13.0 ms). Its title shows the
measured end-to-end total; the component sum is disclosed separately
because the two are timed over different boundaries. The figures for
E1–E5 do not exist yet. Results and their caveats:
`09_live_runs_and_results.md`.
