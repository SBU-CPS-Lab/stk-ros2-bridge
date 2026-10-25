# The experiment harness

What turns the closed loop into experiments: the run matrices (E1, E2,
E3 with its fault replay, E6), provenance that decides whether a number
may be published, seeded faults that can be replayed exactly, a run
orchestrator that checks every step of one run, and a campaign runner
that resumes after an interruption.

## 1. Why the harness is split in two

| Module | Role |
|---|---|
| `acav/run_record.py` | Provenance + serialisation. **The contract between harness and analysis.** |
| `acav/fault_injection.py` | Seeded wheel faults (E3 fault replay) |
| `acav/campaign.py` | The run matrices, and what the results mean |
| `acav/campaign_runner.py` | Executes a matrix. Knows nothing about STK |
| `acav/campaign_cli.py` | `ros2 run acav run_acav_campaign` |

The first three are pure: no ROS, no subprocess, no sockets. The fourth
is thin wiring, and takes its launcher by injection so the whole loop —
resume, validation, aggregation — is exercised in tests here without
STK or MATLAB. This is the same division the [scheduler](scheduler.md) draws between
`scheduling.py` and `scheduler_node.py`, and for a sharper reason: a
campaign is tens of runs, each needing STK and MATLAB for 10-16
minutes ([case study](case_study.md) section 8). It is an overnight thing on one machine and it
cannot be iterated on. Everything about it that can be decided in milliseconds on
any machine is decided by tested code before the night starts.

**The harness does not know how to launch a run.** The launch topology —
bridge with the right backend and execution mode, the Simulink twin, the
scheduler, shutdown in the right order — belongs to the Windows machine
that has STK on it. `CommandLauncher` runs whatever command the operator
supplies, once per run, with the run's configuration substituted in.
Encoding a guess at that topology here would produce a supervisor that
looks finished, cannot be exercised anywhere it is written, and fails at
2 am in the middle of a campaign.

## 2. Provenance, and the one rule it enforces

The case study (section 8) commits the paper to reporting **no number the mock backend
produced**. A result file that records only outcomes puts the burden of
remembering which backend ran on whoever assembles the figures, months
later, across several hundred files. That is not a burden a person
carries reliably, so `RunRecord` carries it instead.

Three properties do the work:

- **The backend is observed, not assumed.** `backend_type` is what the
  bridge reported on `BridgeStatus`, not what the campaign intended to
  launch. A run configured for COM that silently fell back to the mock
  must not be indistinguishable from a real one.
- **Reportability is derived, never stored.** `RunRecord.reportable` is
  recomputed from the backend on every access. A stored flag can be
  edited, copied from a neighbouring file, or go stale; a derived one
  cannot. `test_run_record.py` asserts that editing the flag in the file
  changes nothing.
- **The signature is verified on read.** It is written into the file for
  readability and recomputed from the result on load. A mismatch means
  the record was hand-edited or truncated, and reading it raises rather
  than letting a corrupted run into a campaign that then reports a
  determinism result about it.

`run_acav_campaign report` exits non-zero and names the offending runs if
any record in the directory is unreportable, even when the experiment's
own claim otherwise held.

**NaN is written as `null`.** `json.dump` emits a bare `NaN` token, which
is not valid JSON and which MATLAB's `jsondecode` rejects — and MATLAB is
a reader this project uses. An unattempted target's pointing error is
legitimately NaN; reading it back as `0.0` would put a perfect score on a
target nobody pointed at.

## 3. Seeded faults, and why the fault is a value

The fault runs are E3's fault-replay part, still labelled E5 in the
campaign builder, run ids and records. Their claim is deterministic
fault *replay*, so the thing being replayed
has to be deterministic first. Three properties, each corresponding to a
way the experiment could look correct while being unreproducible:

- **The seed fully determines the fault.** `plan_fault` uses a private
  `random.Random(seed)`, never the module-level RNG. A fault drawn from
  the global generator depends on how many other draws happened first —
  reproducible only by accident, and not at all once anything else in
  the process starts using randomness.
- **The fault is resolved before the run, not sampled at injection.** It
  is written into the record, so it can be diffed between two runs and
  replayed on a machine whose RNG differs.
- **Replay reads the record, not the rule.** `build_e5_replay` rebuilds
  each run's fault from its record via `fault_from_dict`. Re-deriving it
  from the seed would make every historical result depend on the current
  sampling rule, so changing that rule would turn a genuine reproduction
  into a silent mismatch.

Seeds are also planned independently of one another, so inserting a seed
cannot renumber anyone else's fault — otherwise "re-run the seed that
failed" would reproduce a different fault than the one that failed.

### The two kinds, and which one E5 uses

| Kind | What it does | What happens |
|---|---|---|
| `saturation` (default) | fills the wheel to capacity, once | momentum guard trips **SAFE** on that step |
| `failure` | derates the wheel so it delivers no torque | pointing collapses and **nothing trips** |

`saturation` is the default because the [case study](case_study.md) (section 9) specifies "inject wheel
saturation … Stateflow → SAFE". The wheel is not jammed there: the
actuator model lets a saturated wheel torque back towards zero, and the
reference confirms it unloads from 0.0198 to 0.0123 N·m·s over the ten
seconds after injection. That is what keeps E5 a fault injection rather
than a termination.

**`failure` was measured, and it disproved the obvious reasoning.** This
harness originally claimed a dead wheel makes momentum build on the
remaining axes until the momentum guard trips. Flying it shows the
opposite: the pointing error grows without bound — 23° to 178° over a
minute — while the healthy wheels stay near zero momentum and **no guard
ever fires**, because the error is about the dead axis and the
controller asks the other two for almost nothing. That is a real gap in
the vehicle's fault detection and worth reporting as one. It is not what
E5 was specified to inject, which is why it is not the default.

### How a fault reaches the twin

The twin is MATLAB, so the fault has to cross a process boundary. It
travels as a small JSON the runner writes beside each result:

```json
{"gains": {"natural_frequency_rad_s": 1.2},
 "fault": {"axis": 0, "at_s": 123.456, "kind": "saturation", "seed": 5}}
```

`adcs.config(scenario, runConfig)` merges it, and `build_acav_twin`
takes a `RunConfigPath` that bakes it into the generated model. The same
file carries per-run gains (nominal runs set none), which had the
same problem: neither is a property of the scenario, so neither belongs
in the YAML. It also carries the run id and the execution mode.

Three details that are load-bearing. A missing run-config file is an
**error**, not a silent default: a gain sweep that quietly used nominal
gains at every point would produce a flat surface that reads as a
finding, and the record would claim gains the twin never used. `axis` is
**zero-based** in the record and converted once inside
`adcs.wheel_fault`, because an off-by-one there faults the wrong wheel
while every other number in the run still looks right — there is a
fixture for it. And the saturation is applied to the state going *into*
a step, before the momentum guard is evaluated, so SAFE trips on the
very step the fault lands; a one-step lag would be invisible in a plot
and would make the recorded epoch disagree with the one the run reacted
to.

`replay_matches` compares determinism signatures, not capture counts: two
runs can agree on how many targets they captured while disagreeing about
every pointing error. A replay that ran a *different* fault is reported
as not reproduced even when the signatures match, because that match says
the fault made no difference rather than that the run was reproduced.

## 4. The matrices

| Builder (`acav/campaign.py`) | `--experiment` | Runs | Shape |
|---|---|---:|---|
| `build_e1_matrix` | E1 | `--repeats` x (1 + 2 + 5) | control, frame_pairing guard on and ablated, five more guard-on mismatch types (`--e1-types`), lockstep |
| `build_e2_scheduler_comparison` | E2 | 2 | kinematic and dynamics-aware, lockstep, on `--scenario` (reference by default) |
| `build_e3_matrix` | E3 | (1 + `--fault-seeds`) x (L + R1 + R05) | per fault cell (none, seeds 0..K-1): lockstep x L (`--repeats-lockstep`, default 2), free-run rtf 1.0 x R1 (`--repeats`), free-run rtf 0.5 x R05 (`--repeats-rtf05`, default `--repeats`); `build_e3_determinism` remains for the older two-mode form |
| `build_e5_fault_campaign` | E5 | `--seeds` | one seeded wheel fault per seed, lockstep |
| `build_e6_latency` | E6 | `--repeats` | identical lockstep runs |

The real-time factor is part of the E3 matrix (`RunSpec.real_time_factor`):
the campaign substitutes it into each run's command through
`{real_time_factor}` and checks it against the record, so all three
modes share one E3 campaign directory. Outside a campaign it is given
to a single run as `--real-time-factor`.

**E3's fault replay** is part of the E3 matrix: every mode gets the same
seeded faults, and the rate reaches each run through `{real_time_factor}`
(`RunSpec.real_time_factor`, checked against the record). The plan emits
every run's exact command (`campaign_runner.planned_commands`). The
E5 builder plans lockstep fault runs only.

E3 sets the same seeds on both sides so that an identical free-run batch
would mean a deterministic loop, not an unvaried one, and groups results
by the mode the bridge *reported*.

E1's mismatch type and guard travel like the forced provider: the run
id, `{mismatch}` / `{mismatch_guard}`, `--mismatch` / `--mismatch-guard`,
`sensing.inject_mismatch` in the bridge YAML copy, `e1_mismatch` in the
run config, `Provenance.mismatch` / `mismatch_guard`; never the
determinism signature. `report` groups E1 by arm and counts a planned
run with a `summary.json` but no record as a fail-stop in its arm. The
two-run treatment (`build_e1_convention`) is kept for existing folders.
E1's frame_pairing treatment sets `sensing.force_attitude_provider` on the bridge,
pairing `ECF Attitude Quaternions` against the inertial command, and
disables the twin's mismatch guard (`attitude_mismatch_policy:
e1_readback_ablation` in the run config): E1 measures what the guard
prevents. The provider is observed on BridgeStatus and recorded; the
runner rejects an E1 record whose read-back provider was never observed.

Run ids are derived from the configuration, never from position, which
is what makes resume possible.

## 5. One run: the run orchestrator

`acav/deploy_run_cli.py` (`scripts/run_acav_run.py`) runs one run end to
end and records every step in `<run>.logs/summary.json`:

| Step | What it checks |
|---|---|
| `run_config` | Writes or validates `<run>.runconfig.json` for this run. |
| `scenario` | With `--scenario`: the file differs from the reference only in its targets (the twin reads the reference). |
| `idle_check` | No other run's bridge, scheduler or twin is publishing. |
| `bridge_connected` | The bridge reaches CONNECTED within 180 s; fails at once if it exits. Records `bridge_connect_s` and every startup phase. |
| `stk_process` | COM runs: exactly one STK Desktop, started by the operator (refused otherwise unless `--allow-com-started-stk`). |
| `stk_graphics` | STK's graphics state matches `--graphics-mode` (default off). |
| `twin_launch` | `--twin-launch matlab-batch` only: MATLAB started. |
| `twin_acking` | The twin acks (manual: `--twin-ack-timeout-s`; automated: 252.6 s). |
| `twin_identity` | The twin loaded this run's config and runs this checkout's code. |
| `scheduler_exit` | The scheduler ran and wrote the record. |
| `stk_graphics_end` | Graphics state held for the whole run. |
| `free_run_start_gate` | Free-run: the start gate opened. |
| `matlab_session` | Automated: MATLAB stopped cleanly. |
| `topic_capture` | The five captured topics were saved. |

Post-run steps are bounded (`POST_RUN_TIMEOUT_S`): one that does not
finish is recorded as failed and the run goes on. Any exception a step
raises is kept in `summary.json` under `errors` (type, message,
traceback). QuickEdit is turned off for the run and restored after.
Run conditions are written into the record's provenance afterwards:
graphics mode, real-time factor, MATLAB version, STK process.

## 6. The campaign runner

`run_acav_campaign {plan,run,report,run-seeds}` (`acav/campaign_cli.py`,
`acav/campaign_runner.py`, `acav/e2_seeds.py`):

- **plan** writes `campaign.json` and lists the runs; launches nothing.
- **run** executes the matrix with the launch command after `--`
  (`CommandLauncher` placeholders: `{run_id}`, `{experiment}`,
  `{scheduler}`, `{execution_mode}`, `{seed}`, `{output}`,
  `{scenario}`, `{backend}`, `{attitude_provider}`, `{run_config}`,
  `{gains_json}`, `{fault_json}`, `{mismatch}`, `{mismatch_guard}`,
  `{real_time_factor}`, `{twin_launch}`). Arguments are passed
  as a list, so a value with spaces or quotes reaches the run intact.
  With `--twin-launch matlab-batch` the runner appends
  `--twin-launch matlab-batch` to each run's command.
- **report** reads the directory; it needs no STK and no MATLAB.
- **run-seeds** (E2 only) runs E2 over seeded scenarios and the
  reference in one command: per seed it builds and verifies the scenario
  (closing any open scenario first, leaving graphics off and checking
  it), skips a seed whose records are complete on this commit, and runs
  E2 into `E2_final_seed<k>_<tag>`; then it rebuilds the reference, runs
  its arm into `E2_final_ref_<tag>` and makes the reference and paired
  reports. It refuses a dirty tree, and a `--commit-tag` that is not
  HEAD (default: HEAD's short hash); seed files come from
  `--scenario-dir` (default `src/acav/config/scenarios/generated` under
  the current directory) and are checked before anything is built.

**Resume rules.** A run whose record exists, is readable, matches the
requested run (seed, fault, scheduler, gains, scenario targets) and
whose `summary.json` records no failed step is skipped. Any failed
step, a MATLAB session that did not stop cleanly included, makes it run
again; the previous attempt's record, run config and logs are renamed
`<name>.superseded-<time>`, never deleted. Resuming over records made by
other code is refused unless `--resume-across-code-versions`; running
from a dirty tree is refused unless `--allow-dirty`. `--no-resume`
re-runs everything. A launcher error costs one run unless
`--stop-on-error`.

**Cleanup.** Before each automated run, processes the previous run
recorded (its launcher's and MATLAB's PIDs, and what they started;
only `matlab.exe` and `libmwros2server.exe`) are killed and listed in
`twin_cleanup.jsonl`.

**Manifest.** `campaign.json` records the matrix, the code version and
the twin launch mode.

## 7. Reportability, coverage and pooling

A record may appear in the paper only if `RunRecord.reportable`
(derived on every read, never stored): a real STK backend; in lockstep,
twin telemetry in every hold window; an access answer for every
attempted target; access computed over a written attitude. `report`
exits 1 and names any unreportable record.

**Target coverage.** Each record must name a known scenario by digest
(the reference, `--scenario`, or a generated scenario in `--scenario-dir`,
in `src/acav/config/scenarios/generated` under the current directory, or
next to the package) and contain
exactly that scenario's targets (`run_record.target_coverage_problems`;
legacy digests via `LEGACY_SCENARIO_DIGESTS`).

**Pooling.** `report` refuses a directory whose records differ in:
STK graphics mode (including "mixed" or "unverified" records), twin
launch mode, how STK was started (`stk_process.started_by`; records
without it count as "not recorded"), or scenario; and it refuses runs
labelled with a diagnostic (`--diagnostic-label`) or made with the
sensing equivalence diagnostic. Each refusal names the groups and their
runs. It also warns unless all records come from one clean commit.

## 8. What the record carries from the twin

| Quantity | How it is derived |
|---|---|
| `peak_momentum_fraction` | per-axis max of \|h\| / capacity over the run |
| `wheel_saturation_events` | the larger of the published flag's rising edges and the twin's cumulative `saturation_events` |
| `entered_safe_mode` | any sample reporting SAFE |
| `wheel_energy_J` | max of the twin's own cumulative integral of \|tau . omega\| |

A saturation event is an entry into saturation, not a step spent
saturated, so the count does not depend on the telemetry rate. Wheel
energy is accumulated by the twin, the only party that holds torque and
rate at the same instant. Momentum crosses the wire in N m s, so a
consumer cannot silently disagree about the capacity. All of these
enter `determinism_signature`.

The E6 latency breakdown comes from the bridge (BridgeStatus mean
sense, actuate and barrier-wait times per second, with their sample
counts), recorded by the scheduler; see [analysis](analysis.md).
