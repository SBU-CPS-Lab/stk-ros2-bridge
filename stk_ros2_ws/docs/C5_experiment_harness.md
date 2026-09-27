# C5 — The experiment harness

What turns the closed loop into experiments: run matrices for E1–E5,
provenance that decides whether a number may be published, seeded faults
that can be replayed exactly, and a campaign driver that survives being
interrupted at run 73 of 100.

## 1. Why the harness is split in two

| Module | Role |
|---|---|
| `acav/run_record.py` | Provenance + serialisation. **The C5/C6 contract.** |
| `acav/fault_injection.py` | Seeded wheel faults for E5 |
| `acav/campaign.py` | The run matrices, and what the results mean |
| `acav/campaign_runner.py` | Executes a matrix. Knows nothing about STK |
| `acav/campaign_cli.py` | `ros2 run acav run_acav_campaign` |

The first three are pure: no ROS, no subprocess, no sockets. The fourth
is thin wiring, and takes its launcher by injection so the whole loop —
resume, validation, aggregation — is exercised in tests here without
STK or MATLAB. This is the same division C4 draws between
`scheduling.py` and `scheduler_node.py`, and for a sharper reason: a
campaign is 40 runs for E3, 49 for E4 and 100+ for E5, each needing STK
and MATLAB. It is an overnight thing on one machine and it cannot be
iterated on. Everything about it that can be decided in milliseconds on
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

C1 section 8 commits the paper to reporting **no number the mock backend
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

E5's claim is deterministic fault *replay*, so the thing being replayed
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

`saturation` is the default because C1 section 9 specifies "inject wheel
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
file carries **E4's gains**, which had the same problem: neither is a
property of the scenario — the scenario is identical in every run of
both — so neither belonged in the YAML.

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

| Builder | Runs | Shape |
|---|---:|---|
| `build_e1_convention` | 2 | validated read-back vs deliberately mis-paired |
| `build_e2_scheduler_comparison` | 2 | kinematic vs dynamics-aware, same targets |
| `build_e3_determinism` | 40 | 20 lockstep vs 20 free-run, identical config |
| `build_e4_gain_sweep` | 49 | 7×7 over bandwidth × profile authority |
| `build_e5_fault_campaign` | N | one seeded wheel fault per seed |

E3 sets the **same** seeds on both sides rather than leaving them unset,
so that if a free-run batch does come out identical it is because the
loop was deterministic and not because nothing in it varied. Its results
are grouped by the mode the bridge *reported*, so a run that asked for
lockstep and fell back belongs to the free-run group rather than
inflating a determinism claim it does not support.

E1's treatment sets `sensing.force_attitude_provider` on the bridge,
pairing the Earth-fixed read-back provider (`ECF Attitude Quaternions`)
against the inertial command form ACAV uses (`AddAttitude Quat`, see
`config/bridge_params_acav.yaml`). The result is wrong by the
Earth-rotation angle and **nothing raises** — which is the finding, and
also why the knob fails loudly on an unrecognised provider name rather
than falling back to the validated one: a typo that silently produced
the nominal pairing would yield a "treatment" that behaves like the
control and reports that mis-pairing costs nothing. The forced provider
is recorded in the run's provenance, so a mis-paired run is identifiable
from the results rather than only from the launch command's history.

E4 does not sweep damping: C3 fixed ζ = 0.707, and a third axis would
take the campaign from an overnight run to a weekend one for a parameter
the design does not treat as free. E4 is only interpretable at all
because E3 establishes that a single run is noise-free — otherwise every
grid point would need repeats and the surface would measure variance.

Run ids are derived from the configuration, never from position in the
list. Positional ids renumber when a seed is inserted and silently
repoint every result an earlier campaign wrote; they are also the only
reason resume can work, since a completed run is identifiable on disk
only if its output name is a function of the run.

## 5. What the runner refuses to do

- **Treat a missing file as an empty result.** A launcher that exits
  cleanly without writing has not produced a run; scoring it as one that
  captured nothing would put launch problems into the headline metric.
- **Merge a record that does not match the run requested.** A wrong seed
  or fault breaks E5's replay claim, a wrong scheduler breaks E2's
  comparison, wrong gains break E4's surface. Each is reported as a
  failure and excluded. Execution mode is deliberately *not* checked —
  see E3 above.
- **Resume over a mismatched record.** An existing file that disagrees
  with its run is re-run rather than trusted.
- **Lose the night to one bad run.** A launcher exception costs one run
  unless `--stop-on-error` is given.

## 6. Running it

```
# Is this the matrix I meant? Cheap, launches nothing.
ros2 run acav run_acav_campaign plan --experiment E3 --output-dir results/E3

# Execute it. Re-issue after a crash; completed runs are skipped.
ros2 run acav run_acav_campaign run --experiment E3 --output-dir results/E3 -- \
    python launch_one_run.py --run {run_id} --mode {execution_mode} \
                             --scheduler {scheduler} --out {output}

# Analyse. Needs no launch command, no STK and no MATLAB.
ros2 run acav run_acav_campaign report --experiment E3 --output-dir results/E3
```

Placeholders available to the launch command: `{run_id}`,
`{experiment}`, `{scheduler}`, `{execution_mode}`, `{seed}`, `{output}`,
`{scenario}`, `{backend}`, `{gains_json}`, `{fault_json}`.

Exit codes: `0` the claim held, `1` it ran but the result does not
support the claim (or runs are missing, or some are unreportable), `2`
the invocation was wrong and nothing was measured.

The launch script is the operator's to write, and it is where the
scheduler's new parameters are passed through: `experiment`, `seed`,
`gains_json`, `fault_json`, `output_json`. The scheduler records the
gains and fault as **configured** and the backend and execution mode as
**observed** — a backend can silently differ from what was asked for and
gains cannot.

A concrete launch script now exists —
`acav/deploy_run_cli.py` (`scripts/run_acav_run.py`), which starts the
bridge, hands off to a human for the MATLAB/Simulink twin (confirmed
over ROS, not just a keypress — see that module's own docstring for
why an unattended MATLAB step is not scripted yet), then runs the
scheduler and captures every process's log. See
[08_deploy_runbook.md](08_deploy_runbook.md) for how to actually run it
on the Windows machine.

## 7. What the record now carries from the twin

`TwinStatus` carries the reaction-wheel state alongside mode and
pointing error, so the run-level quantities in `RunResult` are populated
rather than left at their defaults:

| Quantity | How it is derived |
|---|---|
| `peak_momentum_fraction` | per-axis max of \|h\| / capacity, over the run |
| `wheel_saturation_events` | count of wheels *entering* saturation |
| `entered_safe_mode` | any sample reporting the SAFE mode name |
| `wheel_energy_J` | max of the twin's own cumulative ∫\|τ·ω\| |

Three of those decisions are load-bearing. A saturation **event** is an
entry, not a step spent saturated: counting steps would make the number
a function of the telemetry rate, so the same run sampled twice as often
would report twice the faults and two runs at different rates could not
be compared — which is exactly what E5 needs of them. Wheel energy is
**accumulated by the twin**, because the twin is the only party holding
the delivered torque and the body rate at the same instant, and a
running total survives a dropped telemetry sample where a consumer-side
integration would silently lose that step's contribution; the consumer
takes the maximum rather than the last value so one late, stale message
cannot under-report the run. Momentum crosses the wire in N·m·s rather
than normalised, so a consumer that disagreed about the capacity cannot
silently disagree about how close to saturation the vehicle was.

These also give `determinism_signature` something to discriminate on: it
already hashed all three, but while they were constant they contributed
nothing, and E3's comparison rested entirely on per-target outcomes.

## 8. Open items

- **E6 needs a latency breakdown the bridge does not publish.**
  `RunResult.mean_step_latency_ms` is now populated, but from the
  scheduler's own tick-to-tick wall clock — which in lockstep is a real
  measure of the achieved step rate, since the barrier does not release
  until every participant has acked. It is *not* the sense/actuate/
  barrier breakdown C1 section 9 asks E6 for; that has to come from the
  bridge measuring its own phases and publishing them. The field is
  labelled for what it actually is rather than filled with the backend
  round-trip time, which would have mislabelled one component as the
  whole.
- **No campaign has been run against live STK.** Everything here is
  verified against an injected launcher. The first real campaign is the
  test of section 5's refusals.
