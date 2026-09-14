# C5 — The experiment harness

What turns the closed loop into experiments: run matrices for E2–E5,
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

`replay_matches` compares determinism signatures, not capture counts: two
runs can agree on how many targets they captured while disagreeing about
every pointing error. A replay that ran a *different* fault is reported
as not reproduced even when the signatures match, because that match says
the fault made no difference rather than that the run was reproduced.

## 4. The matrices

| Builder | Runs | Shape |
|---|---:|---|
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

## 7. Open items

- **E1 has no knob yet.** It needs the bridge to deliberately mis-pair
  the attitude read-back (an inertial provider against a `CBFQuat`
  command — the bug found during bridge bring-up). Today
  `core/sensing.py` picks the provider from the validation report and
  there is no way to force the wrong one, so E1 is not expressible as a
  campaign variation. That is a small, deliberate change in `stk_bridge`
  rather than in the harness, and it is not made here: shipping a
  `RunSpec` field that nothing honours would look like support for an
  experiment that cannot run.
- **E6 is not a campaign.** Per-step latency comes from the bridge's own
  diagnostics over a single run, not from a matrix.
  `RunResult.mean_step_latency_ms` is carried through the record already;
  what is missing is the scheduler populating it.
- **`RunResult`'s run-level fields are still defaults.** Peak momentum,
  saturation events, SAFE entry and wheel energy are serialised and
  round-trip correctly, but the scheduler does not yet fill them in —
  they need the twin to report momentum and saturation, which is a
  `TwinStatus` field and a deliberate message change (see
  `docs/C3_adcs_twin.md` section 7). Until then E5's `safe_mode_runs`
  count reads as zero, and although `determinism_signature` does hash
  those three quantities, they are constant across every run and so
  contribute nothing that could distinguish one from another — E3's
  comparison currently rests entirely on the per-target outcomes.
- **No campaign has been run against live STK.** Everything here is
  verified against an injected launcher. The first real campaign is the
  test of section 5's refusals.
