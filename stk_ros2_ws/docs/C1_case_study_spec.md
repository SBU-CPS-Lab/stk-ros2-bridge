# C1: ACAV case-study specification

**Attitude-Coupled Access Verification.** The case study that exercises
the framework end to end. Where this document and
`src/acav/config/acav_scenario.yaml` (the reference scenario) disagree,
the YAML wins; `acav/spec.py` loads it and cross-checks it.

## 1. Claim under test

> Mission planning that computes target access assuming instantaneous,
> perfect pointing over-promises imaging capacity. The over-promise is
> measurable, attributable to attitude dynamics, and only observable in
> a closed, time-synchronised loop.

A null result (both schedulers capture the same targets) is a valid
outcome; the target sets span feasibility regimes so that either answer
is informative.

## 2. Why the loop must be closed

Access depends on attitude, attitude on the controller, and the
controller on when access begins. The loop carries two paths that cannot
be precomputed:

| Direction | Content |
|---|---|
| Twin -> STK | The achieved attitude, every 0.1 s, `AddAttitude Quat` (inertial; read back through `Attitude Quaternions`) |
| STK -> twin and scheduler | Orbit, attitude read-back, and sensor access computed at the achieved attitude |

## 3. Honest scope

For a simple conic sensor the access geometry could be re-implemented in
MATLAB; the paper must not claim otherwise. The defensible claim is that
the STK scenario the mission team already uses becomes the evaluation
environment for control design, with no second geometry model to drift
from it. HPOP truth propagation means the twin is evaluated against a
perturbed orbit it does not itself model.

## 4. Spacecraft

SSS-1, a 6U agile Earth-observation CubeSat in a 500 km sun-synchronous
circular orbit (i = 97.4 deg), HPOP. Values marked `[ASSUMED]` in the
YAML are defensible defaults, not measured properties.

| Quantity | Value |
|---|---|
| Mass | 8.0 kg `[ASSUMED]` |
| Principal inertia | diag(0.123, 0.096, 0.041) kg m^2 `[ASSUMED]` |
| Wheel torque limit | 2 mN m `[ASSUMED]` |
| Wheel momentum capacity | 20 mN m s `[ASSUMED]` |

Body frame: +Z boresight (nadir when unslewed), +X along velocity.
Cross-track slews rotate about +X, the largest inertia. Derived limits
(`acav/slew_model.py`): 0.932 deg/s^2, 9.32 deg/s, crossover 93.2 deg;
every slew here is below it, so all profiles are triangular.

## 5. Sensor and targets

Simple conic sensor, **2.0 deg half-angle** along +Z (about 17.5 km
footprint radius at 500 km). The half-angle sets how much pointing error
matters; a wide field of view would make the effect disappear.

Targets are placed relative to the ground track: the builder asks STK
for the sub-satellite point at `window_start + t_image_s` and offsets
it by `cross_track_km` (`acav/scenario_builder.py`). Window: 720 s from
1 Jun 2024 10:30:00 UTC, step 0.1 s.

**Reference scenario** (`config/acav_scenario.yaml`), 7 targets:

| ID | t_image (s) | cross-track (km) | margin (s) | regime |
|---|---:|---:|---:|---|
| T1 | 60 | -180 | +42.8 | comfortable |
| T2 | 110 | +210 | +28.5 | comfortable |
| T3 | 127 | +40 | +0.2 | knife-edge |
| T4 | 250 | -240 | +103.6 | comfortable |
| T5 | 265 | -60 | -1.9 | infeasible |
| T6 | 400 | +220 | +115.6 | comfortable |
| T7 | 410 | +90 | -5.6 | infeasible |

Margins are planning estimates (slew plus an 8 s settle estimate);
STK and the twin decide the outcome. `test_spec.py` guards that the set
spans feasible and marginal regimes. The reference file must stay
byte-identical: records name it by digest.

**Seeded scenarios** (`acav/scenario_generator.py`, generator version
2; committed for seeds 1-10 under `src/acav/config/scenarios/generated`):
the reference with only its target list replaced and its ground station
removed. 22 targets with image epochs in [40, 700] s, at least 4 s
apart, the last at 650 s or later, cross-track offsets 20-260 km, and at
least 11 comfortable (margin > 25 s), 4 marginal (0-1 s) and 4
infeasible (<= -1.5 s) transitions, the rest moderate (>= 5 s). The
same seed and generator version give the same bytes. The twin always
reads the reference YAML; the runner refuses a scenario that differs
from the reference anywhere but its targets (`acav/scenarios.py`
`twin_parameter_differences`).

## 6. Capture criterion

A target is **CAPTURED** at its image epoch t_i if and only if both:

1. STK reports sensor access to the target at t_i, computed over the
   attitude the twin actually achieved (geometry);
2. the pointing error stays at or below **0.5 deg** for all t in
   [t_i - 2 s, t_i] (dynamics), judged on the largest error the twin
   reports for each sample (`max_boresight_error_since_last_rad`).

Condition 1 alone is what conventional planning checks; the gap is the
headline quantity. Outcomes (`acav/metrics.py` `CaptureOutcome`):

| Outcome | Meaning | Reportable |
|---|---|---|
| `captured` | Both conditions held. | yes |
| `not_settled` | Access existed; the controller had not settled. The failure mode under study. | yes |
| `no_access` | Settled, but no access. | yes |
| `no_access_and_not_settled` | Neither. | yes |
| `not_attempted` | The scheduler declined. | yes |
| `safe_mode` | In SAFE at the image epoch (fault runs). | yes |
| `no_telemetry` | No twin sample in the hold window. In free-run an outcome of asynchronous coupling; in lockstep a harness failure. | free-run only |
| `no_access_answer` | STK's access answer never arrived. | no |
| `access_unverified` | The written attitude never reached t_i by the end of the run, so access was never computed over the twin's attitude. | no |

Tolerances: capture 0.5 deg, hold 2.0 s, SLEW -> SETTLE entry 2.0 deg.
`capture_rate` (over attempted targets) and `coverage_rate` (over all
targets) are always reported together.

## 7. Ground station: dropped

The reference YAML still defines a terrain-masked ground station GS1,
and the builder still creates it (the file stays byte-identical so that
existing records keep their scenario digest). No experiment uses it and
the paper makes no claim about it; seeded scenarios omit it.

## 8. Compute budget (measured)

A lockstep step costs about 60 ms on the deploy host (E6, graphics off,
operator-started STK; [09_live_runs_and_results.md](09_live_runs_and_results.md)).
Measured wall time per automated run, including MATLAB start and build:

| Run kind | Wall time per run | Source |
|---|---|---|
| Lockstep, rtf n/a | 9.5-11.4 min | E5_P9d_lock1/2, E6_P9 seed0/2, E2_P10_seed1 |
| Free-run, rtf 1.0 | 15.3-16.2 min | E3_P9_rtf1, E5_P9d_free1, E5_P10_free2 |
| Free-run, rtf 0.5 | 37.6 min (one run, manual twin) | E3_P6_rtf05_8345551 |

Commitments: the scenario is one 12-minute window, not a full orbit;
E6 reports the measured latency; no reported number comes from the mock
backend (`RunRecord.unreportable_reasons`).

## 9. Experiments

| ID | Question | Design |
|---|---|---|
| **E1** | Does convention validation matter? | Two lockstep runs: the validated attitude pairing against a deliberately mis-paired read-back (`ECF Attitude Quaternions` against the inertial command) with the twin's mismatch guard explicitly ablated. Expected: error inflated by the Earth-rotation angle, capture collapsing, nothing erroring. |
| **E2** | Does kinematic planning over-promise? | Kinematic against dynamics-aware scheduling on the same targets, lockstep, on the reference scenario and on seeded scenarios. Capture rate, coverage rate, over-promise gap, outcome breakdown. **Headline.** |
| **E3** | What does lockstep buy, and what does free-run cost? | Repeated lockstep and free-run runs with identical configuration, free-run at rtf 1.0 and 0.5; compare determinism signatures, outcomes and coupling metrics. Includes **deterministic fault replay**: a seeded wheel fault, SAFE on the momentum guard, and bit-identical reproduction in lockstep (formerly E5). |
| **E6** | What does it cost? | Repeated identical lockstep runs: per-step sense, actuate and barrier-wait times, achieved step rate. Graphics off, no CPU sampler, operator-started STK. |

**Dropped and merged.** E4 (controller gain sweep) is dropped; its
campaign builder remains in `acav/campaign.py` but no experiment uses
it. E5 (fault replay) is merged into E3; the E5 campaign builder still
plans the seeded lockstep fault runs (`run_acav_campaign --experiment
E5`). Candidate studies not yet decided (a communication-step study, an
offline file-exchange baseline) are in
[dev/phase3_backlog.md](dev/phase3_backlog.md), not here. Repeat
counts for the final runs are not fixed yet; the runbook writes them as
parameters ([08_deploy_runbook.md](08_deploy_runbook.md)).

## 10. Executable contracts

| Artifact | Role |
|---|---|
| `config/acav_scenario.yaml` | The reference scenario. |
| `acav/spec.py` | Loader; recomputes derived values and cross-checks duplicates (`AcavSpec.validate`). |
| `acav/slew_model.py` | The slew-time model shared by the scheduler and the twin's reference. |
| `acav/metrics.py` | The capture criterion and run metrics, shared by scheduler, harness and analysis. |
| `acav/run_record.py` | The record, its provenance and the reportability rules. |
| `acav/spec_cli.py` | `ros2 run acav describe_scenario`. |

The contracts are code, not prose, because E2 is meaningful only if the
scheduler's notion of a slew matches what the twin achieves, and the
capture criterion is applied identically everywhere it is used.
