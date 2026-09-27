# C1 — ACAV Case Study Specification

**Attitude-Coupled Access Verification** for the STK↔ROS 2 bridge paper.

This is the contract for phases C2–C6. Where it disagrees with anything
downstream, this document and `config/acav_scenario.yaml` win.

---

## 1. Claim under test

> Mission planning that computes target access assuming instantaneous,
> perfect pointing over-promises imaging capacity. The over-promise is
> measurable, attributable to attitude dynamics, and only observable in
> a closed, time-synchronized loop.

The null result — that dynamics-aware and kinematic planning capture the
same targets — is a publishable outcome too, and the experiment is
designed not to preclude it. Nothing in the scenario forces a
particular answer; §5's target set spans feasibility regimes so that
*whatever* happens is informative.

## 2. Why the loop must be closed

The conventional workflow is one-way: propagate in STK, export
ephemeris, analyse offline. That workflow cannot answer this question,
because access depends on attitude, attitude depends on the controller,
and the controller depends on when access begins. Breaking the loop at
any point requires assuming away the coupling under study.

Specifically, the bridge is load-bearing for two data paths:

| Direction | Content | Why it can't be precomputed |
|---|---|---|
| Twin → STK | Achieved attitude, 10 Hz, `AddAttitude Quat` (inertial; read back through `Attitude Quaternions`) | Depends on controller response to a schedule not known in advance |
| STK → Twin | Sensor access at the achieved attitude | Depends on the attitude just injected |

## 3. Honest scope

For a simple conic sensor, the access *geometry* is reimplementable in
MATLAB. The paper must not claim otherwise. The defensible claim is:

> The same STK scenario the mission team already uses for analysis
> becomes the evaluation environment for control design — no
> re-implementation, and therefore no divergence between the analysis
> model and the design model.

Two scenario choices harden this against the "just reimplement it"
objection:

- **HPOP** truth propagation rather than two-body, so the twin is
  evaluated against a perturbed orbit it does not itself model.
- **Terrain-masked ground-station contact** (§7). Terrain is applied to
  the *ground station*, not the imaging targets, because a sensor at
  ≤30° off-nadir is essentially never terrain-masked. Claiming terrain
  mattered for imaging would be dishonest.

## 4. Spacecraft

SSS-1, a 6U agile EO CubeSat in a 500 km sun-synchronous orbit
(i = 97.4°, circular). Values marked `[ASSUMED]` in the YAML are
defensible defaults, not measured properties of a real vehicle.

| Quantity | Value | Source |
|---|---|---|
| Mass | 8.0 kg | `[ASSUMED]` typical 6U with wheels |
| Inertia (principal) | diag(0.123, 0.096, 0.041) kg·m² | `[ASSUMED]` 100×226×366 mm, 8 kg |
| Wheel max torque | 2 mNm | `[ASSUMED]` CubeSat class |
| Wheel momentum capacity | 20 mNms | `[ASSUMED]` |
| Truth propagator | HPOP | §3 |

**Body frame:** +Z = sensor boresight (nadir when unslewed), +X = along
velocity, +Y completes the right-handed set. Cross-track slews are
therefore rotations about +X, which carries the *largest* inertia
(0.123) — the conservative choice, since it yields the slowest slew and
the scheduler never promises more agility than the vehicle has.

**Derived slew limits** (recomputed in `slew_model.py`, not trusted from
the YAML comments):

- α = τ/J = 0.0163 rad/s² = **0.932 °/s²**
- ω_max = h/J = 0.163 rad/s = **9.32 °/s**
- triangular→trapezoidal crossover = ω_max²/α = **93.2°**

Every slew in this study is below the crossover, so all profiles are
triangular. The trapezoidal branch is implemented and tested anyway, so
a reader who resizes the wheels does not silently fall off the model's
validity range.

## 5. Sensor and targets

**Sensor:** simple conic, **2.0° half-angle**, boresight along +Z body.
At 500 km this gives a ~17.5 km footprint radius.

The half-angle is the single most consequential parameter in the study.
It sets how much pointing error matters: at 2.0°, a 1° error is a
significant fraction of the field of view, which is exactly the regime
where attitude dynamics decide capture success. A wide-FOV sensor would
make the entire effect disappear.

**Targets are defined relative to the ground track**, not as hardcoded
lat/lon. Fixed coordinates break the moment the epoch shifts — the
satellite simply isn't over them, and the scenario silently yields zero
access. C2 instead queries STK for the sub-satellite point at
(window_start + `t_image_s`), offsets perpendicular to the ground track
by `cross_track_km`, and creates the target there.

Seven targets over a 720 s window, spanning three feasibility regimes:

| ID | t_image (s) | cross-track (km) | slew (°) | need (s) | have (s) | margin (s) | regime |
|---|---:|---:|---:|---:|---:|---:|---|
| T1 | 60 | −180 | 19.7 | 17.2 | 60.0 | **+42.8** | comfortable |
| T2 | 110 | +210 | 42.3 | 21.5 | 50.0 | **+28.5** | comfortable |
| T3 | 127 | +40 | 18.1 | 16.8 | 17.0 | **+0.2** | knife-edge |
| T4 | 250 | −240 | 30.0 | 19.4 | 123.0 | **+103.6** | comfortable |
| T5 | 265 | −60 | 18.6 | 16.9 | 15.0 | **−1.9** | infeasible |
| T6 | 400 | +220 | 30.4 | 19.4 | 135.0 | **+115.6** | comfortable |
| T7 | 410 | +90 | 13.4 | 15.6 | 10.0 | **−5.6** | infeasible |

"need" = analytic slew time + `settle_estimate_s` (8 s). These are
*planning estimates*, informational only; STK and the twin decide the
actual outcome.

An earlier draft had every gap comfortably feasible, which would have
made E2 unable to measure anything. `test_spec.py::
test_scenario_spans_feasible_and_marginal_regimes` caught it and now
guards the property. That test should not be relaxed to accommodate a
convenient target set.

## 6. Capture criterion

A target is **CAPTURED** at its image epoch `t_i` if and only if **both**:

1. **STK reports sensor access** to the target at `t_i` — the geometry
   half, evaluated at the achieved attitude.
2. **Pointing error ≤ 0.5°** for all t ∈ [`t_i` − 2 s, `t_i`] — the
   dynamics half.

Condition 1 alone is what conventional planning checks. The gap between
"condition 1 only" and "both conditions" is the paper's headline
quantity.

Failure is attributed, not just recorded:

| Outcome | Meaning |
|---|---|
| `NOT_SETTLED` | Access existed; the controller hadn't converged. **The failure mode the study is about.** |
| `NO_ACCESS` | Settled, but pointing stably at the wrong place, or geometry never permitted it |
| `NO_ACCESS_AND_NOT_SETTLED` | Neither |
| `NOT_ATTEMPTED` | Scheduler declined |
| `SAFE_MODE` | In SAFE at the image epoch (E5 runs) |

Tolerances: capture 0.5° (a quarter of the sensor half-angle), hold
2.0 s, SLEW→SETTLE entry 2.0°.

Missing telemetry in the hold window **raises** rather than scoring a
capture failure — a harness bug must not be recorded as a physics
result.

## 7. Ground station

GS1, placed on the ground track at t = 600 s, 10° minimum elevation,
**terrain masking enabled**. If the STK licence lacks terrain, C2's
verifier reports it and the study degrades to a spherical-Earth
elevation mask — in which case the paper must not claim terrain
masking. `verify_acav_scenario.py` (C2) records which mode was used.

## 8. Compute budget

At a measured ~10 ms per closed-loop step:

| Span | Steps @ 0.1 s | Wall clock/run |
|---|---:|---|
| Full orbit (95 min) | 57,000 | ~10 min |
| **Imaging window (12 min)** | **7,200** | **~1.2 min** |

E4 (49 gain combinations) and E5 (100 Monte Carlo seeds) are only
feasible on the windowed scenario: ~1 h and ~2 h respectively, both
overnight-feasible. The full-orbit versions are not.

**Methodological commitments this forces, to be stated in the paper:**

1. The scenario is a single ~12-minute imaging window, not a full orbit.
2. E6 reports measured latency, so the scaling is transparent rather
   than implied.
3. The **mock backend** is used for scheduler development and the
   scheduler's internal feasibility checks. **No reported number comes
   from the mock.** C5 must record the backend type in every result and
   C6 must assert on it.

## 9. Experiments

| ID | Question | Design |
|---|---|---|
| **E1** | Does convention validation matter? | Nominal run vs. run with attitude read-back deliberately mis-paired (Earth-fixed provider against the inertial `Quat` command — the bug found during bridge bring-up, and, in its other direction, what the first live runs actually did). Expected: error inflated by the Earth-rotation angle, capture collapsing, **and nothing erroring**. |
| **E2** | Does kinematic planning over-promise? | Same target set, two schedulers. Metrics: capture rate, coverage rate, over-promise gap, outcome breakdown. **Headline.** |
| **E3** | Does lockstep deliver determinism? | 20 runs lockstep, 20 free-run, identical config. Compare `determinism_signature`. Expected: 1 distinct signature vs. many. |
| **E4** | Can the loop tune the controller? | Gain sweep against E2's capture metric. Meaningful only because E3 established each evaluation is noise-free. |
| **E5** | Deterministic fault replay? | Inject wheel saturation at a random epoch across seeds; Stateflow → SAFE; re-run a failing seed and show bit-identical reproduction. |
| **E6** | What does it cost? | Per-step latency breakdown (sense/actuate/barrier), achievable step rate, wall clock per simulated minute. |

Both `capture_rate` (denominator = attempted) and `coverage_rate`
(denominator = all targets) are always reported together. Either alone
is misleading: a scheduler attempting one easy target scores 100%
capture rate.

## 10. C1 deliverables

| Artifact | Role |
|---|---|
| `config/acav_scenario.yaml` | The scenario contract, read by C2–C6 |
| `acav/spec.py` | Loader; recomputes derived values, cross-checks duplicates |
| `acav/slew_model.py` | Shared slew-time model — **C3/C4 contract** |
| `acav/metrics.py` | Capture criterion and metrics — **C4/C5/C6 contract** |
| `acav/spec_cli.py` | `ros2 run acav describe_scenario` |
| `test/` | 54 tests, all passing |

**Why the contracts are executable rather than prose.** E2 compares a
kinematic schedule against a dynamics-aware one. That comparison is
meaningful only if the scheduler's notion of "how long does a slew take"
matches what the Simulink twin achieves. Two independently-written slew
models would make E2 measure their disagreement rather than the effect
under study. Same for the capture criterion across C4/C5/C6.

Cross-checks enforced at load time (`AcavSpec.validate`): hold-time
duplicates must agree; capture tolerance must be tighter than the FOV;
settle-entry must be looser than capture; targets must lie inside the
window with room for their hold window; the fast rate group must give
≥5 samples per hold window.

## 11. Open items for C2

- Verify `SimpleConic` sensor creation and FOV setting against the live
  STK 12.6 Object Model — same version-drift risk that produced four
  rounds of fixes during the bridge's own bring-up.
- Confirm terrain availability; record which masking mode was used.
- Confirm ground-track query returns usable sub-satellite points at the
  chosen epoch under HPOP.
- Ship `verify_acav_scenario.py` in the spirit of
  `check_environment.py`: the scenario should tell you what's wrong,
  not produce empty access lists.

## 12. Next

C2 builds this scenario in STK and verifies it empirically. It is
STK-facing API work in the same category that needed four rounds of
live-install correction during the bridge's own bring-up, so it ships
its own verifier rather than assuming any command that ACKs took
effect.
