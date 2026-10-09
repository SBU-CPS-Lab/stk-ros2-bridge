# Live runs and results so far

Every run of this project's code against a real STK 12.6 install, what it
showed, what it changed, and the experiment data that exists. The design
documents (`00`–`07`, `C1`–`C6`) say what the system is; the runbook
(`08`) says how to run it; this file says what has actually been run and
measured. Written for whoever writes the paper: every number here comes
from a run record, a log, or a test output, and each section says which.

Run data is not in the repository (`results/` is ignored). The two E6
campaigns are kept as the bundles `E6_run/` and `E6_run2/`, E1 as
`E1_run/` and E2 as `E2_run/` (records, run configurations, per-run logs
and topic captures, `campaign.json`, and the report figures).

## 1. Bridge against live STK

`check_environment.py --backend com`: **27/27 checks passed** on the
deploy machine (Windows 10, STK 12.6). The STK 12.6 behaviours that
required changes during bring-up are listed in `06_troubleshooting.md`.

## 2. C2 — scenario build and verification

`verify_acav_scenario.py --backend com`, four runs (2026-09-09 to 09-13):

| Run | Result | What it settled |
|---|---|---|
| 1 | 4 FAIL | `SetState Classical` argument order and angle unit (degrees). Two failures were verifier design errors: a boresight check that passed regardless of the cone, and mask checks that compared a baseline with itself. Both now refuse to conclude without a treatment that actually applied. |
| 2 | 4 FAIL | Orbit exact; HPOP perturbations active; sensor cone reads back 2.0000°. Remaining failures: static objects serve a *fixed* data provider (no `ExecSingle`), and latitude/longitude are unit dimensions separate from angle. |
| 3 | 10 pass, 2 fail, 2 skip | Angles read as radians, targets placed and reachable, ground-station contact 340.1 s. Failures: the verifier placed its probes in radians where `SetPosition ... Geodetic` consumes degrees. |
| 4 | **12 pass, 0 fail, 2 skip** | Boresight confirmed as body +Z (a nadir target inside a measured 1.8–2.2° cone); terrain mask applied via `SetAzElMask`. The two skips were a verifier design error (STK has no constant-elevation mask), since fixed. |

## 3. C3 — the twin cross-check

`test_acav_twin` (MATLAB R2024a, development machine) replays the Python
reference's fixtures; current fixture version 6, which adds the
moving-geometry section described in `C3_adcs_twin.md` section 9. The
Simulink model passes diagram update on the development machine and the
deploy machine. Results at each change are recorded in `C3_adcs_twin.md`
section 5.

## 4. Closed-loop bring-up on the deploy machine

From the first attempt to a run that captures, in order. Each row is one
deploy bundle or one fix round; commit hashes refer to `main`.

| When | Symptom | Cause | Fix |
|---|---|---|---|
| 09-17 | runner never saw the bridge connect | runner subscribed VOLATILE to a TRANSIENT_LOCAL status topic | `c2e6e6c` |
| 09-19 | bridge never reached CONNECTED | it tried to re-create the open scenario; `canonical_object_path("*")` built `*/*/...` paths | `f749267` |
| 09-20 | twin never acknowledged | Subscribe output 1 (`IsNew`) wired instead of output 2 (message); RealTime orbit propagator discarded the HPOP ephemeris; ROS string fields unencoded | `4526520` |
| 09-21 | lockstep abort did not abort; startup races | barrier timeout swallowed by a generic `except`; participants starting late; best-effort telemetry lost in lockstep | `83a6d79`, `4ca2bff` |
| 09-26 (try1) | 2 h run, vehicle never moved, no target scored | the model stamped messages with **wall-clock** time: every attitude sample was written to STK outside the scenario | `32709f5` |
| 09-27 (try2) | aimless rotation, every target SAFE, no access | **frame mismatch**: an inertial twin read and commanded attitude in the Earth-fixed form; plus attitude history left by an earlier run on the reused scenario | `d4544b7` |
| 09-27 (try3) | "illogical rotations" | the twin's command and its barrier ack travel on separate topics; on 7.3% of steps STK was sensed before the command landed and reported its basic attitude, and the twin then took on the body rate STK derived across that jump (up to ~810°/s) | `a83eb2c` |
| 09-30 | T1 missed by ~42° | a Target's position came through STK's fixed provider, which ignores the requested frame and time: Earth-fixed coordinates were used as inertial | `1cba9ee` … `c80b394` |
| 10-04 | slews ended ~10° short; no re-plan from SETTLE | slew planned to a stale target direction; mode logic could not re-plan after a target change | `fd5afa2` … `b8edbae` |

Narrative detail for each is in `08_deploy_runbook.md` section 7.

Facts about STK 12.6 established by these runs, beyond those in
`06_troubleshooting.md`:

- Past the newest real-time attitude sample, STK reports the object's
  basic attitude (in try3, 228 of 265 such reads); it does not hold or
  extrapolate the last sample.
- `SetAttitude <path> ClearData` does clear the real-time attitude
  history (confirmed by the `attitude_history_clears` startup check).
- The attitude it reports back equals the commanded sample exactly once
  commands land on time: 0 of 7199 steps differed by more than 0.000002°
  in each E6 run.

## 5. E6 — loop cost

Five identical lockstep runs per campaign (`build_e6_latency`, dynamics-
aware scheduler, scenario window 720 s, step 0.1 s), real COM backend,
all records reportable. Numbers are means over the five runs; ranges in
brackets.

| | Campaign A (2026-09-30) | Campaign B (2026-10-05) |
|---|---|---|
| Measured total per step | 927.4 ms (861–981) | **658.1 ms (599–703)** |
| Sense | 891.6 ms (830–939) | **621.5 ms (570–663)** |
| Actuate | 18.7 ms (16.3–22.4) | **18.9 ms (14.4–21.1)** |
| Barrier wait | 13.0 ms (11.6–15.0) | **13.0 ms (11.1–14.1)** |
| Wall time per run (scheduler) | 104–118 min | 72–85 min |
| Captured / attempted | 4 / 5 (T3 missed) | **5 / 5** |
| Not attempted (declined) | T5, T7 | T5, T7 |
| Peak wheel momentum (fraction, per axis) | 0.62, 0.63, 0.13 | 0.52, 0.50, 0.08 |
| Actuator-limiting episodes | 8 | 3 |
| SAFE entered | no | no |
| Runs bit-identical (commands and twin status) | yes, 5/5 | yes, 5/5 |

Reading the two campaigns:

- **Campaign B is the result.** Its sense time is lower because the
  bridge reuses STK data-provider handles instead of looking them up on
  every query (`b8edbae`); data are still read fresh at every step. Per
  read in campaign B: orbit ~200–260 ms, body rate ~180–220 ms, attitude
  ~150–190 ms. Sense is ~94% of the step, so the loop's cost is reading
  STK over COM, not ROS 2 or the barrier.
- **Campaign A's capture outcomes are superseded.** It ran before the
  guidance fixes of 10-04: its slews ended ~10° short and settled at the
  torque limit, which is what made T3 miss. Its latency figures remain a
  valid "before the handle cache" measurement; the guidance changes do
  not affect sensing.
- **Determinism.** Within each campaign the five runs produced
  byte-identical command and twin-status streams. This is direct
  evidence for E3's lockstep half; it is not the free-run comparison.
- **Saturation counts** are actuator-limiting episodes (torque or
  momentum clipping, counted on entry), not momentum-capacity hits; peak
  momentum stayed at or below 63% of capacity.
- **End-of-run "barrier abort".** At t = 720 s STK has no data beyond its
  analysis period, the twin cannot acknowledge that step, and the bridge
  logs an ERROR (2 of 5 runs in B). The scheduler has already written its
  record; the error does not affect results.
- **Provenance gaps in both campaigns:** the records do not name the
  code version (B corresponds to `b8edbae`, A to the branch at
  `c80b394`, both inferred from dates, not recorded), and
  `scenario_sha256` is empty because no scenario path was passed. Both
  are recorded from now on (section 9).

## 6. E1 — attitude read-back mismatch (2026-10-08)

One lockstep run per arm, dynamics-aware scheduler, real COM backend.
The treatment reads attitude back through `ECF Attitude Quaternions`
while the twin commands inertial quaternions, with the twin's mismatch
guard disabled (`attitude_mismatch_policy: e1_readback_ablation`); with
the guard on, the same mis-pairing stops the run instead. Both records
carry `188ab52…+dirty`.

| | Control (validated pairing) | Treatment (forced ECF read-back) |
|---|---|---|
| Captured / attempted | **5 / 5** | **0 / 5** |
| Outcomes | all `captured` (T5, T7 declined) | all `no_access_and_not_settled` |
| Pointing error at image | T1 0.0019°, T2 0.0017°, T3 0.107°, T4 0.0016°, T6 0.0016° | 143.4°, 147.0°, 98.6°, 51.2°, 126.1° |
| Actuator-limiting episodes | 3 | 61 |
| Peak wheel momentum (fraction) | 0.52, 0.50, 0.08 | 0.31, 0.46, 0.10 |
| SAFE entered | no | no |
| Step / sense (ms) | 557.4 / 528.1 | 583.0 / 552.5 |

- **The treatment was applied.** The bridge log shows
  `ATTITUDE READ-BACK PROVIDER FORCED to 'ECF Attitude Quaternions'`.
  Independently, the topic captures show STK's read-back differing from
  the command written for the same time by 48.2° (median; 47.6–48.8° over
  the first 300 s). That is the Earth rotation angle at the scenario
  epoch (47.6°), growing at the Earth's rotation rate.
- **Mechanism.** Each step the twin integrates from the attitude it is
  given. Fed the Earth-fixed read-back as if it were inertial, its next
  command is rotated by that angle again, so the commanded attitude
  jumps ~48° every 0.1 s step. The treatment is therefore a runaway, not
  a small bias, and the paper should describe it that way.
- **The control reproduces E6.** Its determinism signature is identical
  to all five E6 campaign-B runs and to E2's dynamics-aware run.
- **Loop health.** Both runs: 0 commands late, rejected or missing. The
  bridge's "attitude not as written" count is 0 in both, but it is not
  evidence for the treatment: that check skips read-backs in a different
  frame from the command, which is every treatment read.
- **Provenance.** The `+dirty` flag counts tracked files only; the change
  was the runbook being edited (committed as `7cc9e39` after the runs).
  The identical signature supports that no code changed, but the records
  are not attributable to a commit. The recorded provider came from the
  scheduler's launch parameter, not from the bridge; it is now observed
  (section 9).
- At t = 720 s both runs log the end-of-window missing body-rate sample
  (section 5); both records were written before it.

## 7. E2 — kinematic vs. dynamics-aware scheduling (2026-10-08)

One lockstep run per scheduler, validated pairing, strict guard, real
COM backend. Both records carry `7cc9e39…+dirty` (the E2 runbook edit,
committed as `363e207` after the runs).

| | Kinematic | Dynamics-aware |
|---|---|---|
| Attempted | 7 | 5 (T5, T7 declined) |
| Captured | 5 | 5 |
| Capture rate | 71.4% | **100%** |
| Coverage (captured / 7) | 71.4% | 71.4% |
| Over-promise gap ((attempted − captured) / attempted) | 28.6% | 0% |
| Failed attempts | T5 `not_settled` (0.29° at image, 0.75° worst in the 2 s hold), T7 `not_settled` (1.03°, 1.07°) | — |
| Actuator-limiting episodes / SAFE | 3 / no | 3 / no |
| Step / sense (ms) | 583.8 / 551.9 | 592.6 / 561.4 |
| Wall time (scheduler) | 70 min 18 s | 71 min 20 s |

- **What it shows.** The kinematic plan promised seven images and the
  closed loop delivered five; the two it lost are exactly the two the
  dynamics-aware plan declined. Capture threshold is 0.5° held for 2 s:
  T5 was inside it at the image epoch but not for the whole hold, i.e.
  still settling. Coverage is unchanged, so the result is fewer failed
  attempts, not more images.
- T1–T4 are bit-identical between the two runs (the schedules agree up
  to T5), and the dynamics-aware run's signature is identical to E1's
  control and to E6 campaign B.
- Loop health: 0 commands late, rejected or missing; no attitude
  mismatches. End-of-window body-rate sample missing at t = 720 s, as in
  section 5.

**Status of E1 and E2.** Both are complete as one run per arm and their
outcomes are consistent with every other run of the same configuration.
Before citing them, re-run both from a clean commit: that makes them
attributable, records the observed read-back provider, and checks the
COM extraction change of `120c0bf` (below) against these signatures.

**Sensing speed-up, not yet verified here.** `120c0bf` reads all column
names of a data-provider result in one COM call instead of two per
column. Its commit message reports a live run going from 70 min 16 s to
24 min 42 s and sense from 553.5 to 171.4 ms with the determinism
signature preserved; that run's bundle has not been reviewed. E1 and E2
ran before the change.

## 8. Experiments not yet run live

| Experiment | Question | Live status |
|---|---|---|
| E1 | Does convention validation matter? (explicit mismatch-guard ablation) | **run** (section 6); re-run from a clean commit before citing |
| E2 | Kinematic vs. dynamics-aware scheduling: capture vs. promised coverage | **run** (section 7); re-run from a clean commit before citing |
| E3 | Lockstep vs. free-run determinism | lockstep half shown by E6 (section 5); free-run half not run |
| E4 | Gain sweep (49 runs) | not run |
| E5 | Wheel-saturation fault injection and exact replay | not run |
| E6 | Loop cost | **done** (campaign B) |

At ~1.2 h per run before `120c0bf`, the full E4 and E5 matrices are days
of machine time; their size should be decided before they start.

## 9. Recorded from now on

- **Code version** (`acav/code_version.py`): the git commit, `+dirty` if
  tracked files were modified, or `unknown (...)`. Written into every run
  record (`provenance.code_version`), every run configuration, the
  campaign manifest and the run's `summary.json`. `run_acav_campaign
  report` lists the versions behind a campaign and warns unless all of
  its records come from one clean commit.
- **Scenario fingerprint**: `scenario_sha256` is now the hash of the file
  the scheduler actually loaded, default path included.
- **Attitude read-back provider, observed.** The bridge reports the
  provider it reads through, and whether it was forced, on
  `BridgeStatus`; the scheduler records it
  (`provenance.attitude_readback_provider`) and derives
  `provenance.attitude_provider` from it rather than from its own launch
  parameter. The campaign runner rejects an E1 record without it.
  `run_acav_run.py` now writes the run's provider into a per-run copy of
  the bridge configuration (`<run>.logs/bridge_params.yaml`), so E1 no
  longer needs hand-made bridge files.

Campaigns that will be cited should therefore be run from a clean
checkout of one commit on `main`.
