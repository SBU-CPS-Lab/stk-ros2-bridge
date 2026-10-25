# acav: the ACAV case study

**Attitude-Coupled Access Verification**, a case study for the
`stk_bridge`: mission planning that assumes instantaneous, perfect
pointing over-promises imaging capacity, and the over-promise is
measurable only in a closed, time-synchronized loop where STK computes
access from the attitude a Simulink ADCS twin actually achieves, not
from an assumed one. `config/acav_scenario.yaml` is the contract every
module reads through `acav.spec.AcavSpec`; nothing else parses it.

The full spec, claim, and honest limitations are in
[`../../docs/acav/case_study.md`](../../docs/acav/case_study.md).
This README covers this package's layout and how to run each part; see
that doc and `docs/matlab/adcs_twin.md` / `docs/acav/scheduler.md`
/ `docs/acav/experiment_harness.md` / `docs/acav/analysis.md` for the design
behind each one.

## Modules

| Area | What it is | Where |
|---|---|---|
| Spec | The spec, slew model, capture metric, scenario YAML | `spec.py`, `slew_model.py`, `metrics.py`, `config/acav_scenario.yaml` |
| Scenario | Build and verify the STK scenario; seeded scenarios | `scenario_builder.py`, `scenario_verifier.py`, `ground_track.py`, `stk_queries.py`, `scenario_generator.py`, `scenarios.py` |
| Twin | The ADCS twin: a tested Python reference plus the MATLAB/Simulink model built from it | `attitude.py`, `twin_reference.py`, `twin_fixtures.py`, `../stk_matlab/+adcs/`, `../stk_matlab/simulink/build_acav_twin.m` |
| Scheduler | The target scheduler (kinematic vs. dynamics-aware) as a ROS 2 node | `scheduling.py`, `capture_tracking.py`, `scheduler_node.py` |
| Harness | Provenance, seeded faults, run matrices, the run orchestrator, the campaign CLI | `run_record.py`, `fault_injection.py`, `campaign.py`, `campaign_runner.py`, `campaign_cli.py`, `e2_seeds.py`, `deploy_run_cli.py`, `twin_launcher.py`, `stk_process.py`, `console.py` |
| Analysis | Reports and figures from a campaign directory | `report_figures.py`, `report_style.py`, `paired_analysis.py`, `coupling_analysis.py`, wired into `campaign_cli.py`'s `report` verb |

The experiments are E1 (interface mismatches), E2 (kinematic against
dynamics-aware scheduling), E3 (lockstep against free-run, with seeded
fault replay) and E6 (loop cost); see the
[case study](../../docs/acav/case_study.md) section 9. How to run each:
[running campaigns](../../docs/acav/running_campaigns.md).

## Running it offline (no STK, no MATLAB, no ROS)

```
cd stk-ros2-bridge
python -m pytest src/acav/test src/stk_bridge/test -q -p no:cacheprovider -rs
```

`test/conftest.py` puts this package on `sys.path`, so this needs no
colcon build. This is the fast loop for everything in `acav/` except
`scheduler_node.py` and `campaign_runner.CommandLauncher`'s actual
subprocess behaviour, which need ROS 2 sourced (or
`osrf/ros:humble-desktop-full` in Docker, which is where this project's
ROS-dependent checks can run on a machine with no ROS 2 install).

## Running it against real STK

```powershell
# With STK Desktop started by you: build the scenario and verify it.
python scripts/build_acav_scenario.py --backend com --verify
# A seeded scenario instead of the reference:
python scripts/build_acav_scenario.py --backend com --verify --scenario config/scenarios/generated/acav_seed1.yaml

# Print the scenario's target-by-target feasibility table.
ros2 run acav describe_scenario

# Regenerate the MATLAB-side fixtures after editing anything twin_reference.py
# or twin_step.m are supposed to agree on:
python scripts/export_twin_fixtures.py
```

Then, in MATLAB (see [ADCS twin](../../docs/matlab/adcs_twin.md) section 6 for the tests):

```matlab
cd src\stk_matlab
setup_stk_ros
test_acav_twin
build_acav_twin
```

## Running a scheduler by itself

```
ros2 run acav acav_scheduler --ros-args \
    -p scheduler_type:=dynamics_aware \
    -p output_json:=/tmp/run.json
```

Needs a bridge running against the same scenario (see
`config/bridge_params_acav.yaml` for the ACAV-specific bridge config —
its header lists every difference from `stk_bridge`'s own default,
among them `acav_scheduler` as a second lockstep barrier participant) and, for a
lockstep run, the MATLAB twin acking the same barrier. See
`docs/acav/scheduler.md` for the full parameter list.

## Running a campaign

```
ros2 run acav run_acav_campaign plan   --experiment E6 --output-dir <folder> --repeats <N>
ros2 run acav run_acav_campaign run    --experiment E6 --output-dir <folder> --repeats <N> --twin-launch matlab-batch -- <run command>
ros2 run acav run_acav_campaign report --experiment E6 --output-dir <folder> --repeats <N>
```

The run command is `scripts/run_acav_run.py` with the campaign's
`{placeholders}`; the exact, checked commands for each experiment are in
[`../../docs/acav/running_campaigns.md`](../../docs/acav/running_campaigns.md)
section 4. `report` needs no STK, no MATLAB and no ROS. A single-folder
report writes `<folder>/figures/*.{pdf,svg,png}` unless `--no-figures`
is given (it degrades to JSON-only if `matplotlib` is not installed);
the E2 paired report (`--include-dir`) writes no figures. The JSON is
printed, and written to a file only with `--json PATH`. E2 over the
seeded scenarios runs in one command, `run_acav_campaign run-seeds`
([running campaigns](../../docs/acav/running_campaigns.md) section 7.4).

## Optional dependencies

```
pip install 'acav[figures]'
```

installs `matplotlib`, needed only by `run_acav_campaign report`'s
figure generation. It is deliberately not a hard dependency: the
deploy machine running `acav_scheduler` and `run_acav_run.py` against
live STK has no reason to pull in a plotting stack it never uses.

## Layout

```
acav/                  library code, see the table above
config/
  acav_scenario.yaml       the reference scenario, read through AcavSpec
                            (must not change: records name it by digest)
  scenarios/generated/     seeded scenarios acav_seed1..10.yaml (generator v2)
  bridge_params_acav.yaml  stk_bridge config for this study (adds acav_scheduler
                            as a lockstep barrier participant)
scripts/                thin shims so acav's console_scripts can run out of a
                        source checkout with no colcon build (see build_cli.py's
                        own docstring for why the split)
test/                   pytest, no ROS needed for anything except scheduler_node.py
                        and campaign_runner.py's real subprocess behaviour
```
