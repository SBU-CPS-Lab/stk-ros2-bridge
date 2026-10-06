# acav — the ACAV case study

**Attitude-Coupled Access Verification**, the worked case study for the
`stk_bridge` paper: mission planning that assumes instantaneous, perfect
pointing over-promises imaging capacity, and the over-promise is
measurable only in a closed, time-synchronized loop where STK computes
access from the attitude a Simulink ADCS twin actually achieves, not
from an assumed one. `config/acav_scenario.yaml` is the contract every
phase below reads through `acav.spec.AcavSpec` — no phase parses it
itself.

The full spec, claim, and honest limitations are in
[`../../docs/C1_case_study_spec.md`](../../docs/C1_case_study_spec.md).
This README covers only this package's layout and how to run each
phase; see that doc and `docs/C3_adcs_twin.md` / `docs/C4_scheduler.md`
/ `docs/C5_experiment_harness.md` / `docs/C6_analysis.md` for the design
behind each one.

## Phases

| Phase | What it is | Where |
|---|---|---|
| C1 | The spec, slew model, capture metric, scenario YAML | `spec.py`, `slew_model.py`, `metrics.py`, `config/acav_scenario.yaml` |
| C2 | Build and verify the STK scenario | `scenario_builder.py`, `scenario_verifier.py`, `ground_track.py`, `stk_queries.py` |
| C3 | The ADCS twin: a tested Python reference plus the MATLAB/Simulink model built from it | `attitude.py`, `twin_reference.py`, `twin_fixtures.py`, `../stk_matlab/+adcs/`, `../stk_matlab/simulink/build_acav_twin.m` |
| C4 | The target scheduler (kinematic vs. dynamics-aware) as a ROS 2 node | `scheduling.py`, `capture_tracking.py`, `scheduler_node.py` |
| C5 | The experiment harness: provenance, seeded faults, run matrices, the campaign CLI | `run_record.py`, `fault_injection.py`, `campaign.py`, `campaign_runner.py`, `campaign_cli.py` |
| C6 | Analysis and figures from a campaign directory | `report_figures.py`, `report_style.py`, wired into `campaign_cli.py`'s `report` verb |

All six are implemented and offline-tested (660 tests, this package
alone). **Experiment E6 has been run live**; E1–E5 have not. Every live
run and its result is recorded in
[`../../docs/09_live_runs_and_results.md`](../../docs/09_live_runs_and_results.md);
how to run a campaign is in
[`../../docs/08_deploy_runbook.md`](../../docs/08_deploy_runbook.md).

## Running it offline (no STK, no MATLAB, no ROS)

```
cd stk_ros2_ws
python -m pytest src/acav/test -q
```

`test/conftest.py` puts this package on `sys.path`, so this needs no
colcon build. This is the fast loop for everything in `acav/` except
`scheduler_node.py` and `campaign_runner.CommandLauncher`'s actual
subprocess behaviour, which need ROS 2 sourced (or
`osrf/ros:humble-desktop-full` in Docker, which is where this project's
own ROS-dependent checks run on a machine with no ROS 2 install).

## Running it against real STK

```powershell
# Build the scenario, and check it was built correctly.
python scripts/build_acav_scenario.py --backend com
python scripts/verify_acav_scenario.py --backend com

# Print the scenario's target-by-target feasibility table.
ros2 run acav describe_scenario

# Regenerate the MATLAB-side fixtures after editing anything twin_reference.py
# or twin_step.m are supposed to agree on:
python scripts/export_twin_fixtures.py
```

Then, in MATLAB (see `docs/C3_adcs_twin.md` section 6 for the full
walkthrough):

```matlab
cd stk_ros2_ws\src\stk_matlab
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
it differs from `stk_bridge`'s own default only in adding
`acav_scheduler` as a second lockstep barrier participant) and, for a
lockstep run, the MATLAB twin acking the same barrier. See
`docs/C4_scheduler.md` for the full parameter list.

## Running a campaign (E1-E6)

```
ros2 run acav run_acav_campaign plan   --experiment E3 --output-dir results/E3
ros2 run acav run_acav_campaign run    --experiment E3 --output-dir results/E3 -- \
    python scripts/run_acav_run.py --run-id {run_id} --experiment {experiment} \
        --scheduler {scheduler} --execution-mode {execution_mode} --seed {seed} \
        --output {output} --scenario {scenario} --backend {backend} \
        --attitude-provider {attitude_provider} --run-config {run_config} \
        --gains-json {gains_json} --fault-json {fault_json} \
        --bridge-config config/bridge_params_acav.yaml
ros2 run acav run_acav_campaign report --experiment E3 --output-dir results/E3
```

`report` needs no STK, no MATLAB and no ROS — it is meant to run
wherever the analysis actually happens, and writes
`results/E3/figures/*.{pdf,svg,png}` unless `--no-figures` is given (it
degrades to JSON-only if `matplotlib` is not installed — see below).
See [`../../docs/08_deploy_runbook.md`](../../docs/08_deploy_runbook.md)
for the full walkthrough of running this on the Windows machine with
STK, including how the MATLAB handoff works and what logs to capture.

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
acav/                  library code -- one module per phase, see the table above
config/
  acav_scenario.yaml       the C1-C6 contract every phase reads through AcavSpec
  bridge_params_acav.yaml  stk_bridge config for this study (adds acav_scheduler
                            as a lockstep barrier participant)
scripts/                thin shims so acav's console_scripts can run out of a
                        source checkout with no colcon build (see build_cli.py's
                        own docstring for why the split)
test/                   pytest, no ROS needed for anything except scheduler_node.py
                        and campaign_runner.py's real subprocess behaviour
```
