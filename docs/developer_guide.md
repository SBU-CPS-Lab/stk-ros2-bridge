# Developer guide

## Layout and layering

```
src/stk_msgs/    ROS 2 interfaces
src/stk_bridge/  backends/ (STK access)  core/ (logic)  interfaces/ (ROS wiring)  nodes/
src/stk_matlab/  +stkros client, +adcs twin, simulink/ model builders
src/acav/        the case study
```

- Only `backends/` may import `agi.stk12`, `win32com`, `pythoncom` or
  Connect socket code. `core/`, `interfaces/` and `nodes/` talk to
  `StkBackend` ([extending backends](bridge/extending_backends.md)).
- Keep logic in ROS-free modules and wire it in a thin node that takes
  its dependencies by injection, so tests need no ROS, STK or MATLAB.
- `acav` modules that decide anything (`scheduling`, `metrics`,
  `campaign`) are pure; `campaign_runner` knows nothing about STK.
- The Python twin reference and `+adcs` must agree; regenerate fixtures
  with `src/acav/scripts/export_twin_fixtures.py` after changing either.

## Tests

```bash
PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider src/acav/test src/stk_bridge/test -q
```

This offline suite (about 1,680 tests) needs no ROS, STK or MATLAB; a
few tests skip outside a sourced ROS 2 workspace. The `code_version`
tests need a git checkout with at least one commit.

MATLAB tests live in `src/stk_matlab/test_*.m` (run from that folder
after `setup_stk_ros`): `test_acav_twin`, `test_simulink_adapter`,
`test_twin_model_runs`, `test_twin_identity`, `test_free_run_twin`,
`test_settle_retarget`, `test_target_rate_reset`, `test_e1_ablation`.
After changing the twin or a `.msg` file, run `test_twin_model_runs`.

Rules for tests: use the mock backend or small fakes; synthetic
fixtures only; a regression test names the behaviour it pins, not the
event that exposed it.

## Comment style

- Write in the present tense and say why, not how the code got here.
  No dates, commit hashes, run names or "we used to".
- Document attributes with `#:` comments; Python docstrings state the
  contract (inputs, outputs, units, what is refused).
- Every MATLAB function starts with an H1 line (`% NAME one-line
  summary`) so `help` works.
- Units go in names or docs (`_m`, `_s`, `_rad`); quaternions are
  `(x, y, z, w)`.

## Things that must not change

- `src/acav/config/acav_scenario.yaml` and
  `config/scenarios/generated/*.yaml`: records name them by SHA-256.
- The twin's code fingerprint covers the `.msg` definitions and twin
  sources; changing them changes the identity the runner checks.

## Known issues

- `CommandInbox(coalesce=False)` silently drops newer commands when
  one is pending.
- `build_twin_template.m` contains a placeholder that must be replaced,
  and the template has not been exercised in MATLAB.
- The builder selects HPOP but does not apply the spacecraft mass or
  drag/SRP areas from the scenario YAML; STK's defaults are used. This
  is the same for every run and arm.
- The MissionMode Stateflow chart is built into the model but not wired
  into the signal path; `+adcs/mission_mode.m` decides the mode.
