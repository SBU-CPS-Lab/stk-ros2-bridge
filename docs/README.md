# Documentation

Start with the [root README](../README.md) for what the project is and
how to build it.

| I want to... | Read |
|---|---|
| install STK, ROS 2 and the workspace | [installation](installation.md) |
| understand the bridge | [architecture](bridge/architecture.md), then [design decisions](bridge/design_decisions.md) |
| know the topics and services | [messages](bridge/messages.md) |
| get units, frames and quaternions right | [frames and units](bridge/frames_and_units.md) |
| understand lockstep and free-run | [time synchronization](bridge/time_synchronization.md) |
| fix an STK or run problem | [STK notes](bridge/stk_notes.md) |
| add a backend or capability | [extending backends](bridge/extending_backends.md) |
| write a MATLAB client | [MATLAB client](matlab/client.md) |
| use the ADCS twin | [ADCS twin](matlab/adcs_twin.md) |
| see the ACAV case study | [case study](acav/case_study.md), [scheduler](acav/scheduler.md), [experiment harness](acav/experiment_harness.md), [running campaigns](acav/running_campaigns.md), [analysis](acav/analysis.md) |
| contribute | [developer guide](developer_guide.md) |

The bridge documents (`bridge/`) are independent of the case study; the
MATLAB documents (`matlab/`) cover the client library and the ADCS
twin; `acav/` documents one study built on both.
