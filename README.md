# mc_GraspMoveBoxController

FSM controller for biped loco-manipulation with `mc_rtc`.

This project defines a controller plugin named `DemoController` (from `src/lib.cpp`) that extends `BWC::BaselineWalkingController` and runs a scripted scenario:
- walk to box 0 -> pick it up -> carry and drop it,
- walk to box 1 -> pick it up -> carry and drop it.

The main configuration is `etc/DemoController.in.yaml`.

## TODO

- Tune footstep manager (double support ratio, etc)

## What is in this package

- `src/DemoController.*`: controller class inheriting from BaselineWalkingController.
- `src/states/GoTo.*`: footstep-planner based navigation state.
- `src/states/MoveHands.*`: hand motion utility state.
- `src/states/PickupBox.*`: approach/grasp/raise sequence.
- `src/states/HoldBox.*`: keep dual-hand hold while moving.
- `src/states/DropoffBox.*`: lower/release/remove-hands sequence.
- `etc/DemoController.in.yaml`: full FSM graph, robot/environment setup, and task parameters.

## Dependencies

At build time this package expects:
- `mc_rtc`
- `baseline_walking_controller`
- `baseline_footstep_planner`

## Build

If this repository is inside your existing mc_rtc workspace, build it with CMake (example):

```bash
cd /home/martin/workspace
cmake -S sandbox/mc_GraspMoveBoxController -B sandbox/mc_GraspMoveBoxController/build
cmake --build sandbox/mc_GraspMoveBoxController/build -j
cmake --install sandbox/mc_GraspMoveBoxController/build
```

## Enable controller in mc_rtc

Set your mc_rtc controller selection so `Enabled` is `DemoController` in your `mc_rtc.yaml`.

Typical local config location is:
- `~/.config/mc_rtc/mc_rtc.yaml`

Example snippet:

```yaml
Enabled: DemoController
```