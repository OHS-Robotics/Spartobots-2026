# Software Overview

## Runtime Model

- `Robot` configures AdvantageKit receivers based on mode and then instantiates `RobotContainer`.
- `RobotContainer` selects real vs sim IO implementations, wires services/controllers, registers dashboard actions, and owns the autonomous chooser.
- `Constants.currentMode` is `REAL` on the roboRIO and otherwise follows `Constants.simMode`.

## Major Packages

- `subsystems/drive`
  Swerve modules, odometry, characterization, pathfinding, and drive-side hub compensation.
- `subsystems/gamepiece`
  Intake, indexers, shooter, and their real/sim IO layers.
- `superstructure/gamepiece`
  Collect/feed/reverse orchestration plus shooter-feed interlocks.
- `targeting`
  Hub-shot solution updates and field-relative assist targets such as ladder, depot, and outpost.
- `operator`
  Controller bindings, auto-assist scheduling, dashboard action tracking, and driver feedback.
- `sim`
  MapleSim field updates, projectile simulation, wall-impact feedback, and AdvantageScope robot-model publishing.
- `game`
  FMS game-data parsing for active vs inactive hub state.

## Autonomous and Assists

The dashboard chooser currently exposes:

- `Competition: Outpost -> Shoot -> Ladder`
- `Do Nothing`

The competition routine is generated in code rather than loaded from a committed PathPlanner `.auto` file. It:

1. resets the pose near the alliance outpost start,
2. pathfinds to the opening shot while spinning up the shooter,
3. feeds only when the robot is both ready and inside the shot window,
4. aligns to the ladder at the end.

Current pathing posture:

- Most auto-assist movement relies on runtime `pathfindToPose(...)` calls rather than authored PathPlanner path files.
- `src/main/deploy/pathplanner` currently contains field settings and the navigation grid, but no committed path or auto folder contents.
- Dynamic obstacle support exists in the local AD* wrapper, but the robot code does not currently feed live obstacle updates into pathfinding.

PathPlanner named commands are still registered for future authored paths:

- `collectStart`
- `collectStop`
- `feedStart`
- `feedStop`
- `shooterOn`
- `shooterOff`
- `alignHub`
- `homeHood`
- `calibrateIntakePivot`

## Operator Model

Default teleop is currently a single-driver layout. The driver controller on USB 0 owns almost all live bindings. The second Xbox controller is constructed, but it is not actively used by the default mapping. See [Driver Controls](DRIVER_CONTROLS.md).

## Tuning and Visibility

- Live tuning is split between `Tuning/Common` and `Tuning/Modes/<MODE>` under the `Spartobots2026` root.
- High-value telemetry also appears as AdvantageKit log outputs under the same root, but not every logged path lives inside a `Telemetry` subtree.
- The operator dashboard publishes action state and auto-chooser state through NetworkTables in addition to SmartDashboard actions.
- Path following logs its target pose, translation error, rotation error, and feedforward/feedback/output chassis speeds under `Drive/PathFollowing/...`.

See:

- [NetworkTables Layout](NETWORKTABLES_LAYOUT.md)
- [Shooter Calibration Mode](SHOOTER_CALIBRATION_MODE.md)
- [Simulation Workflow](SIMULATION_WORKFLOW.md)

## Hardware vs Sim

Each subsystem uses an IO abstraction so the same command logic can run on real hardware, in desktop simulation, or in replay-oriented desktop sessions. `RobotContainer` swaps the implementations during construction based on the current mode.
