# Simulation Workflow

## Prerequisites

- Java 17 or newer is required for all Gradle build, test, and simulation commands.
- Desktop simulation assumes `Constants.simMode` remains `SIM`.
- AdvantageKit logging remains mode-specific:
  - `REAL`: WPILOG + NT4
  - `SIM`: NT4 only
  - `REPLAY`: replay reader + `_sim` writer

## Running Desktop Simulation

1. Install and select Java 17.
2. From the repo root, run `./gradlew simulateJava`.
3. Open AdvantageScope and connect to NetworkTables 4.
4. Use the `Simulation/ResetField` dashboard command to reset the field state without restarting sim.

## AdvantageScope Data To Watch

- `FieldSimulation/RobotPose3d`
- `FieldSimulation/RobotParts/SwerveModules`
- `FieldSimulation/GamePieces/Fuel`
- `Shooter/Simulation/ShotTrajectory`
- `Shooter/Simulation/ActiveFuelProjectiles`
- `GamePieceSimulation/Stage/IntakeOccupied`
- `GamePieceSimulation/Stage/HopperOccupied`
- `GamePieceSimulation/Stage/ShooterOccupied`
- `GamePieceSimulation/Transfers/LastEvent`
- `Vision/Summary/RobotPosesAccepted`
- `Vision/Summary/RobotPosesRejected`
- `Vision/Camera0/Frustum`
- `Vision/Camera1/Frustum`
- `AdvantageScope/Robot/Components`

## AdvantageScope Robot Asset Setup

- Place `model.glb` in `src/main/deploy/advantagescope/assets/Robot_Spartobots2026/`.
- Export articulated meshes to:
  - `model_0.glb` for the intake pivot
  - `model_1.glb` for the hopper extension
  - `model_2.glb` for the shooter hood
- The repo already publishes the matching component poses under `AdvantageScope/Robot/Components`.

## Known Limitations

- The internal conveyor path is modeled as staged transfers, not rigid-body internal physics.
- The repo does not include the final CAD-to-GLB exports; those still need to be generated from team CAD.
- Mechanism masses, inertias, and reductions are estimated defaults and should be tuned against the real robot.
- Local verification still depends on a Java 17 runtime being available on the machine.
