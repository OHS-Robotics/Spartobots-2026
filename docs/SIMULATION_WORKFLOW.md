# Simulation Workflow

## Preconditions

- Java 17 is required for all Gradle build, test, and simulation commands.
- Desktop simulation assumes `Constants.simMode` remains `SIM`.
- AdvantageKit logging stays mode-specific:
  - `REAL`: WPILOG + NT4
  - `SIM`: NT4 only
  - `REPLAY`: replay reader + `_sim` writer

## What Desktop Sim Covers

- Swerve drivetrain on MapleSim's rebuilt 2026 field.
- PhotonVision-based camera simulation.
- Shooter projectile launch, trajectory display, and hub-hit counting.
- Simulated field game pieces and robot pose visualization.
- Robot-model component poses for AdvantageScope assets.
- Wall-impact detection that feeds the driver rumble logic.

## Running Desktop Simulation

1. Verify `src/main/java/frc/robot/Constants.java` sets `simMode = Mode.SIM`.
2. From the repo root, run `./gradlew simulateJava`.
3. Open AdvantageScope and connect to NetworkTables 4.
4. Use the `Simulation/ResetField` dashboard command to reset the field state without restarting sim.
5. Disabling the robot also resets the simulated field state.

## AdvantageScope Data To Watch

- `/Spartobots2026/Simulation/Field/RobotPose`
- `/Spartobots2026/Simulation/Field/RobotPose3d`
- `/Spartobots2026/Simulation/Field/RobotParts/SwerveModules`
- `/Spartobots2026/Simulation/Field/GamePieces/Fuel`
- `/Spartobots2026/Simulation/Field/GamePieces/Note`
- `/Spartobots2026/Simulation/Field/GamePieces/Coral`
- `/Spartobots2026/Simulation/Field/GamePieces/Algae`
- `/Spartobots2026/Simulation/Shooter/ShotTrajectory`
- `/Spartobots2026/Simulation/Shooter/ActiveFuelProjectiles`
- `/Spartobots2026/Simulation/Shooter/ShotsLaunched`
- `/Spartobots2026/Simulation/Shooter/HubHits`
- `/Spartobots2026/Simulation/Shooter/FeedRateRatio`
- `/Spartobots2026/Vision/Summary/RobotPosesAccepted`
- `/Spartobots2026/Vision/Summary/RobotPosesRejected`
- `/Spartobots2026/Simulation/RobotModel/Components`

## AdvantageScope Robot Asset Setup

- Place `model.glb` in `src/main/deploy/advantagescope/assets/Robot_Spartobots2026/`.
- Export articulated meshes to:
  - `model_0.glb` for the intake pivot
  - `model_1.glb` for the hopper extension
  - `model_2.glb` for the shooter hood
- The repo publishes the matching component poses under
  `/Spartobots2026/Simulation/RobotModel/Components`.

## Known Limitations

- The internal conveyor path is modeled as staged transfers, not rigid-body internal physics.
- The repo does not include the final CAD-to-GLB exports; those still need to be generated from team CAD.
- Mechanism masses, inertias, and reductions are estimated defaults and should be tuned against the real robot.
- Replay mode is separate from physics sim. Use `Mode.REPLAY` when analyzing logs rather than running MapleSim.
