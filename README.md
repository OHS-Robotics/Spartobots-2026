# Spartobots 2026 (FRC Team 4687)

Java/WPILib robot code for FRC Team 4687 Spartobots' 2026 robot. The project uses command-based WPILib with AdvantageKit logging, PathPlanner pathfinding, PhotonVision camera integration, and MapleSim-based desktop simulation.

## Highlights

- Swerve drive with pose estimation, programmatic autonomous, and field-relative auto-assist targets.
- Game-piece stack made of intake, indexers, and a hooded shooter with homing and calibration tooling.
- Shared NetworkTables and AdvantageKit logging rooted at `Spartobots2026` for tuning, telemetry, simulation, and operator dashboards.

## Quick Start

1. Install WPILib 2026 and Java 17.
2. From the repo root, run:
   - `./gradlew test`
   - `./gradlew simulateJava`
   - `./gradlew deploy`
3. Open AdvantageScope or another NT4 client when tuning or simulating.
4. Use `src/main/java/frc/robot/Constants.java` to choose desktop `SIM` vs `REPLAY`. A real roboRIO always runs in `REAL`.

## Daily Commands

- `./gradlew test` runs unit tests.
- `./gradlew build` builds deployable artifacts.
- `./gradlew simulateJava` launches desktop simulation.
- `./gradlew deploy` deploys robot code plus `src/main/deploy/` assets to the roboRIO.
- `./gradlew replayWatch` opens the AdvantageKit replay utility.

`compileJava` depends on `spotlessApply`, so normal Gradle builds automatically reformat Java and Gradle sources.

## Project Layout

- `src/main/java/frc/robot` robot code, subsystem logic, targeting, simulation, and operator flows.
- `src/test/java/frc/robot` unit tests for autonomous behavior, targeting, interlocks, parsing, and drive logic.
- `src/main/deploy` PathPlanner assets and AdvantageScope robot assets copied during deploy.
- `docs` runbooks and subsystem-level documentation.
- `vendordeps` WPILib/vendor dependency manifests.

## Key Docs

- [Development Workflow](docs/DEVELOPMENT_WORKFLOW.md)
- [Software Overview](docs/SOFTWARE_OVERVIEW.md)
- [Driver Controls](docs/DRIVER_CONTROLS.md)
- [Simulation Workflow](docs/SIMULATION_WORKFLOW.md)
- [NetworkTables Layout](docs/NETWORKTABLES_LAYOUT.md)
- [Real Robot Tuning and Calibration](docs/REAL_ROBOT_TUNING_AND_OFFSETS.md)
- [Shooter Calibration Mode](docs/SHOOTER_CALIBRATION_MODE.md)

## License

Project-specific code in this repository is licensed under the BSD 3-Clause
License as Team 4687 Spartobots code. See `LICENSE` for the repo license and
`THIRD_PARTY_NOTICES.md`, `AdvantageKit-License.md`, and `WPILib-License.md`
for retained third-party attribution and license notices.
