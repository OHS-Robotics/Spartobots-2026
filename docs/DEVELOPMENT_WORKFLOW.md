# Development Workflow

## Toolchain

- WPILib 2026 with GradleRIO dependencies installed.
- Java 17 for build, test, simulation, and deploy tasks.
- Git available in `PATH` if you want real `BuildConstants` metadata instead of the fallback generator.
- AdvantageScope recommended for NT4, AdvantageKit logs, and sim visualization.

## Core Commands

- `./gradlew test`
  Runs the unit-test suite.
- `./gradlew build`
  Produces the robot JAR and deploy artifacts.
- `./gradlew simulateJava`
  Starts desktop simulation.
- `./gradlew deploy`
  Deploys robot code and `src/main/deploy` assets to the roboRIO.
- `./gradlew replayWatch`
  Starts the AdvantageKit replay tool.

## Runtime Modes

`RobotBase.isReal()` forces `REAL` on the roboRIO. Desktop mode comes from `Constants.simMode` in `src/main/java/frc/robot/Constants.java`:

- `SIM`
  Physics simulation with NT4 logging.
- `REPLAY`
  AdvantageKit replay input plus a sibling `_sim` log writer.

Switch `Constants.simMode` before running desktop tasks.

## Recommended Loop

1. Run `./gradlew test` before pushing or deploying.
2. Use `./gradlew simulateJava` to validate non-hardware logic and field interactions.
3. Deploy with `./gradlew deploy` once the robot is on the correct network and the team number is configured.
4. After live tuning, copy permanent values back into code or your chosen persistence workflow and re-test.

## Deploy Notes

- Team number comes from `.wpilib/wpilib_preferences.json` or Gradle command-line overrides.
- Deploy copies everything under `src/main/deploy` to `/home/lvuser/deploy`.
- If the current branch name starts with `event`, deploy tasks stage all changes and attempt an automatic commit through the `eventDeploy` task.
- If Git is unavailable, the build generates a fallback `BuildConstants.java` with `UNKNOWN` Git metadata.

## Formatting Behavior

`compileJava` depends on `spotlessApply`, so `test`, `build`, and `deploy` may rewrite formatting in `.java` and `.gradle` files. Check `git diff` after running Gradle if you want to review the formatter changes separately.

## Working With Sim and Logs

- Desktop sim publishes NT4 data only.
- Replay mode disables real-time timing and writes a sibling `*_sim.wpilog`.
- In `SIM`, the field resets on startup and on disable. The SmartDashboard action `Simulation/ResetField` performs the same reset manually.

## Related Docs

- [Software Overview](SOFTWARE_OVERVIEW.md)
- [Driver Controls](DRIVER_CONTROLS.md)
- [Simulation Workflow](SIMULATION_WORKFLOW.md)
- [Real Robot Tuning and Calibration](REAL_ROBOT_TUNING_AND_OFFSETS.md)
