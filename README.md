# Spartobots 2026 Usage Guide

This project is a WPILib 2026 Java robot codebase for FRC team `4687`.

## Prerequisites

- WPILib 2026 installed
- Java 17 available on your path
- Xbox controller connected as Driver Station USB device `0`

Check your Java version before running Gradle:

```bash
java -version
```

This project targets Java 17 in [build.gradle](build.gradle).
The simulator readiness gate also assumes Java 17. See [docs/simulator-readiness.md](docs/simulator-readiness.md).

## Real Robot

### 1. Preflight

- Power the roboRIO, radio, motor controllers, NavX, and vision coprocessor/cameras
- Insert a USB stick in the roboRIO if you want AdvantageKit log files saved locally
- Confirm the Driver Station team number is `4687`
- Confirm the Xbox controller is on USB port `0`

The robot expects two PhotonVision camera names from [VisionConstants.java](src/main/java/frc/robot/subsystems/vision/VisionConstants.java):

- `Arducam_OV9281_USB_Camera`
- `Arducam_OV9281_USB_Camera (1)`

### 2. Build and deploy

From the repo root:

```bash
./gradlew build
./gradlew deploy
```

You can also deploy from the WPILib VS Code command palette if that is your normal workflow.

### 3. Driver Station workflow

- Connect to the robot and enable Teleop or Autonomous from Driver Station
- Select the autonomous routine from the dashboard chooser named `Auto Choices`
- Watch logs and NetworkTables values in AdvantageScope if you want field pose, operator feedback, or subsystem status

On real hardware, [Robot.java](src/main/java/frc/robot/Robot.java) publishes:

- NT4 data for live viewing
- WPILOG data to the roboRIO USB stick

### 4. Driver controls

The robot uses a single Xbox controller configured in [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java).

- Left stick: translate the robot
- Right stick X: rotate the robot
- Left trigger: acquire game pieces
- Right trigger: auto-face and score
- Left bumper: outpost feed assist
- `Y`: quick park assist
- `X`: manual override
- `B`: cancel the current assisted action and recover

The default drive command is field-relative swerve drive from [DriveCommands.java](src/main/java/frc/robot/commands/DriveCommands.java).

## Simulation

Simulation is enabled by default on desktop because [Constants.java](src/main/java/frc/robot/Constants.java) sets `simMode = Mode.SIM`.

### 1. Start sim

From the repo root:

```bash
./gradlew simulateJava
```

This project uses:

- MapleSim for the swerve drivetrain and field simulation
- PhotonVision sim for cameras
- AdvantageKit NT4 publishing for live inspection

### 2. What to expect

- The same Xbox controller mapping is used in sim
- The Driver Station sim support is enabled in Gradle
- The WPILib sim GUI is disabled by default in [build.gradle](build.gradle), so the normal way to inspect state is through AdvantageScope/NetworkTables logs
- If you want the WPILib sim GUI, change `wpi.sim.addGui().defaultEnabled = false` to `true` in `build.gradle`
- `FieldSimulation/*` outputs are logged every cycle for robot pose, module poses, and game pieces

### 3. Resetting the sim field

Simulation resets automatically when the robot enters Disabled.

You can also press `Start` on the Xbox controller in sim to reset the field manually. The reset uses the currently selected auto routine so the robot starts from the matching autonomous setup pose.

### 4. Running autos in sim

- Start `simulateJava`
- Open Driver Station sim
- Choose an auto from `Auto Choices`
- Enable Autonomous
- Watch pose, planned behavior, and logged outputs in AdvantageScope

### 5. Simulator readiness checklist

Before moving to the real robot, run the automated tests and the manual simulator rehearsal from [docs/simulator-readiness.md](docs/simulator-readiness.md).
That guide also marks which subsystems are currently `SIM_ONLY` versus `PLACEHOLDER`, so a simulator pass is not mistaken for hardware-fidelity proof.

## Common Issues

### Gradle fails during configuration

If Gradle reports that it is using Java 11, switch to Java 17 first. This project will not configure correctly on Java 11.

### No live logs in sim

Make sure you are connected to the robot program over NT4 with AdvantageScope or another NetworkTables client.

### Vision is missing on the real robot

Confirm the PhotonVision camera names match the values in [VisionConstants.java](src/main/java/frc/robot/subsystems/vision/VisionConstants.java).

## Optional: Log Replay

If you want desktop log replay instead of physics sim, change `simMode` in [Constants.java](src/main/java/frc/robot/Constants.java) from `Mode.SIM` to `Mode.REPLAY`, then run the desktop app against an AdvantageKit log.
