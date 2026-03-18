# Real Robot Tuning and Calibration Runbook

This runbook is the hardware-side checklist for taking the current codebase from first power-on through a field-ready tuning pass.

Assumptions:

- Java 17 is installed.
- Drive, cameras, and game-piece mechanisms are wired to the IDs in the subsystem constants.
- Beam-break-based staging is not implemented; collect/feed behavior is currently sensorless and open-loop.
- Shop space with AprilTags is available for vision validation.

## 1. Preflight

1. Run `./gradlew test build`.
2. Put the robot on blocks before mechanism and drive characterization work.
3. Assign roles: driver, tuner/logger, and safety operator on the E-stop.
4. Connect AdvantageScope or Glass to NT4.
5. Confirm the root table is `/Spartobots2026`.
6. Verify the robot is actually running on hardware. `Constants.simMode` only affects desktop sessions.

## 2. Controls During Tuning

Default tuning uses a single controller on USB 0. The most relevant bring-up controls are:

- Left stick: translation
- Right stick X: rotation
- Right stick press: hub auto-align plus shooter demand
- Right trigger: shooter demand
- Left trigger: manual feed plus indexers
- `Y`: collect with indexers
- `X`: collect without indexers
- `A`: reverse intake/hopper/indexers
- `B`: stop game-piece flow
- POV up/down: hood setpoint jog
- POV left/right: intake pivot jog
- Back: stop game-piece flow
- Start: intake pivot hard-stop calibration

See [Driver Controls](DRIVER_CONTROLS.md) for the full map, including paddle shortcuts and dashboard actions.

If you need direct hopper/L1 climber extension jog, temporarily set `enableMechanismBringupBindings = true` in `src/main/java/frc/robot/operator/OperatorBindings.java`, redeploy, and use Back/Start for retract/extend jog. That mode disables the paddle shortcuts on purpose.

## 3. Safety and Electrical Gate

Pass this gate before mechanism tuning:

1. No persistent swerve drive or turn disconnect alerts.
2. No persistent gyro disconnect alert.
3. No persistent vision camera disconnect alerts.
4. All expected CAN devices stay present through repeated enable/disable cycles.
5. Shooter hood, intake pivot, and hopper/L1 climber all move freely without binding.

## 4. Calibration First

Do offsets and hard-stop calibration before tuning closed loops.

### 4.1 Swerve Azimuth Zero Offsets

File:

- `src/main/java/frc/robot/subsystems/drive/DriveConstants.java`

Procedure:

1. Disable the robot.
2. Physically align all wheels straight forward.
3. Read the module turn positions in logs.
4. Update:
   - `frontLeftZeroRotation`
   - `frontRightZeroRotation`
   - `backLeftZeroRotation`
   - `backRightZeroRotation`
5. Redeploy and verify each module reports near `0 rad` when physically straight.

### 4.2 Intake Pivot Calibration

Tuning keys:

- `/Spartobots2026/GamePiece/Intake/Tuning/Common/Pivot/Calibration/RetractedPositionRotations`
- `/Spartobots2026/GamePiece/Intake/Tuning/Common/Pivot/Calibration/ExtendedPositionRotations`

Telemetry keys:

- `/Spartobots2026/GamePiece/Intake/Telemetry/Pivot/EncoderPositionRotations`
- `/Spartobots2026/GamePiece/Intake/Telemetry/Pivot/EncoderPositionNormalized`

Procedure:

1. Enable the robot.
2. Run `Intake/CalibratePivotToHardStops` from SmartDashboard or press Start.
3. If calibration fails, use POV left/right to confirm direction and clear any obstruction, then rerun.
4. Confirm normalized telemetry is near `0.0` at retract and `1.0` at extend.

### 4.3 Hopper/L1 Climber Extension Calibration

Tuning keys:

- `/Spartobots2026/GamePiece/Hopper/Tuning/Common/Extension/Calibration/RetractedPositionRotations`
- `/Spartobots2026/GamePiece/Hopper/Tuning/Common/Extension/Calibration/ExtendedPositionRotations`

Telemetry keys:

- `/Spartobots2026/GamePiece/Hopper/Telemetry/Extension/EncoderPositionRotations`
- `/Spartobots2026/GamePiece/Hopper/Telemetry/Extension/EncoderPositionNormalized`

Procedure:

1. If needed, temporarily enable mechanism bring-up bindings.
2. Jog the extension to the retracted hard stop and record the encoder value into the retracted key.
3. Jog the extension to the extended hard stop and record the encoder value into the extended key.
4. Confirm normalized telemetry is near `0.0` at retract and `1.0` at extend.

### 4.4 Shooter Hood Homing and Range Validation

Tuning keys:

- `/Spartobots2026/GamePiece/Shooter/Tuning/Common/Hood/Calibration/RetractedPositionRotations`
- `/Spartobots2026/GamePiece/Shooter/Tuning/Common/Hood/Calibration/ExtendedPositionRotations`

Telemetry keys:

- `/Spartobots2026/GamePiece/Shooter/Telemetry/Hood/EncoderPositionRotations`
- `/Spartobots2026/GamePiece/Shooter/Telemetry/Hood/MeasuredAngleFromFloorDegrees`
- `/Spartobots2026/GamePiece/Shooter/Telemetry/Hood/SetpointErrorDegrees`

Procedure:

1. Run `Shooter/HomeHoodToHardStop` from SmartDashboard.
2. Confirm the hood-homing status outputs report a successful homing cycle.
3. Use POV up/down to verify the commanded range is monotonic and stays inside the physical hood limits.
4. Only adjust the hood calibration entries if the encoder reference or mechanical range has changed.

## 5. Open-Loop Mechanism Bring-Up

Verify sign, scale, and direction under these common-tuning paths:

### Intake

- `/Spartobots2026/GamePiece/Intake/Tuning/Common/Drive/Speed`
- `/Spartobots2026/GamePiece/Intake/Tuning/Common/Drive/Direction`
- `/Spartobots2026/GamePiece/Intake/Tuning/Common/Pivot/SpeedScale`
- `/Spartobots2026/GamePiece/Intake/Tuning/Common/Pivot/Inverted`

### Hopper / L1 Climber

- `/Spartobots2026/GamePiece/Hopper/Tuning/Common/Agitator/Speed`
- `/Spartobots2026/GamePiece/Hopper/Tuning/Common/Agitator/Direction`
- `/Spartobots2026/GamePiece/Hopper/Tuning/Common/Extension/SpeedScale`
- `/Spartobots2026/GamePiece/Hopper/Tuning/Common/Extension/Inverted`

### Indexers

- `/Spartobots2026/GamePiece/Indexers/Tuning/Common/Top/Speed`
- `/Spartobots2026/GamePiece/Indexers/Tuning/Common/Top/SpeedScale`
- `/Spartobots2026/GamePiece/Indexers/Tuning/Common/Top/Direction`
- `/Spartobots2026/GamePiece/Indexers/Tuning/Common/Bottom/Speed`
- `/Spartobots2026/GamePiece/Indexers/Tuning/Common/Bottom/SpeedScale`
- `/Spartobots2026/GamePiece/Indexers/Tuning/Common/Bottom/Direction`

Pass condition: nothing runs backward relative to collect, feed, or reverse intent.

## 6. Shooter Closed-Loop Tuning

Mode-specific keys:

### Wheels

- `/Spartobots2026/GamePiece/Shooter/Tuning/Modes/REAL/PID/Wheels/Velocity/Kp`
- `/Spartobots2026/GamePiece/Shooter/Tuning/Modes/REAL/PID/Wheels/Velocity/Ki`
- `/Spartobots2026/GamePiece/Shooter/Tuning/Modes/REAL/PID/Wheels/Velocity/Kd`
- `/Spartobots2026/GamePiece/Shooter/Tuning/Modes/REAL/PID/Wheels/Velocity/Kv`

### Hood

- `/Spartobots2026/GamePiece/Shooter/Tuning/Modes/REAL/PID/Hood/Position/Kp`
- `/Spartobots2026/GamePiece/Shooter/Tuning/Modes/REAL/PID/Hood/Position/Ki`
- `/Spartobots2026/GamePiece/Shooter/Tuning/Modes/REAL/PID/Hood/Position/Kd`

Common wheel configuration:

- `/Spartobots2026/GamePiece/Shooter/Tuning/Common/Wheels/SpeedScale`
- `/Spartobots2026/GamePiece/Shooter/Tuning/Common/Wheels/Pair1Direction`
- `/Spartobots2026/GamePiece/Shooter/Tuning/Common/Wheels/Pair2Direction`

Useful telemetry:

- `/Spartobots2026/GamePiece/Shooter/Telemetry/Wheels/Pair1VelocityRadPerSec`
- `/Spartobots2026/GamePiece/Shooter/Telemetry/Wheels/Pair2VelocityRadPerSec`
- `/Spartobots2026/GamePiece/Shooter/Telemetry/Wheels/Pair1CommandRadPerSec`
- `/Spartobots2026/GamePiece/Shooter/Telemetry/Wheels/Pair2CommandRadPerSec`
- `/Spartobots2026/GamePiece/Shooter/Telemetry/Hood/SetpointErrorDegrees`

Suggested pass targets:

- wheel steady-state error below `8%`
- hood steady-state error below `1.5 deg`

For fixed-setpoint real-shot characterization, use [Shooter Calibration Mode](SHOOTER_CALIBRATION_MODE.md).

## 7. Vision and Hub-Alignment Validation

1. Verify both cameras connect reliably.
2. Validate accepted robot poses around known tag locations.
3. If fusion is biased, tune transforms in `src/main/java/frc/robot/subsystems/vision/VisionConstants.java`.
4. Tune align-to-angle keys:
   - `/Spartobots2026/Drive/Actions/AlignToAngle/Tuning/Modes/REAL/Kp`
   - `/Spartobots2026/Drive/Actions/AlignToAngle/Tuning/Modes/REAL/Ki`
   - `/Spartobots2026/Drive/Actions/AlignToAngle/Tuning/Modes/REAL/Kd`
   - `/Spartobots2026/Drive/Actions/AlignToAngle/Tuning/Modes/REAL/Kff`
   - `/Spartobots2026/Drive/Actions/AlignToAngle/Tuning/Modes/REAL/MaxVelocityRadPerSec`
   - `/Spartobots2026/Drive/Actions/AlignToAngle/Tuning/Modes/REAL/MaxAccelerationRadPerSecSquared`
5. Tune shared motion compensation:
   - `/Spartobots2026/Targeting/Hub/Tuning/Modes/REAL/MotionCompensation/VelocityScale`
   - `/Spartobots2026/Targeting/Hub/Tuning/Modes/REAL/MotionCompensation/LeadSeconds`

## 8. Drive Characterization and Path Tuning

Run these SmartDashboard actions:

- `Drive/WheelRadiusCharacterization`
- `Drive/SimpleFFCharacterization`
- `Drive/SysId/QuasistaticForward`
- `Drive/SysId/QuasistaticReverse`
- `Drive/SysId/DynamicForward`
- `Drive/SysId/DynamicReverse`

Then tune:

- `/Spartobots2026/Drive/Tuning/Modes/REAL/Module/DrivePID/Kp`
- `/Spartobots2026/Drive/Tuning/Modes/REAL/Module/DrivePID/Ki`
- `/Spartobots2026/Drive/Tuning/Modes/REAL/Module/DrivePID/Kd`
- `/Spartobots2026/Drive/Tuning/Modes/REAL/Module/TurnPID/Kp`
- `/Spartobots2026/Drive/Tuning/Modes/REAL/Module/TurnPID/Ki`
- `/Spartobots2026/Drive/Tuning/Modes/REAL/Module/TurnPID/Kd`
- `/Spartobots2026/Drive/Tuning/Modes/REAL/PathPlanner/TranslationPID/Kp`
- `/Spartobots2026/Drive/Tuning/Modes/REAL/PathPlanner/TranslationPID/Ki`
- `/Spartobots2026/Drive/Tuning/Modes/REAL/PathPlanner/TranslationPID/Kd`
- `/Spartobots2026/Drive/Tuning/Modes/REAL/PathPlanner/RotationPID/Kp`
- `/Spartobots2026/Drive/Tuning/Modes/REAL/PathPlanner/RotationPID/Ki`
- `/Spartobots2026/Drive/Tuning/Modes/REAL/PathPlanner/RotationPID/Kd`

## 9. Integrated Validation

Before signing off:

1. Run collect, reverse, manual feed, and shooter spin-up in teleop.
2. Validate hub auto-align from multiple field positions.
3. Verify ladder, depot, trench, and outpost auto-assist actions.
4. Run the competition autonomous on both alliances if field space is available.
5. Disable and re-enable repeatedly to confirm no subsystem boots into a bad state.

## 10. Closeout

1. If you enabled mechanism bring-up bindings, turn them back off and redeploy.
2. Capture final tuned values in code or your persistence workflow. NetworkTables edits are not versioned automatically by this repo.
3. Run `./gradlew test build` one more time before the robot leaves the shop.
