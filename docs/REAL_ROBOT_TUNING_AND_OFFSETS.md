# Real Robot Functionalization and Tuning Runbook (Single-Day)

This runbook is the executable checklist for a single long tuning day, assuming:
- drive is already operational,
- intake/hopper/indexers/shooter still need full functionalization,
- beam breaks are intentionally unused (`DIO = -1`),
- shop space with AprilTags is available.

## 0) Preflight and safety

1. Use Java 17 for all Gradle tasks.
   - Example:
     - `export JAVA_HOME=$(/usr/libexec/java_home -v 17)`
     - `./gradlew test build`
2. Put robot on blocks before mechanism and drive characterization work.
3. Assign roles: driver, tuner/logger, safety operator on E-stop.
4. Connect AdvantageScope or Glass to NetworkTables.
5. Confirm root table is `Spartobots2026`.

## 1) One-time code-side defaults for this session

These should already be set in code before the tuning session starts:

1. Enable temporary mechanism bring-up controls in:
   - `src/main/java/frc/robot/RobotContainer.java`
   - `ENABLE_MECHANISM_BRINGUP_BINDINGS = true`
2. Enable sensorless auto-hold fallback in:
   - `src/main/java/frc/robot/subsystems/body/GamePieceManagerConstants.java`
   - `sensorlessCollectToHoldSeconds = 0.45`

## 2) Active operator controls during bring-up

With `ENABLE_MECHANISM_BRINGUP_BINDINGS = true`, bindings are:

- Drive:
  - Left stick = translation
  - Right stick X = rotation
- Shooter:
  - Right trigger (hold) = shooter demand
  - POV up/down (hold) = hood manual jog
- Feed manager:
  - Left trigger (hold) = manual feed
  - `Y` (hold) = collect with indexers
  - `X` (hold) = collect without indexers
  - `A` (hold) = reverse
  - `B` (press) = set `IDLE`
- Auto assist:
  - Left bumper = cancel active auto-assist
  - Right bumper = auto drive under trench
- Temporary bring-up:
  - POV left/right (hold) = intake pivot jog
  - Back/start (hold) = hopper extension jog

Note: while bring-up mode is enabled, paddle remap triggers are intentionally disabled to avoid
button conflicts with Back/Start.

## 2.5) Recommended single-day timeline

1. 45 min: preflight software gate (`test/build`, deploy baseline).
2. 30 min: safety + electrical baseline gate.
3. 90 min: mechanism direction and endpoint calibration.
4. 60 min: open-loop body bring-up.
5. 75 min: shooter closed-loop tuning.
6. 30 min: sensorless collect/hold tuning.
7. 60 min: vision + hub alignment validation.
8. 75 min: drive characterization + module/path tuning.
9. 60 min: integrated teleop + auto validation.
10. 35 min: stabilization, disable bring-up bindings, redeploy, regression pass.

## 3) Safety and electrical baseline gate

Pass this gate before mechanism tuning:

1. No persistent module disconnect alerts:
   - drive motor alerts
   - turn motor alerts
2. No persistent gyro disconnect alert.
3. No persistent camera disconnect alerts (`Vision camera 0/1`).
4. All expected CAN devices present and stable under enable/disable cycles.

## 4) Calibration pass (offsets/endpoints first)

Complete all calibration before PID tuning.

### 4.1 Swerve azimuth zero offsets

Files:
- `src/main/java/frc/robot/subsystems/drive/DriveConstants.java`

Procedure:
1. Disable robot.
2. Physically align all wheels straight forward.
3. Read module turn positions from AdvantageScope logs.
4. Update:
   - `frontLeftZeroRotation`
   - `frontRightZeroRotation`
   - `backLeftZeroRotation`
   - `backRightZeroRotation`
5. Deploy and verify each module reads near `0 rad` when physically straight.

### 4.2 Intake pivot calibration

Tuning keys:
- `/Spartobots2026/Subsystems/Body/Intake/Tuning/Common/Pivot/Calibration/RetractedPositionRotations`
- `/Spartobots2026/Subsystems/Body/Intake/Tuning/Common/Pivot/Calibration/ExtendedPositionRotations`

Telemetry keys:
- `/Spartobots2026/Subsystems/Body/Intake/Telemetry/Pivot/EncoderPositionRotations`
- `/Spartobots2026/Subsystems/Body/Intake/Telemetry/Pivot/EncoderPositionNormalized`

Procedure:
1. Enable robot.
2. Hold POV left to retract to hard stop, record rotations, write retracted key.
3. Hold POV right to extend to hard stop, record rotations, write extended key.
4. Confirm normalized telemetry is near `0.0` at retract and `1.0` at extend.

### 4.3 Hopper extension calibration

Tuning keys:
- `/Spartobots2026/Subsystems/Body/Hopper/Tuning/Common/Extension/Calibration/RetractedPositionRotations`
- `/Spartobots2026/Subsystems/Body/Hopper/Tuning/Common/Extension/Calibration/ExtendedPositionRotations`

Telemetry keys:
- `/Spartobots2026/Subsystems/Body/Hopper/Telemetry/Extension/EncoderPositionRotations`
- `/Spartobots2026/Subsystems/Body/Hopper/Telemetry/Extension/EncoderPositionNormalized`

Procedure:
1. Hold Back to retract to hard stop, record rotations, write retracted key.
2. Hold Start to extend to hard stop, record rotations, write extended key.
3. Confirm normalized telemetry is near `0.0` at retract and `1.0` at extend.

### 4.4 Shooter hood calibration

Tuning keys:
- `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Common/Hood/Calibration/RetractedPositionRotations`
- `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Common/Hood/Calibration/ExtendedPositionRotations`

Telemetry keys:
- `/Spartobots2026/Subsystems/Body/Shooter/Telemetry/Hood/EncoderPositionRotations`
- `/Spartobots2026/Subsystems/Body/Shooter/Telemetry/Hood/MeasuredAngleFromFloorDegrees`
- `/Spartobots2026/Subsystems/Body/Shooter/Telemetry/Hood/SetpointErrorDegrees`

Procedure:
1. Hold POV down to minimum physical hood position, write retracted key.
2. Hold POV up to maximum physical hood position, write extended key.
3. Verify angle and normalized readings are monotonic and stable.

## 5) Open-loop body bring-up

Tune signs/scales under:
- Intake common tuning:
  - `/Spartobots2026/Subsystems/Body/Intake/Tuning/Common/Drive/Speed`
  - `/Spartobots2026/Subsystems/Body/Intake/Tuning/Common/Drive/Direction`
  - `/Spartobots2026/Subsystems/Body/Intake/Tuning/Common/Pivot/SpeedScale`
  - `/Spartobots2026/Subsystems/Body/Intake/Tuning/Common/Pivot/Inverted`
- Hopper common tuning:
  - `/Spartobots2026/Subsystems/Body/Hopper/Tuning/Common/Agitator/Speed`
  - `/Spartobots2026/Subsystems/Body/Hopper/Tuning/Common/Agitator/Direction`
  - `/Spartobots2026/Subsystems/Body/Hopper/Tuning/Common/Extension/SpeedScale`
  - `/Spartobots2026/Subsystems/Body/Hopper/Tuning/Common/Extension/Inverted`
- Indexers common tuning:
  - `/Spartobots2026/Subsystems/Body/Indexers/Tuning/Common/Top/Speed`
  - `/Spartobots2026/Subsystems/Body/Indexers/Tuning/Common/Top/SpeedScale`
  - `/Spartobots2026/Subsystems/Body/Indexers/Tuning/Common/Top/Direction`
  - `/Spartobots2026/Subsystems/Body/Indexers/Tuning/Common/Bottom/Speed`
  - `/Spartobots2026/Subsystems/Body/Indexers/Tuning/Common/Bottom/SpeedScale`
  - `/Spartobots2026/Subsystems/Body/Indexers/Tuning/Common/Bottom/Direction`

Pass condition: no mechanism runs opposite expected direction for collect/feed/reverse.

## 6) Shooter closed-loop tuning

Mode-specific keys:
- Wheels:
  - `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Modes/REAL/PID/Wheels/Velocity/Kp`
  - `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Modes/REAL/PID/Wheels/Velocity/Ki`
  - `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Modes/REAL/PID/Wheels/Velocity/Kd`
  - `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Modes/REAL/PID/Wheels/Velocity/Kv`
- Hood:
  - `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Modes/REAL/PID/Hood/Position/Kp`
  - `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Modes/REAL/PID/Hood/Position/Ki`
  - `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Modes/REAL/PID/Hood/Position/Kd`

Common wheel config:
- `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Common/Wheels/SpeedScale`
- `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Common/Wheels/Pair1Direction`
- `/Spartobots2026/Subsystems/Body/Shooter/Tuning/Common/Wheels/Pair2Direction`

Telemetry:
- `/Spartobots2026/Subsystems/Body/Shooter/Telemetry/Wheels/Pair1VelocityRadPerSec`
- `/Spartobots2026/Subsystems/Body/Shooter/Telemetry/Wheels/Pair2VelocityRadPerSec`
- `/Spartobots2026/Subsystems/Body/Shooter/Telemetry/Wheels/Pair1CommandRadPerSec`
- `/Spartobots2026/Subsystems/Body/Shooter/Telemetry/Wheels/Pair2CommandRadPerSec`
- `/Spartobots2026/Subsystems/Body/Shooter/Telemetry/Hood/SetpointErrorDegrees`

Pass condition:
- wheel steady-state error < 8%
- hood steady-state error < 1.5 deg

## 7) Sensorless collect/hold behavior

Because beam breaks are disabled, tune sensorless fallback timing:

- Constant in code:
  - `GamePieceManagerConstants.sensorlessCollectToHoldSeconds`
- Recommended start:
  - `0.45 s`

Procedure:
1. Run repeated collect cycles with representative game pieces.
2. Verify collect transitions to `HOLD` predictably.
3. Adjust once if needed, redeploy, and re-verify.

## 8) Vision and hub-alignment validation

1. Verify both cameras connected.
2. Validate accepted vision poses are stable near known tag locations.
3. If fusion is biased, tune camera transforms in:
   - `src/main/java/frc/robot/subsystems/vision/VisionConstants.java`
4. Tune align-to-angle keys:
   - `/Spartobots2026/Commands/Drive/Tuning/Modes/REAL/AlignToAngle/Kp`
   - `/Spartobots2026/Commands/Drive/Tuning/Modes/REAL/AlignToAngle/Ki`
   - `/Spartobots2026/Commands/Drive/Tuning/Modes/REAL/AlignToAngle/Kd`
   - `/Spartobots2026/Commands/Drive/Tuning/Modes/REAL/AlignToAngle/Kff`
   - `/Spartobots2026/Commands/Drive/Tuning/Modes/REAL/AlignToAngle/MaxVelocityRadPerSec`
   - `/Spartobots2026/Commands/Drive/Tuning/Modes/REAL/AlignToAngle/MaxAccelerationRadPerSecSquared`

## 9) Drive characterization and path tuning

Run from auto chooser:
- `Drive Simple FF Characterization`
- `Drive Wheel Radius Characterization`

Then tune drive/path keys:
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/Module/DrivePID/Kp`
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/Module/DrivePID/Ki`
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/Module/DrivePID/Kd`
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/Module/TurnPID/Kp`
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/Module/TurnPID/Ki`
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/Module/TurnPID/Kd`
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/PathPlanner/TranslationPID/Kp`
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/PathPlanner/TranslationPID/Ki`
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/PathPlanner/TranslationPID/Kd`
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/PathPlanner/RotationPID/Kp`
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/PathPlanner/RotationPID/Ki`
- `/Spartobots2026/Subsystems/Drive/Tuning/Modes/REAL/PathPlanner/RotationPID/Kd`

Motion compensation shared keys:
- `/Spartobots2026/Tuning/Modes/REAL/HubMotionCompensation/VelocityScale`
- `/Spartobots2026/Tuning/Modes/REAL/HubMotionCompensation/LeadSeconds`

## 10) Integrated validation and autos

Validate end-to-end behavior with:
- teleop collect/feed/reverse loops
- `Hub Opening Shot (Interlocked Feed)`
- `Trench Collect (Timed)`
- `Outpost Collect (Timed)`
- start-match routine

Pass condition: no safety trips, no repeated jams, no command deadlocks, reliable shot cycle.

## 11) Copy-back checklist (required before closeout)

Copy tuned values from live NT/characterization output into code defaults:

1. Drive:
   - `src/main/java/frc/robot/subsystems/drive/DriveConstants.java`
2. Drive command alignment defaults:
   - `src/main/java/frc/robot/commands/DriveCommands.java`
3. Path defaults:
   - `src/main/java/frc/robot/subsystems/drive/Drive.java`
4. Shooter:
   - `src/main/java/frc/robot/subsystems/body/shooter/ShooterConstants.java`
5. Intake/Hopper/Indexers:
   - `src/main/java/frc/robot/subsystems/body/IntakeConstants.java`
   - `src/main/java/frc/robot/subsystems/body/HopperConstants.java`
   - `src/main/java/frc/robot/subsystems/body/IndexersConstants.java`
6. Vision:
   - `src/main/java/frc/robot/subsystems/vision/VisionConstants.java`

After copy-back:
1. Set `ENABLE_MECHANISM_BRINGUP_BINDINGS = false`.
2. Deploy and power-cycle.
3. Run quick regression: drive, collect, shoot, align, one auto.

## 12) Test cases / acceptance criteria

1. TC-01 Connectivity: no persistent drive/turn/gyro/camera disconnect alerts.
2. TC-02 Intake Pivot Calibration: normalized position tracks retract/extend endpoints.
3. TC-03 Hopper Extension Calibration: normalized position tracks retract/extend endpoints.
4. TC-04 Shooter Hood Control: <1.5 deg steady-state hood error, no sustained oscillation.
5. TC-05 Shooter Wheel Velocity: both wheel pairs within 8% steady-state error.
6. TC-06 Sensorless Collect/Hold: collect transitions to hold by timeout reliably.
7. TC-07 Feed/Reverse/Jam Handling: manual feed/reverse works and jam recovery is stable.
8. TC-08 Vision Fusion: accepted vision poses remain stable and improve field pose consistency.
9. TC-09 Drive FF + Wheel Radius: characterization outputs finite values and behavior improves.
10. TC-10 Alignment + Auto: align-to-hub and selected autos complete reliably.
11. TC-11 End-to-End Teleop Cycle: collect -> stage -> spin -> feed -> recover repeats cleanly.
