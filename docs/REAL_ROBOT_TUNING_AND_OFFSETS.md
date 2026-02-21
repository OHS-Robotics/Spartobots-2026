# Real Robot Tuning and Offset Calibration Runbook

This is a practical order of operations for your current codebase to:
- zero/calibrate all mechanism offsets,
- tune live gains on the robot,
- copy final values back into code so they survive reboots.

## 0) Safety and setup

1. Put the robot on blocks before any drive or shooter gain work.
2. Keep one person on E-stop at all times.
3. Use AdvantageScope (or Glass/Shuffleboard) with NetworkTables connected.
4. On real robot, your tuning keys are under `.../REAL/...` because `Constants.currentMode == REAL`.
5. For swerve calibration, load `AdvantageScope Swerve Calibration.json` from repo root.

## 1) Calibrate all offsets first

Do this before PID tuning.

### 1.1 Swerve azimuth zero offsets (most critical)

Files/constants:
- `src/main/java/frc/robot/subsystems/drive/DriveConstants.java`
- `frontLeftZeroRotation`, `frontRightZeroRotation`, `backLeftZeroRotation`, `backRightZeroRotation`

Procedure:
1. Disable robot.
2. Physically point all 4 wheels straight forward.
3. In AdvantageScope read:
   - `/Drive/Module0/TurnPosition/value` (FL)
   - `/Drive/Module1/TurnPosition/value` (FR)
   - `/Drive/Module2/TurnPosition/value` (BL)
   - `/Drive/Module3/TurnPosition/value` (BR)
4. Update each zero constant using:
   - `newZeroRad = oldZeroRad + measuredTurnPositionRad`
5. Deploy/reboot.
6. Re-check with wheels physically straight: each turn position should now be near `0 rad` (target within about `+/-0.02 rad`).

Notes:
- Module order is FL, FR, BL, BR.
- If module wiring order changes, update `moduleIndexToHardwareIndex` first.

### 1.2 Intake pivot calibration endpoints

Files/constants:
- Runtime NT keys:
  - `/Body/Intake/Tuning/Pivot/Calibration/RetractedPositionRotations`
  - `/Body/Intake/Tuning/Pivot/Calibration/ExtendedPositionRotations`
- Telemetry:
  - `/Body/Intake/Telemetry/Pivot/EncoderPositionRotations`
  - `/Body/Intake/Telemetry/Pivot/EncoderPositionNormalized`
- Defaults in code:
  - `src/main/java/frc/robot/subsystems/body/IntakeConstants.java`

Procedure:
1. Enable robot.
2. Use D-pad left/right (existing bindings) to jog pivot to fully retracted hard stop.
3. Record encoder rotations and write to retracted calibration key.
4. Jog to fully extended hard stop.
5. Record encoder rotations and write to extended calibration key.
6. Confirm normalized position reads near `0` at retracted and near `1` at extended.

### 1.3 Hopper extension calibration endpoints

Files/constants:
- Runtime NT keys:
  - `/Body/Hopper/Tuning/Extension/Calibration/RetractedPositionRotations`
  - `/Body/Hopper/Tuning/Extension/Calibration/ExtendedPositionRotations`
- Telemetry:
  - `/Body/Hopper/Telemetry/Extension/EncoderPositionRotations`
  - `/Body/Hopper/Telemetry/Extension/EncoderPositionNormalized`
- Defaults in code:
  - `src/main/java/frc/robot/subsystems/body/HopperConstants.java`

Procedure is the same pattern as intake:
1. Use `start` (in) / `Y` (out) bindings to hit each mechanical endpoint.
2. Write those values to retracted/extended calibration keys.
3. Verify normalized position goes 0 to 1.

### 1.4 Shooter hood calibration endpoints

Files/constants:
- Runtime NT keys:
  - `/Body/Shooter/Tuning/Hood/Calibration/RetractedPositionRotations`
  - `/Body/Shooter/Tuning/Hood/Calibration/ExtendedPositionRotations`
- Telemetry:
  - `/Body/Shooter/Telemetry/Hood/EncoderPositionRotations`
  - `/Body/Shooter/Telemetry/Hood/MeasuredAngleFromFloorDegrees`
- Defaults in code:
  - `src/main/java/frc/robot/subsystems/body/shooter/ShooterConstants.java`

Procedure:
1. Use D-pad up/down to jog hood to your minimum physical angle.
2. Set retracted calibration value to current hood encoder rotations.
3. Jog to maximum physical angle.
4. Set extended calibration value.
5. Verify measured hood angle and normalized position track correctly across the full range.

### 1.5 Gyro heading convention check

Files/constants:
- `src/main/java/frc/robot/subsystems/drive/GyroIONavX.java`
- `src/main/java/frc/robot/subsystems/drive/DriveConstants.java` -> `navxYawInverted`

Procedure:
1. Boot robot pointed in a known field direction.
2. Rotate robot CCW by hand and confirm yaw increases with your field convention.
3. If sign is wrong, flip `navxYawInverted`.

### 1.6 Vision camera extrinsics

Files/constants:
- `src/main/java/frc/robot/subsystems/vision/VisionConstants.java`
- `robotToCamera0`, `robotToCamera1`

Procedure:
1. Measure camera translations from robot center (meters, WPILib frame).
2. Measure camera pitch/yaw/roll.
3. Update transforms.
4. Verify accepted vision poses overlay odometry correctly at multiple spots on the field.

### 1.7 Shooter frame offsets

Files/constants:
- `src/main/java/frc/robot/subsystems/body/shooter/ShooterConstants.java`
- `shooterFacingOffset`
- `shooterMuzzleOffsetOnRobot` (currently used for simulation projectile spawn)

Procedure:
1. Use align-to-hub behavior and check whether chassis yaw points exactly where the shooter exits.
2. If robot points left of target, decrease `shooterFacingOffset` degrees.
3. If robot points right of target, increase `shooterFacingOffset` degrees.

## 2) Tune gains and runtime parameters

Only start this after offsets are correct.

## 2.1 Drive feedforward and wheel radius characterization

Run from auto chooser:
- `Drive Simple FF Characterization`
- `Drive Wheel Radius Characterization`

Use console output values to update:
- `DriveConstants.driveKs`
- `DriveConstants.driveKv`
- `DriveConstants.wheelRadiusMeters` (if characterization indicates mismatch)

## 2.2 Drive module closed-loop gains (live NT tuning)

Runtime keys:
- `/Drive/Tuning/REAL/Module/DrivePID/Kp`
- `/Drive/Tuning/REAL/Module/DrivePID/Ki`
- `/Drive/Tuning/REAL/Module/DrivePID/Kd`
- `/Drive/Tuning/REAL/Module/TurnPID/Kp`
- `/Drive/Tuning/REAL/Module/TurnPID/Ki`
- `/Drive/Tuning/REAL/Module/TurnPID/Kd`

Recommended order:
1. Turn PID first.
2. Drive PID second.

Practical method:
1. Set `Ki = 0` initially.
2. Increase `Kp` until oscillation starts, then back off 20-30%.
3. Add a small `Kd` to damp overshoot.
4. Add minimal `Ki` only if steady-state error remains.

## 2.3 Path-following PID

Runtime keys:
- `/Drive/Tuning/REAL/PathPlanner/TranslationPID/Kp|Ki|Kd`
- `/Drive/Tuning/REAL/PathPlanner/RotationPID/Kp|Ki|Kd`

Tune with short repeatable autos (line, L, S-curve), then validate with full paths.

## 2.4 Align-to-angle PID

Runtime keys:
- `/Drive/Commands/Tuning/REAL/AlignToAngle/Kp`
- `/Drive/Commands/Tuning/REAL/AlignToAngle/Ki`
- `/Drive/Commands/Tuning/REAL/AlignToAngle/Kd`
- `/Drive/Commands/Tuning/REAL/AlignToAngle/MaxVelocityRadPerSec`
- `/Drive/Commands/Tuning/REAL/AlignToAngle/MaxAccelerationRadPerSecSquared`

Tune while using hub-align behavior under driver translation demand.

## 2.5 Shooter wheel + hood closed-loop gains

Runtime keys:
- Wheels:
  - `/Body/Shooter/Tuning/PID/REAL/Wheels/Velocity/Kp|Ki|Kd|Kv`
  - `/Body/Shooter/Tuning/Wheels/SpeedScale`
  - `/Body/Shooter/Tuning/Wheels/Pair1Direction`
  - `/Body/Shooter/Tuning/Wheels/Pair2Direction`
- Hood:
  - `/Body/Shooter/Tuning/PID/REAL/Hood/Position/Kp|Ki|Kd`

Telemetry to watch:
- `/Body/Shooter/Telemetry/Wheels/Pair1VelocityRadPerSec`
- `/Body/Shooter/Telemetry/Wheels/Pair2VelocityRadPerSec`
- `/Body/Shooter/Telemetry/Wheels/Pair1CommandRadPerSec`
- `/Body/Shooter/Telemetry/Wheels/Pair2CommandRadPerSec`
- `/Body/Shooter/Telemetry/Hood/SetpointErrorDegrees`

Recommended order:
1. Tune wheels first (`Kv`, then `Kp`, then small `Kd`; `Ki` usually zero).
2. Tune hood second (`Kp`, then small `Kd`; add `Ki` only if needed).

## 2.6 Shot motion compensation

Runtime keys:
- `/Body/Shooter/Tuning/MotionCompensation/REAL/VelocityScale`
- `/Body/Shooter/Tuning/MotionCompensation/REAL/LeadSeconds`

Tune by shooting while translating laterally:
- If shots trail robot motion, increase `VelocityScale` or `LeadSeconds`.
- If shots over-lead, decrease them.

## 2.7 Intake/hopper/agitator runtime parameters

These are open-loop settings (not PID):
- Intake:
  - `/Body/Intake/Tuning/Drive/Speed`
  - `/Body/Intake/Tuning/Drive/Direction`
  - `/Body/Intake/Tuning/Pivot/SpeedScale`
  - `/Body/Intake/Tuning/Pivot/Inverted`
- Hopper:
  - `/Body/Hopper/Tuning/Belt/Speed`
  - `/Body/Hopper/Tuning/Belt/Direction`
  - `/Body/Hopper/Tuning/Extension/SpeedScale`
  - `/Body/Hopper/Tuning/Extension/Inverted`
- Agitators:
  - `/Body/Agitators/Tuning/Top/Speed`, `/Top/SpeedScale`, `/Top/Direction`
  - `/Body/Agitators/Tuning/Bottom/Speed`, `/Bottom/SpeedScale`, `/Bottom/Direction`

## 3) Persist tuned values back to code

NetworkTables live tuning is great for iteration, but final comp values should be copied to code defaults:

1. Drive constants:
   - `src/main/java/frc/robot/subsystems/drive/DriveConstants.java`
2. Drive command angle defaults:
   - `src/main/java/frc/robot/commands/DriveCommands.java`
3. Path defaults:
   - `src/main/java/frc/robot/subsystems/drive/Drive.java` (default path PID constants)
4. Shooter defaults:
   - `src/main/java/frc/robot/subsystems/body/shooter/ShooterConstants.java`
5. Intake/hopper/agitator defaults:
   - `src/main/java/frc/robot/subsystems/body/IntakeConstants.java`
   - `src/main/java/frc/robot/subsystems/body/HopperConstants.java`
   - `src/main/java/frc/robot/subsystems/body/AgitatorsConstants.java`
6. Vision transforms:
   - `src/main/java/frc/robot/subsystems/vision/VisionConstants.java`

After copy-back, rebuild/deploy and power-cycle once to confirm robot boots with correct behavior before relying on live NT edits.
