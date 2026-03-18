# Shooter Calibration Mode

This mode lets the shooter run fixed hood and wheel setpoints while you collect real-shot data. It is intended for mechanism characterization, not normal hub auto-targeting.

When calibration mode is enabled, the shooter uses the configured calibration setpoints instead of the normal hub-solver output.

## Dashboard actions

SmartDashboard actions:

- `Shooter/Calibration/EnableMode`
- `Shooter/Calibration/DisableMode`
- `Shooter/Calibration/RecordSample`
- `Shooter/HomeHoodToHardStop`

## NetworkTables controls

Path root:

- `/Spartobots2026/GamePiece/Shooter/Tuning/Common/Calibration`

Set these before a shot:

- `Enabled`
- `Hood/SetpointRotations`
- `Wheels/Pair1SetpointRadPerSec`
- `Wheels/Pair2SetpointRadPerSec`
- `Measurement/DistanceMeters`
- `Measurement/HeightDeltaMeters`
- `Measurement/AirtimeSeconds`
- `Measurement/VideoLaunchAngleDegrees`
- `Measurement/Notes`

Notes:

- `DistanceMeters` is horizontal distance from release point to the target plane you are measuring.
- `HeightDeltaMeters` is `target height - launch height`.
- `AirtimeSeconds` should be time to the target plane, not time until floor impact.
- `VideoLaunchAngleDegrees` is optional. Leave it as `NaN` if you are deriving angle from distance plus airtime instead.

## Workflow

1. Run `Shooter/HomeHoodToHardStop` so the hood starts from a known reference.
2. Enable calibration mode from SmartDashboard or by setting `Enabled = true`.
3. Enter fixed hood and wheel setpoints in the calibration table.
4. Fire using the normal shooter-demand flow.
5. Measure distance and airtime for the shot, then update the measurement fields.
6. Press `Shooter/Calibration/RecordSample` to snapshot the configured and measured values.
7. Repeat across hood positions and wheel speeds.

## Recorded output

Telemetry is published under:

- `/Spartobots2026/GamePiece/Shooter/Telemetry/Calibration`

Useful entries include:

- current mode-enabled state
- configured hood and wheel setpoints
- configured measurement fields
- `RecordedSampleCount`
- `LastRecorded/...` for the most recent captured sample

The last recorded sample includes:

- commanded hood and wheel setpoints
- measured hood and wheel values at record time
- estimated launch speed from measured wheel speed
- raw measurement fields
- derived horizontal velocity
- derived vertical velocity
- derived launch speed
- derived launch angle

Derived ballistic estimate from distance and airtime:

```text
vx = distance / airtime
vy = (heightDelta + 0.5 * g * airtime^2) / airtime
launchSpeed = hypot(vx, vy)
launchAngle = atan2(vy, vx)
```

## Expected Use

- For fastest tuning, build an empirical lookup table from distance to hood rotations and wheel
  speed.
- For a physics-backed model, use the recorded samples to fit `hood rotations -> exit angle` and
  `wheel speed -> launch speed`.
