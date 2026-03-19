# Driver Controls

The default layout is a single-controller scheme on the driver controller at USB port 0. The controller at USB port 1 is currently instantiated but not used by the default bindings.

## Default Controls

| Control | Action |
| --- | --- |
| Left stick | Translate the robot |
| Right stick X | Rotate the robot |
| Right stick press | Hold hub auto-align and shooter demand together |
| Right trigger | Hold shooter demand with trigger-based throttle scaling |
| Left trigger | Hold manual feed plus indexers |
| Left bumper | Cancel the active auto-assist |
| Right bumper | Schedule `AutoAssist/DriveUnderTrench` |
| Left stick press | Hold X-lock on the swerve modules |
| `Y` | Hold collect with indexers enabled |
| `X` | Hold collect without indexers |
| `A` | Hold reverse on intake, hopper, and indexers |
| `B` | Stop game-piece flow |
| POV up | Jog hood setpoint up |
| POV down | Jog hood setpoint down |
| POV left | Jog intake pivot inward |
| POV right | Jog intake pivot outward |
| Back | Stop game-piece flow |
| Start | Run intake pivot hard-stop calibration |

## Paddle Shortcuts

When mechanism bring-up bindings are disabled, these raw driver-controller buttons are active:

| Button | Action |
| --- | --- |
| 7 | `AutoAssist/ParkAtLadderL1` |
| 8 | `AutoAssist/DriveToOutpost` |
| 11 | `AutoAssist/AlignToDepot` |

These are intended for controllers with paddle/back-button hardware that exposes those raw button numbers.

## Mechanism Bring-Up Mode

`enableMechanismBringupBindings` in `src/main/java/frc/robot/operator/OperatorBindings.java` is `false` by default.

If you temporarily set it to `true`:

- Back becomes manual hopper/L1 climber retract jog.
- Start becomes manual hopper/L1 climber extend jog.
- The paddle auto-assist shortcuts are disabled to avoid conflicts.

Switch it back to `false` after bring-up if you want the normal competition layout restored.

## Dashboard Actions

SmartDashboard also exposes actions that are useful during testing and tuning:

- `Shooter/HomeHoodToHardStop`
- `Shooter/Calibration/EnableMode`
- `Shooter/Calibration/DisableMode`
- `Shooter/Calibration/RecordSample`
- `Intake/CalibratePivotToHardStops`
- `Drive/WheelRadiusCharacterization`
- `Drive/SimpleFFCharacterization`
- `Drive/SysId/QuasistaticForward`
- `Drive/SysId/QuasistaticReverse`
- `Drive/SysId/DynamicForward`
- `Drive/SysId/DynamicReverse`
- `Simulation/ResetField` in desktop sim only
