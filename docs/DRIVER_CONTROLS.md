# Driver Controls

The default layout supports either Xbox controller on USB port 0 or 1. Driver translation/rotation inputs and most game-piece controls are mirrored across both controllers. The driver and operator right bumpers are extra collect shortcuts.

## Default Controls

| Control | Action |
| --- | --- |
| Left stick | Translate the robot |
| Right stick X | Rotate the robot |
| Right stick press | Hold hub auto-align and shooter demand together |
| Right trigger | Hold shooter demand with trigger-based throttle scaling |
| Left trigger | Hold manual feed plus indexers; the intake cycles between down and halfway up and pulses the rollers near the down end |
| Left bumper | Cancel the active auto-assist |
| Driver/operator right bumper | Toggle collect without indexers |
| Left stick press | Hold X-lock on the swerve modules |
| `A` | Toggle collect without indexers |
| `B` | Hold reverse on intake and indexers |
| POV up | Jog hood setpoint up |
| POV down | Jog hood setpoint down |
| POV left | Jog intake pivot inward |
| POV right | Jog intake pivot outward |

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
