# NetworkTables Layout

All robot-published and robot-read NetworkTables keys are now organized under:

- `Spartobots2026`

## Top-Level Sections

- `Spartobots2026/Subsystems`
- `Spartobots2026/Commands`
- `Spartobots2026/Tuning`

## Conventions

- Subsystem common tuning:
  - `Spartobots2026/Subsystems/<Path>/Tuning/Common/...`
- Subsystem mode-specific tuning:
  - `Spartobots2026/Subsystems/<Path>/Tuning/Modes/<REAL|SIM|REPLAY>/...`
- Subsystem telemetry:
  - `Spartobots2026/Subsystems/<Path>/Telemetry/...`
- Command mode-specific tuning:
  - `Spartobots2026/Commands/<Path>/Tuning/Modes/<REAL|SIM|REPLAY>/...`
- Shared mode-specific tuning:
  - `Spartobots2026/Tuning/Modes/<REAL|SIM|REPLAY>/<Domain>/...`

## Current Robot Paths

- `Subsystems/Drive`
  - Mode tuning in `Tuning/Modes/<mode>`
  - Logging toggle in `Tuning/Common/LogHubAimVector`
- `Commands/Drive`
  - Mode tuning in `Tuning/Modes/<mode>/AlignToAngle/...`
- `Subsystems/Body/Intake`
  - Common tuning + telemetry
- `Subsystems/Body/Hopper`
  - Common tuning + telemetry
- `Subsystems/Body/Indexers`
  - Common tuning + telemetry
- `Subsystems/Body/GamePieceManager`
  - Common tuning (sensor override controls)
- `Subsystems/Body/Shooter`
  - Common tuning + mode PID tuning + telemetry
- Shared domain:
  - `Tuning/Modes/<mode>/HubMotionCompensation/...`
    - Used by both `Drive` and `Shooter`
