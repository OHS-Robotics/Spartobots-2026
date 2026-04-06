# NetworkTables Layout

All robot-published keys live under:

- `/Spartobots2026`

## Shared Structure

For domains that use `NetworkTablesUtil`, the standard sections are:

- `/<root>/<domain>/Tuning/Common/...`
- `/<root>/<domain>/Tuning/Modes/<REAL|SIM|REPLAY>/...`
- `/<root>/<domain>/Telemetry/...`
- `/<root>/<domain>/State/...`
- `/<root>/<domain>/Actions/...`

Not every domain uses every section. AdvantageKit `Logger.recordOutput(...)` values also share the same root, but many of them are written directly to paths such as `/Spartobots2026/Simulation/...` and `/Spartobots2026/Targeting/...` instead of a `Telemetry` subtree.

## Active Domains

- `Drive`
  Common and mode-specific drive tuning.
- `Drive/Actions/AlignToAngle`
  Mode-specific auto-align PID and motion-limit tuning.
- `GamePiece/Intake`
  Intake tuning and telemetry.
- `GamePiece/Indexers`
  Top and bottom indexer tuning and telemetry.
- `GamePiece/Shooter`
  Shooter common tuning, mode-specific PID tuning, telemetry, and calibration controls.
- `Game/State`
  FMS-derived active/inactive hub status.
- `Operator/Actions`
  Dashboard action availability/scheduled/running state.
- `Operator/State/AutoChooser`
  Selected autonomous and chooser options.
- `Targeting/Hub/Tuning/Modes/<MODE>/MotionCompensation`
  Shared motion-compensation tuning used by drive and shooter logic.
- `Simulation/...`
  Logged sim-only outputs such as field state, projectiles, and robot-model component poses.

## High-Value Paths

### Auto Chooser

- `/Spartobots2026/Operator/State/AutoChooser/Selected`
- `/Spartobots2026/Operator/State/AutoChooser/Options`

### Operator Action State

- `/Spartobots2026/Operator/Actions/<ActionPath>/State/Available`
- `/Spartobots2026/Operator/Actions/<ActionPath>/State/Scheduled`
- `/Spartobots2026/Operator/Actions/<ActionPath>/State/Running`
- `/Spartobots2026/Operator/Actions/<ActionPath>/State/LastTriggeredTimestampSeconds`

### Drive Tuning

- `/Spartobots2026/Drive/Tuning/Common/LogHubAimVector`
- `/Spartobots2026/Drive/Tuning/Modes/<MODE>/Module/DrivePID/...`
- `/Spartobots2026/Drive/Tuning/Modes/<MODE>/Module/TurnPID/...`
- `/Spartobots2026/Drive/Tuning/Modes/<MODE>/PathPlanner/TranslationPID/...`
- `/Spartobots2026/Drive/Tuning/Modes/<MODE>/PathPlanner/RotationPID/...`
- `/Spartobots2026/Drive/Actions/AlignToAngle/Tuning/Modes/<MODE>/...`

### Path Following Logs

- `/Spartobots2026/Drive/Odometry/Trajectory`
- `/Spartobots2026/Drive/Odometry/TrajectorySetpoint`
- `/Spartobots2026/Drive/PathFollowing/Enabled`
- `/Spartobots2026/Drive/PathFollowing/RotationOverrideActive`
- `/Spartobots2026/Drive/PathFollowing/TargetPose`
- `/Spartobots2026/Drive/PathFollowing/TargetRotationDegrees`
- `/Spartobots2026/Drive/PathFollowing/TranslationErrorX`
- `/Spartobots2026/Drive/PathFollowing/TranslationErrorY`
- `/Spartobots2026/Drive/PathFollowing/RotationErrorRadians`
- `/Spartobots2026/Drive/PathFollowing/RotationErrorDegrees`
- `/Spartobots2026/Drive/PathFollowing/FeedforwardSpeeds`
- `/Spartobots2026/Drive/PathFollowing/FeedbackSpeeds`
- `/Spartobots2026/Drive/PathFollowing/OutputSpeeds`

### Shooter Calibration

- `/Spartobots2026/GamePiece/Shooter/Tuning/Common/Calibration/...`
- `/Spartobots2026/GamePiece/Shooter/Telemetry/Calibration/...`

### Hub Motion Compensation

- `/Spartobots2026/Targeting/Hub/Tuning/Modes/<MODE>/MotionCompensation/VelocityScale`
- `/Spartobots2026/Targeting/Hub/Tuning/Modes/<MODE>/MotionCompensation/LeadSeconds`

### Game State

- `/Spartobots2026/Game/State/RawGameData`
- `/Spartobots2026/Game/State/HubState`
- `/Spartobots2026/Game/State/HubActive`
- `/Spartobots2026/Game/State/GameDataValid`

## Subsystem Naming

- Intake: `/Spartobots2026/GamePiece/Intake/...`
- Indexers: `/Spartobots2026/GamePiece/Indexers/...`
- Shooter: `/Spartobots2026/GamePiece/Shooter/...`

## Tuning vs Logged Outputs

- Use `Tuning/Common` for values shared across all modes.
- Use `Tuning/Modes/<MODE>` for values that differ between `REAL`, `SIM`, and `REPLAY`.
- Expect some frequently inspected values to exist only as log outputs, especially under `/Spartobots2026/Simulation/...`, `/Spartobots2026/Targeting/...`, `/Spartobots2026/Operator/...`, and `/Spartobots2026/Game/...`.
