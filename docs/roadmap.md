# Roadmap: Swerve to Scoring Auto

Tailored to: swerve drivetrain (NEO/SparkMAX), RoboRIO2, PhotonVision (AprilTags-only), PathPlanner, scoring auto target.

- **0) Repo Hygiene & Guardrails**
  - Add `docs/controls.md`, `docs/autos.md`, `docs/bringup.md`; ensure `.gitignore` is sane.
  - Expand `Constants` into sections: `CAN` (drive/steer IDs, Pigeon/ADXRS), `Swerve` (wheelbase, trackwidth, wheel radius, drive/steer gear ratios, max speed/accel), `Vision` (camera pose, stddevs), `Tuning` (PID/FF, pose estimator stddevs), `OI`, `Auto`.
  - Enable logging: WPILib `DataLogManager` (or AdvantageKit). Log pose, module states, gyro, drive/steer setpoints, voltages, currents, vision measurements, active auto name.
  - Add `RobotState` snapshot (pose, velocities, alliance, vision health, CAN health).
  - Simulation harness: swerve sim (WPILib kinematics + `SwerveDrivePoseEstimator`), a sim dashboard page with field view.

- **1) Swerve Drivetrain Bring-up (NEO/SparkMAX)**
  - Hardware constants: fill CAN IDs, drive/steer inversions, encoder offsets, gyro orientation, wheel radius, gear ratios, max chassis speed.
  - Subsystem: `Swerve` with module class wrapping two SparkMAXes + absolute encoder; set current limits, idle mode brake, ramp rates, and sign conventions; expose `drive(ChassisSpeeds)` and `setX()` lock.
  - Kinematics: `SwerveDriveKinematics` built from module translations; create `SwerveDrivePoseEstimator`.
  - Default command: `DefaultDriveCommand` using field-relative mode, deadbands, slew rate limiters, and a precision/slow toggle.
  - Characterization: run SysId (quasistatic/dynamic) for drive motors; set `ks/kv/ka` and drive PID. Tune steer PID separately.
  - Health checks: periodic CAN fault logging, overtemp/brownout warnings; a “drive straight” and “rotate 180°” routine for quick verification.

- **2) Operator Layer & Dashboard**
  - Controls: map left stick → translation, right stick → rotation; bumpers for slow mode; button for gyro zero; button for X-lock; button to toggle brake/coast.
  - Shuffleboard/SmartDashboard: tunable drive/steer PID, FF, max speed/accel, slew rates; pose reset button; vision on/off gating; field widget with live pose and planned path.
  - Safety: ensure auto cancels on teleop start; add “cancel all” button for ops.

- **3) Localization & Vision (PhotonVision, AprilTags-only)**
  - Camera config: set camera name, resolution, exposure; LED policy off unless needed; load AprilTag layout JSON (season field layout).
  - Mount geometry: camera-to-robot transform (x, y, z in meters; roll/pitch/yaw in radians). Add per-camera measurement stddevs.
  - Fusion: feed `EstimatedRobotPose` into `SwerveDrivePoseEstimator` with latency compensation; gate by ambiguity, tag count, and distance; reject outliers.
  - Alliance awareness: use `DriverStation.getAlliance()` to flip poses/paths where needed.

- **4) PathPlanner Integration**
  - Add/verify vendordep; set paths in `src/main/deploy/pathplanner/`.
  - Configure HolonomicAutoBuilder (or equivalent): supply kinematics, pose supplier/reset, `ChassisSpeeds` output, X/Y/Theta PID, drive base radius, event map for markers, and alliance flipping enabled.
  - Tuning: start conservative max speed/accel; tune X/Y PID, then Theta PID; verify heading holds while translating.

- **5) Autonomous Building Blocks**
  - Commands: `DriveDistance`, `RotateToHeading`, `FollowPath(pathName)`, `HoldHeadingWhileTranslate`, `SetXLock`, `ResetPoseTo(path start)`, `ApplyVisionNow` toggle.
  - Event markers: hook intake/shooter/arm commands in the auto builder map.
  - Timeouts: every step gets a timeout; add fallback (skip if not ready).
  - Start pose: derive from PathPlanner path; ensure gyro is zeroed to field frame at auto start.

- **6) Scoring Auto (initial target)**
  - Goal: Score preloaded game piece + leave/start second action if time allows.
  - Flow example:
    1) `ResetPoseTo(path0.start)` using known starting pose.  
    2) If mechanism ready: score command (e.g., `ShootPreload` or `PlacePreload`) with timeout.  
    3) `FollowPath("score_and_leave")` with heading control; include event markers for mechanism sequencing if needed.  
    4) End with `SetXLock` or face-field-safe heading.
  - Verification: dry-run in sim; on-robot, check completion time, path error envelope, final pose error < tolerance; log review after each attempt.

- **7) Testing & Regression**
  - Unit tests: kinematics helpers, odometry math, command factories (timeouts, requirements).
  - Sim tests: drive command responsiveness; path follower hits waypoints; vision injection nudges pose correctly.
  - On-robot checklist: gyro zero check, drive straight 3 m (drift spec), 90°/180° turns, path tracking error under target, alliance flip verification, auto chooser selects correct routine.
  - Logging workflow: keep log review ritual; bookmark good/bad runs.

- **8) Operations & Maintenance**
  - Tuning order: gyro bias → steer PID → drive PID/FF (from SysId) → Theta PID → X/Y PID → pose estimator stddevs → auto PID caps.
  - Pre-match: set start pose, select auto, confirm brake/coast, vision lock, battery/CAN health.
  - Calibration: periodic steer encoder offset validation; check wheel radius and track/wheelbase if drift appears.

- **9) Stretch/Quality-of-Life**
  - Resilient auto state machine (branch/skip on failure).
  - Dynamic micro-adjust paths (last-second re-aim).
  - Better dashboards: path preview, live event marker status, quick toggles for logging level.

**Details to confirm/fill in**
- Swerve module type and gear ratios; wheel diameter/radius; wheelbase/trackwidth.
- Gyro model and mounting orientation.
- Camera pose (x, y, z, roll/pitch/yaw) and desired stddevs.
- Scoring mechanism specifics for the first auto (what action/height, expected cycle time).
