# Swerve Bring-up Checklist

Process to take a fresh swerve chassis (NEO/SparkMAX) to "driveable" and ready for auto tuning.

## Pre-flight
- Update all SparkMAX firmware; label CAN IDs and confirm uniqueness with REV Hardware Client.
- Record absolute encoder offsets with wheels straight; note steering inversion per module.
- Verify gyro orientation (right-hand rule) and mounting; write down the forward vector.
- Measure wheel radius/diameter, wheelbase, and trackwidth; keep them in `Constants`.
- Set current limits, brake mode for steer, and ramp rates before first spin.

## Static checks (robot off ground)
- With modules lifted, command a tiny drive output: forward stick should spin wheels "backward" if bevel gears face left (adjust drive inversion if not).
- Command a tiny rotation: modules should steer the same direction and settle; if a module spins endlessly, fix absolute offset before proceeding.
- Zero gyro, then command a field-relative forward; modules should point forward (if not, fix gyro axis/inversion).

## First-contact on carpet
- Start with slow/precision mode enabled; drive a short straight line, then a gentle strafe, then a slow 90° rotate.
- Engage X-lock; verify modules toe in correctly and the robot resists push.
- Check for brownout or CAN faults; log any frame drops or motor faults.

## Calibration steps
- Set and store steering offsets once modules are mechanically straight; avoid re-zeroing unless hardware is changed.
- Run SysId (drive) on blocks: quasistatic forward/reverse, dynamic forward/reverse. Apply `ks/kv/ka` to drive FF and tune drive PID.
- Tune steer PID with small step inputs; confirm no oscillation and minimal overshoot.
- Configure `SwerveDrivePoseEstimator` with reasonable state/vision stddevs; confirm pose stays stable with gyro-only.

## Regression / dashboard hooks
- Buttons: gyro zero, pose reset to start pose, X-lock, vision enable toggle, cancel all.
- Widgets: field view with live pose and planned path; numbers for chassis speeds, module angles, and pose estimator residuals.
- Logs to capture: pose, gyro, module states, drive/steer setpoints, currents/voltages, vision measurements, chosen auto name.

## Go/No-Go before auto
- Gyro zeroed to field frame; odometry aligns with physical field.
- No CAN faults or over-temp warnings after a 2-3 minute drive/strafe/rotate cycle.
- Drive straight 3 m and back; drift within expected tolerance and heading holds within a few degrees.
- Rotate 180° and confirm pose reflects it; enable vision and ensure the estimator snaps without jumps.
