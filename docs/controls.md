# Controls

Operator layer for the 2026 swerve robot. Default hardware: two Xbox-style controllers (one-driver mode first, add operator bindings when mechanisms arrive). Field-relative drive is assumed unless noted.

## Driver (Xbox)
- Left stick: translation X/Y (field-relative), apply circular deadband + slew limiting.
- Right stick X: angular velocity (CCW positive), slew-limited to avoid tip.
- Left bumper: slow/precision mode (cuts max speed/accel ~50%).
- Right bumper: enable heading hold when stick rotation is near zero (keeps last commanded heading).
- A: zero gyro to field frame (use when robot is physically square to field).
- B: X-lock (module toes in, for defense/parking).
- Y: face-field heading preset (e.g., 0°/180° toggle) for quick re-aim; keep configurable.
- X: toggle brake/coast for drive motors.
- Back/Share: toggle vision fusion (gate PhotonVision measurements).
- Start/Menu: cancel all commands/autos (safety).
- D-pad Up/Down: optional slow raise/lower speed scaling increments.

## Operator (reserve)
- Keep open for mechanism controls; avoid conflicts with driver’s safety buttons.
- If no dedicated operator this season, map key mechanism actions onto driver bumpers/face buttons only when it does not compromise driving safety.

## Dashboard hooks
- Field widget: live pose, planned path preview, and active auto name.
- Tunables: drive/steer PID + FF, max speed/accel, rotation PID, slew limits, heading hold gains, vision enable toggle.
- Buttons: pose reset, gyro zero, vision on/off, apply X-lock, cancel all.

## Simulation notes
- Same bindings as real robot; ensure keyboard/gamepad emulation works in sim.
- Include a "drive straight" and "rotate 180°" quick button for regression checks.

## Safety/ops checklist
- Gyro zero only when robot is square to the field border.
- Before a match: set start pose, pick auto on dashboard, verify brake/coast mode, confirm vision enable state, and ensure slow mode works.
