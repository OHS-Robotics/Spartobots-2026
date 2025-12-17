# Autonomous Routines

Design notes for 2026 autos using PathPlanner + holonomic auto builder.

## Philosophy
- Prioritize repeatable completion over extra cycles; tight tolerances come after stability.
- Alliance-aware: enable path flipping and pose transforms; choose start poses per alliance in code.
- Every step gets a timeout and a safe fallback (skip or X-lock).
- Log everything: start pose, chosen path, timing, pose error envelope, and final pose.

## Paths & assets
- Paths live in `src/main/deploy/pathplanner/` (commit these).
- Name paths by outcome, not by date: `score_and_leave`, `leave_only`, `two_piece_wip`, etc.
- Event markers should map to commands via the auto builder’s event map; keep marker names short.
- Keep start pose baked into each path file; code should reset odometry to that pose at auto start.

## Routines (initial target list)
- `score_and_leave`: score preload, drive out of community, finish in X-lock or safe heading.
- `leave_only`: minimal baseline cross; use when mechanisms are offline.
- `path_debug`: short S-curve for tuning X/Y/Theta PID and heading hold.

## Implementation checklist
- Holonomic auto builder: provide kinematics, pose supplier/reset, `ChassisSpeeds` output, X/Y/Theta PID, and drive radius; enable alliance flipping.
- Default constraints: conservative max speed/accel first; bump only after on-field verification.
- Heading handling: hold heading during translations; avoid large heading swings while accelerating.
- Vision gating: disable vision corrections during the first second of auto; allow mid-auto fusion once moving steadily.
- Fallback: if pose reset/gyro is not ready, skip scoring and run `leave_only` with X-lock finish.

## Testing & verification
- Simulation: run each path, plot pose error, and confirm the auto ends within tolerance before enabling on-robot.
- On-robot dry runs: verify gyro zero, set start pose, run once without preload to confirm driving behavior.
- Acceptance gates: completion time within budget, path error under target, final pose error small enough for teleop handoff, and no timeouts triggered.
