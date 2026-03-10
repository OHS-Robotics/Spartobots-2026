# Simulator Readiness Guide

This checklist is the handoff gate before testing on the real robot.

## Prerequisite

- Use Java 17 for every Gradle command in this repo.
- `./gradlew test` and `./gradlew simulateJava` are part of the readiness gate and should be run with the same Java 17 install.

## Fidelity Matrix

Simulator results do not mean the same thing for every subsystem.

| Subsystem | Current Fidelity | What simulator results mean |
| --- | --- | --- |
| Drive | `SIM_ONLY` | Good confidence in command flow, pose reset, odometry, and heading behavior. Real carpet and hardware tuning still need validation. |
| Vision | `SIM_ONLY` | Good confidence in pose-observation plumbing and field-tag visibility. Camera placement, exposure, and real coprocessor behavior still need validation. |
| Intake | `PLACEHOLDER` | Only validates goal/state contracts and superstructure flow. Does not prove the real mechanism matches the sim. |
| Indexer | `PLACEHOLDER` | Only validates goal/state contracts and possession flow. Does not prove the real mechanism matches the sim. |
| Shooter | `PLACEHOLDER` | Only validates shot-state orchestration and readiness gates. Does not prove the real mechanism matches the sim. |
| Endgame | `PLACEHOLDER` | Only validates safe-goal sequencing and park flow. Does not prove the real mechanism matches the sim. |

Any simulator pass that depends on a `PLACEHOLDER` subsystem must be reported as `logic validated, hardware fidelity unproven`.

## Automated Readiness Gate

- `./gradlew test`
- Subsystem contract coverage for `SimpleIntake`, `SimpleIndexer`, `SimpleShooter`, and `SimpleEndgame`
- Superstructure readiness and phase-transition coverage
- Driver-intent command coverage for cancel/recover and manual override behavior
- Match simulation coverage for preload reset, floor/depot acquisition, ejection, outpost refill gating, and shot spawning
- Drive simulation coverage for pose reset and forward odometry movement
- Vision coverage for pose filtering and PhotonVision sim observation smoke
- Auto routine coverage for every shipped chooser option
- Capability mismatch coverage so placeholder mechanisms disable unsafe real-robot features

## Manual Simulator Rehearsal

Run this full checklist before first robot bring-up and after any change to drive, vision, auto, targeting, `RobotContainer`, or simulation geometry/settings.

1. Start `simulateJava`, Driver Station sim, and AdvantageScope with Java 17.
2. Confirm disabled reset places the robot at the selected auto start pose and preloads one piece.
3. Drive manually and confirm `FieldSimulation/RobotPose`, `FieldSimulation/RobotPose3d`, and module poses look correct.
4. Test left-trigger acquisition from both floor and depot and confirm held-fuel logs update.
5. Test right-trigger aim/fire on both alliances and across active-hub changes.
6. Test left-bumper outpost refill only when empty and aligned.
7. Test `Y` quick park, `B` cancel/recover, and `X` manual override in the middle of assisted actions.
8. Run every autonomous family once and record final pose, piece state, score change, and whether the sequence completed without deadlock or scheduler errors.

## Exit Criteria

- Automated tests are green under Java 17.
- Manual simulator checklist passes on blue and red for every alliance-dependent behavior.
- No blocker remains in drive control, pose estimation, autonomous navigation, reset behavior, command scheduling, or cancel/safety flow.
- Placeholder mechanisms remain explicitly marked as unproven for hardware fidelity.
