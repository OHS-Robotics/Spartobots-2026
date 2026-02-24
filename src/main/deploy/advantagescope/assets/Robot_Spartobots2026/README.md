# Robot_Spartobots2026 Asset Notes

This asset is configured for component poses published at:

- `AdvantageScope/Robot/Components` (`Pose3d[]`, robot-relative)

Component index order (must match `components` in `config.json`):

1. Front-left swerve module steer
2. Front-right swerve module steer
3. Back-left swerve module steer
4. Back-right swerve module steer
5. Intake pivot
6. Hopper extension
7. Shooter hood

Expected model files in this folder:

- `model.glb` (full robot body)
- `model_0.glb` ... `model_6.glb` (component meshes in index order)

If a component appears offset or rotated in AdvantageScope, tune:

- `zeroedPosition` / `zeroedRotations` in `config.json`
- the corresponding geometry constants in `RobotContainer`
