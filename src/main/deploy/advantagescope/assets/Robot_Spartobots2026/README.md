# Robot_Spartobots2026 Asset Notes

This asset is configured for component poses published at:

- `Spartobots2026/Simulation/RobotModel/Components` (`Pose3d[]`, robot-relative)

Component index order (must match `components` in `config.json`):

1. Intake pivot
2. Shooter hood

Expected model files in this folder:

- `model.glb` (full robot body)
- `model_0.glb` ... `model_1.glb` (component meshes in index order)

If a component appears offset or rotated in AdvantageScope, tune:

- `zeroedPosition` / `zeroedRotations` in `config.json`
- the corresponding geometry constants in `src/main/java/frc/robot/sim/RobotVisualizationPublisher.java`
