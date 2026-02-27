package frc.robot.subsystems.body;

import frc.robot.sim.InternalGamePieceSimulation;

public class GamePieceSensorIOSim implements GamePieceSensorIO {
  private final InternalGamePieceSimulation simulation;

  public GamePieceSensorIOSim(InternalGamePieceSimulation simulation) {
    this.simulation = simulation;
  }

  @Override
  public void updateInputs(GamePieceSensorIOInputs inputs) {
    inputs.intakeConfigured = true;
    inputs.hopperConfigured = true;
    inputs.shooterConfigured = true;
    inputs.intakeDetected = simulation.hasIntakePiece();
    inputs.hopperDetected = simulation.hasHopperPiece();
    inputs.shooterDetected = simulation.hasShooterPiece();
  }
}
