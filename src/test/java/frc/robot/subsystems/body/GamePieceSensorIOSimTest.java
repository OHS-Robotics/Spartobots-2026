package frc.robot.subsystems.body;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.sim.InternalGamePieceSimulation;
import org.junit.jupiter.api.Test;

class GamePieceSensorIOSimTest {
  @Test
  void mirrorsInternalSimulationStageOccupancy() {
    InternalGamePieceSimulation simulation = new InternalGamePieceSimulation();
    GamePieceSensorIOSim io = new GamePieceSensorIOSim(simulation);
    GamePieceSensorIO.GamePieceSensorIOInputs inputs =
        new GamePieceSensorIO.GamePieceSensorIOInputs();

    io.updateInputs(inputs);
    assertFalse(inputs.intakeDetected);
    assertFalse(inputs.hopperDetected);
    assertFalse(inputs.shooterDetected);

    simulation.queueCapturedPiece();
    simulation.update(
        0.0,
        GamePieceManager.Mode.COLLECT,
        0.5,
        1.0,
        false,
        new Pose2d(1.0, 1.0, Rotation2d.kZero),
        new Translation2d());

    io.updateInputs(inputs);
    assertTrue(inputs.intakeDetected);
    assertFalse(inputs.hopperDetected);
    assertFalse(inputs.shooterDetected);
  }
}
