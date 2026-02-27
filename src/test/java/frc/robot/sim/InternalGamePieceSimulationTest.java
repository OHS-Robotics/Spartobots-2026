package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.body.GamePieceManager;
import frc.robot.subsystems.body.GamePieceManagerConstants;
import org.junit.jupiter.api.Test;

class InternalGamePieceSimulationTest {
  private static final Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.kZero);
  private static final Translation2d robotVelocity = new Translation2d();

  @Test
  void progressesCapturedPieceThroughStagesAndConsumesShooterPiece() {
    InternalGamePieceSimulation simulation = new InternalGamePieceSimulation();

    simulation.queueCapturedPiece();
    simulation.update(0.0, GamePieceManager.Mode.COLLECT, 0.5, 1.0, false, robotPose, robotVelocity);
    assertTrue(simulation.hasIntakePiece());

    simulation.update(
        GamePieceManagerConstants.simIntakeToHopperSeconds + 0.05,
        GamePieceManager.Mode.COLLECT,
        0.5,
        1.0,
        false,
        robotPose,
        robotVelocity);
    assertFalse(simulation.hasIntakePiece());
    assertTrue(simulation.hasHopperPiece());

    simulation.update(
        GamePieceManagerConstants.simIntakeToHopperSeconds
            + GamePieceManagerConstants.simHopperToShooterSeconds
            + 0.10,
        GamePieceManager.Mode.FEED,
        0.0,
        1.0,
        false,
        robotPose,
        robotVelocity);
    assertFalse(simulation.hasHopperPiece());
    assertTrue(simulation.hasShooterPiece());
    assertTrue(simulation.consumeShooterPiece());
    assertFalse(simulation.hasShooterPiece());
  }

  @Test
  void reverseEjectsIntakePieceBackToField() {
    InternalGamePieceSimulation simulation = new InternalGamePieceSimulation();
    simulation.queueCapturedPiece();
    simulation.update(0.0, GamePieceManager.Mode.COLLECT, 0.5, 1.0, false, robotPose, robotVelocity);
    assertTrue(simulation.hasIntakePiece());

    simulation.update(
        GamePieceManagerConstants.simReverseStepSeconds + 0.05,
        GamePieceManager.Mode.REVERSE,
        -0.5,
        1.0,
        false,
        robotPose,
        robotVelocity);

    assertFalse(simulation.hasIntakePiece());
    assertNotNull(simulation.pollPendingEjection());
  }
}
