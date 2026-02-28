package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.body.GamePieceManager;
import frc.robot.subsystems.body.GamePieceManagerConstants;
import frc.robot.subsystems.body.IntakeConstants;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.ArrayDeque;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class InternalGamePieceSimulation {
  public record PendingEjection(Translation2d position, Translation2d velocityMetersPerSecond) {}

  private boolean intakeOccupied = false;
  private boolean hopperOccupied = false;
  private boolean shooterOccupied = false;
  private int queuedCapturedPieces = 0;
  private double lastUpdateTimestampSeconds = Double.NaN;
  private double intakeToHopperTimerSeconds = 0.0;
  private double hopperToShooterTimerSeconds = 0.0;
  private double reverseTimerSeconds = 0.0;
  private String lastEvent = "Reset";
  private Pose3d lastEjectPose = new Pose3d();
  private final Queue<PendingEjection> pendingEjections = new ArrayDeque<>();

  public void reset() {
    intakeOccupied = false;
    hopperOccupied = false;
    shooterOccupied = false;
    queuedCapturedPieces = 0;
    lastUpdateTimestampSeconds = Double.NaN;
    intakeToHopperTimerSeconds = 0.0;
    hopperToShooterTimerSeconds = 0.0;
    reverseTimerSeconds = 0.0;
    lastEvent = "Reset";
    lastEjectPose = new Pose3d();
    pendingEjections.clear();
  }

  public void queueCapturedPiece() {
    if (queuedCapturedPieces >= GamePieceManagerConstants.simIntakeCapacity) {
      lastEvent = "CaptureDropped";
      return;
    }
    queuedCapturedPieces++;
    lastEvent = "CaptureQueued";
  }

  public void update(
      double timestampSeconds,
      GamePieceManager.Mode mode,
      double intakeAppliedOutput,
      double pivotNormalized,
      boolean manualFeedIndexerAllowed,
      Pose2d robotPose,
      Translation2d robotFieldVelocityMetersPerSecond) {
    double deltaTimeSeconds =
        Double.isFinite(lastUpdateTimestampSeconds)
            ? Math.max(0.0, timestampSeconds - lastUpdateTimestampSeconds)
            : 0.0;
    lastUpdateTimestampSeconds = timestampSeconds;

    boolean intakeForward =
        intakeAppliedOutput > IntakeConstants.simForwardOutputThreshold
            && pivotNormalized >= GamePieceManagerConstants.simIntakeCaptureMinPivotNormalized;
    boolean collectActive =
        mode == GamePieceManager.Mode.COLLECT || mode == GamePieceManager.Mode.COLLECT_NO_INDEXER;
    boolean feedActive =
        mode == GamePieceManager.Mode.FEED
            || (mode == GamePieceManager.Mode.MANUAL_FEED && manualFeedIndexerAllowed);
    boolean reverseActive =
        mode == GamePieceManager.Mode.REVERSE || mode == GamePieceManager.Mode.UNJAM;

    if (!intakeOccupied && queuedCapturedPieces > 0 && intakeForward) {
      intakeOccupied = true;
      queuedCapturedPieces--;
      lastEvent = "CapturedToIntake";
    }

    if (reverseActive) {
      intakeToHopperTimerSeconds = 0.0;
      hopperToShooterTimerSeconds = 0.0;
      reverseTimerSeconds += deltaTimeSeconds;
      while (reverseTimerSeconds >= GamePieceManagerConstants.simReverseStepSeconds) {
        reverseTimerSeconds -= GamePieceManagerConstants.simReverseStepSeconds;
        reverseOneStage(robotPose, robotFieldVelocityMetersPerSecond);
      }
      return;
    }

    reverseTimerSeconds = 0.0;

    if (collectActive && intakeOccupied && !hopperOccupied) {
      intakeToHopperTimerSeconds += deltaTimeSeconds;
      if (intakeToHopperTimerSeconds >= GamePieceManagerConstants.simIntakeToHopperSeconds) {
        intakeOccupied = false;
        hopperOccupied = true;
        intakeToHopperTimerSeconds = 0.0;
        lastEvent = "TransferredIntakeToHopper";
      }
    } else {
      intakeToHopperTimerSeconds = 0.0;
    }

    if (feedActive && hopperOccupied && !shooterOccupied) {
      hopperToShooterTimerSeconds += deltaTimeSeconds;
      if (hopperToShooterTimerSeconds >= GamePieceManagerConstants.simHopperToShooterSeconds) {
        hopperOccupied = false;
        shooterOccupied = true;
        hopperToShooterTimerSeconds = 0.0;
        lastEvent = "TransferredHopperToShooter";
      }
    } else {
      hopperToShooterTimerSeconds = 0.0;
    }
  }

  private void reverseOneStage(Pose2d robotPose, Translation2d robotFieldVelocityMetersPerSecond) {
    boolean ejectIntake = intakeOccupied;
    boolean newIntakeOccupied = hopperOccupied;
    boolean newHopperOccupied = shooterOccupied;
    intakeOccupied = newIntakeOccupied;
    hopperOccupied = newHopperOccupied;
    shooterOccupied = false;

    if (ejectIntake) {
      Translation2d robotForwardUnit =
          new Translation2d(robotPose.getRotation().getCos(), robotPose.getRotation().getSin());
      Translation2d ejectOffset =
          robotPose
              .getTranslation()
              .plus(
                  robotForwardUnit.times(
                      (DriveConstants.bumperLengthXMeters * 0.5)
                          + (IntakeConstants.simMapleIntakeExtensionMeters * 0.5)));
      Translation2d ejectVelocity =
          robotFieldVelocityMetersPerSecond.plus(
              robotForwardUnit.times(GamePieceManagerConstants.simEjectSpeedMetersPerSec));
      pendingEjections.add(new PendingEjection(ejectOffset, ejectVelocity));
      lastEjectPose =
          new Pose3d(
              ejectOffset.getX(),
              ejectOffset.getY(),
              0.05,
              new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians()));
      lastEvent = "EjectedFromIntake";
    } else if (newIntakeOccupied || newHopperOccupied) {
      lastEvent = "ReversedStages";
    }
  }

  public boolean hasIntakePiece() {
    return intakeOccupied;
  }

  public boolean hasHopperPiece() {
    return hopperOccupied;
  }

  public boolean hasShooterPiece() {
    return shooterOccupied;
  }

  public boolean consumeShooterPiece() {
    if (!shooterOccupied) {
      return false;
    }
    shooterOccupied = false;
    lastEvent = "ConsumedShooterPiece";
    return true;
  }

  public int getQueuedCount() {
    return queuedCapturedPieces;
  }

  public PendingEjection pollPendingEjection() {
    return pendingEjections.poll();
  }

  public void logOutputs() {
    Logger.recordOutput("GamePieceSimulation/Stage/IntakeOccupied", intakeOccupied);
    Logger.recordOutput("GamePieceSimulation/Stage/HopperOccupied", hopperOccupied);
    Logger.recordOutput("GamePieceSimulation/Stage/ShooterOccupied", shooterOccupied);
    Logger.recordOutput("GamePieceSimulation/Transfers/LastEvent", lastEvent);
    Logger.recordOutput("GamePieceSimulation/Transfers/QueuedCount", queuedCapturedPieces);
    Logger.recordOutput("GamePieceSimulation/EjectPose", lastEjectPose);
  }
}
