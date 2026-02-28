package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.body.IntakeConstants;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.ArrayDeque;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class InternalGamePieceSimulation {
  public enum Mode {
    COLLECT,
    COLLECT_NO_INDEXER,
    FEED,
    MANUAL_FEED,
    REVERSE,
    UNJAM
  }

  public static final double simIntakeToHopperSeconds = 0.25;
  public static final double simHopperToShooterSeconds = 0.18;
  public static final double simReverseStepSeconds = 0.2;
  public static final double simIntakeCaptureMinPivotNormalized = 0.0;
  public static final int simIntakeCapacity = 1;
  public static final double simEjectSpeedMetersPerSec = 1.2;

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
    if (queuedCapturedPieces >= simIntakeCapacity) {
      lastEvent = "CaptureDropped";
      return;
    }
    queuedCapturedPieces++;
    lastEvent = "CaptureQueued";
  }

  public void update(
      double timestampSeconds,
      Mode mode,
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
            && pivotNormalized >= simIntakeCaptureMinPivotNormalized;
    boolean collectActive = mode == Mode.COLLECT || mode == Mode.COLLECT_NO_INDEXER;
    boolean feedActive =
        mode == Mode.FEED || (mode == Mode.MANUAL_FEED && manualFeedIndexerAllowed);
    boolean reverseActive = mode == Mode.REVERSE || mode == Mode.UNJAM;

    if (!intakeOccupied && queuedCapturedPieces > 0 && intakeForward) {
      intakeOccupied = true;
      queuedCapturedPieces--;
      lastEvent = "CapturedToIntake";
    }

    if (reverseActive) {
      intakeToHopperTimerSeconds = 0.0;
      hopperToShooterTimerSeconds = 0.0;
      reverseTimerSeconds += deltaTimeSeconds;
      while (reverseTimerSeconds >= simReverseStepSeconds) {
        reverseTimerSeconds -= simReverseStepSeconds;
        reverseOneStage(robotPose, robotFieldVelocityMetersPerSecond);
      }
      return;
    }

    reverseTimerSeconds = 0.0;

    if (collectActive && intakeOccupied && !hopperOccupied) {
      intakeToHopperTimerSeconds += deltaTimeSeconds;
      if (intakeToHopperTimerSeconds >= simIntakeToHopperSeconds) {
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
      if (hopperToShooterTimerSeconds >= simHopperToShooterSeconds) {
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
          robotFieldVelocityMetersPerSecond.plus(robotForwardUnit.times(simEjectSpeedMetersPerSec));
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
