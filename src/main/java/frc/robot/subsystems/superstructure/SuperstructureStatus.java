package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.ShotSolution;
import frc.robot.subsystems.superstructure.Superstructure.PieceState;
import java.util.Optional;

public record SuperstructureStatus(
    Optional<SuperstructureGoal> requestedGoal,
    SuperstructureGoal activeGoal,
    PieceState pieceState,
    boolean hasPiece,
    boolean shooterReady,
    boolean driveAligned,
    Pose2d targetPose,
    Rotation2d targetHeading,
    ShotSolution shotSolution,
    boolean isAtGoal) {
  public boolean canShootNow(boolean poseTrusted) {
    return "READY".equals(shootReadinessReason(poseTrusted));
  }

  public String shootReadinessReason(boolean poseTrusted) {
    if (!(activeGoal instanceof SuperstructureGoal.HubShot)) {
      return "MODE_NOT_SCORING";
    }
    if (!hasPiece) {
      return "NO_GAME_PIECE";
    }
    if (!poseTrusted) {
      return "POSE_UNTRUSTED";
    }
    if (!shooterReady) {
      return "SHOOTER_NOT_READY";
    }
    if (!driveAligned) {
      return "DRIVE_NOT_ALIGNED";
    }
    if (!isAtGoal) {
      return "WAITING_FOR_FIRE_WINDOW";
    }
    return "READY";
  }
}
