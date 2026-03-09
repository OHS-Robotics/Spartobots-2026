package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.TargetSelector.HubSelection;
import frc.robot.subsystems.superstructure.Superstructure.PieceState;

public sealed interface ShotSolution permits ShotSolution.HubShotSolution {
  Pose2d robotPose();

  ChassisSpeeds chassisVelocity();

  HubSelection selectedHub();

  PieceState pieceState();

  Pose2d targetPose();

  Rotation2d targetHeading();

  double launchSpeedMetersPerSec();

  Rotation2d launchAngle();

  double launchHeightMeters();

  double airtimeSeconds();

  static ShotSolution of(
      Pose2d robotPose,
      ChassisSpeeds chassisVelocity,
      HubSelection selectedHub,
      PieceState pieceState,
      Pose2d targetPose,
      Rotation2d targetHeading,
      double launchSpeedMetersPerSec,
      Rotation2d launchAngle,
      double launchHeightMeters,
      double airtimeSeconds) {
    return new HubShotSolution(
        robotPose,
        chassisVelocity,
        selectedHub,
        pieceState,
        targetPose,
        targetHeading,
        launchSpeedMetersPerSec,
        launchAngle,
        launchHeightMeters,
        airtimeSeconds);
  }

  record HubShotSolution(
      Pose2d robotPose,
      ChassisSpeeds chassisVelocity,
      HubSelection selectedHub,
      PieceState pieceState,
      Pose2d targetPose,
      Rotation2d targetHeading,
      double launchSpeedMetersPerSec,
      Rotation2d launchAngle,
      double launchHeightMeters,
      double airtimeSeconds)
      implements ShotSolution {}
}
