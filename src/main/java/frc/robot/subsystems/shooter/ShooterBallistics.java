package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.TargetSelector.HubSelection;
import frc.robot.subsystems.superstructure.Superstructure.PieceState;

public class ShooterBallistics {
  private double launchSpeedMetersPerSec = ShooterConstants.defaultLaunchSpeedMetersPerSec;
  private Rotation2d launchAngle = ShooterConstants.defaultLaunchAngle;
  private double launchHeightMeters = ShooterConstants.defaultLaunchHeightMeters;

  public void setLaunchState(double launchSpeedMetersPerSec, Rotation2d launchAngle) {
    this.launchSpeedMetersPerSec =
        Math.max(launchSpeedMetersPerSec, ShooterConstants.minLaunchSpeedMetersPerSec);
    this.launchAngle = launchAngle;
  }

  public void setLaunchHeightMeters(double launchHeightMeters) {
    this.launchHeightMeters = launchHeightMeters;
  }

  public ShotSolution solveHubShot(
      Pose2d robotPose,
      ChassisSpeeds chassisVelocity,
      HubSelection selectedHub,
      PieceState pieceState,
      Pose3d hubPose) {
    double airtimeSeconds = estimateHubShotAirtimeSeconds(robotPose, hubPose);
    Translation2d compensationOffset =
        getFieldRelativeVelocity(robotPose, chassisVelocity).times(-Math.max(0.0, airtimeSeconds));
    Pose2d targetPose =
        new Pose2d(
            hubPose.toPose2d().getTranslation().plus(compensationOffset),
            hubPose.toPose2d().getRotation());
    return ShotSolution.of(
        robotPose,
        new ChassisSpeeds(
            chassisVelocity.vxMetersPerSecond,
            chassisVelocity.vyMetersPerSecond,
            chassisVelocity.omegaRadiansPerSecond),
        selectedHub,
        pieceState,
        targetPose,
        computeHeadingToTarget(robotPose, targetPose.getTranslation()),
        launchSpeedMetersPerSec,
        launchAngle,
        launchHeightMeters,
        airtimeSeconds);
  }

  public double estimateHubShotAirtimeSeconds(Pose2d robotPose, Pose3d hubPose) {
    double horizontalDistanceMeters =
        robotPose.getTranslation().getDistance(hubPose.toPose2d().getTranslation());
    double targetHeightDeltaMeters = hubPose.getZ() - launchHeightMeters;
    return estimateShotAirtimeSeconds(horizontalDistanceMeters, targetHeightDeltaMeters);
  }

  private double estimateShotAirtimeSeconds(
      double horizontalDistanceMeters, double targetHeightDeltaMeters) {
    double shotSpeed =
        Math.max(launchSpeedMetersPerSec, ShooterConstants.minLaunchSpeedMetersPerSec);
    double vx = shotSpeed * Math.cos(launchAngle.getRadians());
    double vy = shotSpeed * Math.sin(launchAngle.getRadians());

    if (Math.abs(vx) < ShooterConstants.minHorizontalVelocityMetersPerSec) {
      return ShooterConstants.fallbackAirtimeSeconds;
    }

    double fallbackTime = horizontalDistanceMeters / Math.abs(vx);

    double a = 0.5 * ShooterConstants.gravityMetersPerSecSquared;
    double b = -vy;
    double c = targetHeightDeltaMeters;
    double discriminant = (b * b) - (4.0 * a * c);

    if (discriminant < 0.0) {
      return clampAirtime(fallbackTime);
    }

    double sqrtDiscriminant = Math.sqrt(discriminant);
    double t1 = (-b - sqrtDiscriminant) / (2.0 * a);
    double t2 = (-b + sqrtDiscriminant) / (2.0 * a);
    double bestTime = selectBestPositiveRoot(t1, t2, horizontalDistanceMeters, vx);

    if (bestTime <= 0.0) {
      bestTime = fallbackTime;
    }

    return clampAirtime(bestTime);
  }

  private static double selectBestPositiveRoot(
      double t1, double t2, double horizontalDistanceMeters, double vx) {
    double bestTime = -1.0;
    double bestDistanceError = Double.POSITIVE_INFINITY;
    double speedX = Math.abs(vx);

    if (t1 > 0.0) {
      double distanceError = Math.abs(speedX * t1 - horizontalDistanceMeters);
      bestDistanceError = distanceError;
      bestTime = t1;
    }

    if (t2 > 0.0) {
      double distanceError = Math.abs(speedX * t2 - horizontalDistanceMeters);
      if (distanceError < bestDistanceError) {
        bestTime = t2;
      }
    }

    return bestTime;
  }

  private static double clampAirtime(double airtimeSeconds) {
    return MathUtil.clamp(
        airtimeSeconds, ShooterConstants.minAirtimeSeconds, ShooterConstants.maxAirtimeSeconds);
  }

  private static Translation2d getFieldRelativeVelocity(
      Pose2d robotPose, ChassisSpeeds chassisVelocity) {
    ChassisSpeeds fieldRelativeVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(chassisVelocity, robotPose.getRotation());
    return new Translation2d(
        fieldRelativeVelocity.vxMetersPerSecond, fieldRelativeVelocity.vyMetersPerSecond);
  }

  private static Rotation2d computeHeadingToTarget(
      Pose2d robotPose, Translation2d targetTranslation) {
    Translation2d toTarget = targetTranslation.minus(robotPose.getTranslation());
    return new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
  }
}
