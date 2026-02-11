package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
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

  public double estimateHubShotAirtimeSeconds(Pose2d robotPose, Pose2d hubPose) {
    double horizontalDistanceMeters =
        robotPose.getTranslation().getDistance(hubPose.getTranslation());
    double targetHeightDeltaMeters = ShooterConstants.hubCenterHeightMeters - launchHeightMeters;
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

    // Solve targetHeightDelta = vy * t - 0.5 * g * t^2 and pick the root that best matches
    // the current horizontal distance.
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
}
