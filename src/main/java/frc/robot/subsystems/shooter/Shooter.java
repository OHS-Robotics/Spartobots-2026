package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public static record HubShotSolution(
      double distanceMeters,
      Rotation2d launchAngle,
      double launchSpeedMetersPerSec,
      double shooterPower,
      double airtimeSeconds,
      boolean feasible) {}

  private double launchHeightMeters = ShooterConstants.defaultLaunchHeightMeters;
  private HubShotSolution latestHubShotSolution =
      new HubShotSolution(
          0.0,
          ShooterConstants.defaultLaunchAngle,
          ShooterConstants.defaultLaunchSpeedMetersPerSec,
          speedToPower(ShooterConstants.defaultLaunchSpeedMetersPerSec),
          ShooterConstants.fallbackAirtimeSeconds,
          false);

  public Rotation2d getHubLaunchAngleSetpoint() {
    return latestHubShotSolution.launchAngle();
  }

  public double getHubLaunchSpeedSetpointMetersPerSec() {
    return latestHubShotSolution.launchSpeedMetersPerSec();
  }

  public double getHubPowerSetpoint() {
    return latestHubShotSolution.shooterPower();
  }

  public double getHubAirtimeSeconds() {
    return latestHubShotSolution.airtimeSeconds();
  }

  public boolean isHubShotSolutionFeasible() {
    return latestHubShotSolution.feasible();
  }

  public HubShotSolution getLatestHubShotSolution() {
    return latestHubShotSolution;
  }

  public void setLaunchHeightMeters(double launchHeightMeters) {
    this.launchHeightMeters = launchHeightMeters;
  }

  public HubShotSolution updateHubShotSolution(Pose2d robotPose, Pose2d hubPose) {
    double horizontalDistanceMeters =
        robotPose.getTranslation().getDistance(hubPose.getTranslation());
    double targetHeightDeltaMeters = ShooterConstants.hubCenterHeightMeters - launchHeightMeters;
    latestHubShotSolution = solveHubShot(horizontalDistanceMeters, targetHeightDeltaMeters);
    logHubShotSolution(latestHubShotSolution);
    return latestHubShotSolution;
  }

  public double estimateHubShotAirtimeSeconds(Pose2d robotPose, Pose2d hubPose) {
    return updateHubShotSolution(robotPose, hubPose).airtimeSeconds();
  }

  private HubShotSolution solveHubShot(double horizontalDistanceMeters, double targetHeightDeltaMeters) {
    Rotation2d preferredAngle = getPreferredAngle(horizontalDistanceMeters);

    HubShotSolution bestSolution = null;
    double bestScore = Double.POSITIVE_INFINITY;
    for (double degrees = ShooterConstants.minLaunchAngle.getDegrees();
        degrees <= ShooterConstants.maxLaunchAngle.getDegrees();
        degrees += ShooterConstants.launchAngleSearchStepDegrees) {
      Rotation2d candidateAngle = Rotation2d.fromDegrees(degrees);
      OptionalDouble launchSpeedResult =
          calculateLaunchSpeedForTarget(
              horizontalDistanceMeters, targetHeightDeltaMeters, candidateAngle);
      if (launchSpeedResult.isEmpty()) {
        continue;
      }

      double launchSpeedMetersPerSec = launchSpeedResult.getAsDouble();
      if (launchSpeedMetersPerSec < ShooterConstants.minLaunchSpeedMetersPerSec
          || launchSpeedMetersPerSec > ShooterConstants.maxLaunchSpeedMetersPerSec) {
        continue;
      }

      double score = Math.abs(candidateAngle.minus(preferredAngle).getDegrees());
      if (score < bestScore) {
        bestScore = score;
        bestSolution =
            createShotSolution(
                horizontalDistanceMeters, targetHeightDeltaMeters, candidateAngle, launchSpeedMetersPerSec, true);
      }
    }

    if (bestSolution != null) {
      return bestSolution;
    }

    OptionalDouble preferredLaunchSpeed =
        calculateLaunchSpeedForTarget(horizontalDistanceMeters, targetHeightDeltaMeters, preferredAngle);
    double fallbackLaunchSpeedMetersPerSec =
        preferredLaunchSpeed.orElse(ShooterConstants.defaultLaunchSpeedMetersPerSec);
    fallbackLaunchSpeedMetersPerSec =
        MathUtil.clamp(
            fallbackLaunchSpeedMetersPerSec,
            ShooterConstants.minLaunchSpeedMetersPerSec,
            ShooterConstants.maxLaunchSpeedMetersPerSec);

    return createShotSolution(
        horizontalDistanceMeters,
        targetHeightDeltaMeters,
        preferredAngle,
        fallbackLaunchSpeedMetersPerSec,
        false);
  }

  private HubShotSolution createShotSolution(
      double horizontalDistanceMeters,
      double targetHeightDeltaMeters,
      Rotation2d launchAngle,
      double launchSpeedMetersPerSec,
      boolean feasible) {
    double airtimeSeconds =
        calculateAirtimeSeconds(
            horizontalDistanceMeters, targetHeightDeltaMeters, launchSpeedMetersPerSec, launchAngle);
    return new HubShotSolution(
        horizontalDistanceMeters,
        launchAngle,
        launchSpeedMetersPerSec,
        speedToPower(launchSpeedMetersPerSec),
        airtimeSeconds,
        feasible);
  }

  private Rotation2d getPreferredAngle(double horizontalDistanceMeters) {
    double distanceProgress =
        MathUtil.clamp(
            (horizontalDistanceMeters - ShooterConstants.hubShotNearDistanceMeters)
                / (ShooterConstants.hubShotFarDistanceMeters - ShooterConstants.hubShotNearDistanceMeters),
            0.0,
            1.0);
    double preferredAngleDegrees =
        MathUtil.interpolate(
            ShooterConstants.hubShotNearPreferredAngle.getDegrees(),
            ShooterConstants.hubShotFarPreferredAngle.getDegrees(),
            distanceProgress);
    preferredAngleDegrees =
        MathUtil.clamp(
            preferredAngleDegrees,
            ShooterConstants.minLaunchAngle.getDegrees(),
            ShooterConstants.maxLaunchAngle.getDegrees());
    return Rotation2d.fromDegrees(preferredAngleDegrees);
  }

  private OptionalDouble calculateLaunchSpeedForTarget(
      double horizontalDistanceMeters, double targetHeightDeltaMeters, Rotation2d launchAngle) {
    double cosTheta = Math.cos(launchAngle.getRadians());
    if (Math.abs(cosTheta) < 1e-6) {
      return OptionalDouble.empty();
    }

    double tanTheta = Math.tan(launchAngle.getRadians());
    double denominator =
        2.0 * cosTheta * cosTheta * (horizontalDistanceMeters * tanTheta - targetHeightDeltaMeters);
    if (denominator <= 0.0) {
      return OptionalDouble.empty();
    }

    double launchSpeedSquared =
        ShooterConstants.gravityMetersPerSecSquared
            * horizontalDistanceMeters
            * horizontalDistanceMeters
            / denominator;
    if (!Double.isFinite(launchSpeedSquared) || launchSpeedSquared <= 0.0) {
      return OptionalDouble.empty();
    }

    return OptionalDouble.of(Math.sqrt(launchSpeedSquared));
  }

  private double calculateAirtimeSeconds(
      double horizontalDistanceMeters,
      double targetHeightDeltaMeters,
      double launchSpeedMetersPerSec,
      Rotation2d launchAngle) {
    double vx = launchSpeedMetersPerSec * Math.cos(launchAngle.getRadians());
    if (Math.abs(vx) < ShooterConstants.minHorizontalVelocityMetersPerSec) {
      return ShooterConstants.fallbackAirtimeSeconds;
    }

    double vy = launchSpeedMetersPerSec * Math.sin(launchAngle.getRadians());
    double airtimeByHorizontal = horizontalDistanceMeters / Math.abs(vx);
    double airtimeByVertical =
        solveVerticalAirtime(vy, targetHeightDeltaMeters).orElse(airtimeByHorizontal);

    return clampAirtime(0.5 * (airtimeByHorizontal + airtimeByVertical));
  }

  private OptionalDouble solveVerticalAirtime(double vy, double targetHeightDeltaMeters) {
    double a = 0.5 * ShooterConstants.gravityMetersPerSecSquared;
    double b = -vy;
    double c = targetHeightDeltaMeters;
    double discriminant = (b * b) - (4.0 * a * c);

    if (discriminant < 0.0) {
      return OptionalDouble.empty();
    }

    double sqrtDiscriminant = Math.sqrt(discriminant);
    double rootA = (-b - sqrtDiscriminant) / (2.0 * a);
    double rootB = (-b + sqrtDiscriminant) / (2.0 * a);
    if (rootA > 0.0 && rootB > 0.0) {
      return OptionalDouble.of(Math.min(rootA, rootB));
    }
    if (rootA > 0.0) {
      return OptionalDouble.of(rootA);
    }
    if (rootB > 0.0) {
      return OptionalDouble.of(rootB);
    }
    return OptionalDouble.empty();
  }

  private static double speedToPower(double launchSpeedMetersPerSec) {
    return MathUtil.clamp(
        launchSpeedMetersPerSec / ShooterConstants.maxLaunchSpeedMetersPerSec, 0.0, 1.0);
  }

  private static double clampAirtime(double airtimeSeconds) {
    return MathUtil.clamp(
        airtimeSeconds, ShooterConstants.minAirtimeSeconds, ShooterConstants.maxAirtimeSeconds);
  }

  private void logHubShotSolution(HubShotSolution solution) {
    Logger.recordOutput("Shooter/HubShot/DistanceMeters", solution.distanceMeters());
    Logger.recordOutput("Shooter/HubShot/LaunchAngleDegrees", solution.launchAngle().getDegrees());
    Logger.recordOutput("Shooter/HubShot/LaunchSpeedMetersPerSec", solution.launchSpeedMetersPerSec());
    Logger.recordOutput("Shooter/HubShot/PowerSetpoint", solution.shooterPower());
    Logger.recordOutput("Shooter/HubShot/AirtimeSeconds", solution.airtimeSeconds());
    Logger.recordOutput("Shooter/HubShot/Feasible", solution.feasible());
  }
}
