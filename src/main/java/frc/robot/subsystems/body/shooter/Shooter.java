package frc.robot.subsystems.body.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static record WheelSpeedSetpoints(double pair1RadPerSec, double pair2RadPerSec) {}

  public static record HubShotSolution(
      double distanceMeters,
      Rotation2d launchAngle,
      double launchSpeedMetersPerSec,
      double shooterPower,
      double airtimeSeconds,
      boolean feasible) {}

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private boolean shotControlEnabled = false;
  private boolean manualHoodOverrideEnabled = false;
  private double launchHeightMeters = ShooterConstants.defaultLaunchHeightMeters;
  private double pair1WheelSetpointRadPerSec = 0.0;
  private double pair2WheelSetpointRadPerSec = 0.0;
  private double hoodSetpointMotorRotations = ShooterConstants.hoodReferenceMotorRotations;
  private double lastSimShotTimestampSeconds = Double.NEGATIVE_INFINITY;
  private HubShotSolution latestHubShotSolution =
      new HubShotSolution(
          0.0,
          ShooterConstants.defaultLaunchAngle,
          ShooterConstants.defaultLaunchSpeedMetersPerSec,
          speedToPower(ShooterConstants.defaultLaunchSpeedMetersPerSec),
          ShooterConstants.fallbackAirtimeSeconds,
          false);

  public Shooter() {
    this(new ShooterIO() {});
  }

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (shotControlEnabled) {
      io.setWheelVelocitySetpoints(pair1WheelSetpointRadPerSec, pair2WheelSetpointRadPerSec);
    } else {
      io.setWheelVelocitySetpoints(0.0, 0.0);
    }
    io.setHoodPositionSetpointRotations(hoodSetpointMotorRotations);

    logControlState();
  }

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

  public double getPair1WheelSetpointRadPerSec() {
    return pair1WheelSetpointRadPerSec;
  }

  public double getPair2WheelSetpointRadPerSec() {
    return pair2WheelSetpointRadPerSec;
  }

  public double getHoodSetpointMotorRotations() {
    return hoodSetpointMotorRotations;
  }

  public boolean isShotControlEnabled() {
    return shotControlEnabled;
  }

  public void setShotControlEnabled(boolean enabled) {
    shotControlEnabled = enabled;
    if (!enabled) {
      lastSimShotTimestampSeconds = Double.NEGATIVE_INFINITY;
    }
  }

  public void setLaunchHeightMeters(double launchHeightMeters) {
    this.launchHeightMeters = launchHeightMeters;
  }

  public double getLaunchHeightMeters() {
    return launchHeightMeters;
  }

  public boolean isManualHoodOverrideEnabled() {
    return manualHoodOverrideEnabled;
  }

  public void setManualHoodOverrideEnabled(boolean enabled) {
    manualHoodOverrideEnabled = enabled;
  }

  public void adjustHoodSetpointDegrees(double deltaDegrees) {
    adjustHoodSetpointRotations(deltaDegrees / ShooterConstants.hoodDegreesPerMotorRotation);
  }

  public void adjustHoodSetpointRotations(double deltaRotations) {
    manualHoodOverrideEnabled = true;
    hoodSetpointMotorRotations =
        MathUtil.clamp(
            hoodSetpointMotorRotations + deltaRotations,
            hoodAngleToMotorRotations(ShooterConstants.minLaunchAngle),
            hoodAngleToMotorRotations(ShooterConstants.maxLaunchAngle));
  }

  public HubShotSolution updateHubShotSolution(Pose2d robotPose, Pose2d hubPose) {
    double horizontalDistanceMeters =
        robotPose.getTranslation().getDistance(hubPose.getTranslation());
    double targetHeightDeltaMeters = ShooterConstants.hubCenterHeightMeters - launchHeightMeters;
    latestHubShotSolution = solveHubShot(horizontalDistanceMeters, targetHeightDeltaMeters);
    applyHubShotSetpoints(latestHubShotSolution);
    logHubShotSolution(latestHubShotSolution);
    return latestHubShotSolution;
  }

  public double estimateHubShotAirtimeSeconds(Pose2d robotPose, Pose2d hubPose) {
    return updateHubShotSolution(robotPose, hubPose).airtimeSeconds();
  }

  private HubShotSolution solveHubShot(
      double horizontalDistanceMeters, double targetHeightDeltaMeters) {
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
                horizontalDistanceMeters,
                targetHeightDeltaMeters,
                candidateAngle,
                launchSpeedMetersPerSec,
                true);
      }
    }

    if (bestSolution != null) {
      return bestSolution;
    }

    OptionalDouble preferredLaunchSpeed =
        calculateLaunchSpeedForTarget(
            horizontalDistanceMeters, targetHeightDeltaMeters, preferredAngle);
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
            horizontalDistanceMeters,
            targetHeightDeltaMeters,
            launchSpeedMetersPerSec,
            launchAngle);
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
                / (ShooterConstants.hubShotFarDistanceMeters
                    - ShooterConstants.hubShotNearDistanceMeters),
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

  private void applyHubShotSetpoints(HubShotSolution solution) {
    if (!manualHoodOverrideEnabled) {
      Rotation2d clampedHoodAngle =
          new Rotation2d(
              MathUtil.clamp(
                  solution.launchAngle().getRadians(),
                  ShooterConstants.minLaunchAngle.getRadians(),
                  ShooterConstants.maxLaunchAngle.getRadians()));
      hoodSetpointMotorRotations = hoodAngleToMotorRotations(clampedHoodAngle);
    }

    WheelSpeedSetpoints wheelSetpoints =
        calculateWheelSetpointsFromLaunchSpeed(solution.launchSpeedMetersPerSec());
    pair1WheelSetpointRadPerSec = wheelSetpoints.pair1RadPerSec();
    pair2WheelSetpointRadPerSec = wheelSetpoints.pair2RadPerSec();
  }

  private WheelSpeedSetpoints calculateWheelSetpointsFromLaunchSpeed(
      double launchSpeedMetersPerSec) {
    double launchSpeedMagnitude = Math.abs(launchSpeedMetersPerSec);
    double baseWheelSurfaceSpeedMetersPerSec =
        launchSpeedMagnitude / ShooterConstants.launchSlipFactor;

    // Differential wheel speed creates controlled backspin.
    double ballSpinRadPerSec =
        (ShooterConstants.targetBallSpinRatio * launchSpeedMagnitude)
            / ShooterConstants.fuelBallRadiusMeters;
    double spinSurfaceDeltaMetersPerSec =
        ballSpinRadPerSec * ShooterConstants.fuelBallRadiusMeters;

    double pair1WheelSpeedRadPerSec =
        (baseWheelSurfaceSpeedMetersPerSec + spinSurfaceDeltaMetersPerSec)
            / ShooterConstants.shooterWheelRadiusMeters;
    double pair2WheelSpeedRadPerSec =
        (baseWheelSurfaceSpeedMetersPerSec - spinSurfaceDeltaMetersPerSec)
            / ShooterConstants.shooterWheelRadiusMeters;

    return new WheelSpeedSetpoints(
        clampWheelSpeedRadPerSec(pair1WheelSpeedRadPerSec),
        clampWheelSpeedRadPerSec(pair2WheelSpeedRadPerSec));
  }

  private static double clampWheelSpeedRadPerSec(double wheelSpeedRadPerSec) {
    return MathUtil.clamp(
        wheelSpeedRadPerSec,
        ShooterConstants.minWheelSpeedRadPerSec,
        ShooterConstants.maxWheelSpeedRadPerSec);
  }

  public static double hoodAngleToMotorRotations(Rotation2d hoodAngle) {
    return ShooterConstants.hoodReferenceMotorRotations
        + (hoodAngle.minus(ShooterConstants.hoodReferenceAngle).getDegrees()
            / ShooterConstants.hoodDegreesPerMotorRotation);
  }

  public static Rotation2d motorRotationsToHoodAngle(double hoodMotorRotations) {
    double hoodAngleDegrees =
        ShooterConstants.hoodReferenceAngle.getDegrees()
            + (hoodMotorRotations - ShooterConstants.hoodReferenceMotorRotations)
                * ShooterConstants.hoodDegreesPerMotorRotation;
    return Rotation2d.fromDegrees(hoodAngleDegrees);
  }

  public Rotation2d getMeasuredHoodAngle() {
    return motorRotationsToHoodAngle(inputs.hoodPositionRotations);
  }

  public double getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec() {
    double pair1SurfaceSpeedMetersPerSec =
        Math.abs(inputs.pair1LeaderVelocityRadPerSec) * ShooterConstants.shooterWheelRadiusMeters;
    double pair2SurfaceSpeedMetersPerSec =
        Math.abs(inputs.pair2LeaderVelocityRadPerSec) * ShooterConstants.shooterWheelRadiusMeters;
    double averageWheelSurfaceSpeedMetersPerSec =
        0.5 * (pair1SurfaceSpeedMetersPerSec + pair2SurfaceSpeedMetersPerSec);
    return ShooterConstants.launchSlipFactor * averageWheelSurfaceSpeedMetersPerSec;
  }

  public boolean shouldTriggerSimulatedShot(double timestampSeconds) {
    if (!areWheelsReadyForSimulatedShot()) {
      return false;
    }

    if ((timestampSeconds - lastSimShotTimestampSeconds) < ShooterConstants.simShotCadenceSeconds) {
      return false;
    }

    lastSimShotTimestampSeconds = timestampSeconds;
    return true;
  }

  private boolean areWheelsReadyForSimulatedShot() {
    if (!shotControlEnabled) {
      return false;
    }

    double pair1SetpointMagnitude = Math.abs(pair1WheelSetpointRadPerSec);
    double pair2SetpointMagnitude = Math.abs(pair2WheelSetpointRadPerSec);
    if (pair1SetpointMagnitude < 1e-3 || pair2SetpointMagnitude < 1e-3) {
      return false;
    }

    double pair1MeasuredMagnitude = Math.abs(inputs.pair1LeaderVelocityRadPerSec);
    double pair2MeasuredMagnitude = Math.abs(inputs.pair2LeaderVelocityRadPerSec);

    return pair1MeasuredMagnitude >= (pair1SetpointMagnitude * ShooterConstants.simWheelReadyRatio)
        && pair2MeasuredMagnitude >= (pair2SetpointMagnitude * ShooterConstants.simWheelReadyRatio);
  }

  private double estimateLaunchSpeedFromWheelSetpointsMetersPerSec() {
    double pair1SurfaceSpeedMetersPerSec =
        pair1WheelSetpointRadPerSec * ShooterConstants.shooterWheelRadiusMeters;
    double pair2SurfaceSpeedMetersPerSec =
        pair2WheelSetpointRadPerSec * ShooterConstants.shooterWheelRadiusMeters;
    double averageWheelSurfaceSpeedMetersPerSec =
        0.5 * (pair1SurfaceSpeedMetersPerSec + pair2SurfaceSpeedMetersPerSec);
    return ShooterConstants.launchSlipFactor * averageWheelSurfaceSpeedMetersPerSec;
  }

  private static double clampAirtime(double airtimeSeconds) {
    return MathUtil.clamp(
        airtimeSeconds, ShooterConstants.minAirtimeSeconds, ShooterConstants.maxAirtimeSeconds);
  }

  private void logControlState() {
    Logger.recordOutput("Shooter/Control/Enabled", shotControlEnabled);
    Logger.recordOutput("Shooter/Control/Pair1SetpointRadPerSec", pair1WheelSetpointRadPerSec);
    Logger.recordOutput("Shooter/Control/Pair2SetpointRadPerSec", pair2WheelSetpointRadPerSec);
    Logger.recordOutput(
        "Shooter/Control/HoodSetpointRotations", hoodSetpointMotorRotations);
    Logger.recordOutput("Shooter/Control/ManualHoodOverrideEnabled", manualHoodOverrideEnabled);
    Logger.recordOutput(
        "Shooter/Control/HoodSetpointDegrees",
        motorRotationsToHoodAngle(hoodSetpointMotorRotations).getDegrees());
    Logger.recordOutput(
        "Shooter/Measured/Pair1VelocityRadPerSec", inputs.pair1LeaderVelocityRadPerSec);
    Logger.recordOutput(
        "Shooter/Measured/Pair2VelocityRadPerSec", inputs.pair2LeaderVelocityRadPerSec);
    Logger.recordOutput("Shooter/Measured/HoodPositionRotations", inputs.hoodPositionRotations);
    Logger.recordOutput(
        "Shooter/Measured/HoodAngleDegrees",
        motorRotationsToHoodAngle(inputs.hoodPositionRotations).getDegrees());
    Logger.recordOutput(
        "Shooter/Measured/EstimatedLaunchSpeedMetersPerSec",
        getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec());
    Logger.recordOutput("Shooter/Simulation/WheelsReady", areWheelsReadyForSimulatedShot());
    Logger.recordOutput("Shooter/Status/Pair1Connected", inputs.pair1Connected);
    Logger.recordOutput("Shooter/Status/Pair2Connected", inputs.pair2Connected);
    Logger.recordOutput("Shooter/Status/HoodConnected", inputs.hoodConnected);
  }

  private void logHubShotSolution(HubShotSolution solution) {
    Logger.recordOutput("Shooter/HubShot/DistanceMeters", solution.distanceMeters());
    Logger.recordOutput("Shooter/HubShot/LaunchAngleDegrees", solution.launchAngle().getDegrees());
    Logger.recordOutput(
        "Shooter/HubShot/LaunchSpeedMetersPerSec", solution.launchSpeedMetersPerSec());
    Logger.recordOutput(
        "Shooter/HubShot/EstimatedLaunchSpeedFromWheelsMetersPerSec",
        estimateLaunchSpeedFromWheelSetpointsMetersPerSec());
    Logger.recordOutput(
        "Shooter/HubShot/EstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec",
        getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec());
    Logger.recordOutput("Shooter/HubShot/PowerSetpoint", solution.shooterPower());
    Logger.recordOutput("Shooter/HubShot/AirtimeSeconds", solution.airtimeSeconds());
    Logger.recordOutput("Shooter/HubShot/Feasible", solution.feasible());
  }
}
