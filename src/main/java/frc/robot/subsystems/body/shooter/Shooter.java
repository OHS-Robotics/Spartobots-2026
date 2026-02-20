package frc.robot.subsystems.body.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final double MIN_NUMERIC_EPSILON = 1e-6;
  private static final double MIN_LAUNCH_ANGLE_SPAN_DEGREES = 1.0;

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
  private final NetworkTable configTable =
      NetworkTableInstance.getDefault().getTable(ShooterConstants.configTableName);

  private final NetworkTableEntry shotControlEnabledEntry = configTable.getEntry("Control/Enabled");
  private final NetworkTableEntry manualHoodOverrideEnabledEntry =
      configTable.getEntry("Control/ManualHoodOverrideEnabled");
  private final NetworkTableEntry manualSetpointsEnabledEntry =
      configTable.getEntry("Control/ManualSetpointsEnabled");
  private final NetworkTableEntry manualPair1SetpointEntry =
      configTable.getEntry("Control/ManualPair1SetpointRadPerSec");
  private final NetworkTableEntry manualPair2SetpointEntry =
      configTable.getEntry("Control/ManualPair2SetpointRadPerSec");
  private final NetworkTableEntry manualHoodSetpointRotationsEntry =
      configTable.getEntry("Control/ManualHoodSetpointRotations");
  private final NetworkTableEntry launchHeightMetersEntry =
      configTable.getEntry("Control/LaunchHeightMeters");

  private final NetworkTableEntry gravityMetersPerSecSquaredEntry =
      configTable.getEntry("Model/GravityMetersPerSecSquared");
  private final NetworkTableEntry shooterWheelRadiusMetersEntry =
      configTable.getEntry("Model/ShooterWheelRadiusMeters");
  private final NetworkTableEntry fuelBallRadiusMetersEntry =
      configTable.getEntry("Model/FuelBallRadiusMeters");
  private final NetworkTableEntry launchSlipFactorEntry = configTable.getEntry("Model/LaunchSlipFactor");
  private final NetworkTableEntry targetBallSpinRatioEntry =
      configTable.getEntry("Model/TargetBallSpinRatio");
  private final NetworkTableEntry minWheelSpeedRadPerSecEntry =
      configTable.getEntry("Model/MinWheelSpeedRadPerSec");
  private final NetworkTableEntry maxWheelSpeedRadPerSecEntry =
      configTable.getEntry("Model/MaxWheelSpeedRadPerSec");
  private final NetworkTableEntry hoodReferenceAngleDegreesEntry =
      configTable.getEntry("Model/HoodReferenceAngleDegrees");
  private final NetworkTableEntry hoodReferenceMotorRotationsEntry =
      configTable.getEntry("Model/HoodReferenceMotorRotations");
  private final NetworkTableEntry hoodDegreesPerMotorRotationEntry =
      configTable.getEntry("Model/HoodDegreesPerMotorRotation");
  private final NetworkTableEntry hubCenterHeightMetersEntry =
      configTable.getEntry("Model/HubCenterHeightMeters");
  private final NetworkTableEntry hubShotNearDistanceMetersEntry =
      configTable.getEntry("Model/HubShotNearDistanceMeters");
  private final NetworkTableEntry hubShotFarDistanceMetersEntry =
      configTable.getEntry("Model/HubShotFarDistanceMeters");
  private final NetworkTableEntry hubShotNearPreferredAngleDegreesEntry =
      configTable.getEntry("Model/HubShotNearPreferredAngleDegrees");
  private final NetworkTableEntry hubShotFarPreferredAngleDegreesEntry =
      configTable.getEntry("Model/HubShotFarPreferredAngleDegrees");
  private final NetworkTableEntry minLaunchAngleDegreesEntry =
      configTable.getEntry("Model/MinLaunchAngleDegrees");
  private final NetworkTableEntry maxLaunchAngleDegreesEntry =
      configTable.getEntry("Model/MaxLaunchAngleDegrees");
  private final NetworkTableEntry launchAngleSearchStepDegreesEntry =
      configTable.getEntry("Model/LaunchAngleSearchStepDegrees");
  private final NetworkTableEntry defaultLaunchSpeedMetersPerSecEntry =
      configTable.getEntry("Model/DefaultLaunchSpeedMetersPerSec");
  private final NetworkTableEntry defaultLaunchAngleDegreesEntry =
      configTable.getEntry("Model/DefaultLaunchAngleDegrees");
  private final NetworkTableEntry minAirtimeSecondsEntry =
      configTable.getEntry("Model/MinAirtimeSeconds");
  private final NetworkTableEntry maxAirtimeSecondsEntry =
      configTable.getEntry("Model/MaxAirtimeSeconds");
  private final NetworkTableEntry fallbackAirtimeSecondsEntry =
      configTable.getEntry("Model/FallbackAirtimeSeconds");
  private final NetworkTableEntry minHorizontalVelocityMetersPerSecEntry =
      configTable.getEntry("Model/MinHorizontalVelocityMetersPerSec");
  private final NetworkTableEntry simShotCadenceSecondsEntry =
      configTable.getEntry("Model/SimShotCadenceSeconds");
  private final NetworkTableEntry simWheelReadyRatioEntry =
      configTable.getEntry("Model/SimWheelReadyRatio");

  private boolean shotControlEnabled = false;
  private boolean triggerSpeedBoostEnabled = false;
  private boolean manualHoodOverrideEnabled = false;
  private boolean manualSetpointsEnabled = false;
  private double launchHeightMeters = ShooterConstants.defaultLaunchHeightMeters;
  private double pair1WheelSetpointRadPerSec = 0.0;
  private double pair2WheelSetpointRadPerSec = 0.0;
  private double hoodSetpointMotorRotations = ShooterConstants.hoodReferenceMotorRotations;
  private double lastSimShotTimestampSeconds = Double.NEGATIVE_INFINITY;
  private double gravityMetersPerSecSquared = ShooterConstants.gravityMetersPerSecSquared;
  private double shooterWheelRadiusMeters = ShooterConstants.shooterWheelRadiusMeters;
  private double fuelBallRadiusMeters = ShooterConstants.fuelBallRadiusMeters;
  private double launchSlipFactor = ShooterConstants.launchSlipFactor;
  private double targetBallSpinRatio = ShooterConstants.targetBallSpinRatio;
  private double minWheelSpeedRadPerSec = ShooterConstants.minWheelSpeedRadPerSec;
  private double maxWheelSpeedRadPerSec = ShooterConstants.maxWheelSpeedRadPerSec;
  private double hoodReferenceAngleDegrees = ShooterConstants.hoodReferenceAngle.getDegrees();
  private double hoodReferenceMotorRotations = ShooterConstants.hoodReferenceMotorRotations;
  private double hoodDegreesPerMotorRotation = ShooterConstants.hoodDegreesPerMotorRotation;
  private double hubCenterHeightMeters = ShooterConstants.hubCenterHeightMeters;
  private double hubShotNearDistanceMeters = ShooterConstants.hubShotNearDistanceMeters;
  private double hubShotFarDistanceMeters = ShooterConstants.hubShotFarDistanceMeters;
  private double hubShotNearPreferredAngleDegrees =
      ShooterConstants.hubShotNearPreferredAngle.getDegrees();
  private double hubShotFarPreferredAngleDegrees = ShooterConstants.hubShotFarPreferredAngle.getDegrees();
  private double minLaunchAngleDegrees = ShooterConstants.minLaunchAngle.getDegrees();
  private double maxLaunchAngleDegrees = ShooterConstants.maxLaunchAngle.getDegrees();
  private double launchAngleSearchStepDegrees = ShooterConstants.launchAngleSearchStepDegrees;
  private double minLaunchSpeedMetersPerSec = ShooterConstants.minLaunchSpeedMetersPerSec;
  private double maxLaunchSpeedMetersPerSec = ShooterConstants.maxLaunchSpeedMetersPerSec;
  private double defaultLaunchSpeedMetersPerSec = ShooterConstants.defaultLaunchSpeedMetersPerSec;
  private double defaultLaunchAngleDegrees = ShooterConstants.defaultLaunchAngle.getDegrees();
  private double minAirtimeSeconds = ShooterConstants.minAirtimeSeconds;
  private double maxAirtimeSeconds = ShooterConstants.maxAirtimeSeconds;
  private double fallbackAirtimeSeconds = ShooterConstants.fallbackAirtimeSeconds;
  private double minHorizontalVelocityMetersPerSec = ShooterConstants.minHorizontalVelocityMetersPerSec;
  private double simShotCadenceSeconds = ShooterConstants.simShotCadenceSeconds;
  private double simWheelReadyRatio = ShooterConstants.simWheelReadyRatio;
  private HubShotSolution latestHubShotSolution =
      new HubShotSolution(
          0.0,
          ShooterConstants.defaultLaunchAngle,
          ShooterConstants.defaultLaunchSpeedMetersPerSec,
          speedToPower(
              ShooterConstants.defaultLaunchSpeedMetersPerSec,
              ShooterConstants.maxLaunchSpeedMetersPerSec),
          ShooterConstants.fallbackAirtimeSeconds,
          false);

  public Shooter() {
    this(new ShooterIO() {});
  }

  public Shooter(ShooterIO io) {
    this.io = io;
    configureNetworkTableDefaults();
    loadNetworkTableConfig();
    hoodSetpointMotorRotations = hoodReferenceMotorRotations;
    latestHubShotSolution =
        new HubShotSolution(
            0.0,
            Rotation2d.fromDegrees(defaultLaunchAngleDegrees),
            defaultLaunchSpeedMetersPerSec,
            speedToPower(defaultLaunchSpeedMetersPerSec),
            fallbackAirtimeSeconds,
            false);
  }

  @Override
  public void periodic() {
    loadNetworkTableConfig();
    if (manualSetpointsEnabled) {
      applyManualSetpointsFromNetworkTables();
    }

    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (shotControlEnabled) {
      io.setWheelVelocitySetpoints(pair1WheelSetpointRadPerSec, pair2WheelSetpointRadPerSec);
    } else {
      io.setWheelVelocitySetpoints(0.0, 0.0);
    }
    io.setHoodPositionSetpointRotations(hoodSetpointMotorRotations);

    logControlState();
    logTuningState();
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
    shotControlEnabledEntry.setBoolean(enabled);
    if (!enabled) {
      lastSimShotTimestampSeconds = Double.NEGATIVE_INFINITY;
    }
  }

  public void setTriggerSpeedBoostEnabled(boolean enabled) {
    triggerSpeedBoostEnabled = enabled;
  }

  public void setLaunchHeightMeters(double launchHeightMeters) {
    this.launchHeightMeters = Math.max(0.0, launchHeightMeters);
    launchHeightMetersEntry.setDouble(this.launchHeightMeters);
  }

  public double getLaunchHeightMeters() {
    return launchHeightMeters;
  }

  public double getHubCenterHeightMeters() {
    return hubCenterHeightMeters;
  }

  public double getMinLaunchSpeedMetersPerSec() {
    return minLaunchSpeedMetersPerSec;
  }

  public double getMaxLaunchSpeedMetersPerSec() {
    return maxLaunchSpeedMetersPerSec;
  }

  public boolean isManualHoodOverrideEnabled() {
    return manualHoodOverrideEnabled;
  }

  public void setManualHoodOverrideEnabled(boolean enabled) {
    manualHoodOverrideEnabled = enabled;
    manualHoodOverrideEnabledEntry.setBoolean(enabled);
  }

  public void adjustHoodSetpointDegrees(double deltaDegrees) {
    if (Math.abs(hoodDegreesPerMotorRotation) < MIN_NUMERIC_EPSILON) {
      return;
    }
    adjustHoodSetpointRotations(deltaDegrees / hoodDegreesPerMotorRotation);
  }

  public void adjustHoodSetpointRotations(double deltaRotations) {
    manualHoodOverrideEnabled = true;
    manualHoodOverrideEnabledEntry.setBoolean(true);
    hoodSetpointMotorRotations = clampHoodSetpointRotations(hoodSetpointMotorRotations + deltaRotations);
    manualHoodSetpointRotationsEntry.setDouble(hoodSetpointMotorRotations);
  }

  public HubShotSolution updateHubShotSolution(Pose2d robotPose, Pose2d hubPose) {
    double horizontalDistanceMeters =
        robotPose.getTranslation().getDistance(hubPose.getTranslation());
    double targetHeightDeltaMeters = hubCenterHeightMeters - launchHeightMeters;
    HubShotSolution solvedShot = solveHubShot(horizontalDistanceMeters, targetHeightDeltaMeters);
    if (triggerSpeedBoostEnabled && !manualSetpointsEnabled) {
      solvedShot =
          applyLaunchSpeedScale(
              solvedShot, ShooterConstants.triggerLaunchSpeedBoostScale, targetHeightDeltaMeters);
    }
    latestHubShotSolution = solvedShot;
    if (!manualSetpointsEnabled) {
      applyHubShotSetpoints(latestHubShotSolution);
    }
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
    for (double degrees = minLaunchAngleDegrees;
        degrees <= (maxLaunchAngleDegrees + MIN_NUMERIC_EPSILON);
        degrees += launchAngleSearchStepDegrees) {
      Rotation2d candidateAngle = Rotation2d.fromDegrees(degrees);
      OptionalDouble launchSpeedResult =
          calculateLaunchSpeedForTarget(
              horizontalDistanceMeters, targetHeightDeltaMeters, candidateAngle);
      if (launchSpeedResult.isEmpty()) {
        continue;
      }

      double launchSpeedMetersPerSec = launchSpeedResult.getAsDouble();
      if (launchSpeedMetersPerSec < minLaunchSpeedMetersPerSec
          || launchSpeedMetersPerSec > maxLaunchSpeedMetersPerSec) {
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
    double fallbackLaunchSpeedMetersPerSec = preferredLaunchSpeed.orElse(defaultLaunchSpeedMetersPerSec);
    fallbackLaunchSpeedMetersPerSec =
        MathUtil.clamp(
            fallbackLaunchSpeedMetersPerSec, minLaunchSpeedMetersPerSec, maxLaunchSpeedMetersPerSec);

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

  private HubShotSolution applyLaunchSpeedScale(
      HubShotSolution baseSolution, double speedScale, double targetHeightDeltaMeters) {
    double clampedScale = Math.max(0.0, speedScale);
    double boostedLaunchSpeedMetersPerSec =
        MathUtil.clamp(
            baseSolution.launchSpeedMetersPerSec() * clampedScale,
            minLaunchSpeedMetersPerSec,
            maxLaunchSpeedMetersPerSec);
    double boostedAirtimeSeconds =
        calculateAirtimeSeconds(
            baseSolution.distanceMeters(),
            targetHeightDeltaMeters,
            boostedLaunchSpeedMetersPerSec,
            baseSolution.launchAngle());
    return new HubShotSolution(
        baseSolution.distanceMeters(),
        baseSolution.launchAngle(),
        boostedLaunchSpeedMetersPerSec,
        speedToPower(boostedLaunchSpeedMetersPerSec),
        boostedAirtimeSeconds,
        baseSolution.feasible());
  }

  private Rotation2d getPreferredAngle(double horizontalDistanceMeters) {
    double distanceSpan =
        Math.max(hubShotFarDistanceMeters - hubShotNearDistanceMeters, MIN_NUMERIC_EPSILON);
    double distanceProgress =
        MathUtil.clamp(
            (horizontalDistanceMeters - hubShotNearDistanceMeters) / distanceSpan,
            0.0,
            1.0);
    double preferredAngleDegrees =
        MathUtil.interpolate(
            hubShotNearPreferredAngleDegrees,
            hubShotFarPreferredAngleDegrees,
            distanceProgress);
    preferredAngleDegrees =
        MathUtil.clamp(preferredAngleDegrees, minLaunchAngleDegrees, maxLaunchAngleDegrees);
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
        gravityMetersPerSecSquared * horizontalDistanceMeters * horizontalDistanceMeters / denominator;
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
    if (Math.abs(vx) < minHorizontalVelocityMetersPerSec) {
      return fallbackAirtimeSeconds;
    }

    double vy = launchSpeedMetersPerSec * Math.sin(launchAngle.getRadians());
    double airtimeByHorizontal = horizontalDistanceMeters / Math.abs(vx);
    double airtimeByVertical =
        solveVerticalAirtime(vy, targetHeightDeltaMeters).orElse(airtimeByHorizontal);

    return clampAirtime(0.5 * (airtimeByHorizontal + airtimeByVertical));
  }

  private OptionalDouble solveVerticalAirtime(double vy, double targetHeightDeltaMeters) {
    double a = 0.5 * gravityMetersPerSecSquared;
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

  private static double speedToPower(double launchSpeedMetersPerSec, double maxLaunchSpeedMetersPerSec) {
    if (maxLaunchSpeedMetersPerSec <= MIN_NUMERIC_EPSILON) {
      return 0.0;
    }
    return MathUtil.clamp(launchSpeedMetersPerSec / maxLaunchSpeedMetersPerSec, 0.0, 1.0);
  }

  private double speedToPower(double launchSpeedMetersPerSec) {
    return speedToPower(launchSpeedMetersPerSec, maxLaunchSpeedMetersPerSec);
  }

  private void applyHubShotSetpoints(HubShotSolution solution) {
    if (!manualHoodOverrideEnabled) {
      Rotation2d clampedHoodAngle =
          new Rotation2d(
              MathUtil.clamp(
                  solution.launchAngle().getRadians(),
                  Rotation2d.fromDegrees(minLaunchAngleDegrees).getRadians(),
                  Rotation2d.fromDegrees(maxLaunchAngleDegrees).getRadians()));
      hoodSetpointMotorRotations = hoodAngleToConfiguredMotorRotations(clampedHoodAngle);
      manualHoodSetpointRotationsEntry.setDouble(hoodSetpointMotorRotations);
    }

    WheelSpeedSetpoints wheelSetpoints =
        calculateWheelSetpointsFromLaunchSpeed(solution.launchSpeedMetersPerSec());
    pair1WheelSetpointRadPerSec = wheelSetpoints.pair1RadPerSec();
    pair2WheelSetpointRadPerSec = wheelSetpoints.pair2RadPerSec();
    manualPair1SetpointEntry.setDouble(pair1WheelSetpointRadPerSec);
    manualPair2SetpointEntry.setDouble(pair2WheelSetpointRadPerSec);
  }

  private WheelSpeedSetpoints calculateWheelSetpointsFromLaunchSpeed(
      double launchSpeedMetersPerSec) {
    double launchSpeedMagnitude = Math.abs(launchSpeedMetersPerSec);
    double baseWheelSurfaceSpeedMetersPerSec = launchSpeedMagnitude / launchSlipFactor;

    // Differential wheel speed creates controlled backspin.
    double ballSpinRadPerSec = (targetBallSpinRatio * launchSpeedMagnitude) / fuelBallRadiusMeters;
    double spinSurfaceDeltaMetersPerSec = ballSpinRadPerSec * fuelBallRadiusMeters;

    double pair1WheelSpeedRadPerSec =
        (baseWheelSurfaceSpeedMetersPerSec + spinSurfaceDeltaMetersPerSec) / shooterWheelRadiusMeters;
    double pair2WheelSpeedRadPerSec =
        (baseWheelSurfaceSpeedMetersPerSec - spinSurfaceDeltaMetersPerSec) / shooterWheelRadiusMeters;

    return new WheelSpeedSetpoints(
        clampWheelSpeedRadPerSec(pair1WheelSpeedRadPerSec),
        clampWheelSpeedRadPerSec(pair2WheelSpeedRadPerSec));
  }

  private double clampWheelSpeedRadPerSec(double wheelSpeedRadPerSec) {
    return MathUtil.clamp(wheelSpeedRadPerSec, minWheelSpeedRadPerSec, maxWheelSpeedRadPerSec);
  }

  private double clampManualWheelSetpointRadPerSec(double wheelSpeedRadPerSec) {
    return MathUtil.clamp(wheelSpeedRadPerSec, -maxWheelSpeedRadPerSec, maxWheelSpeedRadPerSec);
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

  private double hoodAngleToConfiguredMotorRotations(Rotation2d hoodAngle) {
    return hoodReferenceMotorRotations
        + ((hoodAngle.getDegrees() - hoodReferenceAngleDegrees) / hoodDegreesPerMotorRotation);
  }

  private Rotation2d configuredMotorRotationsToHoodAngle(double hoodMotorRotations) {
    double hoodAngleDegrees =
        hoodReferenceAngleDegrees
            + ((hoodMotorRotations - hoodReferenceMotorRotations) * hoodDegreesPerMotorRotation);
    return Rotation2d.fromDegrees(hoodAngleDegrees);
  }

  private double clampHoodSetpointRotations(double hoodSetpointRotations) {
    double minHoodRotations =
        hoodAngleToConfiguredMotorRotations(Rotation2d.fromDegrees(minLaunchAngleDegrees));
    double maxHoodRotations =
        hoodAngleToConfiguredMotorRotations(Rotation2d.fromDegrees(maxLaunchAngleDegrees));
    return MathUtil.clamp(
        hoodSetpointRotations, Math.min(minHoodRotations, maxHoodRotations), Math.max(minHoodRotations, maxHoodRotations));
  }

  public Rotation2d getMeasuredHoodAngle() {
    return configuredMotorRotationsToHoodAngle(inputs.hoodPositionRotations);
  }

  public double getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec() {
    double pair1SurfaceSpeedMetersPerSec =
        Math.abs(inputs.pair1LeaderVelocityRadPerSec) * shooterWheelRadiusMeters;
    double pair2SurfaceSpeedMetersPerSec =
        Math.abs(inputs.pair2LeaderVelocityRadPerSec) * shooterWheelRadiusMeters;
    double averageWheelSurfaceSpeedMetersPerSec =
        0.5 * (pair1SurfaceSpeedMetersPerSec + pair2SurfaceSpeedMetersPerSec);
    return launchSlipFactor * averageWheelSurfaceSpeedMetersPerSec;
  }

  public boolean shouldTriggerSimulatedShot(double timestampSeconds) {
    if (!areWheelsReadyForSimulatedShot()) {
      return false;
    }

    if ((timestampSeconds - lastSimShotTimestampSeconds) < simShotCadenceSeconds) {
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

    return pair1MeasuredMagnitude >= (pair1SetpointMagnitude * simWheelReadyRatio)
        && pair2MeasuredMagnitude >= (pair2SetpointMagnitude * simWheelReadyRatio);
  }

  private double estimateLaunchSpeedFromWheelSetpointsMetersPerSec() {
    double pair1SurfaceSpeedMetersPerSec =
        pair1WheelSetpointRadPerSec * shooterWheelRadiusMeters;
    double pair2SurfaceSpeedMetersPerSec =
        pair2WheelSetpointRadPerSec * shooterWheelRadiusMeters;
    double averageWheelSurfaceSpeedMetersPerSec =
        0.5 * (pair1SurfaceSpeedMetersPerSec + pair2SurfaceSpeedMetersPerSec);
    return launchSlipFactor * averageWheelSurfaceSpeedMetersPerSec;
  }

  private double clampAirtime(double airtimeSeconds) {
    return MathUtil.clamp(airtimeSeconds, minAirtimeSeconds, maxAirtimeSeconds);
  }

  private void configureNetworkTableDefaults() {
    shotControlEnabledEntry.setDefaultBoolean(false);
    manualHoodOverrideEnabledEntry.setDefaultBoolean(false);
    manualSetpointsEnabledEntry.setDefaultBoolean(false);
    manualPair1SetpointEntry.setDefaultDouble(0.0);
    manualPair2SetpointEntry.setDefaultDouble(0.0);
    manualHoodSetpointRotationsEntry.setDefaultDouble(ShooterConstants.hoodReferenceMotorRotations);
    launchHeightMetersEntry.setDefaultDouble(ShooterConstants.defaultLaunchHeightMeters);

    gravityMetersPerSecSquaredEntry.setDefaultDouble(ShooterConstants.gravityMetersPerSecSquared);
    shooterWheelRadiusMetersEntry.setDefaultDouble(ShooterConstants.shooterWheelRadiusMeters);
    fuelBallRadiusMetersEntry.setDefaultDouble(ShooterConstants.fuelBallRadiusMeters);
    launchSlipFactorEntry.setDefaultDouble(ShooterConstants.launchSlipFactor);
    targetBallSpinRatioEntry.setDefaultDouble(ShooterConstants.targetBallSpinRatio);
    minWheelSpeedRadPerSecEntry.setDefaultDouble(ShooterConstants.minWheelSpeedRadPerSec);
    maxWheelSpeedRadPerSecEntry.setDefaultDouble(ShooterConstants.maxWheelSpeedRadPerSec);
    hoodReferenceAngleDegreesEntry.setDefaultDouble(ShooterConstants.hoodReferenceAngle.getDegrees());
    hoodReferenceMotorRotationsEntry.setDefaultDouble(ShooterConstants.hoodReferenceMotorRotations);
    hoodDegreesPerMotorRotationEntry.setDefaultDouble(ShooterConstants.hoodDegreesPerMotorRotation);
    hubCenterHeightMetersEntry.setDefaultDouble(ShooterConstants.hubCenterHeightMeters);
    hubShotNearDistanceMetersEntry.setDefaultDouble(ShooterConstants.hubShotNearDistanceMeters);
    hubShotFarDistanceMetersEntry.setDefaultDouble(ShooterConstants.hubShotFarDistanceMeters);
    hubShotNearPreferredAngleDegreesEntry.setDefaultDouble(
        ShooterConstants.hubShotNearPreferredAngle.getDegrees());
    hubShotFarPreferredAngleDegreesEntry.setDefaultDouble(
        ShooterConstants.hubShotFarPreferredAngle.getDegrees());
    minLaunchAngleDegreesEntry.setDefaultDouble(ShooterConstants.minLaunchAngle.getDegrees());
    maxLaunchAngleDegreesEntry.setDefaultDouble(ShooterConstants.maxLaunchAngle.getDegrees());
    launchAngleSearchStepDegreesEntry.setDefaultDouble(ShooterConstants.launchAngleSearchStepDegrees);
    defaultLaunchSpeedMetersPerSecEntry.setDefaultDouble(ShooterConstants.defaultLaunchSpeedMetersPerSec);
    defaultLaunchAngleDegreesEntry.setDefaultDouble(ShooterConstants.defaultLaunchAngle.getDegrees());
    minAirtimeSecondsEntry.setDefaultDouble(ShooterConstants.minAirtimeSeconds);
    maxAirtimeSecondsEntry.setDefaultDouble(ShooterConstants.maxAirtimeSeconds);
    fallbackAirtimeSecondsEntry.setDefaultDouble(ShooterConstants.fallbackAirtimeSeconds);
    minHorizontalVelocityMetersPerSecEntry.setDefaultDouble(
        ShooterConstants.minHorizontalVelocityMetersPerSec);
    simShotCadenceSecondsEntry.setDefaultDouble(ShooterConstants.simShotCadenceSeconds);
    simWheelReadyRatioEntry.setDefaultDouble(ShooterConstants.simWheelReadyRatio);
  }

  private void loadNetworkTableConfig() {
    shotControlEnabled = shotControlEnabledEntry.getBoolean(shotControlEnabled);
    manualHoodOverrideEnabled = manualHoodOverrideEnabledEntry.getBoolean(manualHoodOverrideEnabled);
    manualSetpointsEnabled = manualSetpointsEnabledEntry.getBoolean(manualSetpointsEnabled);
    launchHeightMeters = Math.max(0.0, launchHeightMetersEntry.getDouble(launchHeightMeters));

    gravityMetersPerSecSquared =
        Math.max(0.01, gravityMetersPerSecSquaredEntry.getDouble(gravityMetersPerSecSquared));
    shooterWheelRadiusMeters =
        Math.max(MIN_NUMERIC_EPSILON, shooterWheelRadiusMetersEntry.getDouble(shooterWheelRadiusMeters));
    fuelBallRadiusMeters =
        Math.max(MIN_NUMERIC_EPSILON, fuelBallRadiusMetersEntry.getDouble(fuelBallRadiusMeters));
    launchSlipFactor = Math.max(0.01, launchSlipFactorEntry.getDouble(launchSlipFactor));
    targetBallSpinRatio =
        MathUtil.clamp(targetBallSpinRatioEntry.getDouble(targetBallSpinRatio), -1.0, 1.0);
    minWheelSpeedRadPerSec = Math.max(0.0, minWheelSpeedRadPerSecEntry.getDouble(minWheelSpeedRadPerSec));
    maxWheelSpeedRadPerSec =
        Math.max(
            minWheelSpeedRadPerSec + MIN_NUMERIC_EPSILON,
            maxWheelSpeedRadPerSecEntry.getDouble(maxWheelSpeedRadPerSec));

    hoodReferenceAngleDegrees = hoodReferenceAngleDegreesEntry.getDouble(hoodReferenceAngleDegrees);
    hoodReferenceMotorRotations =
        hoodReferenceMotorRotationsEntry.getDouble(hoodReferenceMotorRotations);
    double loadedHoodDegreesPerMotorRotation =
        hoodDegreesPerMotorRotationEntry.getDouble(hoodDegreesPerMotorRotation);
    if (Math.abs(loadedHoodDegreesPerMotorRotation) < MIN_NUMERIC_EPSILON) {
      loadedHoodDegreesPerMotorRotation =
          Math.copySign(MIN_NUMERIC_EPSILON, hoodDegreesPerMotorRotation);
    }
    hoodDegreesPerMotorRotation = loadedHoodDegreesPerMotorRotation;

    hubCenterHeightMeters = Math.max(0.0, hubCenterHeightMetersEntry.getDouble(hubCenterHeightMeters));
    hubShotNearDistanceMeters =
        Math.max(0.0, hubShotNearDistanceMetersEntry.getDouble(hubShotNearDistanceMeters));
    hubShotFarDistanceMeters =
        Math.max(
            hubShotNearDistanceMeters + MIN_NUMERIC_EPSILON,
            hubShotFarDistanceMetersEntry.getDouble(hubShotFarDistanceMeters));
    hubShotNearPreferredAngleDegrees =
        hubShotNearPreferredAngleDegreesEntry.getDouble(hubShotNearPreferredAngleDegrees);
    hubShotFarPreferredAngleDegrees =
        hubShotFarPreferredAngleDegreesEntry.getDouble(hubShotFarPreferredAngleDegrees);

    minLaunchAngleDegrees = minLaunchAngleDegreesEntry.getDouble(minLaunchAngleDegrees);
    maxLaunchAngleDegrees =
        Math.max(
            minLaunchAngleDegrees + MIN_LAUNCH_ANGLE_SPAN_DEGREES,
            maxLaunchAngleDegreesEntry.getDouble(maxLaunchAngleDegrees));
    launchAngleSearchStepDegrees =
        Math.max(
            0.01, launchAngleSearchStepDegreesEntry.getDouble(launchAngleSearchStepDegrees));
    defaultLaunchAngleDegrees =
        MathUtil.clamp(
            defaultLaunchAngleDegreesEntry.getDouble(defaultLaunchAngleDegrees),
            minLaunchAngleDegrees,
            maxLaunchAngleDegrees);
    defaultLaunchSpeedMetersPerSec =
        Math.max(0.0, defaultLaunchSpeedMetersPerSecEntry.getDouble(defaultLaunchSpeedMetersPerSec));

    minAirtimeSeconds = Math.max(0.0, minAirtimeSecondsEntry.getDouble(minAirtimeSeconds));
    maxAirtimeSeconds =
        Math.max(
            minAirtimeSeconds + MIN_NUMERIC_EPSILON,
            maxAirtimeSecondsEntry.getDouble(maxAirtimeSeconds));
    fallbackAirtimeSeconds =
        MathUtil.clamp(
            fallbackAirtimeSecondsEntry.getDouble(fallbackAirtimeSeconds),
            minAirtimeSeconds,
            maxAirtimeSeconds);
    minHorizontalVelocityMetersPerSec =
        Math.max(
            MIN_NUMERIC_EPSILON,
            minHorizontalVelocityMetersPerSecEntry.getDouble(minHorizontalVelocityMetersPerSec));
    simShotCadenceSeconds = Math.max(0.0, simShotCadenceSecondsEntry.getDouble(simShotCadenceSeconds));
    simWheelReadyRatio = MathUtil.clamp(simWheelReadyRatioEntry.getDouble(simWheelReadyRatio), 0.0, 1.0);

    minLaunchSpeedMetersPerSec = launchSlipFactor * shooterWheelRadiusMeters * minWheelSpeedRadPerSec;
    maxLaunchSpeedMetersPerSec = launchSlipFactor * shooterWheelRadiusMeters * maxWheelSpeedRadPerSec;
    if ((maxLaunchSpeedMetersPerSec - minLaunchSpeedMetersPerSec) < MIN_NUMERIC_EPSILON) {
      maxLaunchSpeedMetersPerSec = minLaunchSpeedMetersPerSec + MIN_NUMERIC_EPSILON;
    }
    defaultLaunchSpeedMetersPerSec =
        MathUtil.clamp(defaultLaunchSpeedMetersPerSec, minLaunchSpeedMetersPerSec, maxLaunchSpeedMetersPerSec);

    pair1WheelSetpointRadPerSec = clampManualWheelSetpointRadPerSec(pair1WheelSetpointRadPerSec);
    pair2WheelSetpointRadPerSec = clampManualWheelSetpointRadPerSec(pair2WheelSetpointRadPerSec);
    hoodSetpointMotorRotations = clampHoodSetpointRotations(hoodSetpointMotorRotations);

    publishSanitizedConfigToNetworkTables();
  }

  private void publishSanitizedConfigToNetworkTables() {
    shotControlEnabledEntry.setBoolean(shotControlEnabled);
    manualHoodOverrideEnabledEntry.setBoolean(manualHoodOverrideEnabled);
    manualSetpointsEnabledEntry.setBoolean(manualSetpointsEnabled);
    manualPair1SetpointEntry.setDouble(pair1WheelSetpointRadPerSec);
    manualPair2SetpointEntry.setDouble(pair2WheelSetpointRadPerSec);
    manualHoodSetpointRotationsEntry.setDouble(hoodSetpointMotorRotations);
    launchHeightMetersEntry.setDouble(launchHeightMeters);

    gravityMetersPerSecSquaredEntry.setDouble(gravityMetersPerSecSquared);
    shooterWheelRadiusMetersEntry.setDouble(shooterWheelRadiusMeters);
    fuelBallRadiusMetersEntry.setDouble(fuelBallRadiusMeters);
    launchSlipFactorEntry.setDouble(launchSlipFactor);
    targetBallSpinRatioEntry.setDouble(targetBallSpinRatio);
    minWheelSpeedRadPerSecEntry.setDouble(minWheelSpeedRadPerSec);
    maxWheelSpeedRadPerSecEntry.setDouble(maxWheelSpeedRadPerSec);
    hoodReferenceAngleDegreesEntry.setDouble(hoodReferenceAngleDegrees);
    hoodReferenceMotorRotationsEntry.setDouble(hoodReferenceMotorRotations);
    hoodDegreesPerMotorRotationEntry.setDouble(hoodDegreesPerMotorRotation);
    hubCenterHeightMetersEntry.setDouble(hubCenterHeightMeters);
    hubShotNearDistanceMetersEntry.setDouble(hubShotNearDistanceMeters);
    hubShotFarDistanceMetersEntry.setDouble(hubShotFarDistanceMeters);
    hubShotNearPreferredAngleDegreesEntry.setDouble(hubShotNearPreferredAngleDegrees);
    hubShotFarPreferredAngleDegreesEntry.setDouble(hubShotFarPreferredAngleDegrees);
    minLaunchAngleDegreesEntry.setDouble(minLaunchAngleDegrees);
    maxLaunchAngleDegreesEntry.setDouble(maxLaunchAngleDegrees);
    launchAngleSearchStepDegreesEntry.setDouble(launchAngleSearchStepDegrees);
    defaultLaunchSpeedMetersPerSecEntry.setDouble(defaultLaunchSpeedMetersPerSec);
    defaultLaunchAngleDegreesEntry.setDouble(defaultLaunchAngleDegrees);
    minAirtimeSecondsEntry.setDouble(minAirtimeSeconds);
    maxAirtimeSecondsEntry.setDouble(maxAirtimeSeconds);
    fallbackAirtimeSecondsEntry.setDouble(fallbackAirtimeSeconds);
    minHorizontalVelocityMetersPerSecEntry.setDouble(minHorizontalVelocityMetersPerSec);
    simShotCadenceSecondsEntry.setDouble(simShotCadenceSeconds);
    simWheelReadyRatioEntry.setDouble(simWheelReadyRatio);
  }

  private void applyManualSetpointsFromNetworkTables() {
    pair1WheelSetpointRadPerSec =
        clampManualWheelSetpointRadPerSec(
            manualPair1SetpointEntry.getDouble(pair1WheelSetpointRadPerSec));
    pair2WheelSetpointRadPerSec =
        clampManualWheelSetpointRadPerSec(
            manualPair2SetpointEntry.getDouble(pair2WheelSetpointRadPerSec));
    hoodSetpointMotorRotations =
        clampHoodSetpointRotations(
            manualHoodSetpointRotationsEntry.getDouble(hoodSetpointMotorRotations));
    manualPair1SetpointEntry.setDouble(pair1WheelSetpointRadPerSec);
    manualPair2SetpointEntry.setDouble(pair2WheelSetpointRadPerSec);
    manualHoodSetpointRotationsEntry.setDouble(hoodSetpointMotorRotations);
  }

  private void logControlState() {
    Logger.recordOutput("Shooter/Control/Enabled", shotControlEnabled);
    Logger.recordOutput("Shooter/Control/TriggerSpeedBoostEnabled", triggerSpeedBoostEnabled);
    Logger.recordOutput(
        "Shooter/Control/TriggerLaunchSpeedBoostScale",
        ShooterConstants.triggerLaunchSpeedBoostScale);
    Logger.recordOutput("Shooter/Control/ManualSetpointsEnabled", manualSetpointsEnabled);
    Logger.recordOutput("Shooter/Control/Pair1SetpointRadPerSec", pair1WheelSetpointRadPerSec);
    Logger.recordOutput("Shooter/Control/Pair2SetpointRadPerSec", pair2WheelSetpointRadPerSec);
    Logger.recordOutput(
        "Shooter/Control/HoodSetpointRotations", hoodSetpointMotorRotations);
    Logger.recordOutput("Shooter/Control/ManualHoodOverrideEnabled", manualHoodOverrideEnabled);
    Logger.recordOutput(
        "Shooter/Control/HoodSetpointDegrees",
        configuredMotorRotationsToHoodAngle(hoodSetpointMotorRotations).getDegrees());
    Logger.recordOutput(
        "Shooter/Measured/Pair1VelocityRadPerSec", inputs.pair1LeaderVelocityRadPerSec);
    Logger.recordOutput(
        "Shooter/Measured/Pair1FollowerVelocityRadPerSec", inputs.pair1FollowerVelocityRadPerSec);
    Logger.recordOutput(
        "Shooter/Measured/Pair2VelocityRadPerSec", inputs.pair2LeaderVelocityRadPerSec);
    Logger.recordOutput(
        "Shooter/Measured/Pair2FollowerVelocityRadPerSec", inputs.pair2FollowerVelocityRadPerSec);
    Logger.recordOutput("Shooter/Measured/Pair1AppliedVolts", inputs.pair1AppliedVolts);
    Logger.recordOutput("Shooter/Measured/Pair2AppliedVolts", inputs.pair2AppliedVolts);
    Logger.recordOutput("Shooter/Measured/Pair1CurrentAmps", inputs.pair1CurrentAmps);
    Logger.recordOutput("Shooter/Measured/Pair2CurrentAmps", inputs.pair2CurrentAmps);
    Logger.recordOutput("Shooter/Measured/HoodPositionRotations", inputs.hoodPositionRotations);
    Logger.recordOutput("Shooter/Measured/HoodVelocityRotationsPerSec", inputs.hoodVelocityRotationsPerSec);
    Logger.recordOutput("Shooter/Measured/HoodAppliedVolts", inputs.hoodAppliedVolts);
    Logger.recordOutput("Shooter/Measured/HoodCurrentAmps", inputs.hoodCurrentAmps);
    Logger.recordOutput(
        "Shooter/Measured/HoodAngleDegrees",
        configuredMotorRotationsToHoodAngle(inputs.hoodPositionRotations).getDegrees());
    Logger.recordOutput(
        "Shooter/Measured/EstimatedLaunchSpeedMetersPerSec",
        getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec());
    Logger.recordOutput("Shooter/Simulation/WheelsReady", areWheelsReadyForSimulatedShot());
    Logger.recordOutput("Shooter/Status/Pair1Connected", inputs.pair1Connected);
    Logger.recordOutput("Shooter/Status/Pair2Connected", inputs.pair2Connected);
    Logger.recordOutput("Shooter/Status/HoodConnected", inputs.hoodConnected);
  }

  private void logTuningState() {
    Logger.recordOutput("Shooter/Tuning/ControlEnabled", shotControlEnabled);
    Logger.recordOutput("Shooter/Tuning/ManualHoodOverrideEnabled", manualHoodOverrideEnabled);
    Logger.recordOutput("Shooter/Tuning/ManualSetpointsEnabled", manualSetpointsEnabled);
    Logger.recordOutput("Shooter/Tuning/LaunchHeightMeters", launchHeightMeters);
    Logger.recordOutput("Shooter/Tuning/GravityMetersPerSecSquared", gravityMetersPerSecSquared);
    Logger.recordOutput("Shooter/Tuning/ShooterWheelRadiusMeters", shooterWheelRadiusMeters);
    Logger.recordOutput("Shooter/Tuning/FuelBallRadiusMeters", fuelBallRadiusMeters);
    Logger.recordOutput("Shooter/Tuning/LaunchSlipFactor", launchSlipFactor);
    Logger.recordOutput("Shooter/Tuning/TargetBallSpinRatio", targetBallSpinRatio);
    Logger.recordOutput("Shooter/Tuning/MinWheelSpeedRadPerSec", minWheelSpeedRadPerSec);
    Logger.recordOutput("Shooter/Tuning/MaxWheelSpeedRadPerSec", maxWheelSpeedRadPerSec);
    Logger.recordOutput("Shooter/Tuning/MinLaunchSpeedMetersPerSec", minLaunchSpeedMetersPerSec);
    Logger.recordOutput("Shooter/Tuning/MaxLaunchSpeedMetersPerSec", maxLaunchSpeedMetersPerSec);
    Logger.recordOutput("Shooter/Tuning/HoodReferenceAngleDegrees", hoodReferenceAngleDegrees);
    Logger.recordOutput("Shooter/Tuning/HoodReferenceMotorRotations", hoodReferenceMotorRotations);
    Logger.recordOutput("Shooter/Tuning/HoodDegreesPerMotorRotation", hoodDegreesPerMotorRotation);
    Logger.recordOutput("Shooter/Tuning/HubCenterHeightMeters", hubCenterHeightMeters);
    Logger.recordOutput("Shooter/Tuning/HubShotNearDistanceMeters", hubShotNearDistanceMeters);
    Logger.recordOutput("Shooter/Tuning/HubShotFarDistanceMeters", hubShotFarDistanceMeters);
    Logger.recordOutput(
        "Shooter/Tuning/HubShotNearPreferredAngleDegrees", hubShotNearPreferredAngleDegrees);
    Logger.recordOutput(
        "Shooter/Tuning/HubShotFarPreferredAngleDegrees", hubShotFarPreferredAngleDegrees);
    Logger.recordOutput("Shooter/Tuning/MinLaunchAngleDegrees", minLaunchAngleDegrees);
    Logger.recordOutput("Shooter/Tuning/MaxLaunchAngleDegrees", maxLaunchAngleDegrees);
    Logger.recordOutput("Shooter/Tuning/LaunchAngleSearchStepDegrees", launchAngleSearchStepDegrees);
    Logger.recordOutput("Shooter/Tuning/DefaultLaunchSpeedMetersPerSec", defaultLaunchSpeedMetersPerSec);
    Logger.recordOutput("Shooter/Tuning/DefaultLaunchAngleDegrees", defaultLaunchAngleDegrees);
    Logger.recordOutput("Shooter/Tuning/MinAirtimeSeconds", minAirtimeSeconds);
    Logger.recordOutput("Shooter/Tuning/MaxAirtimeSeconds", maxAirtimeSeconds);
    Logger.recordOutput("Shooter/Tuning/FallbackAirtimeSeconds", fallbackAirtimeSeconds);
    Logger.recordOutput(
        "Shooter/Tuning/MinHorizontalVelocityMetersPerSec", minHorizontalVelocityMetersPerSec);
    Logger.recordOutput("Shooter/Tuning/SimShotCadenceSeconds", simShotCadenceSeconds);
    Logger.recordOutput("Shooter/Tuning/SimWheelReadyRatio", simWheelReadyRatio);
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
