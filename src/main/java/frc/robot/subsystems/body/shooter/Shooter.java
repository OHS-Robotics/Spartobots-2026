package frc.robot.subsystems.body.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static record WheelSpeedSetpoints(double pair1RadPerSec, double pair2RadPerSec) {}

  private static record MotionCompensatedHubShotSolution(
      HubShotSolution shotSolution, Pose2d compensatedHubPose, int iterations, boolean converged) {}

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
  private double wheelSpeedScale = ShooterConstants.defaultWheelSpeedScale;
  private double pair1Direction = ShooterConstants.defaultPair1Direction;
  private double pair2Direction = ShooterConstants.defaultPair2Direction;
  private double hoodSetpointMotorRotations = 0.0;
  private double hoodRetractedPositionRotations =
      ShooterConstants.defaultHoodRetractedPositionRotations;
  private double hoodExtendedPositionRotations =
      ShooterConstants.defaultHoodExtendedPositionRotations;
  private double hubMotionCompVelocityScale = ShooterConstants.hubMotionCompensationVelocityScale;
  private double hubMotionCompLeadSeconds = ShooterConstants.hubMotionCompensationLeadSeconds;
  private double lastSimShotTimestampSeconds = Double.NEGATIVE_INFINITY;
  private HubShotSolution latestHubShotSolution =
      new HubShotSolution(
          0.0,
          ShooterConstants.defaultLaunchAngle,
          ShooterConstants.defaultLaunchSpeedMetersPerSec,
          speedToPower(ShooterConstants.defaultLaunchSpeedMetersPerSec),
          ShooterConstants.fallbackAirtimeSeconds,
          false);
  private final NetworkTable subsystemTable =
      NetworkTableInstance.getDefault().getTable(ShooterConstants.configTableName);
  private final NetworkTable tuningTable = subsystemTable.getSubTable("Tuning");
  private final NetworkTable motionCompTuningTable =
      tuningTable.getSubTable("MotionCompensation").getSubTable(Constants.currentMode.name());
  private final NetworkTable telemetryTable = subsystemTable.getSubTable("Telemetry");
  private final NetworkTableEntry hoodRetractedPositionEntry =
      tuningTable.getEntry("Hood/Calibration/RetractedPositionRotations");
  private final NetworkTableEntry hoodExtendedPositionEntry =
      tuningTable.getEntry("Hood/Calibration/ExtendedPositionRotations");
  private final NetworkTableEntry hoodEncoderPositionEntry =
      telemetryTable.getEntry("Hood/EncoderPositionRotations");
  private final NetworkTableEntry hoodEncoderVelocityEntry =
      telemetryTable.getEntry("Hood/EncoderVelocityRotationsPerSec");
  private final NetworkTableEntry hoodEncoderNormalizedPositionEntry =
      telemetryTable.getEntry("Hood/EncoderPositionNormalized");
  private final NetworkTableEntry hoodSetpointEntry =
      telemetryTable.getEntry("Hood/SetpointRotations");
  private final NetworkTableEntry hoodSetpointAngleEntry =
      telemetryTable.getEntry("Hood/SetpointAngleFromFloorDegrees");
  private final NetworkTableEntry hoodMeasuredAngleEntry =
      telemetryTable.getEntry("Hood/MeasuredAngleFromFloorDegrees");
  private final NetworkTableEntry hoodSetpointErrorEntry =
      telemetryTable.getEntry("Hood/SetpointErrorDegrees");
  private final NetworkTableEntry hoodMinAngleFromFloorEntry =
      telemetryTable.getEntry("Hood/Calibration/MinAngleFromFloorDegrees");
  private final NetworkTableEntry hoodMaxAngleFromFloorEntry =
      telemetryTable.getEntry("Hood/Calibration/MaxAngleFromFloorDegrees");
  private final NetworkTableEntry pair1MeasuredVelocityEntry =
      telemetryTable.getEntry("Wheels/Pair1VelocityRadPerSec");
  private final NetworkTableEntry pair2MeasuredVelocityEntry =
      telemetryTable.getEntry("Wheels/Pair2VelocityRadPerSec");
  private final NetworkTableEntry pair1CommandVelocityEntry =
      telemetryTable.getEntry("Wheels/Pair1CommandRadPerSec");
  private final NetworkTableEntry pair2CommandVelocityEntry =
      telemetryTable.getEntry("Wheels/Pair2CommandRadPerSec");
  private final NetworkTableEntry shotOverlayTargetDistanceEntry =
      telemetryTable.getEntry("Overlay/TargetDistanceMeters");
  private final NetworkTableEntry shotOverlayHubHeightEntry =
      telemetryTable.getEntry("Overlay/HubCenterHeightMeters");
  private final NetworkTableEntry shotOverlayLaunchHeightEntry =
      telemetryTable.getEntry("Overlay/LaunchHeightMeters");
  private final NetworkTableEntry shotOverlaySolutionAngleEntry =
      telemetryTable.getEntry("Overlay/SolutionAngleFromFloorDegrees");
  private final NetworkTableEntry shotOverlaySetpointLaunchSpeedEntry =
      telemetryTable.getEntry("Overlay/Setpoint/LaunchSpeedMetersPerSec");
  private final NetworkTableEntry shotOverlayMeasuredLaunchSpeedEntry =
      telemetryTable.getEntry("Overlay/Measured/LaunchSpeedMetersPerSec");
  private final NetworkTableEntry shotOverlaySetpointHeightEntry =
      telemetryTable.getEntry("Overlay/Setpoint/HeightAtTargetMeters");
  private final NetworkTableEntry shotOverlayMeasuredHeightEntry =
      telemetryTable.getEntry("Overlay/Measured/HeightAtTargetMeters");
  private final NetworkTableEntry shotOverlaySetpointClearanceEntry =
      telemetryTable.getEntry("Overlay/Setpoint/ClearanceAtTargetMeters");
  private final NetworkTableEntry shotOverlayMeasuredClearanceEntry =
      telemetryTable.getEntry("Overlay/Measured/ClearanceAtTargetMeters");
  private final NetworkTableEntry shotOverlayMinAngleClearanceEntry =
      telemetryTable.getEntry("Overlay/Calibration/MinAngleClearanceAtTargetMeters");
  private final NetworkTableEntry shotOverlayMaxAngleClearanceEntry =
      telemetryTable.getEntry("Overlay/Calibration/MaxAngleClearanceAtTargetMeters");
  private final NetworkTableEntry wheelSpeedScaleEntry = tuningTable.getEntry("Wheels/SpeedScale");
  private final NetworkTableEntry pair1DirectionEntry =
      tuningTable.getEntry("Wheels/Pair1Direction");
  private final NetworkTableEntry pair2DirectionEntry =
      tuningTable.getEntry("Wheels/Pair2Direction");
  private final NetworkTableEntry hubMotionCompVelocityScaleEntry =
      motionCompTuningTable.getEntry("VelocityScale");
  private final NetworkTableEntry hubMotionCompLeadSecondsEntry =
      motionCompTuningTable.getEntry("LeadSeconds");

  public Shooter() {
    this(new ShooterIO() {});
  }

  public Shooter(ShooterIO io) {
    this.io = io;
    configureNetworkTableDefaults();
    loadNetworkTableConfig();
    hoodSetpointMotorRotations = hoodAngleToMotorRotations(ShooterConstants.defaultLaunchAngle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    loadNetworkTableConfig();
    clampHoodSetpointToCalibrationRange();
    double pair1VelocityCommandRadPerSec =
        shotControlEnabled ? getPair1VelocityCommandSetpointRadPerSec() : 0.0;
    double pair2VelocityCommandRadPerSec =
        shotControlEnabled ? getPair2VelocityCommandSetpointRadPerSec() : 0.0;

    io.setWheelVelocitySetpoints(pair1VelocityCommandRadPerSec, pair2VelocityCommandRadPerSec);
    io.setHoodPositionSetpointRotations(hoodSetpointMotorRotations);
    publishHoodEncoderToNetworkTables();
    publishWheelTelemetryToNetworkTables(
        pair1VelocityCommandRadPerSec, pair2VelocityCommandRadPerSec);
    publishHoodShotOverlayToNetworkTables(
        pair1VelocityCommandRadPerSec, pair2VelocityCommandRadPerSec);

    logControlState(pair1VelocityCommandRadPerSec, pair2VelocityCommandRadPerSec);
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
    manualHoodOverrideEnabled = true;
    double targetAngleDegrees =
        MathUtil.clamp(
            motorRotationsToHoodAngle(hoodSetpointMotorRotations).getDegrees() + deltaDegrees,
            ShooterConstants.minLaunchAngle.getDegrees(),
            ShooterConstants.maxLaunchAngle.getDegrees());
    hoodSetpointMotorRotations =
        hoodAngleToMotorRotations(Rotation2d.fromDegrees(targetAngleDegrees));
  }

  public void adjustHoodSetpointRotations(double deltaRotations) {
    manualHoodOverrideEnabled = true;
    hoodSetpointMotorRotations =
        clampToHoodCalibrationRange(hoodSetpointMotorRotations + deltaRotations);
  }

  public HubShotSolution updateHubShotSolution(Pose2d robotPose, Pose2d hubPose) {
    return updateHubShotSolution(robotPose, hubPose, new Translation2d());
  }

  public HubShotSolution updateHubShotSolution(
      Pose2d robotPose, Pose2d hubPose, Translation2d robotFieldVelocityMetersPerSec) {
    Translation2d robotFieldVelocity =
        robotFieldVelocityMetersPerSec != null
            ? robotFieldVelocityMetersPerSec
            : new Translation2d();
    double targetHeightDeltaMeters = ShooterConstants.hubCenterHeightMeters - launchHeightMeters;
    MotionCompensatedHubShotSolution motionCompensatedSolution =
        solveHubShotWithMotionCompensation(
            robotPose, hubPose, robotFieldVelocity, targetHeightDeltaMeters);
    latestHubShotSolution = motionCompensatedSolution.shotSolution();
    applyHubShotSetpoints(latestHubShotSolution);
    logHubShotSolution(
        latestHubShotSolution,
        hubPose,
        motionCompensatedSolution.compensatedHubPose(),
        robotFieldVelocity,
        motionCompensatedSolution.iterations(),
        motionCompensatedSolution.converged());
    return latestHubShotSolution;
  }

  public double estimateHubShotAirtimeSeconds(Pose2d robotPose, Pose2d hubPose) {
    return estimateHubShotAirtimeSeconds(robotPose, hubPose, new Translation2d());
  }

  public double estimateHubShotAirtimeSeconds(
      Pose2d robotPose, Pose2d hubPose, Translation2d robotFieldVelocityMetersPerSec) {
    return updateHubShotSolution(robotPose, hubPose, robotFieldVelocityMetersPerSec)
        .airtimeSeconds();
  }

  private void configureNetworkTableDefaults() {
    hoodRetractedPositionEntry.setDefaultDouble(
        ShooterConstants.defaultHoodRetractedPositionRotations);
    hoodExtendedPositionEntry.setDefaultDouble(
        ShooterConstants.defaultHoodExtendedPositionRotations);
    wheelSpeedScaleEntry.setDefaultDouble(ShooterConstants.defaultWheelSpeedScale);
    pair1DirectionEntry.setDefaultDouble(ShooterConstants.defaultPair1Direction);
    pair2DirectionEntry.setDefaultDouble(ShooterConstants.defaultPair2Direction);
    hubMotionCompVelocityScaleEntry.setDefaultDouble(
        ShooterConstants.hubMotionCompensationVelocityScale);
    hubMotionCompLeadSecondsEntry.setDefaultDouble(
        ShooterConstants.hubMotionCompensationLeadSeconds);
    hoodMinAngleFromFloorEntry.setDefaultDouble(
        ShooterConstants.minHoodAngleFromFloor.getDegrees());
    hoodMaxAngleFromFloorEntry.setDefaultDouble(
        ShooterConstants.maxHoodAngleFromFloor.getDegrees());
  }

  private void loadNetworkTableConfig() {
    hoodRetractedPositionRotations =
        hoodRetractedPositionEntry.getDouble(hoodRetractedPositionRotations);
    hoodExtendedPositionRotations =
        hoodExtendedPositionEntry.getDouble(hoodExtendedPositionRotations);
    hoodRetractedPositionEntry.setDouble(hoodRetractedPositionRotations);
    hoodExtendedPositionEntry.setDouble(hoodExtendedPositionRotations);
    wheelSpeedScale = clampWheelSpeedScale(wheelSpeedScaleEntry.getDouble(wheelSpeedScale));
    pair1Direction = normalizeDirection(pair1DirectionEntry.getDouble(pair1Direction));
    pair2Direction = normalizeDirection(pair2DirectionEntry.getDouble(pair2Direction));
    hubMotionCompVelocityScale =
        clampMotionCompVelocityScale(
            hubMotionCompVelocityScaleEntry.getDouble(hubMotionCompVelocityScale));
    hubMotionCompLeadSeconds =
        clampMotionCompLeadSeconds(
            hubMotionCompLeadSecondsEntry.getDouble(hubMotionCompLeadSeconds));
    wheelSpeedScaleEntry.setDouble(wheelSpeedScale);
    pair1DirectionEntry.setDouble(pair1Direction);
    pair2DirectionEntry.setDouble(pair2Direction);
    hubMotionCompVelocityScaleEntry.setDouble(hubMotionCompVelocityScale);
    hubMotionCompLeadSecondsEntry.setDouble(hubMotionCompLeadSeconds);
  }

  private void publishHoodEncoderToNetworkTables() {
    Rotation2d setpointAngle = motorRotationsToHoodAngle(hoodSetpointMotorRotations);
    Rotation2d measuredAngle = motorRotationsToHoodAngle(inputs.hoodPositionRotations);
    hoodEncoderPositionEntry.setDouble(inputs.hoodPositionRotations);
    hoodEncoderVelocityEntry.setDouble(inputs.hoodVelocityRotationsPerSec);
    hoodEncoderNormalizedPositionEntry.setDouble(
        getHoodNormalizedPosition(inputs.hoodPositionRotations));
    hoodSetpointEntry.setDouble(hoodSetpointMotorRotations);
    hoodSetpointAngleEntry.setDouble(setpointAngle.getDegrees());
    hoodMeasuredAngleEntry.setDouble(measuredAngle.getDegrees());
    hoodSetpointErrorEntry.setDouble(setpointAngle.minus(measuredAngle).getDegrees());
    hoodMinAngleFromFloorEntry.setDouble(ShooterConstants.minHoodAngleFromFloor.getDegrees());
    hoodMaxAngleFromFloorEntry.setDouble(ShooterConstants.maxHoodAngleFromFloor.getDegrees());
  }

  private void publishWheelTelemetryToNetworkTables(
      double pair1VelocityCommandRadPerSec, double pair2VelocityCommandRadPerSec) {
    pair1MeasuredVelocityEntry.setDouble(inputs.pair1LeaderVelocityRadPerSec);
    pair2MeasuredVelocityEntry.setDouble(inputs.pair2LeaderVelocityRadPerSec);
    pair1CommandVelocityEntry.setDouble(pair1VelocityCommandRadPerSec);
    pair2CommandVelocityEntry.setDouble(pair2VelocityCommandRadPerSec);
  }

  private void publishHoodShotOverlayToNetworkTables(
      double pair1VelocityCommandRadPerSec, double pair2VelocityCommandRadPerSec) {
    Rotation2d setpointHoodAngle = motorRotationsToHoodAngle(hoodSetpointMotorRotations);
    Rotation2d measuredHoodAngle = motorRotationsToHoodAngle(inputs.hoodPositionRotations);

    double setpointLaunchSpeedMetersPerSec =
        estimateLaunchSpeedFromWheelVelocitiesMetersPerSec(
            pair1VelocityCommandRadPerSec, pair2VelocityCommandRadPerSec);
    double measuredLaunchSpeedMetersPerSec =
        getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec();

    double targetDistanceMeters = Math.max(0.0, latestHubShotSolution.distanceMeters());
    double hubCenterHeightMeters = ShooterConstants.hubCenterHeightMeters;

    OptionalDouble setpointHeightAtTargetMeters =
        calculateProjectileHeightAtDistance(
            targetDistanceMeters, setpointLaunchSpeedMetersPerSec, setpointHoodAngle);
    OptionalDouble measuredHeightAtTargetMeters =
        calculateProjectileHeightAtDistance(
            targetDistanceMeters, measuredLaunchSpeedMetersPerSec, measuredHoodAngle);
    OptionalDouble minAngleHeightAtTargetMeters =
        calculateProjectileHeightAtDistance(
            targetDistanceMeters,
            setpointLaunchSpeedMetersPerSec,
            ShooterConstants.minHoodAngleFromFloor);
    OptionalDouble maxAngleHeightAtTargetMeters =
        calculateProjectileHeightAtDistance(
            targetDistanceMeters,
            setpointLaunchSpeedMetersPerSec,
            ShooterConstants.maxHoodAngleFromFloor);

    shotOverlayTargetDistanceEntry.setDouble(targetDistanceMeters);
    shotOverlayHubHeightEntry.setDouble(hubCenterHeightMeters);
    shotOverlayLaunchHeightEntry.setDouble(launchHeightMeters);
    shotOverlaySolutionAngleEntry.setDouble(latestHubShotSolution.launchAngle().getDegrees());
    shotOverlaySetpointLaunchSpeedEntry.setDouble(setpointLaunchSpeedMetersPerSec);
    shotOverlayMeasuredLaunchSpeedEntry.setDouble(measuredLaunchSpeedMetersPerSec);
    shotOverlaySetpointHeightEntry.setDouble(optionalDoubleOrNaN(setpointHeightAtTargetMeters));
    shotOverlayMeasuredHeightEntry.setDouble(optionalDoubleOrNaN(measuredHeightAtTargetMeters));
    shotOverlaySetpointClearanceEntry.setDouble(
        optionalClearanceToHubOrNaN(setpointHeightAtTargetMeters, hubCenterHeightMeters));
    shotOverlayMeasuredClearanceEntry.setDouble(
        optionalClearanceToHubOrNaN(measuredHeightAtTargetMeters, hubCenterHeightMeters));
    shotOverlayMinAngleClearanceEntry.setDouble(
        optionalClearanceToHubOrNaN(minAngleHeightAtTargetMeters, hubCenterHeightMeters));
    shotOverlayMaxAngleClearanceEntry.setDouble(
        optionalClearanceToHubOrNaN(maxAngleHeightAtTargetMeters, hubCenterHeightMeters));
  }

  private OptionalDouble calculateProjectileHeightAtDistance(
      double horizontalDistanceMeters, double launchSpeedMetersPerSec, Rotation2d launchAngle) {
    double launchSpeedMagnitude = Math.abs(launchSpeedMetersPerSec);
    double cosTheta = Math.cos(launchAngle.getRadians());
    if (horizontalDistanceMeters < 0.0
        || launchSpeedMagnitude < 1e-6
        || Math.abs(cosTheta) < 1e-6) {
      return OptionalDouble.empty();
    }

    double timeSeconds = horizontalDistanceMeters / (launchSpeedMagnitude * cosTheta);
    if (!Double.isFinite(timeSeconds) || timeSeconds < 0.0) {
      return OptionalDouble.empty();
    }

    double sinTheta = Math.sin(launchAngle.getRadians());
    double projectileHeightMeters =
        launchHeightMeters
            + (launchSpeedMagnitude * sinTheta * timeSeconds)
            - (0.5 * ShooterConstants.gravityMetersPerSecSquared * timeSeconds * timeSeconds);
    if (!Double.isFinite(projectileHeightMeters)) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(projectileHeightMeters);
  }

  private void clampHoodSetpointToCalibrationRange() {
    hoodSetpointMotorRotations = clampToHoodCalibrationRange(hoodSetpointMotorRotations);
  }

  private MotionCompensatedHubShotSolution solveHubShotWithMotionCompensation(
      Pose2d robotPose,
      Pose2d hubPose,
      Translation2d robotFieldVelocityMetersPerSec,
      double targetHeightDeltaMeters) {
    double airtimeGuessSeconds = sanitizeAirtimeSeed(latestHubShotSolution.airtimeSeconds());
    Pose2d compensatedHubPose = hubPose;
    HubShotSolution solvedShot =
        solveHubShot(
            robotPose.getTranslation().getDistance(hubPose.getTranslation()),
            targetHeightDeltaMeters);
    int iterations = 0;
    boolean converged = false;

    for (int i = 0; i < ShooterConstants.hubMotionCompensationMaxIterations; i++) {
      compensatedHubPose =
          getCompensatedHubPose(hubPose, robotFieldVelocityMetersPerSec, airtimeGuessSeconds);
      double compensatedDistanceMeters =
          robotPose.getTranslation().getDistance(compensatedHubPose.getTranslation());
      solvedShot = solveHubShot(compensatedDistanceMeters, targetHeightDeltaMeters);
      iterations = i + 1;

      double airtimeErrorSeconds = Math.abs(solvedShot.airtimeSeconds() - airtimeGuessSeconds);
      if (airtimeErrorSeconds <= ShooterConstants.hubMotionCompensationAirtimeToleranceSeconds) {
        converged = true;
        break;
      }

      airtimeGuessSeconds = solvedShot.airtimeSeconds();
    }

    if (!converged && iterations > 0) {
      compensatedHubPose =
          getCompensatedHubPose(hubPose, robotFieldVelocityMetersPerSec, airtimeGuessSeconds);
      double compensatedDistanceMeters =
          robotPose.getTranslation().getDistance(compensatedHubPose.getTranslation());
      solvedShot = solveHubShot(compensatedDistanceMeters, targetHeightDeltaMeters);
    }

    return new MotionCompensatedHubShotSolution(
        solvedShot, compensatedHubPose, iterations, converged);
  }

  private Pose2d getCompensatedHubPose(
      Pose2d hubPose, Translation2d robotFieldVelocityMetersPerSec, double shotAirtimeSeconds) {
    double clampedAirtimeSeconds = Math.max(0.0, shotAirtimeSeconds + hubMotionCompLeadSeconds);
    Translation2d compensationOffset =
        robotFieldVelocityMetersPerSec.times(-clampedAirtimeSeconds * hubMotionCompVelocityScale);
    return new Pose2d(hubPose.getTranslation().plus(compensationOffset), hubPose.getRotation());
  }

  private HubShotSolution solveHubShot(
      double horizontalDistanceMeters, double targetHeightDeltaMeters) {
    Rotation2d preferredAngle = getPreferredAngle(horizontalDistanceMeters);

    HubShotSolution bestDescendingTopEntrySolution = null;
    double bestDescendingTopEntryLaunchSpeedMetersPerSec = Double.POSITIVE_INFINITY;
    double bestDescendingTopEntryPreferredScore = Double.POSITIVE_INFINITY;
    HubShotSolution bestPreferredSolution = null;
    double bestPreferredScore = Double.POSITIVE_INFINITY;
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

      HubShotSolution candidateSolution =
          createShotSolution(
              horizontalDistanceMeters,
              targetHeightDeltaMeters,
              candidateAngle,
              launchSpeedMetersPerSec,
              true);
      boolean candidateDescendingAtTarget =
          calculateVerticalVelocityAtDistance(
                      horizontalDistanceMeters, launchSpeedMetersPerSec, candidateAngle)
                  .orElse(Double.POSITIVE_INFINITY)
              <= -ShooterConstants.hubTopEntryMinDescentVelocityMetersPerSec;
      double score = Math.abs(candidateAngle.minus(preferredAngle).getDegrees());
      if (candidateDescendingAtTarget
          && ((launchSpeedMetersPerSec < (bestDescendingTopEntryLaunchSpeedMetersPerSec - 1e-6))
              || (Math.abs(launchSpeedMetersPerSec - bestDescendingTopEntryLaunchSpeedMetersPerSec)
                      <= 1e-6
                  && score < bestDescendingTopEntryPreferredScore))) {
        bestDescendingTopEntryLaunchSpeedMetersPerSec = launchSpeedMetersPerSec;
        bestDescendingTopEntryPreferredScore = score;
        bestDescendingTopEntrySolution = candidateSolution;
      }

      if (score < bestPreferredScore) {
        bestPreferredScore = score;
        bestPreferredSolution = candidateSolution;
      }
    }

    if (bestDescendingTopEntrySolution != null) {
      return bestDescendingTopEntrySolution;
    }

    if (bestPreferredSolution != null) {
      return bestPreferredSolution;
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

  private OptionalDouble calculateVerticalVelocityAtDistance(
      double horizontalDistanceMeters, double launchSpeedMetersPerSec, Rotation2d launchAngle) {
    double launchSpeedMagnitude = Math.abs(launchSpeedMetersPerSec);
    double cosTheta = Math.cos(launchAngle.getRadians());
    if (horizontalDistanceMeters < 0.0
        || launchSpeedMagnitude < 1e-6
        || Math.abs(cosTheta) < 1e-6) {
      return OptionalDouble.empty();
    }

    double timeSeconds = horizontalDistanceMeters / (launchSpeedMagnitude * cosTheta);
    if (!Double.isFinite(timeSeconds) || timeSeconds < 0.0) {
      return OptionalDouble.empty();
    }

    double initialVerticalVelocityMetersPerSec =
        launchSpeedMagnitude * Math.sin(launchAngle.getRadians());
    double verticalVelocityMetersPerSec =
        initialVerticalVelocityMetersPerSec
            - (ShooterConstants.gravityMetersPerSecSquared * timeSeconds);
    if (!Double.isFinite(verticalVelocityMetersPerSec)) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(verticalVelocityMetersPerSec);
  }

  private double calculateApexHeightMeters(double launchSpeedMetersPerSec, Rotation2d launchAngle) {
    double launchSpeedMagnitude = Math.abs(launchSpeedMetersPerSec);
    if (launchSpeedMagnitude < 1e-6) {
      return launchHeightMeters;
    }
    double initialVerticalVelocityMetersPerSec =
        launchSpeedMagnitude * Math.sin(launchAngle.getRadians());
    double apexHeightMeters =
        launchHeightMeters
            + ((initialVerticalVelocityMetersPerSec * initialVerticalVelocityMetersPerSec)
                / (2.0 * ShooterConstants.gravityMetersPerSecSquared));
    return Double.isFinite(apexHeightMeters) ? apexHeightMeters : launchHeightMeters;
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

    double airtimeByHorizontal = horizontalDistanceMeters / Math.abs(vx);
    if (!Double.isFinite(airtimeByHorizontal) || airtimeByHorizontal <= 0.0) {
      return ShooterConstants.fallbackAirtimeSeconds;
    }

    double vy = launchSpeedMetersPerSec * Math.sin(launchAngle.getRadians());
    double airtimeByVertical =
        solveVerticalAirtimeNear(vy, targetHeightDeltaMeters, airtimeByHorizontal)
            .orElse(airtimeByHorizontal);

    return clampAirtime(airtimeByVertical);
  }

  private OptionalDouble solveVerticalAirtimeNear(
      double vy, double targetHeightDeltaMeters, double preferredTimeSeconds) {
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

    boolean rootAValid = rootA > 0.0 && Double.isFinite(rootA);
    boolean rootBValid = rootB > 0.0 && Double.isFinite(rootB);
    if (rootAValid && rootBValid) {
      double rootAError = Math.abs(rootA - preferredTimeSeconds);
      double rootBError = Math.abs(rootB - preferredTimeSeconds);
      return OptionalDouble.of(rootAError <= rootBError ? rootA : rootB);
    }
    if (rootAValid) {
      return OptionalDouble.of(rootA);
    }
    if (rootBValid) {
      return OptionalDouble.of(rootB);
    }
    return OptionalDouble.empty();
  }

  private static double speedToPower(double launchSpeedMetersPerSec) {
    return MathUtil.clamp(
        launchSpeedMetersPerSec / ShooterConstants.maxLaunchSpeedMetersPerSec, 0.0, 1.0);
  }

  private static double sanitizeAirtimeSeed(double airtimeSeconds) {
    if (!Double.isFinite(airtimeSeconds) || airtimeSeconds <= 0.0) {
      return ShooterConstants.fallbackAirtimeSeconds;
    }
    return clampAirtime(airtimeSeconds);
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
    double spinSurfaceDeltaMetersPerSec = ballSpinRadPerSec * ShooterConstants.fuelBallRadiusMeters;

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

  private static double clampWheelSpeedScale(double speedScale) {
    return MathUtil.clamp(speedScale, 0.0, 1.5);
  }

  private static double clampMotionCompVelocityScale(double velocityScale) {
    return MathUtil.clamp(velocityScale, 0.0, 3.0);
  }

  private static double clampMotionCompLeadSeconds(double leadSeconds) {
    return MathUtil.clamp(leadSeconds, -0.25, 0.25);
  }

  private static double normalizeDirection(double direction) {
    return direction < 0.0 ? -1.0 : 1.0;
  }

  public double hoodAngleToMotorRotations(Rotation2d hoodAngle) {
    double normalizedAngle =
        clampUnitInterval(
            inverseInterpolate(
                hoodAngle.getDegrees(),
                ShooterConstants.minLaunchAngle.getDegrees(),
                ShooterConstants.maxLaunchAngle.getDegrees()));
    return interpolate(
        hoodRetractedPositionRotations, hoodExtendedPositionRotations, normalizedAngle);
  }

  public Rotation2d motorRotationsToHoodAngle(double hoodMotorRotations) {
    double normalizedHoodPosition =
        clampUnitInterval(
            inverseInterpolate(
                hoodMotorRotations, hoodRetractedPositionRotations, hoodExtendedPositionRotations));
    double hoodAngleDegrees =
        interpolate(
            ShooterConstants.minLaunchAngle.getDegrees(),
            ShooterConstants.maxLaunchAngle.getDegrees(),
            normalizedHoodPosition);
    return Rotation2d.fromDegrees(hoodAngleDegrees);
  }

  public Rotation2d getMeasuredHoodAngle() {
    return motorRotationsToHoodAngle(inputs.hoodPositionRotations);
  }

  public double getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec() {
    return estimateLaunchSpeedFromWheelVelocitiesMetersPerSec(
        inputs.pair1LeaderVelocityRadPerSec, inputs.pair2LeaderVelocityRadPerSec);
  }

  public boolean areWheelsAtSpeedForShot() {
    return areWheelsAtSpeed(ShooterConstants.wheelReadyToleranceRatio);
  }

  public boolean isReadyToFire() {
    return shotControlEnabled
        && latestHubShotSolution.feasible()
        && inputs.pair1Connected
        && inputs.pair2Connected
        && inputs.hoodConnected
        && areWheelsAtSpeedForShot();
  }

  public boolean shouldTriggerSimulatedShot(double timestampSeconds, double feedRateRatio) {
    if (!areWheelsReadyForSimulatedShot()) {
      return false;
    }

    double clampedFeedRateRatio = MathUtil.clamp(Math.abs(feedRateRatio), 0.0, 1.0);
    if (clampedFeedRateRatio < 1e-3) {
      return false;
    }

    double shotCadenceSeconds = ShooterConstants.simShotCadenceSeconds / clampedFeedRateRatio;
    if ((timestampSeconds - lastSimShotTimestampSeconds) < shotCadenceSeconds) {
      return false;
    }

    lastSimShotTimestampSeconds = timestampSeconds;
    return true;
  }

  private boolean areWheelsReadyForSimulatedShot() {
    return shotControlEnabled && areWheelsAtSpeed(1.0 - ShooterConstants.simWheelReadyRatio);
  }

  private boolean areWheelsAtSpeed(double toleranceRatio) {
    double clampedToleranceRatio = MathUtil.clamp(toleranceRatio, 0.0, 1.0);
    double pair1SetpointMagnitude = Math.abs(getPair1VelocityCommandSetpointRadPerSec());
    double pair2SetpointMagnitude = Math.abs(getPair2VelocityCommandSetpointRadPerSec());
    if (pair1SetpointMagnitude < ShooterConstants.minWheelSetpointForReadinessRadPerSec
        || pair2SetpointMagnitude < ShooterConstants.minWheelSetpointForReadinessRadPerSec) {
      return false;
    }

    double pair1MeasuredMagnitude = Math.abs(inputs.pair1LeaderVelocityRadPerSec);
    double pair2MeasuredMagnitude = Math.abs(inputs.pair2LeaderVelocityRadPerSec);
    double pair1LowerBound = pair1SetpointMagnitude * (1.0 - clampedToleranceRatio);
    double pair1UpperBound = pair1SetpointMagnitude * (1.0 + clampedToleranceRatio);
    double pair2LowerBound = pair2SetpointMagnitude * (1.0 - clampedToleranceRatio);
    double pair2UpperBound = pair2SetpointMagnitude * (1.0 + clampedToleranceRatio);

    return pair1MeasuredMagnitude >= pair1LowerBound
        && pair1MeasuredMagnitude <= pair1UpperBound
        && pair2MeasuredMagnitude >= pair2LowerBound
        && pair2MeasuredMagnitude <= pair2UpperBound;
  }

  private double estimateLaunchSpeedFromWheelSetpointsMetersPerSec() {
    return estimateLaunchSpeedFromWheelVelocitiesMetersPerSec(
        getPair1VelocityCommandSetpointRadPerSec(), getPair2VelocityCommandSetpointRadPerSec());
  }

  private static double estimateLaunchSpeedFromWheelVelocitiesMetersPerSec(
      double pair1WheelVelocityRadPerSec, double pair2WheelVelocityRadPerSec) {
    double pair1SurfaceSpeedMetersPerSec =
        Math.abs(pair1WheelVelocityRadPerSec) * ShooterConstants.shooterWheelRadiusMeters;
    double pair2SurfaceSpeedMetersPerSec =
        Math.abs(pair2WheelVelocityRadPerSec) * ShooterConstants.shooterWheelRadiusMeters;
    double averageWheelSurfaceSpeedMetersPerSec =
        0.5 * (pair1SurfaceSpeedMetersPerSec + pair2SurfaceSpeedMetersPerSec);
    return ShooterConstants.launchSlipFactor * averageWheelSurfaceSpeedMetersPerSec;
  }

  private double getPair1VelocityCommandSetpointRadPerSec() {
    return pair1WheelSetpointRadPerSec * wheelSpeedScale * pair1Direction;
  }

  private double getPair2VelocityCommandSetpointRadPerSec() {
    return pair2WheelSetpointRadPerSec * wheelSpeedScale * pair2Direction;
  }

  private double clampToHoodCalibrationRange(double hoodPositionRotations) {
    return MathUtil.clamp(
        hoodPositionRotations,
        Math.min(hoodRetractedPositionRotations, hoodExtendedPositionRotations),
        Math.max(hoodRetractedPositionRotations, hoodExtendedPositionRotations));
  }

  private double getHoodNormalizedPosition(double hoodPositionRotations) {
    return clampUnitInterval(
        inverseInterpolate(
            hoodPositionRotations, hoodRetractedPositionRotations, hoodExtendedPositionRotations));
  }

  private static double interpolate(double start, double end, double fraction) {
    return start + ((end - start) * fraction);
  }

  private static double inverseInterpolate(double value, double start, double end) {
    double span = end - start;
    if (Math.abs(span) < 1e-6) {
      return 0.0;
    }
    return (value - start) / span;
  }

  private static double clampUnitInterval(double value) {
    return MathUtil.clamp(value, 0.0, 1.0);
  }

  private static double optionalDoubleOrNaN(OptionalDouble value) {
    return value.orElse(Double.NaN);
  }

  private static double optionalClearanceToHubOrNaN(
      OptionalDouble projectileHeightMeters, double hubCenterHeightMeters) {
    return projectileHeightMeters.isPresent()
        ? projectileHeightMeters.getAsDouble() - hubCenterHeightMeters
        : Double.NaN;
  }

  private static double clampAirtime(double airtimeSeconds) {
    return MathUtil.clamp(
        airtimeSeconds, ShooterConstants.minAirtimeSeconds, ShooterConstants.maxAirtimeSeconds);
  }

  private void logControlState(
      double pair1VelocityCommandRadPerSec, double pair2VelocityCommandRadPerSec) {
    Logger.recordOutput("Shooter/Control/Enabled", shotControlEnabled);
    Logger.recordOutput("Shooter/Control/Pair1SetpointRadPerSec", pair1WheelSetpointRadPerSec);
    Logger.recordOutput("Shooter/Control/Pair2SetpointRadPerSec", pair2WheelSetpointRadPerSec);
    Logger.recordOutput("Shooter/Control/WheelSpeedScale", wheelSpeedScale);
    Logger.recordOutput("Shooter/Control/Pair1Direction", pair1Direction);
    Logger.recordOutput("Shooter/Control/Pair2Direction", pair2Direction);
    Logger.recordOutput("Shooter/Control/Pair1CommandRadPerSec", pair1VelocityCommandRadPerSec);
    Logger.recordOutput("Shooter/Control/Pair2CommandRadPerSec", pair2VelocityCommandRadPerSec);
    Logger.recordOutput("Shooter/Control/HoodSetpointRotations", hoodSetpointMotorRotations);
    Logger.recordOutput(
        "Shooter/Control/HoodSetpointNormalized",
        getHoodNormalizedPosition(hoodSetpointMotorRotations));
    Logger.recordOutput("Shooter/Control/ManualHoodOverrideEnabled", manualHoodOverrideEnabled);
    Logger.recordOutput(
        "Shooter/Control/HoodSetpointDegrees",
        motorRotationsToHoodAngle(hoodSetpointMotorRotations).getDegrees());
    Logger.recordOutput(
        "Shooter/Config/HoodRetractedPositionRotations", hoodRetractedPositionRotations);
    Logger.recordOutput(
        "Shooter/Config/HoodExtendedPositionRotations", hoodExtendedPositionRotations);
    Logger.recordOutput(
        "Shooter/Measured/Pair1VelocityRadPerSec", inputs.pair1LeaderVelocityRadPerSec);
    Logger.recordOutput(
        "Shooter/Measured/Pair2VelocityRadPerSec", inputs.pair2LeaderVelocityRadPerSec);
    Logger.recordOutput("Shooter/Measured/HoodPositionRotations", inputs.hoodPositionRotations);
    Logger.recordOutput(
        "Shooter/Measured/HoodPositionNormalized",
        getHoodNormalizedPosition(inputs.hoodPositionRotations));
    Logger.recordOutput(
        "Shooter/Measured/HoodAngleDegrees",
        motorRotationsToHoodAngle(inputs.hoodPositionRotations).getDegrees());
    Logger.recordOutput(
        "Shooter/Measured/EstimatedLaunchSpeedMetersPerSec",
        getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec());
    Logger.recordOutput("Shooter/Simulation/WheelsReady", areWheelsReadyForSimulatedShot());
    Logger.recordOutput("Shooter/Interlock/WheelsReadyForShot", areWheelsAtSpeedForShot());
    Logger.recordOutput("Shooter/Interlock/ReadyToFire", isReadyToFire());
    Logger.recordOutput("Shooter/Status/Pair1Connected", inputs.pair1Connected);
    Logger.recordOutput("Shooter/Status/Pair2Connected", inputs.pair2Connected);
    Logger.recordOutput("Shooter/Status/HoodConnected", inputs.hoodConnected);
  }

  private void logHubShotSolution(
      HubShotSolution solution,
      Pose2d baseHubPose,
      Pose2d compensatedHubPose,
      Translation2d robotFieldVelocityMetersPerSec,
      int motionSolveIterations,
      boolean motionSolveConverged) {
    Translation2d compensationOffset =
        compensatedHubPose.getTranslation().minus(baseHubPose.getTranslation());
    double apexHeightMeters =
        calculateApexHeightMeters(solution.launchSpeedMetersPerSec(), solution.launchAngle());
    OptionalDouble verticalVelocityAtHubMetersPerSec =
        calculateVerticalVelocityAtDistance(
            solution.distanceMeters(), solution.launchSpeedMetersPerSec(), solution.launchAngle());
    boolean topEntrySatisfied =
        verticalVelocityAtHubMetersPerSec.orElse(Double.POSITIVE_INFINITY)
            <= -ShooterConstants.hubTopEntryMinDescentVelocityMetersPerSec;
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
    Logger.recordOutput("Shooter/HubShot/BaseTargetPose", baseHubPose);
    Logger.recordOutput("Shooter/HubShot/CompensatedTargetPose", compensatedHubPose);
    Logger.recordOutput(
        "Shooter/HubShot/RobotFieldVelocityX", robotFieldVelocityMetersPerSec.getX());
    Logger.recordOutput(
        "Shooter/HubShot/RobotFieldVelocityY", robotFieldVelocityMetersPerSec.getY());
    Logger.recordOutput("Shooter/HubShot/CompensationOffsetX", compensationOffset.getX());
    Logger.recordOutput("Shooter/HubShot/CompensationOffsetY", compensationOffset.getY());
    Logger.recordOutput("Shooter/HubShot/ApexHeightMeters", apexHeightMeters);
    Logger.recordOutput(
        "Shooter/HubShot/ApexAboveHubCenterMeters",
        apexHeightMeters - ShooterConstants.hubCenterHeightMeters);
    Logger.recordOutput(
        "Shooter/HubShot/VerticalVelocityAtHubMetersPerSec",
        optionalDoubleOrNaN(verticalVelocityAtHubMetersPerSec));
    Logger.recordOutput("Shooter/HubShot/TopEntrySatisfied", topEntrySatisfied);
    Logger.recordOutput("Shooter/HubShot/MotionCompVelocityScale", hubMotionCompVelocityScale);
    Logger.recordOutput("Shooter/HubShot/MotionCompLeadSeconds", hubMotionCompLeadSeconds);
    Logger.recordOutput("Shooter/HubShot/MotionSolveIterations", motionSolveIterations);
    Logger.recordOutput("Shooter/HubShot/MotionSolveConverged", motionSolveConverged);
  }
}
