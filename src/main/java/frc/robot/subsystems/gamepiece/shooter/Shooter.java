package frc.robot.subsystems.gamepiece.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.targeting.HubTargetingGeometry;
import frc.robot.util.NetworkTablesUtil;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final double loopPeriodSeconds = 0.02;

  private enum HoodHomingPhase {
    RELAX_BEFORE_CALIBRATION,
    SEEK_RETRACTED_HARD_STOP,
    SEEK_EXTENDED_HARD_STOP
  }

  private static record WheelSpeedSetpoints(double pair1RadPerSec, double pair2RadPerSec) {}

  private static record MotionCompensatedHubShotSolution(
      HubShotSolution shotSolution, Pose2d compensatedHubPose, int iterations, boolean converged) {}

  private static record CalibrationDerivedMeasurement(
      double horizontalVelocityMetersPerSec,
      double verticalVelocityMetersPerSec,
      double launchSpeedMetersPerSec,
      Rotation2d launchAngle,
      boolean valid) {}

  private static record PredictedHubShot(
      boolean scoring,
      double horizontalErrorMeters,
      double verticalErrorMeters,
      double evaluationTimeSeconds,
      boolean descendingAtTarget) {}

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
  private boolean trenchSafetyRetractOverrideEnabled = false;
  private boolean manualHoodOverrideEnabled = false;
  private boolean manualWheelOverrideEnabled = false;
  private boolean manualHoodJogAllowedBeforeHoming = false;
  private double launchHeightMeters = ShooterConstants.defaultLaunchHeightMeters;
  private double pair1WheelSetpointRadPerSec = 0.0;
  private double pair2WheelSetpointRadPerSec = 0.0;
  private double pair1VelocityCommandRadPerSec = 0.0;
  private double pair2VelocityCommandRadPerSec = 0.0;
  private double wheelSpeedScale = ShooterConstants.defaultWheelSpeedScale;
  private double operatorWheelThrottleScale = 1.0;
  private double pair1Direction = ShooterConstants.defaultPair1Direction;
  private double pair2Direction = ShooterConstants.defaultPair2Direction;
  private double hoodSetpointMotorRotations = 0.0;
  private double hoodRetractedPositionRotations =
      ShooterConstants.defaultHoodRetractedPositionRotations;
  private double hoodExtendedPositionRotations =
      ShooterConstants.defaultHoodExtendedPositionRotations;
  private double wheelVelocityKp = ShooterConstants.shooterVelocityKp;
  private double wheelVelocityKi = ShooterConstants.shooterVelocityKi;
  private double wheelVelocityKd = ShooterConstants.shooterVelocityKd;
  private double wheelVelocityKv = ShooterConstants.shooterVelocityKv;
  private double hoodPositionKp = ShooterConstants.hoodPositionKp;
  private double hoodPositionKi = ShooterConstants.hoodPositionKi;
  private double hoodPositionKd = ShooterConstants.hoodPositionKd;
  private boolean hoodHomingActive = false;
  private boolean hoodHomed = false;
  private boolean hoodHomingSucceeded = false;
  private HoodHomingPhase hoodHomingPhase = HoodHomingPhase.SEEK_RETRACTED_HARD_STOP;
  private double hoodHomingElapsedSeconds = 0.0;
  private double hoodHomingPhaseElapsedSeconds = 0.0;
  private double hoodHomingStallSeconds = 0.0;
  private double hoodHomingRetractedHardStopRotations = 0.0;
  private double hoodHomingExtendedHardStopRotations = 0.0;
  private double hubMotionCompVelocityScale = ShooterConstants.hubMotionCompensationVelocityScale;
  private double hubMotionCompLeadSeconds = ShooterConstants.hubMotionCompensationLeadSeconds;
  private double hubShotPredictionLaunchLeadSeconds =
      ShooterConstants.hubShotPredictionLaunchLeadSeconds;
  private double hubAimHeightOffsetMeters = ShooterConstants.defaultHubAimHeightOffsetMeters;
  private boolean calibrationModeEnabled = false;
  private double calibrationHoodSetpointRotations = 0.0;
  private double calibrationPair1WheelSetpointRadPerSec = 0.0;
  private double calibrationPair2WheelSetpointRadPerSec = 0.0;
  private double calibrationMeasuredDistanceMeters = 0.0;
  private double calibrationMeasuredHeightDeltaMeters =
      ShooterConstants.defaultHubAimHeightMeters - ShooterConstants.defaultLaunchHeightMeters;
  private double calibrationMeasuredAirtimeSeconds = Double.NaN;
  private double calibrationVideoLaunchAngleDegrees = Double.NaN;
  private String calibrationNotes = "";
  private int calibrationRecordedSampleCount = 0;
  private double calibrationLastRecordedTimestampSeconds = Double.NaN;
  private double lastSimShotTimestampSeconds = Double.NEGATIVE_INFINITY;
  private double lastHubMotionSampleTimestampSeconds = Double.NaN;
  private Pose2d latestHubRobotPose = Pose2d.kZero;
  private Pose2d latestHubPose = Pose2d.kZero;
  private Translation2d latestRobotFieldVelocityMetersPerSec = new Translation2d();
  private Translation2d latestRobotFieldAccelerationMetersPerSecSquared = new Translation2d();
  private HubShotSolution latestHubShotSolution =
      new HubShotSolution(
          0.0,
          ShooterConstants.defaultLaunchAngle,
          ShooterConstants.defaultLaunchSpeedMetersPerSec,
          speedToPower(ShooterConstants.defaultLaunchSpeedMetersPerSec),
          ShooterConstants.fallbackAirtimeSeconds,
          false);
  private PredictedHubShot latestPredictedHubShot =
      new PredictedHubShot(false, Double.NaN, Double.NaN, Double.NaN, false);
  private double wheelPowerRamp = 0.01; // amount increase per update, about 2 seconds
  private double wheelPowerPercent = 0.0;
  private final NetworkTable subsystemTable =
      NetworkTablesUtil.domain(ShooterConstants.configTableName);
  private final NetworkTable tuningTable = NetworkTablesUtil.tuningCommon(subsystemTable);
  private final NetworkTable pidTuningTable =
      NetworkTablesUtil.tuningMode(subsystemTable).getSubTable("PID");
  private final NetworkTable motionCompTuningTable =
      NetworkTablesUtil.tuningMode("Targeting/Hub").getSubTable("MotionCompensation");
  private final NetworkTable telemetryTable = NetworkTablesUtil.telemetry(subsystemTable);
  private final NetworkTable calibrationTuningTable = tuningTable.getSubTable("Calibration");
  private final NetworkTable calibrationTelemetryTable = telemetryTable.getSubTable("Calibration");
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
  private final NetworkTableEntry hubAimHeightOffsetEntry =
      tuningTable.getEntry("Hub/AimHeightOffsetMeters");
  private final NetworkTableEntry wheelVelocityKpEntry =
      pidTuningTable.getEntry("Wheels/Velocity/Kp");
  private final NetworkTableEntry wheelVelocityKiEntry =
      pidTuningTable.getEntry("Wheels/Velocity/Ki");
  private final NetworkTableEntry wheelVelocityKdEntry =
      pidTuningTable.getEntry("Wheels/Velocity/Kd");
  private final NetworkTableEntry wheelVelocityKvEntry =
      pidTuningTable.getEntry("Wheels/Velocity/Kv");
  private final NetworkTableEntry hoodPositionKpEntry = pidTuningTable.getEntry("Hood/Position/Kp");
  private final NetworkTableEntry hoodPositionKiEntry = pidTuningTable.getEntry("Hood/Position/Ki");
  private final NetworkTableEntry hoodPositionKdEntry = pidTuningTable.getEntry("Hood/Position/Kd");
  private final NetworkTableEntry hubMotionCompVelocityScaleEntry =
      motionCompTuningTable.getEntry("VelocityScale");
  private final NetworkTableEntry hubMotionCompLeadSecondsEntry =
      motionCompTuningTable.getEntry("LeadSeconds");
  private final NetworkTableEntry hubShotPredictionLaunchLeadSecondsEntry =
      motionCompTuningTable.getEntry("PredictionLaunchLeadSeconds");
  private final NetworkTableEntry calibrationModeEnabledEntry =
      calibrationTuningTable.getEntry("Enabled");
  private final NetworkTableEntry calibrationHoodSetpointRotationsEntry =
      calibrationTuningTable.getEntry("Hood/SetpointRotations");
  private final NetworkTableEntry calibrationPair1WheelSetpointEntry =
      calibrationTuningTable.getEntry("Wheels/Pair1SetpointRadPerSec");
  private final NetworkTableEntry calibrationPair2WheelSetpointEntry =
      calibrationTuningTable.getEntry("Wheels/Pair2SetpointRadPerSec");
  private final NetworkTableEntry calibrationDistanceEntry =
      calibrationTuningTable.getEntry("Measurement/DistanceMeters");
  private final NetworkTableEntry calibrationHeightDeltaEntry =
      calibrationTuningTable.getEntry("Measurement/HeightDeltaMeters");
  private final NetworkTableEntry calibrationAirtimeEntry =
      calibrationTuningTable.getEntry("Measurement/AirtimeSeconds");
  private final NetworkTableEntry calibrationVideoLaunchAngleEntry =
      calibrationTuningTable.getEntry("Measurement/VideoLaunchAngleDegrees");
  private final NetworkTableEntry calibrationNotesEntry =
      calibrationTuningTable.getEntry("Measurement/Notes");
  private final NetworkTableEntry calibrationModeTelemetryEntry =
      calibrationTelemetryTable.getEntry("ModeEnabled");
  private final NetworkTableEntry calibrationConfiguredHoodSetpointEntry =
      calibrationTelemetryTable.getEntry("Configured/HoodSetpointRotations");
  private final NetworkTableEntry calibrationConfiguredHoodAngleEntry =
      calibrationTelemetryTable.getEntry("Configured/HoodSetpointDegrees");
  private final NetworkTableEntry calibrationConfiguredPair1WheelSetpointEntry =
      calibrationTelemetryTable.getEntry("Configured/Wheels/Pair1SetpointRadPerSec");
  private final NetworkTableEntry calibrationConfiguredPair2WheelSetpointEntry =
      calibrationTelemetryTable.getEntry("Configured/Wheels/Pair2SetpointRadPerSec");
  private final NetworkTableEntry calibrationConfiguredDistanceEntry =
      calibrationTelemetryTable.getEntry("Configured/Measurement/DistanceMeters");
  private final NetworkTableEntry calibrationConfiguredHeightDeltaEntry =
      calibrationTelemetryTable.getEntry("Configured/Measurement/HeightDeltaMeters");
  private final NetworkTableEntry calibrationConfiguredAirtimeEntry =
      calibrationTelemetryTable.getEntry("Configured/Measurement/AirtimeSeconds");
  private final NetworkTableEntry calibrationConfiguredVideoLaunchAngleEntry =
      calibrationTelemetryTable.getEntry("Configured/Measurement/VideoLaunchAngleDegrees");
  private final NetworkTableEntry calibrationConfiguredNotesEntry =
      calibrationTelemetryTable.getEntry("Configured/Measurement/Notes");
  private final NetworkTableEntry calibrationRecordedSampleCountEntry =
      calibrationTelemetryTable.getEntry("RecordedSampleCount");
  private final NetworkTableEntry calibrationLastRecordedTimestampEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/TimestampSeconds");
  private final NetworkTableEntry calibrationLastRecordedModeEnabledEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/ModeEnabled");
  private final NetworkTableEntry calibrationLastRecordedHoodSetpointEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/HoodSetpointRotations");
  private final NetworkTableEntry calibrationLastRecordedHoodSetpointAngleEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/HoodSetpointDegrees");
  private final NetworkTableEntry calibrationLastRecordedMeasuredHoodPositionEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/HoodMeasuredRotations");
  private final NetworkTableEntry calibrationLastRecordedMeasuredHoodAngleEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/HoodMeasuredDegrees");
  private final NetworkTableEntry calibrationLastRecordedPair1SetpointEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Wheels/Pair1SetpointRadPerSec");
  private final NetworkTableEntry calibrationLastRecordedPair2SetpointEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Wheels/Pair2SetpointRadPerSec");
  private final NetworkTableEntry calibrationLastRecordedPair1MeasuredEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Wheels/Pair1MeasuredRadPerSec");
  private final NetworkTableEntry calibrationLastRecordedPair2MeasuredEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Wheels/Pair2MeasuredRadPerSec");
  private final NetworkTableEntry calibrationLastRecordedEstimatedLaunchSpeedEntry =
      calibrationTelemetryTable.getEntry(
          "LastRecorded/Wheels/EstimatedLaunchSpeedFromMeasuredMetersPerSec");
  private final NetworkTableEntry calibrationLastRecordedDistanceEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Measurement/DistanceMeters");
  private final NetworkTableEntry calibrationLastRecordedHeightDeltaEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Measurement/HeightDeltaMeters");
  private final NetworkTableEntry calibrationLastRecordedAirtimeEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Measurement/AirtimeSeconds");
  private final NetworkTableEntry calibrationLastRecordedVideoLaunchAngleEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Measurement/VideoLaunchAngleDegrees");
  private final NetworkTableEntry calibrationLastRecordedNotesEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Measurement/Notes");
  private final NetworkTableEntry calibrationLastRecordedDerivedValidEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Derived/Valid");
  private final NetworkTableEntry calibrationLastRecordedHorizontalVelocityEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Derived/HorizontalVelocityMetersPerSec");
  private final NetworkTableEntry calibrationLastRecordedVerticalVelocityEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Derived/VerticalVelocityMetersPerSec");
  private final NetworkTableEntry calibrationLastRecordedDerivedLaunchSpeedEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Derived/LaunchSpeedMetersPerSec");
  private final NetworkTableEntry calibrationLastRecordedDerivedLaunchAngleEntry =
      calibrationTelemetryTable.getEntry("LastRecorded/Derived/LaunchAngleDegrees");

  public Shooter() {
    this(new ShooterIO() {});
  }

  public Shooter(ShooterIO io) {
    this.io = io;
    hoodSetpointMotorRotations = hoodAngleToMotorRotations(ShooterConstants.defaultLaunchAngle);
    calibrationHoodSetpointRotations = hoodSetpointMotorRotations;
    WheelSpeedSetpoints defaultCalibrationWheelSetpoints =
        calculateWheelSetpointsFromLaunchSpeed(ShooterConstants.defaultLaunchSpeedMetersPerSec);
    calibrationPair1WheelSetpointRadPerSec = defaultCalibrationWheelSetpoints.pair1RadPerSec();
    calibrationPair2WheelSetpointRadPerSec = defaultCalibrationWheelSetpoints.pair2RadPerSec();
    configureNetworkTableDefaults();
    resetPidTuningEntriesToDefaults();
    loadNetworkTableConfig();
  }

  @Override
  public void periodic() {
    if (shotControlEnabled) {
      if (shouldUseFullWheelPowerTarget()) {
        wheelPowerPercent = 1.0;
      } else {
        wheelPowerPercent += wheelPowerRamp;
        wheelPowerPercent = Math.min(wheelPowerPercent, 1.0);
      }
    } else wheelPowerPercent = 0.0;

    io.updateInputs(inputs);
    Logger.processInputs(NetworkTablesUtil.logPath("GamePiece/Shooter/Inputs"), inputs);
    loadNetworkTableConfig();
    clampHoodSetpointToCalibrationRange();
    if (calibrationModeEnabled) {
      hoodSetpointMotorRotations = calibrationHoodSetpointRotations;
      pair1WheelSetpointRadPerSec = calibrationPair1WheelSetpointRadPerSec;
      pair2WheelSetpointRadPerSec = calibrationPair2WheelSetpointRadPerSec;
    }
    double pair1VelocityTargetRadPerSec =
        shotControlEnabled ? getPair1VelocityTargetSetpointRadPerSec() * wheelPowerPercent : 0.0;
    double pair2VelocityTargetRadPerSec =
        shotControlEnabled ? getPair2VelocityTargetSetpointRadPerSec() * wheelPowerPercent : 0.0;
    pair1VelocityCommandRadPerSec =
        updateWheelVelocityCommand(pair1VelocityCommandRadPerSec, pair1VelocityTargetRadPerSec);
    pair2VelocityCommandRadPerSec =
        updateWheelVelocityCommand(pair2VelocityCommandRadPerSec, pair2VelocityTargetRadPerSec);

    io.setWheelVelocityClosedLoopGains(
        wheelVelocityKp, wheelVelocityKi, wheelVelocityKd, wheelVelocityKv);
    io.setHoodPositionClosedLoopGains(hoodPositionKp, hoodPositionKi, hoodPositionKd);
    io.setWheelVelocitySetpoints(pair1VelocityCommandRadPerSec, pair2VelocityCommandRadPerSec);
    if (hoodHomingActive) {
      updateHoodHoming();
    } else if (!canApplyHoodSetpointWithoutHoming()) {
      // On real hardware, hold the current measured position until homing establishes a reference.
      io.setHoodPositionSetpointRotations(inputs.hoodPositionRotations);
    } else {
      io.setHoodPositionSetpointRotations(getAppliedHoodSetpointMotorRotations());
    }
    publishHoodEncoderToNetworkTables();
    publishWheelTelemetryToNetworkTables(
        pair1VelocityCommandRadPerSec, pair2VelocityCommandRadPerSec);
    publishHoodShotOverlayToNetworkTables(
        pair1VelocityCommandRadPerSec, pair2VelocityCommandRadPerSec);
    publishCalibrationTelemetry();
    refreshPredictedHubShot();

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

  public boolean isHubShotPredictedToScore() {
    return calibrationModeEnabled
        || (latestHubShotSolution.feasible() && latestPredictedHubShot.scoring());
  }

  public boolean isHubShotSolutionFeasible() {
    return isHubShotPredictedToScore();
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
    setShotControlEnabled(enabled, false);
  }

  public void setShotControlEnabled(boolean enabled, boolean skipWheelPowerRamp) {
    shotControlEnabled = enabled;
    if (enabled && skipWheelPowerRamp) {
      wheelPowerPercent = 1.0;
    }
    if (!enabled) {
      lastSimShotTimestampSeconds = Double.NEGATIVE_INFINITY;
    }
  }

  public void setOperatorWheelThrottleScale(double throttleScale) {
    operatorWheelThrottleScale = clampOperatorWheelThrottleScale(throttleScale);
  }

  public void setLaunchHeightMeters(double launchHeightMeters) {
    this.launchHeightMeters = launchHeightMeters;
  }

  public double getLaunchHeightMeters() {
    return launchHeightMeters;
  }

  public double getHubAimHeightMeters() {
    return ShooterConstants.hubCenterHeightMeters + hubAimHeightOffsetMeters;
  }

  public boolean isManualHoodOverrideEnabled() {
    return manualHoodOverrideEnabled;
  }

  public boolean isTrenchSafetyRetractOverrideEnabled() {
    return trenchSafetyRetractOverrideEnabled;
  }

  public boolean isManualWheelOverrideEnabled() {
    return manualWheelOverrideEnabled;
  }

  public void setTrenchSafetyRetractOverrideEnabled(boolean enabled) {
    trenchSafetyRetractOverrideEnabled = enabled;
  }

  public void setManualHoodOverrideEnabled(boolean enabled) {
    manualHoodOverrideEnabled = calibrationModeEnabled ? false : enabled;
    if (!manualHoodOverrideEnabled) {
      manualHoodJogAllowedBeforeHoming = false;
    }
  }

  public void setManualWheelOverrideEnabled(boolean enabled) {
    manualWheelOverrideEnabled = calibrationModeEnabled ? false : enabled;
  }

  public void setManualHoodSetpointDegrees(double hoodAngleDegrees) {
    Rotation2d targetAngle =
        Rotation2d.fromDegrees(
            MathUtil.clamp(
                hoodAngleDegrees,
                ShooterConstants.minLaunchAngle.getDegrees(),
                ShooterConstants.maxLaunchAngle.getDegrees()));
    if (calibrationModeEnabled) {
      setCalibrationHoodSetpointRotations(hoodAngleToMotorRotations(targetAngle));
      return;
    }
    manualHoodOverrideEnabled = true;
    manualHoodJogAllowedBeforeHoming = false;
    hoodSetpointMotorRotations = hoodAngleToMotorRotations(targetAngle);
  }

  public void setManualWheelSpeedRpm(double wheelSpeedRpm) {
    double targetWheelSpeedRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(Math.abs(wheelSpeedRpm));
    WheelSpeedSetpoints manualWheelSetpoints =
        sharedCalibrationWheelSetpoints(targetWheelSpeedRadPerSec, targetWheelSpeedRadPerSec);
    if (calibrationModeEnabled) {
      setCalibrationWheelSetpointsRadPerSec(
          manualWheelSetpoints.pair1RadPerSec(), manualWheelSetpoints.pair2RadPerSec());
      return;
    }
    manualWheelOverrideEnabled = true;
    pair1WheelSetpointRadPerSec = manualWheelSetpoints.pair1RadPerSec();
    pair2WheelSetpointRadPerSec = manualWheelSetpoints.pair2RadPerSec();
  }

  public void adjustHoodSetpointDegrees(double deltaDegrees) {
    if (calibrationModeEnabled) {
      double targetAngleDegrees =
          MathUtil.clamp(
              motorRotationsToHoodAngle(calibrationHoodSetpointRotations).getDegrees()
                  + deltaDegrees,
              ShooterConstants.minLaunchAngle.getDegrees(),
              ShooterConstants.maxLaunchAngle.getDegrees());
      setCalibrationHoodSetpointRotations(
          hoodAngleToMotorRotations(Rotation2d.fromDegrees(targetAngleDegrees)));
      return;
    }
    prepareForManualHoodJog();
    double targetAngleDegrees =
        MathUtil.clamp(
            motorRotationsToHoodAngle(hoodSetpointMotorRotations).getDegrees() + deltaDegrees,
            ShooterConstants.minLaunchAngle.getDegrees(),
            ShooterConstants.maxLaunchAngle.getDegrees());
    hoodSetpointMotorRotations =
        hoodAngleToMotorRotations(Rotation2d.fromDegrees(targetAngleDegrees));
  }

  public void adjustHoodSetpointRotations(double deltaRotations) {
    if (calibrationModeEnabled) {
      setCalibrationHoodSetpointRotations(calibrationHoodSetpointRotations + deltaRotations);
      return;
    }
    prepareForManualHoodJog();
    hoodSetpointMotorRotations =
        clampToHoodCalibrationRange(hoodSetpointMotorRotations + deltaRotations);
  }

  public boolean isCalibrationModeEnabled() {
    return calibrationModeEnabled;
  }

  public void setCalibrationModeEnabled(boolean enabled) {
    calibrationModeEnabled = enabled;
    if (enabled) {
      manualHoodOverrideEnabled = false;
      manualWheelOverrideEnabled = false;
      manualHoodJogAllowedBeforeHoming = false;
      hoodSetpointMotorRotations = calibrationHoodSetpointRotations;
      pair1WheelSetpointRadPerSec = calibrationPair1WheelSetpointRadPerSec;
      pair2WheelSetpointRadPerSec = calibrationPair2WheelSetpointRadPerSec;
    }
    calibrationModeEnabledEntry.setBoolean(calibrationModeEnabled);
  }

  public void setCalibrationHoodSetpointRotations(double hoodSetpointRotations) {
    calibrationHoodSetpointRotations = clampToHoodCalibrationRange(hoodSetpointRotations);
    calibrationHoodSetpointRotationsEntry.setDouble(calibrationHoodSetpointRotations);
    if (calibrationModeEnabled) {
      hoodSetpointMotorRotations = calibrationHoodSetpointRotations;
    }
  }

  public void setCalibrationWheelSetpointsRadPerSec(double pair1RadPerSec, double pair2RadPerSec) {
    WheelSpeedSetpoints normalizedCalibrationWheelSetpoints =
        sharedCalibrationWheelSetpoints(pair1RadPerSec, pair2RadPerSec);
    calibrationPair1WheelSetpointRadPerSec = normalizedCalibrationWheelSetpoints.pair1RadPerSec();
    calibrationPair2WheelSetpointRadPerSec = normalizedCalibrationWheelSetpoints.pair2RadPerSec();
    calibrationPair1WheelSetpointEntry.setDouble(calibrationPair1WheelSetpointRadPerSec);
    calibrationPair2WheelSetpointEntry.setDouble(calibrationPair2WheelSetpointRadPerSec);
    if (calibrationModeEnabled) {
      pair1WheelSetpointRadPerSec = calibrationPair1WheelSetpointRadPerSec;
      pair2WheelSetpointRadPerSec = calibrationPair2WheelSetpointRadPerSec;
    }
  }

  public void setCalibrationMeasurement(
      double distanceMeters,
      double heightDeltaMeters,
      double airtimeSeconds,
      double videoLaunchAngleDegrees,
      String notes) {
    calibrationMeasuredDistanceMeters = Math.max(0.0, distanceMeters);
    calibrationMeasuredHeightDeltaMeters = heightDeltaMeters;
    calibrationMeasuredAirtimeSeconds = airtimeSeconds;
    calibrationVideoLaunchAngleDegrees = videoLaunchAngleDegrees;
    calibrationNotes = notes != null ? notes : "";
    calibrationDistanceEntry.setDouble(calibrationMeasuredDistanceMeters);
    calibrationHeightDeltaEntry.setDouble(calibrationMeasuredHeightDeltaMeters);
    calibrationAirtimeEntry.setDouble(calibrationMeasuredAirtimeSeconds);
    calibrationVideoLaunchAngleEntry.setDouble(calibrationVideoLaunchAngleDegrees);
    calibrationNotesEntry.setString(calibrationNotes);
  }

  public Command enableCalibrationModeCommand() {
    return Commands.runOnce(() -> setCalibrationModeEnabled(true), this).ignoringDisable(true);
  }

  public Command disableCalibrationModeCommand() {
    return Commands.runOnce(() -> setCalibrationModeEnabled(false), this).ignoringDisable(true);
  }

  public Command recordCalibrationSampleCommand() {
    return Commands.runOnce(this::recordCalibrationSample, this).ignoringDisable(true);
  }

  public Command homeHoodToHardStopCommand() {
    return startHoodHomingToHardStopCommand()
        .andThen(Commands.waitUntil(() -> !hoodHomingActive))
        .finallyDo(
            interrupted -> {
              if (interrupted) {
                cancelHoodHoming();
              }
            });
  }

  public Command startHoodHomingToHardStopCommand() {
    return runOnce(this::startHoodHoming);
  }

  public void cancelHoodHomingToHardStop() {
    cancelHoodHoming();
  }

  public boolean isHoodHomingActive() {
    return hoodHomingActive;
  }

  public boolean isHoodHomed() {
    return hoodHomed;
  }

  public boolean didLastHoodHomingSucceed() {
    return hoodHomingSucceeded;
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
    updateRobotMotionState(robotFieldVelocity);
    latestHubRobotPose = robotPose;
    latestHubPose = hubPose;
    latestRobotFieldVelocityMetersPerSec = robotFieldVelocity;
    if (calibrationModeEnabled) {
      latestHubShotSolution = createCalibrationModeShotSolution();
      refreshPredictedHubShot();
      applyHubShotSetpoints(latestHubShotSolution);
      return latestHubShotSolution;
    }

    double targetHeightMeters = getHubAimHeightMeters();
    double targetHeightDeltaMeters = targetHeightMeters - launchHeightMeters;
    MotionCompensatedHubShotSolution motionCompensatedSolution =
        solveHubShotWithMotionCompensation(
            robotPose, hubPose, robotFieldVelocity, targetHeightDeltaMeters);
    latestHubShotSolution = motionCompensatedSolution.shotSolution();
    refreshPredictedHubShot();
    applyHubShotSetpoints(latestHubShotSolution);
    logHubShotSolution(
        latestHubShotSolution,
        hubPose,
        motionCompensatedSolution.compensatedHubPose(),
        targetHeightMeters,
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
    hubAimHeightOffsetEntry.setDefaultDouble(ShooterConstants.defaultHubAimHeightOffsetMeters);
    wheelVelocityKpEntry.setDefaultDouble(ShooterConstants.shooterVelocityKp);
    wheelVelocityKiEntry.setDefaultDouble(ShooterConstants.shooterVelocityKi);
    wheelVelocityKdEntry.setDefaultDouble(ShooterConstants.shooterVelocityKd);
    wheelVelocityKvEntry.setDefaultDouble(ShooterConstants.shooterVelocityKv);
    hoodPositionKpEntry.setDefaultDouble(ShooterConstants.hoodPositionKp);
    hoodPositionKiEntry.setDefaultDouble(ShooterConstants.hoodPositionKi);
    hoodPositionKdEntry.setDefaultDouble(ShooterConstants.hoodPositionKd);
    hubMotionCompVelocityScaleEntry.setDefaultDouble(
        ShooterConstants.hubMotionCompensationVelocityScale);
    hubMotionCompLeadSecondsEntry.setDefaultDouble(
        ShooterConstants.hubMotionCompensationLeadSeconds);
    hubShotPredictionLaunchLeadSecondsEntry.setDefaultDouble(
        ShooterConstants.hubShotPredictionLaunchLeadSeconds);
    hoodMinAngleFromFloorEntry.setDefaultDouble(
        ShooterConstants.minHoodAngleFromFloor.getDegrees());
    hoodMaxAngleFromFloorEntry.setDefaultDouble(
        ShooterConstants.maxHoodAngleFromFloor.getDegrees());
    calibrationModeEnabledEntry.setDefaultBoolean(false);
    calibrationHoodSetpointRotationsEntry.setDefaultDouble(calibrationHoodSetpointRotations);
    calibrationPair1WheelSetpointEntry.setDefaultDouble(calibrationPair1WheelSetpointRadPerSec);
    calibrationPair2WheelSetpointEntry.setDefaultDouble(calibrationPair2WheelSetpointRadPerSec);
    calibrationDistanceEntry.setDefaultDouble(calibrationMeasuredDistanceMeters);
    calibrationHeightDeltaEntry.setDefaultDouble(calibrationMeasuredHeightDeltaMeters);
    calibrationAirtimeEntry.setDefaultDouble(calibrationMeasuredAirtimeSeconds);
    calibrationVideoLaunchAngleEntry.setDefaultDouble(calibrationVideoLaunchAngleDegrees);
    calibrationNotesEntry.setDefaultString(calibrationNotes);
  }

  private void resetPidTuningEntriesToDefaults() {
    wheelVelocityKpEntry.setDouble(ShooterConstants.shooterVelocityKp);
    wheelVelocityKiEntry.setDouble(ShooterConstants.shooterVelocityKi);
    wheelVelocityKdEntry.setDouble(ShooterConstants.shooterVelocityKd);
    wheelVelocityKvEntry.setDouble(ShooterConstants.shooterVelocityKv);
    hoodPositionKpEntry.setDouble(ShooterConstants.hoodPositionKp);
    hoodPositionKiEntry.setDouble(ShooterConstants.hoodPositionKi);
    hoodPositionKdEntry.setDouble(ShooterConstants.hoodPositionKd);
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
    hubAimHeightOffsetMeters =
        sanitizeFinite(
            hubAimHeightOffsetEntry.getDouble(hubAimHeightOffsetMeters), hubAimHeightOffsetMeters);
    wheelVelocityKp =
        sanitizeFinite(wheelVelocityKpEntry.getDouble(wheelVelocityKp), wheelVelocityKp);
    wheelVelocityKi =
        sanitizeFinite(wheelVelocityKiEntry.getDouble(wheelVelocityKi), wheelVelocityKi);
    wheelVelocityKd =
        sanitizeFinite(wheelVelocityKdEntry.getDouble(wheelVelocityKd), wheelVelocityKd);
    wheelVelocityKv =
        sanitizeFinite(wheelVelocityKvEntry.getDouble(wheelVelocityKv), wheelVelocityKv);
    hoodPositionKp = sanitizeFinite(hoodPositionKpEntry.getDouble(hoodPositionKp), hoodPositionKp);
    hoodPositionKi = sanitizeFinite(hoodPositionKiEntry.getDouble(hoodPositionKi), hoodPositionKi);
    hoodPositionKd = sanitizeFinite(hoodPositionKdEntry.getDouble(hoodPositionKd), hoodPositionKd);
    hubMotionCompVelocityScale =
        clampMotionCompVelocityScale(
            hubMotionCompVelocityScaleEntry.getDouble(hubMotionCompVelocityScale));
    hubMotionCompLeadSeconds =
        clampMotionCompLeadSeconds(
            hubMotionCompLeadSecondsEntry.getDouble(hubMotionCompLeadSeconds));
    hubShotPredictionLaunchLeadSeconds =
        clampPredictionLaunchLeadSeconds(
            hubShotPredictionLaunchLeadSecondsEntry.getDouble(hubShotPredictionLaunchLeadSeconds));
    calibrationModeEnabled = calibrationModeEnabledEntry.getBoolean(calibrationModeEnabled);
    if (calibrationModeEnabled) {
      manualHoodOverrideEnabled = false;
      manualWheelOverrideEnabled = false;
    }
    calibrationHoodSetpointRotations =
        clampToHoodCalibrationRange(
            sanitizeFinite(
                calibrationHoodSetpointRotationsEntry.getDouble(calibrationHoodSetpointRotations),
                calibrationHoodSetpointRotations));
    calibrationPair1WheelSetpointRadPerSec =
        clampCalibrationWheelSpeedRadPerSec(
            sanitizeFinite(
                calibrationPair1WheelSetpointEntry.getDouble(
                    calibrationPair1WheelSetpointRadPerSec),
                calibrationPair1WheelSetpointRadPerSec));
    calibrationPair2WheelSetpointRadPerSec =
        clampCalibrationWheelSpeedRadPerSec(
            sanitizeFinite(
                calibrationPair2WheelSetpointEntry.getDouble(
                    calibrationPair2WheelSetpointRadPerSec),
                calibrationPair2WheelSetpointRadPerSec));
    WheelSpeedSetpoints normalizedCalibrationWheelSetpoints =
        sharedCalibrationWheelSetpoints(
            calibrationPair1WheelSetpointRadPerSec, calibrationPair2WheelSetpointRadPerSec);
    calibrationPair1WheelSetpointRadPerSec = normalizedCalibrationWheelSetpoints.pair1RadPerSec();
    calibrationPair2WheelSetpointRadPerSec = normalizedCalibrationWheelSetpoints.pair2RadPerSec();
    calibrationMeasuredDistanceMeters =
        Math.max(
            0.0,
            sanitizeFinite(
                calibrationDistanceEntry.getDouble(calibrationMeasuredDistanceMeters),
                calibrationMeasuredDistanceMeters));
    calibrationMeasuredHeightDeltaMeters =
        sanitizeFinite(
            calibrationHeightDeltaEntry.getDouble(calibrationMeasuredHeightDeltaMeters),
            calibrationMeasuredHeightDeltaMeters);
    calibrationMeasuredAirtimeSeconds =
        sanitizeFiniteOrNaN(
            calibrationAirtimeEntry.getDouble(calibrationMeasuredAirtimeSeconds),
            calibrationMeasuredAirtimeSeconds);
    calibrationVideoLaunchAngleDegrees =
        sanitizeFiniteOrNaN(
            calibrationVideoLaunchAngleEntry.getDouble(calibrationVideoLaunchAngleDegrees),
            calibrationVideoLaunchAngleDegrees);
    calibrationNotes = calibrationNotesEntry.getString(calibrationNotes);
    wheelSpeedScaleEntry.setDouble(wheelSpeedScale);
    pair1DirectionEntry.setDouble(pair1Direction);
    pair2DirectionEntry.setDouble(pair2Direction);
    hubAimHeightOffsetEntry.setDouble(hubAimHeightOffsetMeters);
    wheelVelocityKpEntry.setDouble(wheelVelocityKp);
    wheelVelocityKiEntry.setDouble(wheelVelocityKi);
    wheelVelocityKdEntry.setDouble(wheelVelocityKd);
    wheelVelocityKvEntry.setDouble(wheelVelocityKv);
    hoodPositionKpEntry.setDouble(hoodPositionKp);
    hoodPositionKiEntry.setDouble(hoodPositionKi);
    hoodPositionKdEntry.setDouble(hoodPositionKd);
    hubMotionCompVelocityScaleEntry.setDouble(hubMotionCompVelocityScale);
    hubMotionCompLeadSecondsEntry.setDouble(hubMotionCompLeadSeconds);
    hubShotPredictionLaunchLeadSecondsEntry.setDouble(hubShotPredictionLaunchLeadSeconds);
    calibrationModeEnabledEntry.setBoolean(calibrationModeEnabled);
    calibrationHoodSetpointRotationsEntry.setDouble(calibrationHoodSetpointRotations);
    calibrationPair1WheelSetpointEntry.setDouble(calibrationPair1WheelSetpointRadPerSec);
    calibrationPair2WheelSetpointEntry.setDouble(calibrationPair2WheelSetpointRadPerSec);
    calibrationDistanceEntry.setDouble(calibrationMeasuredDistanceMeters);
    calibrationHeightDeltaEntry.setDouble(calibrationMeasuredHeightDeltaMeters);
    calibrationAirtimeEntry.setDouble(calibrationMeasuredAirtimeSeconds);
    calibrationVideoLaunchAngleEntry.setDouble(calibrationVideoLaunchAngleDegrees);
    calibrationNotesEntry.setString(calibrationNotes);
  }

  private static double sanitizeFinite(double value, double fallback) {
    return Double.isFinite(value) ? value : fallback;
  }

  private static double sanitizeFiniteOrNaN(double value, double fallback) {
    return Double.isNaN(value) || Double.isFinite(value) ? value : fallback;
  }

  private void publishHoodEncoderToNetworkTables() {
    double appliedHoodSetpointRotations = getAppliedHoodSetpointMotorRotations();
    Rotation2d setpointAngle = motorRotationsToHoodAngle(appliedHoodSetpointRotations);
    Rotation2d measuredAngle = motorRotationsToHoodAngle(inputs.hoodPositionRotations);
    hoodEncoderPositionEntry.setDouble(inputs.hoodPositionRotations);
    hoodEncoderVelocityEntry.setDouble(inputs.hoodVelocityRotationsPerSec);
    hoodEncoderNormalizedPositionEntry.setDouble(
        getHoodNormalizedPosition(inputs.hoodPositionRotations));
    hoodSetpointEntry.setDouble(appliedHoodSetpointRotations);
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
    Rotation2d setpointHoodAngle =
        motorRotationsToHoodAngle(getAppliedHoodSetpointMotorRotations());
    Rotation2d measuredHoodAngle = motorRotationsToHoodAngle(inputs.hoodPositionRotations);

    double setpointLaunchSpeedMetersPerSec =
        estimateLaunchSpeedFromWheelVelocitiesMetersPerSec(
            pair1VelocityCommandRadPerSec, pair2VelocityCommandRadPerSec);
    double measuredLaunchSpeedMetersPerSec =
        getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec();

    double targetDistanceMeters =
        Math.max(
            0.0,
            calibrationModeEnabled
                ? calibrationMeasuredDistanceMeters
                : latestHubShotSolution.distanceMeters());
    double targetHeightMeters =
        calibrationModeEnabled
            ? launchHeightMeters + calibrationMeasuredHeightDeltaMeters
            : getHubAimHeightMeters();
    Rotation2d solutionAngle =
        calibrationModeEnabled
            ? motorRotationsToHoodAngle(calibrationHoodSetpointRotations)
            : latestHubShotSolution.launchAngle();

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
    shotOverlayHubHeightEntry.setDouble(targetHeightMeters);
    shotOverlayLaunchHeightEntry.setDouble(launchHeightMeters);
    shotOverlaySolutionAngleEntry.setDouble(solutionAngle.getDegrees());
    shotOverlaySetpointLaunchSpeedEntry.setDouble(setpointLaunchSpeedMetersPerSec);
    shotOverlayMeasuredLaunchSpeedEntry.setDouble(measuredLaunchSpeedMetersPerSec);
    shotOverlaySetpointHeightEntry.setDouble(optionalDoubleOrNaN(setpointHeightAtTargetMeters));
    shotOverlayMeasuredHeightEntry.setDouble(optionalDoubleOrNaN(measuredHeightAtTargetMeters));
    shotOverlaySetpointClearanceEntry.setDouble(
        optionalClearanceToTargetHeightOrNaN(setpointHeightAtTargetMeters, targetHeightMeters));
    shotOverlayMeasuredClearanceEntry.setDouble(
        optionalClearanceToTargetHeightOrNaN(measuredHeightAtTargetMeters, targetHeightMeters));
    shotOverlayMinAngleClearanceEntry.setDouble(
        optionalClearanceToTargetHeightOrNaN(minAngleHeightAtTargetMeters, targetHeightMeters));
    shotOverlayMaxAngleClearanceEntry.setDouble(
        optionalClearanceToTargetHeightOrNaN(maxAngleHeightAtTargetMeters, targetHeightMeters));
  }

  private void publishCalibrationTelemetry() {
    calibrationModeTelemetryEntry.setBoolean(calibrationModeEnabled);
    calibrationConfiguredHoodSetpointEntry.setDouble(calibrationHoodSetpointRotations);
    calibrationConfiguredHoodAngleEntry.setDouble(
        motorRotationsToHoodAngle(calibrationHoodSetpointRotations).getDegrees());
    calibrationConfiguredPair1WheelSetpointEntry.setDouble(calibrationPair1WheelSetpointRadPerSec);
    calibrationConfiguredPair2WheelSetpointEntry.setDouble(calibrationPair2WheelSetpointRadPerSec);
    calibrationConfiguredDistanceEntry.setDouble(calibrationMeasuredDistanceMeters);
    calibrationConfiguredHeightDeltaEntry.setDouble(calibrationMeasuredHeightDeltaMeters);
    calibrationConfiguredAirtimeEntry.setDouble(calibrationMeasuredAirtimeSeconds);
    calibrationConfiguredVideoLaunchAngleEntry.setDouble(calibrationVideoLaunchAngleDegrees);
    calibrationConfiguredNotesEntry.setString(calibrationNotes);
    calibrationRecordedSampleCountEntry.setDouble(calibrationRecordedSampleCount);
    calibrationLastRecordedTimestampEntry.setDouble(calibrationLastRecordedTimestampSeconds);
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

  private void refreshPredictedHubShot() {
    latestPredictedHubShot =
        predictCurrentHubShot(
            latestHubRobotPose, latestHubPose, latestRobotFieldVelocityMetersPerSec);
  }

  private void updateRobotMotionState(Translation2d robotFieldVelocityMetersPerSec) {
    double nowSeconds = Timer.getFPGATimestamp();
    Translation2d measuredAccelerationMetersPerSecSquared = new Translation2d();
    if (Double.isFinite(lastHubMotionSampleTimestampSeconds)) {
      double dtSeconds = nowSeconds - lastHubMotionSampleTimestampSeconds;
      double effectiveDtSeconds =
          Double.isFinite(dtSeconds) && dtSeconds >= (0.5 * loopPeriodSeconds) && dtSeconds <= 0.1
              ? dtSeconds
              : loopPeriodSeconds;
      measuredAccelerationMetersPerSecSquared =
          clampVectorMagnitude(
              robotFieldVelocityMetersPerSec
                  .minus(latestRobotFieldVelocityMetersPerSec)
                  .times(1.0 / effectiveDtSeconds),
              ShooterConstants.hubPredictionMaxRobotAccelerationMetersPerSecSquared);
    }
    latestRobotFieldAccelerationMetersPerSecSquared = measuredAccelerationMetersPerSecSquared;
    lastHubMotionSampleTimestampSeconds = nowSeconds;
  }

  private PredictedHubShot predictCurrentHubShot(
      Pose2d robotPose, Pose2d hubPose, Translation2d robotFieldVelocityMetersPerSec) {
    Translation2d launchOrigin =
        getPredictedLaunchOriginFieldPosition(
            robotPose,
            robotFieldVelocityMetersPerSec,
            latestRobotFieldAccelerationMetersPerSecSquared,
            hubShotPredictionLaunchLeadSeconds);
    Translation2d projectedRobotFieldVelocityAtLaunch =
        getPredictedRobotFieldVelocityAtLaunch(
            robotFieldVelocityMetersPerSec,
            latestRobotFieldAccelerationMetersPerSecSquared,
            hubShotPredictionLaunchLeadSeconds);
    Rotation2d launchYaw = robotPose.getRotation().plus(ShooterConstants.shooterFacingOffset);
    Rotation2d launchPitch = getMeasuredHoodAngle();
    double launchSpeedMetersPerSec =
        MathUtil.clamp(
            getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec(),
            0.0,
            ShooterConstants.maxLaunchSpeedMetersPerSec);
    double horizontalLaunchSpeedMetersPerSec =
        launchSpeedMetersPerSec * Math.cos(launchPitch.getRadians());
    Translation2d projectileFieldVelocityMetersPerSec =
        projectedRobotFieldVelocityAtLaunch.plus(
            new Translation2d(horizontalLaunchSpeedMetersPerSec, launchYaw));
    double horizontalVelocityMagnitudeSquared =
        projectileFieldVelocityMetersPerSec.getNorm()
            * projectileFieldVelocityMetersPerSec.getNorm();
    if (horizontalVelocityMagnitudeSquared
        < (ShooterConstants.minHorizontalVelocityMetersPerSec
            * ShooterConstants.minHorizontalVelocityMetersPerSec)) {
      return new PredictedHubShot(false, Double.NaN, Double.NaN, Double.NaN, false);
    }

    Translation2d vectorToHub = hubPose.getTranslation().minus(launchOrigin);
    // Evaluate the measured shot at the point of closest horizontal approach to the hub center.
    double evaluationTimeSeconds =
        Math.max(
                0.0,
                vectorToHub.getX() * projectileFieldVelocityMetersPerSec.getX()
                    + vectorToHub.getY() * projectileFieldVelocityMetersPerSec.getY())
            / horizontalVelocityMagnitudeSquared;
    if (!Double.isFinite(evaluationTimeSeconds)
        || evaluationTimeSeconds > ShooterConstants.maxAirtimeSeconds) {
      return new PredictedHubShot(false, Double.NaN, Double.NaN, Double.NaN, false);
    }

    Translation2d predictedFieldPosition =
        launchOrigin.plus(projectileFieldVelocityMetersPerSec.times(evaluationTimeSeconds));
    double predictedHeightMeters =
        launchHeightMeters
            + (launchSpeedMetersPerSec * Math.sin(launchPitch.getRadians()) * evaluationTimeSeconds)
            - (0.5
                * ShooterConstants.gravityMetersPerSecSquared
                * evaluationTimeSeconds
                * evaluationTimeSeconds);
    double targetHeightMeters = getHubAimHeightMeters();
    double horizontalErrorMeters = predictedFieldPosition.getDistance(hubPose.getTranslation());
    double verticalErrorMeters = predictedHeightMeters - targetHeightMeters;
    double verticalVelocityMetersPerSec =
        (launchSpeedMetersPerSec * Math.sin(launchPitch.getRadians()))
            - (ShooterConstants.gravityMetersPerSecSquared * evaluationTimeSeconds);
    boolean descendingAtTarget =
        verticalVelocityMetersPerSec <= -ShooterConstants.hubTopEntryMinDescentVelocityMetersPerSec;
    boolean scoring =
        horizontalErrorMeters <= ShooterConstants.hubPredictedScoreToleranceXYMeters
            && Math.abs(verticalErrorMeters) <= ShooterConstants.hubPredictedScoreToleranceZMeters
            && descendingAtTarget;
    return new PredictedHubShot(
        scoring,
        horizontalErrorMeters,
        verticalErrorMeters,
        evaluationTimeSeconds,
        descendingAtTarget);
  }

  private Translation2d getPredictedLaunchOriginFieldPosition(
      Pose2d robotPose,
      Translation2d robotFieldVelocityMetersPerSec,
      Translation2d robotFieldAccelerationMetersPerSecSquared,
      double leadSeconds) {
    Translation2d launchOrigin = HubTargetingGeometry.getLaunchOriginFieldPosition(robotPose);
    double clampedLeadSeconds = clampPredictionLaunchLeadSeconds(leadSeconds);
    return launchOrigin
        .plus(robotFieldVelocityMetersPerSec.times(clampedLeadSeconds))
        .plus(
            robotFieldAccelerationMetersPerSecSquared.times(
                0.5 * clampedLeadSeconds * clampedLeadSeconds));
  }

  private Translation2d getPredictedRobotFieldVelocityAtLaunch(
      Translation2d robotFieldVelocityMetersPerSec,
      Translation2d robotFieldAccelerationMetersPerSecSquared,
      double leadSeconds) {
    double clampedLeadSeconds = clampPredictionLaunchLeadSeconds(leadSeconds);
    return robotFieldVelocityMetersPerSec.plus(
        robotFieldAccelerationMetersPerSecSquared.times(clampedLeadSeconds));
  }

  private void clampHoodSetpointToCalibrationRange() {
    hoodSetpointMotorRotations = clampToHoodCalibrationRange(hoodSetpointMotorRotations);
  }

  private boolean canApplyHoodSetpointWithoutHoming() {
    return hoodHomed
        || Constants.currentMode != Constants.Mode.REAL
        || manualHoodJogAllowedBeforeHoming;
  }

  private void prepareForManualHoodJog() {
    manualHoodOverrideEnabled = true;
    if (!hoodHomed && Constants.currentMode == Constants.Mode.REAL) {
      if (!manualHoodJogAllowedBeforeHoming) {
        hoodSetpointMotorRotations = clampToHoodCalibrationRange(inputs.hoodPositionRotations);
      }
      manualHoodJogAllowedBeforeHoming = true;
    }
  }

  private void startHoodHoming() {
    hoodHomingActive = true;
    hoodHomed = false;
    hoodHomingSucceeded = false;
    manualHoodJogAllowedBeforeHoming = false;
    hoodHomingPhase = HoodHomingPhase.RELAX_BEFORE_CALIBRATION;
    hoodHomingElapsedSeconds = 0.0;
    hoodHomingPhaseElapsedSeconds = 0.0;
    hoodHomingStallSeconds = 0.0;
    hoodHomingRetractedHardStopRotations = inputs.hoodPositionRotations;
    hoodHomingExtendedHardStopRotations = inputs.hoodPositionRotations;
  }

  private void cancelHoodHoming() {
    hoodHomingActive = false;
    hoodHomingElapsedSeconds = 0.0;
    hoodHomingPhaseElapsedSeconds = 0.0;
    hoodHomingStallSeconds = 0.0;
    io.setHoodOpenLoopOutput(0.0);
  }

  private void updateHoodHoming() {
    hoodHomingElapsedSeconds += loopPeriodSeconds;
    hoodHomingPhaseElapsedSeconds += loopPeriodSeconds;
    if (hoodHomingPhase == HoodHomingPhase.RELAX_BEFORE_CALIBRATION) {
      io.setHoodOpenLoopOutput(0.0);
      if (hoodHomingPhaseElapsedSeconds
          >= ShooterConstants.hoodHomingRelaxBeforeCalibrationSeconds) {
        hoodHomingPhase = HoodHomingPhase.SEEK_RETRACTED_HARD_STOP;
        hoodHomingPhaseElapsedSeconds = 0.0;
        hoodHomingStallSeconds = 0.0;
      }
      return;
    }

    if (hoodHomingPhase == HoodHomingPhase.SEEK_RETRACTED_HARD_STOP) {
      io.setHoodOpenLoopOutput(ShooterConstants.hoodHomingOutputTowardRetractedHardStop);
    } else {
      io.setHoodOpenLoopOutput(ShooterConstants.hoodHomingOutputTowardExtendedHardStop);
    }

    boolean velocityNearZero =
        Math.abs(inputs.hoodVelocityRotationsPerSec)
            <= ShooterConstants.hoodHomingMaxVelocityRotationsPerSec;
    boolean currentHigh =
        Math.abs(inputs.hoodCurrentAmps) >= ShooterConstants.hoodHomingMinCurrentAmps;
    if (velocityNearZero && currentHigh) {
      hoodHomingStallSeconds += loopPeriodSeconds;
    } else {
      hoodHomingStallSeconds = 0.0;
    }

    if (hoodHomingStallSeconds >= ShooterConstants.hoodHomingStallConfirmSeconds) {
      if (hoodHomingPhase == HoodHomingPhase.SEEK_RETRACTED_HARD_STOP) {
        io.setHoodEncoderPositionRotations(
            ShooterConstants.hoodRetractedHardStopReferenceRotations);
        hoodHomingRetractedHardStopRotations =
            ShooterConstants.hoodRetractedHardStopReferenceRotations;
        hoodHomingPhase = HoodHomingPhase.SEEK_EXTENDED_HARD_STOP;
        hoodHomingPhaseElapsedSeconds = 0.0;
        hoodHomingStallSeconds = 0.0;
        return;
      }

      hoodHomingExtendedHardStopRotations = inputs.hoodPositionRotations;
      double measuredTravelRotations =
          Math.abs(hoodHomingExtendedHardStopRotations - hoodHomingRetractedHardStopRotations);
      finishHoodHoming(measuredTravelRotations >= ShooterConstants.hoodHomingMinTravelRotations);
      return;
    }

    if (hoodHomingPhaseElapsedSeconds >= ShooterConstants.hoodHomingTimeoutSeconds) {
      finishHoodHoming(false);
    }
  }

  private void finishHoodHoming(boolean success) {
    hoodHomingActive = false;
    hoodHomingSucceeded = success;
    manualHoodJogAllowedBeforeHoming = false;
    io.setHoodOpenLoopOutput(0.0);
    if (!success) {
      return;
    }

    hoodHomed = true;
    setHoodCalibrationFromHardStops(
        hoodHomingRetractedHardStopRotations, hoodHomingExtendedHardStopRotations);
    hoodSetpointMotorRotations = hoodExtendedPositionRotations;
    manualHoodOverrideEnabled = false;
  }

  private void setHoodCalibrationFromHardStops(
      double retractedHardStopPositionRotations, double extendedHardStopPositionRotations) {
    hoodRetractedPositionRotations = retractedHardStopPositionRotations;
    hoodExtendedPositionRotations = extendedHardStopPositionRotations;
    hoodExtendedPositionEntry.setDouble(hoodExtendedPositionRotations);
    hoodRetractedPositionEntry.setDouble(hoodRetractedPositionRotations);
  }

  public void recordCalibrationSample() {
    calibrationRecordedSampleCount++;
    calibrationLastRecordedTimestampSeconds = Timer.getFPGATimestamp();
    calibrationRecordedSampleCountEntry.setDouble(calibrationRecordedSampleCount);
    Rotation2d hoodSetpointAngle = motorRotationsToHoodAngle(calibrationHoodSetpointRotations);
    Rotation2d measuredHoodAngle = motorRotationsToHoodAngle(inputs.hoodPositionRotations);
    double estimatedLaunchSpeedFromMeasuredWheelsMetersPerSec =
        getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec();
    CalibrationDerivedMeasurement derivedMeasurement =
        deriveCalibrationMeasurement(
            calibrationMeasuredDistanceMeters,
            calibrationMeasuredHeightDeltaMeters,
            calibrationMeasuredAirtimeSeconds);

    calibrationLastRecordedTimestampEntry.setDouble(calibrationLastRecordedTimestampSeconds);
    calibrationLastRecordedModeEnabledEntry.setBoolean(calibrationModeEnabled);
    calibrationLastRecordedHoodSetpointEntry.setDouble(calibrationHoodSetpointRotations);
    calibrationLastRecordedHoodSetpointAngleEntry.setDouble(hoodSetpointAngle.getDegrees());
    calibrationLastRecordedMeasuredHoodPositionEntry.setDouble(inputs.hoodPositionRotations);
    calibrationLastRecordedMeasuredHoodAngleEntry.setDouble(measuredHoodAngle.getDegrees());
    calibrationLastRecordedPair1SetpointEntry.setDouble(calibrationPair1WheelSetpointRadPerSec);
    calibrationLastRecordedPair2SetpointEntry.setDouble(calibrationPair2WheelSetpointRadPerSec);
    calibrationLastRecordedPair1MeasuredEntry.setDouble(inputs.pair1LeaderVelocityRadPerSec);
    calibrationLastRecordedPair2MeasuredEntry.setDouble(inputs.pair2LeaderVelocityRadPerSec);
    calibrationLastRecordedEstimatedLaunchSpeedEntry.setDouble(
        estimatedLaunchSpeedFromMeasuredWheelsMetersPerSec);
    calibrationLastRecordedDistanceEntry.setDouble(calibrationMeasuredDistanceMeters);
    calibrationLastRecordedHeightDeltaEntry.setDouble(calibrationMeasuredHeightDeltaMeters);
    calibrationLastRecordedAirtimeEntry.setDouble(calibrationMeasuredAirtimeSeconds);
    calibrationLastRecordedVideoLaunchAngleEntry.setDouble(calibrationVideoLaunchAngleDegrees);
    calibrationLastRecordedNotesEntry.setString(calibrationNotes);
    calibrationLastRecordedDerivedValidEntry.setBoolean(derivedMeasurement.valid());
    calibrationLastRecordedHorizontalVelocityEntry.setDouble(
        derivedMeasurement.horizontalVelocityMetersPerSec());
    calibrationLastRecordedVerticalVelocityEntry.setDouble(
        derivedMeasurement.verticalVelocityMetersPerSec());
    calibrationLastRecordedDerivedLaunchSpeedEntry.setDouble(
        derivedMeasurement.launchSpeedMetersPerSec());
    calibrationLastRecordedDerivedLaunchAngleEntry.setDouble(
        derivedMeasurement.launchAngle().getDegrees());

    String calibrationLogRoot =
        NetworkTablesUtil.logPath("GamePiece/Shooter/Calibration/LastRecorded");
    Logger.recordOutput(calibrationLogRoot + "/SampleCount", calibrationRecordedSampleCount);
    Logger.recordOutput(
        calibrationLogRoot + "/TimestampSeconds", calibrationLastRecordedTimestampSeconds);
    Logger.recordOutput(calibrationLogRoot + "/ModeEnabled", calibrationModeEnabled);
    Logger.recordOutput(
        calibrationLogRoot + "/HoodSetpointRotations", calibrationHoodSetpointRotations);
    Logger.recordOutput(
        calibrationLogRoot + "/HoodSetpointDegrees", hoodSetpointAngle.getDegrees());
    Logger.recordOutput(
        calibrationLogRoot + "/HoodMeasuredRotations", inputs.hoodPositionRotations);
    Logger.recordOutput(
        calibrationLogRoot + "/HoodMeasuredDegrees", measuredHoodAngle.getDegrees());
    Logger.recordOutput(
        calibrationLogRoot + "/Wheels/Pair1SetpointRadPerSec",
        calibrationPair1WheelSetpointRadPerSec);
    Logger.recordOutput(
        calibrationLogRoot + "/Wheels/Pair2SetpointRadPerSec",
        calibrationPair2WheelSetpointRadPerSec);
    Logger.recordOutput(
        calibrationLogRoot + "/Wheels/Pair1MeasuredRadPerSec", inputs.pair1LeaderVelocityRadPerSec);
    Logger.recordOutput(
        calibrationLogRoot + "/Wheels/Pair2MeasuredRadPerSec", inputs.pair2LeaderVelocityRadPerSec);
    Logger.recordOutput(
        calibrationLogRoot + "/Wheels/EstimatedLaunchSpeedFromMeasuredMetersPerSec",
        estimatedLaunchSpeedFromMeasuredWheelsMetersPerSec);
    Logger.recordOutput(
        calibrationLogRoot + "/Measurement/DistanceMeters", calibrationMeasuredDistanceMeters);
    Logger.recordOutput(
        calibrationLogRoot + "/Measurement/HeightDeltaMeters",
        calibrationMeasuredHeightDeltaMeters);
    Logger.recordOutput(
        calibrationLogRoot + "/Measurement/AirtimeSeconds", calibrationMeasuredAirtimeSeconds);
    Logger.recordOutput(
        calibrationLogRoot + "/Measurement/VideoLaunchAngleDegrees",
        calibrationVideoLaunchAngleDegrees);
    Logger.recordOutput(calibrationLogRoot + "/Measurement/Notes", calibrationNotes);
    Logger.recordOutput(calibrationLogRoot + "/Derived/Valid", derivedMeasurement.valid());
    Logger.recordOutput(
        calibrationLogRoot + "/Derived/HorizontalVelocityMetersPerSec",
        derivedMeasurement.horizontalVelocityMetersPerSec());
    Logger.recordOutput(
        calibrationLogRoot + "/Derived/VerticalVelocityMetersPerSec",
        derivedMeasurement.verticalVelocityMetersPerSec());
    Logger.recordOutput(
        calibrationLogRoot + "/Derived/LaunchSpeedMetersPerSec",
        derivedMeasurement.launchSpeedMetersPerSec());
    Logger.recordOutput(
        calibrationLogRoot + "/Derived/LaunchAngleDegrees",
        derivedMeasurement.launchAngle().getDegrees());
  }

  private HubShotSolution createCalibrationModeShotSolution() {
    Rotation2d launchAngle = motorRotationsToHoodAngle(calibrationHoodSetpointRotations);
    double launchSpeedMetersPerSec =
        estimateLaunchSpeedFromWheelVelocitiesMetersPerSec(
            calibrationPair1WheelSetpointRadPerSec, calibrationPair2WheelSetpointRadPerSec);
    double airtimeSeconds =
        (Double.isFinite(calibrationMeasuredAirtimeSeconds)
                && calibrationMeasuredAirtimeSeconds > 0.0)
            ? clampAirtime(calibrationMeasuredAirtimeSeconds)
            : calculateAirtimeSeconds(
                calibrationMeasuredDistanceMeters,
                calibrationMeasuredHeightDeltaMeters,
                launchSpeedMetersPerSec,
                launchAngle);
    return new HubShotSolution(
        calibrationMeasuredDistanceMeters,
        launchAngle,
        launchSpeedMetersPerSec,
        speedToPower(launchSpeedMetersPerSec),
        airtimeSeconds,
        true);
  }

  private CalibrationDerivedMeasurement deriveCalibrationMeasurement(
      double distanceMeters, double heightDeltaMeters, double airtimeSeconds) {
    if (!(distanceMeters > 0.0) || !(airtimeSeconds > 0.0) || !Double.isFinite(heightDeltaMeters)) {
      return invalidCalibrationDerivedMeasurement();
    }

    double horizontalVelocityMetersPerSec = distanceMeters / airtimeSeconds;
    double verticalVelocityMetersPerSec =
        (heightDeltaMeters
                + (0.5
                    * ShooterConstants.gravityMetersPerSecSquared
                    * airtimeSeconds
                    * airtimeSeconds))
            / airtimeSeconds;
    double launchSpeedMetersPerSec =
        Math.hypot(horizontalVelocityMetersPerSec, verticalVelocityMetersPerSec);
    if (!Double.isFinite(horizontalVelocityMetersPerSec)
        || !Double.isFinite(verticalVelocityMetersPerSec)
        || !Double.isFinite(launchSpeedMetersPerSec)
        || launchSpeedMetersPerSec <= 0.0) {
      return invalidCalibrationDerivedMeasurement();
    }

    return new CalibrationDerivedMeasurement(
        horizontalVelocityMetersPerSec,
        verticalVelocityMetersPerSec,
        launchSpeedMetersPerSec,
        Rotation2d.fromRadians(
            Math.atan2(verticalVelocityMetersPerSec, horizontalVelocityMetersPerSec)),
        true);
  }

  private CalibrationDerivedMeasurement invalidCalibrationDerivedMeasurement() {
    return new CalibrationDerivedMeasurement(
        Double.NaN, Double.NaN, Double.NaN, Rotation2d.fromDegrees(Double.NaN), false);
  }

  private MotionCompensatedHubShotSolution solveHubShotWithMotionCompensation(
      Pose2d robotPose,
      Pose2d hubPose,
      Translation2d robotFieldVelocityMetersPerSec,
      double targetHeightDeltaMeters) {
    double airtimeGuessSeconds = sanitizeAirtimeSeed(latestHubShotSolution.airtimeSeconds());
    Pose2d compensatedHubPose = hubPose;
    Translation2d launchOrigin = HubTargetingGeometry.getLaunchOriginFieldPosition(robotPose);
    HubShotSolution solvedShot =
        solveHubShot(launchOrigin.getDistance(hubPose.getTranslation()), targetHeightDeltaMeters);
    int iterations = 0;
    boolean converged = false;

    for (int i = 0; i < ShooterConstants.hubMotionCompensationMaxIterations; i++) {
      compensatedHubPose =
          getCompensatedHubPose(hubPose, robotFieldVelocityMetersPerSec, airtimeGuessSeconds);
      double compensatedDistanceMeters =
          launchOrigin.getDistance(compensatedHubPose.getTranslation());
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
          launchOrigin.getDistance(compensatedHubPose.getTranslation());
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

  private static double launchSpeedToAverageWheelSurfaceSpeedMetersPerSec(
      double launchSpeedMetersPerSec) {
    return launchSpeedMetersPerSec / ShooterConstants.launchSpeedFromWheelSurfaceSpeedScale;
  }

  private static double averageWheelSurfaceSpeedToLaunchSpeedMetersPerSec(
      double averageWheelSurfaceSpeedMetersPerSec) {
    return ShooterConstants.launchSpeedFromWheelSurfaceSpeedScale
        * averageWheelSurfaceSpeedMetersPerSec;
  }

  private static double sanitizeAirtimeSeed(double airtimeSeconds) {
    if (!Double.isFinite(airtimeSeconds) || airtimeSeconds <= 0.0) {
      return ShooterConstants.fallbackAirtimeSeconds;
    }
    return clampAirtime(airtimeSeconds);
  }

  private void applyHubShotSetpoints(HubShotSolution solution) {
    if (calibrationModeEnabled) {
      hoodSetpointMotorRotations = calibrationHoodSetpointRotations;
      pair1WheelSetpointRadPerSec = calibrationPair1WheelSetpointRadPerSec;
      pair2WheelSetpointRadPerSec = calibrationPair2WheelSetpointRadPerSec;
      return;
    }

    if (!manualHoodOverrideEnabled) {
      Rotation2d clampedHoodAngle =
          new Rotation2d(
              MathUtil.clamp(
                  solution.launchAngle().getRadians(),
                  ShooterConstants.minLaunchAngle.getRadians(),
                  ShooterConstants.maxLaunchAngle.getRadians()));
      hoodSetpointMotorRotations = hoodAngleToMotorRotations(clampedHoodAngle);
    }

    if (!manualWheelOverrideEnabled) {
      WheelSpeedSetpoints wheelSetpoints =
          calculateWheelSetpointsFromLaunchSpeed(solution.launchSpeedMetersPerSec());
      pair1WheelSetpointRadPerSec = wheelSetpoints.pair1RadPerSec();
      pair2WheelSetpointRadPerSec = wheelSetpoints.pair2RadPerSec();
    }
  }

  private WheelSpeedSetpoints calculateWheelSetpointsFromLaunchSpeed(
      double launchSpeedMetersPerSec) {
    double launchSpeedMagnitude = Math.abs(launchSpeedMetersPerSec);
    double baseWheelSurfaceSpeedMetersPerSec =
        launchSpeedToAverageWheelSurfaceSpeedMetersPerSec(launchSpeedMagnitude);
    // This mechanism is a single shared drum, so both logical pair setpoints must match.
    double sharedWheelSpeedRadPerSec =
        baseWheelSurfaceSpeedMetersPerSec / ShooterConstants.shooterWheelRadiusMeters;
    return sharedWheelSetpoints(sharedWheelSpeedRadPerSec, sharedWheelSpeedRadPerSec);
  }

  private static double clampWheelSpeedRadPerSec(double wheelSpeedRadPerSec) {
    return MathUtil.clamp(
        wheelSpeedRadPerSec,
        ShooterConstants.minWheelSpeedRadPerSec,
        ShooterConstants.maxWheelSpeedRadPerSec);
  }

  private static WheelSpeedSetpoints sharedWheelSetpoints(
      double pair1WheelSpeedRadPerSec, double pair2WheelSpeedRadPerSec) {
    double sharedWheelSpeedRadPerSec =
        clampWheelSpeedRadPerSec(
            resolveSharedWheelSetpoint(pair1WheelSpeedRadPerSec, pair2WheelSpeedRadPerSec));
    return new WheelSpeedSetpoints(sharedWheelSpeedRadPerSec, sharedWheelSpeedRadPerSec);
  }

  private static double clampCalibrationWheelSpeedRadPerSec(double wheelSpeedRadPerSec) {
    return MathUtil.clamp(
        Math.abs(wheelSpeedRadPerSec), 0.0, ShooterConstants.maxWheelSpeedRadPerSec);
  }

  private static WheelSpeedSetpoints sharedCalibrationWheelSetpoints(
      double pair1WheelSpeedRadPerSec, double pair2WheelSpeedRadPerSec) {
    double sharedWheelSpeedRadPerSec =
        clampCalibrationWheelSpeedRadPerSec(
            resolveSharedWheelSetpoint(pair1WheelSpeedRadPerSec, pair2WheelSpeedRadPerSec));
    return new WheelSpeedSetpoints(sharedWheelSpeedRadPerSec, sharedWheelSpeedRadPerSec);
  }

  private static double resolveSharedWheelSetpoint(double pair1RadPerSec, double pair2RadPerSec) {
    if (Math.abs(pair1RadPerSec) <= 1e-9) {
      return pair2RadPerSec;
    }
    if (Math.abs(pair2RadPerSec) <= 1e-9) {
      return pair1RadPerSec;
    }
    return 0.5 * (pair1RadPerSec + pair2RadPerSec);
  }

  private static double clampWheelSpeedScale(double speedScale) {
    return MathUtil.clamp(speedScale, 0.0, 1.5);
  }

  private static double clampOperatorWheelThrottleScale(double throttleScale) {
    return MathUtil.clamp(throttleScale, 0.0, 1.0);
  }

  private static Translation2d clampVectorMagnitude(Translation2d vector, double maxMagnitude) {
    if (!Double.isFinite(maxMagnitude) || maxMagnitude <= 0.0) {
      return new Translation2d();
    }
    double vectorMagnitude = vector.getNorm();
    if (!Double.isFinite(vectorMagnitude)) {
      return new Translation2d();
    }
    if (vectorMagnitude <= maxMagnitude) {
      return vector;
    }
    return vector.times(maxMagnitude / vectorMagnitude);
  }

  private static double clampMotionCompVelocityScale(double velocityScale) {
    return MathUtil.clamp(velocityScale, 0.0, 3.0);
  }

  private static double clampMotionCompLeadSeconds(double leadSeconds) {
    return MathUtil.clamp(leadSeconds, -0.25, 0.25);
  }

  private static double clampPredictionLaunchLeadSeconds(double leadSeconds) {
    return MathUtil.clamp(leadSeconds, 0.0, 0.25);
  }

  private static double normalizeDirection(double direction) {
    return direction < 0.0 ? -1.0 : 1.0;
  }

  public double hoodAngleToMotorRotations(Rotation2d hoodAngle) {
    double normalizedAngle =
        clampUnitInterval(
            inverseInterpolate(
                hoodAngle.getDegrees(),
                ShooterConstants.maxLaunchAngle.getDegrees(),
                ShooterConstants.minLaunchAngle.getDegrees()));
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
            ShooterConstants.maxLaunchAngle.getDegrees(),
            ShooterConstants.minLaunchAngle.getDegrees(),
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

  public boolean isSpinupComplete() {
    return shotControlEnabled
        && !hoodHomingActive
        && inputs.pair1Connected
        && inputs.pair2Connected
        && inputs.hoodConnected
        && areWheelsAtSpeedForShot();
  }

  public boolean isReadyToFire() {
    return isSpinupComplete() && isHubShotPredictedToScore();
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

  public void resetSimulationState() {
    lastSimShotTimestampSeconds = Double.NEGATIVE_INFINITY;
    pair1VelocityCommandRadPerSec = 0.0;
    pair2VelocityCommandRadPerSec = 0.0;
    operatorWheelThrottleScale = 1.0;
    hoodHomingActive = false;
    hoodHomed = false;
    hoodHomingSucceeded = false;
    hoodHomingPhase = HoodHomingPhase.RELAX_BEFORE_CALIBRATION;
    hoodHomingElapsedSeconds = 0.0;
    hoodHomingPhaseElapsedSeconds = 0.0;
    hoodHomingStallSeconds = 0.0;
    io.resetSimulationState();
  }

  private boolean areWheelsReadyForSimulatedShot() {
    return shotControlEnabled && areWheelsAtSpeed(1.0 - ShooterConstants.simWheelReadyRatio);
  }

  private boolean areWheelsAtSpeed(double toleranceRatio) {
    double clampedToleranceRatio = MathUtil.clamp(toleranceRatio, 0.0, 1.0);
    double pair1SetpointMagnitude = Math.abs(pair1VelocityCommandRadPerSec);
    double pair2SetpointMagnitude = Math.abs(pair2VelocityCommandRadPerSec);
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
        pair1VelocityCommandRadPerSec, pair2VelocityCommandRadPerSec);
  }

  private static double estimateLaunchSpeedFromWheelVelocitiesMetersPerSec(
      double pair1WheelVelocityRadPerSec, double pair2WheelVelocityRadPerSec) {
    double pair1SurfaceSpeedMetersPerSec =
        Math.abs(pair1WheelVelocityRadPerSec) * ShooterConstants.shooterWheelRadiusMeters;
    double pair2SurfaceSpeedMetersPerSec =
        Math.abs(pair2WheelVelocityRadPerSec) * ShooterConstants.shooterWheelRadiusMeters;
    double averageWheelSurfaceSpeedMetersPerSec =
        0.5 * (pair1SurfaceSpeedMetersPerSec + pair2SurfaceSpeedMetersPerSec);
    return averageWheelSurfaceSpeedToLaunchSpeedMetersPerSec(averageWheelSurfaceSpeedMetersPerSec);
  }

  private double getPair1VelocityTargetSetpointRadPerSec() {
    return pair1WheelSetpointRadPerSec
        * wheelSpeedScale
        * operatorWheelThrottleScale
        * pair1Direction;
  }

  private double getPair2VelocityTargetSetpointRadPerSec() {
    return pair2WheelSetpointRadPerSec
        * wheelSpeedScale
        * operatorWheelThrottleScale
        * pair2Direction;
  }

  private static double rampWheelVelocityCommand(
      double currentRadPerSec, double targetRadPerSec, boolean shotControlEnabled) {
    double rampRateRadPerSecSquared =
        Math.abs(targetRadPerSec) > Math.abs(currentRadPerSec)
            ? ShooterConstants.wheelCommandRampUpRadPerSecSquared
            : !shotControlEnabled
                ? ShooterConstants.wheelCommandReleaseRampDownRadPerSecSquared
                : Constants.currentMode == Constants.Mode.SIM
                    ? ShooterConstants.simWheelCommandRampDownRadPerSecSquared
                    : ShooterConstants.wheelCommandRampDownRadPerSecSquared;
    double maxStepRadPerSec = rampRateRadPerSecSquared * loopPeriodSeconds;
    double deltaRadPerSec = targetRadPerSec - currentRadPerSec;
    if (Math.abs(deltaRadPerSec) <= maxStepRadPerSec) {
      return targetRadPerSec;
    }
    return currentRadPerSec + Math.copySign(maxStepRadPerSec, deltaRadPerSec);
  }

  private double updateWheelVelocityCommand(double currentRadPerSec, double targetRadPerSec) {
    return calibrationModeEnabled
        ? targetRadPerSec
        : rampWheelVelocityCommand(currentRadPerSec, targetRadPerSec, shotControlEnabled);
  }

  private boolean shouldUseFullWheelPowerTarget() {
    return calibrationModeEnabled || Constants.currentMode == Constants.Mode.SIM;
  }

  private double clampToHoodCalibrationRange(double hoodPositionRotations) {
    return MathUtil.clamp(
        hoodPositionRotations,
        Math.min(hoodRetractedPositionRotations, hoodExtendedPositionRotations),
        Math.max(hoodRetractedPositionRotations, hoodExtendedPositionRotations));
  }

  private double getAppliedHoodSetpointMotorRotations() {
    return trenchSafetyRetractOverrideEnabled
        ? hoodRetractedPositionRotations
        : hoodSetpointMotorRotations;
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

  private static double optionalClearanceToTargetHeightOrNaN(
      OptionalDouble projectileHeightMeters, double targetHeightMeters) {
    return projectileHeightMeters.isPresent()
        ? projectileHeightMeters.getAsDouble() - targetHeightMeters
        : Double.NaN;
  }

  private static double clampAirtime(double airtimeSeconds) {
    return MathUtil.clamp(
        airtimeSeconds, ShooterConstants.minAirtimeSeconds, ShooterConstants.maxAirtimeSeconds);
  }

  private void logControlState(
      double pair1VelocityCommandRadPerSec, double pair2VelocityCommandRadPerSec) {
    double appliedHoodSetpointRotations = getAppliedHoodSetpointMotorRotations();
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/Enabled"), shotControlEnabled);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/Pair1SetpointRadPerSec"),
        pair1WheelSetpointRadPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/Pair2SetpointRadPerSec"),
        pair2WheelSetpointRadPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/WheelSpeedScale"), wheelSpeedScale);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/OperatorWheelThrottleScale"),
        operatorWheelThrottleScale);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/Pair1Direction"), pair1Direction);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/Pair2Direction"), pair2Direction);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/Pair1CommandRadPerSec"),
        pair1VelocityCommandRadPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/Pair2CommandRadPerSec"),
        pair2VelocityCommandRadPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/HoodSetpointRotations"),
        appliedHoodSetpointRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/HoodSetpointNormalized"),
        getHoodNormalizedPosition(appliedHoodSetpointRotations));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/ManualHoodOverrideEnabled"),
        manualHoodOverrideEnabled);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/ManualWheelOverrideEnabled"),
        manualWheelOverrideEnabled);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/TrenchSafetyRetractOverrideEnabled"),
        trenchSafetyRetractOverrideEnabled);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/Active"), hoodHomingActive);
    Logger.recordOutput(NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/Homed"), hoodHomed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/LastRunSucceeded"),
        hoodHomingSucceeded);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/ElapsedSeconds"),
        hoodHomingElapsedSeconds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/PhaseElapsedSeconds"),
        hoodHomingPhaseElapsedSeconds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/StallSeconds"), hoodHomingStallSeconds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/Phase"),
        switch (hoodHomingPhase) {
          case RELAX_BEFORE_CALIBRATION -> "RELAX_BEFORE_CALIBRATION";
          case SEEK_RETRACTED_HARD_STOP -> "SEEK_RETRACTED_HARD_STOP";
          case SEEK_EXTENDED_HARD_STOP -> "SEEK_EXTENDED_HARD_STOP";
        });
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/MeasuredCurrentAmps"),
        inputs.hoodCurrentAmps);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/MeasuredVelocityRotationsPerSec"),
        inputs.hoodVelocityRotationsPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/MeasuredRetractedHardStopRotations"),
        hoodHomingRetractedHardStopRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/MeasuredExtendedHardStopRotations"),
        hoodHomingExtendedHardStopRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Homing/MeasuredTravelRotations"),
        Math.abs(hoodHomingExtendedHardStopRotations - hoodHomingRetractedHardStopRotations));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Control/HoodSetpointDegrees"),
        motorRotationsToHoodAngle(appliedHoodSetpointRotations).getDegrees());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Config/HoodRetractedPositionRotations"),
        hoodRetractedPositionRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Config/HoodExtendedPositionRotations"),
        hoodExtendedPositionRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Measured/Pair1VelocityRadPerSec"),
        inputs.pair1LeaderVelocityRadPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Measured/Pair2VelocityRadPerSec"),
        inputs.pair2LeaderVelocityRadPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Measured/HoodPositionRotations"),
        inputs.hoodPositionRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Measured/HoodPositionNormalized"),
        getHoodNormalizedPosition(inputs.hoodPositionRotations));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Measured/HoodAngleDegrees"),
        motorRotationsToHoodAngle(inputs.hoodPositionRotations).getDegrees());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Measured/EstimatedLaunchSpeedMetersPerSec"),
        getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Calibration/ModeEnabled"),
        calibrationModeEnabled);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Calibration/HoodSetpointRotations"),
        calibrationHoodSetpointRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Calibration/HoodSetpointDegrees"),
        motorRotationsToHoodAngle(calibrationHoodSetpointRotations).getDegrees());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Calibration/Pair1SetpointRadPerSec"),
        calibrationPair1WheelSetpointRadPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Calibration/Pair2SetpointRadPerSec"),
        calibrationPair2WheelSetpointRadPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Calibration/Measurement/DistanceMeters"),
        calibrationMeasuredDistanceMeters);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Calibration/Measurement/HeightDeltaMeters"),
        calibrationMeasuredHeightDeltaMeters);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Calibration/Measurement/AirtimeSeconds"),
        calibrationMeasuredAirtimeSeconds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath(
            "GamePiece/Shooter/Calibration/Measurement/VideoLaunchAngleDegrees"),
        calibrationVideoLaunchAngleDegrees);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Calibration/RecordedSampleCount"),
        calibrationRecordedSampleCount);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Simulation/Shooter/WheelsReady"),
        areWheelsReadyForSimulatedShot());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Interlock/WheelsReadyForShot"),
        areWheelsAtSpeedForShot());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Interlock/HubShotPredictedToScore"),
        isHubShotPredictedToScore());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Interlock/ReadyToFire"), isReadyToFire());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/PredictedHorizontalErrorMeters"),
        latestPredictedHubShot.horizontalErrorMeters());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/PredictedVerticalErrorMeters"),
        latestPredictedHubShot.verticalErrorMeters());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/PredictedEvaluationTimeSeconds"),
        latestPredictedHubShot.evaluationTimeSeconds());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/PredictedDescendingAtTarget"),
        latestPredictedHubShot.descendingAtTarget());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/RobotFieldAccelerationX"),
        latestRobotFieldAccelerationMetersPerSecSquared.getX());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/RobotFieldAccelerationY"),
        latestRobotFieldAccelerationMetersPerSecSquared.getY());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/PredictionLaunchLeadSeconds"),
        hubShotPredictionLaunchLeadSeconds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/PredictedScoreToleranceXYMeters"),
        ShooterConstants.hubPredictedScoreToleranceXYMeters);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/PredictedScoreToleranceZMeters"),
        ShooterConstants.hubPredictedScoreToleranceZMeters);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Status/Pair1Connected"),
        inputs.pair1Connected);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Status/Pair2Connected"),
        inputs.pair2Connected);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Shooter/Status/HoodConnected"), inputs.hoodConnected);
  }

  private void logHubShotSolution(
      HubShotSolution solution,
      Pose2d baseHubPose,
      Pose2d compensatedHubPose,
      double targetHeightMeters,
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
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/DistanceMeters"),
        solution.distanceMeters());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/LaunchAngleDegrees"),
        solution.launchAngle().getDegrees());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/LaunchSpeedMetersPerSec"),
        solution.launchSpeedMetersPerSec());
    Logger.recordOutput(
        NetworkTablesUtil.logPath(
            "Targeting/Hub/Shooter/EstimatedLaunchSpeedFromWheelsMetersPerSec"),
        estimateLaunchSpeedFromWheelSetpointsMetersPerSec());
    Logger.recordOutput(
        NetworkTablesUtil.logPath(
            "Targeting/Hub/Shooter/EstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec"),
        getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/PowerSetpoint"), solution.shooterPower());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/AirtimeSeconds"),
        solution.airtimeSeconds());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/TargetHeightMeters"), targetHeightMeters);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/TargetHeightOffsetMeters"),
        hubAimHeightOffsetMeters);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/Feasible"), solution.feasible());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/BaseTargetPose"), baseHubPose);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/CompensatedTargetPose"),
        compensatedHubPose);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/RobotFieldVelocityX"),
        robotFieldVelocityMetersPerSec.getX());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/RobotFieldVelocityY"),
        robotFieldVelocityMetersPerSec.getY());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/CompensationOffsetX"),
        compensationOffset.getX());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/CompensationOffsetY"),
        compensationOffset.getY());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/ApexHeightMeters"), apexHeightMeters);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/ApexAboveTargetMeters"),
        apexHeightMeters - targetHeightMeters);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/ApexAboveHubCenterMeters"),
        apexHeightMeters - ShooterConstants.hubCenterHeightMeters);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/VerticalVelocityAtHubMetersPerSec"),
        optionalDoubleOrNaN(verticalVelocityAtHubMetersPerSec));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/TopEntrySatisfied"), topEntrySatisfied);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/MotionCompVelocityScale"),
        hubMotionCompVelocityScale);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/MotionCompLeadSeconds"),
        hubMotionCompLeadSeconds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/MotionSolveIterations"),
        motionSolveIterations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Shooter/MotionSolveConverged"),
        motionSolveConverged);
  }
}
