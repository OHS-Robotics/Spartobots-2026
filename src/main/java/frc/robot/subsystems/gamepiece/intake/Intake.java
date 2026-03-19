package frc.robot.subsystems.gamepiece.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NetworkTablesUtil;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final double loopPeriodSeconds = 0.02;

  private enum IntakePivotCalibrationPhase {
    SEEK_RETRACTED_HARD_STOP,
    SEEK_EXTENDED_HARD_STOP
  }

  private double targetIntakeSpeed = IntakeConstants.defaultIntakeSpeed;
  private Command currentIntakeRunCommand;
  public boolean intakeRunning = false;
  private double lastAppliedIntakeDriveSpeed = 0.0;
  private double lastAppliedIntakePivotSpeed = 0.0;
  private double intakePivotRetractedPositionRotations =
      IntakeConstants.defaultIntakePivotRetractedPositionRotations;
  private double intakePivotExtendedPositionRotations =
      IntakeConstants.defaultIntakePivotExtendedPositionRotations;
  private boolean intakePivotCalibrationActive = false;
  private boolean intakePivotCalibrated = false;
  private boolean intakePivotCalibrationSucceeded = false;
  private IntakePivotCalibrationPhase intakePivotCalibrationPhase =
      IntakePivotCalibrationPhase.SEEK_RETRACTED_HARD_STOP;
  private double intakePivotCalibrationElapsedSeconds = 0.0;
  private double intakePivotCalibrationPhaseElapsedSeconds = 0.0;
  private double intakePivotCalibrationStallSeconds = 0.0;
  private double intakePivotCalibrationRetractedHardStopRotations = 0.0;
  private double intakePivotCalibrationExtendedHardStopRotations = 0.0;
  private boolean calibrationModeEnabled = false;
  private double calibrationDriveVelocitySetpointRotationsPerSec =
      IntakeConstants.defaultCalibrationDriveVelocitySetpointRotationsPerSec;
  private double calibrationPivotPositionSetpointRotations =
      IntakeConstants.defaultCalibrationPivotPositionSetpointRotations;
  private double driveVelocityKp = IntakeConstants.intakeDriveVelocityKp;
  private double driveVelocityKi = IntakeConstants.intakeDriveVelocityKi;
  private double driveVelocityKd = IntakeConstants.intakeDriveVelocityKd;
  private double driveVelocityKv = IntakeConstants.intakeDriveVelocityKv;
  private double pivotPositionKp = IntakeConstants.intakePivotPositionKp;
  private double pivotPositionKi = IntakeConstants.intakePivotPositionKi;
  private double pivotPositionKd = IntakeConstants.intakePivotPositionKd;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final NetworkTable subsystemTable =
      NetworkTablesUtil.domain(IntakeConstants.configTableName);
  private final NetworkTable tuningTable = NetworkTablesUtil.tuningCommon(subsystemTable);
  private final NetworkTable pidTuningTable =
      NetworkTablesUtil.tuningMode(subsystemTable).getSubTable("PID");
  private final NetworkTable telemetryTable = NetworkTablesUtil.telemetry(subsystemTable);
  private final NetworkTable calibrationTuningTable = tuningTable.getSubTable("Calibration");
  private final NetworkTable calibrationTelemetryTable = telemetryTable.getSubTable("Calibration");

  private final NetworkTableEntry intakeSpeedEntry = tuningTable.getEntry("Drive/Speed");
  private final NetworkTableEntry intakePivotSpeedScaleEntry =
      tuningTable.getEntry("Pivot/SpeedScale");
  private final NetworkTableEntry intakePivotInvertedEntry = tuningTable.getEntry("Pivot/Inverted");
  private final NetworkTableEntry intakeDriveDirectionEntry =
      tuningTable.getEntry("Drive/Direction");
  private final NetworkTableEntry intakePivotRetractedPositionEntry =
      tuningTable.getEntry("Pivot/Calibration/RetractedPositionRotations");
  private final NetworkTableEntry intakePivotExtendedPositionEntry =
      tuningTable.getEntry("Pivot/Calibration/ExtendedPositionRotations");
  private final NetworkTableEntry driveVelocityKpEntry =
      pidTuningTable.getEntry("Drive/Velocity/Kp");
  private final NetworkTableEntry driveVelocityKiEntry =
      pidTuningTable.getEntry("Drive/Velocity/Ki");
  private final NetworkTableEntry driveVelocityKdEntry =
      pidTuningTable.getEntry("Drive/Velocity/Kd");
  private final NetworkTableEntry driveVelocityKvEntry =
      pidTuningTable.getEntry("Drive/Velocity/Kv");
  private final NetworkTableEntry pivotPositionKpEntry =
      pidTuningTable.getEntry("Pivot/Position/Kp");
  private final NetworkTableEntry pivotPositionKiEntry =
      pidTuningTable.getEntry("Pivot/Position/Ki");
  private final NetworkTableEntry pivotPositionKdEntry =
      pidTuningTable.getEntry("Pivot/Position/Kd");
  private final NetworkTableEntry calibrationModeEnabledEntry =
      calibrationTuningTable.getEntry("Enabled");
  private final NetworkTableEntry calibrationDriveVelocitySetpointEntry =
      calibrationTuningTable.getEntry("Drive/VelocitySetpointRotationsPerSec");
  private final NetworkTableEntry calibrationPivotPositionSetpointEntry =
      calibrationTuningTable.getEntry("Pivot/PositionSetpointRotations");
  private final NetworkTableEntry intakePivotEncoderPositionEntry =
      telemetryTable.getEntry("Pivot/EncoderPositionRotations");
  private final NetworkTableEntry intakePivotEncoderVelocityEntry =
      telemetryTable.getEntry("Pivot/EncoderVelocityRpm");
  private final NetworkTableEntry intakePivotEncoderNormalizedPositionEntry =
      telemetryTable.getEntry("Pivot/EncoderPositionNormalized");
  private final NetworkTableEntry intakeDriveAppliedOutputEntry =
      telemetryTable.getEntry("Drive/AppliedOutput");
  private final NetworkTableEntry intakePivotAppliedOutputEntry =
      telemetryTable.getEntry("Pivot/AppliedOutput");
  private final NetworkTableEntry intakeDriveEstimatedVelocityEntry =
      telemetryTable.getEntry("Drive/EstimatedVelocityRotationsPerSec");
  private final NetworkTableEntry intakeDriveEstimatedPositionEntry =
      telemetryTable.getEntry("Drive/EstimatedPositionRotations");
  private final NetworkTableEntry calibrationModeTelemetryEntry =
      calibrationTelemetryTable.getEntry("ModeEnabled");
  private final NetworkTableEntry calibrationConfiguredDriveVelocityEntry =
      calibrationTelemetryTable.getEntry("Configured/Drive/VelocitySetpointRotationsPerSec");
  private final NetworkTableEntry calibrationConfiguredPivotPositionEntry =
      calibrationTelemetryTable.getEntry("Configured/Pivot/PositionSetpointRotations");
  private final NetworkTableEntry calibrationMeasuredDriveVelocityEntry =
      calibrationTelemetryTable.getEntry("Measured/Drive/VelocityRotationsPerSec");
  private final NetworkTableEntry calibrationMeasuredPivotPositionEntry =
      calibrationTelemetryTable.getEntry("Measured/Pivot/PositionRotations");

  public Intake() {
    this(new IntakeIO() {});
  }

  public Intake(IntakeIO io) {
    this.io = io;
    configureNetworkTableDefaults();
    loadNetworkTableConfig();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(NetworkTablesUtil.logPath("GamePiece/Intake/Inputs"), inputs);
    loadNetworkTableConfig();
    io.setDriveVelocityClosedLoopGains(
        driveVelocityKp, driveVelocityKi, driveVelocityKd, driveVelocityKv);
    io.setPivotPositionClosedLoopGains(pivotPositionKp, pivotPositionKi, pivotPositionKd);
    if (intakePivotCalibrationActive) {
      updateIntakePivotCalibration();
    } else if (calibrationModeEnabled) {
      applyCalibrationControl();
    }
    publishPivotEncoderToNetworkTables();
    publishActuatorStateToNetworkTables();
    publishCalibrationTelemetry();
    logTelemetry();
  }

  public void updateIntake() {
    if (calibrationModeEnabled) {
      return;
    }
    lastAppliedIntakeDriveSpeed =
        applyDirection(
            targetIntakeSpeed,
            intakeDriveDirectionEntry,
            IntakeConstants.defaultIntakeDriveDirection);
    io.setDriveOutput(lastAppliedIntakeDriveSpeed);
    intakeRunning = Math.abs(lastAppliedIntakeDriveSpeed) > 1e-3;
  }

  public void reverseTargetSpeed() {
    targetIntakeSpeed *= -1;
    intakeSpeedEntry.setDouble(targetIntakeSpeed);
  }

  public void setTargetIntakeSpeed(double speed) {
    targetIntakeSpeed = clampSpeed(speed);
    intakeSpeedEntry.setDouble(targetIntakeSpeed);
  }

  public double getTargetIntakeSpeed() {
    return targetIntakeSpeed;
  }

  public double getActualIntakeSpeed() {
    return inputs.driveAppliedOutput;
  }

  public void stopIntake() {
    lastAppliedIntakeDriveSpeed = 0.0;
    io.setDriveOutput(0.0);
    intakeRunning = false;
  }

  public void setIntakePivotSpeed(double speed) {
    if (intakePivotCalibrationActive || calibrationModeEnabled) {
      return;
    }
    lastAppliedIntakePivotSpeed =
        applyScaleAndInversion(speed, intakePivotSpeedScaleEntry, intakePivotInvertedEntry);
    io.setPivotOutput(lastAppliedIntakePivotSpeed);
  }

  public void stopIntakePivot() {
    if (intakePivotCalibrationActive) {
      return;
    }
    lastAppliedIntakePivotSpeed = 0.0;
    io.setPivotOutput(0.0);
  }

  public Command manualIntakePivotWhileHeldCommand(double speed) {
    return Commands.runEnd(() -> setIntakePivotSpeed(speed), this::stopIntakePivot, this);
  }

  public Command calibrateIntakePivotToHardStopsCommand() {
    return runOnce(this::startIntakePivotCalibration)
        .andThen(Commands.waitUntil(() -> !intakePivotCalibrationActive))
        .finallyDo(
            interrupted -> {
              if (interrupted) {
                cancelIntakePivotCalibration();
              }
            });
  }

  public boolean isIntakePivotCalibrationActive() {
    return intakePivotCalibrationActive;
  }

  public boolean isIntakePivotCalibrated() {
    return intakePivotCalibrated;
  }

  public boolean didLastIntakePivotCalibrationSucceed() {
    return intakePivotCalibrationSucceeded;
  }

  public boolean isCalibrationModeEnabled() {
    return calibrationModeEnabled;
  }

  public void setCalibrationModeEnabled(boolean enabled) {
    calibrationModeEnabled = enabled;
    calibrationModeEnabledEntry.setBoolean(calibrationModeEnabled);
    if (!enabled) {
      stopIntake();
      stopIntakePivot();
    }
  }

  public void setCalibrationDriveVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
    calibrationDriveVelocitySetpointRotationsPerSec =
        sanitizeFinite(velocityRotationsPerSec, calibrationDriveVelocitySetpointRotationsPerSec);
    calibrationDriveVelocitySetpointEntry.setDouble(
        calibrationDriveVelocitySetpointRotationsPerSec);
  }

  public void setCalibrationPivotPositionSetpointRotations(double positionRotations) {
    calibrationPivotPositionSetpointRotations = clampToPivotCalibrationRange(positionRotations);
    calibrationPivotPositionSetpointEntry.setDouble(calibrationPivotPositionSetpointRotations);
  }

  public Command enableCalibrationModeCommand() {
    return Commands.runOnce(() -> setCalibrationModeEnabled(true), this).ignoringDisable(true);
  }

  public Command disableCalibrationModeCommand() {
    return Commands.runOnce(() -> setCalibrationModeEnabled(false), this).ignoringDisable(true);
  }

  public Command toggleIntakeCommand() {
    return runOnce(
        () -> {
          if (!intakeRunning) {
            updateIntake();
            currentIntakeRunCommand = run(this::updateIntake);
            CommandScheduler.getInstance().schedule(currentIntakeRunCommand);
            intakeRunning = true;
          } else {
            stopIntake();
            stopCommand(currentIntakeRunCommand);
            currentIntakeRunCommand = null;
            intakeRunning = false;
          }
        });
  }

  public Command increaseIntakeSpeed() {
    return Commands.runOnce(
        () -> {
          targetIntakeSpeed = Math.min(targetIntakeSpeed + 0.05, 1.0);
          intakeSpeedEntry.setDouble(targetIntakeSpeed);
        });
  }

  public Command decreaseIntakeSpeed() {
    return Commands.runOnce(
        () -> {
          targetIntakeSpeed = Math.max(targetIntakeSpeed - 0.05, 0.0);
          intakeSpeedEntry.setDouble(targetIntakeSpeed);
        });
  }

  private void configureNetworkTableDefaults() {
    intakeSpeedEntry.setDefaultDouble(IntakeConstants.defaultIntakeSpeed);
    intakePivotSpeedScaleEntry.setDefaultDouble(IntakeConstants.defaultIntakePivotSpeedScale);
    intakePivotRetractedPositionEntry.setDefaultDouble(
        IntakeConstants.defaultIntakePivotRetractedPositionRotations);
    intakePivotExtendedPositionEntry.setDefaultDouble(
        IntakeConstants.defaultIntakePivotExtendedPositionRotations);
    intakeDriveDirectionEntry.setDefaultDouble(IntakeConstants.defaultIntakeDriveDirection);
    intakePivotInvertedEntry.setDefaultBoolean(IntakeConstants.defaultIntakePivotInverted);
    driveVelocityKpEntry.setDefaultDouble(IntakeConstants.intakeDriveVelocityKp);
    driveVelocityKiEntry.setDefaultDouble(IntakeConstants.intakeDriveVelocityKi);
    driveVelocityKdEntry.setDefaultDouble(IntakeConstants.intakeDriveVelocityKd);
    driveVelocityKvEntry.setDefaultDouble(IntakeConstants.intakeDriveVelocityKv);
    pivotPositionKpEntry.setDefaultDouble(IntakeConstants.intakePivotPositionKp);
    pivotPositionKiEntry.setDefaultDouble(IntakeConstants.intakePivotPositionKi);
    pivotPositionKdEntry.setDefaultDouble(IntakeConstants.intakePivotPositionKd);
    calibrationModeEnabledEntry.setDefaultBoolean(false);
    calibrationDriveVelocitySetpointEntry.setDefaultDouble(
        IntakeConstants.defaultCalibrationDriveVelocitySetpointRotationsPerSec);
    calibrationPivotPositionSetpointEntry.setDefaultDouble(
        IntakeConstants.defaultCalibrationPivotPositionSetpointRotations);
  }

  private void loadNetworkTableConfig() {
    targetIntakeSpeed = clampSpeed(intakeSpeedEntry.getDouble(targetIntakeSpeed));
    intakeSpeedEntry.setDouble(targetIntakeSpeed);
    intakePivotSpeedScaleEntry.setDouble(
        clampSpeedScale(
            intakePivotSpeedScaleEntry.getDouble(IntakeConstants.defaultIntakePivotSpeedScale)));
    intakePivotRetractedPositionRotations =
        intakePivotRetractedPositionEntry.getDouble(intakePivotRetractedPositionRotations);
    intakePivotExtendedPositionRotations =
        intakePivotExtendedPositionEntry.getDouble(intakePivotExtendedPositionRotations);
    intakePivotRetractedPositionEntry.setDouble(intakePivotRetractedPositionRotations);
    intakePivotExtendedPositionEntry.setDouble(intakePivotExtendedPositionRotations);
    double intakeDriveDirection =
        normalizeDirection(
            intakeDriveDirectionEntry.getDouble(IntakeConstants.defaultIntakeDriveDirection));
    intakeDriveDirectionEntry.setDouble(intakeDriveDirection);
    driveVelocityKp =
        sanitizeFinite(driveVelocityKpEntry.getDouble(driveVelocityKp), driveVelocityKp);
    driveVelocityKi =
        sanitizeFinite(driveVelocityKiEntry.getDouble(driveVelocityKi), driveVelocityKi);
    driveVelocityKd =
        sanitizeFinite(driveVelocityKdEntry.getDouble(driveVelocityKd), driveVelocityKd);
    driveVelocityKv =
        sanitizeFinite(driveVelocityKvEntry.getDouble(driveVelocityKv), driveVelocityKv);
    pivotPositionKp =
        sanitizeFinite(pivotPositionKpEntry.getDouble(pivotPositionKp), pivotPositionKp);
    pivotPositionKi =
        sanitizeFinite(pivotPositionKiEntry.getDouble(pivotPositionKi), pivotPositionKi);
    pivotPositionKd =
        sanitizeFinite(pivotPositionKdEntry.getDouble(pivotPositionKd), pivotPositionKd);
    calibrationModeEnabled = calibrationModeEnabledEntry.getBoolean(calibrationModeEnabled);
    calibrationDriveVelocitySetpointRotationsPerSec =
        sanitizeFinite(
            calibrationDriveVelocitySetpointEntry.getDouble(
                calibrationDriveVelocitySetpointRotationsPerSec),
            calibrationDriveVelocitySetpointRotationsPerSec);
    calibrationPivotPositionSetpointRotations =
        clampToPivotCalibrationRange(
            sanitizeFinite(
                calibrationPivotPositionSetpointEntry.getDouble(
                    calibrationPivotPositionSetpointRotations),
                calibrationPivotPositionSetpointRotations));
    driveVelocityKpEntry.setDouble(driveVelocityKp);
    driveVelocityKiEntry.setDouble(driveVelocityKi);
    driveVelocityKdEntry.setDouble(driveVelocityKd);
    driveVelocityKvEntry.setDouble(driveVelocityKv);
    pivotPositionKpEntry.setDouble(pivotPositionKp);
    pivotPositionKiEntry.setDouble(pivotPositionKi);
    pivotPositionKdEntry.setDouble(pivotPositionKd);
    calibrationModeEnabledEntry.setBoolean(calibrationModeEnabled);
    calibrationDriveVelocitySetpointEntry.setDouble(
        calibrationDriveVelocitySetpointRotationsPerSec);
    calibrationPivotPositionSetpointEntry.setDouble(calibrationPivotPositionSetpointRotations);
  }

  private double applyScaleAndInversion(
      double speed, NetworkTableEntry speedScaleEntry, NetworkTableEntry invertedEntry) {
    double scaledSpeed = clampSpeed(speed) * clampSpeedScale(speedScaleEntry.getDouble(1.0));
    return applyInversion(scaledSpeed, invertedEntry);
  }

  private double applyDirection(
      double speed, NetworkTableEntry directionEntry, double defaultDirection) {
    return clampSpeed(speed) * normalizeDirection(directionEntry.getDouble(defaultDirection));
  }

  private double applyInversion(double speed, NetworkTableEntry invertedEntry) {
    return invertedEntry.getBoolean(false) ? -speed : speed;
  }

  private double clampSpeed(double speed) {
    return MathUtil.clamp(speed, -1.0, 1.0);
  }

  private double clampSpeedScale(double speedScale) {
    return MathUtil.clamp(speedScale, 0.0, 1.0);
  }

  private void applyCalibrationControl() {
    io.setDriveVelocitySetpointRotationsPerSec(calibrationDriveVelocitySetpointRotationsPerSec);
    io.setPivotPositionSetpointRotations(calibrationPivotPositionSetpointRotations);
    lastAppliedIntakeDriveSpeed = 0.0;
    lastAppliedIntakePivotSpeed = 0.0;
    intakeRunning = Math.abs(calibrationDriveVelocitySetpointRotationsPerSec) > 1e-3;
  }

  private void publishPivotEncoderToNetworkTables() {
    intakePivotEncoderPositionEntry.setDouble(getIntakePivotMeasuredPositionRotations());
    intakePivotEncoderVelocityEntry.setDouble(inputs.pivotVelocityRpm);
    intakePivotEncoderNormalizedPositionEntry.setDouble(getIntakePivotMeasuredPositionNormalized());
  }

  private void publishActuatorStateToNetworkTables() {
    intakeDriveAppliedOutputEntry.setDouble(inputs.driveAppliedOutput);
    intakeDriveEstimatedVelocityEntry.setDouble(inputs.driveVelocityRotationsPerSec);
    intakeDriveEstimatedPositionEntry.setDouble(inputs.drivePositionRotations);
    intakePivotAppliedOutputEntry.setDouble(inputs.pivotAppliedOutput);
  }

  private void publishCalibrationTelemetry() {
    calibrationModeTelemetryEntry.setBoolean(calibrationModeEnabled);
    calibrationConfiguredDriveVelocityEntry.setDouble(
        calibrationDriveVelocitySetpointRotationsPerSec);
    calibrationConfiguredPivotPositionEntry.setDouble(calibrationPivotPositionSetpointRotations);
    calibrationMeasuredDriveVelocityEntry.setDouble(inputs.driveVelocityRotationsPerSec);
    calibrationMeasuredPivotPositionEntry.setDouble(inputs.pivotPositionRotations);
  }

  public double getIntakePivotMeasuredPositionRotations() {
    return inputs.pivotPositionRotations;
  }

  public double getIntakeDriveCurrentAmps() {
    return inputs.driveCurrentAmps;
  }

  public double getIntakePivotCurrentAmps() {
    return inputs.pivotCurrentAmps;
  }

  public double getIntakePivotMeasuredPositionNormalized() {
    return clampUnitInterval(
        inverseInterpolate(
            getIntakePivotMeasuredPositionRotations(),
            intakePivotRetractedPositionRotations,
            intakePivotExtendedPositionRotations));
  }

  public double getIntakePivotRetractedPositionRotations() {
    return intakePivotRetractedPositionRotations;
  }

  public double getIntakePivotExtendedPositionRotations() {
    return intakePivotExtendedPositionRotations;
  }

  public void resetSimulationState() {
    io.resetSimulationState();
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

  private static double normalizeDirection(double direction) {
    return direction < 0.0 ? -1.0 : 1.0;
  }

  private static double sanitizeFinite(double value, double fallback) {
    return Double.isFinite(value) ? value : fallback;
  }

  private double clampToPivotCalibrationRange(double positionRotations) {
    return MathUtil.clamp(
        positionRotations,
        intakePivotRetractedPositionRotations,
        intakePivotExtendedPositionRotations);
  }

  private void startIntakePivotCalibration() {
    intakePivotCalibrationActive = true;
    intakePivotCalibrated = false;
    intakePivotCalibrationSucceeded = false;
    intakePivotCalibrationPhase = IntakePivotCalibrationPhase.SEEK_RETRACTED_HARD_STOP;
    intakePivotCalibrationElapsedSeconds = 0.0;
    intakePivotCalibrationPhaseElapsedSeconds = 0.0;
    intakePivotCalibrationStallSeconds = 0.0;
    intakePivotCalibrationRetractedHardStopRotations = getIntakePivotMeasuredPositionRotations();
    intakePivotCalibrationExtendedHardStopRotations = getIntakePivotMeasuredPositionRotations();
  }

  private void cancelIntakePivotCalibration() {
    intakePivotCalibrationActive = false;
    intakePivotCalibrationElapsedSeconds = 0.0;
    intakePivotCalibrationPhaseElapsedSeconds = 0.0;
    intakePivotCalibrationStallSeconds = 0.0;
    setRawPivotOutput(0.0);
  }

  private void updateIntakePivotCalibration() {
    intakePivotCalibrationElapsedSeconds += loopPeriodSeconds;
    intakePivotCalibrationPhaseElapsedSeconds += loopPeriodSeconds;

    if (intakePivotCalibrationPhase == IntakePivotCalibrationPhase.SEEK_RETRACTED_HARD_STOP) {
      setRawPivotOutput(IntakeConstants.intakePivotCalibrationOutputTowardRetractedHardStop);
    } else {
      setRawPivotOutput(IntakeConstants.intakePivotCalibrationOutputTowardExtendedHardStop);
    }

    boolean velocityNearZero =
        Math.abs(inputs.pivotVelocityRpm) <= IntakeConstants.intakePivotCalibrationMaxVelocityRpm;
    boolean currentHigh =
        Math.abs(inputs.pivotCurrentAmps) >= IntakeConstants.intakePivotCalibrationMinCurrentAmps;
    if (velocityNearZero && currentHigh) {
      intakePivotCalibrationStallSeconds += loopPeriodSeconds;
    } else {
      intakePivotCalibrationStallSeconds = 0.0;
    }

    if (intakePivotCalibrationStallSeconds
        >= IntakeConstants.intakePivotCalibrationStallConfirmSeconds) {
      if (intakePivotCalibrationPhase == IntakePivotCalibrationPhase.SEEK_RETRACTED_HARD_STOP) {
        intakePivotCalibrationRetractedHardStopRotations =
            getIntakePivotMeasuredPositionRotations();
        intakePivotCalibrationPhase = IntakePivotCalibrationPhase.SEEK_EXTENDED_HARD_STOP;
        intakePivotCalibrationPhaseElapsedSeconds = 0.0;
        intakePivotCalibrationStallSeconds = 0.0;
        return;
      }

      intakePivotCalibrationExtendedHardStopRotations = getIntakePivotMeasuredPositionRotations();
      double measuredTravelRotations =
          Math.abs(
              intakePivotCalibrationExtendedHardStopRotations
                  - intakePivotCalibrationRetractedHardStopRotations);
      finishIntakePivotCalibration(
          measuredTravelRotations >= IntakeConstants.intakePivotCalibrationMinTravelRotations);
      return;
    }

    if (intakePivotCalibrationPhaseElapsedSeconds
        >= IntakeConstants.intakePivotCalibrationTimeoutSeconds) {
      finishIntakePivotCalibration(false);
    }
  }

  private void finishIntakePivotCalibration(boolean success) {
    intakePivotCalibrationActive = false;
    intakePivotCalibrationSucceeded = success;
    setRawPivotOutput(0.0);
    if (!success) {
      return;
    }

    intakePivotCalibrated = true;
    setPivotCalibrationFromHardStops(
        intakePivotCalibrationRetractedHardStopRotations,
        intakePivotCalibrationExtendedHardStopRotations);
    calibrationPivotPositionSetpointRotations =
        clampToPivotCalibrationRange(calibrationPivotPositionSetpointRotations);
    calibrationPivotPositionSetpointEntry.setDouble(calibrationPivotPositionSetpointRotations);
  }

  private void setPivotCalibrationFromHardStops(
      double retractedRotations, double extendedRotations) {
    intakePivotRetractedPositionRotations = retractedRotations;
    intakePivotExtendedPositionRotations = extendedRotations;
    intakePivotRetractedPositionEntry.setDouble(intakePivotRetractedPositionRotations);
    intakePivotExtendedPositionEntry.setDouble(intakePivotExtendedPositionRotations);
  }

  private void setRawPivotOutput(double speed) {
    lastAppliedIntakePivotSpeed = clampSpeed(speed);
    io.setPivotOutput(lastAppliedIntakePivotSpeed);
  }

  private void stopCommand(Command command) {
    if (command != null) {
      CommandScheduler.getInstance().cancel(command);
    }
  }

  private void logTelemetry() {
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Config/TargetSpeed"), targetIntakeSpeed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Config/PivotSpeedScale"),
        clampSpeedScale(
            intakePivotSpeedScaleEntry.getDouble(IntakeConstants.defaultIntakePivotSpeedScale)));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Config/DriveDirection"),
        normalizeDirection(
            intakeDriveDirectionEntry.getDouble(IntakeConstants.defaultIntakeDriveDirection)));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Config/PivotInverted"),
        intakePivotInvertedEntry.getBoolean(IntakeConstants.defaultIntakePivotInverted));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Config/PivotRetractedPositionRotations"),
        intakePivotRetractedPositionRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Config/PivotExtendedPositionRotations"),
        intakePivotExtendedPositionRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Calibration/ClosedLoop/Enabled"),
        calibrationModeEnabled);
    Logger.recordOutput(
        NetworkTablesUtil.logPath(
            "GamePiece/Intake/Calibration/ClosedLoop/DriveVelocitySetpointRotationsPerSec"),
        calibrationDriveVelocitySetpointRotationsPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath(
            "GamePiece/Intake/Calibration/ClosedLoop/PivotPositionSetpointRotations"),
        calibrationPivotPositionSetpointRotations);

    Logger.recordOutput(NetworkTablesUtil.logPath("GamePiece/Intake/State/Running"), intakeRunning);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/State/LastAppliedDriveOutput"),
        lastAppliedIntakeDriveSpeed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/State/LastAppliedPivotOutput"),
        lastAppliedIntakePivotSpeed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/State/ActualDriveOutput"),
        inputs.driveAppliedOutput);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/State/ActualPivotOutput"),
        inputs.pivotAppliedOutput);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Calibration/Pivot/Active"),
        intakePivotCalibrationActive);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Calibration/Pivot/Calibrated"),
        intakePivotCalibrated);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Calibration/Pivot/LastRunSucceeded"),
        intakePivotCalibrationSucceeded);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Calibration/Pivot/ElapsedSeconds"),
        intakePivotCalibrationElapsedSeconds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Calibration/Pivot/StallSeconds"),
        intakePivotCalibrationStallSeconds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Calibration/Pivot/Phase"),
        intakePivotCalibrationPhase == IntakePivotCalibrationPhase.SEEK_RETRACTED_HARD_STOP
            ? "SEEK_RETRACTED_HARD_STOP"
            : "SEEK_EXTENDED_HARD_STOP");
    Logger.recordOutput(
        NetworkTablesUtil.logPath(
            "GamePiece/Intake/Calibration/Pivot/MeasuredRetractedHardStopRotations"),
        intakePivotCalibrationRetractedHardStopRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath(
            "GamePiece/Intake/Calibration/Pivot/MeasuredExtendedHardStopRotations"),
        intakePivotCalibrationExtendedHardStopRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Measured/DriveCurrentAmps"),
        inputs.driveCurrentAmps);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Measured/PivotCurrentAmps"),
        inputs.pivotCurrentAmps);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Measured/PivotEncoderPositionRotations"),
        getIntakePivotMeasuredPositionRotations());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Measured/PivotEncoderVelocityRpm"),
        inputs.pivotVelocityRpm);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Intake/Measured/PivotEncoderPositionNormalized"),
        getIntakePivotMeasuredPositionNormalized());
  }
}
