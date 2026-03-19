package frc.robot.subsystems.gamepiece.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NetworkTablesUtil;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private double targetAgitatorSpeed = HopperConstants.defaultHopperAgitatorSpeed;
  private Command currentAgitatorRunCommand;
  private boolean agitatorRunning = false;
  private double lastAppliedAgitatorSpeed = 0.0;
  private double lastAppliedExtensionSpeed = 0.0;
  private double hopperExtensionRetractedPositionRotations =
      HopperConstants.defaultHopperExtensionRetractedPositionRotations;
  private double hopperExtensionExtendedPositionRotations =
      HopperConstants.defaultHopperExtensionExtendedPositionRotations;
  private boolean calibrationModeEnabled = false;
  private double calibrationAgitatorVelocitySetpointRotationsPerSec =
      HopperConstants.defaultCalibrationAgitatorVelocitySetpointRotationsPerSec;
  private double calibrationExtensionPositionSetpointRotations =
      HopperConstants.defaultCalibrationExtensionPositionSetpointRotations;
  private double agitatorVelocityKp = HopperConstants.hopperAgitatorVelocityKp;
  private double agitatorVelocityKi = HopperConstants.hopperAgitatorVelocityKi;
  private double agitatorVelocityKd = HopperConstants.hopperAgitatorVelocityKd;
  private double agitatorVelocityKv = HopperConstants.hopperAgitatorVelocityKv;
  private double extensionPositionKp = HopperConstants.hopperExtensionPositionKp;
  private double extensionPositionKi = HopperConstants.hopperExtensionPositionKi;
  private double extensionPositionKd = HopperConstants.hopperExtensionPositionKd;

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private final NetworkTable subsystemTable =
      NetworkTablesUtil.domain(HopperConstants.configTableName);
  private final NetworkTable tuningTable = NetworkTablesUtil.tuningCommon(subsystemTable);
  private final NetworkTable pidTuningTable =
      NetworkTablesUtil.tuningMode(subsystemTable).getSubTable("PID");
  private final NetworkTable telemetryTable = NetworkTablesUtil.telemetry(subsystemTable);
  private final NetworkTable calibrationTuningTable = tuningTable.getSubTable("Calibration");
  private final NetworkTable calibrationTelemetryTable = telemetryTable.getSubTable("Calibration");

  private final NetworkTableEntry hopperAgitatorSpeedEntry = tuningTable.getEntry("Agitator/Speed");
  private final NetworkTableEntry hopperExtensionSpeedScaleEntry =
      tuningTable.getEntry("Extension/SpeedScale");
  private final NetworkTableEntry hopperExtensionInvertedEntry =
      tuningTable.getEntry("Extension/Inverted");
  private final NetworkTableEntry hopperAgitatorDirectionEntry =
      tuningTable.getEntry("Agitator/Direction");
  private final NetworkTableEntry hopperExtensionRetractedPositionEntry =
      tuningTable.getEntry("Extension/Calibration/RetractedPositionRotations");
  private final NetworkTableEntry hopperExtensionExtendedPositionEntry =
      tuningTable.getEntry("Extension/Calibration/ExtendedPositionRotations");
  private final NetworkTableEntry agitatorVelocityKpEntry =
      pidTuningTable.getEntry("Agitator/Velocity/Kp");
  private final NetworkTableEntry agitatorVelocityKiEntry =
      pidTuningTable.getEntry("Agitator/Velocity/Ki");
  private final NetworkTableEntry agitatorVelocityKdEntry =
      pidTuningTable.getEntry("Agitator/Velocity/Kd");
  private final NetworkTableEntry agitatorVelocityKvEntry =
      pidTuningTable.getEntry("Agitator/Velocity/Kv");
  private final NetworkTableEntry extensionPositionKpEntry =
      pidTuningTable.getEntry("Extension/Position/Kp");
  private final NetworkTableEntry extensionPositionKiEntry =
      pidTuningTable.getEntry("Extension/Position/Ki");
  private final NetworkTableEntry extensionPositionKdEntry =
      pidTuningTable.getEntry("Extension/Position/Kd");
  private final NetworkTableEntry calibrationModeEnabledEntry =
      calibrationTuningTable.getEntry("Enabled");
  private final NetworkTableEntry calibrationAgitatorVelocitySetpointEntry =
      calibrationTuningTable.getEntry("Agitator/VelocitySetpointRotationsPerSec");
  private final NetworkTableEntry calibrationExtensionPositionSetpointEntry =
      calibrationTuningTable.getEntry("Extension/PositionSetpointRotations");
  private final NetworkTableEntry hopperExtensionEncoderPositionEntry =
      telemetryTable.getEntry("Extension/EncoderPositionRotations");
  private final NetworkTableEntry hopperExtensionEncoderVelocityEntry =
      telemetryTable.getEntry("Extension/EncoderVelocityRpm");
  private final NetworkTableEntry hopperExtensionEncoderNormalizedPositionEntry =
      telemetryTable.getEntry("Extension/EncoderPositionNormalized");
  private final NetworkTableEntry hopperAgitatorAppliedOutputEntry =
      telemetryTable.getEntry("Agitator/AppliedOutput");
  private final NetworkTableEntry hopperExtensionAppliedOutputEntry =
      telemetryTable.getEntry("Extension/AppliedOutput");
  private final NetworkTableEntry hopperAgitatorEstimatedVelocityEntry =
      telemetryTable.getEntry("Agitator/EstimatedVelocityRotationsPerSec");
  private final NetworkTableEntry hopperAgitatorEstimatedPositionEntry =
      telemetryTable.getEntry("Agitator/EstimatedPositionRotations");
  private final NetworkTableEntry calibrationModeTelemetryEntry =
      calibrationTelemetryTable.getEntry("ModeEnabled");
  private final NetworkTableEntry calibrationConfiguredAgitatorVelocityEntry =
      calibrationTelemetryTable.getEntry("Configured/Agitator/VelocitySetpointRotationsPerSec");
  private final NetworkTableEntry calibrationConfiguredExtensionPositionEntry =
      calibrationTelemetryTable.getEntry("Configured/Extension/PositionSetpointRotations");
  private final NetworkTableEntry calibrationMeasuredAgitatorVelocityEntry =
      calibrationTelemetryTable.getEntry("Measured/Agitator/VelocityRotationsPerSec");
  private final NetworkTableEntry calibrationMeasuredExtensionPositionEntry =
      calibrationTelemetryTable.getEntry("Measured/Extension/PositionRotations");

  public Hopper() {
    this(new HopperIO() {});
  }

  public Hopper(HopperIO io) {
    this.io = io;
    configureNetworkTableDefaults();
    loadNetworkTableConfig();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(NetworkTablesUtil.logPath("GamePiece/Hopper/Inputs"), inputs);
    loadNetworkTableConfig();
    io.setAgitatorVelocityClosedLoopGains(
        agitatorVelocityKp, agitatorVelocityKi, agitatorVelocityKd, agitatorVelocityKv);
    io.setExtensionPositionClosedLoopGains(
        extensionPositionKp, extensionPositionKi, extensionPositionKd);
    if (calibrationModeEnabled) {
      applyCalibrationControl();
    }
    publishExtensionEncoderToNetworkTables();
    publishActuatorStateToNetworkTables();
    publishCalibrationTelemetry();
    logTelemetry();
  }

  public void updateAgitator() {
    if (calibrationModeEnabled) {
      return;
    }
    lastAppliedAgitatorSpeed =
        applyDirection(
            targetAgitatorSpeed,
            hopperAgitatorDirectionEntry,
            HopperConstants.defaultHopperAgitatorDirection);
    io.setAgitatorOutput(lastAppliedAgitatorSpeed);
    agitatorRunning = Math.abs(lastAppliedAgitatorSpeed) > 1e-3;
  }

  public void reverseAgitatorSpeed() {
    targetAgitatorSpeed *= -1;
    hopperAgitatorSpeedEntry.setDouble(targetAgitatorSpeed);
  }

  public void setTargetAgitatorSpeed(double speed) {
    targetAgitatorSpeed = clampSpeed(speed);
    hopperAgitatorSpeedEntry.setDouble(targetAgitatorSpeed);
  }

  public double getTargetAgitatorSpeed() {
    return targetAgitatorSpeed;
  }

  public double getActualAgitatorSpeed() {
    return inputs.agitatorAppliedOutput;
  }

  public void stopAgitator() {
    lastAppliedAgitatorSpeed = 0.0;
    io.setAgitatorOutput(0.0);
    agitatorRunning = false;
  }

  public void setHopperExtensionSpeed(double speed) {
    if (calibrationModeEnabled) {
      return;
    }
    lastAppliedExtensionSpeed =
        applyScaleAndInversion(speed, hopperExtensionSpeedScaleEntry, hopperExtensionInvertedEntry);
    io.setExtensionOutput(lastAppliedExtensionSpeed);
  }

  public void stopHopperExtension() {
    lastAppliedExtensionSpeed = 0.0;
    io.setExtensionOutput(0.0);
  }

  public boolean isCalibrationModeEnabled() {
    return calibrationModeEnabled;
  }

  public void setCalibrationModeEnabled(boolean enabled) {
    calibrationModeEnabled = enabled;
    calibrationModeEnabledEntry.setBoolean(calibrationModeEnabled);
    if (!enabled) {
      stopAgitator();
      stopHopperExtension();
    }
  }

  public void setCalibrationAgitatorVelocitySetpointRotationsPerSec(
      double velocityRotationsPerSec) {
    calibrationAgitatorVelocitySetpointRotationsPerSec =
        sanitizeFinite(velocityRotationsPerSec, calibrationAgitatorVelocitySetpointRotationsPerSec);
    calibrationAgitatorVelocitySetpointEntry.setDouble(
        calibrationAgitatorVelocitySetpointRotationsPerSec);
  }

  public void setCalibrationExtensionPositionSetpointRotations(double positionRotations) {
    calibrationExtensionPositionSetpointRotations =
        clampToExtensionCalibrationRange(positionRotations);
    calibrationExtensionPositionSetpointEntry.setDouble(
        calibrationExtensionPositionSetpointRotations);
  }

  public Command enableCalibrationModeCommand() {
    return Commands.runOnce(() -> setCalibrationModeEnabled(true), this).ignoringDisable(true);
  }

  public Command disableCalibrationModeCommand() {
    return Commands.runOnce(() -> setCalibrationModeEnabled(false), this).ignoringDisable(true);
  }

  public Command toggleAgitatorCommand() {
    return runOnce(
        () -> {
          if (!agitatorRunning) {
            updateAgitator();
            currentAgitatorRunCommand = run(this::updateAgitator);
            CommandScheduler.getInstance().schedule(currentAgitatorRunCommand);
            agitatorRunning = true;
          } else {
            stopAgitator();
            stopCommand(currentAgitatorRunCommand);
            currentAgitatorRunCommand = null;
            agitatorRunning = false;
          }
        });
  }

  public Command increaseAgitatorSpeed() {
    return Commands.runOnce(
        () -> {
          targetAgitatorSpeed = Math.min(targetAgitatorSpeed + 0.05, 1.0);
          hopperAgitatorSpeedEntry.setDouble(targetAgitatorSpeed);
        });
  }

  public Command decreaseAgitatorSpeed() {
    return Commands.runOnce(
        () -> {
          targetAgitatorSpeed = Math.max(targetAgitatorSpeed - 0.05, 0.0);
          hopperAgitatorSpeedEntry.setDouble(targetAgitatorSpeed);
        });
  }

  private void configureNetworkTableDefaults() {
    hopperAgitatorSpeedEntry.setDefaultDouble(HopperConstants.defaultHopperAgitatorSpeed);
    hopperExtensionSpeedScaleEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionSpeedScale);
    hopperExtensionRetractedPositionEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionRetractedPositionRotations);
    hopperExtensionExtendedPositionEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionExtendedPositionRotations);
    hopperAgitatorDirectionEntry.setDefaultDouble(HopperConstants.defaultHopperAgitatorDirection);
    hopperExtensionInvertedEntry.setDefaultBoolean(HopperConstants.defaultHopperExtensionInverted);
    agitatorVelocityKpEntry.setDefaultDouble(HopperConstants.hopperAgitatorVelocityKp);
    agitatorVelocityKiEntry.setDefaultDouble(HopperConstants.hopperAgitatorVelocityKi);
    agitatorVelocityKdEntry.setDefaultDouble(HopperConstants.hopperAgitatorVelocityKd);
    agitatorVelocityKvEntry.setDefaultDouble(HopperConstants.hopperAgitatorVelocityKv);
    extensionPositionKpEntry.setDefaultDouble(HopperConstants.hopperExtensionPositionKp);
    extensionPositionKiEntry.setDefaultDouble(HopperConstants.hopperExtensionPositionKi);
    extensionPositionKdEntry.setDefaultDouble(HopperConstants.hopperExtensionPositionKd);
    calibrationModeEnabledEntry.setDefaultBoolean(false);
    calibrationAgitatorVelocitySetpointEntry.setDefaultDouble(
        HopperConstants.defaultCalibrationAgitatorVelocitySetpointRotationsPerSec);
    calibrationExtensionPositionSetpointEntry.setDefaultDouble(
        HopperConstants.defaultCalibrationExtensionPositionSetpointRotations);
  }

  private void loadNetworkTableConfig() {
    targetAgitatorSpeed = clampSpeed(hopperAgitatorSpeedEntry.getDouble(targetAgitatorSpeed));
    hopperAgitatorSpeedEntry.setDouble(targetAgitatorSpeed);
    hopperExtensionSpeedScaleEntry.setDouble(
        clampSpeedScale(
            hopperExtensionSpeedScaleEntry.getDouble(
                HopperConstants.defaultHopperExtensionSpeedScale)));
    hopperExtensionRetractedPositionRotations =
        hopperExtensionRetractedPositionEntry.getDouble(hopperExtensionRetractedPositionRotations);
    hopperExtensionExtendedPositionRotations =
        hopperExtensionExtendedPositionEntry.getDouble(hopperExtensionExtendedPositionRotations);
    hopperExtensionRetractedPositionEntry.setDouble(hopperExtensionRetractedPositionRotations);
    hopperExtensionExtendedPositionEntry.setDouble(hopperExtensionExtendedPositionRotations);
    double hopperAgitatorDirection =
        normalizeDirection(
            hopperAgitatorDirectionEntry.getDouble(HopperConstants.defaultHopperAgitatorDirection));
    hopperAgitatorDirectionEntry.setDouble(hopperAgitatorDirection);
    agitatorVelocityKp =
        sanitizeFinite(agitatorVelocityKpEntry.getDouble(agitatorVelocityKp), agitatorVelocityKp);
    agitatorVelocityKi =
        sanitizeFinite(agitatorVelocityKiEntry.getDouble(agitatorVelocityKi), agitatorVelocityKi);
    agitatorVelocityKd =
        sanitizeFinite(agitatorVelocityKdEntry.getDouble(agitatorVelocityKd), agitatorVelocityKd);
    agitatorVelocityKv =
        sanitizeFinite(agitatorVelocityKvEntry.getDouble(agitatorVelocityKv), agitatorVelocityKv);
    extensionPositionKp =
        sanitizeFinite(
            extensionPositionKpEntry.getDouble(extensionPositionKp), extensionPositionKp);
    extensionPositionKi =
        sanitizeFinite(
            extensionPositionKiEntry.getDouble(extensionPositionKi), extensionPositionKi);
    extensionPositionKd =
        sanitizeFinite(
            extensionPositionKdEntry.getDouble(extensionPositionKd), extensionPositionKd);
    calibrationModeEnabled = calibrationModeEnabledEntry.getBoolean(calibrationModeEnabled);
    calibrationAgitatorVelocitySetpointRotationsPerSec =
        sanitizeFinite(
            calibrationAgitatorVelocitySetpointEntry.getDouble(
                calibrationAgitatorVelocitySetpointRotationsPerSec),
            calibrationAgitatorVelocitySetpointRotationsPerSec);
    calibrationExtensionPositionSetpointRotations =
        clampToExtensionCalibrationRange(
            sanitizeFinite(
                calibrationExtensionPositionSetpointEntry.getDouble(
                    calibrationExtensionPositionSetpointRotations),
                calibrationExtensionPositionSetpointRotations));
    agitatorVelocityKpEntry.setDouble(agitatorVelocityKp);
    agitatorVelocityKiEntry.setDouble(agitatorVelocityKi);
    agitatorVelocityKdEntry.setDouble(agitatorVelocityKd);
    agitatorVelocityKvEntry.setDouble(agitatorVelocityKv);
    extensionPositionKpEntry.setDouble(extensionPositionKp);
    extensionPositionKiEntry.setDouble(extensionPositionKi);
    extensionPositionKdEntry.setDouble(extensionPositionKd);
    calibrationModeEnabledEntry.setBoolean(calibrationModeEnabled);
    calibrationAgitatorVelocitySetpointEntry.setDouble(
        calibrationAgitatorVelocitySetpointRotationsPerSec);
    calibrationExtensionPositionSetpointEntry.setDouble(
        calibrationExtensionPositionSetpointRotations);
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
    io.setAgitatorVelocitySetpointRotationsPerSec(
        calibrationAgitatorVelocitySetpointRotationsPerSec);
    io.setExtensionPositionSetpointRotations(calibrationExtensionPositionSetpointRotations);
    lastAppliedAgitatorSpeed = 0.0;
    lastAppliedExtensionSpeed = 0.0;
    agitatorRunning = Math.abs(calibrationAgitatorVelocitySetpointRotationsPerSec) > 1e-3;
  }

  private void publishExtensionEncoderToNetworkTables() {
    hopperExtensionEncoderPositionEntry.setDouble(getHopperExtensionMeasuredPositionRotations());
    hopperExtensionEncoderVelocityEntry.setDouble(inputs.extensionVelocityRpm);
    hopperExtensionEncoderNormalizedPositionEntry.setDouble(
        getHopperExtensionMeasuredPositionNormalized());
  }

  private void publishActuatorStateToNetworkTables() {
    hopperAgitatorAppliedOutputEntry.setDouble(inputs.agitatorAppliedOutput);
    hopperAgitatorEstimatedVelocityEntry.setDouble(inputs.agitatorVelocityRotationsPerSec);
    hopperAgitatorEstimatedPositionEntry.setDouble(inputs.agitatorPositionRotations);
    hopperExtensionAppliedOutputEntry.setDouble(inputs.extensionAppliedOutput);
  }

  private void publishCalibrationTelemetry() {
    calibrationModeTelemetryEntry.setBoolean(calibrationModeEnabled);
    calibrationConfiguredAgitatorVelocityEntry.setDouble(
        calibrationAgitatorVelocitySetpointRotationsPerSec);
    calibrationConfiguredExtensionPositionEntry.setDouble(
        calibrationExtensionPositionSetpointRotations);
    calibrationMeasuredAgitatorVelocityEntry.setDouble(inputs.agitatorVelocityRotationsPerSec);
    calibrationMeasuredExtensionPositionEntry.setDouble(inputs.extensionPositionRotations);
  }

  public double getHopperExtensionMeasuredPositionRotations() {
    return inputs.extensionPositionRotations;
  }

  public double getHopperAgitatorCurrentAmps() {
    return inputs.agitatorCurrentAmps;
  }

  public double getHopperExtensionCurrentAmps() {
    return inputs.extensionCurrentAmps;
  }

  public double getHopperExtensionMeasuredPositionNormalized() {
    return clampUnitInterval(
        inverseInterpolate(
            getHopperExtensionMeasuredPositionRotations(),
            hopperExtensionRetractedPositionRotations,
            hopperExtensionExtendedPositionRotations));
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

  private double clampToExtensionCalibrationRange(double positionRotations) {
    return MathUtil.clamp(
        positionRotations,
        hopperExtensionRetractedPositionRotations,
        hopperExtensionExtendedPositionRotations);
  }

  private void stopCommand(Command command) {
    if (command != null) {
      CommandScheduler.getInstance().cancel(command);
    }
  }

  private void logTelemetry() {
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Config/TargetAgitatorSpeed"),
        targetAgitatorSpeed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Config/ExtensionSpeedScale"),
        clampSpeedScale(
            hopperExtensionSpeedScaleEntry.getDouble(
                HopperConstants.defaultHopperExtensionSpeedScale)));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Config/AgitatorDirection"),
        normalizeDirection(
            hopperAgitatorDirectionEntry.getDouble(
                HopperConstants.defaultHopperAgitatorDirection)));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Config/ExtensionInverted"),
        hopperExtensionInvertedEntry.getBoolean(HopperConstants.defaultHopperExtensionInverted));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Config/ExtensionRetractedPositionRotations"),
        hopperExtensionRetractedPositionRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Config/ExtensionExtendedPositionRotations"),
        hopperExtensionExtendedPositionRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Calibration/ClosedLoop/Enabled"),
        calibrationModeEnabled);
    Logger.recordOutput(
        NetworkTablesUtil.logPath(
            "GamePiece/Hopper/Calibration/ClosedLoop/AgitatorVelocitySetpointRotationsPerSec"),
        calibrationAgitatorVelocitySetpointRotationsPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath(
            "GamePiece/Hopper/Calibration/ClosedLoop/ExtensionPositionSetpointRotations"),
        calibrationExtensionPositionSetpointRotations);

    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/State/AgitatorRunning"), agitatorRunning);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/State/LastAppliedAgitatorOutput"),
        lastAppliedAgitatorSpeed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/State/LastAppliedExtensionOutput"),
        lastAppliedExtensionSpeed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/State/ActualAgitatorOutput"),
        inputs.agitatorAppliedOutput);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/State/ActualExtensionOutput"),
        inputs.extensionAppliedOutput);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Measured/AgitatorCurrentAmps"),
        inputs.agitatorCurrentAmps);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Measured/ExtensionCurrentAmps"),
        inputs.extensionCurrentAmps);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Measured/ExtensionEncoderPositionRotations"),
        getHopperExtensionMeasuredPositionRotations());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Measured/ExtensionEncoderVelocityRpm"),
        inputs.extensionVelocityRpm);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Measured/ExtensionEncoderPositionNormalized"),
        getHopperExtensionMeasuredPositionNormalized());
  }
}
