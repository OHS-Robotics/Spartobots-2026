package frc.robot.subsystems.gamepiece.indexers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NetworkTablesUtil;
import org.littletonrobotics.junction.Logger;

public class Indexers extends SubsystemBase {
  private double targetTopIndexerSpeed = IndexersConstants.defaultTopIndexerSpeed;
  private double targetBottomIndexerSpeed = IndexersConstants.defaultBottomIndexerSpeed;
  private double topIndexerSpeedScale = IndexersConstants.defaultTopIndexerSpeedScale;
  private double bottomIndexerSpeedScale = IndexersConstants.defaultBottomIndexerSpeedScale;
  private Command currentIndexerRunCommand;
  private boolean indexerRunning = false;
  private double lastAppliedTopIndexerSpeed = 0.0;
  private double lastAppliedBottomIndexerSpeed = 0.0;
  private boolean calibrationModeEnabled = false;
  private double calibrationTopVelocitySetpointRotationsPerSec =
      IndexersConstants.defaultCalibrationTopVelocitySetpointRotationsPerSec;
  private double calibrationBottomVelocitySetpointRotationsPerSec =
      IndexersConstants.defaultCalibrationBottomVelocitySetpointRotationsPerSec;
  private double velocityKp = IndexersConstants.indexerVelocityKp;
  private double velocityKi = IndexersConstants.indexerVelocityKi;
  private double velocityKd = IndexersConstants.indexerVelocityKd;
  private double velocityKv = IndexersConstants.indexerVelocityKv;

  private final IndexersIO io;
  private final IndexersIOInputsAutoLogged inputs = new IndexersIOInputsAutoLogged();

  private final NetworkTable subsystemTable =
      NetworkTablesUtil.domain(IndexersConstants.configTableName);
  private final NetworkTable tuningTable = NetworkTablesUtil.tuningCommon(subsystemTable);
  private final NetworkTable pidTuningTable =
      NetworkTablesUtil.tuningMode(subsystemTable).getSubTable("PID");
  private final NetworkTable telemetryTable = NetworkTablesUtil.telemetry(subsystemTable);
  private final NetworkTable calibrationTuningTable = tuningTable.getSubTable("Calibration");
  private final NetworkTable calibrationTelemetryTable = telemetryTable.getSubTable("Calibration");

  private final NetworkTableEntry topIndexerSpeedEntry = tuningTable.getEntry("Top/Speed");
  private final NetworkTableEntry bottomIndexerSpeedEntry = tuningTable.getEntry("Bottom/Speed");
  private final NetworkTableEntry topIndexerSpeedScaleEntry =
      tuningTable.getEntry("Top/SpeedScale");
  private final NetworkTableEntry bottomIndexerSpeedScaleEntry =
      tuningTable.getEntry("Bottom/SpeedScale");
  private final NetworkTableEntry topIndexerDirectionEntry = tuningTable.getEntry("Top/Direction");
  private final NetworkTableEntry bottomIndexerDirectionEntry =
      tuningTable.getEntry("Bottom/Direction");
  private final NetworkTableEntry velocityKpEntry = pidTuningTable.getEntry("Velocity/Kp");
  private final NetworkTableEntry velocityKiEntry = pidTuningTable.getEntry("Velocity/Ki");
  private final NetworkTableEntry velocityKdEntry = pidTuningTable.getEntry("Velocity/Kd");
  private final NetworkTableEntry velocityKvEntry = pidTuningTable.getEntry("Velocity/Kv");
  private final NetworkTableEntry calibrationModeEnabledEntry =
      calibrationTuningTable.getEntry("Enabled");
  private final NetworkTableEntry calibrationTopVelocitySetpointEntry =
      calibrationTuningTable.getEntry("Top/VelocitySetpointRotationsPerSec");
  private final NetworkTableEntry calibrationBottomVelocitySetpointEntry =
      calibrationTuningTable.getEntry("Bottom/VelocitySetpointRotationsPerSec");
  private final NetworkTableEntry topIndexerAppliedOutputEntry =
      telemetryTable.getEntry("Top/AppliedOutput");
  private final NetworkTableEntry bottomIndexerAppliedOutputEntry =
      telemetryTable.getEntry("Bottom/AppliedOutput");
  private final NetworkTableEntry topIndexerEstimatedVelocityEntry =
      telemetryTable.getEntry("Top/EstimatedVelocityRotationsPerSec");
  private final NetworkTableEntry bottomIndexerEstimatedVelocityEntry =
      telemetryTable.getEntry("Bottom/EstimatedVelocityRotationsPerSec");
  private final NetworkTableEntry topIndexerEstimatedPositionEntry =
      telemetryTable.getEntry("Top/EstimatedPositionRotations");
  private final NetworkTableEntry bottomIndexerEstimatedPositionEntry =
      telemetryTable.getEntry("Bottom/EstimatedPositionRotations");
  private final NetworkTableEntry calibrationModeTelemetryEntry =
      calibrationTelemetryTable.getEntry("ModeEnabled");
  private final NetworkTableEntry calibrationConfiguredTopVelocityEntry =
      calibrationTelemetryTable.getEntry("Configured/Top/VelocitySetpointRotationsPerSec");
  private final NetworkTableEntry calibrationConfiguredBottomVelocityEntry =
      calibrationTelemetryTable.getEntry("Configured/Bottom/VelocitySetpointRotationsPerSec");
  private final NetworkTableEntry calibrationMeasuredTopVelocityEntry =
      calibrationTelemetryTable.getEntry("Measured/Top/VelocityRotationsPerSec");
  private final NetworkTableEntry calibrationMeasuredBottomVelocityEntry =
      calibrationTelemetryTable.getEntry("Measured/Bottom/VelocityRotationsPerSec");

  public Indexers() {
    this(new IndexersIO() {});
  }

  public Indexers(IndexersIO io) {
    this.io = io;
    configureNetworkTableDefaults();
    loadNetworkTableConfig();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(NetworkTablesUtil.logPath("GamePiece/Indexers/Inputs"), inputs);
    loadNetworkTableConfig();
    io.setVelocityClosedLoopGains(velocityKp, velocityKi, velocityKd, velocityKv);
    if (calibrationModeEnabled) {
      applyCalibrationControl();
    }
    publishActuatorStateToNetworkTables();
    publishCalibrationTelemetry();
    logTelemetry();
  }

  public void updateIndexers() {
    if (calibrationModeEnabled) {
      return;
    }
    lastAppliedTopIndexerSpeed =
        applyDirectionAndScale(
            targetTopIndexerSpeed,
            topIndexerSpeedScale,
            topIndexerDirectionEntry,
            IndexersConstants.defaultTopIndexerDirection);
    lastAppliedBottomIndexerSpeed =
        applyDirectionAndScale(
            targetBottomIndexerSpeed,
            bottomIndexerSpeedScale,
            bottomIndexerDirectionEntry,
            IndexersConstants.defaultBottomIndexerDirection);
    io.setTopOutput(lastAppliedTopIndexerSpeed);
    io.setBottomOutput(lastAppliedBottomIndexerSpeed);
    indexerRunning =
        Math.abs(lastAppliedTopIndexerSpeed) > 1e-3
            || Math.abs(lastAppliedBottomIndexerSpeed) > 1e-3;
  }

  public void reverseIndexerSpeed() {
    targetTopIndexerSpeed *= -1;
    targetBottomIndexerSpeed *= -1;
    topIndexerSpeedEntry.setDouble(targetTopIndexerSpeed);
    bottomIndexerSpeedEntry.setDouble(targetBottomIndexerSpeed);
  }

  public void setTargetIndexerSpeed(double speed) {
    targetTopIndexerSpeed = clampSpeed(speed);
    targetBottomIndexerSpeed = clampSpeed(speed);
    topIndexerSpeedEntry.setDouble(targetTopIndexerSpeed);
    bottomIndexerSpeedEntry.setDouble(targetBottomIndexerSpeed);
  }

  public void setTargetTopIndexerSpeed(double speed) {
    targetTopIndexerSpeed = clampSpeed(speed);
    topIndexerSpeedEntry.setDouble(targetTopIndexerSpeed);
  }

  public void setTargetBottomIndexerSpeed(double speed) {
    targetBottomIndexerSpeed = clampSpeed(speed);
    bottomIndexerSpeedEntry.setDouble(targetBottomIndexerSpeed);
  }

  public void setManualOutputs(double topOutput, double bottomOutput) {
    if (calibrationModeEnabled) {
      return;
    }
    lastAppliedTopIndexerSpeed =
        applyDirection(
            topOutput, topIndexerDirectionEntry, IndexersConstants.defaultTopIndexerDirection);
    lastAppliedBottomIndexerSpeed =
        applyDirection(
            bottomOutput,
            bottomIndexerDirectionEntry,
            IndexersConstants.defaultBottomIndexerDirection);
    io.setTopOutput(lastAppliedTopIndexerSpeed);
    io.setBottomOutput(lastAppliedBottomIndexerSpeed);
    indexerRunning =
        Math.abs(lastAppliedTopIndexerSpeed) > 1e-3
            || Math.abs(lastAppliedBottomIndexerSpeed) > 1e-3;
  }

  public boolean isCalibrationModeEnabled() {
    return calibrationModeEnabled;
  }

  public void setCalibrationModeEnabled(boolean enabled) {
    calibrationModeEnabled = enabled;
    calibrationModeEnabledEntry.setBoolean(calibrationModeEnabled);
    if (!enabled) {
      stopIndexers();
    }
  }

  public void setCalibrationVelocitySetpointsRotationsPerSec(
      double topVelocityRotationsPerSec, double bottomVelocityRotationsPerSec) {
    calibrationTopVelocitySetpointRotationsPerSec =
        sanitizeFinite(topVelocityRotationsPerSec, calibrationTopVelocitySetpointRotationsPerSec);
    calibrationBottomVelocitySetpointRotationsPerSec =
        sanitizeFinite(
            bottomVelocityRotationsPerSec, calibrationBottomVelocitySetpointRotationsPerSec);
    calibrationTopVelocitySetpointEntry.setDouble(calibrationTopVelocitySetpointRotationsPerSec);
    calibrationBottomVelocitySetpointEntry.setDouble(
        calibrationBottomVelocitySetpointRotationsPerSec);
  }

  public Command enableCalibrationModeCommand() {
    return Commands.runOnce(() -> setCalibrationModeEnabled(true), this).ignoringDisable(true);
  }

  public Command disableCalibrationModeCommand() {
    return Commands.runOnce(() -> setCalibrationModeEnabled(false), this).ignoringDisable(true);
  }

  public double getTargetIndexerSpeed() {
    return (targetTopIndexerSpeed + targetBottomIndexerSpeed) / 2.0;
  }

  public void stopIndexers() {
    lastAppliedTopIndexerSpeed = 0.0;
    lastAppliedBottomIndexerSpeed = 0.0;
    io.setTopOutput(0.0);
    io.setBottomOutput(0.0);
    indexerRunning = false;
  }

  public Command toggleIndexerCommand() {
    return runOnce(
        () -> {
          if (!indexerRunning) {
            updateIndexers();
            currentIndexerRunCommand = run(this::updateIndexers);
            CommandScheduler.getInstance().schedule(currentIndexerRunCommand);
            indexerRunning = true;
          } else {
            stopIndexers();
            stopCommand(currentIndexerRunCommand);
            currentIndexerRunCommand = null;
            indexerRunning = false;
          }
        });
  }

  public Command increaseIndexerSpeed() {
    return Commands.runOnce(
        () -> {
          targetTopIndexerSpeed = Math.min(targetTopIndexerSpeed + 0.05, 1.0);
          targetBottomIndexerSpeed = Math.min(targetBottomIndexerSpeed + 0.05, 1.0);
          topIndexerSpeedEntry.setDouble(targetTopIndexerSpeed);
          bottomIndexerSpeedEntry.setDouble(targetBottomIndexerSpeed);
        });
  }

  public Command decreaseIndexerSpeed() {
    return Commands.runOnce(
        () -> {
          targetTopIndexerSpeed = Math.max(targetTopIndexerSpeed - 0.05, 0.0);
          targetBottomIndexerSpeed = Math.max(targetBottomIndexerSpeed - 0.05, 0.0);
          topIndexerSpeedEntry.setDouble(targetTopIndexerSpeed);
          bottomIndexerSpeedEntry.setDouble(targetBottomIndexerSpeed);
        });
  }

  private void configureNetworkTableDefaults() {
    topIndexerSpeedEntry.setDefaultDouble(IndexersConstants.defaultTopIndexerSpeed);
    bottomIndexerSpeedEntry.setDefaultDouble(IndexersConstants.defaultBottomIndexerSpeed);
    topIndexerSpeedScaleEntry.setDefaultDouble(IndexersConstants.defaultTopIndexerSpeedScale);
    bottomIndexerSpeedScaleEntry.setDefaultDouble(IndexersConstants.defaultBottomIndexerSpeedScale);
    topIndexerDirectionEntry.setDefaultDouble(IndexersConstants.defaultTopIndexerDirection);
    bottomIndexerDirectionEntry.setDefaultDouble(IndexersConstants.defaultBottomIndexerDirection);
    velocityKpEntry.setDefaultDouble(IndexersConstants.indexerVelocityKp);
    velocityKiEntry.setDefaultDouble(IndexersConstants.indexerVelocityKi);
    velocityKdEntry.setDefaultDouble(IndexersConstants.indexerVelocityKd);
    velocityKvEntry.setDefaultDouble(IndexersConstants.indexerVelocityKv);
    calibrationModeEnabledEntry.setDefaultBoolean(false);
    calibrationTopVelocitySetpointEntry.setDefaultDouble(
        IndexersConstants.defaultCalibrationTopVelocitySetpointRotationsPerSec);
    calibrationBottomVelocitySetpointEntry.setDefaultDouble(
        IndexersConstants.defaultCalibrationBottomVelocitySetpointRotationsPerSec);
  }

  private void loadNetworkTableConfig() {
    targetTopIndexerSpeed = clampSpeed(topIndexerSpeedEntry.getDouble(targetTopIndexerSpeed));
    targetBottomIndexerSpeed =
        clampSpeed(bottomIndexerSpeedEntry.getDouble(targetBottomIndexerSpeed));
    topIndexerSpeedEntry.setDouble(targetTopIndexerSpeed);
    bottomIndexerSpeedEntry.setDouble(targetBottomIndexerSpeed);
    topIndexerSpeedScale =
        clampSpeedScale(
            topIndexerSpeedScaleEntry.getDouble(IndexersConstants.defaultTopIndexerSpeedScale));
    bottomIndexerSpeedScale =
        clampSpeedScale(
            bottomIndexerSpeedScaleEntry.getDouble(
                IndexersConstants.defaultBottomIndexerSpeedScale));
    topIndexerSpeedScaleEntry.setDouble(topIndexerSpeedScale);
    bottomIndexerSpeedScaleEntry.setDouble(bottomIndexerSpeedScale);
    double topDirection =
        normalizeDirection(
            topIndexerDirectionEntry.getDouble(IndexersConstants.defaultTopIndexerDirection));
    double bottomDirection =
        normalizeDirection(
            bottomIndexerDirectionEntry.getDouble(IndexersConstants.defaultBottomIndexerDirection));
    topIndexerDirectionEntry.setDouble(topDirection);
    bottomIndexerDirectionEntry.setDouble(bottomDirection);
    velocityKp = sanitizeFinite(velocityKpEntry.getDouble(velocityKp), velocityKp);
    velocityKi = sanitizeFinite(velocityKiEntry.getDouble(velocityKi), velocityKi);
    velocityKd = sanitizeFinite(velocityKdEntry.getDouble(velocityKd), velocityKd);
    velocityKv = sanitizeFinite(velocityKvEntry.getDouble(velocityKv), velocityKv);
    calibrationModeEnabled = calibrationModeEnabledEntry.getBoolean(calibrationModeEnabled);
    calibrationTopVelocitySetpointRotationsPerSec =
        sanitizeFinite(
            calibrationTopVelocitySetpointEntry.getDouble(
                calibrationTopVelocitySetpointRotationsPerSec),
            calibrationTopVelocitySetpointRotationsPerSec);
    calibrationBottomVelocitySetpointRotationsPerSec =
        sanitizeFinite(
            calibrationBottomVelocitySetpointEntry.getDouble(
                calibrationBottomVelocitySetpointRotationsPerSec),
            calibrationBottomVelocitySetpointRotationsPerSec);
    velocityKpEntry.setDouble(velocityKp);
    velocityKiEntry.setDouble(velocityKi);
    velocityKdEntry.setDouble(velocityKd);
    velocityKvEntry.setDouble(velocityKv);
    calibrationModeEnabledEntry.setBoolean(calibrationModeEnabled);
    calibrationTopVelocitySetpointEntry.setDouble(calibrationTopVelocitySetpointRotationsPerSec);
    calibrationBottomVelocitySetpointEntry.setDouble(
        calibrationBottomVelocitySetpointRotationsPerSec);
  }

  private double applyDirectionAndScale(
      double speed, double speedScale, NetworkTableEntry directionEntry, double defaultDirection) {
    double scaledSpeed = clampSpeed(speed) * clampSpeedScale(speedScale);
    return clampSpeed(scaledSpeed * normalizeDirection(directionEntry.getDouble(defaultDirection)));
  }

  private double applyDirection(
      double speed, NetworkTableEntry directionEntry, double defaultDirection) {
    return clampSpeed(speed * normalizeDirection(directionEntry.getDouble(defaultDirection)));
  }

  private double clampSpeed(double speed) {
    return MathUtil.clamp(speed, -1.0, 1.0);
  }

  private double clampSpeedScale(double speedScale) {
    return MathUtil.clamp(speedScale, 0.0, 2.0);
  }

  private void applyCalibrationControl() {
    io.setTopVelocitySetpointRotationsPerSec(calibrationTopVelocitySetpointRotationsPerSec);
    io.setBottomVelocitySetpointRotationsPerSec(calibrationBottomVelocitySetpointRotationsPerSec);
    lastAppliedTopIndexerSpeed = 0.0;
    lastAppliedBottomIndexerSpeed = 0.0;
    indexerRunning =
        Math.abs(calibrationTopVelocitySetpointRotationsPerSec) > 1e-3
            || Math.abs(calibrationBottomVelocitySetpointRotationsPerSec) > 1e-3;
  }

  private static double normalizeDirection(double direction) {
    return direction < 0.0 ? -1.0 : 1.0;
  }

  private static double sanitizeFinite(double value, double fallback) {
    return Double.isFinite(value) ? value : fallback;
  }

  public double getTopIndexerCurrentAmps() {
    return inputs.topCurrentAmps;
  }

  public double getBottomIndexerCurrentAmps() {
    return inputs.bottomCurrentAmps;
  }

  public double getAverageIndexerCurrentAmps() {
    return 0.5 * (getTopIndexerCurrentAmps() + getBottomIndexerCurrentAmps());
  }

  public double getAverageAppliedOutput() {
    return 0.5 * (inputs.topAppliedOutput + inputs.bottomAppliedOutput);
  }

  public double getAverageAppliedOutputMagnitude() {
    return 0.5 * (Math.abs(inputs.topAppliedOutput) + Math.abs(inputs.bottomAppliedOutput));
  }

  public void resetSimulationState() {
    io.resetSimulationState();
  }

  private void stopCommand(Command command) {
    if (command != null) {
      CommandScheduler.getInstance().cancel(command);
    }
  }

  private void publishActuatorStateToNetworkTables() {
    topIndexerAppliedOutputEntry.setDouble(inputs.topAppliedOutput);
    bottomIndexerAppliedOutputEntry.setDouble(inputs.bottomAppliedOutput);
    topIndexerEstimatedVelocityEntry.setDouble(inputs.topVelocityRotationsPerSec);
    bottomIndexerEstimatedVelocityEntry.setDouble(inputs.bottomVelocityRotationsPerSec);
    topIndexerEstimatedPositionEntry.setDouble(inputs.topPositionRotations);
    bottomIndexerEstimatedPositionEntry.setDouble(inputs.bottomPositionRotations);
  }

  private void publishCalibrationTelemetry() {
    calibrationModeTelemetryEntry.setBoolean(calibrationModeEnabled);
    calibrationConfiguredTopVelocityEntry.setDouble(calibrationTopVelocitySetpointRotationsPerSec);
    calibrationConfiguredBottomVelocityEntry.setDouble(
        calibrationBottomVelocitySetpointRotationsPerSec);
    calibrationMeasuredTopVelocityEntry.setDouble(inputs.topVelocityRotationsPerSec);
    calibrationMeasuredBottomVelocityEntry.setDouble(inputs.bottomVelocityRotationsPerSec);
  }

  private void logTelemetry() {
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/Config/TargetTopSpeed"),
        targetTopIndexerSpeed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/Config/TargetBottomSpeed"),
        targetBottomIndexerSpeed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/Config/TopSpeedScale"), topIndexerSpeedScale);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/Config/BottomSpeedScale"),
        bottomIndexerSpeedScale);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/Config/TopDirection"),
        normalizeDirection(
            topIndexerDirectionEntry.getDouble(IndexersConstants.defaultTopIndexerDirection)));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/Config/BottomDirection"),
        normalizeDirection(
            bottomIndexerDirectionEntry.getDouble(
                IndexersConstants.defaultBottomIndexerDirection)));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/Calibration/ClosedLoop/Enabled"),
        calibrationModeEnabled);
    Logger.recordOutput(
        NetworkTablesUtil.logPath(
            "GamePiece/Indexers/Calibration/ClosedLoop/TopVelocitySetpointRotationsPerSec"),
        calibrationTopVelocitySetpointRotationsPerSec);
    Logger.recordOutput(
        NetworkTablesUtil.logPath(
            "GamePiece/Indexers/Calibration/ClosedLoop/BottomVelocitySetpointRotationsPerSec"),
        calibrationBottomVelocitySetpointRotationsPerSec);

    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/State/Running"), indexerRunning);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/State/LastAppliedTopOutput"),
        lastAppliedTopIndexerSpeed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/State/LastAppliedBottomOutput"),
        lastAppliedBottomIndexerSpeed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/State/ActualTopOutput"),
        inputs.topAppliedOutput);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/State/ActualBottomOutput"),
        inputs.bottomAppliedOutput);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/Measured/TopCurrentAmps"),
        inputs.topCurrentAmps);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Indexers/Measured/BottomCurrentAmps"),
        inputs.bottomCurrentAmps);
  }
}
