package frc.robot.subsystems.body;

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

  private final IndexersIO io;
  private final IndexersIOInputsAutoLogged inputs = new IndexersIOInputsAutoLogged();

  private final NetworkTable subsystemTable =
      NetworkTablesUtil.subsystemTable(IndexersConstants.configTableName);
  private final NetworkTable tuningTable = NetworkTablesUtil.tuningCommonTable(subsystemTable);
  private final NetworkTable telemetryTable = NetworkTablesUtil.telemetryTable(subsystemTable);

  private final NetworkTableEntry topIndexerSpeedEntry = tuningTable.getEntry("Top/Speed");
  private final NetworkTableEntry bottomIndexerSpeedEntry = tuningTable.getEntry("Bottom/Speed");
  private final NetworkTableEntry topIndexerSpeedScaleEntry =
      tuningTable.getEntry("Top/SpeedScale");
  private final NetworkTableEntry bottomIndexerSpeedScaleEntry =
      tuningTable.getEntry("Bottom/SpeedScale");
  private final NetworkTableEntry topIndexerDirectionEntry = tuningTable.getEntry("Top/Direction");
  private final NetworkTableEntry bottomIndexerDirectionEntry =
      tuningTable.getEntry("Bottom/Direction");
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
    Logger.processInputs("Indexers", inputs);
    loadNetworkTableConfig();
    publishActuatorStateToNetworkTables();
    logTelemetry();
  }

  public void updateIndexers() {
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
  }

  private double applyDirectionAndScale(
      double speed, double speedScale, NetworkTableEntry directionEntry, double defaultDirection) {
    double scaledSpeed = clampSpeed(speed) * clampSpeedScale(speedScale);
    return clampSpeed(scaledSpeed * normalizeDirection(directionEntry.getDouble(defaultDirection)));
  }

  private double clampSpeed(double speed) {
    return MathUtil.clamp(speed, -1.0, 1.0);
  }

  private double clampSpeedScale(double speedScale) {
    return MathUtil.clamp(speedScale, 0.0, 2.0);
  }

  private static double normalizeDirection(double direction) {
    return direction < 0.0 ? -1.0 : 1.0;
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

  private void logTelemetry() {
    Logger.recordOutput("Indexers/Config/TargetTopSpeed", targetTopIndexerSpeed);
    Logger.recordOutput("Indexers/Config/TargetBottomSpeed", targetBottomIndexerSpeed);
    Logger.recordOutput("Indexers/Config/TopSpeedScale", topIndexerSpeedScale);
    Logger.recordOutput("Indexers/Config/BottomSpeedScale", bottomIndexerSpeedScale);
    Logger.recordOutput(
        "Indexers/Config/TopDirection",
        normalizeDirection(
            topIndexerDirectionEntry.getDouble(IndexersConstants.defaultTopIndexerDirection)));
    Logger.recordOutput(
        "Indexers/Config/BottomDirection",
        normalizeDirection(
            bottomIndexerDirectionEntry.getDouble(
                IndexersConstants.defaultBottomIndexerDirection)));

    Logger.recordOutput("Indexers/State/Running", indexerRunning);
    Logger.recordOutput("Indexers/State/LastAppliedTopOutput", lastAppliedTopIndexerSpeed);
    Logger.recordOutput("Indexers/State/LastAppliedBottomOutput", lastAppliedBottomIndexerSpeed);
    Logger.recordOutput("Indexers/State/ActualTopOutput", inputs.topAppliedOutput);
    Logger.recordOutput("Indexers/State/ActualBottomOutput", inputs.bottomAppliedOutput);
    Logger.recordOutput("Indexers/Measured/TopCurrentAmps", inputs.topCurrentAmps);
    Logger.recordOutput("Indexers/Measured/BottomCurrentAmps", inputs.bottomCurrentAmps);
  }
}
