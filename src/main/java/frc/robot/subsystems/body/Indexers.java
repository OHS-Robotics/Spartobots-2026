package frc.robot.subsystems.body;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexers extends SubsystemBase {
  private static final double loopPeriodSeconds = 0.02;
  private static final double indexerEstimatedMaxVelocityRotationsPerSec = 12.0;

  private double targetTopIndexerSpeed = IndexersConstants.defaultTopIndexerSpeed;
  private double targetBottomIndexerSpeed = IndexersConstants.defaultBottomIndexerSpeed;
  private double topIndexerSpeedScale = IndexersConstants.defaultTopIndexerSpeedScale;
  private double bottomIndexerSpeedScale = IndexersConstants.defaultBottomIndexerSpeedScale;
  private Command currentIndexerRunCommand;
  private boolean indexerRunning = false;
  private double lastAppliedTopIndexerSpeed = 0.0;
  private double lastAppliedBottomIndexerSpeed = 0.0;
  private double topIndexerEstimatedPositionRotations = 0.0;
  private double bottomIndexerEstimatedPositionRotations = 0.0;

  private final SparkMax topIndexer =
      new SparkMax(IndexersConstants.topIndexerCanId, MotorType.kBrushed);
  private final SparkMax bottomIndexer =
      new SparkMax(IndexersConstants.bottomIndexerCanId, MotorType.kBrushed);

  private final NetworkTable subsystemTable =
      NetworkTableInstance.getDefault().getTable(IndexersConstants.configTableName);
  private final NetworkTable tuningTable = subsystemTable.getSubTable("Tuning");
  private final NetworkTable telemetryTable = subsystemTable.getSubTable("Telemetry");

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
    SparkBaseConfig brakeConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

    configureNetworkTableDefaults();
    loadNetworkTableConfig();

    topIndexer.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    bottomIndexer.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
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
    topIndexer.set(lastAppliedTopIndexerSpeed);
    bottomIndexer.set(lastAppliedBottomIndexerSpeed);
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

  public double getTargetIndexerSpeed() {
    return (targetTopIndexerSpeed + targetBottomIndexerSpeed) / 2.0;
  }

  public void stopIndexers() {
    lastAppliedTopIndexerSpeed = 0.0;
    lastAppliedBottomIndexerSpeed = 0.0;
    topIndexer.set(0.0);
    bottomIndexer.set(0.0);
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
    return topIndexer.getOutputCurrent();
  }

  public double getBottomIndexerCurrentAmps() {
    return bottomIndexer.getOutputCurrent();
  }

  public double getAverageIndexerCurrentAmps() {
    return 0.5 * (getTopIndexerCurrentAmps() + getBottomIndexerCurrentAmps());
  }

  public double getAverageAppliedOutput() {
    return 0.5 * (topIndexer.get() + bottomIndexer.get());
  }

  private void stopCommand(Command command) {
    if (command != null) {
      CommandScheduler.getInstance().cancel(command);
    }
  }

  private void publishActuatorStateToNetworkTables() {
    double topAppliedOutput = topIndexer.get();
    double bottomAppliedOutput = bottomIndexer.get();
    double topEstimatedVelocityRotationsPerSec =
        topAppliedOutput * indexerEstimatedMaxVelocityRotationsPerSec;
    double bottomEstimatedVelocityRotationsPerSec =
        bottomAppliedOutput * indexerEstimatedMaxVelocityRotationsPerSec;

    topIndexerEstimatedPositionRotations += topEstimatedVelocityRotationsPerSec * loopPeriodSeconds;
    bottomIndexerEstimatedPositionRotations +=
        bottomEstimatedVelocityRotationsPerSec * loopPeriodSeconds;

    topIndexerAppliedOutputEntry.setDouble(topAppliedOutput);
    bottomIndexerAppliedOutputEntry.setDouble(bottomAppliedOutput);
    topIndexerEstimatedVelocityEntry.setDouble(topEstimatedVelocityRotationsPerSec);
    bottomIndexerEstimatedVelocityEntry.setDouble(bottomEstimatedVelocityRotationsPerSec);
    topIndexerEstimatedPositionEntry.setDouble(topIndexerEstimatedPositionRotations);
    bottomIndexerEstimatedPositionEntry.setDouble(bottomIndexerEstimatedPositionRotations);
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
    Logger.recordOutput("Indexers/State/ActualTopOutput", topIndexer.get());
    Logger.recordOutput("Indexers/State/ActualBottomOutput", bottomIndexer.get());
    Logger.recordOutput("Indexers/Measured/TopCurrentAmps", topIndexer.getOutputCurrent());
    Logger.recordOutput("Indexers/Measured/BottomCurrentAmps", bottomIndexer.getOutputCurrent());
  }
}
