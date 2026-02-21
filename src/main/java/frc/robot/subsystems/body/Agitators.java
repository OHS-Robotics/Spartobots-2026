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

public class Agitators extends SubsystemBase {
  private static final double loopPeriodSeconds = 0.02;
  private static final double agitatorEstimatedMaxVelocityRotationsPerSec = 12.0;

  private double targetTopAgitatorSpeed = AgitatorsConstants.defaultTopAgitatorSpeed;
  private double targetBottomAgitatorSpeed = AgitatorsConstants.defaultBottomAgitatorSpeed;
  private Command currentAgitatorRunCommand;
  private boolean agitatorRunning = false;
  private double lastAppliedTopAgitatorSpeed = 0.0;
  private double lastAppliedBottomAgitatorSpeed = 0.0;
  private double topAgitatorEstimatedPositionRotations = 0.0;
  private double bottomAgitatorEstimatedPositionRotations = 0.0;

  private final SparkMax topAgitator =
      new SparkMax(AgitatorsConstants.topAgitatorCanId, MotorType.kBrushed);
  private final SparkMax bottomAgitator =
      new SparkMax(AgitatorsConstants.bottomAgitatorCanId, MotorType.kBrushed);

  private final NetworkTable subsystemTable =
      NetworkTableInstance.getDefault().getTable(AgitatorsConstants.configTableName);
  private final NetworkTable tuningTable = subsystemTable.getSubTable("Tuning");
  private final NetworkTable telemetryTable = subsystemTable.getSubTable("Telemetry");

  private final NetworkTableEntry topAgitatorSpeedEntry = tuningTable.getEntry("Top/Speed");
  private final NetworkTableEntry bottomAgitatorSpeedEntry = tuningTable.getEntry("Bottom/Speed");
  private final NetworkTableEntry topAgitatorDirectionEntry = tuningTable.getEntry("Top/Direction");
  private final NetworkTableEntry bottomAgitatorDirectionEntry =
      tuningTable.getEntry("Bottom/Direction");
  private final NetworkTableEntry topAgitatorAppliedOutputEntry =
      telemetryTable.getEntry("Top/AppliedOutput");
  private final NetworkTableEntry bottomAgitatorAppliedOutputEntry =
      telemetryTable.getEntry("Bottom/AppliedOutput");
  private final NetworkTableEntry topAgitatorEstimatedVelocityEntry =
      telemetryTable.getEntry("Top/EstimatedVelocityRotationsPerSec");
  private final NetworkTableEntry bottomAgitatorEstimatedVelocityEntry =
      telemetryTable.getEntry("Bottom/EstimatedVelocityRotationsPerSec");
  private final NetworkTableEntry topAgitatorEstimatedPositionEntry =
      telemetryTable.getEntry("Top/EstimatedPositionRotations");
  private final NetworkTableEntry bottomAgitatorEstimatedPositionEntry =
      telemetryTable.getEntry("Bottom/EstimatedPositionRotations");

  public Agitators() {
    SparkBaseConfig brakeConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

    configureNetworkTableDefaults();
    loadNetworkTableConfig();

    topAgitator.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    bottomAgitator.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    loadNetworkTableConfig();
    publishActuatorStateToNetworkTables();
    logTelemetry();
  }

  public void updateAgitators() {
    lastAppliedTopAgitatorSpeed =
        applyDirection(
            targetTopAgitatorSpeed,
            topAgitatorDirectionEntry,
            AgitatorsConstants.defaultTopAgitatorDirection);
    lastAppliedBottomAgitatorSpeed =
        applyDirection(
            targetBottomAgitatorSpeed,
            bottomAgitatorDirectionEntry,
            AgitatorsConstants.defaultBottomAgitatorDirection);
    topAgitator.set(lastAppliedTopAgitatorSpeed);
    bottomAgitator.set(lastAppliedBottomAgitatorSpeed);
    agitatorRunning =
        Math.abs(lastAppliedTopAgitatorSpeed) > 1e-3
            || Math.abs(lastAppliedBottomAgitatorSpeed) > 1e-3;
  }

  public void reverseAgitatorSpeed() {
    targetTopAgitatorSpeed *= -1;
    targetBottomAgitatorSpeed *= -1;
    topAgitatorSpeedEntry.setDouble(targetTopAgitatorSpeed);
    bottomAgitatorSpeedEntry.setDouble(targetBottomAgitatorSpeed);
  }

  public void setTargetAgitatorSpeed(double speed) {
    targetTopAgitatorSpeed = clampSpeed(speed);
    targetBottomAgitatorSpeed = clampSpeed(speed);
    topAgitatorSpeedEntry.setDouble(targetTopAgitatorSpeed);
    bottomAgitatorSpeedEntry.setDouble(targetBottomAgitatorSpeed);
  }

  public double getTargetAgitatorSpeed() {
    return (targetTopAgitatorSpeed + targetBottomAgitatorSpeed) / 2.0;
  }

  public void stopAgitators() {
    lastAppliedTopAgitatorSpeed = 0.0;
    lastAppliedBottomAgitatorSpeed = 0.0;
    topAgitator.set(0.0);
    bottomAgitator.set(0.0);
    agitatorRunning = false;
  }

  public Command toggleAgitatorCommand() {
    return runOnce(
        () -> {
          if (!agitatorRunning) {
            updateAgitators();
            currentAgitatorRunCommand = run(this::updateAgitators);
            CommandScheduler.getInstance().schedule(currentAgitatorRunCommand);
            agitatorRunning = true;
          } else {
            stopAgitators();
            stopCommand(currentAgitatorRunCommand);
            currentAgitatorRunCommand = null;
            agitatorRunning = false;
          }
        });
  }

  public Command increaseAgitatorSpeed() {
    return Commands.runOnce(
        () -> {
          targetTopAgitatorSpeed = Math.min(targetTopAgitatorSpeed + 0.05, 1.0);
          targetBottomAgitatorSpeed = Math.min(targetBottomAgitatorSpeed + 0.05, 1.0);
          topAgitatorSpeedEntry.setDouble(targetTopAgitatorSpeed);
          bottomAgitatorSpeedEntry.setDouble(targetBottomAgitatorSpeed);
        });
  }

  public Command decreaseAgitatorSpeed() {
    return Commands.runOnce(
        () -> {
          targetTopAgitatorSpeed = Math.max(targetTopAgitatorSpeed - 0.05, 0.0);
          targetBottomAgitatorSpeed = Math.max(targetBottomAgitatorSpeed - 0.05, 0.0);
          topAgitatorSpeedEntry.setDouble(targetTopAgitatorSpeed);
          bottomAgitatorSpeedEntry.setDouble(targetBottomAgitatorSpeed);
        });
  }

  private void configureNetworkTableDefaults() {
    topAgitatorSpeedEntry.setDefaultDouble(AgitatorsConstants.defaultTopAgitatorSpeed);
    bottomAgitatorSpeedEntry.setDefaultDouble(AgitatorsConstants.defaultBottomAgitatorSpeed);
    topAgitatorDirectionEntry.setDefaultDouble(AgitatorsConstants.defaultTopAgitatorDirection);
    bottomAgitatorDirectionEntry.setDefaultDouble(
        AgitatorsConstants.defaultBottomAgitatorDirection);
  }

  private void loadNetworkTableConfig() {
    targetTopAgitatorSpeed = clampSpeed(topAgitatorSpeedEntry.getDouble(targetTopAgitatorSpeed));
    targetBottomAgitatorSpeed =
        clampSpeed(bottomAgitatorSpeedEntry.getDouble(targetBottomAgitatorSpeed));
    topAgitatorSpeedEntry.setDouble(targetTopAgitatorSpeed);
    bottomAgitatorSpeedEntry.setDouble(targetBottomAgitatorSpeed);
    double topDirection =
        normalizeDirection(
            topAgitatorDirectionEntry.getDouble(AgitatorsConstants.defaultTopAgitatorDirection));
    double bottomDirection =
        normalizeDirection(
            bottomAgitatorDirectionEntry.getDouble(
                AgitatorsConstants.defaultBottomAgitatorDirection));
    topAgitatorDirectionEntry.setDouble(topDirection);
    bottomAgitatorDirectionEntry.setDouble(bottomDirection);
  }

  private double applyDirection(
      double speed, NetworkTableEntry directionEntry, double defaultDirection) {
    return clampSpeed(speed) * normalizeDirection(directionEntry.getDouble(defaultDirection));
  }

  private double clampSpeed(double speed) {
    return MathUtil.clamp(speed, -1.0, 1.0);
  }

  private static double normalizeDirection(double direction) {
    return direction < 0.0 ? -1.0 : 1.0;
  }

  public double getTopAgitatorCurrentAmps() {
    return topAgitator.getOutputCurrent();
  }

  public double getBottomAgitatorCurrentAmps() {
    return bottomAgitator.getOutputCurrent();
  }

  public double getAverageAgitatorCurrentAmps() {
    return 0.5 * (getTopAgitatorCurrentAmps() + getBottomAgitatorCurrentAmps());
  }

  public double getAverageAppliedOutput() {
    return 0.5 * (topAgitator.get() + bottomAgitator.get());
  }

  private void stopCommand(Command command) {
    if (command != null) {
      CommandScheduler.getInstance().cancel(command);
    }
  }

  private void publishActuatorStateToNetworkTables() {
    double topAppliedOutput = topAgitator.get();
    double bottomAppliedOutput = bottomAgitator.get();
    double topEstimatedVelocityRotationsPerSec =
        topAppliedOutput * agitatorEstimatedMaxVelocityRotationsPerSec;
    double bottomEstimatedVelocityRotationsPerSec =
        bottomAppliedOutput * agitatorEstimatedMaxVelocityRotationsPerSec;

    topAgitatorEstimatedPositionRotations +=
        topEstimatedVelocityRotationsPerSec * loopPeriodSeconds;
    bottomAgitatorEstimatedPositionRotations +=
        bottomEstimatedVelocityRotationsPerSec * loopPeriodSeconds;

    topAgitatorAppliedOutputEntry.setDouble(topAppliedOutput);
    bottomAgitatorAppliedOutputEntry.setDouble(bottomAppliedOutput);
    topAgitatorEstimatedVelocityEntry.setDouble(topEstimatedVelocityRotationsPerSec);
    bottomAgitatorEstimatedVelocityEntry.setDouble(bottomEstimatedVelocityRotationsPerSec);
    topAgitatorEstimatedPositionEntry.setDouble(topAgitatorEstimatedPositionRotations);
    bottomAgitatorEstimatedPositionEntry.setDouble(bottomAgitatorEstimatedPositionRotations);
  }

  private void logTelemetry() {
    Logger.recordOutput("Agitators/Config/TargetTopSpeed", targetTopAgitatorSpeed);
    Logger.recordOutput("Agitators/Config/TargetBottomSpeed", targetBottomAgitatorSpeed);
    Logger.recordOutput(
        "Agitators/Config/TopDirection",
        normalizeDirection(
            topAgitatorDirectionEntry.getDouble(AgitatorsConstants.defaultTopAgitatorDirection)));
    Logger.recordOutput(
        "Agitators/Config/BottomDirection",
        normalizeDirection(
            bottomAgitatorDirectionEntry.getDouble(
                AgitatorsConstants.defaultBottomAgitatorDirection)));

    Logger.recordOutput("Agitators/State/Running", agitatorRunning);
    Logger.recordOutput("Agitators/State/LastAppliedTopOutput", lastAppliedTopAgitatorSpeed);
    Logger.recordOutput("Agitators/State/LastAppliedBottomOutput", lastAppliedBottomAgitatorSpeed);
    Logger.recordOutput("Agitators/State/ActualTopOutput", topAgitator.get());
    Logger.recordOutput("Agitators/State/ActualBottomOutput", bottomAgitator.get());
    Logger.recordOutput("Agitators/Measured/TopCurrentAmps", topAgitator.getOutputCurrent());
    Logger.recordOutput("Agitators/Measured/BottomCurrentAmps", bottomAgitator.getOutputCurrent());
  }
}
