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

public class Agitators extends SubsystemBase {
  private double targetTopAgitatorSpeed = AgitatorsConstants.defaultTopAgitatorSpeed;
  private double targetBottomAgitatorSpeed = AgitatorsConstants.defaultBottomAgitatorSpeed;
  private Command currentAgitatorRunCommand;
  private boolean agitatorRunning = false;

  private final SparkMax topAgitator =
      new SparkMax(AgitatorsConstants.topAgitatorCanId, MotorType.kBrushed);
  private final SparkMax bottomAgitator =
      new SparkMax(AgitatorsConstants.bottomAgitatorCanId, MotorType.kBrushed);

  private final NetworkTable configTable =
      NetworkTableInstance.getDefault().getTable(AgitatorsConstants.configTableName);
  private final NetworkTableEntry topAgitatorSpeedEntry = configTable.getEntry("TopAgitator/Speed");
  private final NetworkTableEntry bottomAgitatorSpeedEntry =
      configTable.getEntry("BottomAgitator/Speed");
  private final NetworkTableEntry topAgitatorInvertedEntry =
      configTable.getEntry("TopAgitator/Inverted");
  private final NetworkTableEntry bottomAgitatorInvertedEntry =
      configTable.getEntry("BottomAgitator/Inverted");

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
  }

  public void updateAgitators() {
    topAgitator.set(applyInversion(targetTopAgitatorSpeed, topAgitatorInvertedEntry));
    bottomAgitator.set(applyInversion(targetBottomAgitatorSpeed, bottomAgitatorInvertedEntry));
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
    topAgitator.set(0.0);
    bottomAgitator.set(0.0);
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

    topAgitatorInvertedEntry.setDefaultBoolean(AgitatorsConstants.defaultTopAgitatorInverted);
    bottomAgitatorInvertedEntry.setDefaultBoolean(AgitatorsConstants.defaultBottomAgitatorInverted);
  }

  private void loadNetworkTableConfig() {
    targetTopAgitatorSpeed = clampSpeed(topAgitatorSpeedEntry.getDouble(targetTopAgitatorSpeed));
    targetBottomAgitatorSpeed =
        clampSpeed(bottomAgitatorSpeedEntry.getDouble(targetBottomAgitatorSpeed));
  }

  private double applyInversion(double speed, NetworkTableEntry invertedEntry) {
    return invertedEntry.getBoolean(false) ? -speed : speed;
  }

  private double clampSpeed(double speed) {
    return MathUtil.clamp(speed, -1.0, 1.0);
  }

  private void stopCommand(Command command) {
    if (command != null) {
      CommandScheduler.getInstance().cancel(command);
    }
  }
}
