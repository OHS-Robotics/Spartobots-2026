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

public class Hopper extends SubsystemBase {
  private double targetBeltSpeed = HopperConstants.defaultHopperBeltSpeed;
  private Command currentBeltRunCommand;
  private boolean beltRunning = false;

  private final SparkMax hopperBelt =
      new SparkMax(HopperConstants.hopperBeltDriveCanId, MotorType.kBrushed);
  private final SparkMax hopperExtension =
      new SparkMax(HopperConstants.hopperExtensionCanId, MotorType.kBrushed);

  private final NetworkTable configTable =
      NetworkTableInstance.getDefault().getTable(HopperConstants.configTableName);
  private final NetworkTableEntry hopperBeltSpeedEntry = configTable.getEntry("HopperBelt/Speed");
  private final NetworkTableEntry hopperExtensionSpeedScaleEntry =
      configTable.getEntry("HopperExtension/SpeedScale");
  private final NetworkTableEntry hopperBeltInvertedEntry =
      configTable.getEntry("HopperBelt/Inverted");
  private final NetworkTableEntry hopperExtensionInvertedEntry =
      configTable.getEntry("HopperExtension/Inverted");

  public Hopper() {
    SparkBaseConfig brakeConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

    configureNetworkTableDefaults();
    loadNetworkTableConfig();

    hopperBelt.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    hopperExtension.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    loadNetworkTableConfig();
  }

  public void updateBelt() {
    hopperBelt.set(applyInversion(targetBeltSpeed, hopperBeltInvertedEntry));
  }

  public void reverseBeltSpeed() {
    targetBeltSpeed *= -1;
    hopperBeltSpeedEntry.setDouble(targetBeltSpeed);
  }

  public void setTargetBeltSpeed(double speed) {
    targetBeltSpeed = clampSpeed(speed);
    hopperBeltSpeedEntry.setDouble(targetBeltSpeed);
  }

  public double getTargetBeltSpeed() {
    return targetBeltSpeed;
  }

  public double getActualBeltSpeed() {
    return hopperBelt.get();
  }

  public void stopBelt() {
    hopperBelt.set(0.0);
  }

  public void setHopperExtensionSpeed(double speed) {
    hopperExtension.set(
        applyScaleAndInversion(
            speed, hopperExtensionSpeedScaleEntry, hopperExtensionInvertedEntry));
  }

  public void stopHopperExtension() {
    hopperExtension.set(0.0);
  }

  public Command toggleBeltCommand() {
    return runOnce(
        () -> {
          if (!beltRunning) {
            updateBelt();
            currentBeltRunCommand = run(this::updateBelt);
            CommandScheduler.getInstance().schedule(currentBeltRunCommand);
            beltRunning = true;
          } else {
            stopBelt();
            stopCommand(currentBeltRunCommand);
            currentBeltRunCommand = null;
            beltRunning = false;
          }
        });
  }

  public Command increaseBeltSpeed() {
    return Commands.runOnce(
        () -> {
          targetBeltSpeed = Math.min(targetBeltSpeed + 0.05, 1.0);
          hopperBeltSpeedEntry.setDouble(targetBeltSpeed);
        });
  }

  public Command decreaseBeltSpeed() {
    return Commands.runOnce(
        () -> {
          targetBeltSpeed = Math.max(targetBeltSpeed - 0.05, 0.0);
          hopperBeltSpeedEntry.setDouble(targetBeltSpeed);
        });
  }

  private void configureNetworkTableDefaults() {
    hopperBeltSpeedEntry.setDefaultDouble(HopperConstants.defaultHopperBeltSpeed);
    hopperExtensionSpeedScaleEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionSpeedScale);

    hopperBeltInvertedEntry.setDefaultBoolean(HopperConstants.defaultHopperBeltInverted);
    hopperExtensionInvertedEntry.setDefaultBoolean(HopperConstants.defaultHopperExtensionInverted);
  }

  private void loadNetworkTableConfig() {
    targetBeltSpeed = clampSpeed(hopperBeltSpeedEntry.getDouble(targetBeltSpeed));
  }

  private double applyScaleAndInversion(
      double speed, NetworkTableEntry speedScaleEntry, NetworkTableEntry invertedEntry) {
    double scaledSpeed = clampSpeed(speed) * clampSpeedScale(speedScaleEntry.getDouble(1.0));
    return applyInversion(scaledSpeed, invertedEntry);
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

  private void stopCommand(Command command) {
    if (command != null) {
      CommandScheduler.getInstance().cancel(command);
    }
  }
}
