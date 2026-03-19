package frc.robot.subsystems.gamepiece.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HopperIOSparkMax implements HopperIO {
  private static final double estimatedAgitatorMaxVelocityRotationsPerSec = 12.0;

  private final SparkMax hopperAgitator =
      new SparkMax(HopperConstants.hopperAgitatorDriveCanId, MotorType.kBrushed);
  private final SparkMax hopperExtension =
      new SparkMax(HopperConstants.hopperExtensionCanId, MotorType.kBrushed);
  private final RelativeEncoder hopperExtensionEncoder = hopperExtension.getEncoder();
  private double agitatorPositionRotations = 0.0;

  public HopperIOSparkMax() {
    SparkBaseConfig agitatorConfig =
        new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(HopperConstants.hopperAgitatorCurrentLimitAmps)
            .voltageCompensation(12.0);
    SparkBaseConfig extensionConfig =
        new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(HopperConstants.hopperExtensionCurrentLimitAmps)
            .voltageCompensation(12.0);
    hopperAgitator.configure(
        agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    hopperExtension.configure(
        extensionConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    double agitatorAppliedOutput = hopperAgitator.get();
    double agitatorVelocityRotationsPerSec =
        agitatorAppliedOutput * estimatedAgitatorMaxVelocityRotationsPerSec;
    agitatorPositionRotations += agitatorVelocityRotationsPerSec * 0.02;

    inputs.agitatorConnected = true;
    inputs.extensionConnected = true;
    inputs.agitatorPositionRotations = agitatorPositionRotations;
    inputs.agitatorVelocityRotationsPerSec = agitatorVelocityRotationsPerSec;
    inputs.agitatorAppliedOutput = agitatorAppliedOutput;
    inputs.agitatorCurrentAmps = hopperAgitator.getOutputCurrent();
    inputs.extensionPositionRotations = hopperExtensionEncoder.getPosition();
    inputs.extensionVelocityRpm = hopperExtensionEncoder.getVelocity();
    inputs.extensionAppliedOutput = hopperExtension.get();
    inputs.extensionCurrentAmps = hopperExtension.getOutputCurrent();
  }

  @Override
  public void setAgitatorOutput(double output) {
    hopperAgitator.set(output);
  }

  @Override
  public void setExtensionOutput(double output) {
    hopperExtension.set(output);
  }
}
