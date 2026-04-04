package frc.robot.subsystems.gamepiece.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class HopperIOSparkMax implements HopperIO {
  private final SparkMax hopperAgitator =
      new SparkMax(HopperConstants.hopperAgitatorDriveCanId, MotorType.kBrushless);
  private final SparkMax hopperExtension =
      new SparkMax(HopperConstants.hopperExtensionCanId, MotorType.kBrushless);
  private final RelativeEncoder hopperExtensionEncoder = hopperExtension.getEncoder();
  private final SparkClosedLoopController hopperExtensionController =
      hopperExtension.getClosedLoopController();

  private double agitatorPositionRotations = 0.0;
  private double extensionPositionKp = HopperConstants.hopperExtensionPositionKp;
  private double extensionPositionKi = HopperConstants.hopperExtensionPositionKi;
  private double extensionPositionKd = HopperConstants.hopperExtensionPositionKd;

  public HopperIOSparkMax() {
    SparkBaseConfig agitatorConfig =
        new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(HopperConstants.hopperAgitatorCurrentLimitAmps)
            .voltageCompensation(12.0);
    SparkMaxConfig extensionConfig = new SparkMaxConfig();
    extensionConfig.idleMode(IdleMode.kCoast);
    extensionConfig.smartCurrentLimit(HopperConstants.hopperExtensionCurrentLimitAmps);
    extensionConfig.voltageCompensation(12.0);
    extensionConfig.closedLoop.pid(
        HopperConstants.hopperExtensionPositionKp,
        HopperConstants.hopperExtensionPositionKi,
        HopperConstants.hopperExtensionPositionKd);
    hopperAgitator.configure(
        agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    hopperExtension.configure(
        extensionConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    double agitatorAppliedOutput = hopperAgitator.get();
    double agitatorVelocityRotationsPerSec =
        agitatorAppliedOutput * HopperConstants.estimatedAgitatorMaxVelocityRotationsPerSec;
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
  public void setAgitatorVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
    // The real agitator path is still sensorless, so approximate the request with a bounded duty
    // cycle until dedicated feedback hardware exists.
    hopperAgitator.set(
        MathUtil.clamp(
            velocityRotationsPerSec / HopperConstants.estimatedAgitatorMaxVelocityRotationsPerSec,
            -1.0,
            1.0));
  }

  @Override
  public void setExtensionOutput(double output) {
    hopperExtension.set(output);
  }

  @Override
  public void setExtensionPositionSetpointRotations(double positionRotations) {
    hopperExtensionController.setSetpoint(
        positionRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setExtensionPositionClosedLoopGains(double kp, double ki, double kd) {
    if (!hasGainChange(extensionPositionKp, kp)
        && !hasGainChange(extensionPositionKi, ki)
        && !hasGainChange(extensionPositionKd, kd)) {
      return;
    }

    extensionPositionKp = kp;
    extensionPositionKi = ki;
    extensionPositionKd = kd;

    SparkMaxConfig extensionConfig = new SparkMaxConfig();
    extensionConfig.closedLoop.pid(kp, ki, kd);
    hopperExtension.configure(
        extensionConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private static boolean hasGainChange(double oldValue, double newValue) {
    return Math.abs(oldValue - newValue) > 1e-9;
  }
}
