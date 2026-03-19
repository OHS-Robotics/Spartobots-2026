package frc.robot.subsystems.gamepiece.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean agitatorConnected = false;
    public boolean extensionConnected = false;
    public double agitatorPositionRotations = 0.0;
    public double agitatorVelocityRotationsPerSec = 0.0;
    public double agitatorAppliedOutput = 0.0;
    public double agitatorCurrentAmps = 0.0;
    public double extensionPositionRotations = 0.0;
    public double extensionVelocityRpm = 0.0;
    public double extensionAppliedOutput = 0.0;
    public double extensionCurrentAmps = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setAgitatorOutput(double output) {}

  public default void setAgitatorVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {}

  public default void setExtensionOutput(double output) {}

  public default void setExtensionPositionSetpointRotations(double positionRotations) {}

  public default void setAgitatorVelocityClosedLoopGains(
      double kp, double ki, double kd, double kv) {}

  public default void setExtensionPositionClosedLoopGains(double kp, double ki, double kd) {}

  public default void resetSimulationState() {}
}
