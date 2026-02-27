package frc.robot.subsystems.body;

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

  public default void setExtensionOutput(double output) {}

  public default void resetSimulationState() {}
}
