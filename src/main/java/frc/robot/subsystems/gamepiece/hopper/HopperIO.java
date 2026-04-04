package frc.robot.subsystems.gamepiece.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean extensionConnected = false;
    public double extensionPositionRotations = 0.0;
    public double extensionVelocityRpm = 0.0;
    public double extensionAppliedOutput = 0.0;
    public double extensionCurrentAmps = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setExtensionOutput(double output) {}

  public default void resetSimulationState() {}
}
