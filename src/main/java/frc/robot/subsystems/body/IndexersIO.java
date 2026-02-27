package frc.robot.subsystems.body;

import org.littletonrobotics.junction.AutoLog;

public interface IndexersIO {
  @AutoLog
  public static class IndexersIOInputs {
    public boolean topConnected = false;
    public boolean bottomConnected = false;
    public double topPositionRotations = 0.0;
    public double topVelocityRotationsPerSec = 0.0;
    public double topAppliedOutput = 0.0;
    public double topCurrentAmps = 0.0;
    public double bottomPositionRotations = 0.0;
    public double bottomVelocityRotationsPerSec = 0.0;
    public double bottomAppliedOutput = 0.0;
    public double bottomCurrentAmps = 0.0;
  }

  public default void updateInputs(IndexersIOInputs inputs) {}

  public default void setTopOutput(double output) {}

  public default void setBottomOutput(double output) {}

  public default void resetSimulationState() {}
}
