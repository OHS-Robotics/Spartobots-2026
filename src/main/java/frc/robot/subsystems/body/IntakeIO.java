package frc.robot.subsystems.body;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean driveConnected = false;
    public boolean pivotConnected = false;
    public double drivePositionRotations = 0.0;
    public double driveVelocityRotationsPerSec = 0.0;
    public double driveAppliedOutput = 0.0;
    public double driveCurrentAmps = 0.0;
    public double pivotPositionRotations = 0.0;
    public double pivotVelocityRpm = 0.0;
    public double pivotAppliedOutput = 0.0;
    public double pivotCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setDriveOutput(double output) {}

  public default void setPivotOutput(double output) {}

  public default void resetSimulationState() {}
}
