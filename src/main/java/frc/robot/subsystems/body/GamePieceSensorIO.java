package frc.robot.subsystems.body;

import org.littletonrobotics.junction.AutoLog;

public interface GamePieceSensorIO {
  @AutoLog
  public static class GamePieceSensorIOInputs {
    public boolean intakeConfigured = false;
    public boolean hopperConfigured = false;
    public boolean shooterConfigured = false;
    public boolean intakeDetected = false;
    public boolean hopperDetected = false;
    public boolean shooterDetected = false;
  }

  public default void updateInputs(GamePieceSensorIOInputs inputs) {}
}
