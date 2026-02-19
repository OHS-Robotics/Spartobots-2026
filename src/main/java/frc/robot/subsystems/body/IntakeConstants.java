package frc.robot.subsystems.body;

public final class IntakeConstants {
  private IntakeConstants() {}

  public static final String configTableName = "IntakeConfig";

  // CAN IDs
  public static final int intakeDriveCanId = 30;
  public static final int intakePivotCanId = 40;

  // Default open-loop speeds
  public static final double defaultIntakeSpeed = 0.5;
  public static final double defaultIntakePivotSpeedScale = 1.0;

  // Default inversion
  public static final boolean defaultIntakeDriveInverted = false;
  public static final boolean defaultIntakePivotInverted = false;
}
