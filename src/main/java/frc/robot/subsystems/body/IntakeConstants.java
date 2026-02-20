package frc.robot.subsystems.body;

public final class IntakeConstants {
  private IntakeConstants() {}

  public static final String configTableName = "Body/Intake";

  // Intake drive roller (velocity-sensitive)
  public static final int intakeDriveCanId = 30;
  public static final ControlSensitivity intakeDriveSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;

  // Intake pivot arm (position-sensitive)
  public static final int intakePivotCanId = 40;
  public static final ControlSensitivity intakePivotSensitivity =
      ControlSensitivity.POSITION_SENSITIVE;

  // Two-point pivot calibration in motor rotations
  public static final double defaultIntakePivotRetractedPositionRotations = 0.0;
  public static final double defaultIntakePivotExtendedPositionRotations = 1.0;

  // Default open-loop speeds
  public static final double defaultIntakeSpeed = 0.5;
  public static final double defaultIntakePivotSpeedScale = 1.0;

  // Default direction/inversion
  public static final double defaultIntakeDriveDirection = -1.0;
  public static final boolean defaultIntakePivotInverted = false;
}
