package frc.robot.subsystems.body;

public final class AgitatorsConstants {
  private AgitatorsConstants() {}

  public static final String configTableName = "AgitatorsConfig";

  // Upper/top agitator (velocity-sensitive)
  public static final int topAgitatorCanId = 36;
  public static final ControlSensitivity topAgitatorSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;

  // Lower/bottom agitator (velocity-sensitive)
  public static final int bottomAgitatorCanId = 37;
  public static final ControlSensitivity bottomAgitatorSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;

  // Default open-loop speeds
  public static final double defaultTopAgitatorSpeed = 20.0;
  public static final double defaultBottomAgitatorSpeed = 20.0;

  // Default direction/inversion
  public static final double defaultTopAgitatorDirection = 1.0;
  public static final double defaultBottomAgitatorDirection = 1.0;
}
