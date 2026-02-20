package frc.robot.subsystems.body;

public final class AgitatorsConstants {
  private AgitatorsConstants() {}

  public static final String configTableName = "AgitatorsConfig";

  // CAN IDs
  public static final int topAgitatorCanId = 36;
  public static final int bottomAgitatorCanId = 37;

  // Default open-loop speeds
  public static final double defaultTopAgitatorSpeed = 2.5;
  public static final double defaultBottomAgitatorSpeed = 2.5;

  // Default inversion
  public static final boolean defaultTopAgitatorInverted = false;
  public static final boolean defaultBottomAgitatorInverted = false;
}
