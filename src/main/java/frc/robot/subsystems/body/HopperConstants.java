package frc.robot.subsystems.body;

public final class HopperConstants {
  private HopperConstants() {}

  public static final String configTableName = "HopperConfig";

  // CAN IDs
  public static final int hopperBeltDriveCanId = 38;
  public static final int hopperExtensionCanId = 39;

  // Default open-loop speeds
  public static final double defaultHopperBeltSpeed = 0.5;
  public static final double defaultHopperExtensionSpeedScale = 1.0;

  // Default inversion
  public static final boolean defaultHopperBeltInverted = false;
  public static final boolean defaultHopperExtensionInverted = false;
}
