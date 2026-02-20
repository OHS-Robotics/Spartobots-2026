package frc.robot.subsystems.body;

public final class HopperConstants {
  private HopperConstants() {}

  public static final String configTableName = "HopperConfig";

  // Hopper belt drive (velocity-sensitive)
  public static final int hopperBeltDriveCanId = 38;
  public static final ControlSensitivity hopperBeltSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;

  // Hopper extension (position-sensitive)
  public static final int hopperExtensionCanId = 39;
  public static final ControlSensitivity hopperExtensionSensitivity =
      ControlSensitivity.POSITION_SENSITIVE;

  // Two-point extension calibration in motor rotations
  public static final double defaultHopperExtensionRetractedPositionRotations = 0.0;
  public static final double defaultHopperExtensionExtendedPositionRotations = 1.0;

  // Default open-loop speeds
  public static final double defaultHopperBeltSpeed = 0.5;
  public static final double defaultHopperExtensionSpeedScale = 1.0;

  // Default direction/inversion
  public static final double defaultHopperBeltDirection = -1.0;
  public static final boolean defaultHopperExtensionInverted = false;
}
