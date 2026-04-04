package frc.robot.subsystems.gamepiece.hopper;

import frc.robot.subsystems.gamepiece.ControlSensitivity;
import frc.robot.subsystems.gamepiece.intake.IntakeConstants;

public final class HopperConstants {
  private HopperConstants() {}

  public static final String configTableName = "GamePiece/Hopper";

  // Hopper extension is mechanically tied to the intake extension on the current robot.
  public static final int hopperExtensionCanId = IntakeConstants.intakePivotCanId;
  public static final ControlSensitivity hopperExtensionSensitivity =
      ControlSensitivity.POSITION_SENSITIVE;
  public static final int hopperExtensionCurrentLimitAmps =
      IntakeConstants.intakePivotCurrentLimitAmps;

  // Two-point extension calibration in motor rotations
  public static final double defaultHopperExtensionRetractedPositionRotations =
      IntakeConstants.defaultIntakePivotRetractedPositionRotations;
  public static final double defaultHopperExtensionExtendedPositionRotations =
      IntakeConstants.defaultIntakePivotExtendedPositionRotations;

  // Default open-loop tuning
  public static final double defaultHopperExtensionSpeedScale = 1.0;
  public static final boolean defaultHopperExtensionInverted = false;
}
