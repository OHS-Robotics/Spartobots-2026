package frc.robot.subsystems.body;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class HopperConstants {
  private HopperConstants() {}

  public static final String configTableName = "Body/Hopper";

  // Hopper agitator drive (velocity-sensitive)
  public static final int hopperAgitatorDriveCanId = 38;
  public static final ControlSensitivity hopperAgitatorSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;
  public static final int hopperAgitatorCurrentLimitAmps = 80;

  // L1 climber winch (reuses the previous hopper-extension control path).
  public static final int l1ClimberCanId = 34;
  public static final int hopperExtensionCanId = l1ClimberCanId;
  public static final ControlSensitivity hopperExtensionSensitivity =
      ControlSensitivity.POSITION_SENSITIVE;
  public static final int hopperExtensionCurrentLimitAmps = 80;

  // Two-point extension calibration in motor rotations
  public static final double defaultHopperExtensionRetractedPositionRotations = 0.0;
  public static final double defaultHopperExtensionExtendedPositionRotations = 1.0;

  // Default open-loop speeds
  public static final double defaultHopperAgitatorSpeed = 0.5;
  public static final double defaultHopperExtensionSpeedScale = 1.0;

  // Default direction/inversion
  public static final double defaultHopperAgitatorDirection = -1.0;
  public static final boolean defaultHopperExtensionInverted = false;

  // Estimated simulation configuration
  public static final double simNominalVoltage = 12.0;
  public static final DCMotor simAgitatorGearbox = DCMotor.getBag(1);
  public static final double simAgitatorReduction = 4.0;
  public static final double simAgitatorMoiKgMetersSq = 0.002;
  public static final double simAgitatorRadiusMeters = Units.inchesToMeters(1.25);
  public static final DCMotor simExtensionGearbox = DCMotor.getBag(1);
  public static final double simExtensionReduction = 18.0;
  public static final double simEstimatedExtensionMassKg = 3.5;
  public static final double simEstimatedExtensionDrumRadiusMeters = Units.inchesToMeters(0.75);
  public static final double simExtensionRetractedHeightMeters = 0.0;
  public static final double simExtensionExtendedHeightMeters = Units.inchesToMeters(8.0);
}
