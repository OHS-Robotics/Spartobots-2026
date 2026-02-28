package frc.robot.subsystems.body;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class IntakeConstants {
  private IntakeConstants() {}

  public static final String configTableName = "Body/Intake";

  // Intake drive roller (velocity-sensitive)
  public static final int intakeDriveCanId = 30;
  public static final ControlSensitivity intakeDriveSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;
  public static final int intakeDriveCurrentLimitAmps = 80;

  // Intake pivot arm (position-sensitive)
  public static final int intakePivotCanId = 40;
  public static final ControlSensitivity intakePivotSensitivity =
      ControlSensitivity.POSITION_SENSITIVE;
  public static final int intakePivotCurrentLimitAmps = 80;

  // Two-point pivot calibration in motor rotations
  public static final double defaultIntakePivotRetractedPositionRotations = 0.0;
  public static final double defaultIntakePivotExtendedPositionRotations = 1.0;

  // Default open-loop speeds
  public static final double defaultIntakeSpeed = 0.5;
  public static final double defaultIntakePivotSpeedScale = 1.0;

  // Default direction/inversion
  public static final double defaultIntakeDriveDirection = -1.0;
  public static final boolean defaultIntakePivotInverted = false;

  // Estimated simulation configuration
  public static final double simNominalVoltage = 12.0;
  public static final DCMotor simDriveGearbox = DCMotor.getNEO(1);
  public static final double simDriveReduction = 3.0;
  public static final double simDriveMoiKgMetersSq = 0.0025;
  public static final double simDriveRollerRadiusMeters = Units.inchesToMeters(1.25);
  public static final DCMotor simPivotGearbox = DCMotor.getNEO(1);
  public static final double simPivotReduction = 80.0;
  public static final double simEstimatedPivotLengthMeters = Units.inchesToMeters(14.0);
  public static final double simEstimatedPivotMoiKgMetersSq = 0.32;
  public static final double simPivotRetractedAngleRadians = Units.degreesToRadians(0.0);
  public static final double simPivotExtendedAngleRadians = Units.degreesToRadians(-82.0);
  public static final double simPivotMinAngleRadians = Units.degreesToRadians(-85.0);
  public static final double simPivotMaxAngleRadians = Units.degreesToRadians(5.0);
  public static final double simMapleIntakeWidthMeters = Units.inchesToMeters(27.0);
  public static final double simMapleIntakeExtensionMeters = Units.inchesToMeters(12.0);
  public static final double simForwardOutputThreshold = 0.1;
}
