package frc.robot.subsystems.gamepiece.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.gamepiece.ControlSensitivity;

public final class IntakeConstants {
  private IntakeConstants() {}

  public static final String configTableName = "GamePiece/Intake";

  // Intake drive rollers (velocity-sensitive). The bottom motor mirrors the top motor.
  public static final int intakeDriveCanId = 30;
  public static final int intakeFollowerCanId = 31;
  public static final ControlSensitivity intakeDriveSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;
  public static final int intakeDriveCurrentLimitAmps = 80;

  // Intake extension/pivot arm (position-sensitive)
  public static final int intakePivotCanId = 51;
  public static final ControlSensitivity intakePivotSensitivity =
      ControlSensitivity.POSITION_SENSITIVE;
  public static final int intakePivotCurrentLimitAmps = 80;

  // Two-point pivot calibration in motor rotations.
  // Each successful calibration run zeros the relative encoder at the retracted hard stop.
  public static final double intakePivotRetractedHardStopReferenceRotations = 0.0;
  public static final double defaultIntakePivotRetractedPositionRotations =
      intakePivotRetractedHardStopReferenceRotations;
  public static final double defaultIntakePivotExtendedPositionRotations = 1.0;
  public static final double intakePivotCalibrationOutputTowardRetractedHardStop = -0.40;
  public static final double intakePivotCalibrationOutputTowardExtendedHardStop = 0.40;
  public static final double intakePivotCalibrationMinCurrentAmps = 8.0;
  public static final double intakePivotCalibrationMaxVelocityRpm = 8.0;
  public static final double intakePivotCalibrationStallConfirmSeconds = 0.20;
  public static final double intakePivotCalibrationTimeoutSeconds = 5.0;
  public static final double intakePivotCalibrationMinTravelRotations = 0.20;

  // Default open-loop speeds
  public static final double defaultIntakeSpeed = 0.25;
  public static final double defaultIntakePivotSpeedScale = 1.0;
  public static final double intakePivotSweepTraversalSecondsAtFullTrigger = 0.75;
  public static final double intakePivotSweepHardStopInsetNormalized = 0.05;
  public static final double intakePivotIntakingPositionInsetNormalized = 0.08;
  // Manual feed stays in the upper half of travel and pulses the rollers near full extension.
  public static final double intakePivotManualFeedLowerLimitNormalized = 0.50;
  public static final double intakePivotManualFeedPulseWindowNormalized = 0.03;

  // Calibration-mode closed-loop defaults
  public static final double defaultCalibrationDriveVelocitySetpointRotationsPerSec = 0.0;
  public static final double defaultCalibrationPivotPositionSetpointRotations =
      defaultIntakePivotRetractedPositionRotations;
  public static final double intakeDriveVelocityKp = 0.02;
  public static final double intakeDriveVelocityKi = 0.0;
  public static final double intakeDriveVelocityKd = 0.0;
  public static final double intakeDriveVelocityKv = 1.0 / 32.0;
  public static final double intakePivotPositionKp = 0.7;
  public static final double intakePivotPositionKi = 0.0;
  public static final double intakePivotPositionKd = 0.02;

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
