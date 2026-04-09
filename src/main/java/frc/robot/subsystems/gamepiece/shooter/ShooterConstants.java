package frc.robot.subsystems.gamepiece.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.gamepiece.ControlSensitivity;

public final class ShooterConstants {
  private ShooterConstants() {}

  public static final String configTableName = "GamePiece/Shooter";

  public static final double gravityMetersPerSecSquared = 9.80665;

  // Single wide drum shooter with one leader and three followers on the same shaft.
  public static final int shooterLeaderCanId = 40;
  public static final int shooterFollowerOneCanId = 41;
  public static final int shooterFollowerTwoCanId = 42;
  public static final int shooterFollowerThreeCanId = 43;
  public static final ControlSensitivity shooterWheelSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;

  // Shooter hood adjuster (position-sensitive)
  public static final int hoodCanId = 50;
  public static final ControlSensitivity hoodSensitivity = ControlSensitivity.POSITION_SENSITIVE;

  // Hardware inversion for the single drum shaft.
  // For normal forward shooter motion, CAN IDs 40 and 42 are inverted while 41 and 43 are not.
  public static final boolean shooterMotor40Inverted = true;
  public static final boolean shooterMotor41Inverted = false;
  public static final boolean shooterMotor42Inverted = true;
  public static final boolean shooterMotor43Inverted = false;
  public static final boolean hoodInverted = false;

  // Electrical limits
  public static final int shooterMotorCurrentLimitAmps = 60;
  public static final int hoodMotorCurrentLimitAmps = 40;

  // Closed-loop gains
  public static final double shooterVelocityKp = 0.00012;
  public static final double shooterVelocityKi = 0.000005;
  public static final double shooterVelocityKd = 0.001;
  public static final double shooterVelocityKv =
      1.0 / Units.rotationsPerMinuteToRadiansPerSecond(5676.0);
  public static final double hoodPositionKp = 0.6;
  public static final double hoodPositionKi = 0.0;
  public static final double hoodPositionKd = 0.3;

  // Runtime shooter wheel tuning
  public static final double defaultWheelSpeedScale = 1.0;
  public static final double defaultPair1Direction = 1.0;
  public static final double defaultPair2Direction = 1.0;
  public static final double wheelCommandRampUpRadPerSecSquared = 700.0;
  public static final double wheelCommandRampDownRadPerSecSquared = 120.0;
  // Release decel is intentionally gentler than the normal downward ramp so the wheels coast
  // down instead of braking hard when the operator lets go of shoot.
  public static final double wheelCommandReleaseRampDownRadPerSecSquared = 70.0;
  public static final double simWheelCommandRampDownRadPerSecSquared = 100000.0;

  // Hardware geometry
  public static final double shooterWheelDiameterInches = 3.965;
  public static final double fuelBallDiameterInches = 5.9;
  public static final double shooterWheelRadiusMeters =
      Units.inchesToMeters(shooterWheelDiameterInches / 2.0);
  public static final double fuelBallRadiusMeters =
      Units.inchesToMeters(fuelBallDiameterInches / 2.0);

  // Ball runs between the moving shooter drum and a stationary hood panel.
  // Ideal no-slip rolling gives ball center speed = 0.5 * drum surface speed.
  public static final double ballCenterSpeedFromWheelSurfaceRatio = 0.5;
  public static final double launchSlipFactor = 1.0;
  public static final double launchSpeedFromWheelSurfaceSpeedScale =
      launchSlipFactor * ballCenterSpeedFromWheelSurfaceRatio;

  // Shooter kinematic limits
  public static final double neoFreeSpeedRadPerSec =
      Units.rotationsPerMinuteToRadiansPerSecond(5676.0);
  public static final double minWheelSpeedRadPerSec =
      Units.rotationsPerMinuteToRadiansPerSecond(600.0);
  public static final double maxWheelSpeedRadPerSec = neoFreeSpeedRadPerSec * 0.90;

  // Shooter geometry
  public static final double defaultLaunchHeightMeters = Units.inchesToMeters(22.0);
  public static final double hubCenterHeightMeters = Units.inchesToMeters(104.0);
  public static final double defaultHubAimHeightOffsetMeters = 0.0;
  public static final double defaultHubAimHeightMeters =
      hubCenterHeightMeters + defaultHubAimHeightOffsetMeters;

  // Distance range used for preferred-angle interpolation
  public static final double hubShotNearDistanceMeters = Units.feetToMeters(6.0);
  public static final double hubShotFarDistanceMeters = Units.feetToMeters(20.0);

  // Preferred hood profile, tuned by distance (near = steeper, far = flatter)
  public static final Rotation2d hubShotNearPreferredAngle = Rotation2d.fromDegrees(66.0);
  public static final Rotation2d hubShotFarPreferredAngle = Rotation2d.fromDegrees(44.0);

  // Hood angular limits from the floor plane. These are the two-point calibration anchors.
  public static final Rotation2d minHoodAngleFromFloor = Rotation2d.fromDegrees(45.0);
  public static final Rotation2d maxHoodAngleFromFloor = Rotation2d.fromDegrees(83.0);

  // Allowed launch envelope searched by solver
  public static final Rotation2d minLaunchAngle = minHoodAngleFromFloor;
  public static final Rotation2d maxLaunchAngle = maxHoodAngleFromFloor;
  public static final double launchAngleSearchStepDegrees = 0.5;

  // Hood hard-stop calibration.
  // Each successful homing cycle zeros the relative encoder at the retracted hard stop.
  public static final double hoodRetractedHardStopReferenceRotations = 0.0;
  // Keep this slow, but still high enough to trip the hood stall-current threshold at the hard
  // stop.
  public static final double hoodHomingOutputTowardRetractedHardStop = -0.1;
  public static final double hoodHomingOutputTowardExtendedHardStop = 0.1;
  public static final double hoodHomingMinCurrentAmps = 18.0;
  public static final double hoodHomingMaxVelocityRotationsPerSec = 0.08;
  public static final double hoodHomingRelaxBeforeCalibrationSeconds = 1.0;
  public static final double hoodHomingStallConfirmSeconds = 0.20;
  public static final double hoodHomingTimeoutSeconds = 4.5;
  public static final double hoodHomingMinTravelRotations = 4.0;

  // Hood two-point calibration in motor rotations.
  // Retracted is the steepest launch-angle hard stop, extended is the flattest.
  // Update the extended value after the first real two-hard-stop homing run.
  public static final double defaultHoodRetractedPositionRotations =
      hoodRetractedHardStopReferenceRotations;
  public static final double defaultHoodExtendedPositionRotations = 16.0;

  // Measured/estimated launch capability of the mechanism
  public static final double minLaunchSpeedMetersPerSec =
      launchSpeedFromWheelSurfaceSpeedScale * shooterWheelRadiusMeters * minWheelSpeedRadPerSec;
  public static final double maxLaunchSpeedMetersPerSec =
      launchSpeedFromWheelSurfaceSpeedScale * shooterWheelRadiusMeters * maxWheelSpeedRadPerSec;
  public static final double defaultLaunchSpeedMetersPerSec = 11.5;
  public static final Rotation2d defaultLaunchAngle = Rotation2d.fromDegrees(56.0);
  public static final double minAirtimeSeconds = 0.05;
  public static final double maxAirtimeSeconds = 3.0;
  public static final double fallbackAirtimeSeconds = 0.6;
  public static final double minHorizontalVelocityMetersPerSec = 0.25;
  public static final double hubTopEntryMinDescentVelocityMetersPerSec = 0.05;
  public static final double hubMotionCompensationVelocityScale = 1.15;
  public static final double hubMotionCompensationLeadSeconds = 0.03;
  public static final int hubMotionCompensationMaxIterations = 8;
  public static final double hubMotionCompensationAirtimeToleranceSeconds = 0.01;
  public static final double hubShotPredictionLaunchLeadSeconds = 0.10;
  public static final double hubPredictionMaxRobotAccelerationMetersPerSecSquared = 8.0;

  // Feed interlock readiness
  public static final double wheelReadyToleranceRatio = 0.08;
  public static final double minWheelSetpointForReadinessRadPerSec = minWheelSpeedRadPerSec;

  // Simulated shot behavior
  public static final double simShotCadenceSeconds = 0.05;
  public static final double simWheelReadyRatio = 0.88;

  // Shooter muzzle relative to robot center (robot frame).
  // The drum sits close to the rear bumper, so model the launch point well behind center.
  public static final Translation2d shooterMuzzleOffsetOnRobot =
      new Translation2d(-Units.inchesToMeters(13.0), 0.0);
  public static final Rotation2d shooterFacingOffset = Rotation2d.fromDegrees(180.0);

  // MapleSim target matching tolerance for projectile hit checks
  public static final double projectileTargetToleranceXYMeters = Units.inchesToMeters(10.0);
  public static final double projectileTargetToleranceZMeters = Units.inchesToMeters(8.0);
  public static final double hubPredictedScoreToleranceXYMeters = projectileTargetToleranceXYMeters;
  public static final double hubPredictedScoreToleranceZMeters = projectileTargetToleranceZMeters;

  // Estimated simulation configuration
  public static final double simNominalVoltage = 12.0;
  public static final DCMotor simWheelGearbox = DCMotor.getNEO(1);
  public static final double simWheelReduction = 1.0;
  public static final double simWheelMoiKgMetersSq = 0.003;
  public static final DCMotor simHoodGearbox = DCMotor.getNEO(1);
  public static final double simHoodReduction = 100.0;
  public static final double simEstimatedHoodLengthMeters = Units.inchesToMeters(11.0);
  public static final double simEstimatedHoodMoiKgMetersSq = 0.08;
  public static final double simHoodMinAngleRadians = minHoodAngleFromFloor.getRadians();
  public static final double simHoodMaxAngleRadians = maxHoodAngleFromFloor.getRadians();
  public static final double simHoodStartingAngleRadians = defaultLaunchAngle.getRadians();
}
