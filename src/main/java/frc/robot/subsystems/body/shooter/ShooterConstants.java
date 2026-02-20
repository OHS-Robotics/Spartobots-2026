package frc.robot.subsystems.body.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
  private ShooterConstants() {}

  public static final String configTableName = "ShooterConfig";

  public static final double gravityMetersPerSecSquared = 9.80665;

  // CAN IDs
  public static final int pair1LeaderCanId = 31;
  public static final int pair1FollowerCanId = 32;
  public static final int pair2LeaderCanId = 33;
  public static final int pair2FollowerCanId = 34;
  public static final int hoodCanId = 35;

  // Motor direction: each pair spins the same direction, the two pairs opposite.
  public static final boolean pair1Inverted = false;
  public static final boolean pair2Inverted = true;
  public static final boolean pairFollowerInverted = false;
  public static final boolean hoodInverted = false;

  // Electrical limits
  public static final int shooterMotorCurrentLimitAmps = 50;
  public static final int hoodMotorCurrentLimitAmps = 30;

  // Closed-loop gains
  public static final double shooterVelocityKp = 0.00035;
  public static final double shooterVelocityKi = 0.0;
  public static final double shooterVelocityKd = 0.0;
  public static final double shooterVelocityKv = 0.0;
  public static final double hoodPositionKp = 2.0;
  public static final double hoodPositionKi = 0.0;
  public static final double hoodPositionKd = 0.0;

  // Hardware geometry
  public static final double shooterWheelRadiusMeters = Units.inchesToMeters(2.0);
  public static final double fuelBallRadiusMeters = 0.075;
  public static final double launchSlipFactor = 0.93;
  public static final double targetBallSpinRatio = 0.10;

  // Shooter kinematic limits
  public static final double neoFreeSpeedRadPerSec =
      Units.rotationsPerMinuteToRadiansPerSecond(5676.0);
  public static final double minWheelSpeedRadPerSec =
      Units.rotationsPerMinuteToRadiansPerSecond(600.0);
  public static final double maxWheelSpeedRadPerSec = neoFreeSpeedRadPerSec * 0.90;

  // Hood calibration (linear map). Calibrate on robot.
  // hoodAngleDeg = referenceAngleDeg + (motorRot - referenceMotorRot) * hoodDegreesPerMotorRotation
  public static final Rotation2d hoodReferenceAngle = Rotation2d.fromDegrees(50.0);
  public static final double hoodReferenceMotorRotations = 0.0;
  public static final double hoodDegreesPerMotorRotation = 6.5;

  // Shooter geometry
  public static final double defaultLaunchHeightMeters = Units.inchesToMeters(30.0);
  public static final double hubCenterHeightMeters = Units.inchesToMeters(104.0);

  // Distance range used for preferred-angle interpolation
  public static final double hubShotNearDistanceMeters = Units.feetToMeters(6.0);
  public static final double hubShotFarDistanceMeters = Units.feetToMeters(20.0);

  // Preferred hood profile, tuned by distance (near = steeper, far = flatter)
  public static final Rotation2d hubShotNearPreferredAngle = Rotation2d.fromDegrees(66.0);
  public static final Rotation2d hubShotFarPreferredAngle = Rotation2d.fromDegrees(44.0);

  // Allowed launch envelope searched by solver
  public static final Rotation2d minLaunchAngle = Rotation2d.fromDegrees(35.0);
  public static final Rotation2d maxLaunchAngle = Rotation2d.fromDegrees(75.0);
  public static final double launchAngleSearchStepDegrees = 0.5;

  // Measured/estimated launch capability of the mechanism
  public static final double minLaunchSpeedMetersPerSec =
      launchSlipFactor * shooterWheelRadiusMeters * minWheelSpeedRadPerSec;
  public static final double maxLaunchSpeedMetersPerSec =
      launchSlipFactor * shooterWheelRadiusMeters * maxWheelSpeedRadPerSec;
  public static final double defaultLaunchSpeedMetersPerSec = 11.5;
  public static final Rotation2d defaultLaunchAngle = Rotation2d.fromDegrees(56.0);
  public static final double minAirtimeSeconds = 0.05;
  public static final double maxAirtimeSeconds = 3.0;
  public static final double fallbackAirtimeSeconds = 0.6;
  public static final double minHorizontalVelocityMetersPerSec = 0.25;
  public static final double triggerLaunchSpeedBoostScale = 1.12;

  // Simulated shot behavior
  public static final double simShotCadenceSeconds = 0.35;
  public static final double simWheelReadyRatio = 0.88;

  // Shooter muzzle relative to robot center (robot frame)
  public static final Translation2d shooterMuzzleOffsetOnRobot = new Translation2d(0.25, 0.0);
  public static final Rotation2d shooterFacingOffset = Rotation2d.kZero;

  // MapleSim target matching tolerance for projectile hit checks
  public static final double projectileTargetToleranceXYMeters = Units.inchesToMeters(10.0);
  public static final double projectileTargetToleranceZMeters = Units.inchesToMeters(8.0);
}
