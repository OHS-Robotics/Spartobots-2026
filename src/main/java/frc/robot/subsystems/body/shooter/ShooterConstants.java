package frc.robot.subsystems.body.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.body.ControlSensitivity;

public final class ShooterConstants {
  private ShooterConstants() {}

  public static final String configTableName = "Body/Shooter";

  public static final double gravityMetersPerSecSquared = 9.80665;

  // Shooter wheel groups (velocity-sensitive)
  public static final int pair1LeaderCanId = 31;
  public static final int pair1FollowerCanId = 32;
  public static final int pair2LeaderCanId = 33;
  public static final int pair2FollowerCanId = 34;
  public static final ControlSensitivity shooterWheelSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;

  // Shooter hood adjuster (position-sensitive)
  public static final int hoodCanId = 35;
  public static final ControlSensitivity hoodSensitivity = ControlSensitivity.POSITION_SENSITIVE;

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

  // Runtime shooter wheel tuning
  public static final double defaultWheelSpeedScale = 1.0;
  public static final double defaultPair1Direction = 1.0;
  public static final double defaultPair2Direction = 1.0;

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

  // Shooter geometry
  public static final double defaultLaunchHeightMeters = Units.inchesToMeters(30.0);
  public static final double hubCenterHeightMeters = Units.inchesToMeters(104.0);

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

  // Hood two-point calibration in motor rotations
  // minHoodAngleFromFloor <-> retracted, maxHoodAngleFromFloor <-> extended
  public static final double defaultHoodRetractedPositionRotations = -2.3076923077;
  public static final double defaultHoodExtendedPositionRotations = 3.8461538462;

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
  public static final double hubMotionCompensationVelocityScale = 1.15;
  public static final double hubMotionCompensationLeadSeconds = 0.03;
  public static final int hubMotionCompensationMaxIterations = 8;
  public static final double hubMotionCompensationAirtimeToleranceSeconds = 0.01;

  // Feed interlock readiness
  public static final double wheelReadyToleranceRatio = 0.08;
  public static final double minWheelSetpointForReadinessRadPerSec =
      Units.rotationsPerMinuteToRadiansPerSecond(300.0);

  // Simulated shot behavior
  public static final double simShotCadenceSeconds = 0.35;
  public static final double simWheelReadyRatio = 0.88;

  // Shooter muzzle relative to robot center (robot frame)
  public static final Translation2d shooterMuzzleOffsetOnRobot = new Translation2d(-0.25, 0.0);
  public static final Rotation2d shooterFacingOffset = Rotation2d.fromDegrees(180.0);

  // MapleSim target matching tolerance for projectile hit checks
  public static final double projectileTargetToleranceXYMeters = Units.inchesToMeters(10.0);
  public static final double projectileTargetToleranceZMeters = Units.inchesToMeters(8.0);
}
