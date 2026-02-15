package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
  private ShooterConstants() {}

  public static final double gravityMetersPerSecSquared = 9.80665;

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
  public static final double minLaunchSpeedMetersPerSec = 4.0;
  public static final double maxLaunchSpeedMetersPerSec = 19.0;
  public static final double defaultLaunchSpeedMetersPerSec = 11.5;
  public static final Rotation2d defaultLaunchAngle = Rotation2d.fromDegrees(56.0);
  public static final double minAirtimeSeconds = 0.05;
  public static final double maxAirtimeSeconds = 3.0;
  public static final double fallbackAirtimeSeconds = 0.6;
  public static final double minHorizontalVelocityMetersPerSec = 0.25;
}
