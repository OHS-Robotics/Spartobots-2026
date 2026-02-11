package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
  private ShooterConstants() {}

  public static final double gravityMetersPerSecSquared = 9.80665;

  // Replace with measured values from your robot.
  public static final double defaultLaunchSpeedMetersPerSec = 11.5;
  public static final Rotation2d defaultLaunchAngle = Rotation2d.fromDegrees(56.0);
  public static final double defaultLaunchHeightMeters = Units.inchesToMeters(30.0);

  // Approximate center of the scoring opening above the carpet.
  public static final double hubCenterHeightMeters = Units.inchesToMeters(104.0);

  public static final double minLaunchSpeedMetersPerSec = 1.0;
  public static final double minAirtimeSeconds = 0.05;
  public static final double maxAirtimeSeconds = 2.5;
  public static final double fallbackAirtimeSeconds = 0.6;
  public static final double minHorizontalVelocityMetersPerSec = 0.25;
}
