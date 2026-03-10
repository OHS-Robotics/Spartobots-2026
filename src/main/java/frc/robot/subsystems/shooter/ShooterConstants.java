package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotSettings;

/** Shooter constants delegated from {@link frc.robot.RobotSettings}. */
public final class ShooterConstants {
  private ShooterConstants() {}

  public static final double gravityMetersPerSecSquared =
      RobotSettings.Shooter.gravityMetersPerSecSquared;

  // Replace with measured values from your robot.
  public static final double defaultLaunchSpeedMetersPerSec =
      RobotSettings.Shooter.defaultLaunchSpeedMetersPerSec;
  public static final Rotation2d defaultLaunchAngle = RobotSettings.Shooter.defaultLaunchAngle;
  public static final double defaultLaunchHeightMeters =
      RobotSettings.Shooter.defaultLaunchHeightMeters;

  public static final double minLaunchSpeedMetersPerSec =
      RobotSettings.Shooter.minLaunchSpeedMetersPerSec;
  public static final double minAirtimeSeconds = RobotSettings.Shooter.minAirtimeSeconds;
  public static final double maxAirtimeSeconds = RobotSettings.Shooter.maxAirtimeSeconds;
  public static final double fallbackAirtimeSeconds = RobotSettings.Shooter.fallbackAirtimeSeconds;
  public static final double minHorizontalVelocityMetersPerSec =
      RobotSettings.Shooter.minHorizontalVelocityMetersPerSec;
}
