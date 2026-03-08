package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.Test;

class ShooterBallisticsTest {
  private static final double EPSILON = 1e-9;

  @Test
  void solveHubShotReturnsCurrentLaunchConfiguration() {
    ShooterBallistics ballistics = new ShooterBallistics();
    Pose2d robotPose = new Pose2d();
    Pose3d hubPose = new Pose3d(4.0, 0.0, 1.8, new Rotation3d());

    ShotSolution solution = ballistics.solveHubShot(robotPose, hubPose);

    assertEquals(ShooterConstants.defaultLaunchSpeedMetersPerSec, solution.launchSpeedMetersPerSec(), EPSILON);
    assertEquals(
        ShooterConstants.defaultLaunchAngle.getRadians(),
        solution.launchAngle().getRadians(),
        EPSILON);
    assertEquals(ShooterConstants.defaultLaunchHeightMeters, solution.launchHeightMeters(), EPSILON);
    assertEquals(
        ballistics.estimateHubShotAirtimeSeconds(robotPose, hubPose),
        solution.airtimeSeconds(),
        EPSILON);
  }

  @Test
  void fallsBackWhenHorizontalVelocityIsTooSmall() {
    ShooterBallistics ballistics = new ShooterBallistics();
    ballistics.setLaunchState(
        ShooterConstants.defaultLaunchSpeedMetersPerSec, Rotation2d.fromDegrees(90.0));

    double airtimeSeconds =
        ballistics.estimateHubShotAirtimeSeconds(
            new Pose2d(), new Pose3d(3.0, 0.0, 1.8, new Rotation3d()));

    assertEquals(ShooterConstants.fallbackAirtimeSeconds, airtimeSeconds, EPSILON);
  }
}
