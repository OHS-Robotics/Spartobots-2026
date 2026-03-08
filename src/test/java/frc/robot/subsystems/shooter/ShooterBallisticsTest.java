package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.TargetSelector;
import frc.robot.subsystems.superstructure.Superstructure.PieceState;
import org.junit.jupiter.api.Test;

class ShooterBallisticsTest {
  private static final double EPSILON = 1e-9;

  @Test
  void solveHubShotReturnsCurrentLaunchConfiguration() {
    ShooterBallistics ballistics = new ShooterBallistics();
    Pose2d robotPose = new Pose2d();
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
    Pose3d hubPose = new Pose3d(4.0, 0.0, 1.8, new Rotation3d());

    ShotSolution solution =
        ballistics.solveHubShot(
            robotPose, chassisSpeeds, TargetSelector.HubSelection.ACTIVE, PieceState.HELD, hubPose);

    assertEquals(robotPose.getX(), solution.robotPose().getX(), EPSILON);
    assertEquals(robotPose.getY(), solution.robotPose().getY(), EPSILON);
    assertEquals(
        robotPose.getRotation().getRadians(),
        solution.robotPose().getRotation().getRadians(),
        EPSILON);
    assertEquals(
        chassisSpeeds.vxMetersPerSecond, solution.chassisVelocity().vxMetersPerSecond, EPSILON);
    assertEquals(
        chassisSpeeds.vyMetersPerSecond, solution.chassisVelocity().vyMetersPerSecond, EPSILON);
    assertEquals(TargetSelector.HubSelection.ACTIVE, solution.selectedHub());
    assertEquals(PieceState.HELD, solution.pieceState());
    assertEquals(
        ShooterConstants.defaultLaunchSpeedMetersPerSec,
        solution.launchSpeedMetersPerSec(),
        EPSILON);
    assertEquals(
        ShooterConstants.defaultLaunchAngle.getRadians(),
        solution.launchAngle().getRadians(),
        EPSILON);
    assertEquals(
        ShooterConstants.defaultLaunchHeightMeters, solution.launchHeightMeters(), EPSILON);
    assertEquals(
        Math.atan2(-solution.airtimeSeconds(), 4.0),
        solution.targetHeading().getRadians(),
        EPSILON);
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
