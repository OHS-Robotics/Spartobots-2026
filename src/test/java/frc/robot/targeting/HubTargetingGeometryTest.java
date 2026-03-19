package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.gamepiece.shooter.ShooterConstants;
import org.junit.jupiter.api.Test;

class HubTargetingGeometryTest {
  @Test
  void launchOriginTracksRobotPoseRotation() {
    Pose2d robotPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(180.0));
    Translation2d launchOrigin = HubTargetingGeometry.getLaunchOriginFieldPosition(robotPose);

    assertEquals(2.25, launchOrigin.getX(), 1e-9);
    assertEquals(3.0, launchOrigin.getY(), 1e-9);
  }

  @Test
  void hubDistanceUsesLaunchOriginInsteadOfRobotCenter() {
    Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(180.0));
    Pose2d hubPose = new Pose2d(5.0, 2.0, Rotation2d.kZero);

    assertEquals(
        2.75, HubTargetingGeometry.getDistanceFromLaunchOriginToHub(robotPose, hubPose), 1e-9);
    assertNotEquals(
        robotPose.getTranslation().getDistance(hubPose.getTranslation()),
        HubTargetingGeometry.getDistanceFromLaunchOriginToHub(robotPose, hubPose),
        1e-9);
  }

  @Test
  void hubAimRotationUsesLaunchOriginInsteadOfRobotCenter() {
    Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(180.0));
    Pose2d hubPose = new Pose2d(5.0, 3.0, Rotation2d.kZero);

    Rotation2d centerBasedRotation =
        new Rotation2d(Math.atan2(1.0, 3.0)).plus(ShooterConstants.shooterFacingOffset);
    Rotation2d launchOriginBasedRotation =
        new Rotation2d(Math.atan2(1.0, 2.75)).plus(ShooterConstants.shooterFacingOffset);

    assertEquals(
        launchOriginBasedRotation.getDegrees(),
        HubTargetingGeometry.getRobotRotationToAimAtHub(robotPose, hubPose).getDegrees(),
        1e-9);
    assertNotEquals(
        centerBasedRotation.getDegrees(),
        HubTargetingGeometry.getRobotRotationToAimAtHub(robotPose, hubPose).getDegrees(),
        1e-9);
  }
}
