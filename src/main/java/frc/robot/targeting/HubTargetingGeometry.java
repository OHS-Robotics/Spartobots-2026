package frc.robot.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.gamepiece.shooter.ShooterConstants;

public final class HubTargetingGeometry {
  private HubTargetingGeometry() {}

  public static Translation2d getLaunchOriginFieldPosition(Pose2d robotPose) {
    return robotPose
        .transformBy(new Transform2d(ShooterConstants.shooterMuzzleOffsetOnRobot, Rotation2d.kZero))
        .getTranslation();
  }

  public static Translation2d getVectorFromLaunchOriginToHub(Pose2d robotPose, Pose2d hubPose) {
    return hubPose.getTranslation().minus(getLaunchOriginFieldPosition(robotPose));
  }

  public static double getDistanceFromLaunchOriginToHub(Pose2d robotPose, Pose2d hubPose) {
    return getVectorFromLaunchOriginToHub(robotPose, hubPose).getNorm();
  }

  public static Rotation2d getRobotRotationToAimAtHub(Pose2d robotPose, Pose2d hubPose) {
    Translation2d toTarget = getVectorFromLaunchOriginToHub(robotPose, hubPose);
    return new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()))
        .plus(ShooterConstants.shooterFacingOffset);
  }
}
