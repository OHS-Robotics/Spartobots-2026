package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldTargets;
import frc.robot.TargetSelector;

public final class AutoFieldUtil {
  private AutoFieldUtil() {}

  public static TargetSelector.ParkZoneSelection selectNearestParkZone(Pose2d robotPose) {
    return selectNearestParkZone(
        robotPose,
        TargetSelector.getParkZone(TargetSelector.ParkZoneSelection.ALLIANCE_LOWER),
        TargetSelector.getParkZone(TargetSelector.ParkZoneSelection.ALLIANCE_UPPER));
  }

  public static TargetSelector.ParkZoneSelection selectNearestParkZone(
      Pose2d robotPose, FieldTargets.FieldZone lowerZone, FieldTargets.FieldZone upperZone) {
    if (lowerZone.contains(robotPose.getTranslation())) {
      return TargetSelector.ParkZoneSelection.ALLIANCE_LOWER;
    }
    if (upperZone.contains(robotPose.getTranslation())) {
      return TargetSelector.ParkZoneSelection.ALLIANCE_UPPER;
    }
    return distanceToZoneCenter(robotPose, lowerZone) <= distanceToZoneCenter(robotPose, upperZone)
        ? TargetSelector.ParkZoneSelection.ALLIANCE_LOWER
        : TargetSelector.ParkZoneSelection.ALLIANCE_UPPER;
  }

  public static Rotation2d computeHeadingToZoneCenter(
      Pose2d robotPose, FieldTargets.FieldZone zone) {
    var toZoneCenter = zone.center().minus(robotPose.getTranslation());
    if (toZoneCenter.getNorm() <= 1e-9) {
      return robotPose.getRotation();
    }
    return new Rotation2d(Math.atan2(toZoneCenter.getY(), toZoneCenter.getX()));
  }

  public static Pose2d computeZoneApproachPose(Pose2d robotPose, FieldTargets.FieldZone zone) {
    return zone.centerPose(computeHeadingToZoneCenter(robotPose, zone));
  }

  private static double distanceToZoneCenter(Pose2d robotPose, FieldTargets.FieldZone zone) {
    return robotPose.getTranslation().getDistance(zone.center());
  }
}
