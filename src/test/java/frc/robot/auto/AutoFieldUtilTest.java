package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldTargets;
import frc.robot.TargetSelector;
import org.junit.jupiter.api.Test;

class AutoFieldUtilTest {
  @Test
  void nearestParkKeepsCurrentZoneWhenAlreadyInsideIt() {
    TargetSelector.ParkZoneSelection selection =
        AutoFieldUtil.selectNearestParkZone(
            FieldTargets.UPPER_PARK_ZONE.blueZone().centerPose(Rotation2d.kZero),
            FieldTargets.LOWER_PARK_ZONE.blueZone(),
            FieldTargets.UPPER_PARK_ZONE.blueZone());

    assertEquals(TargetSelector.ParkZoneSelection.ALLIANCE_UPPER, selection);
  }

  @Test
  void headingFallsBackToCurrentRotationAtZoneCenter() {
    Pose2d pose =
        new Pose2d(FieldTargets.LOWER_PARK_ZONE.blueZone().center(), Rotation2d.fromDegrees(27.0));

    Rotation2d heading =
        AutoFieldUtil.computeHeadingToZoneCenter(pose, FieldTargets.LOWER_PARK_ZONE.blueZone());

    assertEquals(pose.getRotation().getRadians(), heading.getRadians(), 1e-9);
  }
}
