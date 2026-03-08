package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.superstructure.SuperstructureGoal;
import org.junit.jupiter.api.Test;

class RobotContainerDriverIntentTest {
  @Test
  void acquirePrefersDepotWhenRobotIsInDepotZone() {
    SuperstructureGoal goal =
        RobotContainer.selectAcquireGoal(
            FieldTargets.DEPOT_INTAKE.blueZone().centerPose(Rotation2d.kZero),
            false,
            FieldTargets.DEPOT_INTAKE.blueZone(),
            FieldTargets.NEUTRAL_FLOOR_INTAKE);

    assertEquals(new SuperstructureGoal.IntakeDepot(SuperstructureGoal.IntakePhase.SETTLE), goal);
  }

  @Test
  void acquireFallsBackToStowWhenAlreadyHoldingPiece() {
    SuperstructureGoal goal =
        RobotContainer.selectAcquireGoal(
            FieldTargets.NEUTRAL_FLOOR_INTAKE.centerPose(Rotation2d.kZero),
            true,
            FieldTargets.DEPOT_INTAKE.blueZone(),
            FieldTargets.NEUTRAL_FLOOR_INTAKE);

    assertEquals(new SuperstructureGoal.Stow(), goal);
  }

  @Test
  void quickParkKeepsCurrentParkZoneWhenAlreadyInsideIt() {
    TargetSelector.ParkZoneSelection selection =
        RobotContainer.selectQuickParkZone(
            FieldTargets.UPPER_PARK_ZONE.blueZone().centerPose(Rotation2d.kZero),
            FieldTargets.LOWER_PARK_ZONE.blueZone(),
            FieldTargets.UPPER_PARK_ZONE.blueZone());

    assertEquals(TargetSelector.ParkZoneSelection.ALLIANCE_UPPER, selection);
  }

  @Test
  void quickParkHeadingFallsBackToCurrentRotationAtZoneCenter() {
    Pose2d pose =
        new Pose2d(FieldTargets.LOWER_PARK_ZONE.blueZone().center(), Rotation2d.fromDegrees(27.0));

    Rotation2d heading =
        RobotContainer.computeHeadingToZoneCenter(pose, FieldTargets.LOWER_PARK_ZONE.blueZone());

    assertEquals(pose.getRotation().getRadians(), heading.getRadians(), 1e-9);
  }
}
