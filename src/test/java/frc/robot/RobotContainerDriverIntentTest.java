package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.superstructure.SuperstructureGoal;
import frc.robot.subsystems.superstructure.SuperstructureStatus;
import java.util.Optional;
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

  @Test
  void shotReadyReasonReportsPoseTrustBeforeAlignment() {
    SuperstructureStatus status =
        new SuperstructureStatus(
            Optional.of(new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE)),
            new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.AIM),
            frc.robot.subsystems.superstructure.Superstructure.PieceState.HELD,
            true,
            true,
            false,
            Pose2d.kZero,
            Rotation2d.kZero,
            null,
            false);

    assertEquals("POSE_UNTRUSTED", RobotContainer.getShotReadyReason(status, false));
    assertFalse(RobotContainer.isShotReady(status, false));
  }

  @Test
  void shotReadyReasonReportsReadyWhenAllGatesPass() {
    SuperstructureStatus status =
        new SuperstructureStatus(
            Optional.of(new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE)),
            new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE),
            frc.robot.subsystems.superstructure.Superstructure.PieceState.READY_TO_FIRE,
            true,
            true,
            true,
            Pose2d.kZero,
            Rotation2d.kZero,
            null,
            true);

    assertEquals("READY", RobotContainer.getShotReadyReason(status, true));
    assertTrue(RobotContainer.isShotReady(status, true));
  }

  @Test
  void currentAutoActionReflectsHubShotPhase() {
    assertEquals(
        "AUTO_FACE_AND_SCORE_FIRE",
        RobotContainer.getCurrentAutoAction(
            new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE)));
  }

  @Test
  void currentAutoActionReflectsQuickParkPhase() {
    assertEquals(
        "QUICK_PARK_LEVEL",
        RobotContainer.getCurrentAutoAction(
            new SuperstructureGoal.Endgame(
                SuperstructureGoal.EndgamePhase.LEVEL,
                TargetSelector.ParkZoneSelection.ALLIANCE_LOWER)));
  }

  @Test
  void poseConfidenceLabelReflectsTrustState() {
    assertEquals("TRUSTED", RobotContainer.getPoseConfidenceLabel(true));
    assertEquals("UNTRUSTED", RobotContainer.getPoseConfidenceLabel(false));
  }

  @Test
  void magazineCountIsBinaryForSinglePieceMagazine() {
    assertEquals(1, RobotContainer.getMagazineCount(true));
    assertEquals(0, RobotContainer.getMagazineCount(false));
  }
}
