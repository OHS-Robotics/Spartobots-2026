package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class TargetSelectorTest {
  private static final double EPSILON = 1e-9;

  @Test
  void selectsActiveHubFromMatchState() {
    MatchStateProvider.MatchState matchState =
        new MatchStateProvider.MatchState(
            Optional.of(Alliance.Blue),
            MatchStateProvider.TowerLetter.R,
            MatchStateProvider.Hub.BLUE,
            MatchStateProvider.Hub.RED,
            MatchStateProvider.Hub.RED,
            MatchStateProvider.MatchPhaseScoringContext.SHIFT_2);

    Pose3d selectedHub =
        TargetSelector.getHubPose3d(TargetSelector.HubSelection.ACTIVE, Alliance.Blue, matchState);

    assertEquals(FieldTargets.HUB.redPose().getX(), selectedHub.getX(), EPSILON);
    assertEquals(FieldTargets.HUB.redPose().getY(), selectedHub.getY(), EPSILON);
  }

  @Test
  void selectsAllianceDepotByAllianceColor() {
    FieldTargets.FieldZone blueDepot =
        TargetSelector.getIntakeZone(
            TargetSelector.IntakeZoneSelection.ALLIANCE_DEPOT, Alliance.Blue);
    FieldTargets.FieldZone redDepot =
        TargetSelector.getIntakeZone(
            TargetSelector.IntakeZoneSelection.ALLIANCE_DEPOT, Alliance.Red);

    assertEquals(
        FieldTargets.DEPOT_INTAKE.blueZone().center().getX(), blueDepot.center().getX(), EPSILON);
    assertEquals(
        FieldTargets.DEPOT_INTAKE.blueZone().center().getY(), blueDepot.center().getY(), EPSILON);
    assertEquals(
        FieldTargets.DEPOT_INTAKE.redZone().center().getX(), redDepot.center().getX(), EPSILON);
    assertEquals(
        FieldTargets.DEPOT_INTAKE.redZone().center().getY(), redDepot.center().getY(), EPSILON);
  }

  @Test
  void mirrorsAutoStartPoseForOpponentAlliance() {
    Pose2d blueLower = FieldTargets.AUTO_START_LOWER.bluePose();
    Pose2d redLower =
        TargetSelector.getAutoStartPose(
            TargetSelector.AutoStartSelection.OPPONENT_LOWER, Alliance.Blue);

    assertEquals(FieldConstants.FIELD_LENGTH_METERS - blueLower.getX(), redLower.getX(), EPSILON);
    assertEquals(FieldConstants.FIELD_WIDTH_METERS - blueLower.getY(), redLower.getY(), EPSILON);
    assertEquals(Math.PI, redLower.getRotation().getRadians(), EPSILON);
  }

  @Test
  void selectsAllianceOutpost() {
    Pose2d outpost =
        TargetSelector.getOutpostPose(TargetSelector.OutpostSelection.ALLIANCE, Alliance.Red);

    assertEquals(FieldTargets.OUTPOST.redPose().getX(), outpost.getX(), EPSILON);
    assertEquals(FieldTargets.OUTPOST.redPose().getY(), outpost.getY(), EPSILON);
  }
}
