package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class FieldTargetingServiceTest {
  private static final double bluePoint5X = 2.6397276107874292;
  private static final double blueLowerPoint5Y = 2.4890436308262713;

  @Test
  void selectsCompetitionAutoPosesByAlliance() {
    Pose2d arbitraryBlueStart = new Pose2d(0.25, 1.75, Rotation2d.fromDegrees(23.0));
    Pose2d arbitraryRedStart = new Pose2d(16.0, 6.1, Rotation2d.fromDegrees(-12.0));

    assertEquals(
        Constants.competitionAutoBlueStart.getX(),
        FieldTargetingService.selectCompetitionAutoStartPose(
                Optional.of(Alliance.Blue), arbitraryBlueStart)
            .getX(),
        1e-9);
    assertEquals(
        Constants.competitionAutoRedStart.getX(),
        FieldTargetingService.selectCompetitionAutoStartPose(
                Optional.of(Alliance.Red), arbitraryRedStart)
            .getX(),
        1e-9);
    assertEquals(
        arbitraryBlueStart.getY(),
        FieldTargetingService.selectCompetitionAutoStartPose(
                Optional.of(Alliance.Blue), arbitraryBlueStart)
            .getY(),
        1e-9);
    assertEquals(
        arbitraryRedStart.getY(),
        FieldTargetingService.selectCompetitionAutoStartPose(
                Optional.of(Alliance.Red), arbitraryRedStart)
            .getY(),
        1e-9);
    assertEquals(
        Rotation2d.kZero.getRadians(),
        FieldTargetingService.selectCompetitionAutoStartPose(
                Optional.of(Alliance.Blue), arbitraryBlueStart)
            .getRotation()
            .getRadians(),
        1e-9);
    assertEquals(
        Rotation2d.kZero.getRadians(),
        FieldTargetingService.selectCompetitionAutoStartPose(
                Optional.of(Alliance.Red), arbitraryRedStart)
            .getRotation()
            .getRadians(),
        1e-9);

    assertEquals(
        -130.0,
        FieldTargetingService.selectOpeningShotPose(Optional.of(Alliance.Blue))
            .getRotation()
            .getDegrees(),
        1e-9);
    assertEquals(
        50.0,
        FieldTargetingService.selectOpeningShotPose(Optional.of(Alliance.Red))
            .getRotation()
            .getDegrees(),
        1e-9);
  }

  @Test
  void competitionAutoStartPosePreservesBackwardsStartingHeading() {
    Pose2d blueBackwardsStart = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(170.0));
    Pose2d redBackwardsStart = new Pose2d(14.5, 1.0, Rotation2d.fromDegrees(10.0));
    Pose2d blueForwardStart = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(10.0));
    Pose2d redForwardStart = new Pose2d(14.5, 1.0, Rotation2d.fromDegrees(170.0));

    assertEquals(
        Rotation2d.kPi.getRadians(),
        FieldTargetingService.selectCompetitionAutoStartPose(
                Optional.of(Alliance.Blue), blueBackwardsStart)
            .getRotation()
            .getRadians(),
        1e-9);
    assertEquals(
        Rotation2d.kZero.getRadians(),
        FieldTargetingService.selectCompetitionAutoStartPose(
                Optional.of(Alliance.Red), redBackwardsStart)
            .getRotation()
            .getRadians(),
        1e-9);
    assertEquals(
        Rotation2d.kZero.getRadians(),
        FieldTargetingService.selectCompetitionAutoStartPose(
                Optional.of(Alliance.Blue), blueForwardStart)
            .getRotation()
            .getRadians(),
        1e-9);
    assertEquals(
        Rotation2d.kPi.getRadians(),
        FieldTargetingService.selectCompetitionAutoStartPose(
                Optional.of(Alliance.Red), redForwardStart)
            .getRotation()
            .getRadians(),
        1e-9);
  }

  @Test
  void selectsCompetitionAutoCycleTargetsByAllianceAndStartingLane() {
    Pose2d blueLowerStart = new Pose2d(2.0, 1.0, Rotation2d.kZero);
    Pose2d blueUpperStart =
        new Pose2d(2.0, Constants.fieldWidth - blueLowerStart.getY(), Rotation2d.kZero);
    Pose2d redMirroredLowerStart = new Pose2d(14.5, Constants.fieldWidth - 1.0, Rotation2d.kPi);
    Pose2d redLowerStart = new Pose2d(14.5, 1.0, Rotation2d.kPi);

    var blueLowerTargets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Blue), blueLowerStart);
    var blueUpperTargets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Blue), blueUpperStart);
    var redMirroredLowerTargets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Red), redMirroredLowerStart);
    var redLowerTargets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Red), redLowerStart);

    assertPoseDistanceFromHub(blueLowerTargets.point1Pose(), Constants.blueHub, 4.0);
    assertEquals(bluePoint5X, blueLowerTargets.point5Pose().getX(), 1e-9);
    assertEquals(blueLowerPoint5Y, blueLowerTargets.point5Pose().getY(), 1e-9);

    assertPoseDistanceFromHub(blueUpperTargets.point1Pose(), Constants.blueHub, 4.0);
    assertPoint5ToPoint1CanPathfind(blueLowerTargets);
    assertPoint5ToPoint1CanPathfind(blueUpperTargets);
    assertEquals("Bump Right In", blueLowerTargets.bumpReturnPathName());
    assertEquals("Bump Left In", blueUpperTargets.bumpReturnPathName());
    assertBumperClearOfBlueSideCenterline(blueLowerTargets.point2Pose());
    assertBumperClearOfBlueSideCenterline(blueLowerTargets.point3Pose());
    assertBumperClearOfLowerLaneMidline(blueLowerTargets.point2Pose());
    assertBumperClearOfLowerLaneMidline(blueLowerTargets.point3Pose());
    assertBumperClearOfBlueSideCenterline(blueUpperTargets.point2Pose());
    assertBumperClearOfBlueSideCenterline(blueUpperTargets.point3Pose());
    assertBumperClearOfUpperLaneMidline(blueUpperTargets.point2Pose());
    assertBumperClearOfUpperLaneMidline(blueUpperTargets.point3Pose());
    assertEquals(5.573116613700566, blueLowerTargets.point4Pose().getX(), 1e-9);
    assertEquals(2.4890436308262713, blueLowerTargets.point4Pose().getY(), 1e-9);
    assertMirroredAcrossFieldWidth(blueLowerTargets.startPose(), blueUpperTargets.startPose());
    assertMirroredAcrossFieldWidth(blueLowerTargets.point1Pose(), blueUpperTargets.point1Pose());
    assertMirroredAcrossFieldWidth(blueLowerTargets.point2Pose(), blueUpperTargets.point2Pose());
    assertMirroredAcrossFieldWidth(blueLowerTargets.point3Pose(), blueUpperTargets.point3Pose());
    assertMirroredAcrossFieldWidth(blueLowerTargets.point4Pose(), blueUpperTargets.point4Pose());
    assertMirroredAcrossFieldWidth(blueLowerTargets.point5Pose(), blueUpperTargets.point5Pose());

    assertPoseDistanceFromHub(redMirroredLowerTargets.point1Pose(), Constants.redHub, 4.0);
    assertEquals(
        Constants.fieldLength - bluePoint5X, redMirroredLowerTargets.point5Pose().getX(), 1e-9);
    assertEquals(
        Constants.fieldWidth - blueLowerPoint5Y, redMirroredLowerTargets.point5Pose().getY(), 1e-9);
    assertEquals("Bump Right In", redMirroredLowerTargets.bumpReturnPathName());
    assertEquals(
        Constants.fieldLength - 5.573116613700566,
        redMirroredLowerTargets.point4Pose().getX(),
        1e-9);
    assertEquals(
        Constants.fieldWidth - 2.4890436308262713,
        redMirroredLowerTargets.point4Pose().getY(),
        1e-9);

    assertPoseDistanceFromHub(redLowerTargets.point1Pose(), Constants.redHub, 4.0);
    assertBumperClearOfRedSideCenterline(redLowerTargets.point2Pose());
    assertBumperClearOfRedSideCenterline(redLowerTargets.point3Pose());
    assertBumperClearOfLowerLaneMidline(redLowerTargets.point2Pose());
    assertBumperClearOfLowerLaneMidline(redLowerTargets.point3Pose());
    assertEquals(Constants.fieldLength - bluePoint5X, redLowerTargets.point5Pose().getX(), 1e-9);
    assertEquals(blueLowerPoint5Y, redLowerTargets.point5Pose().getY(), 1e-9);
    assertEquals("Bump Left In", redLowerTargets.bumpReturnPathName());
    assertEquals(
        Constants.fieldLength - 5.573116613700566, redLowerTargets.point4Pose().getX(), 1e-9);
    assertEquals(2.4890436308262713, redLowerTargets.point4Pose().getY(), 1e-9);
    assertPoint5ToPoint1CanPathfind(redMirroredLowerTargets);
    assertPoint5ToPoint1CanPathfind(redLowerTargets);
  }

  @Test
  void competitionAutoPoint5TargetsMatchBumpReturnPathEndpoints() throws Exception {
    Pose2d blueLowerStart = new Pose2d(2.0, 1.0, Rotation2d.kZero);
    Pose2d blueUpperStart =
        new Pose2d(2.0, Constants.fieldWidth - blueLowerStart.getY(), Rotation2d.kZero);

    var blueLowerTargets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Blue), blueLowerStart);
    var blueUpperTargets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Blue), blueUpperStart);
    var redLowerTargets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Red), blueLowerStart);

    assertPoint4MatchesBumpReturnStart(blueLowerTargets.point4Pose(), "Bump Right In");
    assertPoint4MatchesBumpReturnStart(blueUpperTargets.point4Pose(), "Bump Left In");
    assertPoint5MatchesBumpReturnEndpoint(blueLowerTargets.point5Pose(), "Bump Right In");
    assertPoint5MatchesBumpReturnEndpoint(blueUpperTargets.point5Pose(), "Bump Left In");
    assertRedTargetsMatchFlippedBumpReturnPath(redLowerTargets);
  }

  @Test
  void competitionAutoPoint2And3MoveTowardDriverStationByCycle() {
    Pose2d blueLowerStart = new Pose2d(2.0, 1.0, Rotation2d.kZero);
    Pose2d blueUpperStart =
        new Pose2d(2.0, Constants.fieldWidth - blueLowerStart.getY(), Rotation2d.kZero);
    Pose2d redLowerStart = new Pose2d(14.5, 1.0, Rotation2d.kPi);

    var blueLowerCycle0Targets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Blue), blueLowerStart, 0);
    var blueLowerCycle1Targets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Blue), blueLowerStart, 1);
    var blueLowerCycle2Targets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Blue), blueLowerStart, 2);
    var blueUpperCycle1Targets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Blue), blueUpperStart, 1);
    var redLowerCycle0Targets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Red), redLowerStart, 0);
    var redLowerCycle1Targets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Red), redLowerStart, 1);

    assertBlueCycleXShift(
        blueLowerCycle0Targets.point2Pose(), blueLowerCycle1Targets.point2Pose(), 1.0);
    assertBlueCycleXShift(
        blueLowerCycle0Targets.point3Pose(), blueLowerCycle1Targets.point3Pose(), 1.0);
    assertBlueCycleXShift(
        blueLowerCycle0Targets.point2Pose(), blueLowerCycle2Targets.point2Pose(), 2.0);
    assertBlueCycleXShift(
        blueLowerCycle0Targets.point3Pose(), blueLowerCycle2Targets.point3Pose(), 2.0);
    assertRedCycleXShift(
        redLowerCycle0Targets.point2Pose(), redLowerCycle1Targets.point2Pose(), 1.0);
    assertRedCycleXShift(
        redLowerCycle0Targets.point3Pose(), redLowerCycle1Targets.point3Pose(), 1.0);
    assertMirroredAcrossFieldWidth(
        blueLowerCycle1Targets.point2Pose(), blueUpperCycle1Targets.point2Pose());
    assertMirroredAcrossFieldWidth(
        blueLowerCycle1Targets.point3Pose(), blueUpperCycle1Targets.point3Pose());
  }

  private static void assertMirroredAcrossFieldWidth(Pose2d lowerPose, Pose2d upperPose) {
    assertEquals(lowerPose.getX(), upperPose.getX(), 1e-9);
    assertEquals(Constants.fieldWidth - lowerPose.getY(), upperPose.getY(), 1e-9);
    assertEquals(-lowerPose.getRotation().getRadians(), upperPose.getRotation().getRadians(), 1e-9);
  }

  private static void assertBlueCycleXShift(
      Pose2d cycle0Pose, Pose2d shiftedPose, double expectedShiftMeters) {
    assertEquals(cycle0Pose.getX() - expectedShiftMeters, shiftedPose.getX(), 1e-9);
    assertEquals(cycle0Pose.getY(), shiftedPose.getY(), 1e-9);
    assertEquals(
        cycle0Pose.getRotation().getRadians(), shiftedPose.getRotation().getRadians(), 1e-9);
  }

  private static void assertPoseDistanceFromHub(
      Pose2d robotPose, Pose2d hubPose, double expectedDistanceMeters) {
    assertEquals(
        expectedDistanceMeters,
        robotPose.getTranslation().getDistance(hubPose.getTranslation()),
        1e-4);
  }

  private static void assertRedCycleXShift(
      Pose2d cycle0Pose, Pose2d shiftedPose, double expectedShiftMeters) {
    assertEquals(cycle0Pose.getX() + expectedShiftMeters, shiftedPose.getX(), 1e-9);
    assertEquals(cycle0Pose.getY(), shiftedPose.getY(), 1e-9);
    assertEquals(
        cycle0Pose.getRotation().getRadians(), shiftedPose.getRotation().getRadians(), 1e-9);
  }

  private static void assertPoint5ToPoint1CanPathfind(
      FieldTargetingService.CompetitionAutoTargets targets) {
    assertTrue(
        targets.point5Pose().getTranslation().getDistance(targets.point1Pose().getTranslation())
            > 0.5,
        "Point 1 should sit outside PathPlanner's already-at-goal cutoff from point 5");
  }

  private static void assertPoint5MatchesBumpReturnEndpoint(Pose2d point5Pose, String pathName)
      throws Exception {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    Waypoint endpoint = path.getWaypoints().get(path.getWaypoints().size() - 1);

    assertEquals(endpoint.anchor().getX(), point5Pose.getX(), 1e-9);
    assertEquals(endpoint.anchor().getY(), point5Pose.getY(), 1e-9);
    assertEquals(
        path.getGoalEndState().rotation().getRadians(),
        point5Pose.getRotation().getRadians(),
        1e-6);
  }

  private static void assertPoint4MatchesBumpReturnStart(Pose2d point4Pose, String pathName)
      throws Exception {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    Waypoint startpoint = path.getWaypoints().get(0);

    assertEquals(startpoint.anchor().getX(), point4Pose.getX(), 1e-9);
    assertEquals(startpoint.anchor().getY(), point4Pose.getY(), 1e-9);
    assertEquals(
        path.getIdealStartingState().rotation().getRadians(),
        point4Pose.getRotation().getRadians(),
        1e-6);
  }

  private static void assertRedTargetsMatchFlippedBumpReturnPath(
      FieldTargetingService.CompetitionAutoTargets targets) throws Exception {
    FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;
    FlippingUtil.fieldSizeX = Constants.fieldLength;
    FlippingUtil.fieldSizeY = Constants.fieldWidth;
    PathPlannerPath flippedPath =
        PathPlannerPath.fromPathFile(targets.bumpReturnPathName()).flipPath();
    Waypoint startpoint = flippedPath.getWaypoints().get(0);
    Waypoint endpoint = flippedPath.getWaypoints().get(flippedPath.getWaypoints().size() - 1);

    assertEquals(startpoint.anchor().getX(), targets.point4Pose().getX(), 1e-9);
    assertEquals(startpoint.anchor().getY(), targets.point4Pose().getY(), 1e-9);
    assertEquals(endpoint.anchor().getX(), targets.point5Pose().getX(), 1e-9);
    assertEquals(endpoint.anchor().getY(), targets.point5Pose().getY(), 1e-9);
  }

  private static void assertBumperClearOfBlueSideCenterline(Pose2d pose) {
    assertTrue(
        pose.getX() + (DriveConstants.bumperLengthXMeters / 2.0) < Constants.midLineX,
        "Blue-side target bumper should stay behind the field centerline");
  }

  private static void assertBumperClearOfRedSideCenterline(Pose2d pose) {
    assertTrue(
        pose.getX() - (DriveConstants.bumperLengthXMeters / 2.0) > Constants.midLineX,
        "Red-side target bumper should stay behind the field centerline");
  }

  private static void assertBumperClearOfLowerLaneMidline(Pose2d pose) {
    assertTrue(
        pose.getY() + (DriveConstants.bumperWidthYMeters / 2.0) < Constants.midLineY,
        "Lower-lane target bumper should stay below the lane midline");
  }

  private static void assertBumperClearOfUpperLaneMidline(Pose2d pose) {
    assertTrue(
        pose.getY() - (DriveConstants.bumperWidthYMeters / 2.0) > Constants.midLineY,
        "Upper-lane target bumper should stay above the lane midline");
  }

  @Test
  void detectsWhenRobotIsNearTrench() {
    assertTrue(
        FieldTargetingService.isNearTrench(
            new Pose2d(
                Constants.blueTrenchBottomInner.getX(),
                Constants.blueTrenchBottomInner.getY() + 0.2,
                Rotation2d.kZero)));
    assertTrue(
        FieldTargetingService.isNearTrench(
            new Pose2d(
                Constants.redTrenchTopOuter.getX() - 0.2,
                Constants.redTrenchTopOuter.getY(),
                Rotation2d.kZero)));
    assertFalse(
        FieldTargetingService.isNearTrench(
            new Pose2d(Constants.fieldLength / 2.0, Constants.fieldWidth / 2.0, Rotation2d.kZero)));
    assertFalse(
        FieldTargetingService.isNearTrench(
            FieldTargetingService.selectOpeningShotPose(Optional.of(Alliance.Blue))));
  }
}
