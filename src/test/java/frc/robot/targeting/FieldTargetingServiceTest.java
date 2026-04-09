package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class FieldTargetingServiceTest {
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
  void selectsCompetitionAutoCycleTargetsByAllianceAndStartingLane() {
    Pose2d blueLowerStart = new Pose2d(2.0, 1.0, Rotation2d.kZero);
    Pose2d blueUpperStart =
        new Pose2d(2.0, Constants.fieldWidth - blueLowerStart.getY(), Rotation2d.kZero);
    Pose2d redMirroredLowerStart = new Pose2d(14.5, Constants.fieldWidth - 1.0, Rotation2d.kPi);
    Pose2d redLowerStart = new Pose2d(14.5, 1.0, Rotation2d.kPi);

    var blueLowerTargets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Blue), blueLowerStart, 1.0);
    var blueUpperTargets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Blue), blueUpperStart, 1.0);
    var redMirroredLowerTargets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Red), redMirroredLowerStart, 1.0);
    var redLowerTargets =
        FieldTargetingService.selectCompetitionAutoTargets(
            Optional.of(Alliance.Red), redLowerStart, 1.0);

    assertEquals(
        Constants.blueTrenchBottomInner.getX() - 0.70, blueLowerTargets.point1Pose().getX(), 1e-9);
    assertEquals(
        Constants.blueTrenchBottomInner.getY() + 0.10, blueLowerTargets.point1Pose().getY(), 1e-9);
    assertEquals(
        Constants.robotStartingLineXMeters - 1.0, blueLowerTargets.point5Pose().getX(), 1e-9);
    assertEquals(blueLowerStart.getY(), blueLowerTargets.point5Pose().getY(), 1e-9);

    assertEquals(
        Constants.blueTrenchBottomInner.getX() - 0.70, blueUpperTargets.point1Pose().getX(), 1e-9);
    assertEquals(
        Constants.fieldWidth - (Constants.blueTrenchBottomInner.getY() + 0.10),
        blueUpperTargets.point1Pose().getY(),
        1e-9);
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

    assertEquals(
        Constants.fieldLength - (Constants.blueTrenchBottomInner.getX() - 0.70),
        redMirroredLowerTargets.point1Pose().getX(),
        1e-9);
    assertEquals(
        Constants.fieldWidth - (Constants.blueTrenchBottomInner.getY() + 0.10),
        redMirroredLowerTargets.point1Pose().getY(),
        1e-9);
    assertEquals(
        Constants.competitionAutoRedStart.getX() + 1.0,
        redMirroredLowerTargets.point5Pose().getX(),
        1e-9);
    assertEquals(redMirroredLowerStart.getY(), redMirroredLowerTargets.point5Pose().getY(), 1e-9);
    assertEquals("Bump Right In", redMirroredLowerTargets.bumpReturnPathName());
    assertEquals(
        Constants.fieldLength - 5.573116613700566,
        redMirroredLowerTargets.point4Pose().getX(),
        1e-9);
    assertEquals(
        Constants.fieldWidth - 2.4890436308262713,
        redMirroredLowerTargets.point4Pose().getY(),
        1e-9);

    assertEquals(
        Constants.fieldLength - (Constants.blueTrenchBottomInner.getX() - 0.70),
        redLowerTargets.point1Pose().getX(),
        1e-9);
    assertEquals(
        Constants.blueTrenchBottomInner.getY() + 0.10, redLowerTargets.point1Pose().getY(), 1e-9);
    assertBumperClearOfRedSideCenterline(redLowerTargets.point2Pose());
    assertBumperClearOfRedSideCenterline(redLowerTargets.point3Pose());
    assertBumperClearOfLowerLaneMidline(redLowerTargets.point2Pose());
    assertBumperClearOfLowerLaneMidline(redLowerTargets.point3Pose());
    assertEquals(
        Constants.competitionAutoRedStart.getX() + 1.0, redLowerTargets.point5Pose().getX(), 1e-9);
    assertEquals(redLowerStart.getY(), redLowerTargets.point5Pose().getY(), 1e-9);
    assertEquals("Bump Left In", redLowerTargets.bumpReturnPathName());
    assertEquals(
        Constants.fieldLength - 5.573116613700566, redLowerTargets.point4Pose().getX(), 1e-9);
    assertEquals(2.4890436308262713, redLowerTargets.point4Pose().getY(), 1e-9);
    assertPoint5ToPoint1CanPathfind(redMirroredLowerTargets);
    assertPoint5ToPoint1CanPathfind(redLowerTargets);
  }

  private static void assertMirroredAcrossFieldWidth(Pose2d lowerPose, Pose2d upperPose) {
    assertEquals(lowerPose.getX(), upperPose.getX(), 1e-9);
    assertEquals(Constants.fieldWidth - lowerPose.getY(), upperPose.getY(), 1e-9);
    assertEquals(-lowerPose.getRotation().getRadians(), upperPose.getRotation().getRadians(), 1e-9);
  }

  private static void assertPoint5ToPoint1CanPathfind(
      FieldTargetingService.CompetitionAutoTargets targets) {
    assertTrue(
        targets.point5Pose().getTranslation().getDistance(targets.point1Pose().getTranslation())
            > 0.5,
        "Point 1 should sit outside PathPlanner's already-at-goal cutoff from point 5");
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
