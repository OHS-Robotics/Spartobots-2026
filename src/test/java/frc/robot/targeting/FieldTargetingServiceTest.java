package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
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
