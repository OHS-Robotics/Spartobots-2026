package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

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

    assertEquals(
        0.0,
        FieldTargetingService.selectLadderAlignPose(Optional.of(Alliance.Blue))
            .getRotation()
            .getDegrees(),
        1e-9);
    assertEquals(
        180.0,
        FieldTargetingService.selectLadderAlignPose(Optional.of(Alliance.Red))
            .getRotation()
            .getDegrees(),
        1e-9);
  }
}
