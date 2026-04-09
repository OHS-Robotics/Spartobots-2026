package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveRoutingTest {
  @BeforeEach
  void setUp() {
    AutoBuilder.resetForTesting();
  }

  @Test
  void selectsAllianceHubPoseByAlliance() {
    assertEquals(
        Constants.blueHub.getX(),
        Drive.selectAllianceHubPose(Optional.of(Alliance.Blue)).getX(),
        1e-9);
    assertEquals(
        Constants.redHub.getX(),
        Drive.selectAllianceHubPose(Optional.of(Alliance.Red)).getX(),
        1e-9);
  }

  @Test
  void selectsOutpostApproachPosesByAlliance() {
    var bluePoses = Drive.selectOutpostApproachPoses(Optional.of(Alliance.Blue));
    assertEquals(Constants.blueOutpostBefore.getX(), bluePoses[0].getX(), 1e-9);
    assertEquals(Constants.blueOutpost.getX(), bluePoses[1].getX(), 1e-9);

    var redPoses = Drive.selectOutpostApproachPoses(Optional.of(Alliance.Red));
    assertEquals(Constants.redOutpostBefore.getX(), redPoses[0].getX(), 1e-9);
    assertEquals(Constants.redOutpost.getX(), redPoses[1].getX(), 1e-9);
  }

  @Test
  void driveToOutpostCommandReturnsDeferredCommand() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    var command = drive.driveToOutpostCommand();
    assertNotNull(command);
    assertEquals("DeferredCommand", command.getClass().getSimpleName());
  }

  @Test
  void pathfindToTranslationReturnsDeferredCommand() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    var command = drive.pathfindToTranslation(new Translation2d(2.0, 3.0));
    assertNotNull(command);
    assertEquals("DeferredCommand", command.getClass().getSimpleName());
  }

  @Test
  void selectsTrenchHeadingTowardTheTarget() {
    assertEquals(
        0.0,
        Drive.selectTrenchHeadingRadians(
            new Pose2d(2.5, 1.0, Rotation2d.kZero), new Translation2d(5.9, 0.6)),
        1e-9);
    assertEquals(
        Math.PI,
        Drive.selectTrenchHeadingRadians(
            new Pose2d(7.5, 1.0, Rotation2d.kZero), new Translation2d(5.9, 0.6)),
        1e-9);
  }

  @Test
  void followNamedPathReturnsDeferredCommand() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    var command = drive.followNamedPath("Depot");
    assertNotNull(command);
    assertEquals("DeferredCommand", command.getClass().getSimpleName());
  }

  @Test
  void deployedPathFilesLoadSuccessfully() throws Exception {
    Path pathsDirectory =
        Path.of(System.getProperty("user.dir"), "src/main/deploy/pathplanner/paths");
    if (!Files.isDirectory(pathsDirectory)) {
      return;
    }

    try (var pathFiles = Files.list(pathsDirectory)) {
      var pathNames =
          pathFiles
              .filter(path -> path.getFileName().toString().endsWith(".path"))
              .map(path -> path.getFileName().toString().replaceFirst("\\.path$", ""))
              .toList();

      assertFalse(pathNames.isEmpty());
      for (String pathName : pathNames) {
        assertNotNull(PathPlannerPath.fromPathFile(pathName), pathName);
      }
    }
  }
}
