package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
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

  @Test
  void pathPlannerProjectSettingsMatchDriveConstants() throws Exception {
    Path settingsPath =
        Path.of(System.getProperty("user.dir"), "src/main/deploy/pathplanner/settings.json");
    JSONObject settings = (JSONObject) new JSONParser().parse(Files.readString(settingsPath));

    assertEquals(DriveConstants.bumperWidthYMeters, getJsonDouble(settings, "robotWidth"), 1e-9);
    assertEquals(DriveConstants.bumperLengthXMeters, getJsonDouble(settings, "robotLength"), 1e-9);
    assertEquals(Boolean.TRUE, settings.get("holonomicMode"));
    assertEquals(
        DriveConstants.maxSpeedMetersPerSec, getJsonDouble(settings, "defaultMaxVel"), 1e-9);
    assertEquals(
        DriveConstants.maxAccelerationMeterPerSecSquared,
        getJsonDouble(settings, "defaultMaxAccel"),
        1e-9);
    assertEquals(
        Math.toDegrees(DriveConstants.maxRotationalSpeedRadiansPerSec),
        getJsonDouble(settings, "defaultMaxAngVel"),
        1e-9);
    assertEquals(
        Math.toDegrees(DriveConstants.maxRotationalAccelerationRadiansPerSecSquared),
        getJsonDouble(settings, "defaultMaxAngAccel"),
        1e-9);
    assertEquals(12.0, getJsonDouble(settings, "defaultNominalVoltage"), 1e-9);
    assertEquals(DriveConstants.robotMassKg, getJsonDouble(settings, "robotMass"), 1e-9);
    assertEquals(DriveConstants.robotMOI, getJsonDouble(settings, "robotMOI"), 1e-9);
    assertEquals(DriveConstants.trackWidth, getJsonDouble(settings, "robotTrackwidth"), 1e-9);
    assertEquals(
        DriveConstants.wheelRadiusMeters, getJsonDouble(settings, "driveWheelRadius"), 1e-9);
    assertEquals(DriveConstants.driveMotorReduction, getJsonDouble(settings, "driveGearing"), 1e-9);
    assertEquals(
        DriveConstants.maxSpeedMetersPerSec, getJsonDouble(settings, "maxDriveSpeed"), 1e-9);
    assertEquals("NEO", settings.get("driveMotorType"));
    assertEquals(
        DriveConstants.driveMotorCurrentLimit, getJsonDouble(settings, "driveCurrentLimit"), 1e-9);
    assertEquals(DriveConstants.wheelCOF, getJsonDouble(settings, "wheelCOF"), 1e-9);

    assertModuleSettingMatches(
        DriveConstants.moduleTranslations[0], settings, "flModuleX", "flModuleY");
    assertModuleSettingMatches(
        DriveConstants.moduleTranslations[1], settings, "frModuleX", "frModuleY");
    assertModuleSettingMatches(
        DriveConstants.moduleTranslations[2], settings, "blModuleX", "blModuleY");
    assertModuleSettingMatches(
        DriveConstants.moduleTranslations[3], settings, "brModuleX", "brModuleY");
  }

  @Test
  void deployedDefaultConstrainedPathsMatchDriveConstants() throws Exception {
    Path pathsDirectory =
        Path.of(System.getProperty("user.dir"), "src/main/deploy/pathplanner/paths");
    if (!Files.isDirectory(pathsDirectory)) {
      return;
    }

    try (var pathFiles = Files.list(pathsDirectory)) {
      var paths =
          pathFiles.filter(path -> path.getFileName().toString().endsWith(".path")).toList();

      assertFalse(paths.isEmpty());
      for (Path path : paths) {
        JSONObject pathJson = (JSONObject) new JSONParser().parse(Files.readString(path));
        if (!Boolean.TRUE.equals(pathJson.get("useDefaultConstraints"))) {
          continue;
        }

        JSONObject constraints = (JSONObject) pathJson.get("globalConstraints");
        String pathName = path.getFileName().toString();
        assertEquals(
            DriveConstants.maxSpeedMetersPerSec,
            getJsonDouble(constraints, "maxVelocity"),
            1e-9,
            pathName);
        assertEquals(
            DriveConstants.maxAccelerationMeterPerSecSquared,
            getJsonDouble(constraints, "maxAcceleration"),
            1e-9,
            pathName);
        assertEquals(
            Math.toDegrees(DriveConstants.maxRotationalSpeedRadiansPerSec),
            getJsonDouble(constraints, "maxAngularVelocity"),
            1e-9,
            pathName);
        assertEquals(
            Math.toDegrees(DriveConstants.maxRotationalAccelerationRadiansPerSecSquared),
            getJsonDouble(constraints, "maxAngularAcceleration"),
            1e-9,
            pathName);
        assertEquals(12.0, getJsonDouble(constraints, "nominalVoltage"), 1e-9, pathName);
      }
    }
  }

  @Test
  void bumpReturnPathsAreMirroredAcrossFieldWidth() throws Exception {
    PathPlannerPath lowerPath = PathPlannerPath.fromPathFile("Bump Right In");
    PathPlannerPath upperPath = PathPlannerPath.fromPathFile("Bump Left In");

    assertEquals(lowerPath.getWaypoints().size(), upperPath.getWaypoints().size());
    for (int i = 0; i < lowerPath.getWaypoints().size(); i++) {
      assertMirroredAcrossFieldWidth(
          lowerPath.getWaypoints().get(i), upperPath.getWaypoints().get(i));
    }
    assertEquals(
        lowerPath.getIdealStartingState().velocityMPS(),
        upperPath.getIdealStartingState().velocityMPS(),
        1e-9);
    assertEquals(
        -lowerPath.getIdealStartingState().rotation().getRadians(),
        upperPath.getIdealStartingState().rotation().getRadians(),
        1e-9);
    assertEquals(
        lowerPath.getGoalEndState().velocityMPS(), upperPath.getGoalEndState().velocityMPS(), 1e-9);
    assertEquals(
        -lowerPath.getGoalEndState().rotation().getRadians(),
        upperPath.getGoalEndState().rotation().getRadians(),
        1e-9);
  }

  @Test
  void pathfinderNavgridIsMirroredAcrossFieldWidth() throws Exception {
    Path navgridPath =
        Path.of(System.getProperty("user.dir"), "src/main/deploy/pathplanner/navgrid.json");
    JSONObject navgrid = (JSONObject) new JSONParser().parse(Files.readString(navgridPath));
    JSONArray grid = (JSONArray) navgrid.get("grid");

    for (int y = 0; y < grid.size(); y++) {
      JSONArray row = (JSONArray) grid.get(y);
      JSONArray mirroredRow = (JSONArray) grid.get(grid.size() - 1 - y);
      assertEquals(row.size(), mirroredRow.size());
      for (int x = 0; x < row.size(); x++) {
        assertEquals(row.get(x), mirroredRow.get(x), "Navgrid mismatch at x=" + x + ", y=" + y);
      }
    }
  }

  private static double getJsonDouble(JSONObject object, String key) {
    Object value = object.get(key);
    assertNotNull(value, key);
    return ((Number) value).doubleValue();
  }

  private static void assertModuleSettingMatches(
      Translation2d moduleTranslation, JSONObject settings, String xKey, String yKey) {
    assertEquals(moduleTranslation.getX(), getJsonDouble(settings, xKey), 1e-9);
    assertEquals(moduleTranslation.getY(), getJsonDouble(settings, yKey), 1e-9);
  }

  private static void assertMirroredAcrossFieldWidth(Waypoint lower, Waypoint upper) {
    assertMirroredAcrossFieldWidth(lower.prevControl(), upper.prevControl());
    assertMirroredAcrossFieldWidth(lower.anchor(), upper.anchor());
    assertMirroredAcrossFieldWidth(lower.nextControl(), upper.nextControl());
  }

  private static void assertMirroredAcrossFieldWidth(
      Translation2d lowerTranslation, Translation2d upperTranslation) {
    assertEquals(lowerTranslation == null, upperTranslation == null);
    if (lowerTranslation == null) {
      return;
    }

    assertEquals(lowerTranslation.getX(), upperTranslation.getX(), 1e-9);
    assertEquals(Constants.fieldWidth - lowerTranslation.getY(), upperTranslation.getY(), 1e-9);
  }
}
