package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.gamepiece.hopper.Hopper;
import frc.robot.subsystems.gamepiece.hopper.HopperIO;
import frc.robot.subsystems.gamepiece.indexers.Indexers;
import frc.robot.subsystems.gamepiece.indexers.IndexersIO;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.intake.IntakeIO;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.subsystems.gamepiece.shooter.ShooterIO;
import frc.robot.superstructure.gamepiece.GamePieceCoordinator;
import frc.robot.targeting.FieldTargetingService;
import frc.robot.targeting.HubTargetingService;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

class AutoRoutinesTest {
  private static final String competitionAutoName = "Competition: Outpost -> Shoot -> Ladder";
  private static final String doNothingAutoName = "Do Nothing";
  private static final String testAutoName = "Test";
  private static final Pattern namedCommandPattern =
      Pattern.compile(
          "\"type\"\\s*:\\s*\"named\"\\s*,\\s*\"data\"\\s*:\\s*\\{\\s*\"name\"\\s*:\\s*\"([^\"]+)\"",
          Pattern.DOTALL);
  private AutoRoutines autoRoutines;

  @BeforeEach
  void setUp() {
    AutoBuilder.resetForTesting();
    autoRoutines = createAutoRoutines();
  }

  @Test
  void exposesOnlyCompetitionAutoAndDoNothing() {
    assertArrayEquals(
        new String[] {competitionAutoName, doNothingAutoName, testAutoName},
        autoRoutines.getAutoOptionNames());
  }

  @Test
  void fallsBackToCompetitionAutoWhenChooserSelectionIsNull() {
    LoggedDashboardChooser<Command> chooser =
        new LoggedDashboardChooser<>("Auto Test", new SendableChooser<>());

    Command selectedAuto = autoRoutines.selectAutonomousCommand(chooser);

    assertNotNull(selectedAuto);
    assertEquals(competitionAutoName, autoRoutines.getSelectedAutoName(selectedAuto));
  }

  @Test
  void spunUpShotFeedsToCompletion() {
    runToCompletion(autoRoutines.buildCompetitionShotCommandForTest(true));

    assertEquals("FEED_COMPLETE", autoRoutines.getCompetitionAutoShotState());
  }

  @Test
  void spinupIncompleteShotSkipsFeedAndFinishes() {
    runToCompletion(autoRoutines.buildCompetitionShotCommandForTest(false));

    assertEquals("SKIPPED_NOT_READY", autoRoutines.getCompetitionAutoShotState());
  }

  @Test
  void readyShotOutsideWindowSkipsFeedAndFinishes() {
    runToCompletion(autoRoutines.buildCompetitionShotCommandForTest(true, false));

    assertEquals("SKIPPED_NO_SHOT_WINDOW", autoRoutines.getCompetitionAutoShotState());
  }

  @Test
  void deployedAutosOnlyReferenceRegisteredNamedCommands() throws IOException {
    Path autosDirectory =
        Path.of(System.getProperty("user.dir"), "src/main/deploy/pathplanner/autos");
    Set<String> allowedNames = new HashSet<>(AutoRoutines.getNamedCommandNames());

    if (!Files.isDirectory(autosDirectory)) {
      return;
    }

    try (var autoFiles = Files.list(autosDirectory)) {
      autoFiles
          .filter(path -> path.getFileName().toString().endsWith(".auto"))
          .forEach(
              path -> {
                String contents;
                try {
                  contents = Files.readString(path);
                } catch (IOException e) {
                  throw new RuntimeException(e);
                }

                Matcher matcher = namedCommandPattern.matcher(contents);
                while (matcher.find()) {
                  assertTrue(
                      allowedNames.contains(matcher.group(1)),
                      () ->
                          "Unsupported named command '"
                              + matcher.group(1)
                              + "' in "
                              + path.getFileName());
                }
              });
    }
  }

  private static AutoRoutines createAutoRoutines() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    Shooter shooter = new Shooter(new ShooterIO() {});
    Intake intake = new Intake(new IntakeIO() {});
    Hopper hopper = new Hopper(new HopperIO() {});
    Indexers indexers = new Indexers(new IndexersIO() {});
    GamePieceCoordinator coordinator = new GamePieceCoordinator(intake, hopper, indexers, shooter);
    HubTargetingService hubTargetingService = new HubTargetingService(drive, shooter);
    FieldTargetingService fieldTargetingService = new FieldTargetingService(drive);
    return new AutoRoutines(
        drive, shooter, indexers, intake, coordinator, hubTargetingService, fieldTargetingService);
  }

  private static void runToCompletion(Command command) {
    command.initialize();
    int maxSteps = 20;
    while (!command.isFinished() && maxSteps-- > 0) {
      command.execute();
    }
    boolean finished = command.isFinished();
    command.end(!finished);
    assertTrue(finished, "Command did not finish within the expected number of steps");
  }
}
