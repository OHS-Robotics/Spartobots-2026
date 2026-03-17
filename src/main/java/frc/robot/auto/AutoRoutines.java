package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gamepiece.hopper.Hopper;
import frc.robot.subsystems.gamepiece.indexers.Indexers;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.superstructure.gamepiece.GamePieceCoordinator;
import frc.robot.targeting.FieldTargetingService;
import frc.robot.targeting.HubTargetingService;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoRoutines {
  private static final double startAutoOpeningShotSeconds = 1.75;
  private static final double startAutoTrenchTimeoutSeconds = 4.0;
  private static final double startAutoOutpostTimeoutSeconds = 3.0;
  private static final double startAutoOutpostToShootTimeoutSeconds = 4.0;
  private static final double startAutoOutpostShootSeconds = 4.0;
  private static final double startAutoLadderAlignTimeoutSeconds = 4.0;
  private static final String defaultAutoName = "Start Match (Outpost -> Shoot -> Ladder)";

  private final Drive drive;
  private final Shooter shooter;
  private final Hopper hopper;
  private final Indexers indexers;
  private final Intake intake;
  private final GamePieceCoordinator gamePieceCoordinator;
  private final HubTargetingService hubTargetingService;
  private final FieldTargetingService fieldTargetingService;
  private final Map<Command, String> autoNamesByCommand = new IdentityHashMap<>();
  private final List<String> autoOptionNames = new ArrayList<>();

  public AutoRoutines(
      Drive drive,
      Shooter shooter,
      Hopper hopper,
      Indexers indexers,
      Intake intake,
      GamePieceCoordinator gamePieceCoordinator,
      HubTargetingService hubTargetingService,
      FieldTargetingService fieldTargetingService) {
    this.drive = drive;
    this.shooter = shooter;
    this.hopper = hopper;
    this.indexers = indexers;
    this.intake = intake;
    this.gamePieceCoordinator = gamePieceCoordinator;
    this.hubTargetingService = hubTargetingService;
    this.fieldTargetingService = fieldTargetingService;
    autoOptionNames.addAll(discoverPathPlannerAutoNames());
  }

  public LoggedDashboardChooser<Command> buildAutoChooser() {
    LoggedDashboardChooser<Command> chooser =
        new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    addAutoOption(chooser, defaultAutoName, outpostStartShootAndLadderAutoRoutine());
    addAutoOption(chooser, "Start Match (Hub + Trench + Outpost)", startOfMatchAutoRoutine());
    addAutoOption(chooser, "Hub Opening Shot (Basic Feed)", openingHubShotAutoRoutine());
    addAutoOption(chooser, "Trench Collect (Timed)", trenchCollectAutoRoutine());
    addAutoOption(chooser, "Trench Collect (Timed) new thing", trenchCollectAutoRoutineNew());
    addAutoOption(chooser, "Outpost Collect (Timed)", outpostCollectAutoRoutine());
    addAutoOption(
        chooser,
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drive));
    addAutoOption(
        chooser,
        "Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(drive));
    addAutoOption(
        chooser,
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    addAutoOption(
        chooser,
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    addAutoOption(
        chooser,
        "Drive SysId (Dynamic Forward)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    addAutoOption(
        chooser,
        "Drive SysId (Dynamic Reverse)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    return chooser;
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand(
        "collectStart",
        Commands.runOnce(
            () -> gamePieceCoordinator.applyBasicCollect(true), intake, hopper, indexers));
    NamedCommands.registerCommand(
        "collectStop",
        Commands.runOnce(gamePieceCoordinator::stopGamePieceFlow, intake, hopper, indexers));
    NamedCommands.registerCommand(
        "feedStart",
        Commands.runOnce(
            () -> gamePieceCoordinator.applyBasicFeed(true), intake, hopper, indexers));
    NamedCommands.registerCommand(
        "feedStop",
        Commands.runOnce(gamePieceCoordinator::stopGamePieceFlow, intake, hopper, indexers));
    NamedCommands.registerCommand(
        "shooterOn",
        Commands.runOnce(() -> shooter.setShotControlEnabled(true))
            .beforeStarting(() -> shooter.setManualHoodOverrideEnabled(false)));
    NamedCommands.registerCommand(
        "shooterOff",
        Commands.runOnce(
            () -> {
              shooter.setShotControlEnabled(false);
              gamePieceCoordinator.stopGamePieceFlow();
            }));
    NamedCommands.registerCommand(
        "alignHub",
        drive.alignToHub(() -> 0.0, () -> 0.0, hubTargetingService::updateAndGetAirtimeSeconds));
    NamedCommands.registerCommand("homeHood", shooter.homeHoodToHardStopCommand());
    NamedCommands.registerCommand(
        "calibrateIntakePivot", intake.calibrateIntakePivotToHardStopsCommand());
  }

  public Command selectAutonomousCommand(LoggedDashboardChooser<Command> autoChooser) {
    Command selectedAuto = autoChooser.get();
    return selectedAuto != null ? selectedAuto : outpostStartShootAndLadderAutoRoutine();
  }

  public Command getDefaultAutonomousCommand() {
    return outpostStartShootAndLadderAutoRoutine();
  }

  public String getSelectedAutoName(Command selectedAuto) {
    if (selectedAuto == null) {
      return defaultAutoName;
    }
    String mappedName = autoNamesByCommand.get(selectedAuto);
    if (mappedName != null) {
      return mappedName;
    }

    String commandName = selectedAuto.getName();
    return (commandName == null || commandName.isBlank()) ? defaultAutoName : commandName;
  }

  public String[] getAutoOptionNames() {
    return autoOptionNames.toArray(String[]::new);
  }

  private void addAutoOption(
      LoggedDashboardChooser<Command> chooser, String name, Command command) {
    chooser.addOption(name, command);
    autoNamesByCommand.put(command, name);
    if (!autoOptionNames.contains(name)) {
      autoOptionNames.add(name);
    }
  }

  private Command startOfMatchAutoRoutine() {
    return Commands.sequence(
            openingHubShotAutoRoutine(), trenchCollectAutoRoutine(), outpostCollectAutoRoutine())
        .finallyDo(
            () -> {
              shooter.setShotControlEnabled(false);
              gamePieceCoordinator.stopGamePieceFlow();
            });
  }

  private Command openingHubShotAutoRoutine() {
    return timedHubShotAutoRoutine(startAutoOpeningShotSeconds);
  }

  private Command outpostStartShootAndLadderAutoRoutine() {
    Pose2d outpostStartPose = fieldTargetingService.getOutpostStartPose();
    Pose2d openingShotPose = fieldTargetingService.getOpeningShotPose();

    return Commands.sequence(
            Commands.runOnce(() -> drive.setPose(outpostStartPose), drive),
            drive.alignToPose(openingShotPose).withTimeout(startAutoOutpostToShootTimeoutSeconds),
            timedHubShotAutoRoutine(startAutoOutpostShootSeconds),
            fieldTargetingService
                .alignToLadderCommand()
                .withTimeout(startAutoLadderAlignTimeoutSeconds))
        .finallyDo(
            () -> {
              shooter.setShotControlEnabled(false);
              gamePieceCoordinator.stopGamePieceFlow();
            });
  }

  private Command timedHubShotAutoRoutine(double shotDurationSeconds) {
    return Commands.deadline(
        Commands.waitSeconds(shotDurationSeconds),
        drive.alignToHub(() -> 0.0, () -> 0.0, hubTargetingService::updateAndGetAirtimeSeconds),
        Commands.run(
                () -> {
                  hubTargetingService.updateAndGetAirtimeSeconds();
                  shooter.setShotControlEnabled(true);
                  gamePieceCoordinator.applyBasicFeed(true);
                },
                shooter,
                intake,
                hopper,
                indexers)
            .beforeStarting(() -> shooter.setManualHoodOverrideEnabled(false))
            .finallyDo(
                () -> {
                  shooter.setShotControlEnabled(false);
                  gamePieceCoordinator.stopGamePieceFlow();
                }));
  }

  private Command trenchCollectAutoRoutineNew() {
    return Commands.sequence(
        fieldTargetingService.autoDriveUnderTrenchCommand(),
        Commands.runOnce(
            () -> {
              intake.setTargetIntakeSpeed(0.65);
              intake.updateIntake();
            },
            intake),
        drive.autoLoadMiddleCommand(),
        Commands.runOnce(intake::stopIntake, intake),
        fieldTargetingService.autoDriveUnderTrenchCommand());
  }

  private Command trenchCollectAutoRoutine() {
    return Commands.deadline(
        fieldTargetingService
            .autoDriveUnderTrenchCommand()
            .withTimeout(startAutoTrenchTimeoutSeconds),
        gamePieceCoordinator.basicCollectWhileHeldCommand(true));
  }

  private Command outpostCollectAutoRoutine() {
    return Commands.deadline(
        fieldTargetingService.driveToOutpostCommand().withTimeout(startAutoOutpostTimeoutSeconds),
        gamePieceCoordinator.basicCollectWhileHeldCommand(true));
  }

  private static List<String> discoverPathPlannerAutoNames() {
    Path autosDirectory = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/autos");
    if (!Files.isDirectory(autosDirectory)) {
      return List.of();
    }

    try (Stream<Path> autoFiles = Files.list(autosDirectory)) {
      return autoFiles
          .filter(path -> path.getFileName().toString().endsWith(".auto"))
          .map(path -> path.getFileName().toString().replaceFirst("\\.auto$", ""))
          .sorted(Comparator.naturalOrder())
          .toList();
    } catch (IOException ignored) {
      return List.of();
    }
  }
}
