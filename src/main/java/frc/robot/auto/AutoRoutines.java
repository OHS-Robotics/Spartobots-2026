package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gamepiece.indexers.Indexers;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.superstructure.gamepiece.GamePieceCoordinator;
import frc.robot.targeting.FieldTargetingService;
import frc.robot.targeting.HubTargetingService;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoRoutines {
  private static final double competitionAutoDriveToShotTimeoutSeconds = 4.0;
  private static final double competitionAutoShotSpinUpTimeoutSeconds = 2.5;
  private static final double competitionAutoFeedSeconds = 0.75;
  private static final double competitionAutoLadderAlignTimeoutSeconds = 4.0;
  private static final double competitionAutoShotPositionToleranceMeters = 0.35;
  private static final String competitionAutoName = "Competition: Outpost -> Shoot -> Ladder";
  private static final String doNothingAutoName = "Do Nothing";
  private static final String testAutoName = "Test";
  private static final String autoShotStateLogKey = "Auto/Competition/ShotState";
  private static final List<String> namedCommandNames =
      List.of(
          "collectStart",
          "collectStop",
          "feedStart",
          "feedStop",
          "shooterOn",
          "shooterOff",
          "alignHub",
          "homeHood",
          "calibrateIntakePivot");

  private final Drive drive;
  private final Shooter shooter;
  private final Indexers indexers;
  private final Intake intake;
  private final GamePieceCoordinator gamePieceCoordinator;
  private final HubTargetingService hubTargetingService;
  private final FieldTargetingService fieldTargetingService;
  private final Map<Command, String> autoNamesByCommand = new IdentityHashMap<>();
  private final List<String> autoOptionNames =
      new ArrayList<>(List.of(competitionAutoName, doNothingAutoName, testAutoName));
  private final Command competitionAutoCommand;
  private final Command doNothingAutoCommand = Commands.none().withName(doNothingAutoName);
  private final Command testAutoCommand;
  private String competitionAutoShotState = "IDLE";

  public AutoRoutines(
      Drive drive,
      Shooter shooter,
      Indexers indexers,
      Intake intake,
      GamePieceCoordinator gamePieceCoordinator,
      HubTargetingService hubTargetingService,
      FieldTargetingService fieldTargetingService) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexers = indexers;
    this.intake = intake;
    this.gamePieceCoordinator = gamePieceCoordinator;
    this.hubTargetingService = hubTargetingService;
    this.fieldTargetingService = fieldTargetingService;
    competitionAutoCommand = buildCompetitionAutoRoutine();
    testAutoCommand = buildTestAutoRoutine();
  }

  private Command buildTestAutoRoutine() {
    return Commands.defer(
        () -> {
          Pose2d newPose =
              new Pose2d(drive.getPose().getX() - 1.5, drive.getPose().getY(), drive.getRotation());

          // return drive.followNamedPath("Test");
          // return drive.pathfindToTranslation(newPose);

          return Commands.sequence(
              drive.autoDriveUnderTrenchCommand(0.0),
              drive.autoDriveUnderTrenchCommand(0.0),
              // drive.pathfindToTranslation(newPose.getTranslation()),
              drive
                  .pathfindToTranslationAndAlignToHub(
                      newPose.getTranslation(), hubTargetingService::updateAndGetAirtimeSeconds)
                  .withTimeout(competitionAutoDriveToShotTimeoutSeconds),
              Commands.waitSeconds(2.0),
              Commands.runOnce(
                  () -> {
                    shooter.setShotControlEnabled(true);
                    // shooter.updateHubShotSolution(drive.getPose(), Constants.blueHub);
                    drive.stop();
                  }),
              Commands.waitSeconds(4.0),
              Commands.deadline(
                  Commands.waitSeconds(6.0),
                  Commands.run(
                      () -> {
                        drive
                            .pathfindToTranslationAndAlignToHub(
                                newPose.getTranslation(),
                                hubTargetingService::updateAndGetAirtimeSeconds)
                            .withTimeout(competitionAutoDriveToShotTimeoutSeconds);
                        // gamePieceCoordinator.applyBasicFeed(true);
                        hopper.setTargetAgitatorSpeed(0.85);
                        hopper.updateAgitator();
                        indexers.setTargetIndexerSpeed(0.80);
                        indexers.updateIndexers();
                      })),
              // Commands.waitSeconds(2.0),
              Commands.runOnce(
                  () -> {
                    gamePieceCoordinator.stopGamePieceFlow();
                    shooter.setShotControlEnabled(false);
                  }));
        },
        Set.of(drive, shooter, intake, hopper, indexers));
  }

  public LoggedDashboardChooser<Command> buildAutoChooser() {
    SendableChooser<Command> baseChooser = new SendableChooser<>();
    addDefaultAutoOption(baseChooser, competitionAutoName, competitionAutoCommand);
    addAutoOption(baseChooser, doNothingAutoName, doNothingAutoCommand);
    addAutoOption(baseChooser, testAutoName, testAutoCommand);
    return new LoggedDashboardChooser<>("Auto Choices", baseChooser);
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand(
        "collectStart",
        Commands.runOnce(() -> gamePieceCoordinator.applyBasicCollect(true), intake, indexers));
    NamedCommands.registerCommand(
        "collectStop", Commands.runOnce(gamePieceCoordinator::stopGamePieceFlow, intake, indexers));
    NamedCommands.registerCommand(
        "feedStart",
        Commands.runOnce(() -> gamePieceCoordinator.applyBasicFeed(true), intake, indexers));
    NamedCommands.registerCommand(
        "feedStop", Commands.runOnce(gamePieceCoordinator::stopGamePieceFlow, intake, indexers));
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
    return selectedAuto != null ? selectedAuto : competitionAutoCommand;
  }

  public Command getDefaultAutonomousCommand() {
    return competitionAutoCommand;
  }

  public String getSelectedAutoName(Command selectedAuto) {
    if (selectedAuto == null) {
      return competitionAutoName;
    }
    String mappedName = autoNamesByCommand.get(selectedAuto);
    if (mappedName != null) {
      return mappedName;
    }

    String commandName = selectedAuto.getName();
    return (commandName == null || commandName.isBlank()) ? competitionAutoName : commandName;
  }

  public String[] getAutoOptionNames() {
    return autoOptionNames.toArray(String[]::new);
  }

  String getCompetitionAutoShotState() {
    return competitionAutoShotState;
  }

  static List<String> getNamedCommandNames() {
    return namedCommandNames;
  }

  Command buildCompetitionShotCommandForTest(boolean spinupComplete) {
    return buildCompetitionShotCommandForTest(spinupComplete, true);
  }

  Command buildCompetitionShotCommandForTest(boolean spinupComplete, boolean inShotWindow) {
    return buildCompetitionShooterShotCommand(
        () -> spinupComplete, () -> inShotWindow, 0.0, 0.0, 0.0);
  }

  private void addDefaultAutoOption(
      SendableChooser<Command> chooser, String name, Command command) {
    chooser.setDefaultOption(name, command);
    autoNamesByCommand.put(command, name);
  }

  private void addAutoOption(SendableChooser<Command> chooser, String name, Command command) {
    chooser.addOption(name, command);
    autoNamesByCommand.put(command, name);
  }

  private Command buildCompetitionAutoRoutine() {
    return Commands.defer(
            () -> {
              Pose2d outpostStartPose = fieldTargetingService.getOutpostStartPose();
              Pose2d openingShotPose = fieldTargetingService.getOpeningShotPose();
              Pose2d endingShotPose = fieldTargetingService.getEndingShotPose();

              return Commands.sequence(
                      Commands.runOnce(
                          () -> {
                            drive.setPose(outpostStartPose);
                            recordCompetitionAutoShotState("RESET_POSE");
                          },
                          drive),
                      Commands.runOnce(() -> recordCompetitionAutoShotState("DRIVE_AND_SHOOT")),
                      buildCompetitionDriveAndShootCommand(openingShotPose),
                      // Commands.runOnce(() -> recordCompetitionAutoShotState("DRIVE_TO_LADDER")),
                      // fieldTargetingService
                      // .alignToLadderCommand()
                      // .withTimeout(competitionAutoLadderAlignTimeoutSeconds),
                      Commands.runOnce(() -> recordCompetitionAutoShotState("DRIVE_UNDER_TRENCH")),
                      drive.autoDriveUnderTrenchCommand(2.0),
                      Commands.runOnce(() -> recordCompetitionAutoShotState("MIDDLE_LOAD")),
                      buildCompetitionMiddleLoadCommand(),
                      Commands.defer(
                          () -> {
                            Pose2d shotPose;
                            if (drive.octant == 0) {
                              shotPose = endingShotPose;
                            } else if (drive.octant == 4) {
                              shotPose = openingShotPose;
                            } else {
                              shotPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
                            }
                            return buildCompetitionDriveAndShootCommand(shotPose);
                          },
                          Set.of(drive, shooter)))
                  .finallyDo(
                      () -> {
                        shooter.setShotControlEnabled(false);
                        gamePieceCoordinator.stopGamePieceFlow();
                        recordCompetitionAutoShotState("IDLE");
                      });
            },
            Set.of(drive, shooter, intake, indexers))
        .withName(competitionAutoName);
  }

  private Command buildCompetitionDriveAndShootCommand(Pose2d openingShotPose) {
    return Commands.deadline(
            buildCompetitionShotCommand(
                // Keep the shooter spun up through the auto shot phase even if the current hub
                // solution is infeasible. The feed interlock remains the final gate on the indexer.
                shooter::isSpinupComplete,
                () ->
                    drive.isNearTranslation(
                        openingShotPose.getTranslation(),
                        competitionAutoShotPositionToleranceMeters)),
            drive
                .pathfindToTranslationAndAlignToHub(
                    openingShotPose.getTranslation(),
                    hubTargetingService::updateAndGetAirtimeSeconds)
                .withTimeout(competitionAutoDriveToShotTimeoutSeconds + competitionAutoFeedSeconds))
        .finallyDo(() -> recordCompetitionAutoShotState("SHOT_PHASE_COMPLETE"));
  }

  private Command buildCompetitionMiddleLoadCommand() {
    return Commands.defer(
        () -> {
          String middlePath;
          if (drive.octant == 5 || drive.octant == 2) {
            middlePath = "Middle Right Test";
          } else {
            middlePath = "Middle Left Test";
          }
          return Commands.sequence(
              Commands.runOnce(
                  () -> {
                    // intake.setTargetIntakeSpeed(IntakeConstants.defaultIntakeSpeed);
                  }),
              drive.followNamedPath(middlePath),
              Commands.runOnce(
                  () -> {
                    // intake.stopIntake();
                  }),
              drive.autoDriveUnderTrenchCommand(0.0));
        },
        Set.of(drive, intake));
  }

  private Command buildCompetitionShotCommand(
      BooleanSupplier shooterSpinupCompleteSupplier, BooleanSupplier inShotWindowSupplier) {
    return buildCompetitionShooterShotCommand(
        shooterSpinupCompleteSupplier,
        inShotWindowSupplier,
        competitionAutoShotSpinUpTimeoutSeconds,
        competitionAutoDriveToShotTimeoutSeconds,
        competitionAutoFeedSeconds);
  }

  private Command buildCompetitionShooterShotCommand(
      BooleanSupplier shooterSpinupCompleteSupplier,
      BooleanSupplier inShotWindowSupplier,
      double spinUpTimeoutSeconds,
      double positionWaitTimeoutSeconds,
      double feedSeconds) {
    Command spinUp =
        Commands.deadline(
                Commands.waitUntil(shooterSpinupCompleteSupplier).withTimeout(spinUpTimeoutSeconds),
                Commands.run(
                    () -> {
                      hubTargetingService.updateAndGetAirtimeSeconds();
                      shooter.setShotControlEnabled(true);
                      recordCompetitionAutoShotState("SPINNING_UP");
                    },
                    shooter))
            .beforeStarting(
                () -> {
                  shooter.setManualHoodOverrideEnabled(false);
                  gamePieceCoordinator.stopGamePieceFlow();
                  recordCompetitionAutoShotState("WAITING_READY");
                });

    Command waitForShotWindow =
        Commands.deadline(
                Commands.waitUntil(inShotWindowSupplier).withTimeout(positionWaitTimeoutSeconds),
                Commands.run(
                    () -> {
                      hubTargetingService.updateAndGetAirtimeSeconds();
                      shooter.setShotControlEnabled(true);
                      recordCompetitionAutoShotState("WAITING_SHOT_WINDOW");
                    },
                    shooter))
            .beforeStarting(() -> recordCompetitionAutoShotState("WAITING_SHOT_WINDOW"));

    Command feedWhileAligned =
        Commands.deadline(
                Commands.waitSeconds(feedSeconds),
                Commands.run(
                    () -> {
                      hubTargetingService.updateAndGetAirtimeSeconds();
                      shooter.setShotControlEnabled(true);
                      gamePieceCoordinator.applyBasicFeed(true);
                      recordCompetitionAutoShotState("FEEDING");
                    },
                    shooter,
                    intake,
                    indexers))
            .finallyDo(
                () -> {
                  shooter.setShotControlEnabled(false);
                  gamePieceCoordinator.stopGamePieceFlow();
                  recordCompetitionAutoShotState("FEED_COMPLETE");
                });

    Command skipShotWindow =
        Commands.runOnce(
            () -> {
              shooter.setShotControlEnabled(false);
              gamePieceCoordinator.stopGamePieceFlow();
              recordCompetitionAutoShotState("SKIPPED_NO_SHOT_WINDOW");
            },
            shooter,
            intake,
            indexers);

    Command skipFeed =
        Commands.runOnce(
            () -> {
              shooter.setShotControlEnabled(false);
              gamePieceCoordinator.stopGamePieceFlow();
              recordCompetitionAutoShotState("SKIPPED_NOT_READY");
            },
            shooter,
            intake,
            indexers);

    return Commands.sequence(
            spinUp,
            Commands.either(
                Commands.sequence(
                    waitForShotWindow,
                    Commands.either(feedWhileAligned, skipShotWindow, inShotWindowSupplier)),
                skipFeed,
                shooterSpinupCompleteSupplier))
        .finallyDo(
            () -> {
              shooter.setShotControlEnabled(false);
              gamePieceCoordinator.stopGamePieceFlow();
            });
  }

  private void recordCompetitionAutoShotState(String state) {
    competitionAutoShotState = state;
    Logger.recordOutput(autoShotStateLogKey, state);
  }
}
