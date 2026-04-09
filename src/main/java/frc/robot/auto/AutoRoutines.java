package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.gamepiece.indexers.Indexers;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.superstructure.gamepiece.GamePieceCoordinator;
import frc.robot.targeting.FieldTargetingService;
import frc.robot.targeting.FieldTargetingService.CompetitionAutoTargets;
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
  private static final double competitionAutoDriverStationRetreatMeters = 1.0;
  private static final double competitionAutoInitialRetreatTimeoutSeconds = 3.0;
  private static final double competitionAutoAimBeforeFeedSeconds = 0.5;
  private static final double competitionAutoPoint5ReadyWaitTimeoutSeconds = 2.0;
  private static final double competitionAutoHubShotFeedSeconds = 3.0;
  private static final double competitionAutoPointDriveTimeoutSeconds = 4.0;
  private static final double competitionAutoPoint5ToPoint2TimeoutSeconds = 6.0;
  private static final double competitionAutoIntakeDriveTimeoutSeconds = 4.5;
  private static final double competitionAutoBumpDriveTimeoutSeconds = 3.0;
  private static final double competitionAutoFastVelocityMetersPerSecond =
      DriveConstants.maxSpeedMetersPerSec * (2.0 / 3.0);
  private static final double competitionAutoIntakeVelocityMetersPerSecond = 1.0;
  private static final PathConstraints competitionAutoFastPathConstraints =
      buildDriveScaledPathConstraints(competitionAutoFastVelocityMetersPerSecond);
  private static final PathConstraints competitionAutoSlowPathConstraints =
      scalePathConstraints(DriveConstants.pathConstraints, 0.60);
  private static final PathConstraints competitionAutoIntakePathConstraints =
      buildDriveScaledPathConstraints(competitionAutoIntakeVelocityMetersPerSecond);
  private static final double competitionAutoIntakeHandoffVelocityMetersPerSecond =
      competitionAutoIntakePathConstraints.maxVelocityMPS();
  private static final double competitionAutoBumpHandoffVelocityMetersPerSecond =
      competitionAutoFastVelocityMetersPerSecond;
  private static final String competitionAutoName = "Competition: Hub Cycle";
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
  private final List<String> autoOptionNames = new ArrayList<>(List.of(competitionAutoName));
  private final Command competitionAutoCommand;
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
  }

  public LoggedDashboardChooser<Command> buildAutoChooser() {
    SendableChooser<Command> baseChooser = new SendableChooser<>();
    addDefaultAutoOption(baseChooser, competitionAutoName, competitionAutoCommand);
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
            .beforeStarting(
                () -> {
                  shooter.setManualHoodOverrideEnabled(false);
                  shooter.setManualWheelOverrideEnabled(false);
                }));
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

  private Command buildCompetitionAutoRoutine() {
    return Commands.defer(
            () -> {
              CompetitionAutoTargets targets =
                  fieldTargetingService.getCompetitionAutoTargets(
                      competitionAutoDriverStationRetreatMeters);

              return Commands.sequence(
                      Commands.runOnce(
                          () -> {
                            drive.setPose(targets.startPose());
                            recordCompetitionAutoShotState("RESET_POSE");
                          },
                          drive),
                      buildInitialHomeAndRetreatCommand(targets.point5Pose()),
                      Commands.repeatingSequence(buildCompetitionCycleCommand(targets)))
                  .finallyDo(
                      () -> {
                        shooter.setShotControlEnabled(false);
                        gamePieceCoordinator.setShooterDemandFromAlign(false);
                        gamePieceCoordinator.stopGamePieceFlow();
                        drive.stop();
                        recordCompetitionAutoShotState("IDLE");
                      });
            },
            Set.of(drive, shooter, intake, indexers))
        .withName(competitionAutoName);
  }

  private Command buildInitialHomeAndRetreatCommand(Pose2d point5Pose) {
    return Commands.parallel(
        drive
            .pathfindToPose(point5Pose, competitionAutoSlowPathConstraints)
            .withTimeout(competitionAutoInitialRetreatTimeoutSeconds),
        shooter.homeHoodToHardStopCommand(),
        intake.calibrateIntakePivotToHardStopsCommand());
  }

  private Command buildCompetitionCycleCommand(CompetitionAutoTargets targets) {
    return Commands.sequence(
        buildStationaryPoint5ShotCommand(),
        buildDriveFromPoint5ThroughTrenchToPoint2Command(targets),
        buildIntakeFromPoint2ToPoint3Command(targets.point2Pose(), targets.point3Pose()),
        Commands.runOnce(() -> recordCompetitionAutoShotState("RETURN_OVER_BUMP")),
        buildReturnOverBumpWhileSpinningUpShooterCommand(targets));
  }

  private Command buildDriveFromPoint5ThroughTrenchToPoint2Command(CompetitionAutoTargets targets) {
    Pose2d point2WithTrenchHeading =
        new Pose2d(
            targets.point2Pose().getTranslation(),
            selectDriverStationRelativeTrenchHeading(
                targets.point5Pose().getTranslation(), targets.point2Pose().getTranslation()));

    return Commands.sequence(
        Commands.runOnce(() -> recordCompetitionAutoShotState("DRIVE_UNDER_TRENCH_TO_POINT2")),
        drive
            .pathfindToPoseWithHeading(
                point2WithTrenchHeading,
                competitionAutoFastPathConstraints,
                competitionAutoIntakeHandoffVelocityMetersPerSecond)
            .withTimeout(competitionAutoPoint5ToPoint2TimeoutSeconds));
  }

  private Command buildReturnOverBumpWhileSpinningUpShooterCommand(CompetitionAutoTargets targets) {
    Command returnOverBump =
        Commands.sequence(
            drive
                .pathfindToPose(
                    targets.point4Pose(),
                    competitionAutoFastPathConstraints,
                    competitionAutoBumpHandoffVelocityMetersPerSecond)
                .withTimeout(competitionAutoPointDriveTimeoutSeconds),
            drive
                .followNamedPath(targets.bumpReturnPathName(), competitionAutoFastPathConstraints)
                .withTimeout(competitionAutoBumpDriveTimeoutSeconds));

    return Commands.deadline(
        returnOverBump, buildShooterDemandFromAlignCommand("SPINNING_UP_FOR_POINT5_SHOT"));
  }

  private Command buildStationaryPoint5ShotCommand() {
    Command feedAfterAim =
        Commands.sequence(
            Commands.waitSeconds(competitionAutoAimBeforeFeedSeconds),
            Commands.waitUntil(shooter::isReadyToFire)
                .withTimeout(competitionAutoPoint5ReadyWaitTimeoutSeconds),
            Commands.run(() -> gamePieceCoordinator.applyBasicFeed(true), intake, indexers)
                .withTimeout(competitionAutoHubShotFeedSeconds));

    return Commands.deadline(
            feedAfterAim,
            drive.alignToHub(() -> 0.0, () -> 0.0, hubTargetingService::updateAndGetAirtimeSeconds),
            buildShooterDemandFromAlignCommand("SHOOTING_AT_POINT5"))
        .finallyDo(
            () -> {
              shooter.setShotControlEnabled(false);
              gamePieceCoordinator.setShooterDemandFromAlign(false);
              gamePieceCoordinator.stopGamePieceFlow();
              drive.stop();
              recordCompetitionAutoShotState("SHOT_PHASE_COMPLETE");
            });
  }

  private Command buildIntakeFromPoint2ToPoint3Command(Pose2d point2Pose, Pose2d point3Pose) {
    Rotation2d intakeHeading =
        getDirectionOfTravelHeading(point2Pose.getTranslation(), point3Pose.getTranslation());
    Pose2d point3WithIntakeHeading = new Pose2d(point3Pose.getTranslation(), intakeHeading);

    return Commands.deadline(
            drive
                .pathfindToPoseWithHeading(
                    point3WithIntakeHeading,
                    competitionAutoIntakePathConstraints,
                    competitionAutoIntakeHandoffVelocityMetersPerSecond)
                .withTimeout(competitionAutoIntakeDriveTimeoutSeconds),
            Commands.run(
                () -> {
                  gamePieceCoordinator.applyBasicCollect(false);
                  recordCompetitionAutoShotState("INTAKING");
                },
                intake,
                indexers))
        .finallyDo(gamePieceCoordinator::stopGamePieceFlow);
  }

  private Command buildShooterDemandFromAlignCommand(String state) {
    return Commands.runEnd(
        () -> {
          hubTargetingService.updateAndGetAirtimeSeconds();
          gamePieceCoordinator.setShooterDemandFromAlign(true);
          shooter.setShotControlEnabled(true, true);
          recordCompetitionAutoShotState(state);
        },
        () -> gamePieceCoordinator.setShooterDemandFromAlign(false));
  }

  private static Rotation2d getDirectionOfTravelHeading(Translation2d start, Translation2d end) {
    Translation2d travel = end.minus(start);
    if (travel.getNorm() < 1e-9) {
      return Rotation2d.kZero;
    }
    return new Rotation2d(Math.atan2(travel.getY(), travel.getX()));
  }

  static Rotation2d selectDriverStationRelativeTrenchHeading(
      Translation2d start, Translation2d end) {
    return end.getX() >= start.getX() ? Rotation2d.kZero : Rotation2d.kPi;
  }

  private static PathConstraints buildDriveScaledPathConstraints(
      double maxVelocityMetersPerSecond) {
    double scale = maxVelocityMetersPerSecond / DriveConstants.maxSpeedMetersPerSec;
    return new PathConstraints(
        maxVelocityMetersPerSecond,
        DriveConstants.maxAccelerationMeterPerSecSquared * scale,
        DriveConstants.maxRotationalSpeedRadiansPerSec * scale,
        DriveConstants.maxRotationalAccelerationRadiansPerSecSquared * scale);
  }

  private static PathConstraints scalePathConstraints(PathConstraints constraints, double scale) {
    return new PathConstraints(
        constraints.maxVelocityMPS() * scale,
        constraints.maxAccelerationMPSSq() * scale,
        constraints.maxAngularVelocityRadPerSec() * scale,
        constraints.maxAngularAccelerationRadPerSecSq() * scale,
        constraints.nominalVoltageVolts(),
        constraints.unlimited());
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
                      shooter.setShotControlEnabled(true, true);
                      recordCompetitionAutoShotState("SPINNING_UP");
                    },
                    shooter))
            .beforeStarting(
                () -> {
                  shooter.setManualHoodOverrideEnabled(false);
                  shooter.setManualWheelOverrideEnabled(false);
                  gamePieceCoordinator.stopGamePieceFlow();
                  recordCompetitionAutoShotState("WAITING_READY");
                });

    Command waitForShotWindow =
        Commands.deadline(
                Commands.waitUntil(inShotWindowSupplier).withTimeout(positionWaitTimeoutSeconds),
                Commands.run(
                    () -> {
                      hubTargetingService.updateAndGetAirtimeSeconds();
                      shooter.setShotControlEnabled(true, true);
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
                      shooter.setShotControlEnabled(true, true);
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
