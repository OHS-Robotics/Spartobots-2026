package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
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
  private static final int competitionAutoCycleCount = 2;
  private static final double competitionAutoHubShotFeedSeconds = 3.0;
  private static final double competitionAutoPointDriveSafetyTimeoutSeconds = 4.0;
  private static final double competitionAutoPoint5ToPoint2SafetyTimeoutSeconds = 6.0;
  private static final double competitionAutoIntakeDriveSafetyTimeoutSeconds = 4.5;
  private static final double competitionAutoBumpDriveSafetyTimeoutSeconds = 4.5;
  private static final double competitionAutoMilestoneTranslationToleranceMeters = 0.20;
  private static final double competitionAutoMilestoneRotationToleranceRadians =
      Math.toRadians(8.0);
  private static final double competitionAutoMovingShotHubAlignmentToleranceRadians =
      Math.toRadians(5.0);
  private static final double competitionAutoFastVelocityMetersPerSecond =
      DriveConstants.maxSpeedMetersPerSec;
  private static final double competitionAutoIntakeVelocityMetersPerSecond = 1.0;
  private static final PathConstraints competitionAutoFastPathConstraints =
      buildDriveScaledPathConstraints(competitionAutoFastVelocityMetersPerSecond);
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

  static int getCompetitionAutoCycleCount() {
    return competitionAutoCycleCount;
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
              List<CompetitionAutoTargets> targetsByCycle = new ArrayList<>();
              for (int cycleIndex = 0; cycleIndex < competitionAutoCycleCount; cycleIndex++) {
                targetsByCycle.add(fieldTargetingService.getCompetitionAutoTargets(cycleIndex));
              }
              CompetitionAutoTargets firstCycleTargets = targetsByCycle.get(0);
              List<Command> autoSteps = new ArrayList<>();
              autoSteps.add(
                  Commands.runOnce(
                      () -> {
                        drive.setPose(firstCycleTargets.startPose());
                        recordCompetitionAutoShotState("RESET_POSE");
                      },
                      drive));
              autoSteps.add(buildOpeningIntakeCycleCommand(firstCycleTargets));
              for (int cycleIndex = 1; cycleIndex < targetsByCycle.size(); cycleIndex++) {
                autoSteps.add(buildCompetitionCycleCommand(targetsByCycle.get(cycleIndex)));
              }
              autoSteps.add(
                  buildShootOnMoveToTrenchEntranceCommand(
                      targetsByCycle.get(targetsByCycle.size() - 1)));

              return Commands.sequence(autoSteps.toArray(Command[]::new))
                  .finallyDo(
                      () -> {
                        shooter.setShotControlEnabled(false);
                        shooter.cancelHoodHomingToHardStop();
                        gamePieceCoordinator.setShooterDemandFromAlign(false);
                        gamePieceCoordinator.stopGamePieceFlow();
                        drive.stop();
                        recordCompetitionAutoShotState("IDLE");
                      });
            },
            Set.of(drive, shooter, intake, indexers))
        .withName(competitionAutoName);
  }

  private Command buildOpeningIntakeCycleCommand(CompetitionAutoTargets targets) {
    return Commands.sequence(
        Commands.parallel(
            buildDriveThroughTrenchToPoint2Command(
                targets.point2Pose(), "OPENING_HOME_INTAKE_AND_CROSS_TRENCH"),
            intake.calibrateIntakePivotToHardStopsCommand()),
        shooter.startHoodHomingToHardStopCommand(),
        buildIntakeFromPoint2ToPoint3Command(targets.point2Pose(), targets.point3Pose()),
        Commands.runOnce(() -> recordCompetitionAutoShotState("RETURN_OVER_BUMP")),
        buildReturnOverBumpWhileSpinningUpShooterCommand(targets));
  }

  private Command buildCompetitionCycleCommand(CompetitionAutoTargets targets) {
    return Commands.sequence(
        buildShootOnMoveToTrenchEntranceCommand(targets),
        buildDriveThroughTrenchToPoint2Command(
            targets.point2Pose(), "DRIVE_UNDER_TRENCH_TO_POINT2"),
        buildIntakeFromPoint2ToPoint3Command(targets.point2Pose(), targets.point3Pose()),
        Commands.runOnce(() -> recordCompetitionAutoShotState("RETURN_OVER_BUMP")),
        buildReturnOverBumpWhileSpinningUpShooterCommand(targets));
  }

  private Command buildShootOnMoveToTrenchEntranceCommand(CompetitionAutoTargets targets) {
    return Commands.defer(
        () -> {
          PathConstraints shotOnMoveConstraints =
              buildShotOnMovePathConstraints(
                  drive.getPose().getTranslation(), targets.point1Pose().getTranslation());
          Command driveToTrenchEntrance =
              withTranslationMilestone(
                  drive.pathfindToTranslationWithHubAim(
                      targets.point1Pose().getTranslation(),
                      hubTargetingService::updateAndGetAirtimeSeconds,
                      shotOnMoveConstraints,
                      shotOnMoveConstraints.maxVelocityMPS()),
                  targets.point1Pose().getTranslation(),
                  competitionAutoPointDriveSafetyTimeoutSeconds);

          return Commands.deadline(
                  driveToTrenchEntrance,
                  buildShooterDemandFromAlignCommand("SHOOTING_ON_MOVE_TO_TRENCH"),
                  Commands.run(
                      () -> {
                        double shotAirtimeSeconds =
                            hubTargetingService.updateAndGetAirtimeSeconds();
                        boolean aimedAtHub =
                            drive.isAimedAtHub(
                                shotAirtimeSeconds,
                                competitionAutoMovingShotHubAlignmentToleranceRadians);
                        gamePieceCoordinator.applyBasicFeedWhenShotWindowAvailable(aimedAtHub);
                      },
                      intake,
                      indexers))
              .finallyDo(
                  () -> {
                    shooter.setShotControlEnabled(false);
                    gamePieceCoordinator.setShooterDemandFromAlign(false);
                    gamePieceCoordinator.stopGamePieceFlow();
                    recordCompetitionAutoShotState("MOVING_SHOT_COMPLETE");
                  });
        },
        Set.of(drive, shooter, intake, indexers));
  }

  private Command buildDriveThroughTrenchToPoint2Command(Pose2d point2Pose, String state) {
    return Commands.defer(
        () -> {
          Rotation2d trenchHeading =
              selectClosestDriverStationRelativeHeading(drive.getPose(), point2Pose);
          Pose2d point2WithTrenchHeading = new Pose2d(point2Pose.getTranslation(), trenchHeading);

          return Commands.sequence(
              Commands.runOnce(() -> recordCompetitionAutoShotState(state)),
              withPoseMilestone(
                  drive.pathfindToPoseWithHeading(
                      point2WithTrenchHeading,
                      competitionAutoFastPathConstraints,
                      competitionAutoIntakeHandoffVelocityMetersPerSecond),
                  point2WithTrenchHeading,
                  competitionAutoPoint5ToPoint2SafetyTimeoutSeconds));
        },
        Set.of(drive));
  }

  private Command buildReturnOverBumpWhileSpinningUpShooterCommand(CompetitionAutoTargets targets) {
    Command returnOverBump =
        Commands.defer(
            () -> {
              Rotation2d bumpHeading =
                  selectClosestDriverStationRelativeHeading(drive.getPose(), targets.point5Pose());
              Pose2d point4WithBumpHeading =
                  new Pose2d(targets.point4Pose().getTranslation(), bumpHeading);
              Translation2d point5Translation = targets.point5Pose().getTranslation();

              return Commands.sequence(
                  withPoseMilestone(
                      drive.pathfindToPoseWithHeading(
                          point4WithBumpHeading,
                          competitionAutoFastPathConstraints,
                          competitionAutoBumpHandoffVelocityMetersPerSecond),
                      point4WithBumpHeading,
                      competitionAutoPointDriveSafetyTimeoutSeconds),
                  withTranslationMilestone(
                      drive.followNamedPathWithHeading(
                          targets.bumpReturnPathName(),
                          bumpHeading,
                          competitionAutoFastPathConstraints),
                      point5Translation,
                      competitionAutoBumpDriveSafetyTimeoutSeconds));
            },
            Set.of(drive));

    return Commands.deadline(
        returnOverBump, buildShooterDemandFromAlignCommand("SPINNING_UP_FOR_POINT5_SHOT"));
  }

  private Command buildIntakeFromPoint2ToPoint3Command(Pose2d point2Pose, Pose2d point3Pose) {
    Rotation2d intakeHeading =
        getDirectionOfTravelHeading(point2Pose.getTranslation(), point3Pose.getTranslation());
    Pose2d point3WithIntakeHeading = new Pose2d(point3Pose.getTranslation(), intakeHeading);

    return Commands.deadline(
            withPoseMilestone(
                drive.pathfindToPoseWithHeading(
                    point3WithIntakeHeading,
                    competitionAutoIntakePathConstraints,
                    competitionAutoIntakeHandoffVelocityMetersPerSecond),
                point3WithIntakeHeading,
                competitionAutoIntakeDriveSafetyTimeoutSeconds),
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

  private Command withPoseMilestone(
      Command driveCommand, Pose2d targetPose, double safetyTimeoutSeconds) {
    return driveCommand
        .until(() -> isAtCompetitionAutoMilestone(targetPose))
        .withTimeout(safetyTimeoutSeconds);
  }

  private Command withTranslationMilestone(
      Command driveCommand, Translation2d targetTranslation, double safetyTimeoutSeconds) {
    return driveCommand
        .until(() -> isAtCompetitionAutoTranslationMilestone(targetTranslation))
        .withTimeout(safetyTimeoutSeconds);
  }

  private boolean isAtCompetitionAutoMilestone(Pose2d targetPose) {
    Pose2d currentPose = drive.getPose();
    double translationErrorMeters =
        currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double rotationErrorRadians =
        Math.abs(
            MathUtil.angleModulus(
                currentPose.getRotation().minus(targetPose.getRotation()).getRadians()));
    return translationErrorMeters <= competitionAutoMilestoneTranslationToleranceMeters
        && rotationErrorRadians <= competitionAutoMilestoneRotationToleranceRadians;
  }

  private boolean isAtCompetitionAutoTranslationMilestone(Translation2d targetTranslation) {
    return drive.getPose().getTranslation().getDistance(targetTranslation)
        <= competitionAutoMilestoneTranslationToleranceMeters;
  }

  private static Rotation2d getDirectionOfTravelHeading(Translation2d start, Translation2d end) {
    Translation2d travel = end.minus(start);
    if (travel.getNorm() < 1e-9) {
      return Rotation2d.kZero;
    }
    return new Rotation2d(Math.atan2(travel.getY(), travel.getX()));
  }

  static Rotation2d selectClosestDriverStationRelativeHeading(
      Pose2d currentPose, Pose2d targetPose) {
    Rotation2d travelPreferredHeading =
        targetPose.getX() >= currentPose.getX() ? Rotation2d.kZero : Rotation2d.kPi;
    return selectClosestDriverStationRelativeHeading(
        currentPose.getRotation(), travelPreferredHeading);
  }

  static Rotation2d selectClosestDriverStationRelativeHeading(
      Rotation2d currentHeading, Rotation2d tieBreakHeading) {
    double zeroHeadingError =
        Math.abs(MathUtil.angleModulus(currentHeading.minus(Rotation2d.kZero).getRadians()));
    double piHeadingError =
        Math.abs(MathUtil.angleModulus(currentHeading.minus(Rotation2d.kPi).getRadians()));
    if (Math.abs(zeroHeadingError - piHeadingError) <= 1e-9) {
      return Math.abs(MathUtil.angleModulus(tieBreakHeading.getRadians())) <= Math.PI / 2.0
          ? Rotation2d.kZero
          : Rotation2d.kPi;
    }
    return zeroHeadingError < piHeadingError ? Rotation2d.kZero : Rotation2d.kPi;
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

  private static PathConstraints buildShotOnMovePathConstraints(
      Translation2d start, Translation2d end) {
    double maxVelocityMetersPerSecond =
        MathUtil.clamp(
            start.getDistance(end) / competitionAutoHubShotFeedSeconds,
            0.25,
            competitionAutoFastVelocityMetersPerSecond);
    return new PathConstraints(
        maxVelocityMetersPerSecond,
        DriveConstants.maxAccelerationMeterPerSecSquared,
        DriveConstants.maxRotationalSpeedRadiansPerSec,
        DriveConstants.maxRotationalAccelerationRadiansPerSecSquared);
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
