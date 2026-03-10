package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldTargets;
import frc.robot.RobotAction;
import frc.robot.RobotSettings;
import frc.robot.TargetSelector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureGoal;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;

public class AutoRoutineFactory {
  private static final double PRELOAD_SCORE_TIMEOUT_SECONDS =
      RobotSettings.Auto.preloadScoreTimeoutSeconds;
  private static final double ACQUIRE_TIMEOUT_SECONDS = RobotSettings.Auto.acquireTimeoutSeconds;
  private static final double SCORE_TIMEOUT_SECONDS = RobotSettings.Auto.scoreTimeoutSeconds;
  private static final double EJECT_TIMEOUT_SECONDS = RobotSettings.Auto.ejectTimeoutSeconds;

  private static final AutoSpec DEFAULT_SPEC =
      AutoSpec.of(
          AutoSpec.StartZone.CENTER,
          AutoSpec.PreloadPolicy.SCORE,
          AutoSpec.AcquisitionSource.NEUTRAL_FLOOR,
          1,
          AutoRisk.BALANCED,
          AutoSpec.ParkOption.NEAREST);

  private final Drive drive;
  private final Superstructure superstructure;
  private final AutoNavigator navigator;

  public AutoRoutineFactory(Drive drive, Superstructure superstructure, AutoNavigator navigator) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.navigator = navigator;
  }

  public static AutoSpec defaultSpec() {
    return DEFAULT_SPEC;
  }

  public static AutoSpec resetSpecFor(AutoChoice autoChoice) {
    return autoChoice.spec().orElse(defaultSpec());
  }

  public AutoChoice defaultAutoChoice() {
    return standardChoice(
        "Floor Cycle",
        defaultSpec(),
        RobotAction.AUTO_FACE_AND_SCORE,
        AutoExpectation.SINGLE_CYCLE,
        "neutral floor open, nearest park clear");
  }

  public List<AutoChoice> initialAutoChoices() {
    List<AutoChoice> choices = new ArrayList<>();
    choices.add(
        standardChoice(
            "Preload Safe",
            AutoSpec.of(
                AutoSpec.StartZone.LOWER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoRisk.SAFE,
                AutoSpec.ParkOption.LOWER),
            RobotAction.AUTO_FACE_AND_SCORE,
            AutoExpectation.PRELOAD_ONLY,
            "clean preload shot, lower park clear"));
    choices.add(
        standardChoice(
            "Preload Safe",
            AutoSpec.of(
                AutoSpec.StartZone.CENTER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoRisk.SAFE,
                AutoSpec.ParkOption.NEAREST),
            RobotAction.AUTO_FACE_AND_SCORE,
            AutoExpectation.PRELOAD_ONLY,
            "center shot lane clear, nearest park clear"));
    choices.add(
        standardChoice(
            "Preload Safe",
            AutoSpec.of(
                AutoSpec.StartZone.UPPER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoRisk.SAFE,
                AutoSpec.ParkOption.UPPER),
            RobotAction.AUTO_FACE_AND_SCORE,
            AutoExpectation.PRELOAD_ONLY,
            "clean preload shot, upper park clear"));
    choices.add(defaultAutoChoice());
    choices.add(
        standardChoice(
            "Floor Cycle",
            AutoSpec.of(
                AutoSpec.StartZone.CENTER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NEUTRAL_FLOOR,
                2,
                AutoRisk.AGGRESSIVE,
                AutoSpec.ParkOption.NEAREST),
            RobotAction.AUTO_FACE_AND_SCORE,
            AutoExpectation.DOUBLE_CYCLE,
            "neutral floor open, center lane stays clear"));
    choices.add(
        standardChoice(
            "2-Cycle Depot",
            AutoSpec.of(
                AutoSpec.StartZone.LOWER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.DEPOT,
                2,
                AutoRisk.AGGRESSIVE,
                AutoSpec.ParkOption.LOWER),
            RobotAction.ACQUIRE_DEPOT,
            AutoExpectation.DOUBLE_CYCLE,
            "alliance depot clear, lower lane stays open"));
    choices.add(
        standardChoice(
            "2-Cycle Depot",
            AutoSpec.of(
                AutoSpec.StartZone.UPPER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.DEPOT,
                2,
                AutoRisk.AGGRESSIVE,
                AutoSpec.ParkOption.UPPER),
            RobotAction.ACQUIRE_DEPOT,
            AutoExpectation.DOUBLE_CYCLE,
            "alliance depot clear, upper lane stays open"));
    choices.add(
        standardChoice(
            "Outpost Feed",
            AutoSpec.of(
                AutoSpec.StartZone.LOWER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoRisk.BALANCED,
                AutoSpec.ParkOption.NONE),
            RobotAction.OUTPOST_FEED,
            AutoExpectation.PRELOAD_ONLY,
            "outpost lane clear, partner feed available"));
    choices.add(
        standardChoice(
            "Outpost Feed",
            AutoSpec.of(
                AutoSpec.StartZone.UPPER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoRisk.BALANCED,
                AutoSpec.ParkOption.NONE),
            RobotAction.OUTPOST_FEED,
            AutoExpectation.PRELOAD_ONLY,
            "outpost lane clear, partner feed available"));
    choices.add(
        standardChoice(
            "Park First Fallback",
            AutoSpec.of(
                AutoSpec.StartZone.CENTER,
                AutoSpec.PreloadPolicy.HOLD,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoRisk.SAFE,
                AutoSpec.ParkOption.NEAREST),
            RobotAction.QUICK_PARK,
            AutoExpectation.PARK_FIRST,
            "nearest park reachable immediately"));
    return choices;
  }

  public AutoCommand compose(AutoSpec spec, AutoMetadata metadata) {
    Command command = buildForMetadata(spec, metadata).withName(metadata.labelFor(spec));
    return new AutoCommand(command, Optional.of(spec), Optional.of(metadata));
  }

  public void resetFor(AutoSpec spec) {
    drive.setPose(TargetSelector.getAutoStartPose(spec.autoStartSelection()));
    if (Constants.currentMode == Constants.Mode.SIM) {
      SimulatedArena.getInstance().resetFieldForAuto();
    }
  }

  public Command build(AutoSpec spec) {
    AutoExecutionState state = new AutoExecutionState();

    Command command =
        Commands.sequence(
                Commands.runOnce(state::start),
                Commands.runOnce(() -> resetFor(spec), drive),
                preloadStep(spec, state))
            .withName("AUTO_SEQUENCE");

    for (int cycleIndex = 0;
        cycleIndex < AutoRuntimePolicy.effectiveCycleCount(spec);
        cycleIndex++) {
      command = command.andThen(guardedCycle(spec, state, cycleIndex));
    }

    return command
        .andThen(parkIfNeeded(spec, state))
        .andThen(cleanupCommand())
        .finallyDo(
            interrupted -> {
              superstructure.clearGoal();
              drive.stop();
            })
        .withName(spec.displayName());
  }

  private AutoChoice standardChoice(
      String family,
      AutoSpec spec,
      RobotAction primaryAction,
      AutoExpectation expectation,
      String assumptions) {
    return AutoChoice.forAuto(
        spec, new AutoMetadata(family, primaryAction, expectation, assumptions));
  }

  private Command buildForMetadata(AutoSpec spec, AutoMetadata metadata) {
    return switch (metadata.primaryAction()) {
      case OUTPOST_FEED -> buildOutpostFeedAuto(spec);
      case QUICK_PARK -> buildParkFirstFallback(spec);
      case IDLE, ACQUIRE_DEPOT, ACQUIRE_FLOOR, AUTO_FACE_AND_SCORE, RECOVER -> build(spec);
    };
  }

  private Command guardedCycle(AutoSpec spec, AutoExecutionState state, int cycleIndex) {
    return Commands.defer(
        () -> {
          if (state.shouldAbortRemainingCycles()
              || !AutoRuntimePolicy.shouldStartCycle(spec, cycleIndex, state.elapsedSeconds())) {
            state.abortRemainingCycles();
            return Commands.none();
          }

          return Commands.sequence(acquireStep(spec, state), scoreStep(state));
        },
        Set.of(drive, superstructure));
  }

  private Command preloadStep(AutoSpec spec, AutoExecutionState state) {
    return switch (spec.preloadPolicy()) {
      case HOLD -> Commands.none();
      case SCORE -> runGoalWithDrive(
              new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE),
              superstructure::isAtGoal,
              PRELOAD_SCORE_TIMEOUT_SECONDS,
              drive.holdShotPose(() -> superstructure.getStatus().shotSolution()))
          .andThen(markAbortIf(superstructure::hasPiece, state));
      case EJECT -> runGoalWithDrive(
              new SuperstructureGoal.Eject(SuperstructureGoal.EjectPhase.FIRE),
              () -> !superstructure.hasPiece(),
              EJECT_TIMEOUT_SECONDS,
              holdStoppedDrive())
          .andThen(markAbortIf(superstructure::hasPiece, state));
    };
  }

  private Command acquireStep(AutoSpec spec, AutoExecutionState state) {
    return Commands.defer(
        () -> {
          SuperstructureGoal goal = acquisitionGoal(spec.acquisitionSource());
          if (goal == null) {
            state.abortRemainingCycles();
            return Commands.none();
          }

          Pose2d targetPose = resolveAcquisitionPose(spec.acquisitionSource());
          Command driveCommand =
              navigator.navigateTo(targetPose, spec.risk()).andThen(holdStoppedDrive());

          return runGoalWithDrive(
                  goal, superstructure::isAtGoal, ACQUIRE_TIMEOUT_SECONDS, driveCommand)
              .andThen(markAbortIf(() -> !superstructure.hasPiece(), state));
        },
        Set.of(drive, superstructure));
  }

  private Command scoreStep(AutoExecutionState state) {
    return Commands.defer(
        () -> {
          if (state.shouldAbortRemainingCycles()) {
            return Commands.none();
          }

          return runGoalWithDrive(
                  new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE),
                  superstructure::isAtGoal,
                  SCORE_TIMEOUT_SECONDS,
                  drive.holdShotPose(() -> superstructure.getStatus().shotSolution()))
              .andThen(markAbortIf(superstructure::hasPiece, state));
        },
        Set.of(drive, superstructure));
  }

  private Command parkIfNeeded(AutoSpec spec, AutoExecutionState state) {
    return Commands.defer(
        () -> {
          if (!AutoRuntimePolicy.canParkNow(spec, state.elapsedSeconds())) {
            return Commands.none();
          }

          TargetSelector.ParkZoneSelection parkZoneSelection = resolveParkZoneSelection(spec);
          FieldTargets.FieldZone parkZone = TargetSelector.getParkZone(parkZoneSelection);
          Pose2d parkPose = AutoFieldUtil.computeZoneApproachPose(drive.getPose(), parkZone);
          double timeoutSeconds = AutoRuntimePolicy.parkReservationSeconds(spec.risk());

          return runGoalWithDrive(
              new SuperstructureGoal.Endgame(
                  SuperstructureGoal.EndgamePhase.LEVEL, parkZoneSelection),
              superstructure::isAtGoal,
              timeoutSeconds,
              navigator.navigateTo(parkPose, spec.risk()).andThen(holdStoppedDrive()));
        },
        Set.of(drive, superstructure));
  }

  private Command cleanupCommand() {
    return Commands.parallel(
        Commands.runOnce(superstructure::clearGoal, superstructure),
        Commands.runOnce(drive::stopWithX, drive));
  }

  private Command buildOutpostFeedAuto(AutoSpec spec) {
    AutoExecutionState state = new AutoExecutionState();
    Pose2d outpostPose = TargetSelector.getOutpostPose(TargetSelector.OutpostSelection.ALLIANCE);

    return Commands.sequence(
            Commands.runOnce(state::start),
            Commands.runOnce(() -> resetFor(spec), drive),
            preloadStep(spec, state),
            runGoalWithDrive(
                new SuperstructureGoal.OutpostAlign(),
                superstructure::isAtGoal,
                ACQUIRE_TIMEOUT_SECONDS,
                navigator.navigateTo(outpostPose, spec.risk()).andThen(holdStoppedDrive())),
            cleanupCommand())
        .finallyDo(
            interrupted -> {
              superstructure.clearGoal();
              drive.stop();
            });
  }

  private Command buildParkFirstFallback(AutoSpec spec) {
    return Commands.sequence(
            Commands.runOnce(() -> resetFor(spec), drive),
            Commands.defer(
                () -> {
                  TargetSelector.ParkZoneSelection parkZoneSelection =
                      resolveParkZoneSelection(spec);
                  FieldTargets.FieldZone parkZone = TargetSelector.getParkZone(parkZoneSelection);
                  Pose2d parkPose =
                      AutoFieldUtil.computeZoneApproachPose(drive.getPose(), parkZone);

                  return runGoalWithDrive(
                      new SuperstructureGoal.Endgame(
                          SuperstructureGoal.EndgamePhase.LEVEL, parkZoneSelection),
                      superstructure::isAtGoal,
                      AutoRuntimePolicy.parkReservationSeconds(spec.risk()),
                      navigator.navigateTo(parkPose, spec.risk()).andThen(holdStoppedDrive()));
                },
                Set.of(drive, superstructure)),
            cleanupCommand())
        .finallyDo(
            interrupted -> {
              superstructure.clearGoal();
              drive.stop();
            });
  }

  private Command runGoalWithDrive(
      SuperstructureGoal goal,
      java.util.function.BooleanSupplier completionCondition,
      double timeoutSeconds,
      Command driveCommand) {
    return Commands.sequence(
            Commands.runOnce(() -> superstructure.setGoal(goal), superstructure),
            Commands.deadline(
                Commands.waitUntil(completionCondition).withTimeout(timeoutSeconds), driveCommand))
        .finallyDo(
            interrupted -> {
              superstructure.clearGoal();
              drive.stop();
            });
  }

  private Command holdStoppedDrive() {
    return Commands.run(drive::stop, drive);
  }

  private Command markAbortIf(
      java.util.function.BooleanSupplier condition, AutoExecutionState state) {
    return Commands.runOnce(
        () -> {
          if (condition.getAsBoolean()) {
            state.abortRemainingCycles();
          }
        });
  }

  private SuperstructureGoal acquisitionGoal(AutoSpec.AcquisitionSource acquisitionSource) {
    return switch (acquisitionSource) {
      case DEPOT -> new SuperstructureGoal.IntakeDepot(SuperstructureGoal.IntakePhase.SETTLE);
      case NEUTRAL_FLOOR -> new SuperstructureGoal.IntakeFloor(
          SuperstructureGoal.IntakePhase.SETTLE);
      case NONE -> null;
    };
  }

  private Pose2d resolveAcquisitionPose(AutoSpec.AcquisitionSource acquisitionSource) {
    FieldTargets.FieldZone zone =
        switch (acquisitionSource) {
          case DEPOT -> TargetSelector.getIntakeZone(
              TargetSelector.IntakeZoneSelection.ALLIANCE_DEPOT);
          case NEUTRAL_FLOOR -> TargetSelector.getIntakeZone(
              TargetSelector.IntakeZoneSelection.NEUTRAL_FLOOR);
          case NONE -> null;
        };
    if (zone == null) {
      return drive.getPose();
    }
    return AutoFieldUtil.computeZoneApproachPose(drive.getPose(), zone);
  }

  private TargetSelector.ParkZoneSelection resolveParkZoneSelection(AutoSpec spec) {
    return switch (spec.parkOption()) {
      case NONE -> AutoFieldUtil.selectNearestParkZone(drive.getPose());
      case NEAREST -> AutoFieldUtil.selectNearestParkZone(drive.getPose());
      case LOWER -> TargetSelector.ParkZoneSelection.ALLIANCE_LOWER;
      case UPPER -> TargetSelector.ParkZoneSelection.ALLIANCE_UPPER;
    };
  }

  private static class AutoExecutionState {
    private final Timer timer = new Timer();
    private boolean abortRemainingCycles = false;

    void start() {
      abortRemainingCycles = false;
      timer.restart();
    }

    double elapsedSeconds() {
      return timer.get();
    }

    boolean shouldAbortRemainingCycles() {
      return abortRemainingCycles;
    }

    void abortRemainingCycles() {
      abortRemainingCycles = true;
    }
  }
}
