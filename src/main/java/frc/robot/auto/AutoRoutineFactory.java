package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldTargets;
import frc.robot.TargetSelector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureGoal;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;

public class AutoRoutineFactory {
  private static final double PRELOAD_SCORE_TIMEOUT_SECONDS = 2.5;
  private static final double ACQUIRE_TIMEOUT_SECONDS = 4.0;
  private static final double SCORE_TIMEOUT_SECONDS = 2.5;
  private static final double EJECT_TIMEOUT_SECONDS = 1.0;

  private static final AutoSpec DEFAULT_SPEC =
      new AutoSpec(
          AutoSpec.StartZone.CENTER,
          AutoSpec.PreloadPolicy.SCORE,
          AutoSpec.AcquisitionSource.NEUTRAL_FLOOR,
          1,
          AutoSpec.RiskTier.BALANCED,
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

  public static AutoSpec resetSpecFor(AutoOption autoOption) {
    return autoOption.spec().orElse(defaultSpec());
  }

  public AutoOption defaultAutoOption() {
    return floorCycleOption(defaultSpec(), "8-12", "neutral floor open, nearest park clear");
  }

  public List<AutoOption> initialAutoOptions() {
    List<AutoOption> options = new ArrayList<>();
    options.add(
        preloadSafeOption(
            new AutoSpec(
                AutoSpec.StartZone.LOWER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoSpec.RiskTier.SAFE,
                AutoSpec.ParkOption.LOWER),
            "4-8",
            "clean preload shot, lower park clear"));
    options.add(
        preloadSafeOption(
            new AutoSpec(
                AutoSpec.StartZone.CENTER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoSpec.RiskTier.SAFE,
                AutoSpec.ParkOption.NEAREST),
            "4-8",
            "center shot lane clear, nearest park clear"));
    options.add(
        preloadSafeOption(
            new AutoSpec(
                AutoSpec.StartZone.UPPER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoSpec.RiskTier.SAFE,
                AutoSpec.ParkOption.UPPER),
            "4-8",
            "clean preload shot, upper park clear"));
    options.add(defaultAutoOption());
    options.add(
        floorCycleOption(
            new AutoSpec(
                AutoSpec.StartZone.CENTER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NEUTRAL_FLOOR,
                2,
                AutoSpec.RiskTier.AGGRESSIVE,
                AutoSpec.ParkOption.NEAREST),
            "10-14",
            "neutral floor open, center lane stays clear"));
    options.add(
        aggressiveDepotOption(
            new AutoSpec(
                AutoSpec.StartZone.LOWER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.DEPOT,
                2,
                AutoSpec.RiskTier.AGGRESSIVE,
                AutoSpec.ParkOption.LOWER),
            "10-14",
            "alliance depot clear, lower lane stays open"));
    options.add(
        aggressiveDepotOption(
            new AutoSpec(
                AutoSpec.StartZone.UPPER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.DEPOT,
                2,
                AutoSpec.RiskTier.AGGRESSIVE,
                AutoSpec.ParkOption.UPPER),
            "10-14",
            "alliance depot clear, upper lane stays open"));
    options.add(
        outpostFeedOption(
            new AutoSpec(
                AutoSpec.StartZone.LOWER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoSpec.RiskTier.BALANCED,
                AutoSpec.ParkOption.NONE),
            "4-8",
            "outpost lane clear, partner feed available"));
    options.add(
        outpostFeedOption(
            new AutoSpec(
                AutoSpec.StartZone.UPPER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoSpec.RiskTier.BALANCED,
                AutoSpec.ParkOption.NONE),
            "4-8",
            "outpost lane clear, partner feed available"));
    options.add(
        parkFirstFallbackOption(
            new AutoSpec(
                AutoSpec.StartZone.CENTER,
                AutoSpec.PreloadPolicy.HOLD,
                AutoSpec.AcquisitionSource.NONE,
                0,
                AutoSpec.RiskTier.SAFE,
                AutoSpec.ParkOption.NEAREST),
            "0-4",
            "nearest park reachable immediately"));
    return options;
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

  private AutoOption preloadSafeOption(AutoSpec spec, String pointsBand, String assumptions) {
    return describedAutoOption("Preload Safe", spec, pointsBand, assumptions, () -> build(spec));
  }

  private AutoOption floorCycleOption(AutoSpec spec, String pointsBand, String assumptions) {
    return describedAutoOption("Floor Cycle", spec, pointsBand, assumptions, () -> build(spec));
  }

  private AutoOption aggressiveDepotOption(AutoSpec spec, String pointsBand, String assumptions) {
    return describedAutoOption("2-Cycle Depot", spec, pointsBand, assumptions, () -> build(spec));
  }

  private AutoOption outpostFeedOption(AutoSpec spec, String pointsBand, String assumptions) {
    return describedAutoOption(
        "Outpost Feed", spec, pointsBand, assumptions, () -> buildOutpostFeedAuto(spec));
  }

  private AutoOption parkFirstFallbackOption(AutoSpec spec, String pointsBand, String assumptions) {
    return describedAutoOption(
        "Park First Fallback", spec, pointsBand, assumptions, () -> buildParkFirstFallback(spec));
  }

  private AutoOption describedAutoOption(
      String family,
      AutoSpec spec,
      String pointsBand,
      String assumptions,
      java.util.function.Supplier<Command> commandFactory) {
    String label =
        String.format(
            "%s | Start %s | Band %s | Risk %s | Assume %s",
            family, spec.startZone().label(), pointsBand, spec.riskTier().label(), assumptions);
    return AutoOption.forAuto(label, spec, commandFactory);
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
              superstructure::atGoal,
              PRELOAD_SCORE_TIMEOUT_SECONDS,
              drive.holdShotPose(() -> superstructure.getStatus().shotSolution()))
          .andThen(markAbortIf(() -> superstructure.hasGamePiece(), state));
      case EJECT -> runGoalWithDrive(
              new SuperstructureGoal.Eject(SuperstructureGoal.EjectPhase.FIRE),
              () -> !superstructure.hasGamePiece(),
              EJECT_TIMEOUT_SECONDS,
              holdStoppedDrive())
          .andThen(markAbortIf(superstructure::hasGamePiece, state));
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
              navigator.navigateTo(targetPose, spec.riskTier()).andThen(holdStoppedDrive());

          return runGoalWithDrive(
                  goal, superstructure::atGoal, ACQUIRE_TIMEOUT_SECONDS, driveCommand)
              .andThen(markAbortIf(() -> !superstructure.hasGamePiece(), state));
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
                  superstructure::atGoal,
                  SCORE_TIMEOUT_SECONDS,
                  drive.holdShotPose(() -> superstructure.getStatus().shotSolution()))
              .andThen(markAbortIf(superstructure::hasGamePiece, state));
        },
        Set.of(drive, superstructure));
  }

  private Command parkIfNeeded(AutoSpec spec, AutoExecutionState state) {
    return Commands.defer(
        () -> {
          if (!AutoRuntimePolicy.shouldAttemptPark(spec, state.elapsedSeconds())) {
            return Commands.none();
          }

          TargetSelector.ParkZoneSelection parkZoneSelection = resolveParkZoneSelection(spec);
          FieldTargets.FieldZone parkZone = TargetSelector.getParkZone(parkZoneSelection);
          Pose2d parkPose = AutoFieldUtil.computeZoneApproachPose(drive.getPose(), parkZone);
          double timeoutSeconds = AutoRuntimePolicy.parkReservationSeconds(spec.riskTier());

          return runGoalWithDrive(
              new SuperstructureGoal.Endgame(
                  SuperstructureGoal.EndgamePhase.LEVEL, parkZoneSelection),
              superstructure::atGoal,
              timeoutSeconds,
              navigator.navigateTo(parkPose, spec.riskTier()).andThen(holdStoppedDrive()));
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
                superstructure::atGoal,
                ACQUIRE_TIMEOUT_SECONDS,
                navigator.navigateTo(outpostPose, spec.riskTier()).andThen(holdStoppedDrive())),
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
                      superstructure::atGoal,
                      AutoRuntimePolicy.parkReservationSeconds(spec.riskTier()),
                      navigator.navigateTo(parkPose, spec.riskTier()).andThen(holdStoppedDrive()));
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
