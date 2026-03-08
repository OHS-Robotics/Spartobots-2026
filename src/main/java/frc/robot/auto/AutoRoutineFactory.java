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
    return scoredAutoOption("Center Score + 1 Floor Park (Balanced)", defaultSpec());
  }

  public List<AutoOption> presetAutoOptions() {
    List<AutoOption> options = new ArrayList<>();
    options.add(
        scoredAutoOption(
            "Lower Score + 1 Depot Park (Balanced)",
            new AutoSpec(
                AutoSpec.StartZone.LOWER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.DEPOT,
                1,
                AutoSpec.RiskTier.BALANCED,
                AutoSpec.ParkOption.LOWER)));
    options.add(defaultAutoOption());
    options.add(
        scoredAutoOption(
            "Upper Score + 1 Depot Park (Balanced)",
            new AutoSpec(
                AutoSpec.StartZone.UPPER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.DEPOT,
                1,
                AutoSpec.RiskTier.BALANCED,
                AutoSpec.ParkOption.UPPER)));
    options.add(
        scoredAutoOption(
            "Lower Score + 2 Depot Park (Aggressive)",
            new AutoSpec(
                AutoSpec.StartZone.LOWER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.DEPOT,
                2,
                AutoSpec.RiskTier.AGGRESSIVE,
                AutoSpec.ParkOption.LOWER)));
    options.add(
        scoredAutoOption(
            "Center Score + 2 Floor Park (Aggressive)",
            new AutoSpec(
                AutoSpec.StartZone.CENTER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.NEUTRAL_FLOOR,
                2,
                AutoSpec.RiskTier.AGGRESSIVE,
                AutoSpec.ParkOption.NEAREST)));
    options.add(
        scoredAutoOption(
            "Upper Score + 2 Depot Park (Aggressive)",
            new AutoSpec(
                AutoSpec.StartZone.UPPER,
                AutoSpec.PreloadPolicy.SCORE,
                AutoSpec.AcquisitionSource.DEPOT,
                2,
                AutoSpec.RiskTier.AGGRESSIVE,
                AutoSpec.ParkOption.UPPER)));
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

  private AutoOption scoredAutoOption(String name, AutoSpec spec) {
    return AutoOption.forAuto(name, spec, () -> build(spec));
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
