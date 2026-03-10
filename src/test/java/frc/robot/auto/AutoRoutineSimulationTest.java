package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldTargets;
import frc.robot.TargetSelector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.endgame.Endgame;
import frc.robot.subsystems.endgame.EndgameGoal;
import frc.robot.subsystems.endgame.EndgameStatus;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerGoal;
import frc.robot.subsystems.indexer.IndexerStatus;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeGoal;
import frc.robot.subsystems.intake.IntakeStatus;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterBallistics;
import frc.robot.subsystems.shooter.ShooterGoal;
import frc.robot.subsystems.shooter.ShooterStatus;
import frc.robot.subsystems.shooter.ShotSolution;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureGoal;
import frc.robot.subsystems.superstructure.SuperstructureStatus;
import frc.robot.testing.SimScenarioHarness;
import frc.robot.testing.SubsystemFidelity;
import frc.robot.testing.WpilibTestSupport;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

class AutoRoutineSimulationTest {
  private static final double PERIOD_SECONDS = 0.02;

  @BeforeEach
  void setUp() {
    WpilibTestSupport.resetSchedulerAndTime();
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("shippedAutoScenarios")
  void shippedAutoChoicesCompleteInScriptedSimHarness(
      SimScenarioHarness.SimScenario<AutoChoice> scenario) {
    AutoHarness harness = new AutoHarness();
    AutoChoice choice = scenario.value();
    AutoCommand autoCommand = choice.compose(harness.factory);
    AutoSpec spec = autoCommand.spec().orElseThrow();

    autoCommand.command().schedule();
    SimScenarioHarness.runUntilComplete(
        scenario, 400, harness::tick, () -> !autoCommand.command().isScheduled());

    int expectedShots = spec.preloadPolicy() == AutoSpec.PreloadPolicy.SCORE ? 1 : 0;

    assertEquals(
        expectedShots,
        harness.requestedHubShotFireCount,
        scenario.name() + " should request the configured preload shot");
    assertEquals(
        spec.parkOption() != AutoSpec.ParkOption.NONE,
        harness.requestedParkCount > 0,
        scenario.name() + " should only park when the spec calls for it");
    if (choice.label().contains("Outpost Feed")) {
      assertTrue(
          harness.requestedOutpostAlignCount > 0,
          "Outpost Feed autos should enter outpost alignment");
    }
    assertTrue(harness.drive.stopCalls > 0, "Autos should stop the drive during cleanup");
  }

  static Stream<SimScenarioHarness.SimScenario<AutoChoice>> shippedAutoScenarios() {
    return new AutoRoutineFactory(null, null, null)
        .initialAutoChoices().stream()
            .map(
                choice ->
                    new SimScenarioHarness.SimScenario<>(
                        choice.label(), SubsystemFidelity.PLACEHOLDER, choice));
  }

  private static final class AutoHarness {
    private final ScriptedDrive drive = new ScriptedDrive();
    private final FakeIntake intake = new FakeIntake();
    private final FakeIndexer indexer = new FakeIndexer();
    private final FakeShooter shooter = new FakeShooter();
    private final FakeEndgame endgame = new FakeEndgame();
    private final Superstructure superstructure =
        new Superstructure(
            drive,
            intake,
            indexer,
            shooter,
            endgame,
            new ShooterBallistics(),
            new Superstructure.Targeting() {
              @Override
              public TargetSelector.HubSelection getSelectedHub() {
                return TargetSelector.HubSelection.ACTIVE;
              }

              @Override
              public Pose3d getHubPose(TargetSelector.HubSelection selection) {
                return new Pose3d(5.0, 0.0, 1.8, new Rotation3d());
              }

              @Override
              public Pose2d getAllianceOutpostPose() {
                return FieldTargets.OUTPOST.bluePose();
              }
            });
    private final AutoRoutineFactory factory =
        new AutoRoutineFactory(drive, superstructure, new TestNavigator(drive));

    private int requestedAcquireCount = 0;
    private int requestedHubShotFireCount = 0;
    private int requestedParkCount = 0;
    private int requestedOutpostAlignCount = 0;

    private int acquisitionTicks = 0;
    private int shotTicks = 0;
    private java.util.Optional<SuperstructureGoal> previousRequestedGoal =
        java.util.Optional.empty();

    private AutoHarness() {
      indexer.holdingPiece = true;
    }

    private void tick() {
      WpilibTestSupport.stepScheduler(PERIOD_SECONDS);
      stepMechanisms();
    }

    private void stepMechanisms() {
      SuperstructureStatus status = superstructure.getStatus();
      countRequestedGoalTransitions(status);

      if (status.activeGoal() instanceof SuperstructureGoal.IntakeDepot
          || status.activeGoal() instanceof SuperstructureGoal.IntakeFloor) {
        acquisitionTicks++;
        if (acquisitionTicks == 2 && !indexer.holdingPiece) {
          indexer.holdingPiece = true;
        }
      } else {
        acquisitionTicks = 0;
      }

      if (status.activeGoal() instanceof SuperstructureGoal.HubShot) {
        ShotSolution shotSolution = status.shotSolution();
        if (shotSolution != null) {
          drive.setPose(
              new Pose2d(shotSolution.robotPose().getTranslation(), status.targetHeading()));
        }
        if (status.activeGoal() instanceof SuperstructureGoal.HubShot hubShot
            && hubShot.phase() == SuperstructureGoal.HubShotPhase.FIRE) {
          shotTicks++;
          if (shotTicks >= 2 && indexer.holdingPiece) {
            indexer.holdingPiece = false;
          }
        }
      } else {
        shotTicks = 0;
      }

      if (status.activeGoal() instanceof SuperstructureGoal.OutpostAlign) {
        drive.alignTo(status.targetHeading());
      }

      if (status.activeGoal() instanceof SuperstructureGoal.Endgame) {
        drive.alignTo(status.targetHeading());
      }
    }

    private void countRequestedGoalTransitions(SuperstructureStatus status) {
      java.util.Optional<SuperstructureGoal> requestedGoal = status.requestedGoal();
      if (requestedGoal.isEmpty()) {
        previousRequestedGoal = requestedGoal;
        return;
      }

      SuperstructureGoal goal = requestedGoal.orElseThrow();
      boolean isNewGoal =
          previousRequestedGoal.map(previous -> !previous.equals(goal)).orElse(true);
      if (!isNewGoal) {
        return;
      }

      if (goal instanceof SuperstructureGoal.HubShot hubShot
          && hubShot.phase() == SuperstructureGoal.HubShotPhase.FIRE) {
        requestedHubShotFireCount++;
      }
      if (goal instanceof SuperstructureGoal.IntakeDepot
          || goal instanceof SuperstructureGoal.IntakeFloor) {
        requestedAcquireCount++;
      }
      if (goal instanceof SuperstructureGoal.OutpostAlign) {
        requestedOutpostAlignCount++;
      }
      if (goal instanceof SuperstructureGoal.Endgame) {
        requestedParkCount++;
      }
      previousRequestedGoal = requestedGoal;
    }
  }

  private static final class TestNavigator implements AutoNavigator {
    private final ScriptedDrive drive;

    private TestNavigator(ScriptedDrive drive) {
      this.drive = drive;
    }

    @Override
    public Command navigateTo(Pose2d targetPose, AutoRisk risk) {
      return Commands.run(() -> drive.setPose(targetPose), drive);
    }
  }

  private static final class ScriptedDrive extends Drive {
    private Pose2d pose = Pose2d.kZero;
    private int stopCalls = 0;

    private ScriptedDrive() {
      super(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
    }

    @Override
    public void periodic() {}

    @Override
    public Pose2d getPose() {
      return pose;
    }

    @Override
    public Rotation2d getRotation() {
      return pose.getRotation();
    }

    @Override
    public void setPose(Pose2d pose) {
      this.pose = pose;
    }

    public void alignTo(Rotation2d heading) {
      pose = new Pose2d(pose.getTranslation(), heading);
    }

    @Override
    public boolean isAimed(java.util.function.Supplier<Rotation2d> targetHeadingSupplier) {
      return Math.abs(getRotation().minus(targetHeadingSupplier.get()).getRadians())
          <= Rotation2d.fromDegrees(1.0).getRadians();
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
      return new ChassisSpeeds();
    }

    @Override
    public Command holdShotPose(java.util.function.Supplier<ShotSolution> shotSolutionSupplier) {
      return Commands.run(
          () -> {
            ShotSolution shotSolution = shotSolutionSupplier.get();
            if (shotSolution != null) {
              pose = shotSolution.robotPose();
            }
          },
          this);
    }

    @Override
    public void stop() {
      stopCalls++;
    }

    @Override
    public void stopWithX() {
      stopCalls++;
    }
  }

  private static final class FakeIntake extends SubsystemBase implements Intake {
    private IntakeGoal goal = IntakeGoal.STOW;

    @Override
    public void setGoal(IntakeGoal goal) {
      this.goal = goal;
    }

    @Override
    public IntakeGoal getGoal() {
      return goal;
    }

    @Override
    public IntakeStatus getStatus() {
      boolean deployed = goal == IntakeGoal.DEPLOY_DEPOT || goal == IntakeGoal.DEPLOY_FLOOR;
      boolean collecting = goal == IntakeGoal.COLLECT_DEPOT || goal == IntakeGoal.COLLECT_FLOOR;
      boolean reversing = goal == IntakeGoal.REVERSE;
      return new IntakeStatus(goal, true, deployed, collecting, reversing);
    }

    @Override
    public boolean isAtGoal() {
      return true;
    }

    @Override
    public void stop() {
      goal = IntakeGoal.STOW;
    }
  }

  private static final class FakeIndexer extends SubsystemBase implements Indexer {
    private IndexerGoal goal = IndexerGoal.IDLE;
    private boolean holdingPiece = false;

    @Override
    public void setGoal(IndexerGoal goal) {
      this.goal = goal;
    }

    @Override
    public IndexerGoal getGoal() {
      return goal;
    }

    @Override
    public IndexerStatus getStatus() {
      return new IndexerStatus(
          goal, true, holdingPiece, goal == IndexerGoal.FEED_SHOOTER, goal == IndexerGoal.PURGE);
    }

    @Override
    public boolean isAtGoal() {
      return true;
    }

    @Override
    public void stop() {
      goal = IndexerGoal.IDLE;
    }
  }

  private static final class FakeShooter extends SubsystemBase implements Shooter {
    private ShooterGoal goal = new ShooterGoal.Stow();

    @Override
    public void setGoal(ShooterGoal goal) {
      this.goal = goal;
    }

    @Override
    public ShooterGoal getGoal() {
      return goal;
    }

    @Override
    public ShooterStatus getStatus() {
      ShotSolution solution = null;
      boolean ready = false;
      if (goal instanceof ShooterGoal.Track track) {
        solution = track.shotSolution();
      } else if (goal instanceof ShooterGoal.Ready readyGoal) {
        solution = readyGoal.shotSolution();
        ready = true;
      } else if (goal instanceof ShooterGoal.Fire fireGoal) {
        solution = fireGoal.shotSolution();
        ready = true;
      }
      return new ShooterStatus(goal, solution, true, ready);
    }

    @Override
    public boolean isAtGoal() {
      return true;
    }

    @Override
    public void stop() {
      goal = new ShooterGoal.Stow();
    }
  }

  private static final class FakeEndgame extends SubsystemBase implements Endgame {
    private EndgameGoal goal = EndgameGoal.STOWED;

    @Override
    public void setGoal(EndgameGoal goal) {
      this.goal = goal;
    }

    @Override
    public EndgameGoal getGoal() {
      return goal;
    }

    @Override
    public EndgameStatus getStatus() {
      return new EndgameStatus(goal, true, goal != EndgameGoal.STOWED);
    }

    @Override
    public boolean isAtGoal() {
      return true;
    }

    @Override
    public void stop() {
      goal = EndgameGoal.STOWED;
    }
  }
}
