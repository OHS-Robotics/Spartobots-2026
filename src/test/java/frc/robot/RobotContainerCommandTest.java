package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureGoal;
import frc.robot.testing.WpilibTestSupport;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotContainerCommandTest {
  @BeforeEach
  void setUp() {
    WpilibTestSupport.resetSchedulerAndTime();
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void cancelRecoverIntentClearsGoalAndStopsDrive() {
    TestDrive drive = new TestDrive();
    Superstructure superstructure = createSuperstructure(drive);

    superstructure.setGoal(
        new SuperstructureGoal.IntakeDepot(SuperstructureGoal.IntakePhase.SETTLE));
    superstructure.periodic();

    Command command = RobotContainer.cancelRecoverIntent(superstructure, drive);
    command.schedule();
    WpilibTestSupport.stepScheduler(0.02);

    assertTrue(superstructure.getStatus().requestedGoal().isEmpty());
    assertTrue(drive.stopCalls > 0);
  }

  @Test
  void manualOverrideKeepsClearingGoalsWhileRunningDriveCommand() {
    TestDrive drive = new TestDrive();
    Superstructure superstructure = createSuperstructure(drive);

    Command rawDriveCommand =
        Commands.run(
            () -> {
              drive.rawDriveTicks++;
              drive.setPose(
                  new Pose2d(
                      drive.getPose().getX() + 0.1,
                      drive.getPose().getY(),
                      drive.getPose().getRotation()));
            },
            drive);

    Command command = RobotContainer.manualOverrideIntent(superstructure, rawDriveCommand);
    command.schedule();

    superstructure.setGoal(new SuperstructureGoal.OutpostAlign());
    WpilibTestSupport.stepScheduler(0.02);
    assertTrue(superstructure.getStatus().requestedGoal().isEmpty());

    superstructure.setGoal(
        new SuperstructureGoal.Endgame(
            SuperstructureGoal.EndgamePhase.LEVEL,
            TargetSelector.ParkZoneSelection.ALLIANCE_LOWER));
    WpilibTestSupport.stepScheduler(0.02);

    assertTrue(superstructure.getStatus().requestedGoal().isEmpty());
    assertTrue(drive.rawDriveTicks >= 2);
    assertTrue(drive.getPose().getX() > 0.0);
  }

  private static Superstructure createSuperstructure(TestDrive drive) {
    return new Superstructure(
        drive,
        new FakeIntake(),
        new FakeIndexer(),
        new FakeShooter(),
        new FakeEndgame(),
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
            return new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0));
          }
        });
  }

  private static final class TestDrive extends Drive {
    private Pose2d pose = Pose2d.kZero;
    private int stopCalls = 0;
    private int rawDriveTicks = 0;

    private TestDrive() {
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

    @Override
    public void stop() {
      stopCalls++;
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
      return new ChassisSpeeds();
    }

    @Override
    public boolean isAimed(java.util.function.Supplier<Rotation2d> targetHeadingSupplier) {
      return true;
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
      return new IntakeStatus(goal, true, goal != IntakeGoal.STOW, false, false);
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
      return new IndexerStatus(goal, true, false, goal == IndexerGoal.FEED_SHOOTER, false);
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
      return new ShooterStatus(goal, null, true, true);
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
