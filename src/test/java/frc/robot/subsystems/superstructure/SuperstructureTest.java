package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.TargetSelector;
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
import org.junit.jupiter.api.Test;

class SuperstructureTest {
  private static final double EPSILON = 1e-9;

  @Test
  void depotIntakeProgressesFromPrepToSettle() {
    FakeDrive drive = new FakeDrive();
    FakeIntake intake = new FakeIntake();
    FakeIndexer indexer = new FakeIndexer();
    Superstructure superstructure =
        createSuperstructure(drive, intake, indexer, new FakeShooter(), new FakeEndgame());

    superstructure.setGoal(
        new SuperstructureGoal.IntakeDepot(SuperstructureGoal.IntakePhase.SETTLE));
    superstructure.periodic();
    assertEquals(
        new SuperstructureGoal.IntakeDepot(SuperstructureGoal.IntakePhase.CAPTURE),
        superstructure.getStatus().activeGoal());
    assertEquals(IntakeGoal.COLLECT_DEPOT, intake.goal);
    assertEquals(IndexerGoal.RECEIVE, indexer.goal);

    indexer.holdingPiece = true;

    superstructure.periodic();
    assertEquals(
        new SuperstructureGoal.IntakeDepot(SuperstructureGoal.IntakePhase.SETTLE),
        superstructure.getStatus().activeGoal());
    assertEquals(Superstructure.PieceState.HELD, superstructure.getStatus().pieceState());
    assertTrue(superstructure.hasPiece());
  }

  @Test
  void floorIntakeProgressesFromPrepToSettle() {
    FakeDrive drive = new FakeDrive();
    FakeIntake intake = new FakeIntake();
    FakeIndexer indexer = new FakeIndexer();
    Superstructure superstructure =
        createSuperstructure(drive, intake, indexer, new FakeShooter(), new FakeEndgame());

    superstructure.setGoal(
        new SuperstructureGoal.IntakeFloor(SuperstructureGoal.IntakePhase.SETTLE));
    superstructure.periodic();
    superstructure.periodic();
    indexer.holdingPiece = true;
    superstructure.periodic();

    assertEquals(
        new SuperstructureGoal.IntakeFloor(SuperstructureGoal.IntakePhase.SETTLE),
        superstructure.getStatus().activeGoal());
    assertEquals(IntakeGoal.HOLD, intake.goal);
    assertEquals(IndexerGoal.SETTLE, indexer.goal);
    assertEquals(Superstructure.PieceState.HELD, superstructure.getStatus().pieceState());
  }

  @Test
  void hubShotWaitsForPieceAndAlignmentBeforeFiring() {
    FakeDrive drive = new FakeDrive();
    drive.pose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0));
    FakeIntake intake = new FakeIntake();
    FakeIndexer indexer = new FakeIndexer();
    FakeShooter shooter = new FakeShooter();
    Superstructure superstructure =
        createSuperstructure(drive, intake, indexer, shooter, new FakeEndgame());

    superstructure.setGoal(new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE));
    superstructure.periodic();
    superstructure.periodic();
    assertEquals(
        new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.AIM),
        superstructure.getStatus().activeGoal());

    indexer.holdingPiece = true;
    superstructure.setGoal(
        new SuperstructureGoal.IntakeDepot(SuperstructureGoal.IntakePhase.SETTLE));
    superstructure.periodic();
    superstructure.periodic();
    superstructure.periodic();
    assertEquals(Superstructure.PieceState.HELD, superstructure.getStatus().pieceState());

    superstructure.setGoal(new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE));
    superstructure.periodic();
    superstructure.periodic();
    assertEquals(
        new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.AIM),
        superstructure.getStatus().activeGoal());
    assertTrue(!superstructure.getStatus().driveAligned());

    drive.pose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    superstructure.periodic();
    superstructure.periodic();
    assertEquals(
        new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE),
        superstructure.getStatus().activeGoal());
    assertInstanceOf(ShooterGoal.Fire.class, shooter.goal);
    assertEquals(IndexerGoal.FEED_SHOOTER, indexer.goal);
    assertEquals(Superstructure.PieceState.EMPTY, superstructure.getStatus().pieceState());
    assertTrue(superstructure.isAtGoal());
  }

  @Test
  void hubShotCompensatesTargetHeadingUsingFieldVelocity() {
    FakeDrive drive = new FakeDrive();
    drive.chassisSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
    Superstructure superstructure =
        createSuperstructure(
            drive, new FakeIntake(), new FakeIndexer(), new FakeShooter(), new FakeEndgame());

    superstructure.setGoal(new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.PREP));
    superstructure.periodic();

    SuperstructureStatus status = superstructure.getStatus();
    assertNotNull(status.shotSolution());
    assertEquals(TargetSelector.HubSelection.ACTIVE, status.shotSolution().selectedHub());
    assertEquals(Superstructure.PieceState.EMPTY, status.shotSolution().pieceState());
    assertTrue(status.targetHeading().getRadians() < 0.0);
    assertEquals(
        Math.atan2(-status.shotSolution().airtimeSeconds(), 5.0),
        status.targetHeading().getRadians(),
        1e-6);
  }

  @Test
  void hubShotRequiresTrustedPoseBeforeFiring() {
    FakeDrive drive = new FakeDrive();
    drive.poseTrusted = false;
    FakeIndexer indexer = new FakeIndexer();
    Superstructure superstructure =
        createSuperstructure(
            drive, new FakeIntake(), indexer, new FakeShooter(), new FakeEndgame());

    indexer.holdingPiece = true;
    superstructure.setGoal(
        new SuperstructureGoal.IntakeDepot(SuperstructureGoal.IntakePhase.SETTLE));
    superstructure.periodic();
    superstructure.periodic();
    superstructure.periodic();

    superstructure.setGoal(new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE));
    superstructure.periodic();
    superstructure.periodic();

    assertEquals(
        new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.AIM),
        superstructure.getStatus().activeGoal());
    assertTrue(!superstructure.getStatus().driveAligned());

    drive.poseTrusted = true;
    superstructure.periodic();
    superstructure.periodic();

    assertEquals(
        new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE),
        superstructure.getStatus().activeGoal());
  }

  @Test
  void outpostAlignKeepsMechanismsSafe() {
    FakeDrive drive = new FakeDrive();
    FakeIntake intake = new FakeIntake();
    FakeIndexer indexer = new FakeIndexer();
    FakeShooter shooter = new FakeShooter();
    FakeEndgame endgame = new FakeEndgame();
    Superstructure superstructure = createSuperstructure(drive, intake, indexer, shooter, endgame);

    superstructure.setGoal(new SuperstructureGoal.OutpostAlign());
    superstructure.periodic();

    assertEquals(new SuperstructureGoal.OutpostAlign(), superstructure.getStatus().activeGoal());
    assertEquals(IntakeGoal.STOW, intake.goal);
    assertEquals(IndexerGoal.HOLD, indexer.goal);
    assertInstanceOf(ShooterGoal.Stow.class, shooter.goal);
    assertEquals(EndgameGoal.STOWED, endgame.goal);
    assertEquals(
        Rotation2d.fromDegrees(180.0).getRadians(),
        superstructure.getStatus().targetHeading().getRadians(),
        EPSILON);
  }

  @Test
  void endgameForcesSafeGoals() {
    FakeDrive drive = new FakeDrive();
    FakeIntake intake = new FakeIntake();
    FakeIndexer indexer = new FakeIndexer();
    FakeShooter shooter = new FakeShooter();
    FakeEndgame endgame = new FakeEndgame();
    Superstructure superstructure = createSuperstructure(drive, intake, indexer, shooter, endgame);

    superstructure.setGoal(
        new SuperstructureGoal.Endgame(
            SuperstructureGoal.EndgamePhase.LEVEL,
            TargetSelector.ParkZoneSelection.ALLIANCE_LOWER));
    superstructure.periodic();
    superstructure.periodic();
    superstructure.periodic();

    assertEquals(
        new SuperstructureGoal.Endgame(
            SuperstructureGoal.EndgamePhase.LEVEL, TargetSelector.ParkZoneSelection.ALLIANCE_LOWER),
        superstructure.getStatus().activeGoal());
    assertEquals(IntakeGoal.STOW, intake.goal);
    assertEquals(IndexerGoal.HOLD, indexer.goal);
    assertInstanceOf(ShooterGoal.Safe.class, shooter.goal);
    assertEquals(EndgameGoal.LEVEL, endgame.goal);
  }

  @Test
  void ejectClearsLatchedPossession() {
    FakeDrive drive = new FakeDrive();
    FakeIndexer indexer = new FakeIndexer();
    Superstructure superstructure =
        createSuperstructure(
            drive, new FakeIntake(), indexer, new FakeShooter(), new FakeEndgame());

    indexer.holdingPiece = true;
    superstructure.setGoal(
        new SuperstructureGoal.IntakeDepot(SuperstructureGoal.IntakePhase.SETTLE));
    superstructure.periodic();
    superstructure.periodic();
    superstructure.periodic();
    assertTrue(superstructure.hasPiece());

    superstructure.setGoal(new SuperstructureGoal.Eject(SuperstructureGoal.EjectPhase.FIRE));
    superstructure.periodic();
    superstructure.periodic();

    assertEquals(
        new SuperstructureGoal.Eject(SuperstructureGoal.EjectPhase.FIRE),
        superstructure.getStatus().activeGoal());
    assertEquals(IndexerGoal.PURGE, indexer.goal);
    assertEquals(Superstructure.PieceState.EMPTY, superstructure.getStatus().pieceState());
    assertTrue(!superstructure.hasPiece());
  }

  private static Superstructure createSuperstructure(
      FakeDrive drive,
      FakeIntake intake,
      FakeIndexer indexer,
      FakeShooter shooter,
      FakeEndgame endgame) {
    return new Superstructure(
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
            return new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0));
          }
        });
  }

  private static final class FakeDrive implements SuperstructureDrive {
    private Pose2d pose = new Pose2d();
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private boolean poseTrusted = true;

    @Override
    public Pose2d getPose() {
      return pose;
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
      return chassisSpeeds;
    }

    @Override
    public Command faceTarget(
        java.util.function.DoubleSupplier xSupplier,
        java.util.function.DoubleSupplier ySupplier,
        java.util.function.Supplier<Rotation2d> targetHeading) {
      return Commands.none();
    }

    @Override
    public boolean isAimed(java.util.function.Supplier<Rotation2d> targetHeadingSupplier) {
      Rotation2d targetHeading = targetHeadingSupplier.get();
      return Math.abs(pose.getRotation().minus(targetHeading).getRadians())
          <= Rotation2d.fromDegrees(3.0).getRadians();
    }

    @Override
    public boolean isPoseTrusted() {
      return poseTrusted;
    }

    @Override
    public Command holdShotPose(java.util.function.Supplier<ShotSolution> shotSolutionSupplier) {
      return Commands.none();
    }
  }

  private static final class FakeIntake implements Intake {
    private IntakeGoal goal = IntakeGoal.STOW;
    private boolean atGoal = true;

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
      return new IntakeStatus(goal, atGoal, true, false, false);
    }

    @Override
    public boolean isAtGoal() {
      return atGoal;
    }

    @Override
    public void stop() {
      goal = IntakeGoal.STOW;
    }
  }

  private static final class FakeIndexer implements Indexer {
    private IndexerGoal goal = IndexerGoal.IDLE;
    private boolean atGoal = true;
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
          goal, atGoal, holdingPiece, goal == IndexerGoal.FEED_SHOOTER, goal == IndexerGoal.PURGE);
    }

    @Override
    public boolean isAtGoal() {
      return atGoal;
    }

    @Override
    public void stop() {
      goal = IndexerGoal.IDLE;
    }
  }

  private static final class FakeShooter implements Shooter {
    private ShooterGoal goal = new ShooterGoal.Stow();
    private boolean atGoal = true;

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
      boolean ready = goal instanceof ShooterGoal.Ready || goal instanceof ShooterGoal.Fire;
      frc.robot.subsystems.shooter.ShotSolution solution = null;
      if (goal instanceof ShooterGoal.Track track) {
        solution = track.shotSolution();
      } else if (goal instanceof ShooterGoal.Ready readyGoal) {
        solution = readyGoal.shotSolution();
      } else if (goal instanceof ShooterGoal.Fire fire) {
        solution = fire.shotSolution();
      }
      return new ShooterStatus(goal, solution, atGoal, ready);
    }

    @Override
    public boolean isAtGoal() {
      return atGoal;
    }

    @Override
    public void stop() {
      goal = new ShooterGoal.Stow();
    }
  }

  private static final class FakeEndgame implements Endgame {
    private EndgameGoal goal = EndgameGoal.STOWED;
    private boolean atGoal = true;

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
      return new EndgameStatus(goal, atGoal, goal != EndgameGoal.STOWED);
    }

    @Override
    public boolean isAtGoal() {
      return atGoal;
    }

    @Override
    public void stop() {
      goal = EndgameGoal.STOWED;
    }
  }
}
