package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldTargets;
import frc.robot.MatchStateProvider;
import frc.robot.TargetSelector;
import frc.robot.subsystems.endgame.SimpleEndgame;
import frc.robot.subsystems.indexer.SimpleIndexer;
import frc.robot.subsystems.intake.SimpleIntake;
import frc.robot.subsystems.shooter.ShooterBallistics;
import frc.robot.subsystems.shooter.ShotSolution;
import frc.robot.subsystems.shooter.SimpleShooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureDrive;
import frc.robot.subsystems.superstructure.SuperstructureGoal;
import java.util.Optional;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.junit.jupiter.api.Test;

class MatchSimulationTest {
  @Test
  void acquiresNearbyFuelDuringIntakeCapture() {
    SpartobotsArena2026Rebuilt arena = new SpartobotsArena2026Rebuilt(false);
    FakeDrive drive = new FakeDrive();
    SimpleIntake intake = new SimpleIntake();
    SimpleIndexer indexer = new SimpleIndexer();
    SimpleShooter shooter = new SimpleShooter();
    SimpleEndgame endgame = new SimpleEndgame();
    Superstructure superstructure = createSuperstructure(drive, intake, indexer, shooter, endgame);
    MatchSimulation simulation =
        new MatchSimulation(arena, drive, superstructure, intake, indexer, shooter, endgame);

    arena.addGamePiece(
        new RebuiltFuelOnField(new edu.wpi.first.math.geometry.Translation2d(0.42, 0.0)));
    superstructure.setGoal(
        new SuperstructureGoal.IntakeFloor(SuperstructureGoal.IntakePhase.SETTLE));
    superstructure.periodic();

    simulation.beforeArenaStep(blueShiftState(MatchStateProvider.Hub.BLUE), 120.0, drive.pose);

    assertTrue(indexer.getStatus().holdingPiece());
    assertEquals(0, arena.gamePiecesOnField().size());
  }

  @Test
  void refillsFromOutpostWhenAlignedAndEmpty() {
    SpartobotsArena2026Rebuilt arena = new SpartobotsArena2026Rebuilt(false);
    FakeDrive drive = new FakeDrive();
    drive.pose = FieldTargets.OUTPOST.bluePose();
    SimpleIntake intake = new SimpleIntake();
    SimpleIndexer indexer = new SimpleIndexer();
    SimpleShooter shooter = new SimpleShooter();
    SimpleEndgame endgame = new SimpleEndgame();
    Superstructure superstructure = createSuperstructure(drive, intake, indexer, shooter, endgame);
    MatchSimulation simulation =
        new MatchSimulation(arena, drive, superstructure, intake, indexer, shooter, endgame);

    superstructure.setGoal(new SuperstructureGoal.OutpostAlign());
    superstructure.periodic();
    for (int i = 0; i < 20; i++) {
      simulation.beforeArenaStep(blueShiftState(MatchStateProvider.Hub.BLUE), 120.0, drive.pose);
    }
    superstructure.periodic();

    assertTrue(indexer.getStatus().holdingPiece());
    assertTrue(superstructure.hasPiece());
    assertEquals(23, arena.getOutpostFuelCount(Alliance.Blue));
  }

  @Test
  void spawnsProjectileWhenHubShotConsumesHeldFuel() {
    SpartobotsArena2026Rebuilt arena = new SpartobotsArena2026Rebuilt(false);
    FakeDrive drive = new FakeDrive();
    SimpleIntake intake = new SimpleIntake();
    SimpleIndexer indexer = new SimpleIndexer();
    SimpleShooter shooter = new SimpleShooter();
    SimpleEndgame endgame = new SimpleEndgame();
    Superstructure superstructure = createSuperstructure(drive, intake, indexer, shooter, endgame);
    MatchSimulation simulation =
        new MatchSimulation(arena, drive, superstructure, intake, indexer, shooter, endgame);

    indexer.setHoldingPiece(true);
    superstructure.periodic();
    simulation.beforeArenaStep(blueShiftState(MatchStateProvider.Hub.BLUE), 120.0, drive.pose);

    superstructure.setGoal(new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE));
    superstructure.periodic();
    simulation.beforeArenaStep(blueShiftState(MatchStateProvider.Hub.BLUE), 120.0, drive.pose);
    superstructure.periodic();
    simulation.beforeArenaStep(blueShiftState(MatchStateProvider.Hub.BLUE), 120.0, drive.pose);

    assertEquals(1, arena.gamePieceLaunched().size());
    assertEquals(0, arena.gamePiecesOnField().size());
  }

  @Test
  void ejectDropsFuelBackOntoFieldWhenHeldPieceIsPurged() {
    SpartobotsArena2026Rebuilt arena = new SpartobotsArena2026Rebuilt(false);
    FakeDrive drive = new FakeDrive();
    SimpleIntake intake = new SimpleIntake();
    SimpleIndexer indexer = new SimpleIndexer();
    SimpleShooter shooter = new SimpleShooter();
    SimpleEndgame endgame = new SimpleEndgame();
    Superstructure superstructure = createSuperstructure(drive, intake, indexer, shooter, endgame);
    MatchSimulation simulation =
        new MatchSimulation(arena, drive, superstructure, intake, indexer, shooter, endgame);

    indexer.setHoldingPiece(true);
    superstructure.periodic();
    simulation.beforeArenaStep(blueShiftState(MatchStateProvider.Hub.BLUE), 120.0, drive.pose);

    superstructure.setGoal(new SuperstructureGoal.Eject(SuperstructureGoal.EjectPhase.FIRE));
    superstructure.periodic();
    simulation.beforeArenaStep(blueShiftState(MatchStateProvider.Hub.BLUE), 120.0, drive.pose);
    superstructure.periodic();
    simulation.beforeArenaStep(blueShiftState(MatchStateProvider.Hub.BLUE), 120.0, drive.pose);
    superstructure.periodic();
    simulation.beforeArenaStep(blueShiftState(MatchStateProvider.Hub.BLUE), 120.0, drive.pose);

    assertEquals(1, arena.gamePiecesOnField().size());
    assertEquals("Fuel", arena.gamePiecesOnField().iterator().next().getType());
    assertTrue(!indexer.getStatus().holdingPiece());
  }

  @Test
  void resetControlsWhetherRobotStartsWithPreload() {
    SpartobotsArena2026Rebuilt arena = new SpartobotsArena2026Rebuilt(false);
    FakeDrive drive = new FakeDrive();
    SimpleIntake intake = new SimpleIntake();
    SimpleIndexer indexer = new SimpleIndexer();
    SimpleShooter shooter = new SimpleShooter();
    SimpleEndgame endgame = new SimpleEndgame();
    Superstructure superstructure = createSuperstructure(drive, intake, indexer, shooter, endgame);
    MatchSimulation simulation =
        new MatchSimulation(arena, drive, superstructure, intake, indexer, shooter, endgame);

    simulation.reset(true);
    assertTrue(indexer.getStatus().holdingPiece());

    simulation.reset(false);
    assertTrue(!indexer.getStatus().holdingPiece());
  }

  @Test
  void outpostRefillRequiresDriveAlignment() {
    SpartobotsArena2026Rebuilt arena = new SpartobotsArena2026Rebuilt(false);
    FakeDrive drive = new FakeDrive();
    drive.pose = FieldTargets.OUTPOST.bluePose();
    drive.poseTrusted = false;
    SimpleIntake intake = new SimpleIntake();
    SimpleIndexer indexer = new SimpleIndexer();
    SimpleShooter shooter = new SimpleShooter();
    SimpleEndgame endgame = new SimpleEndgame();
    Superstructure superstructure = createSuperstructure(drive, intake, indexer, shooter, endgame);
    MatchSimulation simulation =
        new MatchSimulation(arena, drive, superstructure, intake, indexer, shooter, endgame);

    superstructure.setGoal(new SuperstructureGoal.OutpostAlign());
    for (int i = 0; i < 20; i++) {
      superstructure.periodic();
      simulation.beforeArenaStep(blueShiftState(MatchStateProvider.Hub.BLUE), 120.0, drive.pose);
    }

    assertTrue(!indexer.getStatus().holdingPiece());
    assertEquals(24, arena.getOutpostFuelCount(Alliance.Blue));
  }

  private static MatchStateProvider.MatchState blueShiftState(MatchStateProvider.Hub activeHub) {
    return new MatchStateProvider.MatchState(
        Optional.of(Alliance.Blue),
        MatchStateProvider.TowerLetter.B,
        MatchStateProvider.Hub.BLUE,
        MatchStateProvider.Hub.RED,
        activeHub,
        MatchStateProvider.MatchPhaseScoringContext.SHIFT_1);
  }

  private static Superstructure createSuperstructure(
      FakeDrive drive,
      SimpleIntake intake,
      SimpleIndexer indexer,
      SimpleShooter shooter,
      SimpleEndgame endgame) {
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
            return FieldTargets.OUTPOST.bluePose();
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
      return Math.abs(pose.getRotation().minus(targetHeadingSupplier.get()).getRadians())
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
}
