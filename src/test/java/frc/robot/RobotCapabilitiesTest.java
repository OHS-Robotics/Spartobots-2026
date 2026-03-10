package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endgame.Endgame;
import frc.robot.subsystems.endgame.EndgameGoal;
import frc.robot.subsystems.endgame.EndgameStatus;
import frc.robot.subsystems.endgame.SimpleEndgame;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerGoal;
import frc.robot.subsystems.indexer.IndexerStatus;
import frc.robot.subsystems.indexer.SimpleIndexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeGoal;
import frc.robot.subsystems.intake.IntakeStatus;
import frc.robot.subsystems.intake.SimpleIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterGoal;
import frc.robot.subsystems.shooter.ShooterStatus;
import frc.robot.subsystems.shooter.SimpleShooter;
import frc.robot.subsystems.superstructure.SuperstructureGoal;
import org.junit.jupiter.api.Test;

class RobotCapabilitiesTest {
  @Test
  void simpleSubsystemsAreMarkedUnavailableOnRealRobot() {
    RobotCapabilities capabilities =
        RobotCapabilities.create(
            Constants.Mode.REAL,
            new SimpleIntake(),
            new SimpleIndexer(),
            new SimpleShooter(),
            new SimpleEndgame());

    assertFalse(capabilities.intake());
    assertFalse(capabilities.indexer());
    assertFalse(capabilities.shooter());
    assertFalse(capabilities.endgame());
    assertFalse(capabilities.supportsAcquire());
    assertFalse(capabilities.supportsHubShot());
    assertFalse(capabilities.supportsOutpostFeed());
    assertFalse(capabilities.supportsQuickPark());
    assertFalse(capabilities.supportsMatchAutos());
  }

  @Test
  void simpleSubsystemsRemainAvailableInSimulation() {
    RobotCapabilities capabilities =
        RobotCapabilities.create(
            Constants.Mode.SIM,
            new SimpleIntake(),
            new SimpleIndexer(),
            new SimpleShooter(),
            new SimpleEndgame());

    assertTrue(capabilities.intake());
    assertTrue(capabilities.indexer());
    assertTrue(capabilities.shooter());
    assertTrue(capabilities.endgame());
    assertTrue(capabilities.supportsAcquire());
    assertTrue(capabilities.supportsHubShot());
    assertTrue(capabilities.supportsOutpostFeed());
    assertTrue(capabilities.supportsQuickPark());
    assertTrue(capabilities.supportsMatchAutos());
  }

  @Test
  void goalSupportTracksAvailableHardware() {
    RobotCapabilities intakeOnlyCaps =
        RobotCapabilities.create(
            Constants.Mode.REAL,
            new HardwareIntake(),
            new HardwareIndexer(),
            new SimpleShooter(),
            new SimpleEndgame());

    assertTrue(
        intakeOnlyCaps.supportsGoal(
            new SuperstructureGoal.IntakeFloor(SuperstructureGoal.IntakePhase.SETTLE)));
    assertTrue(
        intakeOnlyCaps.supportsGoal(
            new SuperstructureGoal.Eject(SuperstructureGoal.EjectPhase.FIRE)));
    assertFalse(
        intakeOnlyCaps.supportsGoal(
            new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE)));
    assertFalse(
        intakeOnlyCaps.supportsGoal(
            new SuperstructureGoal.Endgame(
                SuperstructureGoal.EndgamePhase.LEVEL,
                TargetSelector.ParkZoneSelection.ALLIANCE_LOWER)));

    RobotCapabilities fullCaps =
        RobotCapabilities.create(
            Constants.Mode.REAL,
            new HardwareIntake(),
            new HardwareIndexer(),
            new HardwareShooter(),
            new HardwareEndgame());

    assertTrue(
        fullCaps.supportsGoal(
            new SuperstructureGoal.HubShot(SuperstructureGoal.HubShotPhase.FIRE)));
    assertTrue(fullCaps.supportsGoal(new SuperstructureGoal.OutpostAlign()));
    assertTrue(
        fullCaps.supportsGoal(
            new SuperstructureGoal.Endgame(
                SuperstructureGoal.EndgamePhase.LEVEL,
                TargetSelector.ParkZoneSelection.ALLIANCE_LOWER)));
  }

  private static class HardwareIntake extends SubsystemBase implements Intake {
    @Override
    public void setGoal(IntakeGoal goal) {}

    @Override
    public IntakeGoal getGoal() {
      return IntakeGoal.STOW;
    }

    @Override
    public IntakeStatus getStatus() {
      return new IntakeStatus(IntakeGoal.STOW, true, false, false, false);
    }

    @Override
    public boolean isAtGoal() {
      return true;
    }

    @Override
    public void stop() {}
  }

  private static class HardwareIndexer extends SubsystemBase implements Indexer {
    @Override
    public void setGoal(IndexerGoal goal) {}

    @Override
    public IndexerGoal getGoal() {
      return IndexerGoal.IDLE;
    }

    @Override
    public IndexerStatus getStatus() {
      return new IndexerStatus(IndexerGoal.IDLE, true, false, false, false);
    }

    @Override
    public boolean isAtGoal() {
      return true;
    }

    @Override
    public void stop() {}
  }

  private static class HardwareShooter extends SubsystemBase implements Shooter {
    @Override
    public void setGoal(ShooterGoal goal) {}

    @Override
    public ShooterGoal getGoal() {
      return new ShooterGoal.Stow();
    }

    @Override
    public ShooterStatus getStatus() {
      return new ShooterStatus(new ShooterGoal.Stow(), null, true, false);
    }

    @Override
    public boolean isAtGoal() {
      return true;
    }

    @Override
    public void stop() {}
  }

  private static class HardwareEndgame extends SubsystemBase implements Endgame {
    @Override
    public void setGoal(EndgameGoal goal) {}

    @Override
    public EndgameGoal getGoal() {
      return EndgameGoal.STOWED;
    }

    @Override
    public EndgameStatus getStatus() {
      return new EndgameStatus(EndgameGoal.STOWED, true, false);
    }

    @Override
    public boolean isAtGoal() {
      return true;
    }

    @Override
    public void stop() {}
  }
}
