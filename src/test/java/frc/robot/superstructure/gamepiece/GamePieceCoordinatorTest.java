package frc.robot.superstructure.gamepiece;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gamepiece.hopper.Hopper;
import frc.robot.subsystems.gamepiece.indexers.Indexers;
import frc.robot.subsystems.gamepiece.indexers.IndexersIO;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.intake.IntakeIO;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.subsystems.gamepiece.shooter.ShooterIO;
import org.junit.jupiter.api.Test;

class GamePieceCoordinatorTest {
  @Test
  void basicFeedInterlockStopsIndexers() {
    FakeIndexersIO indexersIO = new FakeIndexersIO();
    Shooter shooter = new Shooter(new ShooterIO() {});
    shooter.setCalibrationModeEnabled(false);
    GamePieceCoordinator coordinator =
        new GamePieceCoordinator(
            new Intake(new IntakeIO() {}), new Hopper(), new Indexers(indexersIO), shooter);

    coordinator.applyBasicFeed(true);

    assertEquals(0.0, indexersIO.topOutput, 1e-9);
    assertEquals(0.0, indexersIO.bottomOutput, 1e-9);
  }

  @Test
  void manualFeedInterlockStopsIndexersWhileAutoAimIsActive() {
    FakeIndexersIO indexersIO = new FakeIndexersIO();
    Shooter shooter = new Shooter(new ShooterIO() {});
    shooter.setCalibrationModeEnabled(false);
    GamePieceCoordinator coordinator =
        new GamePieceCoordinator(
            new Intake(new IntakeIO() {}), new Hopper(), new Indexers(indexersIO), shooter);

    coordinator.setShooterDemandFromAlign(true);
    Command command = coordinator.runManualFeedAndIndexersWhileHeldCommand(() -> 1.0);

    command.initialize();
    command.execute();

    assertEquals(0.0, indexersIO.topOutput, 1e-9);
    assertEquals(0.0, indexersIO.bottomOutput, 1e-9);

    command.end(false);
  }

  @Test
  void manualFeedRunsSharedIndexerAndBeltNegativeOnTriggerPress() {
    FakeIndexersIO indexersIO = new FakeIndexersIO();
    Shooter shooter = new Shooter(new ShooterIO() {});
    shooter.setCalibrationModeEnabled(false);
    GamePieceCoordinator coordinator =
        new GamePieceCoordinator(
            new Intake(new IntakeIO() {}), new Hopper(), new Indexers(indexersIO), shooter);

    Command command = coordinator.runManualFeedAndIndexersWhileHeldCommand(() -> 1.0);

    command.initialize();
    command.execute();
    command.execute();

    assertEquals(-1.0, indexersIO.topOutput, 1e-9);
    assertEquals(-1.0, indexersIO.bottomOutput, 1e-9);

    command.end(false);
  }

  private static class FakeIndexersIO implements IndexersIO {
    double topOutput = 0.0;
    double bottomOutput = 0.0;

    @Override
    public void setTopOutput(double output) {
      topOutput = output;
    }

    @Override
    public void setBottomOutput(double output) {
      bottomOutput = output;
    }
  }
}
