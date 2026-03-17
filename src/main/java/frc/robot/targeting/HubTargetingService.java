package frc.robot.targeting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.superstructure.gamepiece.GamePieceCoordinator;
import java.util.function.DoubleSupplier;

public class HubTargetingService {
  private final Drive drive;
  private final Shooter shooter;

  public HubTargetingService(Drive drive, Shooter shooter) {
    this.drive = drive;
    this.shooter = shooter;
  }

  public void update() {
    updateAndGetAirtimeSeconds();
  }

  public double updateAndGetAirtimeSeconds() {
    shooter.updateHubShotSolution(
        drive.getPose(),
        drive.getAllianceHubPose(),
        drive.getFieldRelativeVelocityMetersPerSecond());
    return shooter.getHubAirtimeSeconds();
  }

  public Command alignToHub(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, GamePieceCoordinator coordinator) {
    return drive
        .alignToHub(xSupplier, ySupplier, this::updateAndGetAirtimeSeconds)
        .alongWith(
            Commands.startEnd(
                () -> coordinator.setShooterDemandFromAlign(true),
                () -> coordinator.setShooterDemandFromAlign(false)));
  }
}
