package frc.robot;

import frc.robot.subsystems.endgame.Endgame;
import frc.robot.subsystems.endgame.SimpleEndgame;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.SimpleIndexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.SimpleIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.SimpleShooter;
import frc.robot.subsystems.superstructure.SuperstructureGoal;

public record RobotCapabilities(
    boolean drive,
    boolean vision,
    boolean intake,
    boolean indexer,
    boolean shooter,
    boolean endgame) {
  public static RobotCapabilities create(
      Constants.Mode mode, Intake intake, Indexer indexer, Shooter shooter, Endgame endgame) {
    boolean realHardwareRequired = mode == Constants.Mode.REAL;
    return new RobotCapabilities(
        true,
        true,
        !realHardwareRequired || !(intake instanceof SimpleIntake),
        !realHardwareRequired || !(indexer instanceof SimpleIndexer),
        !realHardwareRequired || !(shooter instanceof SimpleShooter),
        !realHardwareRequired || !(endgame instanceof SimpleEndgame));
  }

  public boolean supportsAcquire() {
    return intake && indexer;
  }

  public boolean supportsHubShot() {
    return indexer && shooter;
  }

  public boolean supportsOutpostFeed() {
    return drive && indexer;
  }

  public boolean supportsQuickPark() {
    return drive && endgame;
  }

  public boolean supportsMatchAutos() {
    return drive && intake && indexer && shooter && endgame;
  }

  public boolean supportsGoal(SuperstructureGoal goal) {
    if (goal instanceof SuperstructureGoal.Stow) {
      return true;
    }
    if (goal instanceof SuperstructureGoal.IntakeDepot
        || goal instanceof SuperstructureGoal.IntakeFloor
        || goal instanceof SuperstructureGoal.Eject) {
      return supportsAcquire();
    }
    if (goal instanceof SuperstructureGoal.HubShot) {
      return supportsHubShot();
    }
    if (goal instanceof SuperstructureGoal.OutpostAlign) {
      return supportsOutpostFeed();
    }
    if (goal instanceof SuperstructureGoal.Endgame) {
      return supportsQuickPark();
    }
    return false;
  }
}
