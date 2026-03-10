package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.TargetSelector;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.testing.SubsystemContractSuite;
import frc.robot.testing.SubsystemFidelity;
import org.junit.jupiter.api.Test;

class SimpleShooterContractTest {
  @Test
  void simpleShooterSatisfiesGoalLifecycleContract() {
    SimpleShooter shooter = new SimpleShooter();
    shooter.setAtGoal(false);

    ShotSolution solution =
        ShotSolution.of(
            Pose2d.kZero,
            new ChassisSpeeds(),
            TargetSelector.HubSelection.ACTIVE,
            Superstructure.PieceState.HELD,
            Pose2d.kZero,
            Rotation2d.fromDegrees(12.0),
            12.5,
            Rotation2d.fromDegrees(45.0),
            0.8,
            0.7);
    ShooterGoal commandedGoal = new ShooterGoal.Fire(solution);
    ShooterGoal stoppedGoal = new ShooterGoal.Stow();

    SubsystemContractSuite.verifyGoalLifecycle(
        "SimpleShooter",
        SubsystemFidelity.PLACEHOLDER,
        new SubsystemContractSuite.GoalController<ShooterGoal, ShooterStatus>() {
          @Override
          public void setGoal(ShooterGoal goal) {
            shooter.setGoal(goal);
          }

          @Override
          public ShooterGoal getGoal() {
            return shooter.getGoal();
          }

          @Override
          public ShooterStatus getStatus() {
            return shooter.getStatus();
          }

          @Override
          public boolean isAtGoal() {
            return shooter.isAtGoal();
          }

          @Override
          public void stop() {
            shooter.stop();
          }
        },
        () -> shooter.getStatus().isAtGoal(),
        commandedGoal,
        stoppedGoal,
        (ShooterStatus status) -> {
          assertFalse(status.isAtGoal());
          assertNotNull(status.activeShotSolution());
          assertTrue(status.readyToShoot());
        },
        (ShooterStatus status) -> {
          assertFalse(status.isAtGoal());
          assertFalse(status.readyToShoot());
          assertTrue(status.goal() instanceof ShooterGoal.Stow);
        });
  }
}
