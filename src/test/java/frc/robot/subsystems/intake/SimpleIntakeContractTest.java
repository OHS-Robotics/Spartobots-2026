package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.testing.SubsystemContractSuite;
import frc.robot.testing.SubsystemFidelity;
import org.junit.jupiter.api.Test;

class SimpleIntakeContractTest {
  @Test
  void simpleIntakeSatisfiesGoalLifecycleContract() {
    SimpleIntake intake = new SimpleIntake();
    intake.setAtGoal(false);

    SubsystemContractSuite.verifyGoalLifecycle(
        "SimpleIntake",
        SubsystemFidelity.PLACEHOLDER,
        new SubsystemContractSuite.GoalController<IntakeGoal, IntakeStatus>() {
          @Override
          public void setGoal(IntakeGoal goal) {
            intake.setGoal(goal);
          }

          @Override
          public IntakeGoal getGoal() {
            return intake.getGoal();
          }

          @Override
          public IntakeStatus getStatus() {
            return intake.getStatus();
          }

          @Override
          public boolean isAtGoal() {
            return intake.isAtGoal();
          }

          @Override
          public void stop() {
            intake.stop();
          }
        },
        () -> intake.getStatus().isAtGoal(),
        IntakeGoal.COLLECT_FLOOR,
        IntakeGoal.STOW,
        (IntakeStatus status) -> {
          assertFalse(status.isAtGoal());
          assertFalse(status.deployed());
          assertTrue(status.collecting());
          assertFalse(status.reversing());
        },
        (IntakeStatus status) -> {
          assertFalse(status.isAtGoal());
          assertFalse(status.deployed());
          assertFalse(status.collecting());
          assertFalse(status.reversing());
        });
  }
}
