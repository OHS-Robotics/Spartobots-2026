package frc.robot.subsystems.endgame;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.testing.SubsystemContractSuite;
import frc.robot.testing.SubsystemFidelity;
import org.junit.jupiter.api.Test;

class SimpleEndgameContractTest {
  @Test
  void simpleEndgameSatisfiesGoalLifecycleContract() {
    SimpleEndgame endgame = new SimpleEndgame();
    endgame.setAtGoal(false);

    SubsystemContractSuite.verifyGoalLifecycle(
        "SimpleEndgame",
        SubsystemFidelity.PLACEHOLDER,
        new SubsystemContractSuite.GoalController<EndgameGoal, EndgameStatus>() {
          @Override
          public void setGoal(EndgameGoal goal) {
            endgame.setGoal(goal);
          }

          @Override
          public EndgameGoal getGoal() {
            return endgame.getGoal();
          }

          @Override
          public EndgameStatus getStatus() {
            return endgame.getStatus();
          }

          @Override
          public boolean isAtGoal() {
            return endgame.isAtGoal();
          }

          @Override
          public void stop() {
            endgame.stop();
          }
        },
        () -> endgame.getStatus().isAtGoal(),
        EndgameGoal.LEVEL,
        EndgameGoal.STOWED,
        (EndgameStatus status) -> {
          assertFalse(status.isAtGoal());
          assertTrue(status.engaged());
        },
        (EndgameStatus status) -> {
          assertFalse(status.isAtGoal());
          assertFalse(status.engaged());
        });
  }
}
