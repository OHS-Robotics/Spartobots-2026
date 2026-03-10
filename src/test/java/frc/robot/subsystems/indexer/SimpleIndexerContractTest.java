package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.testing.SubsystemContractSuite;
import frc.robot.testing.SubsystemFidelity;
import org.junit.jupiter.api.Test;

class SimpleIndexerContractTest {
  @Test
  void simpleIndexerSatisfiesGoalLifecycleContract() {
    SimpleIndexer indexer = new SimpleIndexer();
    indexer.setAtGoal(false);
    indexer.setHoldingPiece(true);

    SubsystemContractSuite.verifyGoalLifecycle(
        "SimpleIndexer",
        SubsystemFidelity.PLACEHOLDER,
        new SubsystemContractSuite.GoalController<IndexerGoal, IndexerStatus>() {
          @Override
          public void setGoal(IndexerGoal goal) {
            indexer.setGoal(goal);
          }

          @Override
          public IndexerGoal getGoal() {
            return indexer.getGoal();
          }

          @Override
          public IndexerStatus getStatus() {
            return indexer.getStatus();
          }

          @Override
          public boolean isAtGoal() {
            return indexer.isAtGoal();
          }

          @Override
          public void stop() {
            indexer.stop();
          }
        },
        () -> indexer.getStatus().isAtGoal(),
        IndexerGoal.FEED_SHOOTER,
        IndexerGoal.IDLE,
        (IndexerStatus status) -> {
          assertFalse(status.isAtGoal());
          assertTrue(status.holdingPiece());
          assertTrue(status.feeding());
          assertFalse(status.purging());
        },
        (IndexerStatus status) -> {
          assertFalse(status.isAtGoal());
          assertTrue(status.holdingPiece());
          assertFalse(status.feeding());
          assertFalse(status.purging());
        });
  }
}
