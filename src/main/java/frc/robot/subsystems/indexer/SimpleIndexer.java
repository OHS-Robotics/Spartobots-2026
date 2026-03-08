package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleIndexer extends SubsystemBase implements Indexer {
  private IndexerGoal goal = IndexerGoal.IDLE;
  private boolean atGoal = true;
  private boolean holdingPiece = false;

  public void setAtGoal(boolean atGoal) {
    this.atGoal = atGoal;
  }

  public void setHoldingPiece(boolean holdingPiece) {
    this.holdingPiece = holdingPiece;
  }

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
    boolean feeding = goal == IndexerGoal.FEED_SHOOTER;
    boolean purging = goal == IndexerGoal.PURGE;
    return new IndexerStatus(goal, atGoal, holdingPiece, feeding, purging);
  }

  @Override
  public boolean atGoal() {
    return atGoal;
  }

  @Override
  public void stop() {
    setGoal(IndexerGoal.IDLE);
  }
}
