package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Indexer extends Subsystem {
  public void setGoal(IndexerGoal goal);

  public IndexerGoal getGoal();

  public IndexerStatus getStatus();

  public boolean isAtGoal();

  public void stop();
}
