package frc.robot.subsystems.endgame;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Endgame extends Subsystem {
  public void setGoal(EndgameGoal goal);

  public EndgameGoal getGoal();

  public EndgameStatus getStatus();

  public boolean isAtGoal();

  public void stop();
}
