package frc.robot.subsystems.endgame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleEndgame extends SubsystemBase implements Endgame {
  private EndgameGoal goal = EndgameGoal.STOWED;
  private boolean atGoal = true;

  public void setAtGoal(boolean atGoal) {
    this.atGoal = atGoal;
  }

  @Override
  public void setGoal(EndgameGoal goal) {
    this.goal = goal;
  }

  @Override
  public EndgameGoal getGoal() {
    return goal;
  }

  @Override
  public EndgameStatus getStatus() {
    return new EndgameStatus(goal, atGoal, goal != EndgameGoal.STOWED);
  }

  @Override
  public boolean atGoal() {
    return atGoal;
  }

  @Override
  public void stop() {
    setGoal(EndgameGoal.STOWED);
  }
}
