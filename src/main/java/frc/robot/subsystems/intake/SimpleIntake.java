package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleIntake extends SubsystemBase implements Intake {
  private IntakeGoal goal = IntakeGoal.STOW;
  private boolean atGoal = true;

  public void setAtGoal(boolean atGoal) {
    this.atGoal = atGoal;
  }

  @Override
  public void setGoal(IntakeGoal goal) {
    this.goal = goal;
  }

  @Override
  public IntakeGoal getGoal() {
    return goal;
  }

  @Override
  public IntakeStatus getStatus() {
    boolean deployed = goal == IntakeGoal.DEPLOY_DEPOT || goal == IntakeGoal.DEPLOY_FLOOR;
    boolean collecting = goal == IntakeGoal.COLLECT_DEPOT || goal == IntakeGoal.COLLECT_FLOOR;
    boolean reversing = goal == IntakeGoal.REVERSE;
    return new IntakeStatus(goal, atGoal, deployed, collecting, reversing);
  }

  @Override
  public boolean atGoal() {
    return atGoal;
  }

  @Override
  public void stop() {
    setGoal(IntakeGoal.STOW);
  }
}
