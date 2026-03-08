package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Intake extends Subsystem {
  public void setGoal(IntakeGoal goal);

  public IntakeGoal getGoal();

  public IntakeStatus getStatus();

  public boolean atGoal();

  public void stop();
}
