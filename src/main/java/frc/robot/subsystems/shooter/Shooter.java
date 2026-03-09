package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Shooter extends Subsystem {
  public void setGoal(ShooterGoal goal);

  public ShooterGoal getGoal();

  public ShooterStatus getStatus();

  public boolean isAtGoal();

  public void stop();
}
