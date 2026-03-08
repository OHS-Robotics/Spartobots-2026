package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleShooter extends SubsystemBase implements Shooter {
  private ShooterGoal goal = new ShooterGoal.Stow();
  private boolean atGoal = true;
  private Boolean readyOverride = null;

  public void setAtGoal(boolean atGoal) {
    this.atGoal = atGoal;
  }

  public void setReadyOverride(Boolean readyOverride) {
    this.readyOverride = readyOverride;
  }

  @Override
  public void setGoal(ShooterGoal goal) {
    this.goal = goal;
  }

  @Override
  public ShooterGoal getGoal() {
    return goal;
  }

  @Override
  public ShooterStatus getStatus() {
    ShotSolution activeSolution = null;
    boolean readyToShoot = false;
    if (goal instanceof ShooterGoal.Track track) {
      activeSolution = track.shotSolution();
    } else if (goal instanceof ShooterGoal.Ready ready) {
      activeSolution = ready.shotSolution();
      readyToShoot = true;
    } else if (goal instanceof ShooterGoal.Fire fire) {
      activeSolution = fire.shotSolution();
      readyToShoot = true;
    }
    if (readyOverride != null) {
      readyToShoot = readyOverride.booleanValue();
    }
    return new ShooterStatus(goal, activeSolution, atGoal, readyToShoot);
  }

  @Override
  public boolean atGoal() {
    return atGoal;
  }

  @Override
  public void stop() {
    setGoal(new ShooterGoal.Stow());
  }
}
