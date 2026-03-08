package frc.robot.subsystems.shooter;

public sealed interface ShooterGoal
    permits ShooterGoal.Stow,
        ShooterGoal.Track,
        ShooterGoal.Ready,
        ShooterGoal.Fire,
        ShooterGoal.Safe {
  public record Stow() implements ShooterGoal {}

  public record Track(ShotSolution shotSolution) implements ShooterGoal {}

  public record Ready(ShotSolution shotSolution) implements ShooterGoal {}

  public record Fire(ShotSolution shotSolution) implements ShooterGoal {}

  public record Safe() implements ShooterGoal {}
}
