package frc.robot.subsystems.shooter;

public record ShooterStatus(
    ShooterGoal goal, ShotSolution activeShotSolution, boolean isAtGoal, boolean readyToShoot) {}
