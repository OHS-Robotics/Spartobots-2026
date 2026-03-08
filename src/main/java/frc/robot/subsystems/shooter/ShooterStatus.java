package frc.robot.subsystems.shooter;

public record ShooterStatus(
    ShooterGoal goal, ShotSolution activeShotSolution, boolean atGoal, boolean readyToShoot) {}
