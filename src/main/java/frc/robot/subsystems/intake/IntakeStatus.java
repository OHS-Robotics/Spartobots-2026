package frc.robot.subsystems.intake;

public record IntakeStatus(
    IntakeGoal goal, boolean isAtGoal, boolean deployed, boolean collecting, boolean reversing) {}
