package frc.robot.subsystems.intake;

public record IntakeStatus(
    IntakeGoal goal, boolean atGoal, boolean deployed, boolean collecting, boolean reversing) {}
