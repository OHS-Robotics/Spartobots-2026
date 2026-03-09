package frc.robot.subsystems.indexer;

public record IndexerStatus(
    IndexerGoal goal, boolean isAtGoal, boolean holdingPiece, boolean feeding, boolean purging) {}
