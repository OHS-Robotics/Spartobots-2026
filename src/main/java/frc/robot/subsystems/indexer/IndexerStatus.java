package frc.robot.subsystems.indexer;

public record IndexerStatus(
    IndexerGoal goal, boolean atGoal, boolean holdingPiece, boolean feeding, boolean purging) {}
