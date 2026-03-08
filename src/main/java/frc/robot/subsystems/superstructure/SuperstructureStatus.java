package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.ShotSolution;
import frc.robot.subsystems.superstructure.Superstructure.PieceState;
import java.util.Optional;

public record SuperstructureStatus(
    Optional<SuperstructureGoal> requestedGoal,
    SuperstructureGoal activeGoal,
    PieceState pieceState,
    boolean hasGamePiece,
    boolean shooterReady,
    boolean driveAligned,
    Pose2d targetPose,
    Rotation2d targetHeading,
    ShotSolution shotSolution,
    boolean atGoal) {}
