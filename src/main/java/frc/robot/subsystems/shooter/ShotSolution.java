package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.TargetSelector.HubSelection;
import frc.robot.subsystems.superstructure.Superstructure.PieceState;

public record ShotSolution(
    Pose2d robotPose,
    ChassisSpeeds chassisVelocity,
    HubSelection selectedHub,
    PieceState pieceState,
    Pose2d targetPose,
    Rotation2d targetHeading,
    double launchSpeedMetersPerSec,
    Rotation2d launchAngle,
    double launchHeightMeters,
    double airtimeSeconds) {}
