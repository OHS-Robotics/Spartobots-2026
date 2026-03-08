package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public record ShotSolution(
    double launchSpeedMetersPerSec,
    Rotation2d launchAngle,
    double launchHeightMeters,
    double airtimeSeconds) {}
