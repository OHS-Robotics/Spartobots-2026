package com.team4687.frc2026.subsystems.body;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class HubSolver {
    public record HubShotSolution(
        double distance,
        double angle,
        double power, // 0-1
        boolean feasible
    ) {}

    private HubShotSolution currentEsimate = new HubShotSolution(0.0, 0.0, 0.0, false);

    public void updateHubShotSolution(Pose2d robotPose, Pose2d targetPose) {
        double dx = targetPose.getX()-robotPose.getX();
        double dy = targetPose.getY()-robotPose.getY();
        double distance = Math.sqrt(dx*dx + dy*dy);

        double angleEstimate = Units.degreesToRadians(distance*3); // rough estimate based on 10m/s start velocity
        double powerEsimate  = ShooterConstants.preferredPower;
    }

    private double solveForPower(double angle, double range) {
        return Math.asin()
    }
}
