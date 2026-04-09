package frc.robot.sim;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.gamepiece.shooter.ShooterConstants;
import frc.robot.targeting.HubTargetingGeometry;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.junit.jupiter.api.Test;

class FieldSimulationManagerTest {
  @Test
  void fuelTelemetryIncludesLaunchedProjectiles() {
    Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
    arena.clearGamePieces();
    int initialFuelPoseCount = FieldSimulationManager.getFuelGamePiecePoses(arena).length;
    arena.addGamePieceProjectile(
        new RebuiltFuelOnFly(
            new Translation2d(2.0, 2.0),
            new Translation2d(),
            new ChassisSpeeds(),
            Rotation2d.kZero,
            Meters.of(0.6),
            MetersPerSecond.of(5.0),
            Radians.of(0.8)));

    assertEquals(
        initialFuelPoseCount + 1, FieldSimulationManager.getFuelGamePiecePoses(arena).length);
  }

  @Test
  void simulatedFuelOriginMatchesRearShooterMuzzle() {
    Pose2d robotPose = new Pose2d(2.0, 3.0, Rotation2d.kZero);
    Rotation2d shooterFacing = robotPose.getRotation().plus(ShooterConstants.shooterFacingOffset);

    RebuiltFuelOnFly projectile =
        new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            FieldSimulationManager.getMapleSimShooterOffsetOnRobot(),
            new ChassisSpeeds(),
            shooterFacing,
            Meters.of(ShooterConstants.defaultLaunchHeightMeters),
            MetersPerSecond.of(5.0),
            Radians.of(0.8));
    Pose3d projectilePose = projectile.getPose3d();
    Translation2d expectedLaunchOrigin =
        HubTargetingGeometry.getLaunchOriginFieldPosition(robotPose);

    assertEquals(expectedLaunchOrigin.getX(), projectilePose.getX(), 1e-9);
    assertEquals(expectedLaunchOrigin.getY(), projectilePose.getY(), 1e-9);
    assertEquals(ShooterConstants.defaultLaunchHeightMeters, projectilePose.getZ(), 1e-9);
    assertEquals(
        robotPose.getX() - ShooterConstants.shooterMuzzleOffsetOnRobot.getNorm(),
        projectilePose.getX(),
        1e-9);
  }
}
