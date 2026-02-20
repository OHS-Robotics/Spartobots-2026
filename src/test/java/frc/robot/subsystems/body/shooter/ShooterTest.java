package frc.robot.subsystems.body.shooter;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class ShooterTest {
  @Test
  void hubShotSolverReturnsFeasibleSolutionWithinEnvelope() {
    Shooter shooter = new Shooter(new FakeShooterIO());
    Shooter.HubShotSolution solution =
        shooter.updateHubShotSolution(
            new Pose2d(2.0, 2.0, Rotation2d.kZero), new Pose2d(5.0, 2.0, Rotation2d.kZero));

    assertTrue(solution.feasible());
    assertTrue(solution.launchSpeedMetersPerSec() >= ShooterConstants.minLaunchSpeedMetersPerSec);
    assertTrue(solution.launchSpeedMetersPerSec() <= ShooterConstants.maxLaunchSpeedMetersPerSec);
    assertTrue(solution.launchAngle().getDegrees() >= ShooterConstants.minLaunchAngle.getDegrees());
    assertTrue(solution.launchAngle().getDegrees() <= ShooterConstants.maxLaunchAngle.getDegrees());
  }

  @Test
  void readyToFireTracksMeasuredWheelReadiness() {
    FakeShooterIO io = new FakeShooterIO();
    Shooter shooter = new Shooter(io);
    shooter.updateHubShotSolution(
        new Pose2d(2.0, 2.0, Rotation2d.kZero), new Pose2d(5.0, 2.0, Rotation2d.kZero));

    shooter.setShotControlEnabled(true);
    shooter.periodic();
    assertFalse(shooter.isReadyToFire());

    io.pair1MeasuredVelocityRadPerSec = io.pair1SetpointRadPerSec;
    io.pair2MeasuredVelocityRadPerSec = io.pair2SetpointRadPerSec;
    shooter.periodic();
    assertTrue(shooter.isReadyToFire());

    io.pair1MeasuredVelocityRadPerSec = io.pair1SetpointRadPerSec * 0.5;
    shooter.periodic();
    assertFalse(shooter.isReadyToFire());
  }

  private static class FakeShooterIO implements ShooterIO {
    double pair1SetpointRadPerSec = 0.0;
    double pair2SetpointRadPerSec = 0.0;
    double hoodSetpointRotations = ShooterConstants.defaultHoodRetractedPositionRotations;
    double pair1MeasuredVelocityRadPerSec = 0.0;
    double pair2MeasuredVelocityRadPerSec = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      inputs.pair1Connected = true;
      inputs.pair2Connected = true;
      inputs.hoodConnected = true;
      inputs.pair1LeaderVelocityRadPerSec = pair1MeasuredVelocityRadPerSec;
      inputs.pair2LeaderVelocityRadPerSec = pair2MeasuredVelocityRadPerSec;
      inputs.pair1FollowerVelocityRadPerSec = pair1MeasuredVelocityRadPerSec;
      inputs.pair2FollowerVelocityRadPerSec = pair2MeasuredVelocityRadPerSec;
      inputs.hoodPositionRotations = hoodSetpointRotations;
      inputs.hoodVelocityRotationsPerSec = 0.0;
    }

    @Override
    public void setWheelVelocitySetpoints(double pair1RadPerSec, double pair2RadPerSec) {
      pair1SetpointRadPerSec = pair1RadPerSec;
      pair2SetpointRadPerSec = pair2RadPerSec;
    }

    @Override
    public void setHoodPositionSetpointRotations(double hoodPositionRotations) {
      hoodSetpointRotations = hoodPositionRotations;
    }
  }
}
