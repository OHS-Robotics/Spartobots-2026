package frc.robot.subsystems.gamepiece.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.NetworkTablesUtil;
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

  @Test
  void hubShotSolverCompensatesForRobotVelocityAndConverges() {
    Shooter shooter = new Shooter(new FakeShooterIO());
    Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.kZero);
    Pose2d hubPose = new Pose2d(6.0, 2.0, Rotation2d.kZero);

    Shooter.HubShotSolution stationarySolution =
        shooter.updateHubShotSolution(robotPose, hubPose, new Translation2d());
    Shooter.HubShotSolution movingSolution =
        shooter.updateHubShotSolution(robotPose, hubPose, new Translation2d(2.0, 0.0));
    Shooter.HubShotSolution movingSolutionRepeat =
        shooter.updateHubShotSolution(robotPose, hubPose, new Translation2d(2.0, 0.0));

    assertTrue(stationarySolution.feasible());
    assertTrue(movingSolution.feasible());
    assertTrue(movingSolution.distanceMeters() < stationarySolution.distanceMeters());
    assertEquals(movingSolution.airtimeSeconds(), movingSolutionRepeat.airtimeSeconds(), 0.02);
    assertEquals(
        movingSolution.distanceMeters()
            / (movingSolution.launchSpeedMetersPerSec() * movingSolution.launchAngle().getCos()),
        movingSolution.airtimeSeconds(),
        0.02);
  }

  @Test
  void hubShotSolverPrefersDescendingEntryAtHub() {
    Shooter shooter = new Shooter(new FakeShooterIO());
    Shooter.HubShotSolution solution =
        shooter.updateHubShotSolution(
            new Pose2d(2.0, 2.0, Rotation2d.kZero),
            new Pose2d(6.0, 2.0, Rotation2d.kZero),
            new Translation2d());

    assertTrue(solution.feasible());
    double launchSpeedMetersPerSec = solution.launchSpeedMetersPerSec();
    double vx = launchSpeedMetersPerSec * solution.launchAngle().getCos();
    double timeToHubSeconds = solution.distanceMeters() / Math.abs(vx);
    double vyAtHubMetersPerSec =
        (launchSpeedMetersPerSec * solution.launchAngle().getSin())
            - (ShooterConstants.gravityMetersPerSecSquared * timeToHubSeconds);
    assertTrue(vyAtHubMetersPerSec <= -ShooterConstants.hubTopEntryMinDescentVelocityMetersPerSec);
  }

  @Test
  void calibrationModeOwnsShooterSetpoints() {
    FakeShooterIO io = new FakeShooterIO();
    Shooter shooter = new Shooter(io);
    double calibrationHoodSetpointRotations =
        ShooterConstants.defaultHoodRetractedPositionRotations + 1.25;

    shooter.setCalibrationHoodSetpointRotations(calibrationHoodSetpointRotations);
    shooter.setCalibrationWheelSetpointsRadPerSec(120.0, 140.0);
    shooter.setCalibrationModeEnabled(true);
    shooter.setShotControlEnabled(true);
    shooter.periodic();

    assertEquals(calibrationHoodSetpointRotations, shooter.getHoodSetpointMotorRotations(), 1e-9);
    assertEquals(120.0, shooter.getPair1WheelSetpointRadPerSec(), 1e-9);
    assertEquals(140.0, shooter.getPair2WheelSetpointRadPerSec(), 1e-9);
    assertEquals(-120.0, io.pair1SetpointRadPerSec, 1e-9);
    assertEquals(-140.0, io.pair2SetpointRadPerSec, 1e-9);
    assertTrue(shooter.isCalibrationModeEnabled());
  }

  @Test
  void calibrationSamplePublishesDerivedBallistics() {
    FakeShooterIO io = new FakeShooterIO();
    Shooter shooter = new Shooter(io);
    io.pair1MeasuredVelocityRadPerSec = 90.0;
    io.pair2MeasuredVelocityRadPerSec = 110.0;
    shooter.periodic();
    shooter.setCalibrationHoodSetpointRotations(
        ShooterConstants.defaultHoodRetractedPositionRotations + 0.75);
    shooter.setCalibrationWheelSetpointsRadPerSec(100.0, 120.0);
    shooter.setCalibrationMeasurement(4.0, 1.0, 0.5, 46.0, "test sample");

    shooter.recordCalibrationSample();

    var calibrationTelemetry =
        NetworkTablesUtil.telemetry(NetworkTablesUtil.domain(ShooterConstants.configTableName))
            .getSubTable("Calibration");
    double expectedHorizontalVelocityMetersPerSec = 8.0;
    double expectedVerticalVelocityMetersPerSec =
        (1.0 + (0.5 * ShooterConstants.gravityMetersPerSecSquared * 0.25)) / 0.5;
    double expectedLaunchSpeedMetersPerSec =
        Math.hypot(expectedHorizontalVelocityMetersPerSec, expectedVerticalVelocityMetersPerSec);
    double expectedLaunchAngleDegrees =
        Math.toDegrees(
            Math.atan2(
                expectedVerticalVelocityMetersPerSec, expectedHorizontalVelocityMetersPerSec));

    assertEquals(1.0, calibrationTelemetry.getEntry("RecordedSampleCount").getDouble(0.0), 1e-9);
    assertTrue(calibrationTelemetry.getEntry("LastRecorded/Derived/Valid").getBoolean(false));
    assertEquals(
        expectedHorizontalVelocityMetersPerSec,
        calibrationTelemetry
            .getEntry("LastRecorded/Derived/HorizontalVelocityMetersPerSec")
            .getDouble(0.0),
        1e-9);
    assertEquals(
        expectedVerticalVelocityMetersPerSec,
        calibrationTelemetry
            .getEntry("LastRecorded/Derived/VerticalVelocityMetersPerSec")
            .getDouble(0.0),
        1e-9);
    assertEquals(
        expectedLaunchSpeedMetersPerSec,
        calibrationTelemetry
            .getEntry("LastRecorded/Derived/LaunchSpeedMetersPerSec")
            .getDouble(0.0),
        1e-9);
    assertEquals(
        expectedLaunchAngleDegrees,
        calibrationTelemetry.getEntry("LastRecorded/Derived/LaunchAngleDegrees").getDouble(0.0),
        1e-9);
    assertEquals(
        "test sample",
        calibrationTelemetry.getEntry("LastRecorded/Measurement/Notes").getString(""));
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
