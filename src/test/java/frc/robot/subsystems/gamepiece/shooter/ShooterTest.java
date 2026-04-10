package frc.robot.subsystems.gamepiece.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.targeting.HubTargetingGeometry;
import frc.robot.util.NetworkTablesUtil;
import org.junit.jupiter.api.Test;

class ShooterTest {
  @Test
  void hubShotSolverReturnsFeasibleSolutionWithinEnvelope() {
    Shooter shooter = new Shooter(new FakeShooterIO());
    shooter.setCalibrationModeEnabled(false);
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
  void hoodCalibrationMapsRetractedStopToSteeperLaunchAngles() {
    var tuningTable =
        NetworkTablesUtil.tuningCommon(NetworkTablesUtil.domain(ShooterConstants.configTableName));
    tuningTable
        .getEntry("Hood/Calibration/RetractedPositionRotations")
        .setDouble(ShooterConstants.defaultHoodRetractedPositionRotations);
    tuningTable
        .getEntry("Hood/Calibration/ExtendedPositionRotations")
        .setDouble(ShooterConstants.defaultHoodExtendedPositionRotations);

    Shooter shooter = new Shooter(new FakeShooterIO());

    assertEquals(
        ShooterConstants.defaultHoodRetractedPositionRotations,
        shooter.hoodAngleToMotorRotations(ShooterConstants.maxLaunchAngle),
        1e-9);
    assertEquals(
        ShooterConstants.defaultHoodExtendedPositionRotations,
        shooter.hoodAngleToMotorRotations(ShooterConstants.minLaunchAngle),
        1e-9);
    assertEquals(
        ShooterConstants.maxLaunchAngle.getDegrees(),
        shooter
            .motorRotationsToHoodAngle(ShooterConstants.defaultHoodRetractedPositionRotations)
            .getDegrees(),
        1e-9);
    assertEquals(
        ShooterConstants.minLaunchAngle.getDegrees(),
        shooter
            .motorRotationsToHoodAngle(ShooterConstants.defaultHoodExtendedPositionRotations)
            .getDegrees(),
        1e-9);
  }

  @Test
  void readyToFireTracksMeasuredWheelReadiness() {
    FakeShooterIO io = new FakeShooterIO();
    Shooter shooter = new Shooter(io);
    shooter.setCalibrationModeEnabled(false);
    Pose2d hubPose = new Pose2d(5.0, 2.0, Rotation2d.kZero);
    Pose2d robotPose =
        new Pose2d(
            2.0,
            2.0,
            HubTargetingGeometry.getRobotRotationToAimAtHub(
                new Pose2d(2.0, 2.0, Rotation2d.kZero), hubPose));
    shooter.updateHubShotSolution(robotPose, hubPose);

    shooter.setShotControlEnabled(true);
    shooter.periodic();
    assertFalse(shooter.isReadyToFire());

    followShooterCommand(io, shooter, 100);
    assertTrue(shooter.isReadyToFire());

    io.pair1MeasuredVelocityRadPerSec = io.pair1SetpointRadPerSec * 0.5;
    io.pair2MeasuredVelocityRadPerSec = io.pair2SetpointRadPerSec * 0.5;
    shooter.periodic();
    assertFalse(shooter.isReadyToFire());
  }

  @Test
  void readyToFireRejectsShotWhenRobotHeadingWouldMissHub() {
    FakeShooterIO io = new FakeShooterIO();
    Shooter shooter = new Shooter(io);
    shooter.setCalibrationModeEnabled(false);
    Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.kZero);
    Pose2d hubPose = new Pose2d(5.0, 2.0, Rotation2d.kZero);
    Shooter.HubShotSolution solution = shooter.updateHubShotSolution(robotPose, hubPose);

    assertTrue(solution.feasible());

    shooter.setShotControlEnabled(true);
    shooter.periodic();
    followShooterCommand(io, shooter, 100);

    assertTrue(shooter.isSpinupComplete());
    assertFalse(shooter.isHubShotPredictedToScore());
    assertFalse(shooter.isReadyToFire());
  }

  @Test
  void readyToFireRejectsShotDuringRapidLateralAcceleration() {
    FakeShooterIO io = new FakeShooterIO();
    Shooter shooter = new Shooter(io);
    shooter.setCalibrationModeEnabled(false);
    Pose2d hubPose = new Pose2d(5.0, 2.0, Rotation2d.kZero);
    Pose2d robotPose =
        new Pose2d(
            2.0,
            2.0,
            HubTargetingGeometry.getRobotRotationToAimAtHub(
                new Pose2d(2.0, 2.0, Rotation2d.kZero), hubPose));

    shooter.updateHubShotSolution(robotPose, hubPose, new Translation2d());
    Shooter.HubShotSolution solution =
        shooter.updateHubShotSolution(robotPose, hubPose, new Translation2d(0.0, 0.10));

    assertTrue(solution.feasible());

    shooter.setShotControlEnabled(true);
    shooter.periodic();
    followShooterCommand(io, shooter, 100);

    assertTrue(shooter.isSpinupComplete());
    assertFalse(shooter.isHubShotPredictedToScore());
    assertFalse(shooter.isReadyToFire());
  }

  @Test
  void spinupCanCompleteEvenWhenShotSolutionIsInfeasible() {
    FakeShooterIO io = new FakeShooterIO();
    Shooter shooter = new Shooter(io);
    shooter.setCalibrationModeEnabled(false);
    Shooter.HubShotSolution solution =
        shooter.updateHubShotSolution(
            new Pose2d(2.0, 2.0, Rotation2d.kZero), new Pose2d(100.0, 2.0, Rotation2d.kZero));

    assertFalse(solution.feasible());

    shooter.setShotControlEnabled(true);
    shooter.periodic();
    followShooterCommand(io, shooter, 100);

    assertTrue(shooter.isSpinupComplete());
    assertFalse(shooter.isReadyToFire());
  }

  @Test
  void hubShotSolverCompensatesForRobotVelocityAndConverges() {
    Shooter shooter = new Shooter(new FakeShooterIO());
    shooter.setCalibrationModeEnabled(false);
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
  void hubShotSolverMeasuresDistanceFromLaunchOrigin() {
    Shooter shooter = new Shooter(new FakeShooterIO());
    shooter.setCalibrationModeEnabled(false);
    Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(180.0));
    Pose2d hubPose = new Pose2d(5.0, 2.0, Rotation2d.kZero);

    shooter.setCalibrationModeEnabled(false);
    Shooter.HubShotSolution solution = shooter.updateHubShotSolution(robotPose, hubPose);

    assertEquals(
        HubTargetingGeometry.getDistanceFromLaunchOriginToHub(robotPose, hubPose),
        solution.distanceMeters(),
        1e-9);
  }

  @Test
  void hubShotSolverPrefersDescendingEntryAtHub() {
    Shooter shooter = new Shooter(new FakeShooterIO());
    shooter.setCalibrationModeEnabled(false);
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
  void hubShotSolutionUsesSharedWheelSetpointForSingleDrumShooter() {
    Shooter shooter = new Shooter(new FakeShooterIO());
    shooter.setCalibrationModeEnabled(false);

    Shooter.HubShotSolution solution =
        shooter.updateHubShotSolution(
            new Pose2d(2.0, 2.0, Rotation2d.kZero), new Pose2d(6.0, 2.0, Rotation2d.kZero));

    assertTrue(solution.feasible());
    assertEquals(
        shooter.getPair1WheelSetpointRadPerSec(), shooter.getPair2WheelSetpointRadPerSec(), 1e-9);
  }

  @Test
  void shooterSpinupReachesFullWheelCommandWithinHalfSecond() {
    FakeShooterIO io = new FakeShooterIO();
    Shooter shooter = new Shooter(io);
    shooter.setCalibrationModeEnabled(false);
    shooter.updateHubShotSolution(
        new Pose2d(2.0, 2.0, Rotation2d.kZero), new Pose2d(6.0, 2.0, Rotation2d.kZero));

    shooter.setShotControlEnabled(true);
    shooter.periodic();

    assertTrue(io.pair1SetpointRadPerSec > 0.0);
    assertTrue(io.pair1SetpointRadPerSec < shooter.getPair1WheelSetpointRadPerSec());

    followShooterCommand(io, shooter, 19);

    assertEquals(shooter.getPair1WheelSetpointRadPerSec(), io.pair1SetpointRadPerSec, 1e-9);
    assertEquals(shooter.getPair2WheelSetpointRadPerSec(), io.pair2SetpointRadPerSec, 1e-9);
  }

  @Test
  void shotControlCanSkipWheelPowerSoftRampForAuto() {
    FakeShooterIO rampedIo = new FakeShooterIO();
    Shooter rampedShooter = new Shooter(rampedIo);
    rampedShooter.setCalibrationModeEnabled(false);
    rampedShooter.updateHubShotSolution(
        new Pose2d(2.0, 2.0, Rotation2d.kZero), new Pose2d(6.0, 2.0, Rotation2d.kZero));

    FakeShooterIO fullPowerIo = new FakeShooterIO();
    Shooter fullPowerShooter = new Shooter(fullPowerIo);
    fullPowerShooter.setCalibrationModeEnabled(false);
    fullPowerShooter.updateHubShotSolution(
        new Pose2d(2.0, 2.0, Rotation2d.kZero), new Pose2d(6.0, 2.0, Rotation2d.kZero));

    rampedShooter.setShotControlEnabled(true);
    fullPowerShooter.setShotControlEnabled(true, true);
    rampedShooter.periodic();
    fullPowerShooter.periodic();

    assertTrue(fullPowerIo.pair1SetpointRadPerSec > rampedIo.pair1SetpointRadPerSec);
    assertTrue(fullPowerIo.pair2SetpointRadPerSec > rampedIo.pair2SetpointRadPerSec);
  }

  @Test
  void simulatedShotTriggersDuringAutoFeedWindow() {
    Shooter shooter = new Shooter(new ShooterIOSim());
    shooter.setCalibrationModeEnabled(false);
    Pose2d hubPose = new Pose2d(5.0, 2.0, Rotation2d.kZero);
    Pose2d robotPose =
        new Pose2d(
            2.0,
            2.0,
            HubTargetingGeometry.getRobotRotationToAimAtHub(
                new Pose2d(2.0, 2.0, Rotation2d.kZero), hubPose));
    shooter.updateHubShotSolution(robotPose, hubPose);
    shooter.setShotControlEnabled(true, true);

    boolean shotTriggered = false;
    for (int cycle = 0; cycle < 175; cycle++) {
      shooter.periodic();
      double timestampSeconds = cycle * 0.02;
      double feedRateRatio = timestampSeconds >= 0.5 ? 1.0 : 0.0;
      shotTriggered |= shooter.shouldTriggerSimulatedShot(timestampSeconds, feedRateRatio);
    }

    assertTrue(
        shotTriggered,
        () ->
            "expected simulated shot during feed window; spinupComplete="
                + shooter.isSpinupComplete()
                + ", launchSpeed="
                + shooter.getEstimatedLaunchSpeedFromMeasuredWheelsMetersPerSec()
                + ", pair1Command="
                + shooter.getPair1WheelSetpointRadPerSec()
                + ", pair2Command="
                + shooter.getPair2WheelSetpointRadPerSec());
  }

  @Test
  void shooterSpinDownRampsWheelCommandAfterDemandEnds() {
    FakeShooterIO io = new FakeShooterIO();
    Shooter shooter = new Shooter(io);
    shooter.setCalibrationModeEnabled(false);
    shooter.updateHubShotSolution(
        new Pose2d(2.0, 2.0, Rotation2d.kZero), new Pose2d(6.0, 2.0, Rotation2d.kZero));

    shooter.setShotControlEnabled(true);
    shooter.periodic();
    followShooterCommand(io, shooter, 100);

    double enabledWheelCommandRadPerSec = io.pair1SetpointRadPerSec;
    assertTrue(enabledWheelCommandRadPerSec > 0.0);

    shooter.setShotControlEnabled(false);
    followShooterCommand(io, shooter, 50);
    assertTrue(io.pair1SetpointRadPerSec > (enabledWheelCommandRadPerSec * 0.78));

    followShooterCommand(io, shooter, 250);
    assertEquals(0.0, io.pair1SetpointRadPerSec, 1e-9);
    assertEquals(0.0, io.pair2SetpointRadPerSec, 1e-9);
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
    assertEquals(130.0, shooter.getPair1WheelSetpointRadPerSec(), 1e-9);
    assertEquals(130.0, shooter.getPair2WheelSetpointRadPerSec(), 1e-9);
    assertEquals(130.0 * ShooterConstants.defaultPair1Direction, io.pair1SetpointRadPerSec, 1e-9);
    assertEquals(130.0 * ShooterConstants.defaultPair2Direction, io.pair2SetpointRadPerSec, 1e-9);
    assertTrue(shooter.isCalibrationModeEnabled());
  }

  @Test
  void trenchSafetyOverrideRetractsHoodWithoutDiscardingCommandedSetpoint() {
    FakeShooterIO io = new FakeShooterIO();
    Shooter shooter = new Shooter(io);

    shooter.adjustHoodSetpointRotations(1.0);
    double commandedHoodSetpointRotations = shooter.getHoodSetpointMotorRotations();

    shooter.setTrenchSafetyRetractOverrideEnabled(true);
    shooter.periodic();

    assertEquals(commandedHoodSetpointRotations, shooter.getHoodSetpointMotorRotations(), 1e-9);
    assertEquals(
        ShooterConstants.defaultHoodRetractedPositionRotations, io.hoodSetpointRotations, 1e-9);

    shooter.setTrenchSafetyRetractOverrideEnabled(false);
    shooter.periodic();

    assertEquals(commandedHoodSetpointRotations, io.hoodSetpointRotations, 1e-9);
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

  private static void followShooterCommand(FakeShooterIO io, Shooter shooter, int cycles) {
    for (int i = 0; i < cycles; i++) {
      io.pair1MeasuredVelocityRadPerSec = io.pair1SetpointRadPerSec;
      io.pair2MeasuredVelocityRadPerSec = io.pair2SetpointRadPerSec;
      shooter.periodic();
    }
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
