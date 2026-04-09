package frc.robot.superstructure.gamepiece;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gamepiece.indexers.Indexers;
import frc.robot.subsystems.gamepiece.indexers.IndexersConstants;
import frc.robot.subsystems.gamepiece.indexers.IndexersIO;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.intake.IntakeConstants;
import frc.robot.subsystems.gamepiece.intake.IntakeIO;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.subsystems.gamepiece.shooter.ShooterIO;
import frc.robot.util.NetworkTablesUtil;
import org.junit.jupiter.api.Test;

class GamePieceCoordinatorTest {
  @Test
  void basicFeedInterlockStopsIndexers() {
    configureDefaultMechanismDirections();
    FakeIndexersIO indexersIO = new FakeIndexersIO();
    Shooter shooter = new Shooter(new ShooterIO() {});
    shooter.setCalibrationModeEnabled(false);
    GamePieceCoordinator coordinator =
        new GamePieceCoordinator(new Intake(new IntakeIO() {}), new Indexers(indexersIO), shooter);

    coordinator.applyBasicFeed(true);

    assertEquals(0.0, indexersIO.topOutput, 1e-9);
    assertEquals(0.0, indexersIO.bottomOutput, 1e-9);
  }

  @Test
  void basicCollectRunsIntakeRollersBeforePivotCalibrationWhileIndexing() {
    configureDefaultMechanismDirections();
    FakeIntakeIO intakeIO = new FakeIntakeIO();
    Intake intake = new Intake(intakeIO);
    intake.periodic();
    FakeIndexersIO indexersIO = new FakeIndexersIO();
    Shooter shooter = new Shooter(new ShooterIO() {});
    shooter.setCalibrationModeEnabled(false);
    GamePieceCoordinator coordinator =
        new GamePieceCoordinator(intake, new Indexers(indexersIO), shooter);

    coordinator.applyBasicCollect(true);

    assertEquals(0.65, Math.abs(intakeIO.driveOutput), 1e-9);
    assertEquals(intake.getIntakePivotIntakingPositionRotations(), intakeIO.pivotSetpoint, 1e-9);
    assertEquals(0.0, intakeIO.pivotOutput, 1e-9);
    assertEquals(0.55, indexersIO.topOutput, 1e-9);
    assertEquals(0.0, indexersIO.bottomOutput, 1e-9);

    intakeIO.pivotPositionRotations = intake.getIntakePivotIntakingPositionRotations();
    intake.periodic();
    coordinator.applyBasicCollect(true);

    assertEquals(0.65, Math.abs(intakeIO.driveOutput), 1e-9);
  }

  @Test
  void basicCollectCommandRunsIntakeRollersBeforePivotCalibrationInTeleop() {
    configureDefaultMechanismDirections();
    FakeIntakeIO intakeIO = new FakeIntakeIO();
    Intake intake = new Intake(intakeIO);
    intake.periodic();
    FakeIndexersIO indexersIO = new FakeIndexersIO();
    Shooter shooter = new Shooter(new ShooterIO() {});
    shooter.setCalibrationModeEnabled(false);
    GamePieceCoordinator coordinator =
        new GamePieceCoordinator(intake, new Indexers(indexersIO), shooter);
    Command command = coordinator.basicCollectWhileHeldCommand(false);

    command.initialize();
    command.execute();

    assertEquals(0.65, Math.abs(intakeIO.driveOutput), 1e-9);
    assertEquals(intake.getIntakePivotIntakingPositionRotations(), intakeIO.pivotSetpoint, 1e-9);
    assertEquals(0.0, intakeIO.pivotOutput, 1e-9);
    assertEquals(0.0, indexersIO.topOutput, 1e-9);
    assertEquals(0.0, indexersIO.bottomOutput, 1e-9);

    intakeIO.pivotPositionRotations = intake.getIntakePivotIntakingPositionRotations();
    intake.periodic();
    command.execute();

    assertEquals(0.65, Math.abs(intakeIO.driveOutput), 1e-9);

    command.end(false);
  }

  @Test
  void manualFeedRunsIndexersWhileAutoAimAssistIsActive() {
    configureDefaultMechanismDirections();
    FakeIntakeIO intakeIO = new FakeIntakeIO();
    Intake intake = new Intake(intakeIO);
    intake.periodic();
    FakeIndexersIO indexersIO = new FakeIndexersIO();
    Shooter shooter = new Shooter(new ShooterIO() {});
    shooter.setCalibrationModeEnabled(false);
    GamePieceCoordinator coordinator =
        new GamePieceCoordinator(intake, new Indexers(indexersIO), shooter);

    coordinator.setShooterDemandFromAlign(true);
    Command command = coordinator.runManualFeedAndIndexersWhileHeldCommand(() -> 1.0);

    command.initialize();
    command.execute();
    command.execute();
    command.execute();

    assertEquals(-1.0, indexersIO.topOutput, 1e-9);
    assertEquals(1.0, indexersIO.bottomOutput, 1e-9);
    assertEquals(
        getExpectedManualFeedSweepSetpoint(intake, 0.5, 1.0), intakeIO.pivotSetpoint, 1e-9);
    assertEquals(0.0, intakeIO.pivotOutput, 1e-9);

    command.end(false);
  }

  @Test
  void manualFeedRunsSharedIndexerBeltAndStartsIntakeSweepOnTriggerPress() {
    configureDefaultMechanismDirections();
    FakeIntakeIO intakeIO = new FakeIntakeIO();
    Intake intake = new Intake(intakeIO);
    intake.periodic();
    FakeIndexersIO indexersIO = new FakeIndexersIO();
    Shooter shooter = new Shooter(new ShooterIO() {});
    shooter.setCalibrationModeEnabled(false);
    GamePieceCoordinator coordinator =
        new GamePieceCoordinator(intake, new Indexers(indexersIO), shooter);

    Command command = coordinator.runManualFeedAndIndexersWhileHeldCommand(() -> 1.0);

    command.initialize();
    command.execute();
    command.execute();
    command.execute();

    assertEquals(-1.0, indexersIO.topOutput, 1e-9);
    assertEquals(1.0, indexersIO.bottomOutput, 1e-9);
    assertEquals(
        getExpectedManualFeedSweepSetpoint(intake, 0.5, 1.0), intakeIO.pivotSetpoint, 1e-9);
    assertEquals(0.0, intakeIO.pivotOutput, 1e-9);

    command.end(false);
  }

  @Test
  void manualFeedScalesIntakeSweepRateWithTrigger() {
    configureDefaultMechanismDirections();
    FakeIntakeIO intakeIO = new FakeIntakeIO();
    Intake intake = new Intake(intakeIO);
    intake.periodic();
    FakeIndexersIO indexersIO = new FakeIndexersIO();
    Shooter shooter = new Shooter(new ShooterIO() {});
    shooter.setCalibrationModeEnabled(false);
    GamePieceCoordinator coordinator =
        new GamePieceCoordinator(intake, new Indexers(indexersIO), shooter);

    double triggerValue = 0.5;
    double expectedThrottleScale = MathUtil.applyDeadband(triggerValue, 0.02);
    Command command = coordinator.runManualFeedAndIndexersWhileHeldCommand(() -> triggerValue);

    command.initialize();
    command.execute();
    command.execute();
    command.execute();

    assertEquals(
        getExpectedManualFeedSweepSetpoint(intake, 0.5, expectedThrottleScale),
        intakeIO.pivotSetpoint,
        1e-9);
    assertEquals(0.0, intakeIO.pivotOutput, 1e-9);
    assertEquals(-expectedThrottleScale, indexersIO.topOutput, 1e-9);
    assertEquals(expectedThrottleScale, indexersIO.bottomOutput, 1e-9);

    command.end(false);
  }

  @Test
  void manualFeedPulsesIntakeDriveAtExtendedLimit() {
    configureDefaultMechanismDirections();
    FakeIntakeIO intakeIO = new FakeIntakeIO();
    Intake intake = new Intake(intakeIO);
    intake.periodic();
    double manualFeedLowerLimit = getManualFeedSweepLowerLimit(intake);
    double manualFeedExtendedLimit = getSweepExtendedLimit(intake);
    intakeIO.pivotPositionRotations = manualFeedExtendedLimit;
    intake.periodic();
    FakeIndexersIO indexersIO = new FakeIndexersIO();
    Shooter shooter = new Shooter(new ShooterIO() {});
    shooter.setCalibrationModeEnabled(false);
    GamePieceCoordinator coordinator =
        new GamePieceCoordinator(intake, new Indexers(indexersIO), shooter);

    Command command = coordinator.runManualFeedAndIndexersWhileHeldCommand(() -> 1.0);

    command.initialize();
    command.execute();
    command.execute();
    command.execute();

    assertTrue(Double.isFinite(intakeIO.pivotSetpoint));
    assertTrue(intakeIO.pivotSetpoint >= manualFeedLowerLimit - 1e-9);
    assertTrue(intakeIO.pivotSetpoint <= manualFeedExtendedLimit + 1e-9);
    assertTrue(Math.abs(intakeIO.driveOutput) > 1e-9);
    assertEquals(-1.0, indexersIO.topOutput, 1e-9);
    assertEquals(1.0, indexersIO.bottomOutput, 1e-9);

    command.end(false);
  }

  @Test
  void manualFeedDoesNotCommandPastCalibratedSweepLimit() {
    configureDefaultMechanismDirections();
    FakeIntakeIO intakeIO = new FakeIntakeIO();
    intakeIO.pivotPositionRotations = 0.995;
    Intake intake = new Intake(intakeIO);
    intake.periodic();
    FakeIndexersIO indexersIO = new FakeIndexersIO();
    Shooter shooter = new Shooter(new ShooterIO() {});
    shooter.setCalibrationModeEnabled(false);
    GamePieceCoordinator coordinator =
        new GamePieceCoordinator(intake, new Indexers(indexersIO), shooter);

    Command command = coordinator.runManualFeedAndIndexersWhileHeldCommand(() -> 1.0);

    command.initialize();
    command.execute();
    command.execute();

    intakeIO.pivotPositionRotations = getSweepExtendedLimit(intake);
    intake.periodic();
    command.execute();

    assertTrue(
        Double.isNaN(intakeIO.pivotSetpoint)
            || intakeIO.pivotSetpoint <= getSweepExtendedLimit(intake));

    command.end(false);
  }

  private static double getExpectedManualFeedSweepSetpoint(
      Intake intake, double startingPositionRotations, double throttleScale) {
    double sweepRetractedLimit = getManualFeedSweepLowerLimit(intake);
    double sweepExtendedLimit = getSweepExtendedLimit(intake);
    double sweepMidpoint = 0.5 * (sweepRetractedLimit + sweepExtendedLimit);
    double sweepHalfTravel = 0.5 * (sweepExtendedLimit - sweepRetractedLimit);
    double clampedStartingPosition =
        MathUtil.clamp(
            startingPositionRotations,
            Math.min(sweepRetractedLimit, sweepExtendedLimit),
            Math.max(sweepRetractedLimit, sweepExtendedLimit));
    double initialSweepPhase =
        Math.asin(
            MathUtil.clamp((clampedStartingPosition - sweepMidpoint) / sweepHalfTravel, -1.0, 1.0));
    double sweepPhaseStep =
        (Math.PI * throttleScale * 0.02)
            / IntakeConstants.intakePivotSweepTraversalSecondsAtFullTrigger;
    return MathUtil.clamp(
        sweepMidpoint + (sweepHalfTravel * Math.sin(initialSweepPhase + sweepPhaseStep)),
        Math.min(sweepRetractedLimit, sweepExtendedLimit),
        Math.max(sweepRetractedLimit, sweepExtendedLimit));
  }

  private static double getManualFeedSweepLowerLimit(Intake intake) {
    return MathUtil.interpolate(
        intake.getIntakePivotRetractedPositionRotations(),
        intake.getIntakePivotExtendedPositionRotations(),
        IntakeConstants.intakePivotManualFeedLowerLimitNormalized);
  }

  private static double getSweepRetractedLimit(Intake intake) {
    return MathUtil.interpolate(
        intake.getIntakePivotRetractedPositionRotations(),
        intake.getIntakePivotExtendedPositionRotations(),
        MathUtil.clamp(IntakeConstants.intakePivotSweepHardStopInsetNormalized, 0.0, 0.5));
  }

  private static double getSweepExtendedLimit(Intake intake) {
    return MathUtil.interpolate(
        intake.getIntakePivotRetractedPositionRotations(),
        intake.getIntakePivotExtendedPositionRotations(),
        1.0 - MathUtil.clamp(IntakeConstants.intakePivotSweepHardStopInsetNormalized, 0.0, 0.5));
  }

  private static void configureDefaultMechanismDirections() {
    var intakeTuningTable =
        NetworkTablesUtil.tuningCommon(NetworkTablesUtil.domain(IntakeConstants.configTableName));
    intakeTuningTable
        .getEntry("Drive/Direction")
        .setDouble(IntakeConstants.defaultIntakeDriveDirection);
    intakeTuningTable
        .getEntry("Pivot/SpeedScale")
        .setDouble(IntakeConstants.defaultIntakePivotSpeedScale);
    intakeTuningTable
        .getEntry("Pivot/Inverted")
        .setBoolean(IntakeConstants.defaultIntakePivotInverted);
    intakeTuningTable
        .getEntry("Pivot/Calibration/RetractedPositionRotations")
        .setDouble(IntakeConstants.defaultIntakePivotRetractedPositionRotations);
    intakeTuningTable
        .getEntry("Pivot/Calibration/ExtendedPositionRotations")
        .setDouble(IntakeConstants.defaultIntakePivotExtendedPositionRotations);
    intakeTuningTable.getSubTable("Calibration").getEntry("Enabled").setBoolean(false);

    var indexerTuningTable =
        NetworkTablesUtil.tuningCommon(NetworkTablesUtil.domain(IndexersConstants.configTableName));
    indexerTuningTable
        .getEntry("Top/Direction")
        .setDouble(IndexersConstants.defaultTopIndexerDirection);
    indexerTuningTable
        .getEntry("Bottom/Direction")
        .setDouble(IndexersConstants.defaultBottomIndexerDirection);
    indexerTuningTable.getSubTable("Calibration").getEntry("Enabled").setBoolean(false);
  }

  private static class FakeIntakeIO implements IntakeIO {
    double pivotPositionRotations = 0.5;
    double driveOutput = 0.0;
    double pivotOutput = 0.0;
    double pivotSetpoint = Double.NaN;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
      inputs.driveConnected = true;
      inputs.pivotConnected = true;
      inputs.pivotPositionRotations = pivotPositionRotations;
    }

    @Override
    public void setDriveOutput(double output) {
      driveOutput = output;
    }

    @Override
    public void setPivotOutput(double output) {
      pivotOutput = output;
    }

    @Override
    public void setPivotPositionSetpointRotations(double positionRotations) {
      pivotSetpoint = positionRotations;
    }
  }

  private static class FakeIndexersIO implements IndexersIO {
    double topOutput = 0.0;
    double bottomOutput = 0.0;

    @Override
    public void setTopOutput(double output) {
      topOutput = output;
    }

    @Override
    public void setBottomOutput(double output) {
      bottomOutput = output;
    }
  }
}
