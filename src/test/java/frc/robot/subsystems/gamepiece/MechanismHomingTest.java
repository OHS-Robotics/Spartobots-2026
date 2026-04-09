package frc.robot.subsystems.gamepiece;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.gamepiece.intake.Intake;
import frc.robot.subsystems.gamepiece.intake.IntakeConstants;
import frc.robot.subsystems.gamepiece.intake.IntakeIO;
import frc.robot.subsystems.gamepiece.shooter.Shooter;
import frc.robot.subsystems.gamepiece.shooter.ShooterConstants;
import frc.robot.subsystems.gamepiece.shooter.ShooterIO;
import frc.robot.util.NetworkTablesUtil;
import org.junit.jupiter.api.Test;

class MechanismHomingTest {
  @Test
  void intakeCalibrationMeasuresExtendedStopThenZerosRetractedStop() {
    var tuningTable =
        NetworkTablesUtil.tuningCommon(NetworkTablesUtil.domain(IntakeConstants.configTableName));
    tuningTable
        .getEntry("Pivot/Calibration/RetractedPositionRotations")
        .setDouble(IntakeConstants.defaultIntakePivotRetractedPositionRotations);
    tuningTable
        .getEntry("Pivot/Calibration/ExtendedPositionRotations")
        .setDouble(IntakeConstants.defaultIntakePivotExtendedPositionRotations);
    tuningTable.getSubTable("Calibration").getEntry("Enabled").setBoolean(false);

    FakeIntakeHomingIO io = new FakeIntakeHomingIO();
    Intake intake = new Intake(io);

    runCommandUntilFinished(
        intake.calibrateIntakePivotToHardStopsCommand(), intake::periodic, io::advanceCalibration);
    intake.periodic();

    assertTrue(intake.isIntakePivotCalibrated());
    assertTrue(intake.didLastIntakePivotCalibrationSucceed());
    assertEquals(
        IntakeConstants.intakePivotRetractedHardStopReferenceRotations,
        io.lastEncoderPositionSetRotations,
        1e-9);
    assertEquals(
        IntakeConstants.intakePivotRetractedHardStopReferenceRotations,
        intake.getIntakePivotRetractedPositionRotations(),
        1e-9);
    assertEquals(
        io.extendedHardStopAfterReferenceRotations,
        intake.getIntakePivotExtendedPositionRotations(),
        1e-9);
    assertEquals(
        IntakeConstants.intakePivotRetractedHardStopReferenceRotations,
        tuningTable.getEntry("Pivot/Calibration/RetractedPositionRotations").getDouble(Double.NaN),
        1e-9);
    assertEquals(
        io.extendedHardStopAfterReferenceRotations,
        tuningTable.getEntry("Pivot/Calibration/ExtendedPositionRotations").getDouble(Double.NaN),
        1e-9);
  }

  @Test
  void hoodHomingZerosRetractedStopThenMeasuresExtendedStop() {
    FakeShooterHomingIO io = new FakeShooterHomingIO();
    Shooter shooter = new Shooter(io);

    runCommandUntilFinished(
        shooter.homeHoodToHardStopCommand(), shooter::periodic, io::advanceHoming);

    var tuningTable =
        NetworkTablesUtil.tuningCommon(NetworkTablesUtil.domain(ShooterConstants.configTableName));

    assertTrue(shooter.isHoodHomed());
    assertTrue(shooter.didLastHoodHomingSucceed());
    assertTrue(
        io.relaxedLoopsBeforeRetractSweep > 0,
        "Observed relaxed loops before retract sweep: " + io.relaxedLoopsBeforeRetractSweep);
    assertEquals(
        ShooterConstants.hoodRetractedHardStopReferenceRotations,
        io.lastEncoderPositionSetRotations,
        1e-9);
    assertEquals(
        ShooterConstants.hoodRetractedHardStopReferenceRotations,
        tuningTable.getEntry("Hood/Calibration/RetractedPositionRotations").getDouble(Double.NaN),
        1e-9);
    assertEquals(
        io.extendedHardStopAfterReferenceRotations,
        tuningTable.getEntry("Hood/Calibration/ExtendedPositionRotations").getDouble(Double.NaN),
        1e-9);
    assertEquals(
        io.extendedHardStopAfterReferenceRotations, shooter.getHoodSetpointMotorRotations(), 1e-9);
  }

  @Test
  void intakeManualPivotJogRespectsCalibratedHardStops() {
    FakeIntakeHomingIO io = new FakeIntakeHomingIO();
    Intake intake = new Intake(io);
    var tuningTable =
        NetworkTablesUtil.tuningCommon(NetworkTablesUtil.domain(IntakeConstants.configTableName));
    tuningTable
        .getEntry("Pivot/SpeedScale")
        .setDouble(IntakeConstants.defaultIntakePivotSpeedScale);
    tuningTable.getEntry("Pivot/Inverted").setBoolean(IntakeConstants.defaultIntakePivotInverted);
    tuningTable.getSubTable("Calibration").getEntry("Enabled").setBoolean(false);

    runCommandUntilFinished(
        intake.calibrateIntakePivotToHardStopsCommand(), intake::periodic, io::advanceCalibration);

    io.pivotPositionRotations = intake.getIntakePivotRetractedPositionRotations();
    intake.periodic();
    intake.setIntakePivotSpeed(-0.25);
    assertEquals(0.0, io.pivotAppliedOutput, 1e-9);

    intake.setIntakePivotSpeed(0.25);
    assertEquals(0.25, io.pivotAppliedOutput, 1e-9);

    io.pivotPositionRotations = intake.getIntakePivotExtendedPositionRotations();
    intake.periodic();
    intake.setIntakePivotSpeed(0.25);
    assertEquals(0.0, io.pivotAppliedOutput, 1e-9);

    intake.setIntakePivotSpeed(-0.25);
    assertEquals(-0.25, io.pivotAppliedOutput, 1e-9);
  }

  @Test
  void intakeManualPivotJogIgnoresCalibrationLimitsBeforeHoming() {
    FakeIntakeHomingIO io = new FakeIntakeHomingIO();
    Intake intake = new Intake(io);
    var tuningTable =
        NetworkTablesUtil.tuningCommon(NetworkTablesUtil.domain(IntakeConstants.configTableName));
    tuningTable
        .getEntry("Pivot/SpeedScale")
        .setDouble(IntakeConstants.defaultIntakePivotSpeedScale);
    tuningTable.getEntry("Pivot/Inverted").setBoolean(IntakeConstants.defaultIntakePivotInverted);
    tuningTable.getSubTable("Calibration").getEntry("Enabled").setBoolean(false);

    io.pivotPositionRotations = IntakeConstants.defaultIntakePivotExtendedPositionRotations + 5.0;
    intake.periodic();
    intake.setIntakePivotSpeed(0.25);
    assertEquals(0.25, io.pivotAppliedOutput, 1e-9);

    intake.setIntakePivotSpeed(-0.25);
    assertEquals(-0.25, io.pivotAppliedOutput, 1e-9);
  }

  private static void runCommandUntilFinished(
      Command command, Runnable periodic, Runnable advanceSimulation) {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    scheduler.schedule(command);

    for (int i = 0; i < 300 && scheduler.isScheduled(command); i++) {
      scheduler.run();
      periodic.run();
      advanceSimulation.run();
    }

    assertFalse(
        scheduler.isScheduled(command), "Command did not finish in the allotted iterations.");
    scheduler.cancelAll();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  private static class FakeIntakeHomingIO implements IntakeIO {
    private static final double moveStepRotations = 0.28;
    private static final double movingCurrentAmps = 3.0;
    private static final double stalledCurrentAmps = 10.0;

    double pivotPositionRotations = 0.65;
    double pivotVelocityRpm = 0.0;
    double pivotAppliedOutput = 0.0;
    double pivotCurrentAmps = 0.0;
    double lastEncoderPositionSetRotations = Double.NaN;
    final double retractedHardStopBeforeReferenceRotations = -0.42;
    final double extendedHardStopBeforeReferenceRotations =
        retractedHardStopBeforeReferenceRotations
            + IntakeConstants.intakePivotRetractedHardStopReferenceRotations
            + 2.63;
    final double extendedHardStopAfterReferenceRotations = 2.63;
    private boolean retractedReferenceEstablished = false;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
      inputs.driveConnected = true;
      inputs.pivotConnected = true;
      inputs.pivotPositionRotations = pivotPositionRotations;
      inputs.pivotVelocityRpm = pivotVelocityRpm;
      inputs.pivotAppliedOutput = pivotAppliedOutput;
      inputs.pivotCurrentAmps = pivotCurrentAmps;
    }

    @Override
    public void setPivotOutput(double output) {
      pivotAppliedOutput = output;
    }

    @Override
    public void setPivotEncoderPositionRotations(double positionRotations) {
      retractedReferenceEstablished = true;
      pivotPositionRotations = positionRotations;
      pivotVelocityRpm = 0.0;
      pivotCurrentAmps = 0.0;
      lastEncoderPositionSetRotations = positionRotations;
    }

    void advanceCalibration() {
      if (pivotAppliedOutput > 1e-6) {
        double target =
            retractedReferenceEstablished
                ? extendedHardStopAfterReferenceRotations
                : extendedHardStopBeforeReferenceRotations;
        advancePivotToward(target);
        return;
      }

      if (pivotAppliedOutput < -1e-6) {
        double target =
            retractedReferenceEstablished
                ? IntakeConstants.intakePivotRetractedHardStopReferenceRotations
                : retractedHardStopBeforeReferenceRotations;
        advancePivotToward(target);
        return;
      }

      pivotVelocityRpm = 0.0;
      pivotCurrentAmps = 0.0;
    }

    private void advancePivotToward(double targetRotations) {
      double deltaRotations = targetRotations - pivotPositionRotations;
      if (Math.abs(deltaRotations) <= moveStepRotations) {
        pivotPositionRotations = targetRotations;
        pivotVelocityRpm = 0.0;
        pivotCurrentAmps = stalledCurrentAmps;
        return;
      }

      double stepRotations = Math.copySign(moveStepRotations, deltaRotations);
      pivotPositionRotations += stepRotations;
      pivotVelocityRpm = (stepRotations / 0.02) * 60.0;
      pivotCurrentAmps = movingCurrentAmps;
    }
  }

  private static class FakeShooterHomingIO implements ShooterIO {
    private static final double moveStepRotations = 0.32;
    private static final double movingCurrentAmps = 6.0;
    private static final double stalledCurrentAmps = 22.0;

    double hoodPositionRotations = 4.8;
    double hoodVelocityRotationsPerSec = 0.0;
    double hoodCurrentAmps = 0.0;
    double hoodOpenLoopOutput = 0.0;
    double hoodSetpointRotations = ShooterConstants.defaultHoodRetractedPositionRotations;
    double lastEncoderPositionSetRotations = Double.NaN;
    final double retractedHardStopBeforeReferenceRotations = -1.75;
    final double extendedHardStopAfterReferenceRotations = 15.48;
    int relaxedLoopsBeforeRetractSweep = 0;
    private boolean retractedReferenceEstablished = false;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      inputs.pair1Connected = true;
      inputs.pair2Connected = true;
      inputs.hoodConnected = true;
      inputs.hoodPositionRotations = hoodPositionRotations;
      inputs.hoodVelocityRotationsPerSec = hoodVelocityRotationsPerSec;
      inputs.hoodCurrentAmps = hoodCurrentAmps;
    }

    @Override
    public void setHoodPositionSetpointRotations(double hoodPositionRotations) {
      hoodSetpointRotations = hoodPositionRotations;
    }

    @Override
    public void setHoodOpenLoopOutput(double output) {
      hoodOpenLoopOutput = output;
    }

    @Override
    public void setHoodEncoderPositionRotations(double hoodPositionRotations) {
      retractedReferenceEstablished = true;
      this.hoodPositionRotations = hoodPositionRotations;
      hoodVelocityRotationsPerSec = 0.0;
      hoodCurrentAmps = 0.0;
      lastEncoderPositionSetRotations = hoodPositionRotations;
    }

    void advanceHoming() {
      if (!retractedReferenceEstablished && Math.abs(hoodOpenLoopOutput) <= 1e-6) {
        relaxedLoopsBeforeRetractSweep++;
      }

      if (hoodOpenLoopOutput < -1e-6) {
        double target =
            retractedReferenceEstablished
                ? ShooterConstants.hoodRetractedHardStopReferenceRotations
                : retractedHardStopBeforeReferenceRotations;
        advanceHoodToward(target);
        return;
      }

      if (hoodOpenLoopOutput > 1e-6) {
        double target =
            retractedReferenceEstablished
                ? extendedHardStopAfterReferenceRotations
                : hoodPositionRotations + moveStepRotations;
        advanceHoodToward(target);
        return;
      }

      hoodVelocityRotationsPerSec = 0.0;
      hoodCurrentAmps = 0.0;
    }

    private void advanceHoodToward(double targetRotations) {
      double deltaRotations = targetRotations - hoodPositionRotations;
      if (Math.abs(deltaRotations) <= moveStepRotations) {
        hoodPositionRotations = targetRotations;
        hoodVelocityRotationsPerSec = 0.0;
        hoodCurrentAmps = stalledCurrentAmps;
        return;
      }

      double stepRotations = Math.copySign(moveStepRotations, deltaRotations);
      hoodPositionRotations += stepRotations;
      hoodVelocityRotationsPerSec = stepRotations / 0.02;
      hoodCurrentAmps = movingCurrentAmps;
    }
  }
}
