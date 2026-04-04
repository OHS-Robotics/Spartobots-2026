package frc.robot.subsystems.gamepiece.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private static final double loopPeriodSeconds = 0.02;
  private static final double simClosedLoopDutyToVoltsScale = IntakeConstants.simNominalVoltage;
  private static final double pivotRotationsPerRadian =
      (IntakeConstants.defaultIntakePivotExtendedPositionRotations
              - IntakeConstants.defaultIntakePivotRetractedPositionRotations)
          / (IntakeConstants.simPivotExtendedAngleRadians
              - IntakeConstants.simPivotRetractedAngleRadians);

  private final FlywheelSim driveSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              IntakeConstants.simDriveGearbox,
              IntakeConstants.simDriveMoiKgMetersSq,
              IntakeConstants.simDriveReduction),
          IntakeConstants.simDriveGearbox);
  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          IntakeConstants.simPivotGearbox,
          IntakeConstants.simPivotReduction,
          IntakeConstants.simEstimatedPivotMoiKgMetersSq,
          IntakeConstants.simEstimatedPivotLengthMeters,
          IntakeConstants.simPivotMinAngleRadians,
          IntakeConstants.simPivotMaxAngleRadians,
          true,
          IntakeConstants.simPivotRetractedAngleRadians);

  private final PIDController driveController =
      new PIDController(
          IntakeConstants.intakeDriveVelocityKp,
          IntakeConstants.intakeDriveVelocityKi,
          IntakeConstants.intakeDriveVelocityKd);
  private final PIDController pivotController =
      new PIDController(
          IntakeConstants.intakePivotPositionKp,
          IntakeConstants.intakePivotPositionKi,
          IntakeConstants.intakePivotPositionKd);

  private boolean driveClosedLoopEnabled = false;
  private boolean pivotClosedLoopEnabled = false;
  private double driveVelocitySetpointRotationsPerSec = 0.0;
  private double pivotPositionSetpointRotations =
      IntakeConstants.defaultIntakePivotRetractedPositionRotations;
  private double driveVelocityKv = IntakeConstants.intakeDriveVelocityKv;
  private double driveAppliedOutput = 0.0;
  private double pivotAppliedOutput = 0.0;
  private double drivePositionRotations = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    double driveVelocityRotationsPerSec = driveSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double pivotPositionRotations = angleToPivotRotations(pivotSim.getAngleRads());
    double driveAppliedVolts =
        driveClosedLoopEnabled
            ? clampVoltage(
                (driveController.calculate(
                            driveVelocityRotationsPerSec, driveVelocitySetpointRotationsPerSec)
                        + (driveVelocityKv * driveVelocitySetpointRotationsPerSec))
                    * simClosedLoopDutyToVoltsScale)
            : clampVoltage(driveAppliedOutput * IntakeConstants.simNominalVoltage);
    double pivotAppliedVolts =
        pivotClosedLoopEnabled
            ? clampVoltage(
                pivotController.calculate(pivotPositionRotations, pivotPositionSetpointRotations)
                    * simClosedLoopDutyToVoltsScale)
            : clampVoltage(pivotAppliedOutput * IntakeConstants.simNominalVoltage);

    driveSim.setInputVoltage(driveAppliedVolts);
    pivotSim.setInputVoltage(pivotAppliedVolts);
    driveSim.update(loopPeriodSeconds);
    pivotSim.update(loopPeriodSeconds);

    drivePositionRotations +=
        driveSim.getAngularVelocityRadPerSec() * loopPeriodSeconds / (2.0 * Math.PI);

    inputs.driveConnected = true;
    inputs.pivotConnected = true;
    inputs.drivePositionRotations = drivePositionRotations;
    inputs.driveVelocityRotationsPerSec = driveSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.driveAppliedOutput = driveAppliedVolts / IntakeConstants.simNominalVoltage;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
    inputs.pivotPositionRotations = angleToPivotRotations(pivotSim.getAngleRads());
    inputs.pivotVelocityRpm = pivotSim.getVelocityRadPerSec() * pivotRotationsPerRadian * 60.0;
    inputs.pivotAppliedOutput = pivotAppliedVolts / IntakeConstants.simNominalVoltage;
    inputs.pivotCurrentAmps = Math.abs(pivotSim.getCurrentDrawAmps());
  }

  @Override
  public void setDriveOutput(double output) {
    driveClosedLoopEnabled = false;
    driveAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setDriveVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
    driveClosedLoopEnabled = true;
    driveVelocitySetpointRotationsPerSec = velocityRotationsPerSec;
    driveAppliedOutput = 0.0;
  }

  @Override
  public void setPivotOutput(double output) {
    pivotClosedLoopEnabled = false;
    pivotAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setPivotPositionSetpointRotations(double positionRotations) {
    pivotClosedLoopEnabled = true;
    pivotPositionSetpointRotations = positionRotations;
    pivotAppliedOutput = 0.0;
  }

  @Override
  public void setPivotEncoderPositionRotations(double positionRotations) {
    pivotClosedLoopEnabled = false;
    pivotPositionSetpointRotations = positionRotations;
    pivotAppliedOutput = 0.0;
    pivotSim.setState(pivotRotationsToAngle(positionRotations), 0.0);
  }

  @Override
  public void setDriveVelocityClosedLoopGains(double kp, double ki, double kd, double kv) {
    driveController.setPID(kp, ki, kd);
    driveVelocityKv = kv;
  }

  @Override
  public void setPivotPositionClosedLoopGains(double kp, double ki, double kd) {
    pivotController.setPID(kp, ki, kd);
  }

  @Override
  public void resetSimulationState() {
    driveClosedLoopEnabled = false;
    pivotClosedLoopEnabled = false;
    driveVelocitySetpointRotationsPerSec = 0.0;
    driveAppliedOutput = 0.0;
    pivotAppliedOutput = 0.0;
    drivePositionRotations = 0.0;
    driveSim.setState(VecBuilder.fill(0.0));
    pivotSim.setState(IntakeConstants.simPivotRetractedAngleRadians, 0.0);
  }

  private static double clampVoltage(double volts) {
    return MathUtil.clamp(
        volts, -IntakeConstants.simNominalVoltage, IntakeConstants.simNominalVoltage);
  }

  private static double angleToPivotRotations(double pivotAngleRadians) {
    return IntakeConstants.defaultIntakePivotRetractedPositionRotations
        + ((pivotAngleRadians - IntakeConstants.simPivotRetractedAngleRadians)
            * pivotRotationsPerRadian);
  }

  private static double pivotRotationsToAngle(double pivotPositionRotations) {
    return IntakeConstants.simPivotRetractedAngleRadians
        + ((pivotPositionRotations - IntakeConstants.defaultIntakePivotRetractedPositionRotations)
            / pivotRotationsPerRadian);
  }
}
