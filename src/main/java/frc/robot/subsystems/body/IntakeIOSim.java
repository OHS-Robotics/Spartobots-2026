package frc.robot.subsystems.body;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private static final double loopPeriodSeconds = 0.02;
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

  private double driveAppliedOutput = 0.0;
  private double pivotAppliedOutput = 0.0;
  private double drivePositionRotations = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    driveSim.setInputVoltage(driveAppliedOutput * IntakeConstants.simNominalVoltage);
    pivotSim.setInputVoltage(pivotAppliedOutput * IntakeConstants.simNominalVoltage);
    driveSim.update(loopPeriodSeconds);
    pivotSim.update(loopPeriodSeconds);

    drivePositionRotations +=
        driveSim.getAngularVelocityRadPerSec() * loopPeriodSeconds / (2.0 * Math.PI);

    inputs.driveConnected = true;
    inputs.pivotConnected = true;
    inputs.drivePositionRotations = drivePositionRotations;
    inputs.driveVelocityRotationsPerSec = driveSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.driveAppliedOutput = driveAppliedOutput;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
    inputs.pivotPositionRotations =
        IntakeConstants.defaultIntakePivotRetractedPositionRotations
            + ((pivotSim.getAngleRads() - IntakeConstants.simPivotRetractedAngleRadians)
                * pivotRotationsPerRadian);
    inputs.pivotVelocityRpm = pivotSim.getVelocityRadPerSec() * pivotRotationsPerRadian * 60.0;
    inputs.pivotAppliedOutput = pivotAppliedOutput;
    inputs.pivotCurrentAmps = Math.abs(pivotSim.getCurrentDrawAmps());
  }

  @Override
  public void setDriveOutput(double output) {
    driveAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setPivotOutput(double output) {
    pivotAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void resetSimulationState() {
    driveAppliedOutput = 0.0;
    pivotAppliedOutput = 0.0;
    drivePositionRotations = 0.0;
    driveSim.setState(VecBuilder.fill(0.0));
    pivotSim.setState(IntakeConstants.simPivotRetractedAngleRadians, 0.0);
  }
}
