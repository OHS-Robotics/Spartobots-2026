package frc.robot.subsystems.gamepiece.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HopperIOSim implements HopperIO {
  private static final double loopPeriodSeconds = 0.02;
  private static final double simClosedLoopDutyToVoltsScale = HopperConstants.simNominalVoltage;
  private static final double extensionRotationsPerMeter =
      (HopperConstants.defaultHopperExtensionExtendedPositionRotations
              - HopperConstants.defaultHopperExtensionRetractedPositionRotations)
          / (HopperConstants.simExtensionExtendedHeightMeters
              - HopperConstants.simExtensionRetractedHeightMeters);

  private final FlywheelSim agitatorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              HopperConstants.simAgitatorGearbox,
              HopperConstants.simAgitatorMoiKgMetersSq,
              HopperConstants.simAgitatorReduction),
          HopperConstants.simAgitatorGearbox);
  private final ElevatorSim extensionSim =
      new ElevatorSim(
          HopperConstants.simExtensionGearbox,
          HopperConstants.simExtensionReduction,
          HopperConstants.simEstimatedExtensionMassKg,
          HopperConstants.simEstimatedExtensionDrumRadiusMeters,
          HopperConstants.simExtensionRetractedHeightMeters,
          HopperConstants.simExtensionExtendedHeightMeters,
          true,
          HopperConstants.simExtensionRetractedHeightMeters);

  private final PIDController agitatorController =
      new PIDController(
          HopperConstants.hopperAgitatorVelocityKp,
          HopperConstants.hopperAgitatorVelocityKi,
          HopperConstants.hopperAgitatorVelocityKd);
  private final PIDController extensionController =
      new PIDController(
          HopperConstants.hopperExtensionPositionKp,
          HopperConstants.hopperExtensionPositionKi,
          HopperConstants.hopperExtensionPositionKd);

  private boolean agitatorClosedLoopEnabled = false;
  private boolean extensionClosedLoopEnabled = false;
  private double agitatorVelocitySetpointRotationsPerSec = 0.0;
  private double extensionPositionSetpointRotations =
      HopperConstants.defaultHopperExtensionRetractedPositionRotations;
  private double agitatorVelocityKv = HopperConstants.hopperAgitatorVelocityKv;
  private double agitatorAppliedOutput = 0.0;
  private double extensionAppliedOutput = 0.0;
  private double agitatorPositionRotations = 0.0;

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    double agitatorVelocityRotationsPerSec =
        agitatorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double extensionPositionRotations =
        metersToExtensionRotations(extensionSim.getPositionMeters());
    double agitatorAppliedVolts =
        agitatorClosedLoopEnabled
            ? clampVoltage(
                (agitatorController.calculate(
                            agitatorVelocityRotationsPerSec,
                            agitatorVelocitySetpointRotationsPerSec)
                        + (agitatorVelocityKv * agitatorVelocitySetpointRotationsPerSec))
                    * simClosedLoopDutyToVoltsScale)
            : clampVoltage(agitatorAppliedOutput * HopperConstants.simNominalVoltage);
    double extensionAppliedVolts =
        extensionClosedLoopEnabled
            ? clampVoltage(
                extensionController.calculate(
                        extensionPositionRotations, extensionPositionSetpointRotations)
                    * simClosedLoopDutyToVoltsScale)
            : clampVoltage(extensionAppliedOutput * HopperConstants.simNominalVoltage);

    agitatorSim.setInputVoltage(agitatorAppliedVolts);
    extensionSim.setInputVoltage(extensionAppliedVolts);
    agitatorSim.update(loopPeriodSeconds);
    extensionSim.update(loopPeriodSeconds);

    agitatorPositionRotations +=
        agitatorSim.getAngularVelocityRadPerSec() * loopPeriodSeconds / (2.0 * Math.PI);

    inputs.agitatorConnected = true;
    inputs.extensionConnected = true;
    inputs.agitatorPositionRotations = agitatorPositionRotations;
    inputs.agitatorVelocityRotationsPerSec =
        agitatorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.agitatorAppliedOutput = agitatorAppliedVolts / HopperConstants.simNominalVoltage;
    inputs.agitatorCurrentAmps = Math.abs(agitatorSim.getCurrentDrawAmps());
    inputs.extensionPositionRotations =
        metersToExtensionRotations(extensionSim.getPositionMeters());
    inputs.extensionVelocityRpm =
        extensionSim.getVelocityMetersPerSecond() * extensionRotationsPerMeter * 60.0;
    inputs.extensionAppliedOutput = extensionAppliedVolts / HopperConstants.simNominalVoltage;
    inputs.extensionCurrentAmps = Math.abs(extensionSim.getCurrentDrawAmps());
  }

  @Override
  public void setAgitatorOutput(double output) {
    agitatorClosedLoopEnabled = false;
    agitatorAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setAgitatorVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
    agitatorClosedLoopEnabled = true;
    agitatorVelocitySetpointRotationsPerSec = velocityRotationsPerSec;
    agitatorAppliedOutput = 0.0;
  }

  @Override
  public void setExtensionOutput(double output) {
    extensionClosedLoopEnabled = false;
    extensionAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setExtensionPositionSetpointRotations(double positionRotations) {
    extensionClosedLoopEnabled = true;
    extensionPositionSetpointRotations = positionRotations;
    extensionAppliedOutput = 0.0;
  }

  @Override
  public void setAgitatorVelocityClosedLoopGains(double kp, double ki, double kd, double kv) {
    agitatorController.setPID(kp, ki, kd);
    agitatorVelocityKv = kv;
  }

  @Override
  public void setExtensionPositionClosedLoopGains(double kp, double ki, double kd) {
    extensionController.setPID(kp, ki, kd);
  }

  @Override
  public void resetSimulationState() {
    agitatorClosedLoopEnabled = false;
    extensionClosedLoopEnabled = false;
    agitatorVelocitySetpointRotationsPerSec = 0.0;
    agitatorAppliedOutput = 0.0;
    extensionAppliedOutput = 0.0;
    agitatorPositionRotations = 0.0;
    agitatorSim.setState(VecBuilder.fill(0.0));
    extensionSim.setState(HopperConstants.simExtensionRetractedHeightMeters, 0.0);
  }

  private static double clampVoltage(double volts) {
    return MathUtil.clamp(
        volts, -HopperConstants.simNominalVoltage, HopperConstants.simNominalVoltage);
  }

  private static double metersToExtensionRotations(double extensionMeters) {
    return HopperConstants.defaultHopperExtensionRetractedPositionRotations
        + ((extensionMeters - HopperConstants.simExtensionRetractedHeightMeters)
            * extensionRotationsPerMeter);
  }
}
