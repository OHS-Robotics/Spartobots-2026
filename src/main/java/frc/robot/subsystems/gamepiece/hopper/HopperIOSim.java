package frc.robot.subsystems.gamepiece.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HopperIOSim implements HopperIO {
  private static final double loopPeriodSeconds = 0.02;
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

  private double agitatorAppliedOutput = 0.0;
  private double extensionAppliedOutput = 0.0;
  private double agitatorPositionRotations = 0.0;

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    agitatorSim.setInputVoltage(agitatorAppliedOutput * HopperConstants.simNominalVoltage);
    extensionSim.setInputVoltage(extensionAppliedOutput * HopperConstants.simNominalVoltage);
    agitatorSim.update(loopPeriodSeconds);
    extensionSim.update(loopPeriodSeconds);

    agitatorPositionRotations +=
        agitatorSim.getAngularVelocityRadPerSec() * loopPeriodSeconds / (2.0 * Math.PI);

    inputs.agitatorConnected = true;
    inputs.extensionConnected = true;
    inputs.agitatorPositionRotations = agitatorPositionRotations;
    inputs.agitatorVelocityRotationsPerSec =
        agitatorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.agitatorAppliedOutput = agitatorAppliedOutput;
    inputs.agitatorCurrentAmps = Math.abs(agitatorSim.getCurrentDrawAmps());
    inputs.extensionPositionRotations =
        HopperConstants.defaultHopperExtensionRetractedPositionRotations
            + ((extensionSim.getPositionMeters()
                    - HopperConstants.simExtensionRetractedHeightMeters)
                * extensionRotationsPerMeter);
    inputs.extensionVelocityRpm =
        extensionSim.getVelocityMetersPerSecond() * extensionRotationsPerMeter * 60.0;
    inputs.extensionAppliedOutput = extensionAppliedOutput;
    inputs.extensionCurrentAmps = Math.abs(extensionSim.getCurrentDrawAmps());
  }

  @Override
  public void setAgitatorOutput(double output) {
    agitatorAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setExtensionOutput(double output) {
    extensionAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void resetSimulationState() {
    agitatorAppliedOutput = 0.0;
    extensionAppliedOutput = 0.0;
    agitatorPositionRotations = 0.0;
    agitatorSim.setState(VecBuilder.fill(0.0));
    extensionSim.setState(HopperConstants.simExtensionRetractedHeightMeters, 0.0);
  }
}
