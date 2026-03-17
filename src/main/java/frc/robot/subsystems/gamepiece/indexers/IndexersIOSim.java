package frc.robot.subsystems.gamepiece.indexers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexersIOSim implements IndexersIO {
  private static final double loopPeriodSeconds = 0.02;

  private final FlywheelSim topSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              IndexersConstants.simIndexerGearbox,
              IndexersConstants.simIndexerMoiKgMetersSq,
              IndexersConstants.simIndexerReduction),
          IndexersConstants.simIndexerGearbox);
  private final FlywheelSim bottomSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              IndexersConstants.simIndexerGearbox,
              IndexersConstants.simIndexerMoiKgMetersSq,
              IndexersConstants.simIndexerReduction),
          IndexersConstants.simIndexerGearbox);

  private double topAppliedOutput = 0.0;
  private double bottomAppliedOutput = 0.0;
  private double topPositionRotations = 0.0;
  private double bottomPositionRotations = 0.0;

  @Override
  public void updateInputs(IndexersIOInputs inputs) {
    topSim.setInputVoltage(topAppliedOutput * IndexersConstants.simNominalVoltage);
    bottomSim.setInputVoltage(bottomAppliedOutput * IndexersConstants.simNominalVoltage);
    topSim.update(loopPeriodSeconds);
    bottomSim.update(loopPeriodSeconds);

    topPositionRotations +=
        topSim.getAngularVelocityRadPerSec() * loopPeriodSeconds / (2.0 * Math.PI);
    bottomPositionRotations +=
        bottomSim.getAngularVelocityRadPerSec() * loopPeriodSeconds / (2.0 * Math.PI);

    inputs.topConnected = true;
    inputs.bottomConnected = true;
    inputs.topPositionRotations = topPositionRotations;
    inputs.topVelocityRotationsPerSec = topSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.topAppliedOutput = topAppliedOutput;
    inputs.topCurrentAmps = Math.abs(topSim.getCurrentDrawAmps());
    inputs.bottomPositionRotations = bottomPositionRotations;
    inputs.bottomVelocityRotationsPerSec =
        bottomSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.bottomAppliedOutput = bottomAppliedOutput;
    inputs.bottomCurrentAmps = Math.abs(bottomSim.getCurrentDrawAmps());
  }

  @Override
  public void setTopOutput(double output) {
    topAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setBottomOutput(double output) {
    bottomAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void resetSimulationState() {
    topAppliedOutput = 0.0;
    bottomAppliedOutput = 0.0;
    topPositionRotations = 0.0;
    bottomPositionRotations = 0.0;
    topSim.setState(VecBuilder.fill(0.0));
    bottomSim.setState(VecBuilder.fill(0.0));
  }
}
