package frc.robot.subsystems.gamepiece.indexers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexersIOSim implements IndexersIO {
  private static final double loopPeriodSeconds = 0.02;
  private static final double simClosedLoopDutyToVoltsScale = IndexersConstants.simNominalVoltage;

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

  private final PIDController topController =
      new PIDController(
          IndexersConstants.indexerVelocityKp,
          IndexersConstants.indexerVelocityKi,
          IndexersConstants.indexerVelocityKd);
  private final PIDController bottomController =
      new PIDController(
          IndexersConstants.indexerVelocityKp,
          IndexersConstants.indexerVelocityKi,
          IndexersConstants.indexerVelocityKd);

  private boolean topClosedLoopEnabled = false;
  private boolean bottomClosedLoopEnabled = false;
  private double topVelocitySetpointRotationsPerSec = 0.0;
  private double bottomVelocitySetpointRotationsPerSec = 0.0;
  private double velocityKv = IndexersConstants.indexerVelocityKv;
  private double topAppliedOutput = 0.0;
  private double bottomAppliedOutput = 0.0;
  private double topPositionRotations = 0.0;
  private double bottomPositionRotations = 0.0;

  @Override
  public void updateInputs(IndexersIOInputs inputs) {
    double topVelocityRotationsPerSec = topSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double bottomVelocityRotationsPerSec =
        bottomSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double topAppliedVolts =
        topClosedLoopEnabled
            ? clampVoltage(
                (topController.calculate(
                            topVelocityRotationsPerSec, topVelocitySetpointRotationsPerSec)
                        + (velocityKv * topVelocitySetpointRotationsPerSec))
                    * simClosedLoopDutyToVoltsScale)
            : clampVoltage(topAppliedOutput * IndexersConstants.simNominalVoltage);
    double bottomAppliedVolts =
        bottomClosedLoopEnabled
            ? clampVoltage(
                (bottomController.calculate(
                            bottomVelocityRotationsPerSec, bottomVelocitySetpointRotationsPerSec)
                        + (velocityKv * bottomVelocitySetpointRotationsPerSec))
                    * simClosedLoopDutyToVoltsScale)
            : clampVoltage(bottomAppliedOutput * IndexersConstants.simNominalVoltage);

    topSim.setInputVoltage(topAppliedVolts);
    bottomSim.setInputVoltage(bottomAppliedVolts);
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
    inputs.topAppliedOutput = topAppliedVolts / IndexersConstants.simNominalVoltage;
    inputs.topCurrentAmps = Math.abs(topSim.getCurrentDrawAmps());
    inputs.bottomPositionRotations = bottomPositionRotations;
    inputs.bottomVelocityRotationsPerSec =
        bottomSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    inputs.bottomAppliedOutput = bottomAppliedVolts / IndexersConstants.simNominalVoltage;
    inputs.bottomCurrentAmps = Math.abs(bottomSim.getCurrentDrawAmps());
  }

  @Override
  public void setTopOutput(double output) {
    topClosedLoopEnabled = false;
    topAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setTopVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
    topClosedLoopEnabled = true;
    topVelocitySetpointRotationsPerSec = velocityRotationsPerSec;
    topAppliedOutput = 0.0;
  }

  @Override
  public void setBottomOutput(double output) {
    bottomClosedLoopEnabled = false;
    bottomAppliedOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setBottomVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
    bottomClosedLoopEnabled = true;
    bottomVelocitySetpointRotationsPerSec = velocityRotationsPerSec;
    bottomAppliedOutput = 0.0;
  }

  @Override
  public void setVelocityClosedLoopGains(double kp, double ki, double kd, double kv) {
    topController.setPID(kp, ki, kd);
    bottomController.setPID(kp, ki, kd);
    velocityKv = kv;
  }

  @Override
  public void resetSimulationState() {
    topClosedLoopEnabled = false;
    bottomClosedLoopEnabled = false;
    topVelocitySetpointRotationsPerSec = 0.0;
    bottomVelocitySetpointRotationsPerSec = 0.0;
    topAppliedOutput = 0.0;
    bottomAppliedOutput = 0.0;
    topPositionRotations = 0.0;
    bottomPositionRotations = 0.0;
    topSim.setState(VecBuilder.fill(0.0));
    bottomSim.setState(VecBuilder.fill(0.0));
  }

  private static double clampVoltage(double volts) {
    return MathUtil.clamp(
        volts, -IndexersConstants.simNominalVoltage, IndexersConstants.simNominalVoltage);
  }
}
