package frc.robot.subsystems.body.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterIOSim implements ShooterIO {
  private static final double loopPeriodSeconds = 0.02;
  private static final double simClosedLoopDutyToVoltsScale = ShooterConstants.simNominalVoltage;
  private static final double simFallbackWheelVelocityKv = ShooterConstants.shooterVelocityKv;
  private static final double hoodRotationsPerRadian =
      (ShooterConstants.defaultHoodExtendedPositionRotations
              - ShooterConstants.defaultHoodRetractedPositionRotations)
          / (ShooterConstants.simHoodMaxAngleRadians - ShooterConstants.simHoodMinAngleRadians);

  private final FlywheelSim pair1Sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              ShooterConstants.simWheelGearbox,
              ShooterConstants.simWheelMoiKgMetersSq,
              ShooterConstants.simWheelReduction),
          ShooterConstants.simWheelGearbox);
  private final FlywheelSim pair2Sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              ShooterConstants.simWheelGearbox,
              ShooterConstants.simWheelMoiKgMetersSq,
              ShooterConstants.simWheelReduction),
          ShooterConstants.simWheelGearbox);
  private final SingleJointedArmSim hoodSim =
      new SingleJointedArmSim(
          ShooterConstants.simHoodGearbox,
          ShooterConstants.simHoodReduction,
          ShooterConstants.simEstimatedHoodMoiKgMetersSq,
          ShooterConstants.simEstimatedHoodLengthMeters,
          ShooterConstants.simHoodMinAngleRadians,
          ShooterConstants.simHoodMaxAngleRadians,
          true,
          ShooterConstants.simHoodStartingAngleRadians);

  private final PIDController pair1Controller = new PIDController(0.0, 0.0, 0.0);
  private final PIDController pair2Controller = new PIDController(0.0, 0.0, 0.0);
  private final PIDController hoodController = new PIDController(0.0, 0.0, 0.0);

  private double pair1VelocitySetpointRadPerSec = 0.0;
  private double pair2VelocitySetpointRadPerSec = 0.0;
  private double hoodPositionSetpointRotations =
      ShooterConstants.defaultHoodRetractedPositionRotations;
  private double pair1AppliedVolts = 0.0;
  private double pair2AppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;
  private double wheelVelocityKv = ShooterConstants.shooterVelocityKv;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double pair1DutyCycleOutput =
        pair1Controller.calculate(
                pair1Sim.getAngularVelocityRadPerSec(), pair1VelocitySetpointRadPerSec)
            + (wheelVelocityKv * pair1VelocitySetpointRadPerSec);
    double pair2DutyCycleOutput =
        pair2Controller.calculate(
                pair2Sim.getAngularVelocityRadPerSec(), pair2VelocitySetpointRadPerSec)
            + (wheelVelocityKv * pair2VelocitySetpointRadPerSec);

    pair1AppliedVolts =
        clampVoltage(pair1DutyCycleOutput * simClosedLoopDutyToVoltsScale);
    pair2AppliedVolts =
        clampVoltage(pair2DutyCycleOutput * simClosedLoopDutyToVoltsScale);
    hoodAppliedVolts =
        clampVoltage(
            hoodController.calculate(
                hoodSim.getAngleRads(), hoodRotationsToAngle(hoodPositionSetpointRotations)));

    pair1Sim.setInputVoltage(pair1AppliedVolts);
    pair2Sim.setInputVoltage(pair2AppliedVolts);
    hoodSim.setInputVoltage(hoodAppliedVolts);

    pair1Sim.update(loopPeriodSeconds);
    pair2Sim.update(loopPeriodSeconds);
    hoodSim.update(loopPeriodSeconds);

    inputs.pair1Connected = true;
    inputs.pair2Connected = true;
    inputs.hoodConnected = true;

    inputs.pair1LeaderVelocityRadPerSec = pair1Sim.getAngularVelocityRadPerSec();
    inputs.pair2LeaderVelocityRadPerSec = pair2Sim.getAngularVelocityRadPerSec();

    inputs.pair1AppliedVolts = pair1AppliedVolts;
    inputs.pair2AppliedVolts = pair2AppliedVolts;
    inputs.pair1CurrentAmps = Math.abs(pair1Sim.getCurrentDrawAmps());
    inputs.pair2CurrentAmps = Math.abs(pair2Sim.getCurrentDrawAmps());

    inputs.hoodPositionRotations = angleToHoodRotations(hoodSim.getAngleRads());
    inputs.hoodVelocityRotationsPerSec = hoodSim.getVelocityRadPerSec() * hoodRotationsPerRadian;
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
  }

  @Override
  public void setWheelVelocitySetpoints(double pair1RadPerSec, double pair2RadPerSec) {
    pair1VelocitySetpointRadPerSec = pair1RadPerSec;
    pair2VelocitySetpointRadPerSec = pair2RadPerSec;
  }

  @Override
  public void setHoodPositionSetpointRotations(double hoodPositionRotations) {
    this.hoodPositionSetpointRotations = hoodPositionRotations;
  }

  @Override
  public void setWheelVelocityClosedLoopGains(double kp, double ki, double kd, double kv) {
    pair1Controller.setPID(kp, ki, kd);
    pair2Controller.setPID(kp, ki, kd);
    wheelVelocityKv = Math.abs(kv) > 1e-6 ? kv : simFallbackWheelVelocityKv;
  }

  @Override
  public void setHoodPositionClosedLoopGains(double kp, double ki, double kd) {
    hoodController.setPID(kp, ki, kd);
  }

  @Override
  public void stop() {
    pair1VelocitySetpointRadPerSec = 0.0;
    pair2VelocitySetpointRadPerSec = 0.0;
    pair1AppliedVolts = 0.0;
    pair2AppliedVolts = 0.0;
    hoodAppliedVolts = 0.0;
  }

  @Override
  public void resetSimulationState() {
    stop();
    pair1Sim.setState(VecBuilder.fill(0.0));
    pair2Sim.setState(VecBuilder.fill(0.0));
    hoodSim.setState(hoodRotationsToAngle(hoodPositionSetpointRotations), 0.0);
  }

  private static double clampVoltage(double volts) {
    return MathUtil.clamp(
        volts, -ShooterConstants.simNominalVoltage, ShooterConstants.simNominalVoltage);
  }

  private static double hoodRotationsToAngle(double hoodPositionRotations) {
    return ShooterConstants.simHoodMinAngleRadians
        + ((hoodPositionRotations - ShooterConstants.defaultHoodRetractedPositionRotations)
            / hoodRotationsPerRadian);
  }

  private static double angleToHoodRotations(double hoodAngleRadians) {
    return ShooterConstants.defaultHoodRetractedPositionRotations
        + ((hoodAngleRadians - ShooterConstants.simHoodMinAngleRadians) * hoodRotationsPerRadian);
  }
}
