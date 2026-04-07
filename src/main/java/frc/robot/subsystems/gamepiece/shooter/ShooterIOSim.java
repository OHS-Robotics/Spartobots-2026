package frc.robot.subsystems.gamepiece.shooter;

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

  private final FlywheelSim drumSim =
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

  private final PIDController wheelController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController hoodController = new PIDController(0.0, 0.0, 0.0);

  private double sharedVelocitySetpointRadPerSec = 0.0;
  private double hoodPositionSetpointRotations =
      ShooterConstants.defaultHoodRetractedPositionRotations;
  private boolean hoodOpenLoopControlEnabled = false;
  private double hoodOpenLoopOutput = 0.0;
  private double wheelAppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;
  private double wheelVelocityKv = ShooterConstants.shooterVelocityKv;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double wheelDutyCycleOutput =
        wheelController.calculate(
                drumSim.getAngularVelocityRadPerSec(), sharedVelocitySetpointRadPerSec)
            + (wheelVelocityKv * sharedVelocitySetpointRadPerSec);

    wheelAppliedVolts = clampVoltage(wheelDutyCycleOutput * simClosedLoopDutyToVoltsScale);
    hoodAppliedVolts =
        hoodOpenLoopControlEnabled
            ? clampVoltage(hoodOpenLoopOutput * simClosedLoopDutyToVoltsScale)
            : clampVoltage(
                hoodController.calculate(
                    hoodSim.getAngleRads(), hoodRotationsToAngle(hoodPositionSetpointRotations)));

    drumSim.setInputVoltage(wheelAppliedVolts);
    hoodSim.setInputVoltage(hoodAppliedVolts);

    drumSim.update(loopPeriodSeconds);
    hoodSim.update(loopPeriodSeconds);

    inputs.pair1Connected = true;
    inputs.pair2Connected = true;
    inputs.hoodConnected = true;

    inputs.pair1LeaderVelocityRadPerSec = drumSim.getAngularVelocityRadPerSec();
    inputs.pair2LeaderVelocityRadPerSec = drumSim.getAngularVelocityRadPerSec();

    inputs.pair1AppliedVolts = wheelAppliedVolts;
    inputs.pair2AppliedVolts = wheelAppliedVolts;
    inputs.pair1CurrentAmps = Math.abs(drumSim.getCurrentDrawAmps());
    inputs.pair2CurrentAmps = Math.abs(drumSim.getCurrentDrawAmps());

    inputs.hoodPositionRotations = angleToHoodRotations(hoodSim.getAngleRads());
    inputs.hoodVelocityRotationsPerSec = hoodSim.getVelocityRadPerSec() * hoodRotationsPerRadian;
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
  }

  @Override
  public void setWheelVelocitySetpoints(double pair1RadPerSec, double pair2RadPerSec) {
    sharedVelocitySetpointRadPerSec = resolveSharedWheelSetpoint(pair1RadPerSec, pair2RadPerSec);
  }

  @Override
  public void setHoodPositionSetpointRotations(double hoodPositionRotations) {
    this.hoodPositionSetpointRotations = hoodPositionRotations;
    hoodOpenLoopControlEnabled = false;
    hoodOpenLoopOutput = 0.0;
  }

  @Override
  public void setHoodOpenLoopOutput(double output) {
    hoodOpenLoopControlEnabled = true;
    hoodOpenLoopOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setHoodEncoderPositionRotations(double hoodPositionRotations) {
    hoodPositionSetpointRotations = hoodPositionRotations;
    hoodOpenLoopControlEnabled = false;
    hoodOpenLoopOutput = 0.0;
    hoodSim.setState(hoodRotationsToAngle(hoodPositionRotations), 0.0);
  }

  @Override
  public void setWheelVelocityClosedLoopGains(double kp, double ki, double kd, double kv) {
    wheelController.setPID(kp, ki, kd);
    wheelVelocityKv = Math.abs(kv) > 1e-6 ? kv : simFallbackWheelVelocityKv;
  }

  @Override
  public void setHoodPositionClosedLoopGains(double kp, double ki, double kd) {
    hoodController.setPID(kp, ki, kd);
  }

  @Override
  public void stop() {
    sharedVelocitySetpointRadPerSec = 0.0;
    wheelAppliedVolts = 0.0;
    hoodOpenLoopControlEnabled = false;
    hoodOpenLoopOutput = 0.0;
    hoodAppliedVolts = 0.0;
  }

  @Override
  public void resetSimulationState() {
    stop();
    drumSim.setState(VecBuilder.fill(0.0));
    hoodSim.setState(hoodRotationsToAngle(hoodPositionSetpointRotations), 0.0);
  }

  private static double resolveSharedWheelSetpoint(double pair1RadPerSec, double pair2RadPerSec) {
    if (Math.abs(pair1RadPerSec) <= 1e-9) {
      return pair2RadPerSec;
    }
    if (Math.abs(pair2RadPerSec) <= 1e-9) {
      return pair1RadPerSec;
    }
    return 0.5 * (pair1RadPerSec + pair2RadPerSec);
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
