package frc.robot.subsystems.body.shooter;

import edu.wpi.first.math.MathUtil;

public class ShooterIOSim implements ShooterIO {
  private double pair1VelocitySetpointRadPerSec = 0.0;
  private double pair2VelocitySetpointRadPerSec = 0.0;
  private double hoodPositionSetpointRotations = ShooterConstants.hoodReferenceMotorRotations;

  private double pair1VelocityRadPerSec = 0.0;
  private double pair2VelocityRadPerSec = 0.0;
  private double hoodPositionRotations = ShooterConstants.hoodReferenceMotorRotations;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Basic first-order response for simulation feedback.
    pair1VelocityRadPerSec += (pair1VelocitySetpointRadPerSec - pair1VelocityRadPerSec) * 0.25;
    pair2VelocityRadPerSec += (pair2VelocitySetpointRadPerSec - pair2VelocityRadPerSec) * 0.25;

    double hoodDeltaPerLoop = 0.08;
    double hoodError = hoodPositionSetpointRotations - hoodPositionRotations;
    hoodPositionRotations += MathUtil.clamp(hoodError, -hoodDeltaPerLoop, hoodDeltaPerLoop);

    inputs.pair1Connected = true;
    inputs.pair2Connected = true;
    inputs.hoodConnected = true;

    inputs.pair1LeaderVelocityRadPerSec = pair1VelocityRadPerSec;
    inputs.pair1FollowerVelocityRadPerSec = pair1VelocityRadPerSec;
    inputs.pair2LeaderVelocityRadPerSec = pair2VelocityRadPerSec;
    inputs.pair2FollowerVelocityRadPerSec = pair2VelocityRadPerSec;

    inputs.pair1AppliedVolts = pair1VelocitySetpointRadPerSec != 0.0 ? 8.0 : 0.0;
    inputs.pair2AppliedVolts = pair2VelocitySetpointRadPerSec != 0.0 ? 8.0 : 0.0;
    inputs.pair1CurrentAmps = Math.abs(pair1VelocityRadPerSec) > 0.0 ? 15.0 : 0.0;
    inputs.pair2CurrentAmps = Math.abs(pair2VelocityRadPerSec) > 0.0 ? 15.0 : 0.0;

    inputs.hoodPositionRotations = hoodPositionRotations;
    inputs.hoodVelocityRotationsPerSec = hoodError;
    inputs.hoodAppliedVolts = Math.abs(hoodError) > 0.01 ? 2.0 : 0.0;
    inputs.hoodCurrentAmps = Math.abs(hoodError) > 0.01 ? 3.0 : 0.0;
  }

  @Override
  public void setWheelVelocitySetpoints(double pair1RadPerSec, double pair2RadPerSec) {
    pair1VelocitySetpointRadPerSec = pair1RadPerSec;
    pair2VelocitySetpointRadPerSec = pair2RadPerSec;
  }

  @Override
  public void setHoodPositionSetpointRotations(double hoodPositionRotations) {
    hoodPositionSetpointRotations = Math.max(0.0, hoodPositionRotations);
    this.hoodPositionSetpointRotations = hoodPositionSetpointRotations;
  }

  @Override
  public void stop() {
    pair1VelocitySetpointRadPerSec = 0.0;
    pair2VelocitySetpointRadPerSec = 0.0;
    hoodPositionSetpointRotations = hoodPositionRotations;
  }
}
