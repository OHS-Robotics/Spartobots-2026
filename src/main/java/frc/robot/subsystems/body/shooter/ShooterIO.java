package frc.robot.subsystems.body.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean pair1Connected = false;
    public boolean pair2Connected = false;
    public boolean hoodConnected = false;

    public double pair1LeaderVelocityRadPerSec = 0.0;
    public double pair1FollowerVelocityRadPerSec = 0.0;
    public double pair2LeaderVelocityRadPerSec = 0.0;
    public double pair2FollowerVelocityRadPerSec = 0.0;

    public double pair1AppliedVolts = 0.0;
    public double pair2AppliedVolts = 0.0;
    public double pair1CurrentAmps = 0.0;
    public double pair2CurrentAmps = 0.0;

    public double hoodPositionRotations = 0.0;
    public double hoodVelocityRotationsPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setWheelVelocitySetpoints(double pair1RadPerSec, double pair2RadPerSec) {}

  public default void setHoodPositionSetpointRotations(double hoodPositionRotations) {}

  public default void stop() {}
}
