// Copyright (c) 2021-2026 Littleton Robotics
// Copyright (c) 2026 Team 4687 Spartobots
//
// SPDX-License-Identifier: BSD-3-Clause

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean turnAbsoluteConnected = false;
    public boolean turnRelativeEncoderSeeded = false;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnZeroOffsetRad = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}

  /** Clear any stored state in the turn position controller. */
  public default void resetTurnPositionController() {}

  /** Seeds the steering motor's relative encoder from the absolute encoder when available. */
  public default boolean syncTurnEncoderToAbsolute() {
    return false;
  }

  /** Captures the current absolute encoder angle as the steering zero offset. */
  public default boolean captureTurnZeroOffsetFromAbsolute() {
    return false;
  }

  /** Restores the steering zero offset to the code-defined default. */
  public default boolean resetTurnZeroOffsetToDefault() {
    return false;
  }

  /** Update drive motor velocity loop gains. */
  public default void setDriveVelocityGains(double kp, double ki, double kd) {}

  /** Update turn motor position loop gains. */
  public default void setTurnPositionGains(double kp, double ki, double kd) {}
}
