// Copyright (c) 2021-2026 Littleton Robotics
// Copyright (c) 2026 Team 4687 Spartobots
//
// SPDX-License-Identifier: BSD-3-Clause

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.NetworkTablesUtil;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderMismatchAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  private Rotation2d angleSetpoint = null;
  private boolean holdingAngle = false;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderMismatchAlert =
        new Alert(
            "Turn encoder mismatch on module " + Integer.toString(index) + ".", AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(
        NetworkTablesUtil.logPath("Drive/Inputs/Module" + Integer.toString(index)), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderMismatchAlert.set(
        inputs.turnAbsoluteConnected
            && inputs.turnRelativeEncoderSeeded
            && Math.abs(inputs.turnRelativeToAbsoluteErrorRad)
                > turnAbsoluteMismatchAlertThresholdRadians);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    runSetpoint(state, true);
  }

  void runSetpoint(SwerveModuleState state, boolean preserveAngleAtLowSpeed) {
    if (!inputs.driveConnected || !inputs.turnConnected) {
      stop();
      return;
    }

    Rotation2d currentAngle = getAngle();
    state.optimize(currentAngle);

    if (angleSetpoint == null) {
      angleSetpoint = currentAngle;
    }

    if (preserveAngleAtLowSpeed
        && Math.abs(state.speedMetersPerSecond) < moduleAngleHoldMinSpeedMetersPerSec) {
      if (!holdingAngle) {
        io.resetTurnPositionController();
      }
      holdingAngle = true;
      state.speedMetersPerSecond = 0.0;
      state.angle = angleSetpoint;
    } else {
      holdingAngle = false;
      angleSetpoint = state.angle;
    }

    state.cosineScale(currentAngle);
    io.setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    angleSetpoint = getAngle();
    holdingAngle = false;
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  /** Update drive motor velocity loop gains. */
  public void setDriveVelocityGains(double kp, double ki, double kd) {
    io.setDriveVelocityGains(kp, ki, kd);
  }

  /** Update turn motor position loop gains. */
  public void setTurnPositionGains(double kp, double ki, double kd) {
    io.setTurnPositionGains(kp, ki, kd);
  }

  public boolean syncTurnEncoderToAbsolute() {
    boolean synced = io.syncTurnEncoderToAbsolute();
    angleSetpoint = getAngle();
    holdingAngle = false;
    return synced;
  }

  public boolean captureTurnZeroOffsetFromAbsolute() {
    boolean captured = io.captureTurnZeroOffsetFromAbsolute();
    angleSetpoint = getAngle();
    holdingAngle = false;
    return captured;
  }
}
