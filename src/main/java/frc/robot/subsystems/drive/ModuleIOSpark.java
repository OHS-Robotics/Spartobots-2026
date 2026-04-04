// Copyright (c) 2021-2026 Littleton Robotics
// Copyright (c) 2026 Team 4687 Spartobots
//
// SPDX-License-Identifier: BSD-3-Clause

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.NetworkTablesUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive + turn motor controllers and CANcoder absolute
 * encoder.
 */
public class ModuleIOSpark implements ModuleIO {
  private final double defaultZeroOffsetRad;
  private final NetworkTableEntry zeroOffsetEntry;

  // Hardware objects
  private final SparkFlex driveSpark;
  private final SparkFlex turnSpark;
  private final CANcoder turnCanCoder;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final PIDController turnController = new PIDController(turnKp, turnKi, turnKd);
  private boolean turnClosedLoop = false;
  private boolean turnRelativeEncoderSeeded = !zeroRelativeTurnEncoderFromAbsolute;
  private double lastTurnPositionRad = 0.0;
  private double lastTurnSetpointRad = Double.NaN;
  private double turnAppliedVolts = 0.0;
  private double driveVelocityKp = driveKp;
  private double driveVelocityKi = driveKi;
  private double driveVelocityKd = driveKd;
  private double turnPositionKp = turnKp;
  private double turnPositionKi = turnKi;
  private double turnPositionKd = turnKd;
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<AngularVelocity> turnVelocity;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOSpark(int module) {
    int hardwareIndex = moduleIndexToHardwareIndex[module];
    defaultZeroOffsetRad =
        switch (hardwareIndex) {
          case 0 -> frontLeftZeroRotation.getRadians();
          case 1 -> frontRightZeroRotation.getRadians();
          case 2 -> backLeftZeroRotation.getRadians();
          case 3 -> backRightZeroRotation.getRadians();
          default -> 0.0;
        };
    NetworkTable moduleTable = NetworkTablesUtil.domain("Drive/Module" + Integer.toString(module));
    zeroOffsetEntry =
        NetworkTablesUtil.tuningCommon(moduleTable).getEntry("Turn/ZeroOffsetRadians");
    // Code constants are the source of truth on boot. Clear any previously captured persistent
    // value so edits in DriveConstants take effect immediately after redeploy.
    zeroOffsetEntry.clearPersistent();
    zeroOffsetEntry.setDouble(defaultZeroOffsetRad);
    driveSpark =
        new SparkFlex(
            switch (hardwareIndex) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnSpark =
        new SparkFlex(
            switch (hardwareIndex) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnCanCoder =
        zeroRelativeTurnEncoderFromAbsolute
            ? new CANcoder(
                switch (hardwareIndex) {
                  case 0 -> frontLeftCANcoderId;
                  case 1 -> frontRightCANcoderId;
                  case 2 -> backLeftCANcoderId;
                  case 3 -> backRightCANcoderId;
                  default -> 0;
                })
            : null;
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    if (zeroRelativeTurnEncoderFromAbsolute) {
      turnCanCoder.getConfigurator().apply(new CANcoderConfiguration());
      turnAbsolutePosition = turnCanCoder.getAbsolutePosition();
      turnVelocity = turnCanCoder.getVelocity();
      turnAbsolutePosition.setUpdateFrequency(odometryFrequency);
      turnVelocity.setUpdateFrequency(50.0);
      turnCanCoder.optimizeBusUtilization();
    } else {
      turnAbsolutePosition = null;
      turnVelocity = null;
    }
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    turnController.setIntegratorRange(-turnMaxIntegralOutputVolts, turnMaxIntegralOutputVolts);

    // Configure drive motor
    var driveConfig = new SparkFlexConfig();
    driveConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(driveKp, driveKi, driveKd);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    var turnConfig = new SparkFlexConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    turnConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(turnSpark, 5, () -> turnEncoder.setPosition(0.0));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.turnZeroOffsetRad = getConfiguredZeroOffsetRad();

    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    inputs.turnAbsoluteConnected = false;
    inputs.turnRelativeEncoderSeeded = turnRelativeEncoderSeeded;
    boolean relativeTurnOk;
    if (zeroRelativeTurnEncoderFromAbsolute) {
      boolean cancoderOk =
          BaseStatusSignal.refreshAll(turnAbsolutePosition, turnVelocity).equals(StatusCode.OK);
      inputs.turnAbsoluteConnected = cancoderOk;
      if (cancoderOk) {
        inputs.turnAbsolutePosition = new Rotation2d(getAngleFromAbsoluteTurnEncoderRadians());
      }
      if (DriverStation.isDisabled() && cancoderOk && !turnClosedLoop) {
        syncTurnEncoderToAbsolute();
      }
    }
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> {
          double positionRad = MathUtil.angleModulus(value);
          lastTurnPositionRad = positionRad;
          inputs.turnPosition = new Rotation2d(positionRad);
        });
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    relativeTurnOk = !sparkStickyFault;
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected =
        turnConnectedDebounce.calculate(
            relativeTurnOk && !sparkStickyFault && turnRelativeEncoderSeeded);
    inputs.turnRelativeEncoderSeeded = turnRelativeEncoderSeeded;

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(
                (Double value) -> {
                  double positionRad = MathUtil.angleModulus(value);
                  return new Rotation2d(positionRad);
                })
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    lastTurnSetpointRad = Double.NaN;
    turnAppliedVolts = output;
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpointRad = rotation.getRadians();
    if (!turnClosedLoop
        || !Double.isFinite(lastTurnSetpointRad)
        || Math.abs(MathUtil.angleModulus(setpointRad - lastTurnSetpointRad))
            > turnSetpointResetThresholdRadians) {
      turnController.reset();
      turnClosedLoop = true;
    }
    lastTurnSetpointRad = setpointRad;
    turnAppliedVolts =
        MathUtil.clamp(turnController.calculate(lastTurnPositionRad, setpointRad), -12.0, 12.0);
    turnSpark.setVoltage(turnAppliedVolts);
  }

  @Override
  public void resetTurnPositionController() {
    turnController.reset();
    turnClosedLoop = false;
    lastTurnSetpointRad = Double.NaN;
  }

  @Override
  public boolean syncTurnEncoderToAbsolute() {
    if (!zeroRelativeTurnEncoderFromAbsolute || turnCanCoder == null) {
      turnRelativeEncoderSeeded = true;
      return true;
    }

    boolean cancoderOk =
        BaseStatusSignal.refreshAll(turnAbsolutePosition, turnVelocity).equals(StatusCode.OK);
    if (!cancoderOk) {
      turnRelativeEncoderSeeded = false;
      return false;
    }

    double absolutePositionRad = MathUtil.angleModulus(getAngleFromAbsoluteTurnEncoderRadians());
    tryUntilOk(turnSpark, 5, () -> turnEncoder.setPosition(absolutePositionRad));
    boolean synced = turnSpark.getLastError() == REVLibError.kOk;
    turnRelativeEncoderSeeded = synced;
    if (synced) {
      lastTurnPositionRad = absolutePositionRad;
      turnController.reset();
      turnClosedLoop = false;
      lastTurnSetpointRad = Double.NaN;
    }
    return synced;
  }

  @Override
  public boolean captureTurnZeroOffsetFromAbsolute() {
    if (!zeroRelativeTurnEncoderFromAbsolute || turnCanCoder == null) {
      return false;
    }

    boolean cancoderOk =
        BaseStatusSignal.refreshAll(turnAbsolutePosition, turnVelocity).equals(StatusCode.OK);
    if (!cancoderOk) {
      return false;
    }

    double rawAbsolutePositionRad = MathUtil.angleModulus(getRawAbsoluteTurnEncoderRadians());
    zeroOffsetEntry.setDouble(rawAbsolutePositionRad);
    return syncTurnEncoderToAbsolute();
  }

  @Override
  public boolean resetTurnZeroOffsetToDefault() {
    if (!zeroRelativeTurnEncoderFromAbsolute || turnCanCoder == null) {
      return false;
    }

    zeroOffsetEntry.setDouble(defaultZeroOffsetRad);
    return syncTurnEncoderToAbsolute();
  }

  @Override
  public void setDriveVelocityGains(double kp, double ki, double kd) {
    if (!hasGainChange(driveVelocityKp, kp)
        && !hasGainChange(driveVelocityKi, ki)
        && !hasGainChange(driveVelocityKd, kd)) {
      return;
    }

    driveVelocityKp = kp;
    driveVelocityKi = ki;
    driveVelocityKd = kd;

    var drivePidConfig = new SparkFlexConfig();
    drivePidConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kp, ki, kd);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                drivePidConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters));
  }

  @Override
  public void setTurnPositionGains(double kp, double ki, double kd) {
    if (!hasGainChange(turnPositionKp, kp)
        && !hasGainChange(turnPositionKi, ki)
        && !hasGainChange(turnPositionKd, kd)) {
      return;
    }

    turnPositionKp = kp;
    turnPositionKi = ki;
    turnPositionKd = kd;
    turnController.setPID(kp, ki, kd);
  }

  private boolean hasGainChange(double oldValue, double newValue) {
    return Math.abs(oldValue - newValue) > 1e-9;
  }

  private double getAngleFromAbsoluteTurnEncoderRadians() {
    return MathUtil.angleModulus(getRawAbsoluteTurnEncoderRadians() - getConfiguredZeroOffsetRad());
  }

  private double getRawAbsoluteTurnEncoderRadians() {
    double positionRad = Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble());
    if (turnEncoderInverted) {
      positionRad = -positionRad;
    }
    return positionRad;
  }

  private double getConfiguredZeroOffsetRad() {
    double configured = zeroOffsetEntry.getDouble(defaultZeroOffsetRad);
    if (!Double.isFinite(configured)) {
      configured = defaultZeroOffsetRad;
    }
    zeroOffsetEntry.setDouble(configured);
    return configured;
  }
}
