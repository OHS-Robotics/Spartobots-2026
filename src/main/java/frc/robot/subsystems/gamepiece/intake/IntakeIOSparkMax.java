package frc.robot.subsystems.gamepiece.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkFlex intakeDrive =
      new SparkFlex(IntakeConstants.intakeDriveCanId, MotorType.kBrushless);
  private final SparkFlex intakeFollower =
      new SparkFlex(IntakeConstants.intakeFollowerCanId, MotorType.kBrushless);
  private final SparkMax intakePivot =
      new SparkMax(IntakeConstants.intakePivotCanId, MotorType.kBrushless);

  private final RelativeEncoder intakeDriveEncoder = intakeDrive.getEncoder();
  private final RelativeEncoder intakePivotEncoder = intakePivot.getEncoder();
  private final SparkClosedLoopController intakeDriveController =
      intakeDrive.getClosedLoopController();
  private final SparkClosedLoopController intakePivotController =
      intakePivot.getClosedLoopController();

  private double driveVelocityKp = IntakeConstants.intakeDriveVelocityKp;
  private double driveVelocityKi = IntakeConstants.intakeDriveVelocityKi;
  private double driveVelocityKd = IntakeConstants.intakeDriveVelocityKd;
  private double driveVelocityKv = IntakeConstants.intakeDriveVelocityKv;
  private double pivotPositionKp = IntakeConstants.intakePivotPositionKp;
  private double pivotPositionKi = IntakeConstants.intakePivotPositionKi;
  private double pivotPositionKd = IntakeConstants.intakePivotPositionKd;

  public IntakeIOSparkMax() {
    SparkFlexConfig intakeDriveConfig = new SparkFlexConfig();
    intakeDriveConfig.idleMode(IdleMode.kCoast);
    intakeDriveConfig.smartCurrentLimit(IntakeConstants.intakeDriveCurrentLimitAmps);
    intakeDriveConfig.voltageCompensation(12.0);
    intakeDriveConfig.encoder.velocityConversionFactor(1.0 / 60.0);
    intakeDriveConfig.closedLoop.pid(
        IntakeConstants.intakeDriveVelocityKp,
        IntakeConstants.intakeDriveVelocityKi,
        IntakeConstants.intakeDriveVelocityKd);
    intakeDriveConfig.closedLoop.feedForward.kV(IntakeConstants.intakeDriveVelocityKv);

    SparkFlexConfig intakeFollowerConfig = new SparkFlexConfig();
    intakeFollowerConfig.follow(intakeDrive, false);
    intakeFollowerConfig.idleMode(IdleMode.kCoast);
    intakeFollowerConfig.smartCurrentLimit(IntakeConstants.intakeDriveCurrentLimitAmps);
    intakeFollowerConfig.voltageCompensation(12.0);

    SparkMaxConfig intakePivotConfig = new SparkMaxConfig();
    intakePivotConfig.idleMode(IdleMode.kBrake);
    intakePivotConfig.smartCurrentLimit(IntakeConstants.intakePivotCurrentLimitAmps);
    intakePivotConfig.voltageCompensation(12.0);
    intakePivotConfig.closedLoop.pid(
        IntakeConstants.intakePivotPositionKp,
        IntakeConstants.intakePivotPositionKi,
        IntakeConstants.intakePivotPositionKd);

    intakeDrive.configure(
        intakeDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    intakeFollower.configure(
        intakeFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    intakePivot.configure(
        intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.pivotConnected = true;
    inputs.drivePositionRotations = intakeDriveEncoder.getPosition();
    inputs.driveVelocityRotationsPerSec = intakeDriveEncoder.getVelocity();
    inputs.driveAppliedOutput = intakeDrive.get();
    inputs.driveCurrentAmps = intakeDrive.getOutputCurrent();
    inputs.pivotPositionRotations = intakePivotEncoder.getPosition();
    inputs.pivotVelocityRpm = intakePivotEncoder.getVelocity();
    inputs.pivotAppliedOutput = intakePivot.get();
    inputs.pivotCurrentAmps = intakePivot.getOutputCurrent();
  }

  @Override
  public void setDriveOutput(double output) {
    intakeDrive.set(output);
  }

  @Override
  public void setDriveVelocitySetpointRotationsPerSec(double velocityRotationsPerSec) {
    intakeDriveController.setSetpoint(
        velocityRotationsPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setPivotOutput(double output) {
    intakePivot.set(output);
  }

  @Override
  public void setPivotPositionSetpointRotations(double positionRotations) {
    intakePivotController.setSetpoint(
        positionRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setPivotEncoderPositionRotations(double positionRotations) {
    intakePivotEncoder.setPosition(positionRotations);
  }

  @Override
  public void setDriveVelocityClosedLoopGains(double kp, double ki, double kd, double kv) {
    if (!hasGainChange(driveVelocityKp, kp)
        && !hasGainChange(driveVelocityKi, ki)
        && !hasGainChange(driveVelocityKd, kd)
        && !hasGainChange(driveVelocityKv, kv)) {
      return;
    }

    driveVelocityKp = kp;
    driveVelocityKi = ki;
    driveVelocityKd = kd;
    driveVelocityKv = kv;

    SparkFlexConfig driveVelocityConfig = new SparkFlexConfig();
    driveVelocityConfig.closedLoop.pid(kp, ki, kd);
    driveVelocityConfig.closedLoop.feedForward.kV(kv);
    intakeDrive.configure(
        driveVelocityConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setPivotPositionClosedLoopGains(double kp, double ki, double kd) {
    if (!hasGainChange(pivotPositionKp, kp)
        && !hasGainChange(pivotPositionKi, ki)
        && !hasGainChange(pivotPositionKd, kd)) {
      return;
    }

    pivotPositionKp = kp;
    pivotPositionKi = ki;
    pivotPositionKd = kd;

    SparkMaxConfig pivotPositionConfig = new SparkMaxConfig();
    pivotPositionConfig.closedLoop.pid(kp, ki, kd);
    intakePivot.configure(
        pivotPositionConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private static boolean hasGainChange(double oldValue, double newValue) {
    return Math.abs(oldValue - newValue) > 1e-9;
  }
}
