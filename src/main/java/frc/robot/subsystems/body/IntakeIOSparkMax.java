package frc.robot.subsystems.body;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax intakeDrive =
      new SparkMax(IntakeConstants.intakeDriveCanId, MotorType.kBrushless);
  private final SparkMax intakePivot =
      new SparkMax(IntakeConstants.intakePivotCanId, MotorType.kBrushless);

  private final RelativeEncoder intakeDriveEncoder = intakeDrive.getEncoder();
  private final RelativeEncoder intakePivotEncoder = intakePivot.getEncoder();

  public IntakeIOSparkMax() {
    SparkBaseConfig intakeDriveConfig =
        new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(IntakeConstants.intakeDriveCurrentLimitAmps)
            .voltageCompensation(12.0);
    SparkBaseConfig intakePivotConfig =
        new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(IntakeConstants.intakePivotCurrentLimitAmps)
            .voltageCompensation(12.0);
    intakeDrive.configure(
        intakeDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    intakePivot.configure(
        intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.pivotConnected = true;
    inputs.drivePositionRotations = intakeDriveEncoder.getPosition();
    inputs.driveVelocityRotationsPerSec = intakeDriveEncoder.getVelocity() / 60.0;
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
  public void setPivotOutput(double output) {
    intakePivot.set(output);
  }
}
