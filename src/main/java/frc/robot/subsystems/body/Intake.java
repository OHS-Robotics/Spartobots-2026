package frc.robot.subsystems.body;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final double loopPeriodSeconds = 0.02;
  private static final double intakeDriveEstimatedMaxVelocityRotationsPerSec = 12.0;

  private double targetIntakeSpeed = IntakeConstants.defaultIntakeSpeed;
  private Command currentIntakeRunCommand;
  public boolean intakeRunning = false;
  private double lastAppliedIntakeDriveSpeed = 0.0;
  private double lastAppliedIntakePivotSpeed = 0.0;
  private double intakeDriveEstimatedPositionRotations = 0.0;
  private double intakePivotRetractedPositionRotations =
      IntakeConstants.defaultIntakePivotRetractedPositionRotations;
  private double intakePivotExtendedPositionRotations =
      IntakeConstants.defaultIntakePivotExtendedPositionRotations;

  private final SparkMax intakeDrive =
      new SparkMax(IntakeConstants.intakeDriveCanId, MotorType.kBrushless);
  private final SparkMax intakePivot =
      new SparkMax(IntakeConstants.intakePivotCanId, MotorType.kBrushless);
  private final RelativeEncoder intakePivotEncoder = intakePivot.getEncoder();

  private final NetworkTable configTable =
      NetworkTableInstance.getDefault().getTable(IntakeConstants.configTableName);
  private final NetworkTableEntry intakeSpeedEntry = configTable.getEntry("IntakeDrive/Speed");
  private final NetworkTableEntry intakePivotSpeedScaleEntry =
      configTable.getEntry("IntakePivot/SpeedScale");
  private final NetworkTableEntry intakePivotInvertedEntry =
      configTable.getEntry("IntakePivot/Inverted");
  private final NetworkTableEntry intakeDriveDirectionEntry =
      configTable.getEntry("IntakeDrive/Direction");
  private final NetworkTableEntry intakePivotRetractedPositionEntry =
      configTable.getEntry("IntakePivot/Calibration/RetractedPositionRotations");
  private final NetworkTableEntry intakePivotExtendedPositionEntry =
      configTable.getEntry("IntakePivot/Calibration/ExtendedPositionRotations");
  private final NetworkTableEntry intakePivotEncoderPositionEntry =
      configTable.getEntry("IntakePivot/EncoderPositionRotations");
  private final NetworkTableEntry intakePivotEncoderVelocityEntry =
      configTable.getEntry("IntakePivot/EncoderVelocityRpm");
  private final NetworkTableEntry intakePivotEncoderNormalizedPositionEntry =
      configTable.getEntry("IntakePivot/EncoderPositionNormalized");
  private final NetworkTableEntry intakeDriveAppliedOutputEntry =
      configTable.getEntry("IntakeDrive/AppliedOutput");
  private final NetworkTableEntry intakePivotAppliedOutputEntry =
      configTable.getEntry("IntakePivot/AppliedOutput");
  private final NetworkTableEntry intakeDriveEstimatedVelocityEntry =
      configTable.getEntry("IntakeDrive/EstimatedVelocityRotationsPerSec");
  private final NetworkTableEntry intakeDriveEstimatedPositionEntry =
      configTable.getEntry("IntakeDrive/EstimatedPositionRotations");

  public Intake() {
    SparkBaseConfig brakeConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

    configureNetworkTableDefaults();
    loadNetworkTableConfig();

    intakeDrive.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    intakePivot.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    loadNetworkTableConfig();
    publishPivotEncoderToNetworkTables();
    publishActuatorStateToNetworkTables();
    logTelemetry();
  }

  public void updateIntake() {
    lastAppliedIntakeDriveSpeed =
        applyDirection(targetIntakeSpeed, intakeDriveDirectionEntry, IntakeConstants.defaultIntakeDriveDirection);
    intakeDrive.set(lastAppliedIntakeDriveSpeed);
  }

  public void reverseTargetSpeed() {
    targetIntakeSpeed *= -1;
    intakeSpeedEntry.setDouble(targetIntakeSpeed);
  }

  public void setTargetIntakeSpeed(double speed) {
    targetIntakeSpeed = clampSpeed(speed);
    intakeSpeedEntry.setDouble(targetIntakeSpeed);
  }

  public double getTargetIntakeSpeed() {
    return targetIntakeSpeed;
  }

  public double getActualIntakeSpeed() {
    return intakeDrive.get();
  }

  public void stopIntake() {
    lastAppliedIntakeDriveSpeed = 0.0;
    intakeDrive.set(0.0);
  }

  public void setIntakePivotSpeed(double speed) {
    lastAppliedIntakePivotSpeed =
        applyScaleAndInversion(speed, intakePivotSpeedScaleEntry, intakePivotInvertedEntry);
    intakePivot.set(lastAppliedIntakePivotSpeed);
  }

  public void stopIntakePivot() {
    lastAppliedIntakePivotSpeed = 0.0;
    intakePivot.set(0.0);
  }

  public Command toggleIntakeCommand() {
    return runOnce(
        () -> {
          if (!intakeRunning) {
            updateIntake();
            currentIntakeRunCommand = run(this::updateIntake);
            CommandScheduler.getInstance().schedule(currentIntakeRunCommand);
            intakeRunning = true;
          } else {
            stopIntake();
            stopCommand(currentIntakeRunCommand);
            currentIntakeRunCommand = null;
            intakeRunning = false;
          }
        });
  }

  public Command increaseIntakeSpeed() {
    return Commands.runOnce(
        () -> {
          targetIntakeSpeed = Math.min(targetIntakeSpeed + 0.05, 1.0);
          intakeSpeedEntry.setDouble(targetIntakeSpeed);
        });
  }

  public Command decreaseIntakeSpeed() {
    return Commands.runOnce(
        () -> {
          targetIntakeSpeed = Math.max(targetIntakeSpeed - 0.05, 0.0);
          intakeSpeedEntry.setDouble(targetIntakeSpeed);
        });
  }

  private void configureNetworkTableDefaults() {
    intakeSpeedEntry.setDefaultDouble(IntakeConstants.defaultIntakeSpeed);
    intakePivotSpeedScaleEntry.setDefaultDouble(IntakeConstants.defaultIntakePivotSpeedScale);
    intakePivotRetractedPositionEntry.setDefaultDouble(
        IntakeConstants.defaultIntakePivotRetractedPositionRotations);
    intakePivotExtendedPositionEntry.setDefaultDouble(
        IntakeConstants.defaultIntakePivotExtendedPositionRotations);
    intakeDriveDirectionEntry.setDefaultDouble(IntakeConstants.defaultIntakeDriveDirection);

    intakePivotInvertedEntry.setDefaultBoolean(IntakeConstants.defaultIntakePivotInverted);
  }

  private void loadNetworkTableConfig() {
    targetIntakeSpeed = clampSpeed(intakeSpeedEntry.getDouble(targetIntakeSpeed));
    intakeSpeedEntry.setDouble(targetIntakeSpeed);
    intakePivotSpeedScaleEntry.setDouble(
        clampSpeedScale(
            intakePivotSpeedScaleEntry.getDouble(IntakeConstants.defaultIntakePivotSpeedScale)));
    intakePivotRetractedPositionRotations =
        intakePivotRetractedPositionEntry.getDouble(intakePivotRetractedPositionRotations);
    intakePivotExtendedPositionRotations =
        intakePivotExtendedPositionEntry.getDouble(intakePivotExtendedPositionRotations);
    intakePivotRetractedPositionEntry.setDouble(intakePivotRetractedPositionRotations);
    intakePivotExtendedPositionEntry.setDouble(intakePivotExtendedPositionRotations);
    double intakeDriveDirection =
        normalizeDirection(intakeDriveDirectionEntry.getDouble(IntakeConstants.defaultIntakeDriveDirection));
    intakeDriveDirectionEntry.setDouble(intakeDriveDirection);
  }

  private double applyScaleAndInversion(
      double speed, NetworkTableEntry speedScaleEntry, NetworkTableEntry invertedEntry) {
    double scaledSpeed = clampSpeed(speed) * clampSpeedScale(speedScaleEntry.getDouble(1.0));
    return applyInversion(scaledSpeed, invertedEntry);
  }

  private double applyDirection(double speed, NetworkTableEntry directionEntry, double defaultDirection) {
    return clampSpeed(speed) * normalizeDirection(directionEntry.getDouble(defaultDirection));
  }

  private double applyInversion(double speed, NetworkTableEntry invertedEntry) {
    return invertedEntry.getBoolean(false) ? -speed : speed;
  }

  private double clampSpeed(double speed) {
    return MathUtil.clamp(speed, -1.0, 1.0);
  }

  private double clampSpeedScale(double speedScale) {
    return MathUtil.clamp(speedScale, 0.0, 1.0);
  }

  private void publishPivotEncoderToNetworkTables() {
    intakePivotEncoderPositionEntry.setDouble(getIntakePivotMeasuredPositionRotations());
    intakePivotEncoderVelocityEntry.setDouble(intakePivotEncoder.getVelocity());
    intakePivotEncoderNormalizedPositionEntry.setDouble(getIntakePivotMeasuredPositionNormalized());
  }

  private void publishActuatorStateToNetworkTables() {
    double driveAppliedOutput = intakeDrive.get();
    double estimatedDriveVelocityRotationsPerSec =
        driveAppliedOutput * intakeDriveEstimatedMaxVelocityRotationsPerSec;
    intakeDriveEstimatedPositionRotations +=
        estimatedDriveVelocityRotationsPerSec * loopPeriodSeconds;

    intakeDriveAppliedOutputEntry.setDouble(driveAppliedOutput);
    intakeDriveEstimatedVelocityEntry.setDouble(estimatedDriveVelocityRotationsPerSec);
    intakeDriveEstimatedPositionEntry.setDouble(intakeDriveEstimatedPositionRotations);
    intakePivotAppliedOutputEntry.setDouble(intakePivot.get());
  }

  public double getIntakePivotMeasuredPositionRotations() {
    return intakePivotEncoder.getPosition();
  }

  public double getIntakePivotMeasuredPositionNormalized() {
    return clampUnitInterval(
        inverseInterpolate(
            getIntakePivotMeasuredPositionRotations(),
            intakePivotRetractedPositionRotations,
            intakePivotExtendedPositionRotations));
  }

  private static double inverseInterpolate(double value, double start, double end) {
    double span = end - start;
    if (Math.abs(span) < 1e-6) {
      return 0.0;
    }
    return (value - start) / span;
  }

  private static double clampUnitInterval(double value) {
    return MathUtil.clamp(value, 0.0, 1.0);
  }

  private static double normalizeDirection(double direction) {
    return direction < 0.0 ? -1.0 : 1.0;
  }

  private void stopCommand(Command command) {
    if (command != null) {
      CommandScheduler.getInstance().cancel(command);
    }
  }

  private void logTelemetry() {
    Logger.recordOutput("Intake/Config/TargetSpeed", targetIntakeSpeed);
    Logger.recordOutput(
        "Intake/Config/PivotSpeedScale",
        clampSpeedScale(
            intakePivotSpeedScaleEntry.getDouble(IntakeConstants.defaultIntakePivotSpeedScale)));
    Logger.recordOutput(
        "Intake/Config/DriveDirection",
        normalizeDirection(intakeDriveDirectionEntry.getDouble(IntakeConstants.defaultIntakeDriveDirection)));
    Logger.recordOutput(
        "Intake/Config/PivotInverted",
        intakePivotInvertedEntry.getBoolean(IntakeConstants.defaultIntakePivotInverted));
    Logger.recordOutput(
        "Intake/Config/PivotRetractedPositionRotations", intakePivotRetractedPositionRotations);
    Logger.recordOutput(
        "Intake/Config/PivotExtendedPositionRotations", intakePivotExtendedPositionRotations);

    Logger.recordOutput("Intake/State/Running", intakeRunning);
    Logger.recordOutput("Intake/State/LastAppliedDriveOutput", lastAppliedIntakeDriveSpeed);
    Logger.recordOutput("Intake/State/LastAppliedPivotOutput", lastAppliedIntakePivotSpeed);
    Logger.recordOutput("Intake/State/ActualDriveOutput", intakeDrive.get());
    Logger.recordOutput("Intake/State/ActualPivotOutput", intakePivot.get());
    Logger.recordOutput("Intake/Measured/DriveCurrentAmps", intakeDrive.getOutputCurrent());
    Logger.recordOutput("Intake/Measured/PivotCurrentAmps", intakePivot.getOutputCurrent());
    Logger.recordOutput(
        "Intake/Measured/PivotEncoderPositionRotations", getIntakePivotMeasuredPositionRotations());
    Logger.recordOutput("Intake/Measured/PivotEncoderVelocityRpm", intakePivotEncoder.getVelocity());
    Logger.recordOutput(
        "Intake/Measured/PivotEncoderPositionNormalized",
        getIntakePivotMeasuredPositionNormalized());
  }
}
