package frc.robot.subsystems.body;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NetworkTablesUtil;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private double targetIntakeSpeed = IntakeConstants.defaultIntakeSpeed;
  private Command currentIntakeRunCommand;
  public boolean intakeRunning = false;
  private double lastAppliedIntakeDriveSpeed = 0.0;
  private double lastAppliedIntakePivotSpeed = 0.0;
  private double intakePivotRetractedPositionRotations =
      IntakeConstants.defaultIntakePivotRetractedPositionRotations;
  private double intakePivotExtendedPositionRotations =
      IntakeConstants.defaultIntakePivotExtendedPositionRotations;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final NetworkTable subsystemTable =
      NetworkTablesUtil.subsystemTable(IntakeConstants.configTableName);
  private final NetworkTable tuningTable = NetworkTablesUtil.tuningCommonTable(subsystemTable);
  private final NetworkTable telemetryTable = NetworkTablesUtil.telemetryTable(subsystemTable);

  private final NetworkTableEntry intakeSpeedEntry = tuningTable.getEntry("Drive/Speed");
  private final NetworkTableEntry intakePivotSpeedScaleEntry =
      tuningTable.getEntry("Pivot/SpeedScale");
  private final NetworkTableEntry intakePivotInvertedEntry = tuningTable.getEntry("Pivot/Inverted");
  private final NetworkTableEntry intakeDriveDirectionEntry =
      tuningTable.getEntry("Drive/Direction");
  private final NetworkTableEntry intakePivotRetractedPositionEntry =
      tuningTable.getEntry("Pivot/Calibration/RetractedPositionRotations");
  private final NetworkTableEntry intakePivotExtendedPositionEntry =
      tuningTable.getEntry("Pivot/Calibration/ExtendedPositionRotations");
  private final NetworkTableEntry intakePivotEncoderPositionEntry =
      telemetryTable.getEntry("Pivot/EncoderPositionRotations");
  private final NetworkTableEntry intakePivotEncoderVelocityEntry =
      telemetryTable.getEntry("Pivot/EncoderVelocityRpm");
  private final NetworkTableEntry intakePivotEncoderNormalizedPositionEntry =
      telemetryTable.getEntry("Pivot/EncoderPositionNormalized");
  private final NetworkTableEntry intakeDriveAppliedOutputEntry =
      telemetryTable.getEntry("Drive/AppliedOutput");
  private final NetworkTableEntry intakePivotAppliedOutputEntry =
      telemetryTable.getEntry("Pivot/AppliedOutput");
  private final NetworkTableEntry intakeDriveEstimatedVelocityEntry =
      telemetryTable.getEntry("Drive/EstimatedVelocityRotationsPerSec");
  private final NetworkTableEntry intakeDriveEstimatedPositionEntry =
      telemetryTable.getEntry("Drive/EstimatedPositionRotations");

  public Intake() {
    this(new IntakeIO() {});
  }

  public Intake(IntakeIO io) {
    this.io = io;
    configureNetworkTableDefaults();
    loadNetworkTableConfig();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    loadNetworkTableConfig();
    publishPivotEncoderToNetworkTables();
    publishActuatorStateToNetworkTables();
    logTelemetry();
  }

  public void updateIntake() {
    lastAppliedIntakeDriveSpeed =
        applyDirection(
            targetIntakeSpeed,
            intakeDriveDirectionEntry,
            IntakeConstants.defaultIntakeDriveDirection);
    io.setDriveOutput(lastAppliedIntakeDriveSpeed);
    intakeRunning = Math.abs(lastAppliedIntakeDriveSpeed) > 1e-3;
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
    return inputs.driveAppliedOutput;
  }

  public void stopIntake() {
    lastAppliedIntakeDriveSpeed = 0.0;
    io.setDriveOutput(0.0);
    intakeRunning = false;
  }

  public void setIntakePivotSpeed(double speed) {
    lastAppliedIntakePivotSpeed =
        applyScaleAndInversion(speed, intakePivotSpeedScaleEntry, intakePivotInvertedEntry);
    io.setPivotOutput(lastAppliedIntakePivotSpeed);
  }

  public void stopIntakePivot() {
    lastAppliedIntakePivotSpeed = 0.0;
    io.setPivotOutput(0.0);
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
        normalizeDirection(
            intakeDriveDirectionEntry.getDouble(IntakeConstants.defaultIntakeDriveDirection));
    intakeDriveDirectionEntry.setDouble(intakeDriveDirection);
  }

  private double applyScaleAndInversion(
      double speed, NetworkTableEntry speedScaleEntry, NetworkTableEntry invertedEntry) {
    double scaledSpeed = clampSpeed(speed) * clampSpeedScale(speedScaleEntry.getDouble(1.0));
    return applyInversion(scaledSpeed, invertedEntry);
  }

  private double applyDirection(
      double speed, NetworkTableEntry directionEntry, double defaultDirection) {
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
    intakePivotEncoderVelocityEntry.setDouble(inputs.pivotVelocityRpm);
    intakePivotEncoderNormalizedPositionEntry.setDouble(getIntakePivotMeasuredPositionNormalized());
  }

  private void publishActuatorStateToNetworkTables() {
    intakeDriveAppliedOutputEntry.setDouble(inputs.driveAppliedOutput);
    intakeDriveEstimatedVelocityEntry.setDouble(inputs.driveVelocityRotationsPerSec);
    intakeDriveEstimatedPositionEntry.setDouble(inputs.drivePositionRotations);
    intakePivotAppliedOutputEntry.setDouble(inputs.pivotAppliedOutput);
  }

  public double getIntakePivotMeasuredPositionRotations() {
    return inputs.pivotPositionRotations;
  }

  public double getIntakeDriveCurrentAmps() {
    return inputs.driveCurrentAmps;
  }

  public double getIntakePivotCurrentAmps() {
    return inputs.pivotCurrentAmps;
  }

  public double getIntakePivotMeasuredPositionNormalized() {
    return clampUnitInterval(
        inverseInterpolate(
            getIntakePivotMeasuredPositionRotations(),
            intakePivotRetractedPositionRotations,
            intakePivotExtendedPositionRotations));
  }

  public void resetSimulationState() {
    io.resetSimulationState();
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
        normalizeDirection(
            intakeDriveDirectionEntry.getDouble(IntakeConstants.defaultIntakeDriveDirection)));
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
    Logger.recordOutput("Intake/State/ActualDriveOutput", inputs.driveAppliedOutput);
    Logger.recordOutput("Intake/State/ActualPivotOutput", inputs.pivotAppliedOutput);
    Logger.recordOutput("Intake/Measured/DriveCurrentAmps", inputs.driveCurrentAmps);
    Logger.recordOutput("Intake/Measured/PivotCurrentAmps", inputs.pivotCurrentAmps);
    Logger.recordOutput(
        "Intake/Measured/PivotEncoderPositionRotations", getIntakePivotMeasuredPositionRotations());
    Logger.recordOutput("Intake/Measured/PivotEncoderVelocityRpm", inputs.pivotVelocityRpm);
    Logger.recordOutput(
        "Intake/Measured/PivotEncoderPositionNormalized",
        getIntakePivotMeasuredPositionNormalized());
  }
}
