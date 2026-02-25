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

public class Hopper extends SubsystemBase {
  private static final double loopPeriodSeconds = 0.02;
  private static final double hopperAgitatorEstimatedMaxVelocityRotationsPerSec = 12.0;

  private double targetAgitatorSpeed = HopperConstants.defaultHopperAgitatorSpeed;
  private Command currentAgitatorRunCommand;
  private boolean agitatorRunning = false;
  private double lastAppliedAgitatorSpeed = 0.0;
  private double lastAppliedExtensionSpeed = 0.0;
  private double hopperAgitatorEstimatedPositionRotations = 0.0;
  private double hopperExtensionRetractedPositionRotations =
      HopperConstants.defaultHopperExtensionRetractedPositionRotations;
  private double hopperExtensionExtendedPositionRotations =
      HopperConstants.defaultHopperExtensionExtendedPositionRotations;

  private final SparkMax hopperAgitator =
      new SparkMax(HopperConstants.hopperAgitatorDriveCanId, MotorType.kBrushed);
  private final SparkMax hopperExtension =
      new SparkMax(HopperConstants.hopperExtensionCanId, MotorType.kBrushed);
  private final RelativeEncoder hopperExtensionEncoder = hopperExtension.getEncoder();

  private final NetworkTable subsystemTable =
      NetworkTableInstance.getDefault().getTable(HopperConstants.configTableName);
  private final NetworkTable tuningTable = subsystemTable.getSubTable("Tuning");
  private final NetworkTable telemetryTable = subsystemTable.getSubTable("Telemetry");

  private final NetworkTableEntry hopperAgitatorSpeedEntry = tuningTable.getEntry("Agitator/Speed");
  private final NetworkTableEntry hopperExtensionSpeedScaleEntry =
      tuningTable.getEntry("Extension/SpeedScale");
  private final NetworkTableEntry hopperExtensionInvertedEntry =
      tuningTable.getEntry("Extension/Inverted");
  private final NetworkTableEntry hopperAgitatorDirectionEntry =
      tuningTable.getEntry("Agitator/Direction");
  private final NetworkTableEntry hopperExtensionRetractedPositionEntry =
      tuningTable.getEntry("Extension/Calibration/RetractedPositionRotations");
  private final NetworkTableEntry hopperExtensionExtendedPositionEntry =
      tuningTable.getEntry("Extension/Calibration/ExtendedPositionRotations");
  private final NetworkTableEntry hopperExtensionEncoderPositionEntry =
      telemetryTable.getEntry("Extension/EncoderPositionRotations");
  private final NetworkTableEntry hopperExtensionEncoderVelocityEntry =
      telemetryTable.getEntry("Extension/EncoderVelocityRpm");
  private final NetworkTableEntry hopperExtensionEncoderNormalizedPositionEntry =
      telemetryTable.getEntry("Extension/EncoderPositionNormalized");
  private final NetworkTableEntry hopperAgitatorAppliedOutputEntry =
      telemetryTable.getEntry("Agitator/AppliedOutput");
  private final NetworkTableEntry hopperExtensionAppliedOutputEntry =
      telemetryTable.getEntry("Extension/AppliedOutput");
  private final NetworkTableEntry hopperAgitatorEstimatedVelocityEntry =
      telemetryTable.getEntry("Agitator/EstimatedVelocityRotationsPerSec");
  private final NetworkTableEntry hopperAgitatorEstimatedPositionEntry =
      telemetryTable.getEntry("Agitator/EstimatedPositionRotations");

  public Hopper() {
    SparkBaseConfig brakeConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

    configureNetworkTableDefaults();
    loadNetworkTableConfig();

    hopperAgitator.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    hopperExtension.configure(
        brakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    loadNetworkTableConfig();
    publishExtensionEncoderToNetworkTables();
    publishActuatorStateToNetworkTables();
    logTelemetry();
  }

  public void updateAgitator() {
    lastAppliedAgitatorSpeed =
        applyDirection(
            targetAgitatorSpeed,
            hopperAgitatorDirectionEntry,
            HopperConstants.defaultHopperAgitatorDirection);
    hopperAgitator.set(lastAppliedAgitatorSpeed);
    agitatorRunning = Math.abs(lastAppliedAgitatorSpeed) > 1e-3;
  }

  public void reverseAgitatorSpeed() {
    targetAgitatorSpeed *= -1;
    hopperAgitatorSpeedEntry.setDouble(targetAgitatorSpeed);
  }

  public void setTargetAgitatorSpeed(double speed) {
    targetAgitatorSpeed = clampSpeed(speed);
    hopperAgitatorSpeedEntry.setDouble(targetAgitatorSpeed);
  }

  public double getTargetAgitatorSpeed() {
    return targetAgitatorSpeed;
  }

  public double getActualAgitatorSpeed() {
    return hopperAgitator.get();
  }

  public void stopAgitator() {
    lastAppliedAgitatorSpeed = 0.0;
    hopperAgitator.set(0.0);
    agitatorRunning = false;
  }

  public void setHopperExtensionSpeed(double speed) {
    lastAppliedExtensionSpeed =
        applyScaleAndInversion(speed, hopperExtensionSpeedScaleEntry, hopperExtensionInvertedEntry);
    hopperExtension.set(lastAppliedExtensionSpeed);
  }

  public void stopHopperExtension() {
    lastAppliedExtensionSpeed = 0.0;
    hopperExtension.set(0.0);
  }

  public Command toggleAgitatorCommand() {
    return runOnce(
        () -> {
          if (!agitatorRunning) {
            updateAgitator();
            currentAgitatorRunCommand = run(this::updateAgitator);
            CommandScheduler.getInstance().schedule(currentAgitatorRunCommand);
            agitatorRunning = true;
          } else {
            stopAgitator();
            stopCommand(currentAgitatorRunCommand);
            currentAgitatorRunCommand = null;
            agitatorRunning = false;
          }
        });
  }

  public Command increaseAgitatorSpeed() {
    return Commands.runOnce(
        () -> {
          targetAgitatorSpeed = Math.min(targetAgitatorSpeed + 0.05, 1.0);
          hopperAgitatorSpeedEntry.setDouble(targetAgitatorSpeed);
        });
  }

  public Command decreaseAgitatorSpeed() {
    return Commands.runOnce(
        () -> {
          targetAgitatorSpeed = Math.max(targetAgitatorSpeed - 0.05, 0.0);
          hopperAgitatorSpeedEntry.setDouble(targetAgitatorSpeed);
        });
  }

  private void configureNetworkTableDefaults() {
    hopperAgitatorSpeedEntry.setDefaultDouble(HopperConstants.defaultHopperAgitatorSpeed);
    hopperExtensionSpeedScaleEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionSpeedScale);
    hopperExtensionRetractedPositionEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionRetractedPositionRotations);
    hopperExtensionExtendedPositionEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionExtendedPositionRotations);
    hopperAgitatorDirectionEntry.setDefaultDouble(HopperConstants.defaultHopperAgitatorDirection);

    hopperExtensionInvertedEntry.setDefaultBoolean(HopperConstants.defaultHopperExtensionInverted);
  }

  private void loadNetworkTableConfig() {
    targetAgitatorSpeed = clampSpeed(hopperAgitatorSpeedEntry.getDouble(targetAgitatorSpeed));
    hopperAgitatorSpeedEntry.setDouble(targetAgitatorSpeed);
    hopperExtensionSpeedScaleEntry.setDouble(
        clampSpeedScale(
            hopperExtensionSpeedScaleEntry.getDouble(
                HopperConstants.defaultHopperExtensionSpeedScale)));
    hopperExtensionRetractedPositionRotations =
        hopperExtensionRetractedPositionEntry.getDouble(hopperExtensionRetractedPositionRotations);
    hopperExtensionExtendedPositionRotations =
        hopperExtensionExtendedPositionEntry.getDouble(hopperExtensionExtendedPositionRotations);
    hopperExtensionRetractedPositionEntry.setDouble(hopperExtensionRetractedPositionRotations);
    hopperExtensionExtendedPositionEntry.setDouble(hopperExtensionExtendedPositionRotations);
    double hopperAgitatorDirection =
        normalizeDirection(
            hopperAgitatorDirectionEntry.getDouble(HopperConstants.defaultHopperAgitatorDirection));
    hopperAgitatorDirectionEntry.setDouble(hopperAgitatorDirection);
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

  private void publishExtensionEncoderToNetworkTables() {
    hopperExtensionEncoderPositionEntry.setDouble(getHopperExtensionMeasuredPositionRotations());
    hopperExtensionEncoderVelocityEntry.setDouble(hopperExtensionEncoder.getVelocity());
    hopperExtensionEncoderNormalizedPositionEntry.setDouble(
        getHopperExtensionMeasuredPositionNormalized());
  }

  private void publishActuatorStateToNetworkTables() {
    double agitatorAppliedOutput = hopperAgitator.get();
    double estimatedAgitatorVelocityRotationsPerSec =
        agitatorAppliedOutput * hopperAgitatorEstimatedMaxVelocityRotationsPerSec;
    hopperAgitatorEstimatedPositionRotations +=
        estimatedAgitatorVelocityRotationsPerSec * loopPeriodSeconds;

    hopperAgitatorAppliedOutputEntry.setDouble(agitatorAppliedOutput);
    hopperAgitatorEstimatedVelocityEntry.setDouble(estimatedAgitatorVelocityRotationsPerSec);
    hopperAgitatorEstimatedPositionEntry.setDouble(hopperAgitatorEstimatedPositionRotations);
    hopperExtensionAppliedOutputEntry.setDouble(hopperExtension.get());
  }

  public double getHopperExtensionMeasuredPositionRotations() {
    return hopperExtensionEncoder.getPosition();
  }

  public double getHopperAgitatorCurrentAmps() {
    return hopperAgitator.getOutputCurrent();
  }

  public double getHopperExtensionCurrentAmps() {
    return hopperExtension.getOutputCurrent();
  }

  public double getHopperExtensionMeasuredPositionNormalized() {
    return clampUnitInterval(
        inverseInterpolate(
            getHopperExtensionMeasuredPositionRotations(),
            hopperExtensionRetractedPositionRotations,
            hopperExtensionExtendedPositionRotations));
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
    Logger.recordOutput("Hopper/Config/TargetAgitatorSpeed", targetAgitatorSpeed);
    Logger.recordOutput(
        "Hopper/Config/ExtensionSpeedScale",
        clampSpeedScale(
            hopperExtensionSpeedScaleEntry.getDouble(
                HopperConstants.defaultHopperExtensionSpeedScale)));
    Logger.recordOutput(
        "Hopper/Config/AgitatorDirection",
        normalizeDirection(
            hopperAgitatorDirectionEntry.getDouble(
                HopperConstants.defaultHopperAgitatorDirection)));
    Logger.recordOutput(
        "Hopper/Config/ExtensionInverted",
        hopperExtensionInvertedEntry.getBoolean(HopperConstants.defaultHopperExtensionInverted));
    Logger.recordOutput(
        "Hopper/Config/ExtensionRetractedPositionRotations",
        hopperExtensionRetractedPositionRotations);
    Logger.recordOutput(
        "Hopper/Config/ExtensionExtendedPositionRotations",
        hopperExtensionExtendedPositionRotations);

    Logger.recordOutput("Hopper/State/AgitatorRunning", agitatorRunning);
    Logger.recordOutput("Hopper/State/LastAppliedAgitatorOutput", lastAppliedAgitatorSpeed);
    Logger.recordOutput("Hopper/State/LastAppliedExtensionOutput", lastAppliedExtensionSpeed);
    Logger.recordOutput("Hopper/State/ActualAgitatorOutput", hopperAgitator.get());
    Logger.recordOutput("Hopper/State/ActualExtensionOutput", hopperExtension.get());
    Logger.recordOutput("Hopper/Measured/AgitatorCurrentAmps", hopperAgitator.getOutputCurrent());
    Logger.recordOutput("Hopper/Measured/ExtensionCurrentAmps", hopperExtension.getOutputCurrent());
    Logger.recordOutput(
        "Hopper/Measured/ExtensionEncoderPositionRotations",
        getHopperExtensionMeasuredPositionRotations());
    Logger.recordOutput(
        "Hopper/Measured/ExtensionEncoderVelocityRpm", hopperExtensionEncoder.getVelocity());
    Logger.recordOutput(
        "Hopper/Measured/ExtensionEncoderPositionNormalized",
        getHopperExtensionMeasuredPositionNormalized());
  }
}
