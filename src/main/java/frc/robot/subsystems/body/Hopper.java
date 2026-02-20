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
  private static final double hopperBeltEstimatedMaxVelocityRotationsPerSec = 12.0;

  private double targetBeltSpeed = HopperConstants.defaultHopperBeltSpeed;
  private Command currentBeltRunCommand;
  private boolean beltRunning = false;
  private double lastAppliedBeltSpeed = 0.0;
  private double lastAppliedExtensionSpeed = 0.0;
  private double hopperBeltEstimatedPositionRotations = 0.0;
  private double hopperExtensionRetractedPositionRotations =
      HopperConstants.defaultHopperExtensionRetractedPositionRotations;
  private double hopperExtensionExtendedPositionRotations =
      HopperConstants.defaultHopperExtensionExtendedPositionRotations;

  private final SparkMax hopperBelt =
      new SparkMax(HopperConstants.hopperBeltDriveCanId, MotorType.kBrushed);
  private final SparkMax hopperExtension =
      new SparkMax(HopperConstants.hopperExtensionCanId, MotorType.kBrushed);
  private final RelativeEncoder hopperExtensionEncoder = hopperExtension.getEncoder();

  private final NetworkTable subsystemTable =
      NetworkTableInstance.getDefault().getTable(HopperConstants.configTableName);
  private final NetworkTable tuningTable = subsystemTable.getSubTable("Tuning");
  private final NetworkTable telemetryTable = subsystemTable.getSubTable("Telemetry");

  private final NetworkTableEntry hopperBeltSpeedEntry = tuningTable.getEntry("Belt/Speed");
  private final NetworkTableEntry hopperExtensionSpeedScaleEntry =
      tuningTable.getEntry("Extension/SpeedScale");
  private final NetworkTableEntry hopperExtensionInvertedEntry =
      tuningTable.getEntry("Extension/Inverted");
  private final NetworkTableEntry hopperBeltDirectionEntry = tuningTable.getEntry("Belt/Direction");
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
  private final NetworkTableEntry hopperBeltAppliedOutputEntry =
      telemetryTable.getEntry("Belt/AppliedOutput");
  private final NetworkTableEntry hopperExtensionAppliedOutputEntry =
      telemetryTable.getEntry("Extension/AppliedOutput");
  private final NetworkTableEntry hopperBeltEstimatedVelocityEntry =
      telemetryTable.getEntry("Belt/EstimatedVelocityRotationsPerSec");
  private final NetworkTableEntry hopperBeltEstimatedPositionEntry =
      telemetryTable.getEntry("Belt/EstimatedPositionRotations");

  public Hopper() {
    SparkBaseConfig brakeConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

    configureNetworkTableDefaults();
    loadNetworkTableConfig();

    hopperBelt.configure(
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

  public void updateBelt() {
    lastAppliedBeltSpeed =
        applyDirection(
            targetBeltSpeed, hopperBeltDirectionEntry, HopperConstants.defaultHopperBeltDirection);
    hopperBelt.set(lastAppliedBeltSpeed);
    beltRunning = Math.abs(lastAppliedBeltSpeed) > 1e-3;
  }

  public void reverseBeltSpeed() {
    targetBeltSpeed *= -1;
    hopperBeltSpeedEntry.setDouble(targetBeltSpeed);
  }

  public void setTargetBeltSpeed(double speed) {
    targetBeltSpeed = clampSpeed(speed);
    hopperBeltSpeedEntry.setDouble(targetBeltSpeed);
  }

  public double getTargetBeltSpeed() {
    return targetBeltSpeed;
  }

  public double getActualBeltSpeed() {
    return hopperBelt.get();
  }

  public void stopBelt() {
    lastAppliedBeltSpeed = 0.0;
    hopperBelt.set(0.0);
    beltRunning = false;
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

  public Command toggleBeltCommand() {
    return runOnce(
        () -> {
          if (!beltRunning) {
            updateBelt();
            currentBeltRunCommand = run(this::updateBelt);
            CommandScheduler.getInstance().schedule(currentBeltRunCommand);
            beltRunning = true;
          } else {
            stopBelt();
            stopCommand(currentBeltRunCommand);
            currentBeltRunCommand = null;
            beltRunning = false;
          }
        });
  }

  public Command increaseBeltSpeed() {
    return Commands.runOnce(
        () -> {
          targetBeltSpeed = Math.min(targetBeltSpeed + 0.05, 1.0);
          hopperBeltSpeedEntry.setDouble(targetBeltSpeed);
        });
  }

  public Command decreaseBeltSpeed() {
    return Commands.runOnce(
        () -> {
          targetBeltSpeed = Math.max(targetBeltSpeed - 0.05, 0.0);
          hopperBeltSpeedEntry.setDouble(targetBeltSpeed);
        });
  }

  private void configureNetworkTableDefaults() {
    hopperBeltSpeedEntry.setDefaultDouble(HopperConstants.defaultHopperBeltSpeed);
    hopperExtensionSpeedScaleEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionSpeedScale);
    hopperExtensionRetractedPositionEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionRetractedPositionRotations);
    hopperExtensionExtendedPositionEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionExtendedPositionRotations);
    hopperBeltDirectionEntry.setDefaultDouble(HopperConstants.defaultHopperBeltDirection);

    hopperExtensionInvertedEntry.setDefaultBoolean(HopperConstants.defaultHopperExtensionInverted);
  }

  private void loadNetworkTableConfig() {
    targetBeltSpeed = clampSpeed(hopperBeltSpeedEntry.getDouble(targetBeltSpeed));
    hopperBeltSpeedEntry.setDouble(targetBeltSpeed);
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
    double hopperBeltDirection =
        normalizeDirection(
            hopperBeltDirectionEntry.getDouble(HopperConstants.defaultHopperBeltDirection));
    hopperBeltDirectionEntry.setDouble(hopperBeltDirection);
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
    double beltAppliedOutput = hopperBelt.get();
    double estimatedBeltVelocityRotationsPerSec =
        beltAppliedOutput * hopperBeltEstimatedMaxVelocityRotationsPerSec;
    hopperBeltEstimatedPositionRotations +=
        estimatedBeltVelocityRotationsPerSec * loopPeriodSeconds;

    hopperBeltAppliedOutputEntry.setDouble(beltAppliedOutput);
    hopperBeltEstimatedVelocityEntry.setDouble(estimatedBeltVelocityRotationsPerSec);
    hopperBeltEstimatedPositionEntry.setDouble(hopperBeltEstimatedPositionRotations);
    hopperExtensionAppliedOutputEntry.setDouble(hopperExtension.get());
  }

  public double getHopperExtensionMeasuredPositionRotations() {
    return hopperExtensionEncoder.getPosition();
  }

  public double getHopperBeltCurrentAmps() {
    return hopperBelt.getOutputCurrent();
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
    Logger.recordOutput("Hopper/Config/TargetBeltSpeed", targetBeltSpeed);
    Logger.recordOutput(
        "Hopper/Config/ExtensionSpeedScale",
        clampSpeedScale(
            hopperExtensionSpeedScaleEntry.getDouble(
                HopperConstants.defaultHopperExtensionSpeedScale)));
    Logger.recordOutput(
        "Hopper/Config/BeltDirection",
        normalizeDirection(
            hopperBeltDirectionEntry.getDouble(HopperConstants.defaultHopperBeltDirection)));
    Logger.recordOutput(
        "Hopper/Config/ExtensionInverted",
        hopperExtensionInvertedEntry.getBoolean(HopperConstants.defaultHopperExtensionInverted));
    Logger.recordOutput(
        "Hopper/Config/ExtensionRetractedPositionRotations",
        hopperExtensionRetractedPositionRotations);
    Logger.recordOutput(
        "Hopper/Config/ExtensionExtendedPositionRotations",
        hopperExtensionExtendedPositionRotations);

    Logger.recordOutput("Hopper/State/BeltRunning", beltRunning);
    Logger.recordOutput("Hopper/State/LastAppliedBeltOutput", lastAppliedBeltSpeed);
    Logger.recordOutput("Hopper/State/LastAppliedExtensionOutput", lastAppliedExtensionSpeed);
    Logger.recordOutput("Hopper/State/ActualBeltOutput", hopperBelt.get());
    Logger.recordOutput("Hopper/State/ActualExtensionOutput", hopperExtension.get());
    Logger.recordOutput("Hopper/Measured/BeltCurrentAmps", hopperBelt.getOutputCurrent());
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
