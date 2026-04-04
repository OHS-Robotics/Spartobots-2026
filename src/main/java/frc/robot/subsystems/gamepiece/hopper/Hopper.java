package frc.robot.subsystems.gamepiece.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NetworkTablesUtil;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private double lastAppliedExtensionSpeed = 0.0;
  private double hopperExtensionRetractedPositionRotations =
      HopperConstants.defaultHopperExtensionRetractedPositionRotations;
  private double hopperExtensionExtendedPositionRotations =
      HopperConstants.defaultHopperExtensionExtendedPositionRotations;

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private final NetworkTable subsystemTable =
      NetworkTablesUtil.domain(HopperConstants.configTableName);
  private final NetworkTable tuningTable = NetworkTablesUtil.tuningCommon(subsystemTable);
  private final NetworkTable telemetryTable = NetworkTablesUtil.telemetry(subsystemTable);

  private final NetworkTableEntry hopperExtensionSpeedScaleEntry =
      tuningTable.getEntry("Extension/SpeedScale");
  private final NetworkTableEntry hopperExtensionInvertedEntry =
      tuningTable.getEntry("Extension/Inverted");
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
  private final NetworkTableEntry hopperExtensionAppliedOutputEntry =
      telemetryTable.getEntry("Extension/AppliedOutput");

  public Hopper() {
    this(new HopperIO() {});
  }

  public Hopper(HopperIO io) {
    this.io = io;
    configureNetworkTableDefaults();
    loadNetworkTableConfig();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(NetworkTablesUtil.logPath("GamePiece/Hopper/Inputs"), inputs);
    loadNetworkTableConfig();
    publishExtensionEncoderToNetworkTables();
    publishActuatorStateToNetworkTables();
    logTelemetry();
  }

  public void setHopperExtensionSpeed(double speed) {
    lastAppliedExtensionSpeed =
        applyScaleAndInversion(speed, hopperExtensionSpeedScaleEntry, hopperExtensionInvertedEntry);
    io.setExtensionOutput(lastAppliedExtensionSpeed);
  }

  public void stopHopperExtension() {
    lastAppliedExtensionSpeed = 0.0;
    io.setExtensionOutput(0.0);
  }

  private void configureNetworkTableDefaults() {
    hopperExtensionSpeedScaleEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionSpeedScale);
    hopperExtensionRetractedPositionEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionRetractedPositionRotations);
    hopperExtensionExtendedPositionEntry.setDefaultDouble(
        HopperConstants.defaultHopperExtensionExtendedPositionRotations);
    hopperExtensionInvertedEntry.setDefaultBoolean(HopperConstants.defaultHopperExtensionInverted);
  }

  private void loadNetworkTableConfig() {
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
  }

  private double applyScaleAndInversion(
      double speed, NetworkTableEntry speedScaleEntry, NetworkTableEntry invertedEntry) {
    double scaledSpeed = clampSpeed(speed) * clampSpeedScale(speedScaleEntry.getDouble(1.0));
    return applyInversion(scaledSpeed, invertedEntry);
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
    hopperExtensionEncoderVelocityEntry.setDouble(inputs.extensionVelocityRpm);
    hopperExtensionEncoderNormalizedPositionEntry.setDouble(
        getHopperExtensionMeasuredPositionNormalized());
  }

  private void publishActuatorStateToNetworkTables() {
    hopperExtensionAppliedOutputEntry.setDouble(inputs.extensionAppliedOutput);
  }

  public double getHopperExtensionMeasuredPositionRotations() {
    return inputs.extensionPositionRotations;
  }

  public double getHopperExtensionCurrentAmps() {
    return inputs.extensionCurrentAmps;
  }

  public double getHopperExtensionMeasuredPositionNormalized() {
    return clampUnitInterval(
        inverseInterpolate(
            getHopperExtensionMeasuredPositionRotations(),
            hopperExtensionRetractedPositionRotations,
            hopperExtensionExtendedPositionRotations));
  }

  public void resetSimulationState() {
    lastAppliedExtensionSpeed = 0.0;
    io.resetSimulationState();
  }

  private void logTelemetry() {
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Config/ExtensionSpeedScale"),
        clampSpeedScale(
            hopperExtensionSpeedScaleEntry.getDouble(
                HopperConstants.defaultHopperExtensionSpeedScale)));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Config/ExtensionInverted"),
        hopperExtensionInvertedEntry.getBoolean(HopperConstants.defaultHopperExtensionInverted));
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Config/ExtensionRetractedPositionRotations"),
        hopperExtensionRetractedPositionRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Config/ExtensionExtendedPositionRotations"),
        hopperExtensionExtendedPositionRotations);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/State/LastAppliedExtensionOutput"),
        lastAppliedExtensionSpeed);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/State/ActualExtensionOutput"),
        inputs.extensionAppliedOutput);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Measured/ExtensionCurrentAmps"),
        inputs.extensionCurrentAmps);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Measured/ExtensionEncoderPositionRotations"),
        getHopperExtensionMeasuredPositionRotations());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Measured/ExtensionEncoderVelocityRpm"),
        inputs.extensionVelocityRpm);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("GamePiece/Hopper/Measured/ExtensionEncoderPositionNormalized"),
        getHopperExtensionMeasuredPositionNormalized());
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
}
