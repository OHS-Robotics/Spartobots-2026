package frc.robot.subsystems.body;

import edu.wpi.first.wpilibj.DigitalInput;

public class GamePieceSensorIODio implements GamePieceSensorIO {
  private final DigitalInput intakeBeamBreak =
      createBeamBreak(GamePieceManagerConstants.intakeBeamBreakChannel);
  private final DigitalInput hopperBeamBreak =
      createBeamBreak(GamePieceManagerConstants.hopperBeamBreakChannel);
  private final DigitalInput shooterBeamBreak =
      createBeamBreak(GamePieceManagerConstants.shooterBeamBreakChannel);

  @Override
  public void updateInputs(GamePieceSensorIOInputs inputs) {
    inputs.intakeConfigured = intakeBeamBreak != null;
    inputs.hopperConfigured = hopperBeamBreak != null;
    inputs.shooterConfigured = shooterBeamBreak != null;
    inputs.intakeDetected = decodeBeamBreak(intakeBeamBreak);
    inputs.hopperDetected = decodeBeamBreak(hopperBeamBreak);
    inputs.shooterDetected = decodeBeamBreak(shooterBeamBreak);
  }

  private static DigitalInput createBeamBreak(int channel) {
    return channel >= 0 ? new DigitalInput(channel) : null;
  }

  private static boolean decodeBeamBreak(DigitalInput sensor) {
    if (sensor == null) {
      return false;
    }
    boolean sensorActive = sensor.get();
    return GamePieceManagerConstants.beamBreakActiveLow ? !sensorActive : sensorActive;
  }
}
