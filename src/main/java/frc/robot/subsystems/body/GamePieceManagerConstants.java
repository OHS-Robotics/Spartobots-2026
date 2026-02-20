package frc.robot.subsystems.body;

public final class GamePieceManagerConstants {
  private GamePieceManagerConstants() {}

  public static final String configTableName = "GamePieceManagerConfig";

  // Set to a valid DIO channel on the real robot to enable each beam break.
  public static final int intakeBeamBreakChannel = -1;
  public static final int hopperBeamBreakChannel = -1;
  public static final int shooterBeamBreakChannel = -1;
  public static final boolean beamBreakActiveLow = true;

  // Feed profile
  public static final double collectIntakeSpeed = 0.65;
  public static final double collectHopperSpeed = 0.55;
  public static final double collectAgitatorSpeed = 0.55;
  public static final double feedHopperSpeed = 0.75;
  public static final double feedAgitatorSpeed = 0.75;
  public static final double reverseSpeed = -0.45;

  // Jam handling
  public static final double jamCurrentThresholdAmps = 35.0;
  public static final double jamDetectionSeconds = 0.35;
  public static final double unjamReverseSeconds = 0.20;

  // Fallback when no sensors are configured
  public static final double sensorlessCollectToHoldSeconds = 0.35;
}
