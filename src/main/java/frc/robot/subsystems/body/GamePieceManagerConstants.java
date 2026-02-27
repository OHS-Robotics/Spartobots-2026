package frc.robot.subsystems.body;

public final class GamePieceManagerConstants {
  private GamePieceManagerConstants() {}

  public static final String configTableName = "Body/GamePieceManager";

  // Set to a valid DIO channel on the real robot to enable each beam break.
  public static final int intakeBeamBreakChannel = -1;
  public static final int hopperBeamBreakChannel = -1;
  public static final int shooterBeamBreakChannel = -1;
  public static final boolean beamBreakActiveLow = true;

  // Feed profile
  public static final double collectIntakeSpeed = 0.65;
  public static final double collectAgitatorSpeed = 0.55;
  public static final double collectIndexerSpeed = 0.55;
  public static final double feedAgitatorSpeed = 0.75;
  public static final double feedIndexerSpeed = 0.75;
  public static final double reverseSpeed = -0.45;

  // Jam handling
  public static final double jamCurrentThresholdAmps = 35.0;
  public static final double jamDetectionSeconds = 0.35;
  public static final double unjamReverseSeconds = 0.20;

  // Fallback when no sensors are configured. Set <= 0 to disable.
  public static final double sensorlessCollectToHoldSeconds = 0.0;

  // Simulation staging
  public static final double simIntakeToHopperSeconds = 0.25;
  public static final double simHopperToShooterSeconds = 0.18;
  public static final double simReverseStepSeconds = 0.2;
  public static final double simIntakeCaptureMinPivotNormalized = 0.0;
  public static final int simIntakeCapacity = 1;
  public static final int simStageCapacity = 1;
  public static final double simEjectSpeedMetersPerSec = 1.2;
}
