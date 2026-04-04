package frc.robot.subsystems.gamepiece.indexers;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.gamepiece.ControlSensitivity;

public final class IndexersConstants {
  private IndexersConstants() {}

  public static final String configTableName = "GamePiece/Indexers";

  // Shared indexer motor that mechanically drives both top and bottom indexer shafts.
  public static final int topIndexerCanId = 60;
  public static final ControlSensitivity topIndexerSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;

  // Belt / agitator conveyor motor.
  public static final int bottomIndexerCanId = 61;
  public static final ControlSensitivity bottomIndexerSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;

  // Default open-loop speeds
  public static final double defaultTopIndexerSpeed = 0.55;
  public static final double defaultBottomIndexerSpeed = 0.55;
  public static final double defaultTopIndexerSpeedScale = 1.0;
  public static final double defaultBottomIndexerSpeedScale = 1.0;

  // Calibration-mode closed-loop defaults
  public static final double estimatedIndexerMaxVelocityRotationsPerSec = 12.0;
  public static final double defaultCalibrationTopVelocitySetpointRotationsPerSec = 0.0;
  public static final double defaultCalibrationBottomVelocitySetpointRotationsPerSec = 0.0;
  public static final double indexerVelocityKp = 0.03;
  public static final double indexerVelocityKi = 0.0;
  public static final double indexerVelocityKd = 0.0;
  public static final double indexerVelocityKv = 1.0 / estimatedIndexerMaxVelocityRotationsPerSec;

  // Default direction/inversion
  public static final double defaultTopIndexerDirection = 1.0;
  public static final double defaultBottomIndexerDirection = 1.0;
  public static final int indexerMotorCurrentLimitAmps = 90;

  // Estimated simulation configuration
  public static final double simNominalVoltage = 12.0;
  public static final DCMotor simIndexerGearbox = DCMotor.getBag(1);
  public static final double simIndexerReduction = 3.0;
  public static final double simIndexerMoiKgMetersSq = 0.0015;
  public static final double simIndexerRadiusMeters = Units.inchesToMeters(1.0);
}
