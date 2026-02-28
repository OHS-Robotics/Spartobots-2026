package frc.robot.subsystems.body;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class IndexersConstants {
  private IndexersConstants() {}

  public static final String configTableName = "Body/Indexers";

  // Upper/top indexer (velocity-sensitive)
  public static final int topIndexerCanId = 36;
  public static final ControlSensitivity topIndexerSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;

  // Lower/bottom indexer (velocity-sensitive)
  public static final int bottomIndexerCanId = 37;
  public static final ControlSensitivity bottomIndexerSensitivity =
      ControlSensitivity.VELOCITY_SENSITIVE;

  // Default open-loop speeds
  public static final double defaultTopIndexerSpeed = 0.55;
  public static final double defaultBottomIndexerSpeed = 0.55;
  public static final double defaultTopIndexerSpeedScale = 1.0;
  public static final double defaultBottomIndexerSpeedScale = 1.25;

  // Default direction/inversion
  public static final double defaultTopIndexerDirection = 1.0;
  public static final double defaultBottomIndexerDirection = -1.0;
  public static final int indexerMotorCurrentLimitAmps = 80;

  // Estimated simulation configuration
  public static final double simNominalVoltage = 12.0;
  public static final DCMotor simIndexerGearbox = DCMotor.getBag(1);
  public static final double simIndexerReduction = 3.0;
  public static final double simIndexerMoiKgMetersSq = 0.0015;
  public static final double simIndexerRadiusMeters = Units.inchesToMeters(1.0);
}
