package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Central edit point for robot-specific hardware IDs, tuning values, and tunable geometry.
 *
 * <p>Keep fixed game/field constants out of this file so edits here stay focused on the robot.
 */
public final class RobotSettings {
  private RobotSettings() {}

  public enum MotorControllerType {
    SPARK_MAX,
    SPARK_FLEX,
    TALON_FX,
    TALON_SRX,
    VICTOR_SPX,
    NONE
  }

  public enum MotorType {
    NEO,
    NEO_550,
    VORTEX,
    FALCON_500,
    KRAKEN_X60,
    BAG,
    CIM,
    MINI_CIM,
    OTHER,
    NONE
  }

  public static final class Controls {
    public static final double intentTriggerThreshold = 0.5;
    public static final double joystickDeadband = 0.1;
    public static final double headingKp = 5.5;
    public static final double headingKd = 0.1;
    public static final double headingMaxVelocity = 45.0;
    public static final double headingMaxAcceleration = 90.0;
    public static final double poseHoldTranslationKp = 2.5;
    public static final double feedforwardStartDelaySeconds = 2.0;
    public static final double feedforwardRampRateVoltsPerSec = 0.1;
    public static final double wheelRadiusMaxVelocityRadPerSec = 0.25;
    public static final double wheelRadiusRampRateRadPerSecSquared = 0.05;
  }

  public static final class Drive {
    public enum GyroType {
      NAVX,
      PIGEON2
    }

    public static final GyroType realGyroType = GyroType.NAVX;

    public static final double maxSpeedMetersPerSec = 4.60248;
    public static final double maxAccelerationMeterPerSecSquared = 3.0;
    public static final double maxRotationalSpeedRadiansPerSec = Units.degreesToRadians(30.0);
    public static final double maxRotationalAccelerationRadiansPerSecSquared =
        Units.degreesToRadians(15.0);
    public static final double odometryFrequency = 100.0;
    public static final double trackWidthMeters = Units.inchesToMeters(20.5);
    public static final double wheelBaseMeters = Units.inchesToMeters(20.5);
    public static final int[] moduleIndexToHardwareIndex = {0, 1, 2, 3};

    public static final double chassisXCommandScalar = 1.0;
    public static final double chassisYCommandScalar = 1.0;
    public static final double chassisOmegaCommandScalar = 1.0;
    public static final boolean navxYawInverted = true;

    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-0.910);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(2.332);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(2.543);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(1.227);

    public static final int pigeonCanId = 1;
    public static final int frontLeftDriveCanId = 8;
    public static final int backLeftDriveCanId = 10;
    public static final int frontRightDriveCanId = 6;
    public static final int backRightDriveCanId = 12;
    public static final int frontLeftTurnCanId = 7;
    public static final int backLeftTurnCanId = 9;
    public static final int frontRightTurnCanId = 5;
    public static final int backRightTurnCanId = 11;
    public static final int frontLeftCanCoderId = 21;
    public static final int frontRightCanCoderId = 22;
    public static final int backLeftCanCoderId = 20;
    public static final int backRightCanCoderId = 23;

    public static final int driveMotorCurrentLimit = 50;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.975);
    public static final double driveMotorReduction = 6.75;
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);

    public static final double driveKp = 0.0010645;
    public static final double driveKi = 0.0;
    public static final double driveKd = 0.01;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.0789;
    public static final double driveSimP = 0.0010645;
    public static final double driveSimD = 0.01;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 12.8;
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);
    public static final boolean turnEncoderInverted = false;
    public static final double turnKp = 2.5;
    public static final double turnKi = 0.5;
    public static final double turnKd = 0.1;
    public static final double turnSimP = 1.0;
    public static final double turnSimD = 0.0;

    public static final double robotMassKg = 74.088;
    public static final double robotMoiKgMetersSq = 6.883;
    public static final double wheelCoefficientOfFriction = 1.2;

    public static final double bumperLengthMeters = Units.inchesToMeters(30.0);
    public static final double bumperWidthMeters = Units.inchesToMeters(30.0);
    public static final double mapleDriveFrictionVolts = 0.1;
    public static final double mapleTurnFrictionVolts = 0.2;
    public static final double mapleSteerInertiaKgMetersSq = 0.02;

    public static final Rotation2d aimTolerance = Rotation2d.fromDegrees(3.0);
    public static final double alignedToleranceRadians = Units.degreesToRadians(5.0);
    public static final double alignKp = 0.0;
    public static final double alignKi = 0.0;
    public static final double alignKd = 0.0;
  }

  public static final class Vision {
    public static final String camera0Name = "Arducam_OV9281_USB_Camera";
    public static final String camera1Name = "Arducam_OV9281_USB_Camera (1)";

    public static final Transform3d robotToCamera0 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0.0), Units.inchesToMeters(-5.0), Units.inchesToMeters(10.5)),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90.0)));
    public static final Transform3d robotToCamera1 =
        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;
    public static final double linearStdDevBaseline = 0.02;
    public static final double angularStdDevBaseline = 0.06;
    public static final double[] cameraStdDevFactors = {1.0, 1.0};
    public static final double linearStdDevMegatag2Factor = 0.5;
    public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
  }

  public static final class Shooter {
    public static final double gravityMetersPerSecSquared = 9.80665;
    public static final double defaultLaunchSpeedMetersPerSec = 11.5;
    public static final Rotation2d defaultLaunchAngle = Rotation2d.fromDegrees(56.0);
    public static final double defaultLaunchHeightMeters = Units.inchesToMeters(30.0);
    public static final double minLaunchSpeedMetersPerSec = 1.0;
    public static final double minAirtimeSeconds = 0.05;
    public static final double maxAirtimeSeconds = 2.5;
    public static final double fallbackAirtimeSeconds = 0.6;
    public static final double minHorizontalVelocityMetersPerSec = 0.25;

    public static final MotorControllerType leftFlywheelController = MotorControllerType.NONE;
    public static final MotorType leftFlywheelMotorType = MotorType.NONE;
    public static final int leftFlywheelCanId = -1;

    public static final MotorControllerType rightFlywheelController = MotorControllerType.NONE;
    public static final MotorType rightFlywheelMotorType = MotorType.NONE;
    public static final int rightFlywheelCanId = -1;

    public static final MotorControllerType hoodController = MotorControllerType.NONE;
    public static final MotorType hoodMotorType = MotorType.NONE;
    public static final int hoodCanId = -1;

    public static final MotorControllerType feederController = MotorControllerType.NONE;
    public static final MotorType feederMotorType = MotorType.NONE;
    public static final int feederCanId = -1;
  }

  public static final class Intake {
    public static final MotorControllerType deployController = MotorControllerType.NONE;
    public static final MotorType deployMotorType = MotorType.NONE;
    public static final int deployCanId = -1;

    public static final MotorControllerType rollerController = MotorControllerType.NONE;
    public static final MotorType rollerMotorType = MotorType.NONE;
    public static final int rollerCanId = -1;
  }

  public static final class Indexer {
    public static final MotorControllerType lowerController = MotorControllerType.NONE;
    public static final MotorType lowerMotorType = MotorType.NONE;
    public static final int lowerCanId = -1;

    public static final MotorControllerType upperController = MotorControllerType.NONE;
    public static final MotorType upperMotorType = MotorType.NONE;
    public static final int upperCanId = -1;
  }

  public static final class Endgame {
    public static final MotorControllerType leftController = MotorControllerType.NONE;
    public static final MotorType leftMotorType = MotorType.NONE;
    public static final int leftCanId = -1;

    public static final MotorControllerType rightController = MotorControllerType.NONE;
    public static final MotorType rightMotorType = MotorType.NONE;
    public static final int rightCanId = -1;
  }

  public static final class Auto {
    public static final double preloadScoreTimeoutSeconds = 2.5;
    public static final double acquireTimeoutSeconds = 4.0;
    public static final double scoreTimeoutSeconds = 2.5;
    public static final double ejectTimeoutSeconds = 1.0;
  }

  public static final class Simulation {
    public static final double intakeCaptureRadiusMeters = 0.45;
    public static final double outpostRefillDistanceMeters = 0.7;
    public static final double outpostRefillSeconds = 0.35;

    public static final Translation2d intakeOffsetMeters = new Translation2d(0.42, 0.0);
    public static final Translation2d shooterOffsetMeters = new Translation2d(0.08, 0.0);
    public static final Translation2d magazineOffsetMeters = new Translation2d(0.05, 0.0);
    public static final Translation2d endgameLeftOffsetMeters = new Translation2d(-0.18, 0.22);
    public static final Translation2d endgameRightOffsetMeters = new Translation2d(-0.18, -0.22);

    public static final double intakeHeightMeters = 0.18;
    public static final double shooterHeightMeters = 0.56;
    public static final double magazineHeightMeters = 0.35;
    public static final double endgameBaseHeightMeters = 0.78;
    public static final double ejectSpeedMetersPerSecond = 1.8;

    public static final double robotBodyBaseHeightMeters = 0.12;
    public static final double moduleHeightAboveGroundMeters = 0.05;
  }
}
