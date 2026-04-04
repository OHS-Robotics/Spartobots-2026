// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 3.65;
  public static final double moduleAngleHoldMinSpeedMetersPerSec = maxSpeedMetersPerSec * 0.01;
  public static final double maxAccelerationMeterPerSecSquared = 4.1;
  public static final double maxRotationalSpeedRadiansPerSec = Units.degreesToRadians(270);
  public static final double maxRotationalAccelerationRadiansPerSecSquared =
      Units.degreesToRadians(90);
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(20.5);
  public static final double wheelBase = Units.inchesToMeters(20.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // FL
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // FR
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // BL
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // BR
      };
  // Maps logical module index (FL, FR, BL, BR) -> hardware position index
  public static final int[] moduleIndexToHardwareIndex = {0, 1, 2, 3};

  // Global chassis-frame correction scalars applied in Drive.runVelocity.
  // Keep at 1.0 unless you intentionally need to correct a frame mismatch.
  public static final double chassisXCommandScalar = 1.0;
  public static final double chassisYCommandScalar = 1.0;
  public static final double chassisOmegaCommandScalar = 1.0;

  // NavX yaw sign to match WPILib's CCW-positive convention.
  public static final boolean navxYawInverted = false;

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-0.910);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(2.332);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(2.543 - .24);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(1.227);

  // Device CAN IDs
  public static final int frontLeftDriveCanId = 8;
  public static final int backLeftDriveCanId = 10;
  public static final int frontRightDriveCanId = 6;
  public static final int backRightDriveCanId = 12;

  public static final int frontLeftTurnCanId = 7;
  public static final int backLeftTurnCanId = 9;
  public static final int frontRightTurnCanId = 5;
  public static final int backRightTurnCanId = 11;

  public static final int frontLeftCANcoderId = 21;
  public static final int frontRightCANcoderId = 22;
  public static final int backLeftCANcoderId = 20;
  public static final int backRightCANcoderId = 23;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final int driveSimMotorCurrentLimit = 150;
  public static final double wheelRadiusMeters = 0.05;
  // public static final double wheelRadiusMeters = Units.inchesToMeters(1.975);
  public static final double driveMotorReduction = 6.75; // Swerve MK4 L2
  // and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.00115;
  public static final double driveKi = 0.0;
  public static final double driveKd = 0.004;
  public static final double driveKs = 0.0;
  // Approximate volts per wheel rad/s for a NEO on MK4 L2 gearing.
  // Replace with measured characterization data when available.
  public static final double driveKv = 0.1363;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.1363;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 12.8;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = false;
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 3.4;
  public static final double turnKi = 0.0;
  public static final double turnKd = 0.2;
  public static final double turnMaxIntegralOutputVolts = 2.0;
  public static final double turnSetpointResetThresholdRadians = Units.degreesToRadians(90.0);
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 49.89;
  public static final double robotMOI = 4.9;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
  public static final double pathfindingSpeedFactor = 0.25;
  public static final PathConstraints pathConstraints =
      new PathConstraints(
          maxSpeedMetersPerSec * pathfindingSpeedFactor,
          maxAccelerationMeterPerSecSquared * pathfindingSpeedFactor,
          maxRotationalSpeedRadiansPerSec * pathfindingSpeedFactor,
          maxRotationalAccelerationRadiansPerSecSquared * pathfindingSpeedFactor);
  public static final double trenchLongAxisAlignmentToleranceRadians = Units.degreesToRadians(2.0);

  // MapleSim configuration
  public static final double bumperLengthXMeters = 0.75;
  public static final double bumperWidthYMeters = 0.75;
  public static final double mapleDriveFrictionVolts = 0.1;
  public static final double mapleTurnFrictionVolts = 0.1;
  public static final double mapleSteerInertiaKgMetersSq = 0.004;

  // alignment config
  public static final double trenchSnapTo = Units.degreesToRadians(180);

  public static DriveTrainSimulationConfig getMapleSimConfig() {
    return MapleSimConfigHolder.mapleSimConfig;
  }

  private static final class MapleSimConfigHolder {
    private static final DriveTrainSimulationConfig mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(robotMassKg))
            .withBumperSize(Meters.of(bumperLengthXMeters), Meters.of(bumperWidthYMeters))
            .withCustomModuleTranslations(moduleTranslations)
            .withTrackLengthTrackWidth(Meters.of(wheelBase), Meters.of(trackWidth))
            .withGyro(COTS.ofNav2X())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    driveGearbox,
                    turnGearbox,
                    driveMotorReduction,
                    turnMotorReduction,
                    Volts.of(mapleDriveFrictionVolts),
                    Volts.of(mapleTurnFrictionVolts),
                    Meters.of(wheelRadiusMeters),
                    KilogramSquareMeters.of(mapleSteerInertiaKgMetersSq),
                    wheelCOF));

    private MapleSimConfigHolder() {}
  }
}
