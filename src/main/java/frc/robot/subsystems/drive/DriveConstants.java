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
import frc.robot.RobotSettings;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/** Derived drive constants. Tune source values in {@link frc.robot.RobotSettings}. */
public class DriveConstants {
  public static final double maxSpeedMetersPerSec = RobotSettings.Drive.maxSpeedMetersPerSec;
  public static final double maxAccelerationMeterPerSecSquared =
      RobotSettings.Drive.maxAccelerationMeterPerSecSquared;
  public static final double maxRotationalSpeedRadiansPerSec =
      RobotSettings.Drive.maxRotationalSpeedRadiansPerSec;
  public static final double maxRotationalAccelerationRadiansPerSecSquared =
      RobotSettings.Drive.maxRotationalAccelerationRadiansPerSecSquared;
  public static final double odometryFrequency = RobotSettings.Drive.odometryFrequency; // Hz
  public static final double trackWidth = RobotSettings.Drive.trackWidthMeters;
  public static final double wheelBase = RobotSettings.Drive.wheelBaseMeters;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // FL
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // FR
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // BL
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // BR
      };
  // Maps logical module index (FL, FR, BL, BR) -> hardware position index
  public static final int[] moduleIndexToHardwareIndex =
      RobotSettings.Drive.moduleIndexToHardwareIndex;

  // Global chassis-frame correction scalars applied in Drive.runVelocity.
  // Keep at 1.0 unless you intentionally need to correct a frame mismatch.
  public static final double chassisXCommandScalar = RobotSettings.Drive.chassisXCommandScalar;
  public static final double chassisYCommandScalar = RobotSettings.Drive.chassisYCommandScalar;
  public static final double chassisOmegaCommandScalar =
      RobotSettings.Drive.chassisOmegaCommandScalar;

  // NavX yaw sign to match WPILib's CCW-positive convention.
  public static final boolean navxYawInverted = RobotSettings.Drive.navxYawInverted;

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = RobotSettings.Drive.frontLeftZeroRotation;
  public static final Rotation2d frontRightZeroRotation =
      RobotSettings.Drive.frontRightZeroRotation;
  public static final Rotation2d backLeftZeroRotation = RobotSettings.Drive.backLeftZeroRotation;
  public static final Rotation2d backRightZeroRotation = RobotSettings.Drive.backRightZeroRotation;

  // Device CAN IDs
  public static final int pigeonCanId = RobotSettings.Drive.pigeonCanId;

  public static final int frontLeftDriveCanId = RobotSettings.Drive.frontLeftDriveCanId;
  public static final int backLeftDriveCanId = RobotSettings.Drive.backLeftDriveCanId;
  public static final int frontRightDriveCanId = RobotSettings.Drive.frontRightDriveCanId;
  public static final int backRightDriveCanId = RobotSettings.Drive.backRightDriveCanId;

  public static final int frontLeftTurnCanId = RobotSettings.Drive.frontLeftTurnCanId;
  public static final int backLeftTurnCanId = RobotSettings.Drive.backLeftTurnCanId;
  public static final int frontRightTurnCanId = RobotSettings.Drive.frontRightTurnCanId;
  public static final int backRightTurnCanId = RobotSettings.Drive.backRightTurnCanId;

  public static final int frontLeftCANcoderId = RobotSettings.Drive.frontLeftCanCoderId;
  public static final int frontRightCANcoderId = RobotSettings.Drive.frontRightCanCoderId;
  public static final int backLeftCANcoderId = RobotSettings.Drive.backLeftCanCoderId;
  public static final int backRightCANcoderId = RobotSettings.Drive.backRightCanCoderId;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = RobotSettings.Drive.driveMotorCurrentLimit;
  public static final double wheelRadiusMeters = RobotSettings.Drive.wheelRadiusMeters;
  public static final double driveMotorReduction = RobotSettings.Drive.driveMotorReduction;
  // and 22 spur teeth
  public static final DCMotor driveGearbox = RobotSettings.Drive.driveGearbox;

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = RobotSettings.Drive.driveKp;
  public static final double driveKi = RobotSettings.Drive.driveKi;
  public static final double driveKd = RobotSettings.Drive.driveKd;
  public static final double driveKs = RobotSettings.Drive.driveKs;
  public static final double driveKv = RobotSettings.Drive.driveKv;
  public static final double driveSimP = RobotSettings.Drive.driveSimP;
  public static final double driveSimD = RobotSettings.Drive.driveSimD;
  public static final double driveSimKs = RobotSettings.Drive.driveSimKs;
  public static final double driveSimKv = RobotSettings.Drive.driveSimKv;

  // Turn motor configuration
  public static final boolean turnInverted = RobotSettings.Drive.turnInverted;
  public static final int turnMotorCurrentLimit = RobotSettings.Drive.turnMotorCurrentLimit;
  public static final double turnMotorReduction = RobotSettings.Drive.turnMotorReduction;
  public static final DCMotor turnGearbox = RobotSettings.Drive.turnGearbox;

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = RobotSettings.Drive.turnEncoderInverted;
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Turn PID configuration
  public static final double turnKp = RobotSettings.Drive.turnKp;
  public static final double turnKi = RobotSettings.Drive.turnKi;
  public static final double turnKd = RobotSettings.Drive.turnKd;
  public static final double turnSimP = RobotSettings.Drive.turnSimP;
  public static final double turnSimD = RobotSettings.Drive.turnSimD;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = RobotSettings.Drive.robotMassKg;
  public static final double robotMOI = RobotSettings.Drive.robotMoiKgMetersSq;
  public static final double wheelCOF = RobotSettings.Drive.wheelCoefficientOfFriction;
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
  public static final PathConstraints pathConstraints =
      new PathConstraints(
          maxSpeedMetersPerSec,
          maxAccelerationMeterPerSecSquared,
          maxRotationalSpeedRadiansPerSec,
          maxRotationalAccelerationRadiansPerSecSquared);

  // MapleSim configuration
  public static final double bumperLengthXMeters = RobotSettings.Drive.bumperLengthMeters;
  public static final double bumperWidthYMeters = RobotSettings.Drive.bumperWidthMeters;
  public static final double mapleDriveFrictionVolts = RobotSettings.Drive.mapleDriveFrictionVolts;
  public static final double mapleTurnFrictionVolts = RobotSettings.Drive.mapleTurnFrictionVolts;
  public static final double mapleSteerInertiaKgMetersSq =
      RobotSettings.Drive.mapleSteerInertiaKgMetersSq;

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(robotMassKg))
          .withBumperSize(Meters.of(bumperLengthXMeters), Meters.of(bumperWidthYMeters))
          .withCustomModuleTranslations(moduleTranslations)
          .withGyro(COTS.ofPigeon2())
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

  // alignment config
  public static final double aligned = RobotSettings.Drive.alignedToleranceRadians;

  public static final double alignKp = RobotSettings.Drive.alignKp;
  public static final double alignKi = RobotSettings.Drive.alignKi;
  public static final double alignKd = RobotSettings.Drive.alignKd;
}
