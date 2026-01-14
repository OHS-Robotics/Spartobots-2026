// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team4687.frc2026;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double deadband = 0.7;
  }

  public static final double MAX_SPEED = Units.feetToMeters(3.0);

  public static final SwerveModuleSimulationConfig 
  // ADD VALUES FOR THESE
        FRONT_LEFT_MODULE_CONFIG = new SwerveModuleSimulationConfig(DCMotor.getNEO(1), DCMotor.getNEO(1), 6.75, 12.8, Voltage.ofBaseUnits(12, Volt), Voltage.ofBaseUnits(12, Volt), Distance.ofBaseUnits(2, Inches), MomentOfInertia.ofBaseUnits(.05, KilogramSquareMeters), 1.19),
        FRONT_RIGHT_MODULE_CONFIG = new SwerveModuleSimulationConfig(DCMotor.getNEO(1), DCMotor.getNEO(1), 6.75, 12.8, Voltage.ofBaseUnits(12, Volt), Voltage.ofBaseUnits(12, Volt), Distance.ofBaseUnits(2, Inches), MomentOfInertia.ofBaseUnits(.05, KilogramSquareMeters), 1.19),
        BACK_LEFT_MODULE_CONFIG = new SwerveModuleSimulationConfig(DCMotor.getNEO(1), DCMotor.getNEO(1), 6.75, 12.8, Voltage.ofBaseUnits(12, Volt), Voltage.ofBaseUnits(12, Volt), Distance.ofBaseUnits(2, Inches), MomentOfInertia.ofBaseUnits(.05, KilogramSquareMeters), 1.19),
        BACK_RIGHT_MODULE_CONFIG = new SwerveModuleSimulationConfig(DCMotor.getNEO(1), DCMotor.getNEO(1), 6.75, 12.8, Voltage.ofBaseUnits(12, Volt), Voltage.ofBaseUnits(12, Volt), Distance.ofBaseUnits(2, Inches), MomentOfInertia.ofBaseUnits(.05, KilogramSquareMeters), 1.19);

// Create and configure a drivetrain simulation configuration
  public static final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(COTS.ofPigeon2())
        // Specify swerve module (for realistic swerve dynamics)
        .withSwerveModules(
                FRONT_LEFT_MODULE_CONFIG, 
                FRONT_RIGHT_MODULE_CONFIG, 
                BACK_LEFT_MODULE_CONFIG, 
                BACK_RIGHT_MODULE_CONFIG)
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(30), Inches.of(30));
}
