// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team4687.frc2026;

import com.team4687.frc2026.Constants.*;
import com.team4687.frc2026.simulation.SwerveDriveSim;
import com.team4687.frc2026.subsystems.drive.SwerveDriveIO;
import com.team4687.frc2026.subsystems.drive.SwerveSubsystem;

import swervelib.SwerveInputStream;

import java.io.File;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //public final SwerveSubsystem swerveDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public final SwerveDriveIO drive;

  //SwerveInputStream driveFieldAngularVelocityStream;
  Supplier<ChassisSpeeds> driveRobotAngularVelocityStream;

  int DebugMode = 0;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverJoystick =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    // determine whether to use real or simulated robot
    if(Robot.isReal()) {
        this.drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve")); // Real implementation
    }
    else {
        this.drive = new SwerveDriveSim(); // Simulation implementation
    }
    
    // Configure the trigger bindings
    configureInputStreams();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drive.setDefaultCommand(drive.driveCommand(driveRobotAngularVelocityStream));
  }

  private void configureInputStreams() {
    /*driveFieldAngularVelocityStream = SwerveInputStream.of(
      this.drive.swerveDrive,
      () -> driverJoystick.getLeftY(),
      () -> (DebugMode == 0 ? driverJoystick.getLeftX() : 0.0)
    ).withControllerRotationAxis(() -> DebugMode == 0 ? driverJoystick.getRightX() : 0.0)
     .deadband(Constants.OperatorConstants.deadband)
     .allianceRelativeControl(true);*/
     
     driveRobotAngularVelocityStream = () -> new ChassisSpeeds(deadband(driverJoystick.getLeftY(), "left") * allianceRelative() * Constants.OperatorConstants.driveAdjust,
                                                              deadband(driverJoystick.getLeftX(), "left") * allianceRelative() * Constants.OperatorConstants.driveAdjust,
                                                              deadband(driverJoystick.getRightX(), "right") * -1 * Constants.OperatorConstants.steerAdjust);

    /*driveRobotAngularVelocityStream = driveFieldAngularVelocityStream.copy()
    .robotRelative(true)
    .allianceRelativeControl(false);*/
  }

  // adjust drive controls based on alliance
  private int allianceRelative() {
    if(DriverStation.getAlliance().isPresent()) {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return 1;
      } else {
        return -1;
      }
    }
    return 1;
  }

  // adjust joystick input based on deadband value
  private double deadband(double stickValue, String stick) {
    int negativeAdjustment;
    if(stickValue < 0) {
      negativeAdjustment = -1;
    } else {
      negativeAdjustment = 1;
    }
    if(stick == "left") {
      if((Math.pow(driverJoystick.getLeftY(), 2) + Math.pow(driverJoystick.getLeftX(), 2)) < Math.pow(Constants.OperatorConstants.deadband, 2)) {
        return 0;
      } else {
        if(Math.abs(stickValue) < Constants.OperatorConstants.axisDeadband) {
          return 0;
        } else {
          return (Math.abs(stickValue) - Constants.OperatorConstants.deadband) / (1 - Constants.OperatorConstants.deadband) * negativeAdjustment;
        }
      }
    } else {
      if((Math.pow(driverJoystick.getRightY(), 2) + Math.pow(driverJoystick.getRightX(), 2)) < Math.pow(Constants.OperatorConstants.deadband, 2)) {
        return 0;
      } else { 
        if(Math.abs(stickValue) < Constants.OperatorConstants.axisDeadband) {
          return 0;
        } else {
          return (Math.abs(stickValue) - Constants.OperatorConstants.deadband) / (1 - Constants.OperatorConstants.deadband) * negativeAdjustment;
        }
      }
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return swerveDrive.changePosition(new Translation2d(0.0, 2.0), Units.feetToMeters(3.0));
    return drive.getAutonomousCommand();
  }
}
