// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  final CommandJoystick driverJoystick = new CommandJoystick(0);
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandGenericHID driverGenericHID = new CommandGenericHID(0);

  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public double applyExpoCurveTranslation(double input) {
		double sign = input / Math.abs(input);
		return Math.pow(input, Constants.Operator.expoCurveExponentTranslation) * sign;
	}
  
  public double applyExpoCurveRotation(double input) {
		double sign = input / Math.abs(input);
		return Math.pow(input, Constants.Operator.expoCurveExponentRotation) * sign;
	}

  private boolean referenceFrameIsField = true;
	public double elevatorPosition = 0.0;
	public boolean isInHighGear = true;

  SwerveInputStream driveFieldAngularVelocityStream;
	SwerveInputStream driveRobotAngularVelocityStream;
	SwerveInputStream driveFieldAngularVelocityKeyboardStream;
	SwerveInputStream driveRobotAngularVelocityKeyboardStream;

  Command driveFieldAnglularVelocity;
	Command driveRobotAngularVelocity;
	Command driveFieldAnglularVelocityKeyboard;
	Command driveRobotAngularVelocityKeyboard;

  
  private void configureDriveInputStreams() {
		if (Constants.Operator.useJoystick) {
			// Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
			driveFieldAngularVelocityStream = SwerveInputStream.of(
					drivebase.getSwerveDrive(), 
					() -> applyExpoCurveTranslation(driverJoystick.getY()), 
					() -> applyExpoCurveTranslation(driverJoystick.getX()))
				.withControllerRotationAxis(() -> applyExpoCurveRotation(driverJoystick.getTwist()))
				.deadband(Constants.Operator.deadband)
				.scaleTranslation(Constants.Operator.scaleTranslationHighGear)
				.scaleRotation(Constants.Operator.scaleRotationHighGear)
				.allianceRelativeControl(true);

			// Clone's the angular velocity input stream and converts it to a robotRelative input stream.
			driveRobotAngularVelocityStream = driveFieldAngularVelocityStream.copy()
				.robotRelative(true)
				.allianceRelativeControl(false);

			driveFieldAngularVelocityKeyboardStream = SwerveInputStream.of(
					drivebase.getSwerveDrive(), 
					() -> -driverJoystick.getY(), 
					() -> -driverJoystick.getX())
				.withControllerRotationAxis(() -> driverJoystick.getRawAxis(2))
				.deadband(Constants.Operator.deadband)
				.scaleTranslation(Constants.Operator.scaleTranslationHighGear)
				.scaleRotation(Constants.Operator.scaleRotationHighGear)
				.allianceRelativeControl(true);

			driveRobotAngularVelocityKeyboardStream = driveFieldAngularVelocityStream.copy()
				.withControllerRotationAxis(() -> driverJoystick.getRawAxis(2))
				.robotRelative(true)
				.allianceRelativeControl(false);
		}
		else {
			// Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
			driveFieldAngularVelocityStream = SwerveInputStream.of(
					drivebase.getSwerveDrive(), 
					() -> applyExpoCurveTranslation(driverXbox.getLeftY()), 
					() -> applyExpoCurveTranslation(driverXbox.getLeftX()))
				.withControllerRotationAxis(() -> applyExpoCurveRotation(driverXbox.getRightX()))
				.deadband(Constants.Operator.deadband)
				.scaleTranslation(Constants.Operator.scaleTranslationHighGear)
				.scaleRotation(Constants.Operator.scaleRotationHighGear)
				.allianceRelativeControl(true);

			// Clone's the angular velocity input stream and converts it to a robotRelative input stream.
			driveRobotAngularVelocityStream = driveFieldAngularVelocityStream.copy()
				.robotRelative(true)
				.allianceRelativeControl(false);

			driveFieldAngularVelocityKeyboardStream = SwerveInputStream.of(drivebase.getSwerveDrive(), 
					() -> -driverXbox.getLeftY(), 
					() -> -driverXbox.getLeftX())
				.withControllerRotationAxis(() -> driverXbox.getRightX())
				.deadband(Constants.Operator.deadband)
				.scaleTranslation(Constants.Operator.scaleTranslationHighGear)
				.scaleRotation(Constants.Operator.scaleRotationHighGear)
				.allianceRelativeControl(true);

			driveRobotAngularVelocityKeyboardStream = driveFieldAngularVelocityStream.copy()
				.withControllerRotationAxis(() -> driverXbox.getRightX())
				.robotRelative(true)
				.allianceRelativeControl(false);
		}

    SmartDashboard.putString("Gear Mode", isInHighGear ? "High" : "Low");
  } 
}