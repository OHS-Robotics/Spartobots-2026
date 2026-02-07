// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team4687.frc2026;

import com.team4687.frc2026.Constants.*;

import com.team4687.frc2026.subsystems.SwerveSubsystem;
import com.team4687.frc2026.subsystems.body.IntakeSubsystem;
import com.team4687.frc2026.subsystems.body.LauncherSubsystem;

import swervelib.SwerveInputStream;

import java.io.File;

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
  public SwerveSubsystem swerveDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public IntakeSubsystem intake      = new IntakeSubsystem();
  public LauncherSubsystem launcher  = new LauncherSubsystem();

  SwerveInputStream driveFieldAngularVelocityStream;
  SwerveInputStream driveRobotAngularVelocityStream;

  int DebugMode = 0;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverJoystick =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureInputStreams();
    configureBindings();
  }

  public void robotPeriodic() {
    swerveDrive.update();
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
    swerveDrive.setDefaultCommand(swerveDrive.driveCommand(driveRobotAngularVelocityStream, driveFieldAngularVelocityStream));

    //driverJoystick.y().onTrue(swerveDrive.getAutonomousCommand());

    // intake
    driverJoystick.rightBumper().whileTrue(intake.increaseSpeed());
    driverJoystick.leftBumper() .whileTrue(intake.decreaseSpeed());
    /*driverJoystick.x().onTrue(intake.runCommand());
    driverJoystick.x().onFalse(intake.stopCommand());*/
    driverJoystick.x().onTrue(intake.toggleCommand());

    // launcher
    // enables both indexer and launcher itself
    driverJoystick.y().onTrue(launcher.toggleLauncherCommand());
    driverJoystick.b().onTrue(launcher.toggleIntakeCommand());

    // launch speed controls
    driverJoystick.povDown().onTrue(launcher.decreaseLaunchSpeed());
    driverJoystick.povUp().onTrue(launcher.increaseLaunchSpeed());
    driverJoystick.povLeft().onTrue(launcher.decreaseIntakeSpeed());
    driverJoystick.povRight().onTrue(launcher.increaseIntakeSpeed());
  }

  private void configureInputStreams() {
    driveFieldAngularVelocityStream = SwerveInputStream.of(
      swerveDrive.swerveDrive,
      () -> driverJoystick.getLeftY(),
      () -> (DebugMode == 0 ? driverJoystick.getLeftX() : 0.0)
    ).withControllerRotationAxis(() -> DebugMode == 0 ? driverJoystick.getRightX() : 0.0)
     .deadband(Constants.OperatorConstants.deadband)
     .allianceRelativeControl(true);

    driveRobotAngularVelocityStream = driveFieldAngularVelocityStream.copy()
    .robotRelative(true)
    .allianceRelativeControl(false);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return swerveDrive.changePosition(new Translation2d(0.0, 2.0), Units.feetToMeters(3.0));
    return swerveDrive.getAutonomousCommand();
  }
}
