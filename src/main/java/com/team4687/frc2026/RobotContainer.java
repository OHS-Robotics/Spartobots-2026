// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team4687.frc2026;

import com.team4687.frc2026.Constants.*;
import com.team4687.frc2026.subsystems.AutoSubsystem;
import com.team4687.frc2026.subsystems.SwerveSubsystem;
import com.team4687.frc2026.subsystems.body.IntakeSubsystem;
import com.team4687.frc2026.subsystems.body.LauncherSubsystem;

import swervelib.SwerveInputStream;

import java.io.File;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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


  public AutoSubsystem auto = new AutoSubsystem(swerveDrive, launcher, intake);

  SwerveInputStream driveFieldAngularVelocityStream;
  SwerveInputStream driveRobotAngularVelocityStream;

  int DebugMode = 0;

  // the auto align command need
  public boolean delayedEventsRun = false;

  private boolean manipulatorRumbling = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverJoystick =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController manipulatorJoystick =
      new CommandXboxController(OperatorConstants.kManipulatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureInputStreams();
    configureBindings();
  }

  public void robotPeriodic() {
    swerveDrive.update();
  }

  public void teleopInit() {
    launcher.launcherEncoder.setPosition(0.0);
    // alignment controls
    if (!delayedEventsRun) {
      System.out.printf("Rotate event added: %s\n", DriverStation.getAlliance().get() == Alliance.Blue ? "blue" : "red");
      driverJoystick.rightStick().whileTrue(
        DriverStation.getAlliance().get() == Alliance.Blue ?
        /*swerveDrive.pointTowardsFixed(Constants.blueHub, Constants.MAX_ROTATIONAL_SPEED) :
        swerveDrive.pointTowardsFixed(Constants.redHub, Constants.MAX_ROTATIONAL_SPEED)*/
        swerveDrive.pointTowardsAndDrive(Constants.blueHub, Constants.MAX_ROTATIONAL_SPEED, driveRobotAngularVelocityStream, driveFieldAngularVelocityStream) :
        swerveDrive.pointTowardsAndDrive(Constants.redHub, Constants.MAX_ROTATIONAL_SPEED, driveRobotAngularVelocityStream, driveFieldAngularVelocityStream)
      );
      driverJoystick.rightStick().whileTrue(launcher.autoAlignAngle(swerveDrive::getPose,
        () -> DriverStation.getAlliance().get() == Alliance.Blue ? Constants.blueHub : Constants.redHub)
      );

      // left/right tower align
      driverJoystick.povLeft().onTrue(DriverStation.getAlliance().get() == Alliance.Blue ?
        swerveDrive.driveTo(new Pose2d(1.0, 5.0, new Rotation2d(-Math.PI/2)), Constants.MAX_SPEED, Constants.MAX_ROTATIONAL_SPEED) :
        swerveDrive.driveTo(new Pose2d(15.5, 3.25, new Rotation2d(Math.PI/2)), Constants.MAX_SPEED, Constants.MAX_ROTATIONAL_SPEED)
      );
      driverJoystick.povRight().onTrue(DriverStation.getAlliance().get() == Alliance.Blue ?
        swerveDrive.driveTo(new Pose2d(1.0, 2.5, new Rotation2d(Math.PI/2)), Constants.MAX_SPEED, Constants.MAX_ROTATIONAL_SPEED) :
        swerveDrive.driveTo(new Pose2d(15.5, 5.25, new Rotation2d(Math.PI/-2)), Constants.MAX_SPEED, Constants.MAX_ROTATIONAL_SPEED)
      );

      // blue feeder align
      driverJoystick.leftTrigger().onTrue(
        swerveDrive.driveTo(new Pose2d(0.6, 0.65, new Rotation2d(Math.PI)), Constants.MAX_SPEED, Constants.MAX_ROTATIONAL_SPEED)
      );

      // red feeder align
      driverJoystick.rightTrigger().onTrue(
        swerveDrive.driveTo(new Pose2d(15.9, 7.4, new Rotation2d(0.0)), Constants.MAX_SPEED, Constants.MAX_ROTATIONAL_SPEED)
      );

      delayedEventsRun = true;

        /*swerveDrive.pointTowardsAndDrive(Constants.blueHub, Constants.MAX_ROTATIONAL_SPEED, driveRobotAngularVelocityStream, driveFieldAngularVelocityStream) :
        swerveDrive.pointTowardsAndDrive(Constants.redHub, Constants.MAX_ROTATIONAL_SPEED, driveRobotAngularVelocityStream, driveFieldAngularVelocityStream)*/
    }
  }

  public void teleopPeriodic() {
    double matchTime = DriverStation.getMatchTime();
    final double rumbleLength = .5;
    final double rumbleStrength = .3;

    if (matchTime > 130 && matchTime < 130+rumbleLength) {
      driverJoystick.setRumble(RumbleType.kBothRumble, rumbleStrength);
    } else if (matchTime > 105 && matchTime < 105+rumbleLength) {
      driverJoystick.setRumble(RumbleType.kBothRumble, rumbleStrength);
    } else if (matchTime > 80 && matchTime < 80+rumbleLength) {
      driverJoystick.setRumble(RumbleType.kBothRumble, rumbleStrength);
    } else if (matchTime > 55 && matchTime < 55+rumbleLength) {
      driverJoystick.setRumble(RumbleType.kBothRumble, rumbleStrength);
    } else if (matchTime > 30 && matchTime < 30+rumbleLength) {
      driverJoystick.setRumble(RumbleType.kBothRumble, rumbleStrength);
    } else {
      driverJoystick.setRumble(RumbleType.kBothRumble, 0.0);
    }
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
    // todo: align to alliance zone
    // todo: climber up/down
    // todo: intake/hopper movement

    manipulatorJoystick.a().onTrue(intake.toggleIntakeCommand());
    manipulatorJoystick.b().onTrue(Commands.parallel(intake.toggleBeltCommand(), launcher.toggleIntakeCommand()));

    manipulatorJoystick.b().onTrue(Commands.runOnce(() -> {
      if (manipulatorRumbling) {
        manipulatorJoystick.setRumble(RumbleType.kBothRumble, 0.0);
        manipulatorRumbling = false;
      }
      else {
        manipulatorJoystick.setRumble(RumbleType.kBothRumble, 1.0);
        manipulatorRumbling = true;
      }
    }));
  
    manipulatorJoystick.rightTrigger().onTrue(launcher.runLauncherCommand(manipulatorJoystick::getRightTriggerAxis));
    manipulatorJoystick.rightTrigger().onFalse(launcher.stopLauncherCommand());

    manipulatorJoystick.leftTrigger().onTrue(launcher.runLauncherCommand(() -> manipulatorJoystick.getRightTriggerAxis() * 0.75));
    manipulatorJoystick.leftTrigger().onFalse(launcher.stopLauncherCommand());

    manipulatorJoystick.povLeft().whileTrue(launcher.decreaseLauncherAngle());
    manipulatorJoystick.povLeft().whileFalse(Commands.runOnce(() -> launcher.launcherAngleDrive.set(0.0), launcher));
    manipulatorJoystick.povRight().whileTrue(launcher.increaseLauncherAngle());
    manipulatorJoystick.povRight().whileFalse(Commands.runOnce(() -> launcher.launcherAngleDrive.set(0.0), launcher));
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
    return auto.getAutonomousCommand();
  }

  // from https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }
}
