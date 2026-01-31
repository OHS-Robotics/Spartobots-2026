package com.team4687.frc2026.subsystems;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import java.lang.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import com.team4687.frc2026.Constants;
import com.team4687.frc2026.subsystems.vision.VisionSubsystem;

public class SwerveSubsystem extends SubsystemBase {

    public final SwerveDrive swerveDrive;
    public final VisionSubsystem vision = new VisionSubsystem();
    PathPlannerAuto testPath;

    public SwerveSubsystem(File directory) {
        //VisionSubsystem.initVision();

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
        }
        catch (Exception e) {
            throw new RuntimeException(e);
        }

        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        final boolean enableFeedForward = true;


        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> {
                if (enableFeedForward) {
                    this.swerveDrive.drive(speeds,
                    swerveDrive.kinematics.toSwerveModuleStates(speeds),
                    feedforwards.linearForces());
                }
                else {
                    this.swerveDrive.setChassisSpeeds(speeds);
                }
            },
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        registerNamedCommands();
        registerPaths();
    }

    private void registerNamedCommands() {
        // todo: move this to a subsystem
        NamedCommands.registerCommand("testOperation", runOnce(() -> System.out.println("Wow doing a dummy operation!")));
    }
    
    private void registerPaths() {
        // todo: move this to a subsystem
        testPath = new PathPlannerAuto("testAuto");
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                                DoubleSupplier headingX, DoubleSupplier headingY) {
        return run(() -> {
            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                scaledInputs.getX(), scaledInputs.getY(),
                headingX.getAsDouble(), headingY.getAsDouble(),
                swerveDrive.getOdometryHeading().getRadians(),
                swerveDrive.getMaximumChassisVelocity()
            ));

        });
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
                                DoubleSupplier angularRotationX) {
        return run(() -> {
            swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(), translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                              angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(), true, false);
        });
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOrientedCommand(ChassisSpeeds velocity) {
        return run(() -> swerveDrive.driveFieldOriented(velocity));
    }

    public Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity) {
        return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
    }

    /**
     * Change the position, relative to the robot, by a given translation at a given speed. All units in meters. +X is backwards, +Y is right
     * @param translation The distance on each axis to translate by, in meters
     * @param speed The speed at which to move, in meters per second. If this is greater than {@link frc.robot.Constants#MAX_SPEED} it will be clamped
     * @return 
     */
    public Command changePosition(Translation2d translation, double speed) {
        final double limitedSpeed = Math.min(speed, Constants.MAX_SPEED);
        return run(() -> drive(translation.times(limitedSpeed/translation.getNorm()), 0.0, false))
        .withTimeout(translation.getNorm()/limitedSpeed);
    }

    public Command changeRotation(double angle, double speed) {
      final double limitedSpeed = Math.min(speed, Constants.MAX_ROTATIONAL_SPEED);
      return run(() -> drive(new Translation2d(0.0, 0.0), limitedSpeed, false)).withTimeout(angle/limitedSpeed);
    }

    /**
     * @param endpoint The point at which to move the robot to. The robot will rotate to the rotational component AFTER moving!
     * @param speed The speed at which to move, in meters per second. If this is greater than {@link frc.robot.Constants#MAX_SPEED} it will be clamped 
     */
    public Command driveTo(Pose2d endpoint, double translationSpeed, double rotationSpeed) {
      final double limitedSpeed = Math.min(translationSpeed, Constants.MAX_SPEED);
      final Translation2d translation = endpoint.minus(getPose()).getTranslation();
      return changePosition(translation, limitedSpeed).andThen(changeRotation(
        endpoint.getRotation().minus(getPose().getRotation()).getDegrees(),
        rotationSpeed));
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public Command getAutonomousCommand() {
        //return changePosition(new Translation2d(0.0, 2.0), Constants.MAX_SPEED/2.0);
        //return driveTo(new Pose2d(0.0, 2.0, new Rotation2d(0.0)), Constants.MAX_SPEED/2.0, Constants.MAX_ROTATIONAL_SPEED/2.0);
        return testPath;
    }

    public void update() {
        swerveDrive.updateOdometry();
        vision.updatePoseEstimate(swerveDrive);
    }
}
