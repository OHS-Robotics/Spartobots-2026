package com.team4687.frc2026.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import java.lang.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import com.team4687.frc2026.Constants;
import com.team4687.frc2026.subsystems.vision.VisionSubsystem;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveSendables sendables = new SwerveSendables();

    public final SwerveDrive swerveDrive;
    public final VisionSubsystem vision = new VisionSubsystem();

    public SwerveSubsystem(File directory) {
        //VisionSubsystem.initVision();

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
        }
        catch (Exception e) {
            throw new RuntimeException(e);
        }

        SmartDashboard.putData("Swerve Config", sendables);
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

    public Command driveCommand(Supplier<ChassisSpeeds> robotVelocity, Supplier<ChassisSpeeds> fieldVelocity) {
        return run(() -> {
            if (sendables.getFieldOriented()) swerveDrive.driveFieldOriented(fieldVelocity.get());
            else {
                ChassisSpeeds speeds = robotVelocity.get();
                swerveDrive.drive(speeds);
            }
        });
    }

    /**
     * @param endpoint The point at which to move the robot to.
     * @param translationSpeed The speed at which to move, in meters per second. If this is greater than {@link frc.robot.Constants#MAX_SPEED} it will be clamped
     * @param rotationSpeed The speed at which to move, in radians per second. If this is greater than {@link frc.robot.Constants#MAX_ROTATIONAL_SPEED} it will be clamped
     */
    public Command driveTo(Pose2d endpoint, double translationSpeed, double rotationSpeed) {
        translationSpeed = Math.min(translationSpeed, Constants.MAX_SPEED);
        rotationSpeed = Math.min(rotationSpeed, Constants.MAX_ROTATIONAL_SPEED);
        PathConstraints constraints = new PathConstraints(Constants.MAX_SPEED, Constants.MAX_ACCELERATION, Constants.MAX_ROTATIONAL_SPEED, Constants.MAX_ROTATIONAL_ACCELERATION);
        return AutoBuilder.pathfindToPose(endpoint, constraints);

        /*final double limitedSpeed = Math.min(translationSpeed, Constants.MAX_SPEED);
        final Translation2d translation = endpoint.minus(getPose()).getTranslation();
        return changePosition(translation, limitedSpeed).andThen(changeRotation(
            endpoint.getRotation().minus(getPose().getRotation()).getDegrees(),
            rotationSpeed));*/

        
    }
    
    /**
     * Change the position, relative to the robot, by a given translation at a given speed. All units in meters. +X is backwards, +Y is right
     * @param translation The distance on each axis to translate by, in meters
     * @param speed The speed at which to move, in meters per second. If this is greater than {@link frc.robot.Constants#MAX_SPEED} it will be clamped
     * @return 
     */
    public Command changePosition(Translation2d translation, double translationSpeed) {
        final Pose2d targetPose = getPose().plus(new Transform2d(translation.getX(), translation.getY(), new Rotation2d()));
        
        return driveTo(targetPose, translationSpeed, 1.0);
    }

    public Command changeRotation(double angle, double speed) {
        final double limitedSpeed = Math.min(speed, Units.radiansToDegrees(Constants.MAX_ROTATIONAL_SPEED));
        double targetAngle = swerveDrive.getOdometryHeading().plus(new Rotation2d(angle)).getRadians();
        return run(() -> drive(
            new Translation2d(0.0, 0.0),
            Math.min(
                Math.max(targetAngle-swerveDrive.getOdometryHeading().getRadians(), -limitedSpeed),
                limitedSpeed
            ),
            false
        )).until(() -> Math.abs(targetAngle-swerveDrive.getOdometryHeading().getRadians()) < 0.174); // 10 degrees
    }

    public Command rotateTo(double angle, double speed) {
        return changeRotation(angle - swerveDrive.getYaw().getDegrees(), speed);
    }

    public Command pointTowards(Pose2d target, double speed) {
        Transform2d transform = target.minus(getPose());
        
        return rotateTo(Math.atan2(transform.getY(), transform.getX()), speed);
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

    public void update() {
        swerveDrive.updateOdometry();
        vision.updatePoseEstimate(swerveDrive);
    }
}
