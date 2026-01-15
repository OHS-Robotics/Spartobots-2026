package com.team4687.frc2026.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.pathplanner.lib.commands.PathPlannerAuto;

import java.lang.Math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;



import com.team4687.frc2026.Constants;
import com.team4687.frc2026.SwerveDriveInterface;

public class SwerveSubsystem implements SwerveDriveInterface {

    public final SwerveDrive swerveDrive;

    public SwerveSubsystem(File directory) {
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
        }
        catch (Exception e) {
            throw new RuntimeException(e);
        }

        configureAutoBuilder();

    }

    /*private void configureAutoBuilder() {
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
    }*/

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, fieldRelative, isOpenLoop);
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
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

    @Override
    public Command driveCommand(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.drive(new Translation2d(velocity.get().vxMetersPerSecond * swerveDrive.getMaximumChassisVelocity(), velocity.get().vyMetersPerSecond * swerveDrive.getMaximumChassisVelocity()),
                              velocity.get().omegaRadiansPerSecond * swerveDrive.getMaximumChassisAngularVelocity(), true, false);
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
     * Change the position, relative to the robot, by a given translation at a given speed. All units in meters.
     * @param translation The distance on each axis to translate by, in meters
     * @param speed The speed at which to move, in meters. If this is greater than {@link frc.robot.Constants#MAX_SPEED} it will be clamped
     * @return 
     */
    public Command changePosition(Translation2d translation, double speed) {
        final double limitedSpeed = Math.min(speed, Constants.MAX_SPEED);
        return run(() -> drive(new ChassisSpeeds(translation.times(limitedSpeed/translation.getNorm()).getX(), translation.times(limitedSpeed/translation.getNorm()).getY(), 0), false, false))
        .withTimeout(translation.getNorm()/limitedSpeed);
    }

    @Override
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPose'");
    }

    @Override
    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    @Override
    public ChassisSpeeds getMeasuredSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    @Override
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("simpleauto");
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setModuleStates'");
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addVisionMeasurement'");
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addVisionMeasurement'");
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }
}
