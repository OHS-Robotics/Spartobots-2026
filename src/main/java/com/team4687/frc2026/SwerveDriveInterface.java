package com.team4687.frc2026;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public interface SwerveDriveInterface extends Subsystem {
    void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop);

    void setModuleStates(SwerveModuleState[] desiredStates);

    ChassisSpeeds getMeasuredSpeeds();

    Rotation2d getGyroYaw();

    Pose2d getPose();

    void setPose(Pose2d pose);

    void resetPose(Pose2d pose);

    default Rotation2d getHeading() {
        return getPose().getRotation();
    }

    default void setHeading(Rotation2d heading) {
        setPose(new Pose2d(getPose().getTranslation(), heading));
    }

    default void zeroHeading() {
        setHeading(new Rotation2d());
    }

    void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds);
    void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);

    void periodic();

    default void configureAutoBuilder() {
        final boolean enableFeedForward = true;


        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                // Use APIs from SwerveDrive interface
                this::getPose, 
                this::setPose,
                this::getMeasuredSpeeds,
                (speeds) -> this.drive(speeds, false, true),

                // Configure the Auto PIDs
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)),

                // Specify the PathPlanner Robot Config
                config,

                // Path Flipping: Determines if the path should be flipped based on the robot's alliance color
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red),

                // Specify the drive subsystem as a requirement of the command
                this);
    }

    Command driveFieldOrientedCommand(SwerveInputStream driveRobotAngularVelocityStream);
}