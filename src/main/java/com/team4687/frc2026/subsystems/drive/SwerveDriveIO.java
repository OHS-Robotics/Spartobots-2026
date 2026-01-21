package com.team4687.frc2026.subsystems.drive;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team4687.frc2026.Constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveDriveIO extends Subsystem {
    void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop);

    void setModuleStates(SwerveModuleState[] desiredStates);

    ChassisSpeeds getMeasuredSpeeds();

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

    default ChassisSpeeds maxSpeedAdjust(ChassisSpeeds speeds) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double omega = speeds.omegaRadiansPerSecond;
        if(Math.abs(vx) > Constants.MAX_SPEED) {
            vx = Constants.MAX_SPEED * Math.abs(vx) / vx;
        }
        if(Math.abs(vy) > Constants.MAX_SPEED) {
            vy = Constants.MAX_SPEED * Math.abs(vy) / vy;
        }
        if(Math.abs(omega) > Constants.MAX_ROTATIONAL_SPEED) {
            omega = Constants.MAX_ROTATIONAL_SPEED * Math.abs(omega) / omega;
        }
        return new ChassisSpeeds(vx, vy, omega);
    }

    default void configureAutoBuilder() {
        //final boolean enableFeedForward = true;
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
                this::resetPose,
                this::getMeasuredSpeeds,
                (speeds) -> this.drive(maxSpeedAdjust(speeds), false, true),

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

    Command getAutonomousCommand();

    Command driveCommand(Supplier<ChassisSpeeds> velocity);

    Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity);
}