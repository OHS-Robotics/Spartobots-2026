package com.team4687.frc2026.simulation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.team4687.frc2026.Constants;
import com.team4687.frc2026.subsystems.drive.SwerveDriveIO;

public class SwerveDriveSim implements SwerveDriveIO {
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;

    private final PathPlannerAuto path;

    public SwerveDriveSim() {
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(Constants.driveTrainSimulationConfig, new Pose2d(3, 3, new Rotation2d())));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // A field2d widget for debugging.c
        field2d = new Field2d();
        SmartDashboard.putData("simulation field", field2d);

        configureAutoBuilder();

        path = new PathPlannerAuto("test");
    }

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop) {
        simulatedDrive.runChassisSpeeds(maxSpeedAdjust(speeds), new Translation2d(0, 0), fieldRelative, false);
    }

    @Override
    public Command driveCommand(Supplier<ChassisSpeeds> velocity) {
        return run(() -> this.drive(velocity.get(), false, false));
    }

    @Override
    public Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity) {
        return run(() -> this.drive(velocity.get(), true, false));
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    @Override
    public ChassisSpeeds getMeasuredSpeeds() {
        return simulatedDrive.getMeasuredSpeedsRobotRelative(true);
    }

    @Override
    public Command getAutonomousCommand() {
        return path;
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
        simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        // update the odometry of the SimplifedSwerveSimulation instance
        simulatedDrive.periodic();

        // send simulation data to dashboard for testing
        field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2d.getObject("odometry").setPose(getPose());
    }

    @Override
    public void resetPose(Pose2d pose) {
        simulatedDrive.resetOdometry(pose);
    }
}
