package com.team4687.frc2026.subsystems;

import com.team4687.frc2026.subsystems.body.ClimberSubsystem;
import com.team4687.frc2026.subsystems.body.IntakeSubsystem;
import com.team4687.frc2026.subsystems.body.LauncherSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoolerPathplanner extends SubsystemBase {
    SwerveSubsystem swerve;
    IntakeSubsystem intake;
    LauncherSubsystem launcher;
    ClimberSubsystem climber;

    public CoolerPathplanner(SwerveSubsystem useSwerve, IntakeSubsystem useIntake, LauncherSubsystem useLauncher, ClimberSubsystem useClimber) {
        swerve = useSwerve;
        intake = useIntake;
        launcher = useLauncher;
        climber = useClimber;
    }

    public Command getPath(String pathName) {
        if (pathName.equals("depotAuto")) return depotAuto();

        System.out.println("AUdyuhwa8f\nAHUDUHUHWAUF\nAHUDWHUWDHU*AF\nUAWDUBYWADHYUF\nADNWINGJ(*uh)\njbhuygfygvbhun");
        return Commands.none();
    }

    // helper functions
    
    public Command goToPose(Pose2d pose, double translationSpeed, double rotationSpeed) {
        final double translationTolerance = 0.05; // when to end
        final double rotationTolerance = Units.degreesToRadians(1.5);

        PIDController rotationController = new PIDController(rotationSpeed, .3, 0.0);
        PIDController driveController    = new PIDController(translationSpeed, .3, 0.0);

        return swerve.startRun(() -> {
            rotationController.reset();
            driveController.reset();
        },() -> {
            Pose2d current = swerve.getPose();
            ChassisSpeeds speeds = new ChassisSpeeds(pose.getX()-current.getX(), pose.getY()-current.getY(), pose.getRotation().minus(current.getRotation()).getRadians());
            double mag = Math.sqrt(speeds.vxMetersPerSecond*speeds.vxMetersPerSecond + speeds.vyMetersPerSecond*speeds.vyMetersPerSecond);

            double driveSpeedNow = Math.abs(driveController.calculate(mag));
            double rotationSpeedNow = rotationController.calculate(speeds.omegaRadiansPerSecond);

            double rotationMag = Math.abs(speeds.omegaRadiansPerSecond);
            
            speeds.vxMetersPerSecond /= mag;
            speeds.vyMetersPerSecond /= mag;
            speeds.omegaRadiansPerSecond /= rotationMag;
            

            if (mag > translationTolerance) {
                speeds.vxMetersPerSecond *= driveSpeedNow;
                speeds.vyMetersPerSecond *= driveSpeedNow;                
            }
            else {
                speeds.vxMetersPerSecond = 0;
                speeds.vyMetersPerSecond = 0;
            }
            
            if (rotationMag > rotationTolerance) {
                speeds.omegaRadiansPerSecond *= rotationSpeedNow;
            }
            else {
                speeds.omegaRadiansPerSecond = 0.0;
            }

            System.out.printf("Rotation speed %f\n", speeds.omegaRadiansPerSecond);

            swerve.driveFieldOriented(speeds);
        }).until(() -> {
            Pose2d current = swerve.getPose();
            ChassisSpeeds speeds = new ChassisSpeeds(pose.getX()-current.getX(), pose.getY()-current.getY(), pose.getRotation().minus(current.getRotation()).getRadians());
            double mag = Math.sqrt(speeds.vxMetersPerSecond*speeds.vxMetersPerSecond + speeds.vyMetersPerSecond*speeds.vyMetersPerSecond);
            double rotationMag = Math.abs(speeds.omegaRadiansPerSecond);
            // System.out.printf("rot %f\n", current.getRotation().getDegrees());
            return mag <= translationTolerance && rotationMag <= rotationTolerance; //(speeds.omegaRadiansPerSecond+Math.PI)%Math.PI < rotationTolerance;
        }).finallyDo(() -> {
            rotationController.close();
            driveController.close();
            swerve.driveFieldOriented(new ChassisSpeeds());
        });

    }

    public Pose2d newPose(double x, double y, double t) { return new Pose2d(x, y, new Rotation2d(t)); }
    
    public double rotFlip(double theta) { return DriverStation.getAlliance().get() == Alliance.Red ? theta + Math.PI : theta; }




    // auto paths

    public Command depotAuto() {
        double AUTO_TRANSLATION_SPEED = Units.feetToMeters(3.0);
        double AUTO_ROTATION_SPEED    = Units.degreesToRadians(90.0);

        Command waitCommand = Commands.waitSeconds(0.15);
        waitCommand.addRequirements(swerve);

        // initialize gyroscope to starting position
        // might not play nice with photonvision
        return Commands.sequence(
            Commands.print("HEllo"),
            runOnce(() -> swerve.resetPose(new Pose2d(3.405, 7.470,
                new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue ? Math.PI : 0.0))
            )),
            goToPose(newPose(1.413, 5.956, rotFlip(0)), AUTO_TRANSLATION_SPEED, AUTO_ROTATION_SPEED),
            intake.runIntake(),
            goToPose(newPose(0.872, 5.956, rotFlip(0)), AUTO_TRANSLATION_SPEED*.5, AUTO_ROTATION_SPEED*.5),
            intake.stopIntakeCommand(),
            goToPose(newPose(2.913, 4.675, rotFlip(156.783)), AUTO_TRANSLATION_SPEED, AUTO_ROTATION_SPEED)
        )
        .andThen(Commands.parallel(
            swerve.run(() -> swerve.swerveDrive.lockPose()), // stop the wheels from moving
            Commands.sequence(
                launcher.autoRunLauncherCommand(),
                intake.startBeltCommand(),
                Commands.waitSeconds(6.0),
                launcher.autoStopLauncherCommand()
            )
        ))
        .finallyDo(() -> {
            // clean up in case we got interrupted
            launcher.stopLauncher();
            intake.stopIntake();
            swerve.swerveDrive.drive(new Translation2d(), 0, false, true);
        });
    }
}
