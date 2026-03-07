package com.team4687.frc2026.subsystems;

import com.team4687.frc2026.subsystems.body.ClimberSubsystem;
import com.team4687.frc2026.subsystems.body.IntakeSubsystem;
import com.team4687.frc2026.subsystems.body.LauncherSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
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
        switch (pathName) {
            case "depotAuto": return 
        }
    }

    // helper functions
    
    public Command goToPose(Pose2d pose) {
        return swerve.run(() -> {
            Pose2d current = swerve.getPose();
            ChassisSpeeds speeds = new ChassisSpeeds(pose.getX()-current.getX(), pose.getY()-current.getY(), pose.getRotation().minus(current.getRotation()).getRadians());
            double mag = Math.sqrt(speeds.vxMetersPerSecond*speeds.vxMetersPerSecond + speeds.vyMetersPerSecond*speeds.vyMetersPerSecond);
            speeds.vxMetersPerSecond /= mag;
            speeds.vyMetersPerSecond /= mag;
        });
    }

}
