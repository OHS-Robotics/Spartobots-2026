package com.team4687.frc2026.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team4687.frc2026.subsystems.body.IntakeSubsystem;
import com.team4687.frc2026.subsystems.body.LauncherSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSubsystem {
    private SwerveSubsystem swerveDrive;
    private LauncherSubsystem launcher;
    private IntakeSubsystem intake;
    private SendableChooser<Command> pathChooser;

    public AutoSubsystem(SwerveSubsystem useSwerve, LauncherSubsystem useLauncher, IntakeSubsystem useIntake) {
        swerveDrive = useSwerve;
        launcher = useLauncher;
        intake = useIntake;

        configureAutoBuilder();
        pathChooser = AutoBuilder.buildAutoChooser("Back fuel launch");
        SmartDashboard.putData("Auto path", pathChooser);
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
            swerveDrive::getPose,
            swerveDrive::resetPose,
            swerveDrive::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> {
                if (enableFeedForward) {
                    swerveDrive.swerveDrive.drive(speeds,
                    swerveDrive.swerveDrive.kinematics.toSwerveModuleStates(speeds),
                    feedforwards.linearForces());
                }
                else {
                    swerveDrive.swerveDrive.setChassisSpeeds(speeds);
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
            swerveDrive
        );

        registerNamedCommands();
        registerPaths();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("ingestFuel", intake.toggleIntakeCommand());
        NamedCommands.registerCommand("expelFuel", launcher.toggleIntakeCommand().andThen(launcher.toggleLauncherCommand()));
    }

    private void registerPaths() {
        // when we make paths they go here
    }

    public Command getAutonomousCommand() {
        return pathChooser.getSelected();
    }
}
