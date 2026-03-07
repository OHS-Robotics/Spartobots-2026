package com.team4687.frc2026.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team4687.frc2026.subsystems.body.ClimberSubsystem;
import com.team4687.frc2026.subsystems.body.IntakeSubsystem;
import com.team4687.frc2026.subsystems.body.LauncherSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoSubsystem {
    private SwerveSubsystem swerveDrive;
    private LauncherSubsystem launcher;
    private IntakeSubsystem intake;
    private ClimberSubsystem climber;
    private SendableChooser<Command> pathChooser;

    private Command testAuto;

    public AutoSubsystem(SwerveSubsystem useSwerve, LauncherSubsystem useLauncher, IntakeSubsystem useIntake, ClimberSubsystem useClimber) {
        swerveDrive = useSwerve;
        launcher = useLauncher;
        intake = useIntake;
        climber = useClimber;
  
        configureAutoBuilder();
        pathChooser = AutoBuilder.buildAutoChooser("depotAuto");
        SmartDashboard.putData("Auto path", pathChooser);
    }

    private void configureAutoBuilder() {
        final boolean enableFeedForward = false;


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
                //if (DriverStation.getAlliance().get() == Alliance.Red) speeds = speeds.times(-1);

                if (enableFeedForward) {
                    speeds=new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
                    swerveDrive.swerveDrive.drive(speeds,
                    swerveDrive.swerveDrive.kinematics.toSwerveModuleStates(speeds),
                    feedforwards.linearForces());
                }
                else {
                    speeds=new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
                    swerveDrive.swerveDrive.setChassisSpeeds(speeds);
                }
            },
            new PPHolonomicDriveController(new PIDConstants(1.0, 0.0, 0.0), new PIDConstants(1.0, 0.0, 0.0)),
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
        NamedCommands.registerCommand("runIntake", intake.runIntake());
        NamedCommands.registerCommand("stopIntake", intake.stopIntakeCommand());
        // Note: running the intake angle might be dangerous, we've had some problems with it catching.
        NamedCommands.registerCommand("runLauncher", launcher.autoRunLauncherCommand().andThen(intake.startIntakeAngle()).andThen(intake.startBeltCommand()));
        NamedCommands.registerCommand("stopLauncher", launcher.autoStopLauncherCommand().andThen(intake.stopIntakeAngle()).andThen(intake.stopBeltCommand()));
        NamedCommands.registerCommand("runClimber", climber.climberDown());
        NamedCommands.registerCommand("readyClimber", climber.initializeClimber());
    }

    private void registerPaths() {
        // when we make paths they go here
        testAuto = Commands.none();
    }

    public Command getAutonomousCommand() {
        return pathChooser.getSelected();
    }
}
