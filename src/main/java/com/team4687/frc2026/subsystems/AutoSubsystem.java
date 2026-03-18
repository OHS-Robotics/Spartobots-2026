package com.team4687.frc2026.subsystems;

import com.team4687.frc2026.subsystems.body.ClimberSubsystem;
import com.team4687.frc2026.subsystems.body.IntakeSubsystem;
import com.team4687.frc2026.subsystems.body.LauncherSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.team4687.frc2026.subsystems.CoolerPathplanner;

public class AutoSubsystem {
    private SwerveSubsystem swerveDrive;
    private LauncherSubsystem launcher;
    private IntakeSubsystem intake;
    private ClimberSubsystem climber;

    private CoolerPathplanner pathPlanner;

    public AutoSubsystem(SwerveSubsystem useSwerve, LauncherSubsystem useLauncher, IntakeSubsystem useIntake, ClimberSubsystem useClimber) {
        swerveDrive = useSwerve;
        launcher = useLauncher;
        intake = useIntake;
        climber = useClimber;
        pathPlanner = new CoolerPathplanner(useSwerve, useIntake, useLauncher, useClimber);
  
        // todo: smartdashboard auto chooser
    }


    public Command getAutonomousCommand() {
        return pathPlanner.getPath("depotAuto");
    }
}
