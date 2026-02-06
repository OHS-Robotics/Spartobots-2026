package com.team4687.frc2026.subsystems.body;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
    double targetIntakeSpeed = 0.30;
    double targetLaunchSpeed = .55;

    Command currentLauncherRunCommand;
    Command currentIntakeRunCommand;
    boolean launcherRunning = false;
    boolean intakeRunning = false;

    SparkMax primaryLauncher   = new SparkMax(18, MotorType.kBrushless); // dummy can ids
    SparkMax secondaryLauncher = new SparkMax(3, MotorType.kBrushless); // runs opposite of primary
    SparkMax launcherIntake    = new SparkMax(20, MotorType.kBrushless); // should not run at full speed, runs with launcher
    SparkMax secondaryIntake   = new SparkMax(41, MotorType.kBrushless); // runs with other intake

    // controls note:
    // d-pad up/down control launcher speed
    // d-pad left-right control launcher intake(indexer)
    // bumpers control intake subsystem speed

    public LauncherSubsystem() {
        SparkBaseConfig primaryLaunchConfig = new SparkMaxConfig().idleMode(IdleMode.kCoast).inverted(true); // important!
        // if this is set to brake it will damage the motors when stopping because of the flywheels.
        SparkBaseConfig secondaryLaunchConfig = new SparkMaxConfig().idleMode(IdleMode.kCoast);
        SparkBaseConfig primaryIntakeConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        SparkBaseConfig secondaryIntakeConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(true);

        // all launcher motors must be coast

        /*SparkBaseConfig secondaryConfig = coastConfig
        .follow(primaryLauncher) // should follow primary launcher motor
        .inverted(true); // run opposite. this will cause damage if not inverted!
        SparkBaseConfig secondaryIntakeConfig = coastConfig
        .follow(launcherIntake)
        .inverted(true);*/
        //primaryLauncher.configure(coastConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //secondaryLauncher.configure(secondaryConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        primaryLauncher.configure(primaryLaunchConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        secondaryLauncher.configure(secondaryLaunchConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        launcherIntake.configure(primaryIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        secondaryIntake.configure(secondaryIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //secondaryIntake.configure(secondaryIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void reverseSpeeds() {
        targetIntakeSpeed *= -1;
        targetLaunchSpeed *= -1;
    }

    public void setTargetLaunchSpeed(double speed) {
        targetLaunchSpeed = speed;
    }

    public void setTargetIntakeSpeed(double speed) {
        targetIntakeSpeed = speed;
    }

    public double getTargetLaunchSpeed() {
        return targetLaunchSpeed;
    }

    public double getTargetIntakeSpeed() {
        return targetIntakeSpeed;
    }

    public double getActualLaunchSpeed() {
        return primaryLauncher.get();
    }

    public double getActualIntakeSpeed() {
        return launcherIntake.get();
    }

    public void stopIntake() {
        secondaryIntake.set(0.0);
        launcherIntake.set(0.0);
    }

    public void stopLauncher() {
        secondaryLauncher.set(0.0);
        primaryLauncher.set(0.0);
    }

    public void startIntake() {
        //primaryLauncher.set(targetLaunchSpeed);
        launcherIntake.set(targetIntakeSpeed);
        secondaryIntake.set(targetIntakeSpeed);
    }

    public void startLauncher() {
        primaryLauncher.set(targetLaunchSpeed);
        secondaryLauncher.set(targetLaunchSpeed);
    }

    public Command toggleLauncherCommand() {
        return runOnce(() -> {
            if (!launcherRunning) {
                startLauncher();

                currentLauncherRunCommand = run(() -> {
                    primaryLauncher.set(targetLaunchSpeed);
                    secondaryLauncher.set(targetLaunchSpeed);
                });
                CommandScheduler.getInstance().schedule(currentLauncherRunCommand);
                launcherRunning = true;
            }
            else {
                stopLauncher();

                currentLauncherRunCommand.end(false);
                currentLauncherRunCommand = null;
                launcherRunning = false;
            }
        });
    }

    public Command toggleIntakeCommand() {
        return runOnce(() -> {
            if (!intakeRunning) {
                startIntake();

                currentIntakeRunCommand = run(() -> {
                    launcherIntake.set(targetIntakeSpeed);
                    secondaryIntake.set(targetIntakeSpeed);
                });
                CommandScheduler.getInstance().schedule(currentIntakeRunCommand);
                intakeRunning = true;
            }
            else {
                stopIntake();

                currentIntakeRunCommand.end(false);
                currentIntakeRunCommand = null;
                intakeRunning = false;
            }
        });
    }

    public Command increaseIntakeSpeed() {
        return Commands.runOnce(() -> targetIntakeSpeed = Math.min(targetIntakeSpeed+0.05, 1.0));
    }

    public Command decreaseIntakeSpeed() {
        return Commands.runOnce(() -> targetIntakeSpeed = Math.max(targetIntakeSpeed-0.05, 0.0));
    }

    public Command increaseLaunchSpeed() {
        return Commands.runOnce(() -> targetLaunchSpeed = Math.min(targetLaunchSpeed+0.05, 1.0));
    }

    public Command decreaseLaunchSpeed() {
        return Commands.runOnce(() -> targetLaunchSpeed = Math.max(targetLaunchSpeed-0.05, 0.0));
    }
}
