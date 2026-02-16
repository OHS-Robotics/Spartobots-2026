package com.team4687.frc2026.subsystems.body;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
    double targetIntakeSpeed = 0.30;
    double targetLaunchSpeed = .55;
    double targetLaunchAngle = 0.0;

    Command currentLauncherRunCommand;
    Command currentIntakeRunCommand;
    boolean launcherRunning = false;
    boolean intakeRunning = false;

    DigitalInput limitSwitch = new DigitalInput(0);
    SparkMax launcherAngleDrive = new SparkMax(35, MotorType.kBrushless);
    RelativeEncoder launcherEncoder = launcherAngleDrive.getEncoder();

    SparkMax primaryLauncherLeft    = new SparkMax(31, MotorType.kBrushless); // runs same as primary right
    SparkMax secondaryLauncherLeft  = new SparkMax(32, MotorType.kBrushless); // runs opposite of primary, same as secondary right
    SparkMax primaryLauncherRight   = new SparkMax(33, MotorType.kBrushless); // runs same as primary left
    SparkMax secondaryLauncherRight = new SparkMax(34, MotorType.kBrushless); // runs opposite of primary, same as secondary left

    SparkMax topAgitator    = new SparkMax(36, MotorType.kBrushless); // intake for launcher
    SparkMax bottomAgitator = new SparkMax(37, MotorType.kBrushless); // i don't know how this works with the top agitator

    // todo: shooter controls

    // controls note:
    // y for shooter
    // b for agitator

    public LauncherSubsystem() {
        SparkBaseConfig pLL = new SparkMaxConfig().idleMode(IdleMode.kCoast); // important!
        // if this is set to brake it will damage the motors when stopping because of the flywheels.
        // primary Launcher Left, secondary Launcher Right, etc.
        SparkBaseConfig sLL = new SparkMaxConfig().idleMode(IdleMode.kCoast).inverted(true);
        SparkBaseConfig pLR = new SparkMaxConfig().idleMode(IdleMode.kCoast).follow(primaryLauncherLeft);
        SparkBaseConfig sLR = new SparkMaxConfig().idleMode(IdleMode.kCoast).inverted(true).follow(secondaryLauncherLeft);
        // all launcher motors must be coast

        // top Agitator, bottom Agitator
        SparkBaseConfig tA = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        // note: check orientations of these
        SparkBaseConfig bA = new SparkMaxConfig().idleMode(IdleMode.kBrake).follow(topAgitator);
        

        primaryLauncherLeft.configure(pLL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        secondaryLauncherLeft.configure(sLL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        primaryLauncherRight.configure(pLR, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        secondaryLauncherRight.configure(sLR, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        topAgitator.configure(tA, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        bottomAgitator.configure(bA, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


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
        return primaryLauncherLeft.get();
    }

    public double getActualIntakeSpeed() {
        return topAgitator.get();
    }

    public void stopIntake() {
        topAgitator.set(0.0);
        bottomAgitator.set(0.0);
    }

    public void stopLauncher() {
        // will disable right as well because of the followers
        primaryLauncherLeft.set(0.0);
        secondaryLauncherLeft.set(0.0);
    }

    public void updateIntake() {
        topAgitator.set(targetIntakeSpeed);
        bottomAgitator.set(targetIntakeSpeed);
    }

    public void updateLauncher() {
        // will update right as well because of the followers
        primaryLauncherLeft.set(targetLaunchSpeed);
        secondaryLauncherLeft.set(targetLaunchSpeed);
    }

    public Command toggleLauncherCommand() {
        return runOnce(() -> {
            if (!launcherRunning) {
                updateLauncher();

                currentLauncherRunCommand = run(this::updateLauncher);
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

    public Command runLauncherCommand() {
        return runOnce(() -> {
            if (!launcherRunning) {
                updateLauncher();

                currentLauncherRunCommand = run(this::updateLauncher);
                CommandScheduler.getInstance().schedule(currentLauncherRunCommand);
                launcherRunning = true;
            }
        });
    }

    public Command stopLauncherCommand() {
        return runOnce(() -> {
            if (launcherRunning) {
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
                updateIntake();

                currentIntakeRunCommand = run(this::updateIntake);
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

    public Command decreaseLauncherAngle() {
        return run(() -> {
            if (limitSwitch.get()) {
                launcherEncoder.setPosition(0);
                launcherAngleDrive.set(0.0);
            }
            else {
                launcherAngleDrive.set(-0.35);
            }
        });
    }

    public Command increaseLauncherAngle() {
        return run(() -> {
            if (launcherEncoder.getPosition() > 2.5) { // placeholder
                launcherAngleDrive.set(0.0);
            }
            else {
                launcherAngleDrive.set(0.35);
            }
        });
    }
}
