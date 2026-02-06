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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    double targetIntakeSpeed = 0.5;
    Command currentRunCommand;
    boolean running = false;

    SparkMax mainMotor = new SparkMax(2, MotorType.kBrushless); // dummy values

    public IntakeSubsystem() {
        SparkBaseConfig config = new SparkMaxConfig().idleMode(IdleMode.kBrake); // important!
        // this is assuming there is not a flywheel on the shaft. a flywheel will damage the motor.

        mainMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void enable() {
        mainMotor.set(targetIntakeSpeed);
    }

    public void reverseTargetSpeed() {
        targetIntakeSpeed *= -1;
    }

    public void setTargetSpeed(double speed) {
        targetIntakeSpeed = speed;
    }

    public double getTargetSpeed() {
        return targetIntakeSpeed;
    }

    public double getActualSpeed() {
        return mainMotor.get();
    }

    public void stop() {
        mainMotor.set(0.0);
    }

    public void start() {
        mainMotor.set(targetIntakeSpeed);
    }

    public Command toggleCommand() {
        return runOnce(() -> {
            if (!running) {
                start();
                currentRunCommand = run(() -> mainMotor.set(targetIntakeSpeed));
                CommandScheduler.getInstance().schedule(currentRunCommand);
                running = true;
            }
            else {
                stop();
                currentRunCommand.end(false);
                currentRunCommand = null;
                running = false;
            }
        });
    }

    public Command runCommand() {
        currentRunCommand = new RunCommand(() -> mainMotor.set(targetIntakeSpeed), this); // put the command in a variable so it can be interrupted later
        
        return currentRunCommand;
    }

    public Command stopCommand() {
        return runOnce(() -> {
            currentRunCommand.end(false);
            this.stop();
        });
    }

    public Command increaseSpeed() {
        return Commands.runOnce(() -> targetIntakeSpeed = Math.min(targetIntakeSpeed+0.05, 1.0));
    }

    public Command decreaseSpeed() {
        return Commands.runOnce(() -> targetIntakeSpeed = Math.max(targetIntakeSpeed-0.05, 0.0));
    }
}
