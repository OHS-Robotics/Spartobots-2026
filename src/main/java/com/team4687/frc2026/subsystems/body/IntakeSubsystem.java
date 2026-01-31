package com.team4687.frc2026.subsystems.body;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    double targetIntakeSpeed = 0.5;
    Command currentRunCommand;

    SparkMax mainMotor = new SparkMax(0, MotorType.kBrushed); // dummy values

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

    public Command toggle() {
        return Commands.none();
    }

    public Command increaseSpeed() {
        return run(() -> targetIntakeSpeed = Math.min(targetIntakeSpeed+0.01, 1.0));
    }

    public Command decreaseSpeed() {
        return run(() -> targetIntakeSpeed = Math.max(targetIntakeSpeed-0.01, 0.0));
    }
}
