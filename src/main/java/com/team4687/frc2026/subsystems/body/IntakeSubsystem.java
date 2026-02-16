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

public class IntakeSubsystem extends SubsystemBase {
    double targetIntakeSpeed = 0.5;
    double targetBeltSpeed = 0.5;
    Command currentIntakeRunCommand;
    Command currentBeltRunCommand;
    public boolean intakeRunning = false;
    boolean beltRunning = false;

    SparkMax intakeDrive  = new SparkMax(30, MotorType.kBrushless);
    SparkMax intakeRotate = new SparkMax(40, MotorType.kBrushless);

    SparkMax hopperBelt  = new SparkMax(38, MotorType.kBrushed);
    SparkMax hopperExtender = new SparkMax(39, MotorType.kBrushed);

    public IntakeSubsystem() {
        SparkBaseConfig intakeDriveConfig  = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        SparkBaseConfig intakeRotateConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        SparkBaseConfig hopperBeltConfig   = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        SparkBaseConfig hopperExtenderConfig  = new SparkMaxConfig().idleMode(IdleMode.kBrake);

        intakeDrive.configure(intakeDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeRotate.configure(intakeRotateConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        hopperBelt.configure(hopperBeltConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        hopperExtender.configure(hopperExtenderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void updateIntake() {
        intakeDrive.set(targetIntakeSpeed);
    }

    public void reverseTargetSpeed() {
        targetIntakeSpeed *= -1;
    }

    public void setTargetIntakeSpeed(double speed) {
        targetIntakeSpeed = speed;
    }

    public double getTargetIntakeSpeed() {
        return targetIntakeSpeed;
    }

    public double getActualIntakeSpeed() {
        return intakeDrive.get();
    }

    public void stopIntake() {
        intakeDrive.set(0.0);
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


    public void updateBelt() {
        hopperBelt.set(targetBeltSpeed);
    }

    public void reverseBeltSpeed() {
        targetBeltSpeed *= -1;
    }

    public void setTargetBeltSpeed(double speed) {
        targetBeltSpeed = speed;
    }

    public double getTargetBeltSpeed() {
        return targetBeltSpeed;
    }

    public double getActualBeltSpeed() {
        return hopperBelt.get();
    }

    public void stopBelt() {
        hopperBelt.set(0.0);
    }

    public Command toggleBeltCommand() {
        return runOnce(() -> {
            if (!beltRunning) {
                updateBelt();
                currentBeltRunCommand = run(this::updateBelt);
                CommandScheduler.getInstance().schedule(currentBeltRunCommand);
                beltRunning = true;
            }
            else {
                stopBelt();
                currentBeltRunCommand.end(false);
                currentBeltRunCommand = null;
                beltRunning = false;
            }
        });
    }

    public Command increaseBeltSpeed() {
        return Commands.runOnce(() -> targetBeltSpeed = Math.min(targetBeltSpeed+0.05, 1.0));
    }

    public Command decreaseBeltSpeed() {
        return Commands.runOnce(() -> targetBeltSpeed = Math.max(targetBeltSpeed-0.05, 0.0));
    }

    // todo: moving hopper and intake
}
