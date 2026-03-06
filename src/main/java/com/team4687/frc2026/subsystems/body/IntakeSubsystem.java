package com.team4687.frc2026.subsystems.body;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    class IntakeSendables implements Sendable {
        public double targetIntakeSpeed = 0.67;
        public double targetBeltSpeed = 0.67;

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("IntakeConfig");
            builder.addDoubleProperty("intakeSpeed", this::getIntakeSpeed, this::setIntakeSpeed);
            builder.addDoubleProperty("beltSpeed", this::getBeltSpeed, this::setBeltSpeed);
        }

        public double getIntakeSpeed() { return targetIntakeSpeed; }
        public double getBeltSpeed() { return targetBeltSpeed; }

        public void setIntakeSpeed(double set) { targetIntakeSpeed = set; }
        public void setBeltSpeed(double set) { targetBeltSpeed = set; }
    }
   
    IntakeSendables sendables = new IntakeSendables();
    Command currentIntakeRunCommand;
    Command currentBeltRunCommand;
    public boolean intakeRunning = false;
    boolean beltRunning = false;

    SparkMax intakeDrive  = new SparkMax(30, MotorType.kBrushless);
    SparkMax intakeRotate = new SparkMax(40, MotorType.kBrushless);

    SparkMax hopperBelt  = new SparkMax(38, MotorType.kBrushless);
    SparkMax hopperExtender = new SparkMax(39, MotorType.kBrushless);

    public RelativeEncoder intakeRotateEncoder = intakeRotate.getEncoder();

    public IntakeSubsystem() {
        SparkBaseConfig intakeDriveConfig  = new SparkMaxConfig().idleMode(IdleMode.kCoast).inverted(true);
        SparkBaseConfig intakeRotateConfig = new SparkMaxConfig().idleMode(IdleMode.kCoast);
        SparkBaseConfig hopperBeltConfig   = new SparkMaxConfig().idleMode(IdleMode.kCoast).inverted(true);
        SparkBaseConfig hopperExtenderConfig  = new SparkMaxConfig().idleMode(IdleMode.kCoast);

        intakeDrive.configure(intakeDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeRotate.configure(intakeRotateConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        hopperBelt.configure(hopperBeltConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        hopperExtender.configure(hopperExtenderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putData( "Intake Config", sendables);
    }

    public void updateIntake() {
        intakeDrive.set(sendables.targetIntakeSpeed);
    }

    public void reverseTargetSpeed() {
        sendables.targetIntakeSpeed *= -1;
    }

    public void settargetIntakeSpeed(double speed) {
        sendables.targetIntakeSpeed = speed;
    }

    public double gettargetIntakeSpeed() {
        return sendables.targetIntakeSpeed;
    }

    public double getActualIntakeSpeed() {
        return intakeDrive.get();
    }

    public void stopIntake() {
        intakeDrive.set(0.0);
    }

    public Command toggleIntakeCommand() {
        return runOnce(() -> {
            System.out.println("Intake toggle");
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

    public Command runIntake() {
        return runOnce(() -> {
            updateIntake();
            updateBelt();
        });
    }

    public Command stopIntakeCommand() {
        return runOnce(() -> {
            stopIntake();
            stopBelt();
        });
    }

    public Command increaseIntakeSpeed() {
        return Commands.runOnce(() -> sendables.targetIntakeSpeed = Math.min(sendables.targetIntakeSpeed+0.05, 1.0));
    }

    public Command decreaseIntakeSpeed() {
        return Commands.runOnce(() -> sendables.targetIntakeSpeed = Math.max(sendables.targetIntakeSpeed-0.05, 0.0));
    }


    public void updateBelt() {
        hopperBelt.set(sendables.targetBeltSpeed);
    }

    public void reverseBeltSpeed() {
        sendables.targetBeltSpeed *= -1;
    }

    public void setTargetBeltSpeed(double speed) {
        sendables.targetBeltSpeed = speed;
    }

    public double getTargetBeltSpeed() {
        return sendables.targetBeltSpeed;
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
        return Commands.runOnce(() -> sendables.targetBeltSpeed = Math.min(sendables.targetBeltSpeed+0.05, 1.0));
    }

    public Command decreaseBeltSpeed() {
        return Commands.runOnce(() -> sendables.targetBeltSpeed = Math.max(sendables.targetBeltSpeed-0.05, 0.0));
    }

    public Command runIntakeAngle(DoubleSupplier val) {
        return run(() -> {
            // Dummy values. Remember to change these!
            if (intakeRotateEncoder.getPosition() > -0.1 && intakeRotateEncoder.getPosition() < 20.0) {
                intakeRotate.set(val.getAsDouble() * 0.25);
            }
            else {
                intakeRotate.set(0.0);
            }
        });
    }

    public Command startIntakeAngle() {
        return Commands.runEnd(() -> {
            // Dummy values. Remember to change these!
            if (intakeRotateEncoder.getPosition() > -0.1 && intakeRotateEncoder.getPosition() < 20.0) {
                intakeRotate.set(-0.25); // inwards?
            }
            else intakeRotate.set(0.0);
        }, () -> intakeRotate.set(0.0), this);
    }

    public Command initializeIntakeAngle() {
        return run(() -> intakeRotate.set(-0.25)) // outwards?
        .until(() -> intakeRotateEncoder.getPosition() < 0.1) // Dummy value
        .andThen(() -> intakeRotate.set(0.0));
    }

    public Command stopIntakeAngle() {
        return Commands.runOnce(() -> intakeRotate.set(0.0));
    }
}
