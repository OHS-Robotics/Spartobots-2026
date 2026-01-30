package com.team4687.frc2026.subsystems.body;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    double targetIntakeSpeed = 0.5;

    SparkMax mainMotor = new SparkMax(0, MotorType.kBrushed); // dummy values

    public void enable() {
        mainMotor.set(targetIntakeSpeed);
    }

    public void disable() {
        mainMotor.set(0.0);
    }

    public void reverse() {
        if (mainMotor.get() != 0) {
            targetIntakeSpeed *= -1;
            mainMotor.set(targetIntakeSpeed);
        }

    }

    public void setSpeed(double speed) {
        if (mainMotor.get() != 0) {
            targetIntakeSpeed = speed;
            mainMotor.set(targetIntakeSpeed);            
        }
    }

    public double getTargetSpeed() {
        return targetIntakeSpeed;
    }

    public double getActualSpeed() {
        return mainMotor.get();
    }

    
}
