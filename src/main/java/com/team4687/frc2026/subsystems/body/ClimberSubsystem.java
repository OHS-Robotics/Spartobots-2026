package com.team4687.frc2026.subsystems.body;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    public SparkMax climber = new SparkMax(32, MotorType.kBrushless); // Todo: check this CAN ID
    public RelativeEncoder climberEncoder = climber.getEncoder();

    public ClimberSubsystem() {

    }

    public Command climberUp() {
        return run(() -> {
            // Dummy value, remember to change.
            if (climberEncoder.getPosition() > -0.1 && climberEncoder.getPosition() < 10.0) {
                climber.set(0.5);
            }
            else climber.set(0.0);
        });
    }

    public Command climberDown() {
        return run(() -> {
            // Dummy value, remember to change.
            if (climberEncoder.getPosition() > -0.1 && climberEncoder.getPosition() < 10.0) {
                climber.set(-0.5);
            }
            else climber.set(0.0);
        });
    }

    public Command climberStop() {
        return runOnce(() -> climber.set(0.0));
    }

    public Command initializeClimber() {
        return run(() -> climber.set(0.25)) // move it up
        .until(() -> climberEncoder.getPosition() > 9.9) // Dummy value
        .andThen(() -> climber.set(0.0));
    }
}
