package com.team4687.frc2026.subsystems.body;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    public SparkMax climber = new SparkMax(34, MotorType.kBrushless); // Todo: check this CAN ID
    public RelativeEncoder climberEncoder = climber.getEncoder();

    public ClimberSubsystem() {
        SparkBaseConfig config = new SparkMaxConfig().idleMode(IdleMode.kBrake);

        climber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command climberUp() {
        return run(() -> {
            // Dummy value, remember to change.
            if (climberEncoder.getPosition() < 55.0) {
                climber.set(1);
            }
            else climber.set(0.0);
        }).finallyDo((boolean interrupt) -> {
            climber.set(0);
        });
    }

    public Command climberDown() {
        return run(() -> {
            // Dummy value, remember to change.
            if (climberEncoder.getPosition() > 1) {
                climber.set(-1);
            }
            else climber.set(0.0);
        }).finallyDo((boolean interrupt) -> {
            climber.set(0);
        });
    }

    public Command climberStop() {
        return runOnce(() -> climber.set(0.0));
    }

    public Command initializeClimber() {
        return run(() -> climber.set(0.25)) // move it up
        .until(() -> climberEncoder.getPosition() > 55.0) // Dummy value
        .andThen(() -> climber.set(0.0));
    }
}
