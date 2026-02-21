package com.team4687.frc2026.subsystems.body;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team4687.frc2026.subsystems.body.HubSolver;

public class LauncherSubsystem extends SubsystemBase {
    public class LauncherSendables implements Sendable {

        public double targetIntakeSpeed = 0.4;
        public double targetLaunchSpeed = 0.75;

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("LauncherConfig");
            builder.addDoubleProperty("intakeSpeed", this::getIntakeSpeed, this::setIntakeSpeed);
            builder.addDoubleProperty("launchSpeed", this::getLaunchSpeed, this::setLaunchSpeed);
        }

        public double getIntakeSpeed() {
            return targetIntakeSpeed;
        }

        public void setIntakeSpeed(double set) {
            targetIntakeSpeed = set;
        }
        
        public double getLaunchSpeed() {
            return targetLaunchSpeed;
        }

        public void setLaunchSpeed(double set) {
            targetLaunchSpeed = set;
        }
    }

    public LauncherSendables sendables = new LauncherSendables();
    double targetLaunchAngle = 0.0; // in encoder rotations, not degrees

    HubSolver solver = new HubSolver();

    Command currentLauncherRunCommand;
    Command currentIntakeRunCommand;
    boolean launcherRunning = false;
    boolean intakeRunning = false;

    DigitalInput limitSwitch = new DigitalInput(0);
    public SparkMax launcherAngleDrive = new SparkMax(35, MotorType.kBrushless);
    public RelativeEncoder launcherEncoder = launcherAngleDrive.getEncoder();

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
        SparkBaseConfig sLL = new SparkMaxConfig().idleMode(IdleMode.kCoast).follow(primaryLauncherLeft);
        SparkBaseConfig pLR = new SparkMaxConfig().idleMode(IdleMode.kCoast).inverted(true);
        SparkBaseConfig sLR = new SparkMaxConfig().idleMode(IdleMode.kCoast).follow(primaryLauncherRight);
        // all launcher motors must be coast

        // top Agitator, bottom Agitator
        SparkBaseConfig tA = new SparkMaxConfig().idleMode(IdleMode.kCoast);
        // note: check orientations of these
        SparkBaseConfig bA = new SparkMaxConfig().idleMode(IdleMode.kCoast).inverted(true);

        SparkBaseConfig angleConfig = new SparkMaxConfig().idleMode(IdleMode.kCoast);
        
        launcherEncoder.setPosition(0.0);

        primaryLauncherLeft.configure(pLL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        secondaryLauncherLeft.configure(sLL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        primaryLauncherRight.configure(pLR, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        secondaryLauncherRight.configure(sLR, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        topAgitator.configure(tA, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        bottomAgitator.configure(bA, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        launcherAngleDrive.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.putData( "Launcher Config", sendables);
    }

    public void reverseSpeeds() {
        sendables.targetIntakeSpeed *= -1;
        sendables.targetLaunchSpeed *= -1;
    }

    public void settargetLaunchSpeed(double speed) {
        sendables.targetLaunchSpeed = speed;
    }

    public void settargetIntakeSpeed(double speed) {
        sendables.targetIntakeSpeed = speed;
    }

    public double gettargetLaunchSpeed() {
        return sendables.targetLaunchSpeed;
    }

    public double gettargetIntakeSpeed() {
        return sendables.targetIntakeSpeed;
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
        primaryLauncherRight.set(0.0);
    }

    public void updateIntake() {
        topAgitator.set(sendables.targetIntakeSpeed);
        bottomAgitator.set(sendables.targetIntakeSpeed);
    }

    public void updateLauncher(double scale) {
        // will update right as well because of the followers
        primaryLauncherLeft.set(sendables.targetLaunchSpeed * scale);
        primaryLauncherRight.set(sendables.targetLaunchSpeed * scale);
    }

    public void updateLauncher() {
        // will update right as well because of the followers
        primaryLauncherLeft.set(sendables.targetLaunchSpeed);
        primaryLauncherRight.set(sendables.targetLaunchSpeed);
    }

    public Command toggleLauncherCommand() {
        return runOnce(() -> {
            System.out.println("launcher toggle");
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

    public Command autoRunLauncherCommand() {
        return runOnce(() -> {
            updateLauncher();
            updateIntake();
        });
    }

    public Command autoStopLauncherCommand() {
        return runOnce(() -> {
            stopLauncher();
            stopIntake();
        });
    }

    public Command runLauncherCommand(DoubleSupplier scale) {
        return runOnce(() -> {
            if (!launcherRunning) {
                updateLauncher(scale.getAsDouble());

                currentLauncherRunCommand = run(() -> updateLauncher(scale.getAsDouble()));
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
        return Commands.runOnce(() -> sendables.targetIntakeSpeed = Math.min(sendables.targetIntakeSpeed+0.05, 1.0));
    }

    public Command decreaseIntakeSpeed() {
        return Commands.runOnce(() -> sendables.targetIntakeSpeed = Math.max(sendables.targetIntakeSpeed-0.05, 0.0));
    }

    public Command increaseLaunchSpeed() {
        return Commands.runOnce(() -> sendables.targetLaunchSpeed = Math.min(sendables.targetLaunchSpeed+0.05, 1.0));
    }

    public Command decreaseLaunchSpeed() {
        return Commands.runOnce(() -> sendables.targetLaunchSpeed = Math.max(sendables.targetLaunchSpeed-0.05, 0.0));
    }

    public Command decreaseLauncherAngle() {
        return run(() -> {
            //if (limitSwitch.get()) {
            System.out.printf("Decrease %f\n", launcherEncoder.getPosition());
            if (launcherEncoder.getPosition() < -16) {
                launcherAngleDrive.set(0.0);
            }
            else {
                launcherAngleDrive.set(-0.1);
            }
        });
    }

    public Command increaseLauncherAngle() {
        return run(() -> {
            System.out.printf("Increase %f\n", launcherEncoder.getPosition());
            if (launcherEncoder.getPosition() > -0.1) {
                launcherAngleDrive.set(0.0);
            }
            else {
                launcherAngleDrive.set(0.1);
            }
        });
    }

    public Command autoAlignAngle(Supplier<Pose2d> robotPose, Supplier<Pose2d> hubPose) {
        // this will not reset the angle drive when the command ends!
        return run(() -> {
            solver.updateHubShotSolution(robotPose.get(), hubPose.get());
            if (!solver.isHubShotSolutionFeasible()) return;

            double target = solver.getHubLaunchAngleSetpoint().getDegrees() * 14.5/45; // might be wrong

            if (target + launcherEncoder.getPosition() < 0.2) {
                if (launcherEncoder.getPosition() < 0) launcherAngleDrive.set(0.1);
                else launcherAngleDrive.set(0.0);
            }
            else if (target + launcherEncoder.getPosition() > 0.2) {
                if (launcherEncoder.getPosition() > -16) launcherAngleDrive.set(-0.1);
                else launcherAngleDrive.set(0.0);
            }
            else {
                launcherAngleDrive.set(0.0);
            }
        }).until(() -> Math.abs(solver.getHubLaunchAngleSetpoint().getDegrees() * 14.5/45 + launcherEncoder.getPosition()) < 0.2);

    }

    // todo: test this
    public Command alignAngle(DoubleSupplier target) {
        return run(() -> {

            double targetDegrees = target.getAsDouble() * 14.5/45; // might be wrong

            if (targetDegrees + launcherEncoder.getPosition() < 0.2) {
                if (launcherEncoder.getPosition() < 0) launcherAngleDrive.set(0.1);
                else launcherAngleDrive.set(0.0);
            }
            else if (targetDegrees + launcherEncoder.getPosition() > 0.2) {
                if (launcherEncoder.getPosition() > -16) launcherAngleDrive.set(-0.1);
                else launcherAngleDrive.set(0.0);
            }
            else {
                launcherAngleDrive.set(0.0);
            }
        }).until(() -> Math.abs(target.getAsDouble() * 14.5/45 + launcherEncoder.getPosition()) < 0.2);
    }
}
