// Copyright (c) 2021-2026 Littleton Robotics
// Copyright (c) 2026 Team 4687 Spartobots
//
// SPDX-License-Identifier: BSD-3-Clause

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.NetworkTablesUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double CONTROL_LOOP_PERIOD_SECONDS = 0.02;
  private static final double ANGLE_KP = 3.0;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 0.35;
  private static final double ANGLE_KFF = 0.35;
  private static final double ANGLE_MAX_VELOCITY = 2.6;
  private static final double ANGLE_MAX_ACCELERATION = 7.0;
  private static final double ANGLE_POSITION_TOLERANCE_RAD = Units.degreesToRadians(1.5);
  private static final double ANGLE_VELOCITY_TOLERANCE_RAD_PER_SEC = Units.degreesToRadians(8.0);
  private static final double ANGLE_GOAL_JUMP_REJECT_RAD = Units.degreesToRadians(120.0);
  private static final double ANGLE_GOAL_JUMP_REJECT_ACTIVE_ERROR_RAD = Units.degreesToRadians(8.0);
  private static final double ANGLE_INTEGRATOR_MAX_ABS_OUTPUT = 1.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double FF_MAX_VOLTAGE = 12.0; // Volts
  private static final double FF_MIN_SAMPLE_VELOCITY_RAD_PER_SEC = 1e-4;
  private static final int FF_MIN_SAMPLE_COUNT = 10;
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final NetworkTable tuningTable =
      NetworkTablesUtil.tuningMode(NetworkTablesUtil.actions("Drive").getSubTable("AlignToAngle"));
  private static final NetworkTableEntry angleKpEntry = tuningTable.getEntry("Kp");
  private static final NetworkTableEntry angleKiEntry = tuningTable.getEntry("Ki");
  private static final NetworkTableEntry angleKdEntry = tuningTable.getEntry("Kd");
  private static final NetworkTableEntry angleKffEntry = tuningTable.getEntry("Kff");
  private static final NetworkTableEntry angleMaxVelocityEntry =
      tuningTable.getEntry("MaxVelocityRadPerSec");
  private static final NetworkTableEntry angleMaxAccelerationEntry =
      tuningTable.getEntry("MaxAccelerationRadPerSecSquared");

  static {
    angleKpEntry.setDefaultDouble(ANGLE_KP);
    angleKiEntry.setDefaultDouble(ANGLE_KI);
    angleKdEntry.setDefaultDouble(ANGLE_KD);
    angleKffEntry.setDefaultDouble(ANGLE_KFF);
    angleMaxVelocityEntry.setDefaultDouble(ANGLE_MAX_VELOCITY);
    angleMaxAccelerationEntry.setDefaultDouble(ANGLE_MAX_ACCELERATION);
  }

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /** Drive command using two joysticks (controlling linear and angular velocities). */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier robotOrientedSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to requested reference frame & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean robotOriented = robotOrientedSupplier.getAsBoolean();
          drive.runVelocity(
              robotOriented
                  ? speeds
                  : ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    return joystickDriveAtAngle(
        drive, xSupplier, ySupplier, rotationSupplier, () -> Double.POSITIVE_INFINITY);
  }

  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier,
      DoubleSupplier linearAccelerationLimitMetersPerSecSquaredSupplier) {
    double initialAngleKp = getTunedValue(angleKpEntry, ANGLE_KP, 0.0, 30.0);
    double initialAngleKi = getTunedValue(angleKiEntry, ANGLE_KI, 0.0, 10.0);
    double initialAngleKd = getTunedValue(angleKdEntry, ANGLE_KD, 0.0, 10.0);
    double initialAngleMaxVelocity =
        getTunedValue(angleMaxVelocityEntry, ANGLE_MAX_VELOCITY, 0.1, 100.0);
    double initialAngleMaxAcceleration =
        getTunedValue(angleMaxAccelerationEntry, ANGLE_MAX_ACCELERATION, 0.1, 200.0);
    initialAngleMaxVelocity =
        Math.min(initialAngleMaxVelocity, Math.max(0.1, drive.getMaxAngularSpeedRadPerSec()));

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            initialAngleKp,
            initialAngleKi,
            initialAngleKd,
            new TrapezoidProfile.Constraints(initialAngleMaxVelocity, initialAngleMaxAcceleration));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(
        ANGLE_POSITION_TOLERANCE_RAD, ANGLE_VELOCITY_TOLERANCE_RAD_PER_SEC);
    angleController.setIntegratorRange(
        -ANGLE_INTEGRATOR_MAX_ABS_OUTPUT, ANGLE_INTEGRATOR_MAX_ABS_OUTPUT);
    final double[] lastGoalRadians = {Double.NaN};
    final double[] lastErrorRadians = {Double.POSITIVE_INFINITY};
    final Translation2d[] lastLinearVelocityMetersPerSec = {Translation2d.kZero};

    // Construct command
    return Commands.run(
            () -> {
              double tunedAngleKp = getTunedValue(angleKpEntry, ANGLE_KP, 0.0, 30.0);
              double tunedAngleKi = getTunedValue(angleKiEntry, ANGLE_KI, 0.0, 10.0);
              double tunedAngleKd = getTunedValue(angleKdEntry, ANGLE_KD, 0.0, 10.0);
              double tunedAngleKff = getTunedValue(angleKffEntry, ANGLE_KFF, 0.0, 5.0);
              double tunedAngleMaxVelocity =
                  getTunedValue(angleMaxVelocityEntry, ANGLE_MAX_VELOCITY, 0.1, 100.0);
              double tunedAngleMaxAcceleration =
                  getTunedValue(angleMaxAccelerationEntry, ANGLE_MAX_ACCELERATION, 0.1, 200.0);
              tunedAngleMaxVelocity =
                  Math.min(
                      tunedAngleMaxVelocity, Math.max(0.1, drive.getMaxAngularSpeedRadPerSec()));
              angleController.setPID(tunedAngleKp, tunedAngleKi, tunedAngleKd);
              angleController.setConstraints(
                  new TrapezoidProfile.Constraints(
                      tunedAngleMaxVelocity, tunedAngleMaxAcceleration));

              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
              Translation2d requestedLinearVelocityMetersPerSec =
                  linearVelocity.times(drive.getMaxLinearSpeedMetersPerSec());
              Translation2d limitedLinearVelocityMetersPerSec =
                  applyLinearAccelerationLimit(
                      lastLinearVelocityMetersPerSec[0],
                      requestedLinearVelocityMetersPerSec,
                      linearAccelerationLimitMetersPerSecSquaredSupplier.getAsDouble(),
                      CONTROL_LOOP_PERIOD_SECONDS);
              lastLinearVelocityMetersPerSec[0] = limitedLinearVelocityMetersPerSec;

              double currentHeadingRadians = drive.getRotation().getRadians();
              double targetHeadingRadians = rotationSupplier.get().getRadians();
              if (!Double.isFinite(targetHeadingRadians)) {
                targetHeadingRadians =
                    Double.isFinite(lastGoalRadians[0])
                        ? lastGoalRadians[0]
                        : currentHeadingRadians;
              }
              if (Double.isFinite(lastGoalRadians[0])) {
                double goalJumpRadians =
                    Math.abs(MathUtil.angleModulus(targetHeadingRadians - lastGoalRadians[0]));
                if (goalJumpRadians > ANGLE_GOAL_JUMP_REJECT_RAD
                    && lastErrorRadians[0] < ANGLE_GOAL_JUMP_REJECT_ACTIVE_ERROR_RAD) {
                  targetHeadingRadians = lastGoalRadians[0];
                }
              }
              lastGoalRadians[0] = targetHeadingRadians;

              // Calculate angular speed
              double omega = angleController.calculate(currentHeadingRadians, targetHeadingRadians);
              double omegaFeedforward = angleController.getSetpoint().velocity * tunedAngleKff;
              omega += omegaFeedforward;
              omega =
                  MathUtil.clamp(
                      omega,
                      -drive.getMaxAngularSpeedRadPerSec(),
                      drive.getMaxAngularSpeedRadPerSec());
              lastErrorRadians[0] =
                  Math.abs(MathUtil.angleModulus(targetHeadingRadians - currentHeadingRadians));
              if (angleController.atGoal()) {
                omega = 0.0;
              }

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      limitedLinearVelocityMetersPerSec.getX(),
                      limitedLinearVelocityMetersPerSec.getY(),
                      omega);

              boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(
            () -> {
              double currentHeadingRadians = drive.getRotation().getRadians();
              angleController.reset(currentHeadingRadians);
              lastGoalRadians[0] = currentHeadingRadians;
              lastErrorRadians[0] = Double.POSITIVE_INFINITY;
              lastLinearVelocityMetersPerSec[0] = drive.getFieldRelativeVelocityMetersPerSecond();
            });
  }

  static Translation2d applyLinearAccelerationLimit(
      Translation2d currentVelocityMetersPerSec,
      Translation2d targetVelocityMetersPerSec,
      double maxAccelerationMetersPerSecSquared,
      double dtSeconds) {
    if (!Double.isFinite(maxAccelerationMetersPerSecSquared)
        || maxAccelerationMetersPerSecSquared <= 0.0
        || !Double.isFinite(dtSeconds)
        || dtSeconds <= 0.0) {
      return targetVelocityMetersPerSec;
    }

    Translation2d sanitizedCurrentVelocityMetersPerSec =
        new Translation2d(
            Double.isFinite(currentVelocityMetersPerSec.getX())
                ? currentVelocityMetersPerSec.getX()
                : 0.0,
            Double.isFinite(currentVelocityMetersPerSec.getY())
                ? currentVelocityMetersPerSec.getY()
                : 0.0);
    Translation2d deltaVelocityMetersPerSec =
        targetVelocityMetersPerSec.minus(sanitizedCurrentVelocityMetersPerSec);
    double maxVelocityStepMetersPerSec = maxAccelerationMetersPerSecSquared * dtSeconds;
    double deltaNormMetersPerSec = deltaVelocityMetersPerSec.getNorm();
    if (deltaNormMetersPerSec <= maxVelocityStepMetersPerSec) {
      return targetVelocityMetersPerSec;
    }

    return sanitizedCurrentVelocityMetersPerSec.plus(
        deltaVelocityMetersPerSec.times(maxVelocityStepMetersPerSec / deltaNormMetersPerSec));
  }

  private static double getTunedValue(
      NetworkTableEntry entry, double fallbackValue, double minValue, double maxValue) {
    double value = MathUtil.clamp(entry.getDouble(fallbackValue), minValue, maxValue);
    entry.setDouble(value);
    return value;
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = Math.min(timer.get() * FF_RAMP_RATE, FF_MAX_VOLTAGE);
                  drive.runCharacterization(voltage);
                  double velocity = Math.abs(drive.getFFCharacterizationVelocity());
                  if (Double.isFinite(velocity)
                      && velocity > FF_MIN_SAMPLE_VELOCITY_RAD_PER_SEC
                      && Double.isFinite(voltage)) {
                    velocitySamples.add(velocity);
                    voltageSamples.add(voltage);
                  }
                },
                drive)
            .until(() -> timer.hasElapsed(FF_MAX_VOLTAGE / FF_RAMP_RATE))

            // After sweep, calculate and print results
            .finallyDo(
                () -> {
                  drive.runCharacterization(0.0);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  int n = velocitySamples.size();
                  if (n < FF_MIN_SAMPLE_COUNT) {
                    System.out.println(
                        "\tInsufficient valid samples ("
                            + n
                            + "). Increase run time or verify module/encoder wiring.");
                    return;
                  }

                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }

                  double denominator = n * sumX2 - sumX * sumX;
                  if (Math.abs(denominator) < 1e-9) {
                    System.out.println(
                        "\tFit failed (degenerate data). Verify robot motion and rerun.");
                    return;
                  }

                  double kS = (sumY * sumX2 - sumX * sumXY) / denominator;
                  double kV = (n * sumXY - sumX * sumY) / denominator;
                  if (!Double.isFinite(kS) || !Double.isFinite(kV)) {
                    System.out.println("\tFit failed (non-finite result). Verify data and rerun.");
                    return;
                  }

                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                  System.out.println("\tSamples: " + n);
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  /*public static Command alignTo(Drive drive, String element) {
    Pose2d target;
  }*/

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
