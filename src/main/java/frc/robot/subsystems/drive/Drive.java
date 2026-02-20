// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Consumer<Pose2d> setSimulationPoseCallback;
  private final LoggedNetworkBoolean logHubAimVector =
      new LoggedNetworkBoolean("/SmartDashboard/Drive/LogHubAimVector", false);
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private boolean wasHubAimVectorLoggingEnabled = false;
  public int octant;

  private PathPlannerAuto outpost;
  private PathPlannerAuto defaultAuto;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO, (pose) -> {});
  }

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> setSimulationPoseCallback) {
    this.gyroIO = gyroIO;
    this.setSimulationPoseCallback = setSimulationPoseCallback;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 1.0), new PIDConstants(5.0, 0.0, 1.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // register commmands to be used in PathPlanner autos
    NamedCommands.registerCommand("trench", Commands.runOnce(() -> autoDriveUnderTrench(), this));
    NamedCommands.registerCommand("outpost", Commands.runOnce(() -> driveToOutpost(), this));

    // define autos here
    outpost = new PathPlannerAuto("Outpost");
    defaultAuto = new PathPlannerAuto("Default");

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    logHubAimVector.periodic();

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(
          sampleTimestamps[i], rawGyroRotation.unaryMinus(), modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    if (!logHubAimVector.get() && wasHubAimVectorLoggingEnabled) {
      clearHubAimLogs();
    }

    determineOctant();

    Logger.recordOutput("octant", octant);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds correctedSpeeds =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond * chassisXCommandScalar,
            speeds.vyMetersPerSecond * chassisYCommandScalar,
            speeds.omegaRadiansPerSecond * chassisOmegaCommandScalar);

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(correctedSpeeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveChassisSpeeds/Requested", speeds);
    Logger.recordOutput("SwerveChassisSpeeds/Corrected", correctedSpeeds);
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);

    Logger.recordOutput("Pose", this.getPose());
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    // Command an X lock without changing kinematics heading offsets
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(new SwerveModuleState(0.0, headings[i]));
    }
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation.unaryMinus(), getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }

  public Command getAutonomousCommand() {
    return outpost;
  }

  public void autoDriveUnderTrench() {
    Translation2d[] poses = determineTrenchPoses();
    double robotAngleRadians = getRotation().getRadians();
    double angleRadians =
        (DriveConstants.trenchSnapTo)
            * Math.round(robotAngleRadians / (DriveConstants.trenchSnapTo));
    CommandScheduler.getInstance().schedule(autoDriveUnderTrenchCommand(poses, angleRadians));
  }

  public Command autoDriveUnderTrenchCommand(Translation2d[] poses, double angle) {
    return AutoBuilder.pathfindToPose(
            new Pose2d(poses[0], new Rotation2d(angle)), pathConstraints, 0)
        .andThen(
            AutoBuilder.pathfindToPose(
                new Pose2d(poses[1], new Rotation2d(angle)), pathConstraints, 0));
  }

  public Translation2d[] determineTrenchPoses() {
    Translation2d[] poses =
        switch (octant) {
          case 0 -> new Translation2d[] {
            Constants.blueTrenchTopInner, Constants.blueTrenchTopOuter
          };
          case 1 -> new Translation2d[] {
            Constants.blueTrenchTopOuter, Constants.blueTrenchTopInner
          };
          case 2 -> new Translation2d[] {Constants.redTrenchTopOuter, Constants.redTrenchTopInner};
          case 3 -> new Translation2d[] {Constants.redTrenchTopInner, Constants.redTrenchTopOuter};
          case 4 -> new Translation2d[] {
            Constants.blueTrenchBottomInner, Constants.blueTrenchBottomOuter
          };
          case 5 -> new Translation2d[] {
            Constants.blueTrenchBottomOuter, Constants.blueTrenchBottomInner
          };
          case 6 -> new Translation2d[] {
            Constants.redTrenchBottomOuter, Constants.redTrenchBottomInner
          };
          case 7 -> new Translation2d[] {
            Constants.redTrenchBottomInner, Constants.redTrenchBottomOuter
          };
          default -> new Translation2d[] {
            Constants.blueTrenchBottomInner, Constants.blueTrenchBottomOuter
          };
        };
    return poses;
  }

  public void driveToOutpost() {
    Pose2d[] poses;
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      poses = new Pose2d[] {Constants.redOutpostBefore, Constants.redOutpost};
    } else {
      poses = new Pose2d[] {Constants.blueOutpostBefore, Constants.blueOutpost};
    }
    CommandScheduler.getInstance().schedule(driveToOutpostCommand(poses));
  }

  public Command driveToOutpostCommand(Pose2d[] poses) {
    return AutoBuilder.pathfindToPose(poses[0], pathConstraints, 0)
        .andThen(AutoBuilder.pathfindToPose(poses[1], pathConstraints, 0));
  }

  public void determineOctant() {
    double x = this.getPose().getX();
    double y = this.getPose().getY();

    if (y > Constants.midLineY) {
      if (x < Constants.blueLine) {
        octant = 0;
      } else if (x < Constants.midLineX) {
        octant = 1;
      } else if (x < Constants.redLine) {
        octant = 2;
      } else {
        octant = 3;
      }
    } else {
      if (x < Constants.blueLine) {
        octant = 4;
      } else if (x < Constants.midLineX) {
        octant = 5;
      } else if (x < Constants.redLine) {
        octant = 6;
      } else {
        octant = 7;
      }
    }
  }

  public Command alignToPose(Pose2d target) {
    return AutoBuilder.pathfindToPose(target, pathConstraints);
  }

  private Pose2d getNearestHub() {
    double distanceToRedHub =
        getPose().getTranslation().getDistance(Constants.redHub.getTranslation());
    double distanceToBlueHub =
        getPose().getTranslation().getDistance(Constants.blueHub.getTranslation());
    return distanceToRedHub < distanceToBlueHub ? Constants.redHub : Constants.blueHub;
  }

  public Pose2d getNearestHubPose() {
    return getNearestHub();
  }

  private Translation2d getFieldRelativeVelocity() {
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
    return new Translation2d(
        fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
  }

  private Pose2d getCompensatedHub(Pose2d targetHub, double shotAirtimeSeconds) {
    double clampedAirtimeSeconds = Math.max(0.0, shotAirtimeSeconds);

    // Aim opposite the current robot velocity so shot travel time is compensated in flight.
    Translation2d compensationOffset = getFieldRelativeVelocity().times(-clampedAirtimeSeconds);
    return new Pose2d(targetHub.getTranslation().plus(compensationOffset), targetHub.getRotation());
  }

  private Rotation2d getRotationToHub(Pose2d hub) {
    Translation2d toTarget = hub.getTranslation().minus(getPose().getTranslation());
    return new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
  }

  public Command alignToHub() {
    return alignToHub(() -> 0.0, () -> 0.0, () -> 0.0);
  }

  public Command alignToHub(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier shotAirtimeSecondsSupplier) {
    return DriveCommands.joystickDriveAtAngle(
        this,
        x,
        y,
        () -> {
          double airtimeSeconds = shotAirtimeSecondsSupplier.getAsDouble();
          Pose2d baseHub = getNearestHub();
          Pose2d compensatedHub = getCompensatedHub(baseHub, airtimeSeconds);
          logHubAimTarget(baseHub, compensatedHub, airtimeSeconds);
          return getRotationToHub(compensatedHub);
        });
  }

  private void logHubAimTarget(Pose2d baseHub, Pose2d compensatedHub, double airtimeSeconds) {
    if (!logHubAimVector.get()) {
      return;
    }

    Pose2d robotPose = getPose();
    Translation2d compensationOffset =
        compensatedHub.getTranslation().minus(baseHub.getTranslation());
    Translation2d vectorToTarget =
        compensatedHub.getTranslation().minus(robotPose.getTranslation());

    Logger.recordOutput("Drive/HubAim/RobotPose", robotPose);
    Logger.recordOutput("Drive/HubAim/BaseTargetPose", baseHub);
    Logger.recordOutput("Drive/HubAim/CompensatedTargetPose", compensatedHub);
    Logger.recordOutput(
        "Drive/HubAim/TargetVectorEndpoints",
        new Pose2d[] {
          new Pose2d(robotPose.getTranslation(), Rotation2d.kZero),
          new Pose2d(compensatedHub.getTranslation(), Rotation2d.kZero)
        });
    Logger.recordOutput("Drive/HubAim/AirtimeSeconds", airtimeSeconds);
    Logger.recordOutput("Drive/HubAim/CompensationOffsetX", compensationOffset.getX());
    Logger.recordOutput("Drive/HubAim/CompensationOffsetY", compensationOffset.getY());
    Logger.recordOutput("Drive/HubAim/VectorToTargetX", vectorToTarget.getX());
    Logger.recordOutput("Drive/HubAim/VectorToTargetY", vectorToTarget.getY());
    wasHubAimVectorLoggingEnabled = true;
  }

  private void clearHubAimLogs() {
    Logger.recordOutput("Drive/HubAim/RobotPose", Pose2d.kZero);
    Logger.recordOutput("Drive/HubAim/BaseTargetPose", Pose2d.kZero);
    Logger.recordOutput("Drive/HubAim/CompensatedTargetPose", Pose2d.kZero);
    Logger.recordOutput("Drive/HubAim/TargetVectorEndpoints", new Pose2d[] {});
    Logger.recordOutput("Drive/HubAim/AirtimeSeconds", 0.0);
    Logger.recordOutput("Drive/HubAim/CompensationOffsetX", 0.0);
    Logger.recordOutput("Drive/HubAim/CompensationOffsetY", 0.0);
    Logger.recordOutput("Drive/HubAim/VectorToTargetX", 0.0);
    Logger.recordOutput("Drive/HubAim/VectorToTargetY", 0.0);
    wasHubAimVectorLoggingEnabled = false;
  }

  public Command alignToOutpost(DoubleSupplier x, DoubleSupplier y) {
    Pose2d target = Constants.blueHub;
    DoubleSupplier angleSetpoint =
        () ->
            Math.atan2(
                this.getPose().getY() - target.getY(), this.getPose().getX() - target.getX());
    return Commands.run(
            () -> {
              DriveCommands.joystickDrive(
                  this,
                  x,
                  y,
                  () ->
                      this.getPose().getRotation().getRadians()
                          - Math.atan2(
                              this.getPose().getY() - target.getY(),
                              this.getPose().getX() - target.getX()),
                  () -> false);
            })
        .until(
            () ->
                Math.abs(this.getPose().getRotation().getRadians() - angleSetpoint.getAsDouble())
                    < DriveConstants.aligned);
  }

  public void alignTo(Pose2d target) {}

  public void update() {}
}
