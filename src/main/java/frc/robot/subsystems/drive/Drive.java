// Copyright (c) 2021-2026 Littleton Robotics
// Copyright (c) 2026 Team 4687 Spartobots
//
// SPDX-License-Identifier: BSD-3-Clause

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.gamepiece.shooter.ShooterConstants;
import frc.robot.targeting.HubTargetingGeometry;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.NetworkTablesUtil;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // PathPlanner PID values
  // Conservative starting values for PathPlanner holonomic control. Final values still need
  // carpet tuning, but these avoid the severe saturation caused by the previous defaults.
  // private static final double defaultPathTranslationKp = 6.0;
  // private static final double defaultPathTranslationKi = 0.0;
  // private static final double defaultPathTranslationKd = 0.3;

  private static final double defaultPathTranslationKp = 4.0;
  private static final double defaultPathTranslationKi = 0.0;
  private static final double defaultPathTranslationKd = 0.0;

  private static final double defaultPathRotationKp = 4.0;
  private static final double defaultPathRotationKi = 0.0;
  private static final double defaultPathRotationKd = 0.0;
  // Large enough to ignore normal motion, small enough to catch gyro resets or bad re-seeds.
  private static final double gyroResyncThresholdRadians = Math.toRadians(45.0);

  /*private static final double defaultPathRotationKp = 4.0;
  private static final double defaultPathRotationKi = 0.0;
  private static final double defaultPathRotationKd = 0.0;*/

  private static final double minAimVectorMagnitudeMeters = 0.10;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Consumer<Pose2d> setSimulationPoseCallback;
  private final TunableHolonomicPathController pathController =
      new TunableHolonomicPathController(
          new PIDConstants(
              defaultPathTranslationKp, defaultPathTranslationKi, defaultPathTranslationKd),
          new PIDConstants(defaultPathRotationKp, defaultPathRotationKi, defaultPathRotationKd),
          0.02);
  private final NetworkTable driveSubsystemTable = NetworkTablesUtil.domain("Drive");
  private final NetworkTable driveCommonTuningTable =
      NetworkTablesUtil.tuningCommon(driveSubsystemTable);
  private final NetworkTable driveTuningTable = NetworkTablesUtil.tuningMode(driveSubsystemTable);
  private final NetworkTableEntry logHubAimVectorEntry =
      driveCommonTuningTable.getEntry("LogHubAimVector");
  private final NetworkTableEntry moduleDriveKpEntry =
      driveTuningTable.getEntry("Module/DrivePID/Kp");
  private final NetworkTableEntry moduleDriveKiEntry =
      driveTuningTable.getEntry("Module/DrivePID/Ki");
  private final NetworkTableEntry moduleDriveKdEntry =
      driveTuningTable.getEntry("Module/DrivePID/Kd");
  private final NetworkTableEntry moduleTurnKpEntry =
      driveTuningTable.getEntry("Module/TurnPID/Kp");
  private final NetworkTableEntry moduleTurnKiEntry =
      driveTuningTable.getEntry("Module/TurnPID/Ki");
  private final NetworkTableEntry moduleTurnKdEntry =
      driveTuningTable.getEntry("Module/TurnPID/Kd");
  private final NetworkTableEntry pathTranslationKpEntry =
      driveTuningTable.getEntry("PathPlanner/TranslationPID/Kp");
  private final NetworkTableEntry pathTranslationKiEntry =
      driveTuningTable.getEntry("PathPlanner/TranslationPID/Ki");
  private final NetworkTableEntry pathTranslationKdEntry =
      driveTuningTable.getEntry("PathPlanner/TranslationPID/Kd");
  private final NetworkTableEntry pathRotationKpEntry =
      driveTuningTable.getEntry("PathPlanner/RotationPID/Kp");
  private final NetworkTableEntry pathRotationKiEntry =
      driveTuningTable.getEntry("PathPlanner/RotationPID/Ki");
  private final NetworkTableEntry pathRotationKdEntry =
      driveTuningTable.getEntry("PathPlanner/RotationPID/Kd");
  private final NetworkTable hubMotionCompTuningTable =
      NetworkTablesUtil.tuningMode("Targeting/Hub").getSubTable("MotionCompensation");
  private final NetworkTableEntry hubMotionCompVelocityScaleEntry =
      hubMotionCompTuningTable.getEntry("VelocityScale");
  private final NetworkTableEntry hubMotionCompLeadSecondsEntry =
      hubMotionCompTuningTable.getEntry("LeadSeconds");
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private boolean wasHubAimVectorLoggingEnabled = false;
  private double moduleDriveKp = Constants.currentMode == Mode.SIM ? driveSimP : driveKp;
  private double moduleDriveKi = Constants.currentMode == Mode.SIM ? 0.0 : driveKi;
  private double moduleDriveKd = Constants.currentMode == Mode.SIM ? driveSimD : driveKd;
  private double moduleTurnKp = Constants.currentMode == Mode.SIM ? turnSimP : turnKp;
  private double moduleTurnKi = Constants.currentMode == Mode.SIM ? 0.0 : turnKi;
  private double moduleTurnKd = Constants.currentMode == Mode.SIM ? turnSimD : turnKd;
  private double pathTranslationKp = defaultPathTranslationKp;
  private double pathTranslationKi = defaultPathTranslationKi;
  private double pathTranslationKd = defaultPathTranslationKd;
  private double pathRotationKp = defaultPathRotationKp;
  private double pathRotationKi = defaultPathRotationKi;
  private double pathRotationKd = defaultPathRotationKd;
  private double hubMotionCompVelocityScale = ShooterConstants.hubMotionCompensationVelocityScale;
  private double hubMotionCompLeadSeconds = ShooterConstants.hubMotionCompensationLeadSeconds;
  public int octant;

  // private Pose2d startingPose = new Pose2d(3.5, 3.9, Rotation2d.fromDegrees(0));
  private Pose2d startingPose = new Pose2d(3.584, 4, Rotation2d.fromDegrees(180));

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = startingPose.getRotation();
  private boolean gyroWasConnected = true;
  private boolean gyroHasBeenInitialized = false;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, startingPose);

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
    this.gyroIO.setAngle(startingPose.getRotation());

    this.setSimulationPoseCallback = setSimulationPoseCallback;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    logHubAimVectorEntry.setDefaultBoolean(false);
    configureDrivePidDefaults();
    loadDrivePidTuning();
    configureHubTargetingDefaults();

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;
    FlippingUtil.fieldSizeX = Constants.fieldLength;
    FlippingUtil.fieldSizeY = Constants.fieldWidth;
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        pathController,
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              NetworkTablesUtil.logPath("Drive/Odometry/Trajectory"),
              activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput(
              NetworkTablesUtil.logPath("Drive/Odometry/TrajectorySetpoint"), targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput(
                        NetworkTablesUtil.logPath("Drive/SysId/State"), state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    loadDrivePidTuning();
    loadHubTargetingTuning();

    odometryLock.lock(); // Prevents odometry updates while reading data
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs(NetworkTablesUtil.logPath("Drive/Inputs/Gyro"), gyroInputs);
      for (var module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput(
          NetworkTablesUtil.logPath("Drive/SwerveStates/Setpoints"), new SwerveModuleState[] {});
      Logger.recordOutput(
          NetworkTablesUtil.logPath("Drive/SwerveStates/SetpointsOptimized"),
          new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    boolean resyncGyro = gyroInputs.connected && !gyroWasConnected;
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
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);

      // Update gyro angle
      if (gyroInputs.connected && !resyncGyro) {
        Rotation2d measuredGyroRotation = gyroInputs.odometryYawPositions[i];
        if (!gyroHasBeenInitialized) {
          // Trust the first connected sample so startup and explicit pose resets can establish a
          // baseline before jump detection kicks in.
          rawGyroRotation = measuredGyroRotation;
        } else {
          double gyroJumpRadians =
              Math.abs(
                  MathUtil.angleModulus(measuredGyroRotation.minus(rawGyroRotation).getRadians()));
          if (gyroJumpRadians > gyroResyncThresholdRadians) {
            // The gyro likely reset or re-zeroed itself. Keep the current field frame and
            // re-seed the gyro once this batch is done so teleop and auto do not flip.
            resyncGyro = true;
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
          } else {
            // Use the real gyro angle.
            rawGyroRotation = measuredGyroRotation;
          }
        }
      } else {
        // Use the angle delta from the kinematics and module deltas.
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      Logger.recordOutput("gyro test", rawGyroRotation);

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    if (resyncGyro) {
      gyroIO.setAngle(rawGyroRotation);
    }
    if (gyroInputs.connected && sampleCount > 0) {
      gyroHasBeenInitialized = true;
    }
    gyroWasConnected = gyroInputs.connected;

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    if (!isHubAimVectorLoggingEnabled() && wasHubAimVectorLoggingEnabled) {
      clearHubAimLogs();
    }

    determineOctant();

    Logger.recordOutput(NetworkTablesUtil.logPath("Drive/State/Octant"), octant);
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
    Logger.recordOutput(NetworkTablesUtil.logPath("Drive/ChassisSpeeds/Requested"), speeds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Drive/ChassisSpeeds/Corrected"), correctedSpeeds);
    Logger.recordOutput(NetworkTablesUtil.logPath("Drive/SwerveStates/Setpoints"), setpointStates);
    Logger.recordOutput(NetworkTablesUtil.logPath("Drive/ChassisSpeeds/Setpoints"), discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Drive/SwerveStates/SetpointsOptimized"), setpointStates);

    Logger.recordOutput(NetworkTablesUtil.logPath("Drive/State/Pose"), this.getPose());
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
    for (var module : modules) {
      module.stop();
    }
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
      modules[i].runSetpoint(new SwerveModuleState(0.0, headings[i]), false);
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
  @AutoLogOutput(key = NetworkTablesUtil.rootTableName + "/Drive/SwerveStates/Measured")
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
  @AutoLogOutput(key = NetworkTablesUtil.rootTableName + "/Drive/ChassisSpeeds/Measured")
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
  @AutoLogOutput(key = NetworkTablesUtil.rootTableName + "/Drive/State/Pose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the current measured angular velocity. */
  public double getYawVelocityRadPerSec() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    rawGyroRotation = pose.getRotation();
    gyroIO.setAngle(rawGyroRotation);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    gyroHasBeenInitialized = true;
    setSimulationPoseCallback.accept(pose);
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

  public Command autoDriveUnderTrenchCommand(double goalEndVelocity) {
    return autoDriveUnderTrenchCommand(goalEndVelocity, pathConstraints);
  }

  public Command autoDriveUnderTrenchCommand(
      double goalEndVelocity, PathConstraints trenchPathConstraints) {
    return Commands.defer(
        () -> {
          Translation2d[] trenchPoses = determineTrenchPoses();
          double trenchHeadingRadians = selectTrenchHeadingRadians(getPose(), trenchPoses[1]);
          return buildTrenchPathCommand(
              trenchPoses[1], trenchHeadingRadians, goalEndVelocity, trenchPathConstraints);
        },
        Set.of(this));
  }

  private Command buildTrenchPathCommand(
      Translation2d targetPose,
      double angleRadians,
      double goalEndVelocity,
      PathConstraints trenchPathConstraints) {
    Rotation2d trenchHeading = new Rotation2d(angleRadians);
    Command trenchPathCommand =
        Commands.either(
            AutoBuilder.pathfindToPose(
                new Pose2d(targetPose, trenchHeading), trenchPathConstraints, goalEndVelocity),
            alignToHeadingCommand(trenchHeading)
                .andThen(
                    AutoBuilder.pathfindToPose(
                        new Pose2d(targetPose, trenchHeading),
                        trenchPathConstraints,
                        goalEndVelocity)),
            () -> isHeadingAligned(trenchHeading));
    return Math.abs(goalEndVelocity) <= 1e-9
        ? trenchPathCommand.finallyDo(this::stop)
        : trenchPathCommand;
  }

  static double selectTrenchHeadingRadians(Pose2d currentPose, Translation2d targetPose) {
    // Face the direction of travel so the path controller does not start the trench move pointed
    // away from the target and arc into the field edge.
    return targetPose.getX() >= currentPose.getX() ? 0.0 : Math.PI;
  }

  private Command alignToHeadingCommand(Rotation2d targetHeading) {
    return DriveCommands.joystickDriveAtAngle(this, () -> 0.0, () -> 0.0, () -> targetHeading)
        .until(() -> isHeadingAligned(targetHeading))
        .andThen(Commands.runOnce(this::stop));
  }

  private boolean isHeadingAligned(Rotation2d targetHeading) {
    double headingErrorRadians =
        Math.abs(MathUtil.angleModulus(getRotation().minus(targetHeading).getRadians()));
    return headingErrorRadians <= trenchLongAxisAlignmentToleranceRadians;
  }

  private Translation2d[] determineTrenchPoses() {
    return switch (octant) {
      case 0 -> new Translation2d[] {Constants.blueTrenchTopInner, Constants.blueTrenchTopOuter};
      case 1 -> new Translation2d[] {Constants.blueTrenchTopOuter, Constants.blueTrenchTopInner};
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
  }

  public Command driveToOutpostCommand() {
    return Commands.defer(
        () -> {
          Pose2d[] poses = selectOutpostApproachPoses(DriverStation.getAlliance());
          return AutoBuilder.pathfindToPose(poses[0], pathConstraints, 0)
              .andThen(AutoBuilder.pathfindToPose(poses[1], pathConstraints, 0));
        },
        Set.of(this));
  }

  private void determineOctant() {
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

  public Command pathfindToPose(Pose2d target, PathConstraints constraints) {
    return pathfindToPose(target, constraints, 0.0);
  }

  public Command pathfindToPose(
      Pose2d target, PathConstraints constraints, double goalEndVelocity) {
    return Commands.defer(
        () -> AutoBuilder.pathfindToPose(target, constraints, goalEndVelocity), Set.of(this));
  }

  public Command pathfindToPoseWithHeading(Pose2d target, PathConstraints constraints) {
    return pathfindToPoseWithHeading(target, constraints, 0.0);
  }

  public Command pathfindToPoseWithHeading(
      Pose2d target, PathConstraints constraints, double goalEndVelocity) {
    return pathfindToPoseWithRotationOverride(
        target, constraints, goalEndVelocity, () -> Optional.of(target.getRotation()));
  }

  public Command pathfindToPoseWithHubAim(
      Pose2d target, DoubleSupplier shotAirtimeSecondsSupplier, PathConstraints constraints) {
    return pathfindToPoseWithRotationOverride(
        target,
        constraints,
        () -> Optional.of(getHubAimRotation(shotAirtimeSecondsSupplier.getAsDouble())));
  }

  public Command pathfindToTranslation(Translation2d target) {
    return pathfindToTranslation(target, pathConstraints);
  }

  public Command pathfindToTranslation(Translation2d target, PathConstraints constraints) {
    return pathfindToTranslation(target, constraints, 0.0);
  }

  public Command pathfindToTranslation(
      Translation2d target, PathConstraints constraints, double goalEndVelocity) {
    return Commands.defer(
        () ->
            AutoBuilder.pathfindToPose(
                new Pose2d(target, getRotation()), constraints, goalEndVelocity),
        Set.of(this));
  }

  public Command pathfindToTranslationWithHubAim(
      Translation2d target,
      DoubleSupplier shotAirtimeSecondsSupplier,
      PathConstraints constraints) {
    return pathfindToTranslationWithHubAim(target, shotAirtimeSecondsSupplier, constraints, 0.0);
  }

  public Command pathfindToTranslationWithHubAim(
      Translation2d target,
      DoubleSupplier shotAirtimeSecondsSupplier,
      PathConstraints constraints,
      double goalEndVelocity) {
    return pathfindToTranslationWithRotationOverride(
        target,
        constraints,
        goalEndVelocity,
        () -> Optional.of(getHubAimRotation(shotAirtimeSecondsSupplier.getAsDouble())));
  }

  public Command followNamedPath(String pathName) {
    return Commands.defer(() -> buildNamedPathCommand(pathName, false), Set.of(this));
  }

  public Command followNamedPath(String pathName, PathConstraints constraints) {
    return Commands.defer(() -> buildNamedPathCommand(pathName, false, constraints), Set.of(this));
  }

  public Command followNamedPathWithHeading(
      String pathName, Rotation2d heading, PathConstraints constraints) {
    return followNamedPathWithRotationOverride(pathName, constraints, () -> Optional.of(heading));
  }

  public Command pathfindThenFollowNamedPath(String pathName) {
    return Commands.defer(() -> buildNamedPathCommand(pathName, true), Set.of(this));
  }

  public Command pathfindToTranslationAndAlignToHub(
      Translation2d target, DoubleSupplier shotAirtimeSecondsSupplier) {
    return pathfindToTranslationAndAlignToHub(target, shotAirtimeSecondsSupplier, pathConstraints);
  }

  public Command pathfindToTranslationAndAlignToHub(
      Translation2d target,
      DoubleSupplier shotAirtimeSecondsSupplier,
      PathConstraints constraints) {
    return pathfindToTranslationAndAlignToHub(target, shotAirtimeSecondsSupplier, constraints, 0.0);
  }

  public Command pathfindToTranslationAndAlignToHub(
      Translation2d target,
      DoubleSupplier shotAirtimeSecondsSupplier,
      PathConstraints constraints,
      double goalEndVelocity) {
    return pathfindToTranslationWithRotationOverride(
            target,
            constraints,
            goalEndVelocity,
            () -> Optional.of(getHubAimRotation(shotAirtimeSecondsSupplier.getAsDouble())))
        .andThen(alignToHub(() -> 0.0, () -> 0.0, shotAirtimeSecondsSupplier));
  }

  public boolean isNearTranslation(Translation2d target, double toleranceMeters) {
    return getPose().getTranslation().getDistance(target) <= toleranceMeters;
  }

  public Command outpostLoadAuto() {
    return AutoBuilder.pathfindToPose(getNearestOutpost(), pathConstraints);
  }

  private Pose2d getNearestHub() {
    double distanceToRedHub =
        getPose().getTranslation().getDistance(Constants.redHub.getTranslation());
    double distanceToBlueHub =
        getPose().getTranslation().getDistance(Constants.blueHub.getTranslation());
    return distanceToRedHub < distanceToBlueHub ? Constants.redHub : Constants.blueHub;
  }

  private Pose2d getNearestOutpost() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return Constants.redOutpost;
      }
      return Constants.blueOutpost;
    }
    return Constants.blueOutpost;
  }

  static Pose2d selectAllianceHubPose(Optional<Alliance> alliance) {
    return alliance.orElse(Alliance.Blue) == Alliance.Red ? Constants.redHub : Constants.blueHub;
  }

  static Pose2d[] selectOutpostApproachPoses(Optional<Alliance> alliance) {
    if (alliance.orElse(Alliance.Blue) == Alliance.Red) {
      return new Pose2d[] {Constants.redOutpostBefore, Constants.redOutpost};
    }
    return new Pose2d[] {Constants.blueOutpostBefore, Constants.blueOutpost};
  }

  public Pose2d getAllianceHubPose() {
    return selectAllianceHubPose(DriverStation.getAlliance());
  }

  public Pose2d getNearestHubPose() {
    return getNearestHub();
  }

  public Translation2d getFieldRelativeVelocityMetersPerSecond() {
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
    return new Translation2d(
        fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
  }

  private Pose2d getCompensatedHub(Pose2d targetHub, double shotAirtimeSeconds) {
    double clampedAirtimeSeconds = Math.max(0.0, shotAirtimeSeconds + hubMotionCompLeadSeconds);

    // Aim opposite the current robot velocity so shot travel time is compensated in flight.
    Translation2d compensationOffset =
        getFieldRelativeVelocityMetersPerSecond()
            .times(-clampedAirtimeSeconds * hubMotionCompVelocityScale);
    return new Pose2d(targetHub.getTranslation().plus(compensationOffset), targetHub.getRotation());
  }

  private Rotation2d getRotationToHub(Pose2d hub) {
    Translation2d toTarget = HubTargetingGeometry.getVectorFromLaunchOriginToHub(getPose(), hub);
    if (toTarget.getNorm() < minAimVectorMagnitudeMeters) {
      return getRotation();
    }
    return HubTargetingGeometry.getRobotRotationToAimAtHub(getPose(), hub);
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
        () -> getHubAimRotation(shotAirtimeSecondsSupplier.getAsDouble()),
        () -> DriveConstants.hubAlignLinearAccelerationLimitMetersPerSecSquared);
  }

  private Command pathfindToTranslationWithRotationOverride(
      Translation2d target, Supplier<Optional<Rotation2d>> rotationTargetOverrideSupplier) {
    return pathfindToTranslationWithRotationOverride(
        target, pathConstraints, rotationTargetOverrideSupplier);
  }

  private Command pathfindToTranslationWithRotationOverride(
      Translation2d target,
      PathConstraints constraints,
      Supplier<Optional<Rotation2d>> rotationTargetOverrideSupplier) {
    return pathfindToTranslationWithRotationOverride(
        target, constraints, 0.0, rotationTargetOverrideSupplier);
  }

  private Command pathfindToTranslationWithRotationOverride(
      Translation2d target,
      PathConstraints constraints,
      double goalEndVelocity,
      Supplier<Optional<Rotation2d>> rotationTargetOverrideSupplier) {
    return pathfindToTranslation(target, constraints, goalEndVelocity)
        .beforeStarting(
            () -> pathController.setRotationTargetOverride(rotationTargetOverrideSupplier))
        .finallyDo(pathController::clearRotationTargetOverride);
  }

  private Command pathfindToPoseWithRotationOverride(
      Pose2d target,
      PathConstraints constraints,
      Supplier<Optional<Rotation2d>> rotationTargetOverrideSupplier) {
    return pathfindToPoseWithRotationOverride(
        target, constraints, 0.0, rotationTargetOverrideSupplier);
  }

  private Command pathfindToPoseWithRotationOverride(
      Pose2d target,
      PathConstraints constraints,
      double goalEndVelocity,
      Supplier<Optional<Rotation2d>> rotationTargetOverrideSupplier) {
    return pathfindToPose(target, constraints, goalEndVelocity)
        .beforeStarting(
            () -> pathController.setRotationTargetOverride(rotationTargetOverrideSupplier))
        .finallyDo(pathController::clearRotationTargetOverride);
  }

  private Command followNamedPathWithRotationOverride(
      String pathName,
      PathConstraints constraints,
      Supplier<Optional<Rotation2d>> rotationTargetOverrideSupplier) {
    return followNamedPath(pathName, constraints)
        .beforeStarting(
            () -> pathController.setRotationTargetOverride(rotationTargetOverrideSupplier))
        .finallyDo(pathController::clearRotationTargetOverride);
  }

  private Command buildNamedPathCommand(String pathName, boolean pathfindFirst) {
    return buildNamedPathCommand(pathName, pathfindFirst, null);
  }

  private Command buildNamedPathCommand(
      String pathName, boolean pathfindFirst, PathConstraints constraintsOverride) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      if (constraintsOverride != null) {
        path = copyPathWithConstraints(path, constraintsOverride);
      }
      return pathfindFirst
          ? AutoBuilder.pathfindThenFollowPath(path, pathConstraints)
          : AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load PathPlanner path '" + pathName + "': " + e.getMessage(),
          e.getStackTrace());
      return Commands.none();
    }
  }

  static PathPlannerPath copyPathWithConstraints(
      PathPlannerPath path, PathConstraints constraints) {
    PathPlannerPath constrainedPath =
        new PathPlannerPath(
            path.getWaypoints(),
            path.getRotationTargets(),
            path.getPointTowardsZones(),
            path.getConstraintZones(),
            path.getEventMarkers(),
            constraints,
            clampIdealStartingState(path.getIdealStartingState(), constraints),
            clampGoalEndState(path.getGoalEndState(), constraints),
            path.isReversed());
    constrainedPath.name = path.name;
    constrainedPath.preventFlipping = path.preventFlipping;
    return constrainedPath;
  }

  private static IdealStartingState clampIdealStartingState(
      IdealStartingState state, PathConstraints constraints) {
    if (state == null) {
      return null;
    }
    return new IdealStartingState(
        clampPathVelocity(state.velocityMPS(), constraints), state.rotation());
  }

  private static GoalEndState clampGoalEndState(GoalEndState state, PathConstraints constraints) {
    if (state == null) {
      return null;
    }
    return new GoalEndState(clampPathVelocity(state.velocityMPS(), constraints), state.rotation());
  }

  private static double clampPathVelocity(
      double velocityMetersPerSecond, PathConstraints constraints) {
    if (constraints == null || constraints.unlimited()) {
      return velocityMetersPerSecond;
    }
    return Math.copySign(
        Math.min(Math.abs(velocityMetersPerSecond), constraints.maxVelocityMPS()),
        velocityMetersPerSecond);
  }

  private Rotation2d getHubAimRotation(double airtimeSeconds) {
    Pose2d baseHub = getAllianceHubPose();
    Pose2d compensatedHub = getCompensatedHub(baseHub, airtimeSeconds);
    logHubAimTarget(baseHub, compensatedHub, airtimeSeconds);
    return getRotationToHub(compensatedHub);
  }

  private void logHubAimTarget(Pose2d baseHub, Pose2d compensatedHub, double airtimeSeconds) {
    if (!isHubAimVectorLoggingEnabled()) {
      return;
    }

    Pose2d robotPose = getPose();
    Translation2d launchOrigin = HubTargetingGeometry.getLaunchOriginFieldPosition(robotPose);
    Translation2d compensationOffset =
        compensatedHub.getTranslation().minus(baseHub.getTranslation());
    Translation2d vectorToTarget = compensatedHub.getTranslation().minus(launchOrigin);

    Logger.recordOutput(NetworkTablesUtil.logPath("Targeting/Hub/Drive/RobotPose"), robotPose);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/LaunchOriginPose"),
        new Pose2d(launchOrigin, Rotation2d.kZero));
    Logger.recordOutput(NetworkTablesUtil.logPath("Targeting/Hub/Drive/BaseTargetPose"), baseHub);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/CompensatedTargetPose"), compensatedHub);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/TargetVectorEndpoints"),
        new Pose2d[] {
          new Pose2d(launchOrigin, Rotation2d.kZero),
          new Pose2d(compensatedHub.getTranslation(), Rotation2d.kZero)
        });
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/AirtimeSeconds"), airtimeSeconds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/CompensationOffsetX"),
        compensationOffset.getX());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/CompensationOffsetY"),
        compensationOffset.getY());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/CompensationVelocityScale"),
        hubMotionCompVelocityScale);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/CompensationLeadSeconds"),
        hubMotionCompLeadSeconds);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/VectorToTargetX"), vectorToTarget.getX());
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/VectorToTargetY"), vectorToTarget.getY());
    wasHubAimVectorLoggingEnabled = true;
  }

  private void clearHubAimLogs() {
    Logger.recordOutput(NetworkTablesUtil.logPath("Targeting/Hub/Drive/RobotPose"), Pose2d.kZero);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/LaunchOriginPose"), Pose2d.kZero);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/BaseTargetPose"), Pose2d.kZero);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/CompensatedTargetPose"), Pose2d.kZero);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/TargetVectorEndpoints"), new Pose2d[] {});
    Logger.recordOutput(NetworkTablesUtil.logPath("Targeting/Hub/Drive/AirtimeSeconds"), 0.0);
    Logger.recordOutput(NetworkTablesUtil.logPath("Targeting/Hub/Drive/CompensationOffsetX"), 0.0);
    Logger.recordOutput(NetworkTablesUtil.logPath("Targeting/Hub/Drive/CompensationOffsetY"), 0.0);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/CompensationVelocityScale"), 0.0);
    Logger.recordOutput(
        NetworkTablesUtil.logPath("Targeting/Hub/Drive/CompensationLeadSeconds"), 0.0);
    Logger.recordOutput(NetworkTablesUtil.logPath("Targeting/Hub/Drive/VectorToTargetX"), 0.0);
    Logger.recordOutput(NetworkTablesUtil.logPath("Targeting/Hub/Drive/VectorToTargetY"), 0.0);
    wasHubAimVectorLoggingEnabled = false;
  }

  private void configureDrivePidDefaults() {
    moduleDriveKpEntry.setDefaultDouble(moduleDriveKp);
    moduleDriveKiEntry.setDefaultDouble(moduleDriveKi);
    moduleDriveKdEntry.setDefaultDouble(moduleDriveKd);
    moduleTurnKpEntry.setDefaultDouble(moduleTurnKp);
    moduleTurnKiEntry.setDefaultDouble(moduleTurnKi);
    moduleTurnKdEntry.setDefaultDouble(moduleTurnKd);
    pathTranslationKpEntry.setDefaultDouble(defaultPathTranslationKp);
    pathTranslationKiEntry.setDefaultDouble(defaultPathTranslationKi);
    pathTranslationKdEntry.setDefaultDouble(defaultPathTranslationKd);
    pathRotationKpEntry.setDefaultDouble(defaultPathRotationKp);
    pathRotationKiEntry.setDefaultDouble(defaultPathRotationKi);
    pathRotationKdEntry.setDefaultDouble(defaultPathRotationKd);
  }

  private boolean isHubAimVectorLoggingEnabled() {
    return logHubAimVectorEntry.getBoolean(false);
  }

  private void loadDrivePidTuning() {
    moduleDriveKp = sanitizeGain(moduleDriveKpEntry.getDouble(moduleDriveKp), moduleDriveKp);
    moduleDriveKi = sanitizeGain(moduleDriveKiEntry.getDouble(moduleDriveKi), moduleDriveKi);
    moduleDriveKd = sanitizeGain(moduleDriveKdEntry.getDouble(moduleDriveKd), moduleDriveKd);
    moduleTurnKp = sanitizeGain(moduleTurnKpEntry.getDouble(moduleTurnKp), moduleTurnKp);
    moduleTurnKi = sanitizeGain(moduleTurnKiEntry.getDouble(moduleTurnKi), moduleTurnKi);
    moduleTurnKd = sanitizeGain(moduleTurnKdEntry.getDouble(moduleTurnKd), moduleTurnKd);
    pathTranslationKp =
        sanitizeGain(pathTranslationKpEntry.getDouble(pathTranslationKp), pathTranslationKp);
    pathTranslationKi =
        sanitizeGain(pathTranslationKiEntry.getDouble(pathTranslationKi), pathTranslationKi);
    pathTranslationKd =
        sanitizeGain(pathTranslationKdEntry.getDouble(pathTranslationKd), pathTranslationKd);
    pathRotationKp = sanitizeGain(pathRotationKpEntry.getDouble(pathRotationKp), pathRotationKp);
    pathRotationKi = sanitizeGain(pathRotationKiEntry.getDouble(pathRotationKi), pathRotationKi);
    pathRotationKd = sanitizeGain(pathRotationKdEntry.getDouble(pathRotationKd), pathRotationKd);

    moduleDriveKpEntry.setDouble(moduleDriveKp);
    moduleDriveKiEntry.setDouble(moduleDriveKi);
    moduleDriveKdEntry.setDouble(moduleDriveKd);
    moduleTurnKpEntry.setDouble(moduleTurnKp);
    moduleTurnKiEntry.setDouble(moduleTurnKi);
    moduleTurnKdEntry.setDouble(moduleTurnKd);
    pathTranslationKpEntry.setDouble(pathTranslationKp);
    pathTranslationKiEntry.setDouble(pathTranslationKi);
    pathTranslationKdEntry.setDouble(pathTranslationKd);
    pathRotationKpEntry.setDouble(pathRotationKp);
    pathRotationKiEntry.setDouble(pathRotationKi);
    pathRotationKdEntry.setDouble(pathRotationKd);

    for (var module : modules) {
      module.setDriveVelocityGains(moduleDriveKp, moduleDriveKi, moduleDriveKd);
      module.setTurnPositionGains(moduleTurnKp, moduleTurnKi, moduleTurnKd);
    }
    pathController.setTranslationPID(pathTranslationKp, pathTranslationKi, pathTranslationKd);
    pathController.setRotationPID(pathRotationKp, pathRotationKi, pathRotationKd);
  }

  private double sanitizeGain(double value, double fallback) {
    return Double.isFinite(value) ? value : fallback;
  }

  private void configureHubTargetingDefaults() {
    hubMotionCompVelocityScaleEntry.setDefaultDouble(
        ShooterConstants.hubMotionCompensationVelocityScale);
    hubMotionCompLeadSecondsEntry.setDefaultDouble(
        ShooterConstants.hubMotionCompensationLeadSeconds);
  }

  private void loadHubTargetingTuning() {
    hubMotionCompVelocityScale =
        MathUtil.clamp(
            hubMotionCompVelocityScaleEntry.getDouble(hubMotionCompVelocityScale), 0.0, 3.0);
    hubMotionCompLeadSeconds =
        MathUtil.clamp(
            hubMotionCompLeadSecondsEntry.getDouble(hubMotionCompLeadSeconds), -0.25, 0.25);
    hubMotionCompVelocityScaleEntry.setDouble(hubMotionCompVelocityScale);
    hubMotionCompLeadSecondsEntry.setDouble(hubMotionCompLeadSeconds);
  }

  public Command alignToOutpost(DoubleSupplier x, DoubleSupplier y) {
    return DriveCommands.joystickDriveAtAngle(
        this,
        x,
        y,
        () -> {
          Pose2d target = getNearestOutpost();
          Translation2d toTarget = target.getTranslation().minus(getPose().getTranslation());
          if (toTarget.getNorm() < minAimVectorMagnitudeMeters) {
            return getRotation();
          }
          return new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
        });
  }

  public void zeroGyro() {
    gyroIO.zeroYaw();
  }
}
