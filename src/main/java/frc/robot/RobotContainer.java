// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final Pose2d SIM_START_POSE = new Pose2d(3.0, 3.0, Rotation2d.kZero);
  private static final double HUB_TOP_LENGTH_Y_METERS = Units.inchesToMeters(47.0);
  private static final double HUB_RAMP_LENGTH_Y_METERS = Units.inchesToMeters(73.0);
  private static final double HUB_WIDTH_X_METERS = Units.inchesToMeters(47.0);
  private static final double HUMP_PEAK_HEIGHT_METERS = Units.inchesToMeters(7.0);
  private static final double HUMP_X_CLEARANCE_MARGIN_METERS = 0.15;
  private static final double ROBOT_BODY_BASE_HEIGHT_METERS = 0.12;
  private static final double MODULE_HEIGHT_ABOVE_GROUND_METERS = 0.05;
  private static final HumpPoseSample FLAT_GROUND_SAMPLE =
      new HumpPoseSample(new double[] {0.0, 0.0, 0.0, 0.0}, 0.0, 0.0, 0.0);

  // Subsystems
  private final Drive drive;
  private final Shooter shooter = new Shooter();

  // Controller
  public final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Vision vision;
  private SwerveDriveSimulation driveSimulation = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        break;

      case SIM:
        // Sim robot, instantiate MapleSim physics implementations
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
        driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig, SIM_START_POSE);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToCamera1,
                    driveSimulation::getSimulatedDriveTrainPose));
        resetSimulationField();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.y().onTrue(drive.getAutonomousCommand());

    controller.povUp().whileTrue(alignToHub());

    controller.povLeft().onTrue(alignToOutpost());

    controller.povRight().toggleOnTrue(alignToHub());

    controller.povDown().onTrue(drive.getDefaultCommand());

    if (Constants.currentMode == Constants.Mode.SIM) {
      controller
          .leftBumper()
          .onTrue(Commands.runOnce(this::resetSimulationField).ignoringDisable(true));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return drive.getAutonomousCommand();
  }

  public Command alignToHub() {
    return drive.alignToHub(
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> shooter.estimateHubShotAirtimeSeconds(drive.getPose(), drive.getNearestHubPose()));
  }

  public Command alignToOutpost() {
    return drive.alignToOutpost(() -> -controller.getLeftX(), () -> -controller.getLeftY());
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) {
      return;
    }

    drive.setPose(SIM_START_POSE);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) {
      return;
    }

    SimulatedArena.getInstance().simulationPeriodic();
    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    HumpPoseSample humpPoseSample = sampleHumpPose(robotPose);
    Logger.recordOutput("FieldSimulation/RobotPose", robotPose);
    Logger.recordOutput(
        "FieldSimulation/RobotPose3d", getSimulatedRobotPose3d(robotPose, humpPoseSample));
    Logger.recordOutput(
        "FieldSimulation/RobotParts/SwerveModules",
        getSimulatedModulePoses(robotPose, humpPoseSample));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Fuel",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Note",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Coral",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/GamePieces/Algae",
        SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  private Pose3d getSimulatedRobotPose3d(Pose2d robotPose, HumpPoseSample humpPoseSample) {
    return new Pose3d(
        new Translation3d(
            robotPose.getX(),
            robotPose.getY(),
            ROBOT_BODY_BASE_HEIGHT_METERS + humpPoseSample.heightMeters()),
        new Rotation3d(
            humpPoseSample.rollRadians(),
            humpPoseSample.pitchRadians(),
            robotPose.getRotation().getRadians()));
  }

  private Pose3d[] getSimulatedModulePoses(Pose2d robotPose, HumpPoseSample humpPoseSample) {
    Pose3d[] modulePoses = new Pose3d[DriveConstants.moduleTranslations.length];
    for (int i = 0; i < modulePoses.length; i++) {
      Translation2d moduleOffset =
          DriveConstants.moduleTranslations[i].rotateBy(robotPose.getRotation());
      Translation2d modulePosition = robotPose.getTranslation().plus(moduleOffset);
      Rotation2d moduleRotation =
          driveSimulation.getModules()[i].getSteerAbsoluteFacing().plus(robotPose.getRotation());

      modulePoses[i] =
          new Pose3d(
              new Translation3d(
                  modulePosition.getX(),
                  modulePosition.getY(),
                  MODULE_HEIGHT_ABOVE_GROUND_METERS + humpPoseSample.moduleHeightsMeters()[i]),
              new Rotation3d(
                  humpPoseSample.rollRadians(),
                  humpPoseSample.pitchRadians(),
                  moduleRotation.getRadians()));
    }
    return modulePoses;
  }

  private HumpPoseSample sampleHumpPose(Pose2d robotPose) {
    double[] moduleHeights = new double[DriveConstants.moduleTranslations.length];
    for (int i = 0; i < moduleHeights.length; i++) {
      Translation2d moduleFieldPosition =
          robotPose
              .getTranslation()
              .plus(DriveConstants.moduleTranslations[i].rotateBy(robotPose.getRotation()));
      moduleHeights[i] = sampleGroundHeightAt(moduleFieldPosition);
    }

    double frontAverage = (moduleHeights[0] + moduleHeights[1]) * 0.5;
    double backAverage = (moduleHeights[2] + moduleHeights[3]) * 0.5;
    double leftAverage = (moduleHeights[0] + moduleHeights[2]) * 0.5;
    double rightAverage = (moduleHeights[1] + moduleHeights[3]) * 0.5;
    double averageHeight =
        (moduleHeights[0] + moduleHeights[1] + moduleHeights[2] + moduleHeights[3]) * 0.25;

    double rollRadians = Math.atan2(leftAverage - rightAverage, DriveConstants.trackWidth);
    double pitchRadians = Math.atan2(frontAverage - backAverage, DriveConstants.wheelBase);
    return new HumpPoseSample(moduleHeights, averageHeight, rollRadians, pitchRadians);
  }

  private double sampleGroundHeightAt(Translation2d position) {
    double blueHeight = sampleSingleHumpHeight(Constants.blueHub.getX(), position);
    double redHeight = sampleSingleHumpHeight(Constants.redHub.getX(), position);
    return Math.max(blueHeight, redHeight);
  }

  private double sampleSingleHumpHeight(double hubCenterXMeters, Translation2d position) {
    double maxHumpXDistance = (HUB_WIDTH_X_METERS / 2.0) + HUMP_X_CLEARANCE_MARGIN_METERS;
    if (Math.abs(position.getX() - hubCenterXMeters) > maxHumpXDistance) {
      return 0.0;
    }

    double yOffsetFromHub = position.getY() - Constants.blueHub.getY();
    double absYOffset = Math.abs(yOffsetFromHub);
    double halfTopLength = HUB_TOP_LENGTH_Y_METERS / 2.0;
    if (absYOffset > (halfTopLength + HUB_RAMP_LENGTH_Y_METERS)) {
      return 0.0;
    }

    if (absYOffset <= halfTopLength) {
      return HUMP_PEAK_HEIGHT_METERS;
    }

    double rampTravelMeters = absYOffset - halfTopLength;
    double rampPercent =
        1.0 - MathUtil.clamp(rampTravelMeters / HUB_RAMP_LENGTH_Y_METERS, 0.0, 1.0);
    return HUMP_PEAK_HEIGHT_METERS * rampPercent;
  }

  private record HumpPoseSample(
      double[] moduleHeightsMeters, double heightMeters, double rollRadians, double pitchRadians) {}
}
